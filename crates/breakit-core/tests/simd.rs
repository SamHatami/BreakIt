//! The SIMD and multi-core rules of `CONTRIBUTING.md` ("Hot paths"), as
//! tests: the native kernels agree with the scalar reference within a
//! tolerance, threads do not change the answer, and the machine running
//! the tests really dispatches to a vector instruction set.

use breakit_core::linsolve::{tri, Backend, LinSolver, Policy, BLOCK_NNZ};
use breakit_core::SimdLevel;

struct Rng(u64);

impl Rng {
    fn unit(&mut self) -> f64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        (self.0 >> 11) as f64 / (1u64 << 53) as f64
    }
}

/// A 3D grid of nodes joined along the axes, with the bottom layer's
/// translations fixed: (unknowns, beam slots).
fn grid(nx: usize, ny: usize, nz: usize) -> (usize, Vec<[Option<usize>; 4]>) {
    let id = |x: usize, y: usize, z: usize| (y * nz + z) * nx + x;
    // Slots of a node: translation then rotation; bottom translations fixed.
    let mut next = 0;
    let mut slots = Vec::new();
    for y in 0..ny {
        for _ in 0..nx * nz {
            let mut take = |free: bool| {
                free.then(|| {
                    next += 3;
                    next - 3
                })
            };
            slots.push([take(y > 0), take(true)]);
        }
    }
    let mut beams = Vec::new();
    for y in 0..ny {
        for z in 0..nz {
            for x in 0..nx {
                let a = id(x, y, z);
                let mut link = |b: usize| {
                    beams.push([slots[a][0], slots[a][1], slots[b][0], slots[b][1]]);
                };
                if x + 1 < nx {
                    link(id(x + 1, y, z));
                }
                if y + 1 < ny {
                    link(id(x, y + 1, z));
                }
                if z + 1 < nz {
                    link(id(x, y, z + 1));
                }
            }
        }
    }
    (next, beams)
}

/// Fill the solver with a random symmetric positive definite matrix: a
/// positive diagonal plus one positive semidefinite 12x12 block per beam.
fn assemble(lin: &mut LinSolver, beams: usize, seed: u64) {
    let mut rng = Rng(seed);
    lin.clear();
    for d in 0..lin.ndof() {
        lin.add_diagonal(d, 0.5 + rng.unit());
    }
    for i in 0..beams {
        // Sum of nine rank-one terms g gᵀ.
        let mut block = [0.0; BLOCK_NNZ];
        for _ in 0..9 {
            let g: [f64; 12] = std::array::from_fn(|_| rng.unit() - 0.5);
            for r in 0..12 {
                for c in 0..=r {
                    block[tri(r, c)] += 40.0 * g[r] * g[c];
                }
            }
        }
        lin.add_block(i, &block);
    }
}

fn policy(tolerance: f64) -> Policy {
    Policy {
        tolerance,
        refactor_cost: 0,
        max_iters: 0,
        max_age: 0,
    }
}

/// Solve the same system on `level`, twice (the second solve starts from
/// the first's state, as substeps do).
fn solve(backend: Backend, level: SimdLevel, par_min_nodes: usize, tolerance: f64) -> Vec<f64> {
    let (n, beams) = grid(6, 9, 5);
    let mut lin = LinSolver::new(n, &beams, backend).expect("the grid has unknowns");
    lin.set_simd(level);
    lin.set_parallel_min_nodes(par_min_nodes);
    let mut rng = Rng(0x9E3779B97F4A7C15);
    let b: Vec<f64> = (0..n).map(|_| rng.unit() - 0.5).collect();
    let mut x = vec![0.0; n];
    for seed in [0xD1B54A32D192ED03, 0xD1B54A32D192ED05] {
        assemble(&mut lin, beams.len(), seed);
        assert!(
            lin.solve(&b, &mut x, &policy(tolerance)),
            "{backend:?} on {level:?} failed"
        );
    }
    x
}

fn relative_difference(a: &[f64], b: &[f64]) -> f64 {
    let norm = |v: &[f64]| v.iter().map(|x| x * x).sum::<f64>().sqrt();
    let diff: Vec<f64> = a.iter().zip(b).map(|(a, b)| a - b).collect();
    norm(&diff) / norm(a)
}

#[test]
fn native_kernels_match_the_scalar_reference() {
    // The block Cholesky path iterates in double precision, block Jacobi in
    // single: the tolerances are those of the arithmetic, not of the method.
    for (backend, tolerance, allowed) in [
        (Backend::Cholesky, 1e-10, 1e-8),
        (Backend::BlockJacobi, 1e-6, 2e-5),
    ] {
        let scalar = solve(backend, SimdLevel::Scalar, usize::MAX, tolerance);
        let native = solve(backend, SimdLevel::Native, usize::MAX, tolerance);
        let diff = relative_difference(&scalar, &native);
        assert!(
            diff < allowed,
            "{backend:?}: native differs from scalar by {diff:e}"
        );
    }
}

#[test]
fn both_backends_solve_the_same_system() {
    let direct = solve(Backend::Cholesky, SimdLevel::Scalar, usize::MAX, 1e-10);
    for level in [SimdLevel::Scalar, SimdLevel::Native] {
        let iterative = solve(Backend::BlockJacobi, level, usize::MAX, 1e-6);
        let diff = relative_difference(&direct, &iterative);
        assert!(diff < 1e-4, "block Jacobi on {level:?} is off by {diff:e}");
    }
}

#[test]
fn threads_do_not_change_the_answer() {
    for level in [SimdLevel::Scalar, SimdLevel::Native] {
        let single = solve(Backend::BlockJacobi, level, usize::MAX, 1e-6);
        let threaded = solve(Backend::BlockJacobi, level, 0, 1e-6);
        assert_eq!(
            single, threaded,
            "threaded block Jacobi on {level:?} is not bit-identical"
        );
    }
}

/// On the CPUs the game targets, `Native` must not silently fall back to
/// scalar code.
#[test]
fn native_level_dispatches_to_simd() {
    assert_eq!(SimdLevel::Scalar.name(), "scalar");
    #[cfg(target_arch = "x86_64")]
    if std::arch::is_x86_feature_detected!("avx2") && std::arch::is_x86_feature_detected!("fma") {
        assert_eq!(SimdLevel::Native.name(), "avx2+fma");
    }
    #[cfg(target_arch = "aarch64")]
    assert_eq!(SimdLevel::Native.name(), "neon");
}
