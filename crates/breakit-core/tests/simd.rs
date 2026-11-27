//! The SIMD and multi-core rules of `CONTRIBUTING.md` ("Hot paths"), as
//! tests: the native kernels agree with the scalar reference within a
//! tolerance, threads do not change the answer, and the machine running
//! the tests really dispatches to a vector instruction set.

use breakit_core::linsolve::{tri, Backend, LinSolver, Policy, BLOCK_NNZ};
use breakit_core::{DynamicSim, Material, Section, SimConfig, SimdLevel, Structure, Support, Vec3};

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

/// A tower of `nx` x `nz` columns and `floors` storeys of 2 m bays, clamped
/// at the base: (structure, top nodes).
fn tower(nx: usize, nz: usize, floors: usize) -> (Structure, Vec<usize>) {
    let (mat, sec) = (Material::steel(), Section::rect(0.1, 0.1));
    let mut s = Structure::new();
    let id = |x: usize, y: usize, z: usize| (y * nz + z) * nx + x;
    for y in 0..=floors {
        for z in 0..nz {
            for x in 0..nx {
                s.add_node(Vec3::new(x as f64, y as f64, z as f64) * 2.0);
            }
        }
    }
    for y in 0..=floors {
        for z in 0..nz {
            for x in 0..nx {
                let a = id(x, y, z);
                if x + 1 < nx && y > 0 {
                    s.add_member(a, id(x + 1, y, z), mat.clone(), sec.clone());
                }
                if z + 1 < nz && y > 0 {
                    s.add_member(a, id(x, y, z + 1), mat.clone(), sec.clone());
                }
                if y < floors {
                    s.add_member(a, id(x, y + 1, z), mat.clone(), sec.clone());
                }
                if y == 0 {
                    s.add_support(Support::fixed(a));
                }
            }
        }
    }
    let top = (0..nx * nz).map(|k| floors * nx * nz + k).collect();
    (s, top)
}

/// Displacement of every node after the tower settles under its weight and
/// a sideways push, elastic.
fn settled_tower(backend: Backend, simd: SimdLevel) -> Vec<Vec3> {
    let (s, top) = tower(3, 3, 5);
    let mut sim = DynamicSim::new(
        &s,
        SimConfig {
            damping: 8.0,
            plasticity: false,
            linear_backend: backend,
            linear_tolerance: 1e-8,
            parallel_min_members: usize::MAX,
            simd,
            ..Default::default()
        },
    );
    for &n in &top {
        sim.set_force(n, Vec3::new(4000.0, 0.0, 1500.0));
    }
    assert!(
        sim.settle(20.0, 1e-7),
        "{backend:?} on {simd:?} did not settle"
    );
    (0..s.nodes.len()).map(|n| sim.displacement(n)).collect()
}

/// `SimConfig::simd` switches the whole simulation, assembly included: the
/// native run ends where the scalar reference does.
#[test]
fn simulation_on_native_matches_the_scalar_reference() {
    for backend in [Backend::Cholesky, Backend::BlockJacobi] {
        let scalar = settled_tower(backend, SimdLevel::Scalar);
        let native = settled_tower(backend, SimdLevel::Native);
        let largest = scalar.iter().map(|d| d.norm()).fold(0.0, f64::max);
        assert!(largest > 1e-3, "the push should move the tower");
        let diff = (scalar.iter().zip(&native))
            .map(|(a, b)| (a - b).norm())
            .fold(0.0, f64::max);
        assert!(
            diff < 1e-6 * largest,
            "{backend:?}: native differs from scalar by {diff:e} of {largest:e} m"
        );
    }
}

/// Everything observable about a run that is pushed until it yields and
/// breaks: node states as bits, and the breaks.
fn collapse_run(parallel_min_members: usize, simd: SimdLevel) -> (Vec<[u64; 7]>, Vec<usize>) {
    // 2,300 members: enough for both assembly and the step check to ask
    // for other threads.
    let (s, top) = tower(8, 8, 13);
    let mut sim = DynamicSim::new(
        &s,
        SimConfig {
            parallel_min_members,
            simd,
            ..Default::default()
        },
    );
    for frame in 0..40 {
        let push = 4.0e4 * 1.15f64.powi(frame);
        for &n in &top {
            sim.set_force(n, Vec3::new(push, 0.0, 0.3 * push));
        }
        sim.step(1.0 / 60.0);
    }
    let state = (0..s.nodes.len())
        .map(|n| {
            let (x, q) = (sim.position(n), sim.orientation(n));
            [x.x, x.y, x.z, q.i, q.j, q.k, q.w].map(f64::to_bits)
        })
        .collect();
    let breaks = sim.break_events().iter().map(|e| e.member).collect();
    (state, breaks)
}

/// Threaded assembly and step check give the single-thread state bit for
/// bit, on any number of threads, through yielding and breaks.
#[test]
fn threaded_simulation_is_bit_identical() {
    for simd in [SimdLevel::Scalar, SimdLevel::Native] {
        let single = collapse_run(usize::MAX, simd);
        assert!(
            !single.1.is_empty(),
            "the push should break members, or the test covers less than it says"
        );
        for threads in [1, 2, 4] {
            let pool = rayon::ThreadPoolBuilder::new()
                .num_threads(threads)
                .build()
                .expect("a thread pool");
            let threaded = pool.install(|| collapse_run(0, simd));
            assert!(
                single == threaded,
                "{simd:?} on {threads} threads is not bit-identical to one thread"
            );
        }
    }
}
