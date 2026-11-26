//! The hot-path rules (`CONTRIBUTING.md`, "Hot paths") for wind: the same
//! forces on any thread count, bit for bit, and the native SIMD level
//! within a tolerance of the scalar reference.
use breakit_core::loads::wind::{Execution, Gusts, Wind};
use breakit_core::{
    DynamicSim, LoadCase, LoadKind, LoadSet, Material, Section, SimConfig, SimdLevel, Structure,
    Support, Vec3, Window,
};

/// A braced lattice of tube columns and rectangular beams, `n` x `n` bays
/// and `h` storeys of 1 m.
fn lattice(n: usize, h: usize) -> Structure {
    let mut s = Structure::new();
    let id = |i: usize, j: usize, k: usize| (k * (n + 1) + j) * (n + 1) + i;
    for k in 0..=h {
        for j in 0..=n {
            for i in 0..=n {
                s.add_node(Vec3::new(i as f64, k as f64, j as f64));
            }
        }
    }
    let m = Material::steel();
    let (tube, rect) = (Section::tube(0.05, 0.005), Section::rect(0.04, 0.08));
    for k in 0..=h {
        for j in 0..=n {
            for i in 0..=n {
                if i < n {
                    s.add_member(id(i, j, k), id(i + 1, j, k), m.clone(), rect.clone());
                }
                if j < n {
                    s.add_member(id(i, j, k), id(i, j + 1, k), m.clone(), rect.clone());
                }
                if k < h {
                    s.add_member(id(i, j, k), id(i, j, k + 1), m.clone(), tube.clone());
                    if i < n {
                        s.add_member(id(i, j, k), id(i + 1, j, k + 1), m.clone(), tube.clone());
                    }
                }
                if k == 0 {
                    s.add_support(Support::fixed(id(i, j, k)));
                }
            }
        }
    }
    s
}

/// Nodal forces of 30 frames of gusty wind with shedding and galloping, on
/// `threads` threads with `exec`.
fn run(s: &Structure, threads: usize, exec: Execution) -> Vec<Vec<Vec3>> {
    let mut wind = Wind::new(18.0, Vec3::new(1.0, 0.0, 0.3));
    wind.gusts = Some(Gusts::default());
    wind.set_execution(exec);
    let mut loads = LoadSet::new(vec![LoadCase::new(
        "wind",
        Window::new(0.0, None, 0.5),
        LoadKind::Wind(wind),
    )]);
    let pool = rayon::ThreadPoolBuilder::new()
        .num_threads(threads)
        .build()
        .expect("a thread pool can be built");
    pool.install(|| {
        let mut sim = DynamicSim::new(s, SimConfig::default());
        loads.prepare(s);
        (0..30)
            .map(|_| {
                loads.step(s, &mut sim, 1.0 / 60.0);
                loads.forces().to_vec()
            })
            .collect()
    })
}

#[test]
fn wind_forces_are_bit_identical_on_any_thread_count() {
    let s = lattice(4, 6);
    let exec = |parallel_min_members| Execution {
        simd: SimdLevel::Native,
        parallel_min_members,
    };
    let reference = run(&s, 1, exec(usize::MAX));
    assert!(reference.last().unwrap().iter().any(|f| f.norm() > 1.0));
    for threads in [1, 2, 3, 4] {
        assert!(
            run(&s, threads, exec(0)) == reference,
            "{threads} threads differ from the single-thread path"
        );
    }
}

#[test]
fn native_simd_wind_matches_the_scalar_reference() {
    let s = lattice(3, 4);
    let exec = |simd| Execution {
        simd,
        parallel_min_members: usize::MAX,
    };
    let scalar = run(&s, 1, exec(SimdLevel::Scalar));
    let native = run(&s, 1, exec(SimdLevel::Native));
    let peak = scalar
        .iter()
        .flatten()
        .map(|f| f.norm())
        .fold(0.0, f64::max);
    for (a, b) in scalar.iter().flatten().zip(native.iter().flatten()) {
        // Last-bit differences feed back through the motion over 30
        // frames; they stay far below anything visible.
        assert!((a - b).norm() <= 1e-6 * peak, "{a} {b}");
    }
}
