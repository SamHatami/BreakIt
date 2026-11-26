//! Pieces that no live member ties together are solved as separate linear
//! systems (`SimConfig::island_min_nodes`): the result is that of one
//! system, on any number of threads, from the start and after a break.

use breakit_core::{DynamicSim, Material, Section, SimConfig, Structure, Support, Vec3};

/// Add a tower of 4 x 4 columns and `floors` storeys of 2 m bays at `x0`,
/// clamped at the base. Returns its top nodes.
fn add_tower(s: &mut Structure, x0: f64, floors: usize) -> Vec<usize> {
    let (mat, sec) = (Material::steel().elastic_only(), Section::rect(0.1, 0.1));
    let first = s.nodes.len();
    let id = |x: usize, y: usize, z: usize| first + (y * 4 + z) * 4 + x;
    for y in 0..=floors {
        for z in 0..4 {
            for x in 0..4 {
                s.add_node(Vec3::new(
                    x0 + 2.0 * x as f64,
                    2.0 * y as f64,
                    2.0 * z as f64,
                ));
            }
        }
    }
    for y in 0..=floors {
        for z in 0..4 {
            for x in 0..4 {
                let a = id(x, y, z);
                if x + 1 < 4 && y > 0 {
                    s.add_member(a, id(x + 1, y, z), mat.clone(), sec.clone());
                }
                if z + 1 < 4 && y > 0 {
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
    (0..16).map(|k| id(k % 4, floors, k / 4)).collect()
}

fn config(island_min_nodes: usize) -> SimConfig {
    SimConfig {
        damping: 2.0,
        linear_tolerance: 1e-10,
        island_min_nodes,
        ..Default::default()
    }
}

fn largest_difference(a: &DynamicSim, b: &DynamicSim) -> (f64, f64) {
    let nodes = 0..a.node_count();
    let largest = (nodes.clone())
        .map(|n| a.displacement(n).norm())
        .fold(0.0, f64::max);
    let diff = nodes
        .map(|n| (a.displacement(n) - b.displacement(n)).norm())
        .fold(0.0, f64::max);
    (diff, largest)
}

/// Two towers that share nothing: two systems, and the motion of one.
#[test]
fn separate_structures_are_solved_as_islands() {
    let mut s = Structure::new();
    let tops = [add_tower(&mut s, 0.0, 6), add_tower(&mut s, 50.0, 9)];
    let run = |island_min_nodes| {
        let mut sim = DynamicSim::new(&s, config(island_min_nodes));
        for (k, top) in tops.iter().enumerate() {
            for &n in top {
                sim.set_force(n, Vec3::new(3000.0, 0.0, 1000.0) * (1.0 + k as f64));
            }
        }
        for _ in 0..40 {
            sim.step(1.0 / 60.0);
        }
        sim
    };
    let (one, islands) = (run(usize::MAX), run(64));
    let (solves, island_solves) = (
        one.solver_stats().linear.solves,
        islands.solver_stats().linear.solves,
    );
    assert_eq!(island_solves, 2 * solves, "each tower has its own system");
    let (diff, largest) = largest_difference(&one, &islands);
    assert!(largest > 1e-3, "the push should move the towers");
    assert!(
        diff < 1e-7 * largest,
        "islands differ from one system by {diff:e} of {largest:e} m"
    );
}

/// Two towers held together by one timber tie, pulled apart until it
/// snaps: (structure, top nodes of each tower).
fn tied_towers() -> (Structure, [Vec<usize>; 2]) {
    let mut s = Structure::new();
    let tops = [add_tower(&mut s, 0.0, 7), add_tower(&mut s, 10.0, 7)];
    s.add_member(
        tops[0][3],
        tops[1][0],
        Material::timber(),
        Section::rect(0.02, 0.02),
    );
    (s, tops)
}

/// Pull the tied towers apart for `frames` frames.
fn pull_apart(
    s: &Structure,
    tops: &[Vec<usize>; 2],
    config: SimConfig,
    frames: usize,
) -> DynamicSim {
    let mut sim = DynamicSim::new(s, config);
    for frame in 0..frames {
        let pull = 2.0e4 * (frame as f64 / 20.0).min(1.0);
        for (k, top) in tops.iter().enumerate() {
            let away = if k == 0 { -1.0 } else { 1.0 };
            for &n in top {
                sim.set_force(n, Vec3::new(away * pull, 0.0, 0.2 * pull));
            }
        }
        sim.step(1.0 / 60.0);
    }
    sim
}

/// When the tie breaks, each tower becomes an island; they go on moving as
/// they do in one system.
#[test]
fn a_structure_that_comes_apart_becomes_islands() {
    let (s, tops) = tied_towers();
    let one = pull_apart(&s, &tops, config(usize::MAX), 60);
    let islands = pull_apart(&s, &tops, config(64), 60);
    assert_eq!(one.break_events().len(), 1, "only the tie should break");
    assert_eq!(one.break_events(), islands.break_events());
    let (solves, island_solves) = (
        one.solver_stats().linear.solves,
        islands.solver_stats().linear.solves,
    );
    assert!(
        island_solves > solves + 40,
        "after the break each tower should have its own system: {island_solves} solves against {solves}"
    );
    let (diff, largest) = largest_difference(&one, &islands);
    assert!(largest > 1e-3, "the pull should move the towers");
    assert!(
        diff < 1e-6 * largest,
        "islands differ from one system by {diff:e} of {largest:e} m"
    );
}

/// Islands solved side by side give the single-thread state bit for bit.
#[test]
fn islands_on_threads_are_bit_identical() {
    let (s, tops) = tied_towers();
    let state = |parallel_min_members| {
        let sim = pull_apart(
            &s,
            &tops,
            SimConfig {
                parallel_min_members,
                ..config(64)
            },
            60,
        );
        assert_eq!(sim.break_events().len(), 1);
        (0..s.nodes.len())
            .map(|n| {
                let (x, q) = (sim.position(n), sim.orientation(n));
                [x.x, x.y, x.z, q.i, q.j, q.k, q.w].map(f64::to_bits)
            })
            .collect::<Vec<_>>()
    };
    let single = state(usize::MAX);
    for threads in [1, 2, 4] {
        let pool = rayon::ThreadPoolBuilder::new()
            .num_threads(threads)
            .build()
            .expect("a thread pool");
        let threaded = pool.install(|| state(0));
        assert!(
            single == threaded,
            "islands on {threads} threads are not bit-identical to one thread"
        );
    }
}
