//! Solver benchmark on game-sized structures.
//!
//! cargo run --release --example bench -- [--sizes s,m,l,xl] [--only name] [--static]
//!
//! Two structures (a braced tower frame and a 3D truss bridge), each at several
//! sizes, each run through two scenarios:
//!
//! * `elastic`: self-weight plus a sideways push, no plasticity. Reports the
//!   cost per 60 Hz frame and the settled displacement of a probe node, with
//!   the static FEM answer next to it when `--static` is given.
//! * `collapse`: the sideways push grows 3% per frame until the first member
//!   breaks and is then held, so the structure yields, breaks up and falls.
//!   Reports mean and worst frame cost and the number of broken members.
use breakit_core::*;
use std::time::Instant;

struct Case {
    name: String,
    s: Structure,
    /// Node whose displacement is reported.
    probe: usize,
    /// Nodes that receive the sideways push, and its direction.
    pushed: Vec<usize>,
    push_dir: Vec3,
}

fn link(s: &mut Structure, a: usize, b: usize, sub: usize, mat: &Material, sec: &Section) {
    let (pa, pb) = (s.nodes[a], s.nodes[b]);
    let mut prev = a;
    for i in 1..sub {
        let n = s.add_node(pa + (pb - pa) * (i as f64 / sub as f64));
        s.add_member(prev, n, mat.clone(), sec.clone());
        prev = n;
    }
    s.add_member(prev, b, mat.clone(), sec.clone());
}

/// Multi-storey frame: columns on a grid, beams both ways on every floor,
/// diagonal bracing on the outer faces. Clamped at the base.
fn tower(nx: usize, nz: usize, floors: usize, sub: usize) -> Case {
    let mat = Material::steel();
    let col = Section::tube(0.10, 0.008);
    let beam = Section::rect(0.10, 0.20);
    let brace = Section::tube(0.05, 0.005);
    let (bay, h) = (4.0, 3.0);
    let mut s = Structure::new();
    let id = |i: usize, j: usize, k: usize| (k * (nz + 1) + j) * (nx + 1) + i;
    for k in 0..=floors {
        for j in 0..=nz {
            for i in 0..=nx {
                s.add_node(Vec3::new(i as f64 * bay, k as f64 * h, j as f64 * bay));
            }
        }
    }
    for k in 0..floors {
        for j in 0..=nz {
            for i in 0..=nx {
                link(&mut s, id(i, j, k), id(i, j, k + 1), sub, &mat, &col);
                if i < nx {
                    link(
                        &mut s,
                        id(i, j, k + 1),
                        id(i + 1, j, k + 1),
                        sub,
                        &mat,
                        &beam,
                    );
                    if (j == 0 || j == nz) && (i + k) % 2 == 0 {
                        link(&mut s, id(i, j, k), id(i + 1, j, k + 1), sub, &mat, &brace);
                    }
                }
                if j < nz {
                    link(
                        &mut s,
                        id(i, j, k + 1),
                        id(i, j + 1, k + 1),
                        sub,
                        &mat,
                        &beam,
                    );
                    if (i == 0 || i == nx) && (j + k) % 2 == 0 {
                        link(&mut s, id(i, j, k), id(i, j + 1, k + 1), sub, &mat, &brace);
                    }
                }
            }
        }
    }
    for j in 0..=nz {
        for i in 0..=nx {
            s.add_support(Support::fixed(id(i, j, 0)));
        }
    }
    let pushed = (0..=nz).map(|j| id(0, j, floors)).collect();
    Case {
        name: format!("tower {}x{}x{}", nx, nz, floors),
        probe: id(nx, nz, floors),
        s,
        pushed,
        push_dir: Vec3::new(1.0, 0.0, 0.3).normalize(),
    }
}

/// Through-truss bridge along X: two Warren trusses with verticals, floor
/// beams, deck stringer and top/bottom lateral bracing. Pinned at four corners.
fn bridge(panels: usize, sub: usize) -> Case {
    let mat = Material::steel();
    let chord = Section::tube(0.12, 0.010);
    let web = Section::tube(0.07, 0.006);
    let floor = Section::rect(0.10, 0.25);
    let lat = Section::tube(0.04, 0.004);
    let (p, w, h) = (4.0, 6.0, 5.0);
    let mut s = Structure::new();
    // node(i, side, top)
    let id = |i: usize, side: usize, top: usize| (i * 2 + side) * 2 + top;
    for i in 0..=panels {
        for side in 0..2 {
            for top in 0..2 {
                s.add_node(Vec3::new(i as f64 * p, top as f64 * h, side as f64 * w));
            }
        }
    }
    for i in 0..=panels {
        for side in 0..2 {
            link(&mut s, id(i, side, 0), id(i, side, 1), sub, &mat, &web);
            if i < panels {
                link(
                    &mut s,
                    id(i, side, 0),
                    id(i + 1, side, 0),
                    sub,
                    &mat,
                    &chord,
                );
                link(
                    &mut s,
                    id(i, side, 1),
                    id(i + 1, side, 1),
                    sub,
                    &mat,
                    &chord,
                );
                let (lo, hi) = if i % 2 == 0 { (i, i + 1) } else { (i + 1, i) };
                link(&mut s, id(lo, side, 0), id(hi, side, 1), sub, &mat, &web);
            }
        }
        link(&mut s, id(i, 0, 0), id(i, 1, 0), sub, &mat, &floor);
        link(&mut s, id(i, 0, 1), id(i, 1, 1), sub, &mat, &lat);
        if i < panels {
            link(&mut s, id(i, 0, 0), id(i + 1, 1, 0), sub, &mat, &lat);
            link(
                &mut s,
                id(i, i % 2, 1),
                id(i + 1, 1 - i % 2, 1),
                sub,
                &mat,
                &lat,
            );
        }
    }
    for i in [0, panels] {
        for side in 0..2 {
            s.add_support(Support::pinned(id(i, side, 0)));
        }
    }
    let mid = panels / 2;
    Case {
        name: format!("bridge {} panels", panels),
        probe: id(mid, 0, 0),
        s,
        pushed: vec![id(mid, 0, 0), id(mid, 1, 0)],
        push_dir: Vec3::new(0.0, -1.0, 0.1).normalize(),
    }
}

fn cases(size: &str) -> Vec<Case> {
    match size {
        "s" => vec![tower(2, 2, 4, 1), bridge(8, 1)],
        "m" => vec![tower(3, 3, 8, 1), bridge(20, 2)],
        "l" => vec![tower(5, 5, 12, 1), bridge(60, 3)],
        "xl" => vec![tower(8, 8, 16, 1), bridge(120, 4)],
        _ => panic!("unknown size {size}"),
    }
}

/// Solver configurations under test: (name, config).
fn configs() -> Vec<(&'static str, SimConfig)> {
    let sub = |substeps: usize| SimConfig {
        substeps,
        ..Default::default()
    };
    // The solver before the step check, implicit plasticity and tension
    // stiffness existed (16 substeps was its default).
    let old = |substeps: usize| SimConfig {
        step_check: false,
        implicit_plasticity: false,
        tension_stiffness: false,
        ..sub(substeps)
    };
    vec![
        ("16 old", old(16)),
        ("16", sub(16)),
        ("8", sub(8)),
        ("4", sub(4)),
        ("2", sub(2)),
        ("1", sub(1)),
    ]
}

struct Timing {
    mean: f64,
    p99: f64,
    max: f64,
}

fn timing(mut t: Vec<f64>) -> Timing {
    t.sort_by(|a, b| a.partial_cmp(b).unwrap());
    Timing {
        mean: t.iter().sum::<f64>() / t.len() as f64,
        p99: t[(t.len() * 99 / 100).min(t.len() - 1)],
        max: *t.last().unwrap(),
    }
}

fn weight(s: &Structure) -> f64 {
    s.self_weight_loads(9.81).iter().map(|l| -l.force.y).sum()
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let arg = |k: &str| {
        args.iter()
            .position(|a| a == k)
            .and_then(|i| args.get(i + 1).cloned())
    };
    let sizes = arg("--sizes").unwrap_or("s,m,l".into());
    let only = arg("--only");
    let with_static = args.iter().any(|a| a == "--static");
    let frames_elastic = 180;
    let frames_collapse = 300;
    let dt = 1.0 / 60.0;

    println!(
        "| structure | members | dofs | solver | elastic ms/frame (mean / p99) | probe disp mm | static mm | collapse ms/frame (mean / p99 / max) | breaks | first break s | factor / reject / backtrack / extra iter per frame |"
    );
    println!("|---|---|---|---|---|---|---|---|---|---|---|");
    for size in sizes.split(',') {
        for case in cases(size) {
            let s = &case.s;
            let push = case.push_dir * (0.2 * weight(s) / case.pushed.len() as f64);
            let stat = if with_static && s.nodes.len() * 6 <= 6000 {
                let mut loads = s.self_weight_loads(9.81);
                for &n in &case.pushed {
                    loads[n].force += push;
                }
                solve_static(s, &loads)
                    .ok()
                    .map(|r| r.displacements[case.probe].norm() * 1e3)
            } else {
                None
            };
            for (name, cfg) in configs() {
                if only.as_deref().is_some_and(|o| o != name) {
                    continue;
                }
                // Elastic: time 180 frames, then let it finish settling.
                let mut sim = DynamicSim::new(
                    s,
                    SimConfig {
                        damping: 8.0,
                        plasticity: false,
                        ..cfg.clone()
                    },
                );
                for &n in &case.pushed {
                    sim.set_force(n, push);
                }
                let mut t = Vec::new();
                for _ in 0..frames_elastic {
                    let t0 = Instant::now();
                    sim.step(dt);
                    t.push(t0.elapsed().as_secs_f64() * 1e3);
                }
                let el = timing(t);
                sim.settle(20.0, 1e-6);
                let disp = sim.displacement(case.probe).norm() * 1e3;

                // Collapse: ramp the push until something breaks, then hold.
                let mut sim = DynamicSim::new(s, cfg.clone());
                let mut scale = 1.0;
                let mut t = Vec::new();
                for _ in 0..frames_collapse {
                    if sim.break_events().is_empty() {
                        scale *= 1.03;
                        for &n in &case.pushed {
                            sim.set_force(n, push * scale);
                        }
                    }
                    let t0 = Instant::now();
                    sim.step(dt);
                    t.push(t0.elapsed().as_secs_f64() * 1e3);
                }
                let co = timing(t);
                let st = sim.solver_stats();
                println!(
                    "| {} | {} | {} | {} | {:.2} / {:.2} | {:.4} | {} | {:.2} / {:.2} / {:.2} | {} | {} | {:.2} / {:.2} / {:.2} / {:.2} |",
                    case.name,
                    s.members.len(),
                    s.nodes.len() * 6,
                    name,
                    el.mean,
                    el.p99,
                    disp,
                    stat.map_or("-".into(), |v| format!("{v:.4}")),
                    co.mean,
                    co.p99,
                    co.max,
                    sim.break_events().len(),
                    sim.break_events()
                        .first()
                        .map_or("-".into(), |e| format!("{:.3}", e.time)),
                    st.factorizations as f64 / frames_collapse as f64,
                    st.stale_rejects as f64 / frames_collapse as f64,
                    st.backtracks as f64 / frames_collapse as f64,
                    st.extra_iterations as f64 / frames_collapse as f64,
                );
            }
        }
    }
}
