//! Collapses onto the floor: cost and penetration with ground contact.
//!
//! cargo run --release --example floor -- [--sizes s,m,l] [--frames 600]
//!
//! A braced steel tower (the `bench` tower) is pushed sideways with a load
//! that grows 3% per frame until the first member breaks and is then held.
//! It is run without a floor (parts fall forever) and with
//! `SimConfig::ground`, for `--frames` 60 Hz frames. A third run stands the
//! same tower on the floor with no supports at all and lets it settle.
use breakit_core::*;
use std::time::Instant;

fn link(s: &mut Structure, a: usize, b: usize, mat: &Material, sec: &Section) {
    s.add_member(a, b, mat.clone(), sec.clone());
}

fn tower(nx: usize, nz: usize, floors: usize, supported: bool) -> (Structure, Vec<usize>) {
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
                link(&mut s, id(i, j, k), id(i, j, k + 1), &mat, &col);
                if i < nx {
                    link(&mut s, id(i, j, k + 1), id(i + 1, j, k + 1), &mat, &beam);
                    if (j == 0 || j == nz) && (i + k) % 2 == 0 {
                        link(&mut s, id(i, j, k), id(i + 1, j, k + 1), &mat, &brace);
                    }
                }
                if j < nz {
                    link(&mut s, id(i, j, k + 1), id(i, j + 1, k + 1), &mat, &beam);
                    if (i == 0 || i == nx) && (j + k) % 2 == 0 {
                        link(&mut s, id(i, j, k), id(i, j + 1, k + 1), &mat, &brace);
                    }
                }
            }
        }
    }
    if supported {
        for j in 0..=nz {
            for i in 0..=nx {
                s.add_support(Support::fixed(id(i, j, 0)));
            }
        }
    }
    let pushed = (0..=nz).map(|j| id(0, j, floors)).collect();
    (s, pushed)
}

fn timing(mut t: Vec<f64>) -> (f64, f64, f64) {
    t.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let mean = t.iter().sum::<f64>() / t.len() as f64;
    (
        mean,
        t[(t.len() * 99 / 100).min(t.len() - 1)],
        *t.last().unwrap(),
    )
}

/// Lowest node relative to where it touches the floor (or to y = 0).
fn deepest(sim: &DynamicSim, n: usize) -> f64 {
    (0..n)
        .map(|i| sim.clearance(i).unwrap_or(sim.position(i).y))
        .fold(f64::INFINITY, f64::min)
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let arg = |k: &str| {
        args.iter()
            .position(|a| a == k)
            .and_then(|i| args.get(i + 1).cloned())
    };
    let sizes = arg("--sizes").unwrap_or("s,m,l".into());
    let frames: usize = arg("--frames").map_or(600, |f| f.parse().unwrap());
    let dt = 1.0 / 60.0;
    println!("| tower | members | run | ms/frame mean / p99 / max | deepest mm | nodes on floor | touch-downs | breaks | refactor % | CG iters/solve |");
    println!("|---|---|---|---|---|---|---|---|---|---|");
    for size in sizes.split(',') {
        let (nx, nz, fl) = match size {
            "s" => (2, 2, 4),
            "m" => (3, 3, 8),
            "l" => (5, 5, 12),
            _ => panic!("unknown size {size}"),
        };
        for run in ["no floor", "floor", "standing"] {
            let (s, pushed) = tower(nx, nz, fl, run != "standing");
            let w: f64 = s.self_weight_loads(9.81).iter().map(|l| -l.force.y).sum();
            let push = Vec3::new(1.0, 0.0, 0.3).normalize() * (0.2 * w / pushed.len() as f64);
            let cfg = SimConfig {
                ground: (run != "no floor").then(Ground::default),
                ..Default::default()
            };
            let mut sim = DynamicSim::new(&s, cfg);
            let mut scale = 1.0;
            let mut t = Vec::new();
            let mut low = f64::INFINITY;
            for _ in 0..frames {
                if run != "standing" && sim.break_events().is_empty() {
                    scale *= 1.03;
                    for &n in &pushed {
                        sim.set_force(n, push * scale);
                    }
                }
                let t0 = Instant::now();
                sim.step(dt);
                t.push(t0.elapsed().as_secs_f64() * 1e3);
                low = low.min(deepest(&sim, s.nodes.len()));
            }
            let (mean, p99, max) = timing(t);
            let on = (0..s.nodes.len())
                .filter(|&n| sim.contact_force(n) > 0.0)
                .count();
            let st = sim.solver_stats().linear;
            println!(
                "| {nx}x{nz}x{fl} | {} | {run} | {mean:.2} / {p99:.2} / {max:.2} | {:.1} | {on} | {} | {} | {:.1} | {:.2} |",
                s.members.len(),
                low * 1e3,
                sim.contact_events().len(),
                sim.break_events().len(),
                100.0 * st.factorizations as f64 / st.solves.max(1) as f64,
                st.cg_iterations as f64 / st.solves.max(1) as f64,
            );
        }
    }
}
