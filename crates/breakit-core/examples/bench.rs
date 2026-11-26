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
mod common;

use breakit_core::*;
use common::{cases, weight};
use std::time::Instant;

/// Solver configurations under test: (name, config).
fn configs() -> Vec<(&'static str, SimConfig)> {
    let d = SimConfig::default;
    let with = |backend, tol| SimConfig {
        linear_backend: backend,
        linear_tolerance: tol,
        ..d()
    };
    use linsolve::Backend::*;
    vec![
        ("default", d()),
        (
            "parallel",
            SimConfig {
                parallel_min_members: 0,
                ..d()
            },
        ),
        ("cholesky", with(Cholesky, 1e-6)),
        ("cholesky-1e-4", with(Cholesky, 1e-4)),
        ("scalar", with(CholeskyScalar, 1e-6)),
        ("scalar-1e-4", with(CholeskyScalar, 1e-4)),
        ("jacobi", with(BlockJacobi, 1e-6)),
        ("jacobi-1e-4", with(BlockJacobi, 1e-4)),
        ("jacobi-1e-3", with(BlockJacobi, 1e-3)),
        (
            "16-substeps",
            SimConfig {
                substeps: 16,
                ..d()
            },
        ),
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

/// Time the kernels of the two Cholesky backends on the matrix pattern of
/// each case, outside the simulation: one numeric factorisation, one direct
/// solve (a triangular solve pair) and one CG iteration (a triangular solve
/// pair, a product with the matrix and the vector updates). Minimum over
/// repetitions.
fn kernels(sizes: &str) {
    use linsolve::{tri, Backend, LinSolver, Policy, BLOCK_NNZ};
    println!(
        "| structure | members | unknowns | backend | factor entries | factor flops (sum of squared column counts) | auto refactor cost | us per factorisation | us per direct solve | us per CG iteration |"
    );
    println!("|---|---|---|---|---|---|---|---|---|---|");
    for size in sizes.split(',') {
        for case in cases(size) {
            let s = &case.s;
            // The DOF numbering of the dynamic solver.
            let mut fixed = vec![(false, false); s.nodes.len()];
            for sup in &s.supports {
                fixed[sup.node].0 |= sup.fixes_translation();
                fixed[sup.node].1 |= sup.fixes_rotation();
            }
            let mut n = 0;
            let mut next = |free: bool| {
                free.then(|| {
                    n += 3;
                    n - 3
                })
            };
            let slots: Vec<_> = fixed.iter().map(|f| (next(!f.0), next(!f.1))).collect();
            let blocks: Vec<_> = s
                .members
                .iter()
                .map(|m| [slots[m.a].0, slots[m.a].1, slots[m.b].0, slots[m.b].1])
                .collect();
            // Any positive definite values do: the work does not depend on
            // them. Each beam gets a diagonally dominant block.
            let mut seed = 0x9E3779B97F4A7C15u64;
            let mut unit = move || {
                seed ^= seed << 13;
                seed ^= seed >> 7;
                seed ^= seed << 17;
                (seed >> 11) as f64 / (1u64 << 53) as f64
            };
            let assemble = |lin: &mut LinSolver, unit: &mut dyn FnMut() -> f64| {
                lin.clear();
                for d in 0..n {
                    lin.add_diagonal(d, 1.0);
                }
                for i in 0..blocks.len() {
                    let mut k = [0.0; BLOCK_NNZ];
                    for r in 0..12 {
                        for c in 0..r {
                            k[tri(r, c)] = 0.2 * (unit() - 0.5);
                        }
                        k[tri(r, r)] = 2.0 + unit();
                    }
                    lin.add_block(i, &k);
                }
            };
            let rhs: Vec<f64> = (0..n).map(|_| unit() - 0.5).collect();
            let mut x = vec![0.0; n];
            for (name, backend) in [
                ("faer scalar", Backend::CholeskyScalar),
                ("node blocks", Backend::Cholesky),
            ] {
                let policy = |max_iters| Policy {
                    tolerance: 1e-30,
                    refactor_cost: usize::MAX / 4,
                    max_iters,
                    max_age: 0,
                };
                let (mut factor, mut direct, mut iteration) = (f64::MAX, f64::MAX, f64::MAX);
                let mut stats = linsolve::Stats::default();
                for _ in 0..8 {
                    // A fresh factorisation and a direct solve, then CG on
                    // that factorisation for a different matrix, to a
                    // tolerance it cannot reach: 5 and 25 iterations.
                    let mut cg = [0.0; 2];
                    for (t, iters) in cg.iter_mut().zip([5, 25]) {
                        let mut lin = LinSolver::new(n, &blocks, backend).unwrap();
                        assemble(&mut lin, &mut unit);
                        assert!(lin.solve(&rhs, &mut x, &policy(1)));
                        let a = lin.stats;
                        factor = factor.min(a.factor_seconds);
                        direct = direct.min(a.solve_seconds);
                        assemble(&mut lin, &mut unit);
                        assert!(lin.solve(&rhs, &mut x, &policy(iters)));
                        let b = lin.stats;
                        assert_eq!(b.cg_iterations - a.cg_iterations, iters);
                        *t = b.solve_seconds - a.solve_seconds;
                        stats = b;
                    }
                    iteration = iteration.min((cg[1] - cg[0]) / 20.0);
                }
                let lin = LinSolver::new(n, &blocks, backend).unwrap();
                assert_eq!(lin.stats.factor_entries, stats.factor_entries);
                println!(
                    "| {} | {} | {} | {name} | {} | {:.3e} | {} | {:.0} | {:.0} | {:.0} |",
                    case.name,
                    s.members.len(),
                    n,
                    lin.stats.factor_entries,
                    lin.stats.factor_flops,
                    lin.stats.auto_refactor_cost,
                    factor * 1e6,
                    direct * 1e6,
                    iteration * 1e6,
                );
            }
        }
    }
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
    let worst = args.iter().any(|a| a == "--worst");
    let short = args.iter().any(|a| a == "--short");
    if args.iter().any(|a| a == "--kernels") {
        return kernels(&sizes);
    }
    let frames_elastic = if short { 2 } else { 180 };
    let frames_collapse = if short { 2 } else { 300 };
    let dt = 1.0 / 60.0;

    println!(
        "| structure | members | dofs | solver | elastic ms/frame (mean / p99) | refactor % / CG iters per solve / ms assembly + factor + solve | probe disp mm | static mm | collapse ms/frame (mean / p99 / max) | refactor % / CG iters per solve / ms assembly + factor + solve | breaks |"
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
                let st_el = sim.solver_stats().linear;
                if !short {
                    sim.settle(20.0, 1e-6);
                }
                let disp = sim.displacement(case.probe).norm() * 1e3;

                // Collapse: ramp the push until something breaks, then hold.
                let mut sim = DynamicSim::new(s, cfg.clone());
                let mut scale = 1.0;
                let mut t = Vec::new();
                let mut frame_stats: Vec<(SolverStats, SolverStats)> = Vec::new();
                for _ in 0..frames_collapse {
                    if sim.break_events().is_empty() {
                        scale *= 1.03;
                        for &n in &case.pushed {
                            sim.set_force(n, push * scale);
                        }
                    }
                    let before = sim.solver_stats();
                    let t0 = Instant::now();
                    sim.step(dt);
                    t.push(t0.elapsed().as_secs_f64() * 1e3);
                    frame_stats.push((before, sim.solver_stats()));
                }
                if worst {
                    let st = sim.solver_stats();
                    let hist = |lo: usize, hi: usize| {
                        frame_stats
                            .iter()
                            .filter(|(a, b)| {
                                (lo..hi).contains(&(b.extra_iterations - a.extra_iterations))
                            })
                            .count()
                    };
                    eprintln!(
                        "  collapse totals: {} substeps, {} extra iterations, {} backtracks; frames with 0 / 1-2 / 3-4 / 5+ extra iterations: {} / {} / {} / {}",
                        st.substeps,
                        st.extra_iterations,
                        st.backtracks,
                        hist(0, 1),
                        hist(1, 3),
                        hist(3, 5),
                        hist(5, 99),
                    );
                    // The slowest collapse frames and what they spent.
                    let mut order: Vec<usize> = (0..t.len()).collect();
                    order.sort_by(|&a, &b| t[b].partial_cmp(&t[a]).unwrap());
                    for &f in order.iter().take(8) {
                        let (a, b) = &frame_stats[f];
                        eprintln!(
                            "  frame {f}: {:.1} ms, solves {}, factorisations {}, cg {}, fallbacks {}, backtracks {}, extra iterations {}, assembly {:.1} factor {:.1} solve {:.1} ms",
                            t[f],
                            b.linear.solves - a.linear.solves,
                            b.linear.factorizations - a.linear.factorizations,
                            b.linear.cg_iterations - a.linear.cg_iterations,
                            b.linear.fallbacks - a.linear.fallbacks,
                            b.backtracks - a.backtracks,
                            b.extra_iterations - a.extra_iterations,
                            (b.linear.assembly_seconds - a.linear.assembly_seconds) * 1e3,
                            (b.linear.factor_seconds - a.linear.factor_seconds) * 1e3,
                            (b.linear.solve_seconds - a.linear.solve_seconds) * 1e3,
                        );
                    }
                }
                let co = timing(t);
                let st_co = sim.solver_stats().linear;
                let work = |st: linsolve::Stats, frames: usize| {
                    let ms = |s: f64| s * 1e3 / frames as f64;
                    format!(
                        "{:.1}% / {:.2} / {:.2} + {:.2} + {:.2}",
                        100.0 * st.factorizations as f64 / st.solves.max(1) as f64,
                        st.cg_iterations as f64 / st.solves.max(1) as f64,
                        ms(st.assembly_seconds),
                        ms(st.factor_seconds),
                        ms(st.solve_seconds),
                    )
                };
                println!(
                    "| {} | {} | {} | {} | {:.2} / {:.2} | {} | {:.4} | {} | {:.2} / {:.2} / {:.2} | {} | {} |",
                    case.name,
                    s.members.len(),
                    s.nodes.len() * 6,
                    name,
                    el.mean,
                    el.p99,
                    work(st_el, frames_elastic),
                    disp,
                    stat.map_or("-".into(), |v| format!("{v:.4}")),
                    co.mean,
                    co.p99,
                    co.max,
                    work(st_co, frames_collapse),
                    sim.break_events().len(),
                );
            }
        }
    }
}
