//! The hot-path gate CI runs (`CONTRIBUTING.md`, "Hot paths").
//!
//! cargo run --release --no-default-features -p breakit-core --example gate
//!
//! Exits with an error if any of these fails:
//!
//! 1. **SIMD is on.** On x86-64 with AVX2 and FMA, and on AArch64, the
//!    native level must dispatch to vector code.
//! 2. **SIMD pays.** Each kernel is timed on the native level and on the
//!    scalar reference in the same process; native must be faster by the
//!    factor in [`SIMD_GATES`]. A ratio measured on one machine in one run
//!    holds up on shared CI runners where absolute times do not.
//! 3. **Threads pay where they are on, and never change the answer.** The
//!    iterative solver and whole frames are run on 1, 2 and 4 threads: the
//!    result must be bit-identical, and from the size where threads are on
//!    by default the best threaded time must not be slower than one thread.
//! 4. **The solver does no more work.** Short deterministic runs must stay
//!    within their budgets of linear solves, CG iterations, factorisations
//!    and step-check retries ([`WORK_BUDGETS`]), which catches algorithmic
//!    regressions that timing on a noisy runner cannot.
//!
//! `--report` prints the tables and never fails.

mod common;

use breakit_core::linsolve::{tri, Backend, LinSolver, Policy, BLOCK_NNZ};
use breakit_core::*;
use common::{bridge, dof_blocks, tower, weight, Case};
use std::time::Instant;

/// (kernel, least speed-up of native over scalar). The limits sit well
/// under what was measured (see `docs/solver-spike.md`) so that only a
/// kernel that stopped vectorising trips them.
const SIMD_GATES: [(&str, f64); 3] = [
    ("block Cholesky factorisation, tower 5x5x12", 1.3),
    ("block Jacobi solve, tower 8x8x16", 1.05),
    ("element assembly, 20 frames of bridge 120 panels", 1.03),
];

/// Threaded time over single-thread time may not exceed this where threads
/// are on by default.
const THREAD_SLACK: f64 = 1.25;

/// (case, linear solves, CG iterations, factorisations, step-check
/// retries) for 60 elastic frames: the measured values (240, 482, 2, 0 and
/// 240, 479, 1, 0) plus 20%, and at least 2 more.
const WORK_BUDGETS: [(&str, usize, usize, usize, usize); 2] = [
    ("tower 3x3x8", 288, 580, 4, 2),
    ("bridge 20 panels", 288, 575, 3, 2),
];

struct Rng(u64);

impl Rng {
    fn unit(&mut self) -> f64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        (self.0 >> 11) as f64 / (1u64 << 53) as f64
    }
}

/// Any positive definite values do: the work does not depend on them. Each
/// beam gets a diagonally dominant block.
fn fill(lin: &mut LinSolver, beams: usize, rng: &mut Rng) {
    lin.clear();
    for d in 0..lin.ndof() {
        lin.add_diagonal(d, 1.0);
    }
    for i in 0..beams {
        let mut k = [0.0; BLOCK_NNZ];
        for r in 0..12 {
            for c in 0..r {
                k[tri(r, c)] = 0.2 * (rng.unit() - 0.5);
            }
            k[tri(r, r)] = 2.0 + rng.unit();
        }
        lin.add_block(i, &k);
    }
}

/// Seconds of one numeric factorisation of `case`'s matrix on `level`.
fn factor_seconds(case: &Case, level: SimdLevel) -> f64 {
    let (n, blocks) = dof_blocks(&case.s);
    let mut lin = LinSolver::new(n, &blocks, Backend::Cholesky).expect("the case has unknowns");
    lin.set_simd(level);
    fill(&mut lin, blocks.len(), &mut Rng(0x9E3779B97F4A7C15));
    let (rhs, mut x) = (vec![1.0; n], vec![0.0; n]);
    let policy = Policy {
        tolerance: 1e-30,
        refactor_cost: usize::MAX / 4,
        max_iters: 1,
        max_age: 0,
    };
    assert!(lin.solve(&rhs, &mut x, &policy));
    lin.stats.factor_seconds
}

/// Seconds of one block-Jacobi solve of `case`'s matrix from zero, and the
/// solution, on `level` with the passes threaded from `par_min_nodes` up.
fn jacobi_solve(case: &Case, level: SimdLevel, par_min_nodes: usize) -> (f64, Vec<f64>) {
    let (n, blocks) = dof_blocks(&case.s);
    let mut lin = LinSolver::new(n, &blocks, Backend::BlockJacobi).expect("the case has unknowns");
    lin.set_simd(level);
    lin.set_parallel_min_nodes(par_min_nodes);
    let mut rng = Rng(0x9E3779B97F4A7C15);
    fill(&mut lin, blocks.len(), &mut rng);
    let rhs: Vec<f64> = (0..n).map(|_| rng.unit() - 0.5).collect();
    let mut x = vec![0.0; n];
    let policy = Policy {
        tolerance: 1e-6,
        refactor_cost: 0,
        max_iters: 0,
        max_age: 0,
    };
    let t0 = Instant::now();
    assert!(lin.solve(&rhs, &mut x, &policy));
    (t0.elapsed().as_secs_f64(), x)
}

/// `frames` elastic frames of `case` under its sideways push: seconds in
/// assembly, seconds in all, and where the probe node ended up.
fn run_frames(case: &Case, config: SimConfig, frames: usize) -> (f64, f64, Vec3) {
    let s = &case.s;
    let mut sim = DynamicSim::new(
        s,
        SimConfig {
            damping: 8.0,
            plasticity: false,
            ..config
        },
    );
    let push = case.push_dir * (0.2 * weight(s) / case.pushed.len() as f64);
    for &n in &case.pushed {
        sim.set_force(n, push);
    }
    let t0 = Instant::now();
    for _ in 0..frames {
        sim.step(1.0 / 60.0);
    }
    let total = t0.elapsed().as_secs_f64();
    let assembly = sim.solver_stats().linear.assembly_seconds;
    (assembly, total, sim.displacement(case.probe))
}

/// The least of `reps` timings of each of the two closures, interleaved so
/// that a slow spell on a shared machine hits both.
fn race(reps: usize, mut a: impl FnMut() -> f64, mut b: impl FnMut() -> f64) -> (f64, f64) {
    let (mut ta, mut tb) = (f64::MAX, f64::MAX);
    for _ in 0..reps {
        ta = ta.min(a());
        tb = tb.min(b());
    }
    (ta, tb)
}

fn main() {
    let report_only = std::env::args().any(|a| a == "--report");
    let mut failures: Vec<String> = Vec::new();

    // 1. Dispatch.
    let native = SimdLevel::Native.name();
    println!("SIMD level: {native}");
    #[cfg(target_arch = "x86_64")]
    let expect_simd =
        std::arch::is_x86_feature_detected!("avx2") && std::arch::is_x86_feature_detected!("fma");
    #[cfg(not(target_arch = "x86_64"))]
    let expect_simd = cfg!(target_arch = "aarch64");
    if expect_simd && native == "scalar" {
        failures.push("the native level fell back to scalar code on a CPU with SIMD".into());
    }

    // 2. Native against the scalar reference.
    println!("\n| kernel | scalar ms | {native} ms | speed-up | required |");
    println!("|---|---|---|---|---|");
    let (tower_l, tower_xl) = (tower(5, 5, 12, 1), tower(8, 8, 16, 1));
    let bridge_xl = bridge(120, 4);
    let serial = |simd| SimConfig {
        simd,
        parallel_min_members: usize::MAX,
        ..Default::default()
    };
    let timings = [
        race(
            9,
            || factor_seconds(&tower_l, SimdLevel::Scalar),
            || factor_seconds(&tower_l, SimdLevel::Native),
        ),
        race(
            9,
            || jacobi_solve(&tower_xl, SimdLevel::Scalar, usize::MAX).0,
            || jacobi_solve(&tower_xl, SimdLevel::Native, usize::MAX).0,
        ),
        race(
            5,
            || run_frames(&bridge_xl, serial(SimdLevel::Scalar), 20).0,
            || run_frames(&bridge_xl, serial(SimdLevel::Native), 20).0,
        ),
    ];
    for ((name, required), (scalar, simd)) in SIMD_GATES.into_iter().zip(timings) {
        let speedup = scalar / simd;
        println!(
            "| {name} | {:.3} | {:.3} | {speedup:.2} | {required:.2} |",
            scalar * 1e3,
            simd * 1e3
        );
        if expect_simd && speedup < required {
            failures.push(format!(
                "{name}: {native} is only {speedup:.2}x the scalar reference"
            ));
        }
    }

    // 3. Threads: same bits, and no loss where they are on by default.
    println!("\n| block Jacobi solve | nodes | threads on by default | 1 thread ms | 2 threads ms | 4 threads ms |");
    println!("|---|---|---|---|---|---|");
    let cores = std::thread::available_parallelism().map_or(1, |n| n.get());
    for case in [
        tower(8, 8, 16, 1),
        tower(14, 14, 24, 1),
        tower(20, 20, 30, 1),
    ] {
        let nodes = case.s.nodes.len();
        let (_, reference) = jacobi_solve(&case, SimdLevel::Native, usize::MAX);
        let mut ms = [0.0; 3];
        for (t, threads) in ms.iter_mut().zip([1, 2, 4]) {
            let pool = rayon::ThreadPoolBuilder::new()
                .num_threads(threads)
                .build()
                .expect("a thread pool can be built");
            let mut best = f64::MAX;
            for _ in 0..5 {
                let (seconds, x) = pool.install(|| jacobi_solve(&case, SimdLevel::Native, 0));
                best = best.min(seconds);
                if x != reference {
                    failures.push(format!(
                        "{}: the solve on {threads} threads is not bit-identical to one thread",
                        case.name
                    ));
                }
            }
            *t = best * 1e3;
        }
        // The default path: threaded from the solver's own threshold.
        let on = {
            let (n, blocks) = dof_blocks(&case.s);
            let lin = LinSolver::new(n, &blocks, Backend::BlockJacobi).expect("unknowns");
            lin.runs_parallel()
        };
        println!(
            "| {} | {nodes} | {} | {:.2} | {:.2} | {:.2} |",
            case.name,
            if on { "yes" } else { "no" },
            ms[0],
            ms[1],
            ms[2]
        );
        let best_threaded = if cores >= 4 { ms[1].min(ms[2]) } else { ms[1] };
        if on && cores >= 2 && best_threaded > THREAD_SLACK * ms[0] {
            failures.push(format!(
                "{}: threads are on by default but cost {:.2} ms against {:.2} ms on one thread",
                case.name, best_threaded, ms[0]
            ));
        }
    }

    // Whole frames with the default configuration, which threads assembly
    // and the step check from `parallel_min_members` up.
    println!("\n| 20 frames, ms per frame (assembly / all) | members | 1 thread | 2 threads | 4 threads |");
    println!("|---|---|---|---|---|");
    {
        let case = &bridge_xl;
        let (_, _, reference) = run_frames(case, serial(SimdLevel::Native), 20);
        let mut ms = [(0.0, 0.0); 3];
        for (t, threads) in ms.iter_mut().zip([1, 2, 4]) {
            let pool = rayon::ThreadPoolBuilder::new()
                .num_threads(threads)
                .build()
                .expect("a thread pool can be built");
            let mut best = (f64::MAX, f64::MAX);
            for _ in 0..3 {
                let (assembly, total, probe) =
                    pool.install(|| run_frames(case, SimConfig::default(), 20));
                best = (best.0.min(assembly), best.1.min(total));
                if probe != reference {
                    failures.push(format!(
                        "{}: frames on {threads} threads are not bit-identical to the single-thread path",
                        case.name
                    ));
                }
            }
            *t = (best.0 * 1e3 / 20.0, best.1 * 1e3 / 20.0);
        }
        println!(
            "| {} | {} | {:.2} / {:.2} | {:.2} / {:.2} | {:.2} / {:.2} |",
            case.name,
            case.s.members.len(),
            ms[0].0,
            ms[0].1,
            ms[1].0,
            ms[1].1,
            ms[2].0,
            ms[2].1
        );
        let best_threaded = if cores >= 4 {
            ms[1].1.min(ms[2].1)
        } else {
            ms[1].1
        };
        if cores >= 2 && best_threaded > THREAD_SLACK * ms[0].1 {
            failures.push(format!(
                "{}: threads are on by default but a frame costs {:.2} ms against {:.2} ms on one thread",
                case.name, best_threaded, ms[0].1
            ));
        }
    }

    // 4. Work budgets.
    println!(
        "\n| 60 elastic frames | solves | CG iterations | factorisations | step-check retries |"
    );
    println!("|---|---|---|---|---|");
    for (case, budget) in [tower(3, 3, 8, 1), bridge(20, 2)].iter().zip(WORK_BUDGETS) {
        assert_eq!(case.name, budget.0);
        let s = &case.s;
        let mut sim = DynamicSim::new(
            s,
            SimConfig {
                damping: 8.0,
                plasticity: false,
                ..Default::default()
            },
        );
        let push = case.push_dir * (0.2 * weight(s) / case.pushed.len() as f64);
        for &n in &case.pushed {
            sim.set_force(n, push);
        }
        for _ in 0..60 {
            sim.step(1.0 / 60.0);
        }
        let st = sim.solver_stats();
        let work = [
            ("linear solves", st.linear.solves, budget.1),
            ("CG iterations", st.linear.cg_iterations, budget.2),
            ("factorisations", st.linear.factorizations, budget.3),
            ("step-check retries", st.extra_iterations, budget.4),
        ];
        println!(
            "| {} | {} | {} | {} | {} |",
            case.name, work[0].1, work[1].1, work[2].1, work[3].1
        );
        for (what, used, allowed) in work {
            if used > allowed {
                failures.push(format!("{}: {used} {what}, budget {allowed}", case.name));
            }
        }
    }

    if failures.is_empty() {
        println!("\nhot-path gate: passed");
    } else {
        println!("\nhot-path gate: FAILED");
        for f in &failures {
            println!("- {f}");
        }
        if !report_only {
            std::process::exit(1);
        }
    }
}
