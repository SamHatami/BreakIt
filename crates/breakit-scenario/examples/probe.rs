//! Prints the bill and a graded run of sample designs, for tuning scenario
//! numbers: `cargo run --release -p breakit-scenario --example probe`.

use std::time::Instant;

use breakit_core::{SimConfig, Structure};
use breakit_scenario::{builtin, evaluate, CostModel, PlayMode};

#[path = "../tests/common/mod.rs"]
mod common;
use common::{jetty, mast, platform, sign_tower, warren};

fn main() {
    let scenarios = builtin();
    let costs = CostModel::default();
    let cases: Vec<(&str, usize, Structure)> = vec![
        ("starter warren 50x5", 0, warren(0.05, 0.005, 1.0)),
        ("warren 40x4", 0, warren(0.04, 0.004, 1.0)),
        ("warren 30x3", 0, warren(0.03, 0.003, 1.0)),
        ("warren 25x2.5", 0, warren(0.025, 0.0025, 1.0)),
        ("warren 20x2", 0, warren(0.02, 0.002, 1.0)),
        ("warren 15x1.5", 0, warren(0.015, 0.0015, 1.0)),
        ("mast 50x5 x3", 1, mast(0.05, 0.005, 3)),
        ("mast 40x4 x3", 1, mast(0.04, 0.004, 3)),
        ("mast 30x3 x3", 1, mast(0.03, 0.003, 3)),
        ("mast 20x2 x3", 1, mast(0.02, 0.002, 3)),
        ("mast 40x4 x2", 1, mast(0.04, 0.004, 2)),
        ("mast 25x2.5 x2", 1, mast(0.025, 0.0025, 2)),
        ("platform 80x8 braced", 2, platform(0.08, 0.008, true)),
        ("platform 60x5 braced", 2, platform(0.06, 0.005, true)),
        ("platform 40x4 braced", 2, platform(0.04, 0.004, true)),
        ("platform 100x10 unbraced", 2, platform(0.1, 0.01, false)),
        ("platform 60x5 unbraced", 2, platform(0.06, 0.005, false)),
        ("sign tower 80x8 x4", 3, sign_tower(0.08, 0.008, 4)),
        ("sign tower 60x5 x4", 3, sign_tower(0.06, 0.005, 4)),
        ("sign tower 50x5 x4", 3, sign_tower(0.05, 0.005, 4)),
        ("sign tower 40x4 x4", 3, sign_tower(0.04, 0.004, 4)),
        ("sign tower 30x3 x4", 3, sign_tower(0.03, 0.003, 4)),
        ("sign tower 60x5 x2", 3, sign_tower(0.06, 0.005, 2)),
        ("sign tower 40x4 x2", 3, sign_tower(0.04, 0.004, 2)),
        ("sign tower 35x3.5 x2", 3, sign_tower(0.035, 0.0035, 2)),
        ("sign tower 30x3 x2", 3, sign_tower(0.03, 0.003, 2)),
        ("sign tower 25x2.5 x2", 3, sign_tower(0.025, 0.0025, 2)),
        ("sign tower 20x2 x2", 3, sign_tower(0.02, 0.002, 2)),
        ("jetty 80x8 braced", 4, jetty(0.08, 0.008, true)),
        ("jetty 60x5 braced", 4, jetty(0.06, 0.005, true)),
        ("jetty 40x4 braced", 4, jetty(0.04, 0.004, true)),
        ("jetty 30x3 braced", 4, jetty(0.03, 0.003, true)),
        ("jetty 25x2.5 braced", 4, jetty(0.025, 0.0025, true)),
        ("jetty 100x10 unbraced", 4, jetty(0.1, 0.01, false)),
        ("jetty 60x5 unbraced", 4, jetty(0.06, 0.005, false)),
        ("jetty 50x5 unbraced", 4, jetty(0.05, 0.005, false)),
    ];
    let only = std::env::args().nth(1);
    for (name, k, s) in cases {
        if only.as_deref().is_some_and(|o| !name.contains(o)) {
            continue;
        }
        let t = Instant::now();
        match evaluate(
            &scenarios[k],
            &s,
            &[],
            &costs,
            PlayMode::Challenge,
            SimConfig::default(),
        ) {
            Ok(r) => {
                println!(
                    "== {name} ({} members) in {:.2} s",
                    s.members.len(),
                    t.elapsed().as_secs_f64()
                );
                print!("{}", r.summary());
                println!(
                    "util {:.2}  reserve {:?}  warning {:?}  parts {:?}",
                    r.peak_utilization, r.reserve_factor, r.warning_time, r.parts
                );
            }
            Err(v) => {
                println!("== {name}: blocked");
                for x in v {
                    println!("  {x}");
                }
            }
        }
    }
}
