#![allow(dead_code, reason = "each test file uses part of the helpers")]
//! Shared helpers for the water validation tests.
use breakit_water::{Config, Drive, FlumeSetup, Program, WaterSim};

pub const G: f64 = 9.81;
pub const DT: f64 = 1.0 / 60.0;

/// A one-cell-wide frictionless flume from x = 0, still level at y = 0.
pub fn flume(
    length: f64,
    depth: f64,
    dx: f64,
    outlet: bool,
    beach: Option<(f64, f64)>,
) -> WaterSim {
    WaterSim::flume(
        FlumeSetup {
            x: (0.0, length),
            z: (0.0, dx),
            dx,
            bed: -depth,
            still_level: 0.0,
            beach,
            outlet,
        },
        Config {
            manning: 0.0,
            ..Config::default()
        },
    )
}

/// The drive of a paddle program at time `t` on `depth`.
pub fn drive(program: &Program, t: f64, depth: f64) -> Drive {
    let (x, v) = program.motion(t, depth, G);
    Drive {
        paddle_velocity: v,
        paddle_position: x,
        current: 0.0,
        still_level: 0.0,
    }
}

/// Times at which `series` (time, value) crosses zero upwards.
pub fn upcrossings(series: &[(f64, f64)]) -> Vec<f64> {
    series
        .windows(2)
        .filter(|w| w[0].1 < 0.0 && w[1].1 >= 0.0)
        .map(|w| w[0].0 + (w[1].0 - w[0].0) * (-w[0].1) / (w[1].1 - w[0].1))
        .collect()
}

pub fn rel(a: f64, b: f64) -> f64 {
    (a / b - 1.0).abs()
}
