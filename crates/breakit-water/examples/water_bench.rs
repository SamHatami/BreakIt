//! Cost of the water on the game's flume: `cargo run --release -p
//! breakit-water --example water_bench`.
use std::time::Instant;

use breakit_water::{Config, Drive, Field, FlumeSetup, Program, WaterSim};

fn main() {
    for (dx, dispersive, tol) in [
        (0.2, true, 1e-3),
        (0.15, true, 1e-3),
        (0.1, true, 1e-3),
        (0.1, false, 1e-3),
    ] {
        let mut sim = WaterSim::flume(
            FlumeSetup {
                x: (-13.6, 14.0),
                z: (-3.0, 3.0),
                dx,
                bed: -3.2,
                still_level: -0.6,
                beach: Some((6.0, 0.4)),
                outlet: false,
            },
            Config {
                dispersive,
                pressure_tolerance: tol,
                ..Config::default()
            },
        );
        let program = Program::Regular {
            height: 0.6,
            period: 3.5,
            ramp: 4.0,
        };
        let dt = 1.0 / 60.0;
        let frames = 1200;
        let (mut solver_s, mut total_s, mut substeps, mut iterations) = (0.0, 0.0, 0, 0);
        let mut worst: f64 = 0.0;
        for f in 0..frames {
            let (x, v) = program.motion(f as f64 * dt, 2.6, 9.81);
            let drive = Drive {
                paddle_velocity: v,
                paddle_position: x,
                current: 0.0,
                still_level: -0.6,
            };
            let t0 = Instant::now();
            let stats = sim.solver.advance(dt);
            solver_s += t0.elapsed().as_secs_f64();
            let _ = Field::capture(&sim.solver);
            let t1 = Instant::now();
            sim.advance(0.0, drive);
            sim.solver.paddle_velocity = v;
            let frame = t0.elapsed().as_secs_f64() + t1.elapsed().as_secs_f64();
            total_s += frame;
            worst = worst.max(frame);
            substeps += stats.substeps;
            iterations += stats.pressure_iterations;
        }
        let n = frames as f64;
        println!(
            "dx {dx} m disp {dispersive} tol {tol}, {} cells: solver {:.2} ms/frame (worst frame {:.2} ms), {:.1} substeps, {:.1} pressure iterations per frame",
            sim.solver.grid.cells(),
            1e3 * solver_s / n,
            1e3 * worst,
            substeps as f64 / n,
            iterations as f64 / n
        );
        let _ = total_s;
    }
}
