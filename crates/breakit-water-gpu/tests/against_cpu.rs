//! The GPU solver against the CPU solver it mirrors, within a tolerance:
//! the GPU works in `f32` and rounds differently from one GPU to the next.
//!
//! These tests need a GPU adapter (a software one will do) and pass
//! without checking anything where there is none.

use breakit_water::{
    Boundary, Config, Drive, FlumeSetup, Grid, PressureSolver, Side, Solver, WaterSim,
};
use breakit_water_gpu::{GpuSolver, Relaxation};

const DT: f64 = 1.0 / 60.0;

fn device() -> Option<(wgpu::Device, wgpu::Queue)> {
    futures_lite::future::block_on(async {
        let instance = wgpu::Instance::new(wgpu::InstanceDescriptor::new_without_display_handle());
        let adapter = instance
            .request_adapter(&wgpu::RequestAdapterOptions::default())
            .await
            .ok()?;
        adapter
            .request_device(&wgpu::DeviceDescriptor::default())
            .await
            .ok()
    })
    .or_else(|| {
        eprintln!("no GPU adapter: skipped");
        None
    })
}

/// The CPU configuration the GPU is compared with: the same pressure
/// relaxation.
fn config() -> Config {
    let r = Relaxation::default();
    Config {
        pressure_solver: PressureSolver::RedBlack {
            sweeps: r.sweeps,
            omega: r.omega,
        },
        ..Config::default()
    }
}

/// A flume with a beach: waves from the paddle shoal, break and run up.
fn flume() -> WaterSim {
    WaterSim::flume(
        FlumeSetup {
            x: (0.0, 36.0),
            z: (0.0, 1.6),
            dx: 0.2,
            bed: 0.0,
            still_level: 1.6,
            beach: Some((20.0, 0.12)),
            outlet: false,
        },
        config(),
    )
}

fn paddle(t: f64) -> Drive {
    let omega = std::f64::consts::TAU / 3.0;
    let ramp = (t / 3.0).min(1.0);
    Drive {
        paddle_velocity: 0.45 * ramp * (omega * t).sin(),
        still_level: 1.6,
        ..Drive::default()
    }
}

/// Root mean square and largest difference of two fields.
fn difference(a: &[f64], b: &[f64]) -> (f64, f64) {
    let mut sum = 0.0;
    let mut most: f64 = 0.0;
    for (x, y) in a.iter().zip(b) {
        sum += (x - y).powi(2);
        most = most.max((x - y).abs());
    }
    ((sum / a.len() as f64).sqrt(), most)
}

#[test]
fn waves_on_a_beach_match_the_cpu_solver() {
    let Some((device, queue)) = device() else {
        return;
    };
    let mut cpu = flume();
    let mut gpu = flume();
    let mut on_gpu = GpuSolver::new(&device, &gpu.solver, Relaxation::default());
    let start = cpu.solver.volume();
    let mut broke = false;
    let mut wave: f64 = 0.0;
    for step in 0..(14.0 / DT) as usize {
        let t = step as f64 * DT;
        cpu.advance(DT, paddle(t));
        gpu.advance_with(DT, paddle(t), |solver, dt| {
            on_gpu.step(&device, &queue, solver, dt)
        });
        assert!(on_gpu.sync(&device, &mut gpu.solver), "the GPU answered");
        broke |= gpu.solver.breaking().iter().any(|b| *b);
        let (rms, most) = difference(cpu.solver.surface(), gpu.solver.surface());
        for z in cpu.solver.surface() {
            wave = wave.max(z - 1.6);
        }
        // Before anything breaks the two differ by rounding only; a broken
        // wave is turbulent in both and the fronts drift apart a little.
        if !broke {
            assert!(most < 2e-3, "t = {t:.2}: surfaces differ by {most} m");
        }
        assert!(rms < 0.02, "t = {t:.2}: surfaces differ by {rms} m rms");
    }
    assert!(wave > 0.25, "waves were made: crest {wave} m");
    assert!(broke, "the waves broke on the beach");
    assert!((gpu.solver.time() - cpu.solver.time()).abs() < 1e-6);
    let (rms, most) = difference(cpu.solver.surface(), gpu.solver.surface());
    eprintln!("after 14 s: {rms:.2e} m rms, {most:.2e} m at most, crest {wave:.3} m");
    // Volume: the paddle is a velocity boundary, so both gain and lose the
    // same water; f32 sums drift by a little.
    let (vc, vg) = (cpu.solver.volume(), gpu.solver.volume());
    assert!(
        (vc - vg).abs() < 2e-3 * start,
        "volume differs: {vc} and {vg} m³ of {start}"
    );
    // The foam rides on the mirror: it formed on the GPU run too.
    assert!(gpu.foam().iter().any(|f| *f > 0.2), "foam where it broke");
}

/// A basin with an open side, a current, a pile and a splash: the terms the
/// flume does not use.
fn basin() -> Solver {
    let mut s = Solver::new(Grid::new(40, 24, 0.25, 0.0, 0.0), config());
    s.set_bed(|x, z| 0.3 * (-((x - 7.0).powi(2) + (z - 2.0).powi(2)) / 4.0).exp());
    s.fill_to(1.0, |_, _| true);
    s.set_surface(|x, z| 1.0 + 0.2 * (-((x - 3.0).powi(2) + (z - 3.0).powi(2)) / 1.5).exp());
    s.set_boundary(
        Side::XMax,
        Boundary::Radiating {
            level: 1.0,
            current: 0.3,
        },
    );
    s.set_boundary(Side::XMin, Boundary::Piston);
    s.paddle_velocity = 0.3;
    s.sponge_velocity = 0.3;
    s.set_sponge(|x, _| if x > 9.0 { 1.0 } else { 0.0 });
    s
}

#[test]
fn a_basin_with_a_current_a_pile_and_a_splash_matches_the_cpu_solver() {
    let Some((device, queue)) = device() else {
        return;
    };
    let mut cpu = basin();
    let mut gpu = basin();
    let mut on_gpu = GpuSolver::new(&device, &gpu, Relaxation::default());
    for step in 0..240 {
        for s in [&mut cpu, &mut gpu] {
            s.clear_blockage();
            s.add_blockage(5.1, 3.1, 0.4, 0.0, 0.0);
            if step == 60 {
                s.displace(6.6, 4.6, 0.1);
            }
        }
        cpu.advance(DT);
        on_gpu.step(&device, &queue, &mut gpu, DT);
        assert!(on_gpu.sync(&device, &mut gpu));
        if step == 61 {
            let c = gpu.grid.c(26, 18);
            let dip = cpu.surface()[c] - gpu.surface()[c];
            assert!(
                dip.abs() < 0.02,
                "the splash reached the GPU: {dip} m apart"
            );
        }
    }
    let (rms, most) = difference(cpu.surface(), gpu.surface());
    eprintln!("basin after 4 s: {rms:.2e} m rms, {most:.2e} m at most");
    assert!(rms < 1e-3 && most < 5e-3, "surface: {rms} rms, {most} max");
    let (rms, most) = difference(cpu.face_u(), gpu.face_u());
    assert!(rms < 5e-3 && most < 0.03, "u: {rms} rms, {most} max");
    let (rms, most) = difference(cpu.face_v(), gpu.face_v());
    assert!(rms < 5e-3 && most < 0.03, "v: {rms} rms, {most} max");
    // The pile left a wake: slower water behind it than beside it.
    let (behind, _) = gpu.cell_velocity(22, 12);
    let (beside, _) = gpu.cell_velocity(22, 20);
    assert!(behind < beside, "wake: {behind} behind, {beside} beside");
}

#[test]
fn the_mirror_may_lag_without_changing_the_water() {
    let Some((device, queue)) = device() else {
        return;
    };
    // One run waits for every step, the other never does: its mirror is a
    // step or more behind while it runs.
    let mut waited = flume();
    let mut lagging = flume();
    let mut gpu_waited = GpuSolver::new(&device, &waited.solver, Relaxation::default());
    let mut gpu_lagging = GpuSolver::new(&device, &lagging.solver, Relaxation::default());
    let mut behind: f64 = 0.0;
    for step in 0..(8.0 / DT) as usize {
        let t = step as f64 * DT;
        waited.advance_with(DT, paddle(t), |s, dt| {
            gpu_waited.step(&device, &queue, s, dt)
        });
        assert!(gpu_waited.sync(&device, &mut waited.solver));
        lagging.advance_with(DT, paddle(t), |s, dt| {
            gpu_lagging.step(&device, &queue, s, dt)
        });
        behind = behind.max(gpu_lagging.time() - lagging.solver.time());
        assert!(gpu_lagging.in_flight() <= 6);
    }
    assert!(gpu_lagging.sync(&device, &mut lagging.solver));
    eprintln!("mirror at most {behind:.3} s behind");
    assert!(behind > 0.0, "the mirror did lag");
    assert!(behind < 0.5, "the mirror kept up: {behind} s behind");
    let (rms, most) = difference(waited.solver.surface(), lagging.solver.surface());
    eprintln!("lagging against waited: {rms:.2e} m rms, {most:.2e} m at most");
    assert!(rms < 0.02, "surfaces differ by {rms} m rms");
}
