//! Simulated water on the play table (see `docs/water.md`).
//!
//! A table that holds water gets a [`LiveWater`]: at rest while building,
//! driven by the sheet's water case during a test. The same simulated
//! water that is drawn pushes on the structure: each physics step the
//! water advances, the structure slows it where it stands in it, and the
//! core load case reads its kinematics from the result.
//!
//! The water can be stepped on the GPU ([`WaterBackend`], F6 or
//! `BREAKIT_WATER=gpu`): the same scheme as compute passes, on a finer
//! grid, read back a step later into the same [`LiveWater`].

use bevy::ecs::system::SystemParam;
use bevy::prelude::*;
use bevy::render::renderer::{RenderDevice, RenderQueue};
use breakit_core::loads::water::Water;
use breakit_core::loads::{Geometry, LoadKind, View};
use breakit_lab::{LabQuality, LabSettings, PlayTable, TableWater};
use breakit_scenario::TestRun;
use breakit_water::{Config, Drive, FlumeSetup, WaterSim};
use breakit_water_gpu::{GpuSolver, Relaxation};
use breakit_water_render::{LiveWater, WaterLook, WaterQuality, WaterRenderPlugin, WaterRenderSet};

use crate::analysis::{BreakitSet, Simulation};
use crate::flow::Screen;
use crate::scenario::Sheets;

pub struct WaterPlugin;

impl Plugin for WaterPlugin {
    fn build(&self, app: &mut App) {
        let backend = match std::env::var("BREAKIT_WATER").as_deref() {
            Ok("gpu") => WaterBackend::Gpu,
            _ => WaterBackend::Cpu,
        };
        app.add_plugins(WaterRenderPlugin)
            .insert_resource(backend)
            .init_resource::<WaterGpu>()
            .add_systems(Update, backend_key.before(sync_water))
            .configure_sets(Update, WaterRenderSet.after(BreakitSet::Physics))
            .add_systems(Update, sync_water.before(BreakitSet::Physics))
            .add_systems(OnExit(Screen::Report), still_water)
            .add_systems(OnEnter(Screen::Build), still_water);
    }
}

/// The built-in play table `index`, with its water left to the simulation
/// where the simulation draws it (a flume; a ravine keeps its stream).
pub fn play_table(index: usize) -> PlayTable {
    let table = PlayTable::builtin(index);
    if simulated(&table).is_some() {
        table.without_still_water()
    } else {
        table
    }
}

/// The water of `table` that is simulated: a flume's.
fn simulated(table: &PlayTable) -> Option<TableWater> {
    table.water().filter(|w| w.paddle)
}

/// What ties the water to the running test.
#[derive(Resource)]
pub struct WaterLink {
    /// Index of the water case in the run's load set.
    case: usize,
    geometry: Geometry,
}

/// Where the water is stepped.
#[derive(Resource, Clone, Copy, Debug, PartialEq, Eq)]
pub enum WaterBackend {
    /// The reference solver, on one core.
    Cpu,
    /// Compute passes on the GPU the game draws with, on a finer grid.
    /// Falls back to the CPU where there is no render device.
    Gpu,
}

/// The GPU half of the water while [`WaterBackend::Gpu`] steps it; made
/// at the first step after the water was (re)built.
#[derive(Resource, Default)]
pub struct WaterGpu(Option<GpuSolver>);

/// F6 switches the water between the CPU and the GPU.
fn backend_key(keys: Res<ButtonInput<KeyCode>>, mut backend: ResMut<WaterBackend>) {
    if keys.just_pressed(KeyCode::F6) {
        *backend = match *backend {
            WaterBackend::Cpu => WaterBackend::Gpu,
            WaterBackend::Gpu => WaterBackend::Cpu,
        };
        info!("water backend: {:?}", *backend);
    }
}

/// One line for the frame-time read-out: where the water runs and on what
/// grid.
pub fn status_line(backend: WaterBackend, gpu: &WaterGpu, live: Option<&LiveWater>) -> String {
    let Some(live) = live else {
        return "water: none".into();
    };
    let g = live.sim.solver.grid;
    let on = match (backend, &gpu.0) {
        (WaterBackend::Gpu, Some(_)) => "GPU",
        (WaterBackend::Gpu, None) => "GPU (idle)",
        (WaterBackend::Cpu, _) => "CPU",
    };
    format!(
        "water: {on}  {:.0} cm cells  {}x{} (F6)",
        g.dx * 100.0,
        g.nx,
        g.nz
    )
}

/// Solver cell size by quality, m: the cost goes with the cell count, and
/// the GPU takes a finer grid than one core.
fn cell_size(quality: LabQuality, backend: WaterBackend) -> f64 {
    match (backend, quality) {
        (WaterBackend::Cpu, LabQuality::Low) => 0.25,
        (WaterBackend::Cpu, LabQuality::Medium | LabQuality::High) => 0.2,
        (WaterBackend::Cpu, LabQuality::Ultra) => 0.15,
        (WaterBackend::Gpu, LabQuality::Low) => 0.2,
        (WaterBackend::Gpu, LabQuality::Medium | LabQuality::High) => 0.15,
        (WaterBackend::Gpu, LabQuality::Ultra) => 0.1,
    }
}

fn water_quality(quality: LabQuality) -> WaterQuality {
    match quality {
        LabQuality::Low => WaterQuality::Low,
        LabQuality::Medium => WaterQuality::Medium,
        LabQuality::High => WaterQuality::High,
        LabQuality::Ultra => WaterQuality::Ultra,
    }
}

/// The table's water at rest, or as the water case of a test wants it.
/// `rest` is the still level the sheet's water case starts from, so the
/// water stands where the test will have it while the player builds.
fn build_water(basin: &TableWater, case: Option<&Water>, rest: Option<f64>, dx: f64) -> LiveWater {
    let level = case.map_or(rest.unwrap_or(basin.level as f64), |w| w.still_level(0.0));
    // A current has to leave at the far end; still water in a closed
    // flume stays put.
    let flowing = case.is_some_and(|w| w.current.norm() > 0.0);
    let setup = FlumeSetup {
        x: (basin.min.x as f64, basin.max.x as f64),
        z: (basin.min.y as f64, basin.max.y as f64),
        dx,
        bed: basin.bed as f64,
        still_level: level,
        beach: basin.beach.map(|(toe, slope)| (toe as f64, slope as f64)),
        outlet: flowing || (basin.paddle && basin.beach.is_none() && case.is_some()),
    };
    LiveWater {
        sim: WaterSim::flume(setup, Config::default()),
        paddle: basin.paddle,
    }
}

/// Keeps the water in step with the table, the quality preset and the
/// start of a test.
#[allow(clippy::too_many_arguments)]
fn sync_water(
    mut commands: Commands,
    table: Res<PlayTable>,
    lab: Res<LabSettings>,
    sim: Option<Res<Simulation>>,
    live: Option<Res<LiveWater>>,
    link: Option<Res<WaterLink>>,
    mut look: ResMut<WaterLook>,
    backend: Res<WaterBackend>,
    mut gpu: ResMut<WaterGpu>,
    sheets: Res<Sheets>,
    mut built_for: Local<Option<(LabQuality, WaterBackend, Option<u64>)>>,
) {
    let want = water_quality(lab.quality).look();
    if *look != want {
        *look = want;
    }
    let Some(basin) = simulated(&table) else {
        if live.is_some() {
            commands.remove_resource::<LiveWater>();
            commands.remove_resource::<WaterLink>();
            gpu.0 = None;
        }
        return;
    };
    let dx = cell_size(lab.quality, *backend);
    let rest = sheet_level(&sheets);
    let built = Some((lab.quality, *backend, rest.map(f64::to_bits)));
    // A test has just started: set the water up for its water case.
    let starting = sim.as_ref().filter(|s| s.is_added() || link.is_none());
    if let Some(run) = starting.and_then(|s| s.run()) {
        let case = run.loads().cases.iter().enumerate().find_map(|(i, c)| {
            if let LoadKind::Water(w) = &c.kind {
                Some((i, w))
            } else {
                None
            }
        });
        if let Some((index, water)) = case {
            commands.insert_resource(build_water(&basin, Some(water), rest, dx));
            commands.insert_resource(WaterLink {
                case: index,
                geometry: Geometry::new(run.structure()),
            });
            gpu.0 = None;
            *built_for = built;
            return;
        }
    }
    // The grid follows the preset and the backend, but not under a
    // running test: that would reset its water.
    let stale = *built_for != built && sim.is_none();
    if live.is_none() || table.is_changed() || stale {
        commands.insert_resource(build_water(&basin, None, rest, dx));
        commands.remove_resource::<WaterLink>();
        gpu.0 = None;
        *built_for = built;
    }
}

/// Back to still water when the test is over.
fn still_water(
    mut commands: Commands,
    table: Res<PlayTable>,
    lab: Res<LabSettings>,
    link: Option<Res<WaterLink>>,
    backend: Res<WaterBackend>,
    sheets: Res<Sheets>,
    mut gpu: ResMut<WaterGpu>,
) {
    if link.is_none() {
        return;
    }
    commands.remove_resource::<WaterLink>();
    if let Some(basin) = simulated(&table) {
        let dx = cell_size(lab.quality, *backend);
        commands.insert_resource(build_water(&basin, None, sheet_level(&sheets), dx));
        gpu.0 = None;
    }
}

/// The still water level of the current sheet's water case, if it has one.
fn sheet_level(sheets: &Sheets) -> Option<f64> {
    sheets.current().def.events.iter().find_map(|e| match e {
        breakit_scenario::Event::Water { level, .. } => Some(*level),
        _ => None,
    })
}

/// What a physics step needs to step the water with it.
#[derive(SystemParam)]
pub struct WaterStep<'w> {
    live: Option<ResMut<'w, LiveWater>>,
    link: Option<Res<'w, WaterLink>>,
    backend: Res<'w, WaterBackend>,
    gpu: ResMut<'w, WaterGpu>,
    device: Option<Res<'w, RenderDevice>>,
    queue: Option<Res<'w, RenderQueue>>,
}

impl WaterStep<'_> {
    /// One physics step of the water of a running test, if the table has
    /// simulated water; see [`step`].
    pub fn step(&mut self, run: &mut TestRun, dt: f64) {
        let (Some(live), Some(link)) = (self.live.as_mut(), self.link.as_ref()) else {
            return;
        };
        let on_gpu = match (*self.backend, &self.device, &self.queue) {
            (WaterBackend::Gpu, Some(device), Some(queue)) => {
                let solver = self.gpu.0.get_or_insert_with(|| {
                    GpuSolver::new(
                        device.wgpu_device(),
                        &live.sim.solver,
                        Relaxation::default(),
                    )
                });
                Some((solver, device.wgpu_device(), &**queue.as_ref()))
            }
            _ => {
                // Back on the CPU: it carries on from the last state read
                // back.
                self.gpu.0 = None;
                None
            }
        };
        step(live, link, run, dt, on_gpu);
    }
}

/// One physics step of the water, to call just before the run's own step:
/// the structure slows the water, the rig drives it, and the run's water
/// case then reads the result. With `on_gpu` the GPU takes the solver step
/// and the result the loads read is the one that has come back, a step
/// behind.
pub fn step(
    live: &mut LiveWater,
    link: &WaterLink,
    run: &mut TestRun,
    dt: f64,
    on_gpu: Option<(&mut GpuSolver, &wgpu::Device, &wgpu::Queue)>,
) {
    let t = run.sim().time();
    let Some(case) = run.loads().cases.get(link.case) else {
        return;
    };
    let LoadKind::Water(water) = &case.kind else {
        return;
    };
    let drive = Drive::from_case(water, t, case.window.factor(t));
    let (cd_tube, cd_rect) = (water.coefficients.tube_cd, water.coefficients.rect_cd);
    live.sim.couple(
        &View {
            structure: run.structure(),
            sim: run.sim(),
            geometry: &link.geometry,
            time: t,
        },
        cd_tube,
        cd_rect,
    );
    match on_gpu {
        Some((gpu, device, queue)) => {
            live.sim
                .advance_with(dt, drive, |solver, dt| gpu.step(device, queue, solver, dt));
        }
        None => {
            live.sim.advance(dt, drive);
        }
    }
    let field = live.sim.field();
    if let Some(LoadKind::Water(water)) = run
        .loads_mut()
        .cases
        .get_mut(link.case)
        .map(|c| &mut c.kind)
    {
        water.field = Some(field);
    }
}
