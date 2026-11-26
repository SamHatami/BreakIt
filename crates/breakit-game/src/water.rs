//! Simulated water on the play table (see `docs/water.md`).
//!
//! A table that holds water gets a [`LiveWater`]: at rest while building,
//! driven by the sheet's water case during a test. The same simulated
//! water that is drawn pushes on the structure: each physics step the
//! water advances, the structure slows it where it stands in it, and the
//! core load case reads its kinematics from the result.

use bevy::prelude::*;
use breakit_core::loads::water::Water;
use breakit_core::loads::{Geometry, LoadKind, View};
use breakit_lab::{LabQuality, LabSettings, PlayTable, TableWater};
use breakit_scenario::TestRun;
use breakit_water::{Config, Drive, FlumeSetup, WaterSim};
use breakit_water_render::{LiveWater, WaterLook, WaterQuality, WaterRenderPlugin, WaterRenderSet};

use crate::analysis::{BreakitSet, Simulation};
use crate::flow::Screen;

pub struct WaterPlugin;

impl Plugin for WaterPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(WaterRenderPlugin)
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

/// Solver cell size by quality, m: the cost goes with the cell count.
fn cell_size(quality: LabQuality) -> f64 {
    match quality {
        LabQuality::Low => 0.25,
        LabQuality::Medium | LabQuality::High => 0.2,
        LabQuality::Ultra => 0.15,
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
fn build_water(basin: &TableWater, case: Option<&Water>, dx: f64) -> LiveWater {
    let level = case.map_or(basin.level as f64, |w| w.still_level(0.0));
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
    mut built_for: Local<Option<LabQuality>>,
) {
    let want = water_quality(lab.quality).look();
    if *look != want {
        *look = want;
    }
    let Some(basin) = simulated(&table) else {
        if live.is_some() {
            commands.remove_resource::<LiveWater>();
            commands.remove_resource::<WaterLink>();
        }
        return;
    };
    let dx = cell_size(lab.quality);
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
            commands.insert_resource(build_water(&basin, Some(water), dx));
            commands.insert_resource(WaterLink {
                case: index,
                geometry: Geometry::new(run.structure()),
            });
            *built_for = Some(lab.quality);
            return;
        }
    }
    let stale = *built_for != Some(lab.quality) && sim.is_none();
    if live.is_none() || table.is_changed() || stale {
        commands.insert_resource(build_water(&basin, None, dx));
        commands.remove_resource::<WaterLink>();
        *built_for = Some(lab.quality);
    }
}

/// Back to still water when the test is over.
fn still_water(
    mut commands: Commands,
    table: Res<PlayTable>,
    lab: Res<LabSettings>,
    link: Option<Res<WaterLink>>,
) {
    if link.is_none() {
        return;
    }
    commands.remove_resource::<WaterLink>();
    if let Some(basin) = simulated(&table) {
        commands.insert_resource(build_water(&basin, None, cell_size(lab.quality)));
    }
}

/// One physics step of the water, to call just before the run's own step:
/// the structure slows the water, the rig drives it, and the run's water
/// case then reads the result.
pub fn step(live: &mut LiveWater, link: &WaterLink, run: &mut TestRun, dt: f64) {
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
    live.sim.advance(dt, drive);
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
