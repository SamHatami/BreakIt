//! Calls the lab's load rigs for the current sheet and drives them from
//! the running test: fan wall, shake actuators and seismograph, flume
//! paddle, model vehicles. The rigs themselves are `breakit_lab::rigs`;
//! what they do comes from `breakit_core::loads::RigState`.

use bevy::prelude::*;
use breakit_core::loads::seismic::TableRig;
use breakit_core::loads::traffic::{VehicleRig, WheelRig};
use breakit_core::loads::RigState;
use breakit_lab::rigs::{
    FanSetup, FlumeDrive, PaddleSetup, RigLive, RigSetup, ShakeSetup, TableDrive, VehicleDrive,
    WheelDrive, WindDrive,
};
use breakit_lab::{LabRigs, RigModels, RigSet};
use breakit_scenario as bs;
use breakit_scenario::{Binding, TestRun};

use crate::analysis::{BreakitSet, Simulation};
use crate::design::from_core;
use crate::scenario::Sheets;

/// The systems that write the lab's rig state for this frame.
#[derive(SystemSet, Debug, Clone, PartialEq, Eq, Hash)]
pub struct RigDrive;

pub struct RigsPlugin;

impl Plugin for RigsPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, load_hero_models).add_systems(
            Update,
            (call_rigs, drive_rigs)
                .chain()
                .in_set(RigDrive)
                .after(BreakitSet::Physics)
                .before(RigSet),
        );
    }
}

/// The rigs a scenario needs at the table, known before the test starts.
pub fn setup_for(scenario: &bs::Scenario) -> RigSetup {
    let mut setup = RigSetup::default();
    for e in &scenario.events {
        if let bs::Event::GroundShake { .. } = e {
            setup.shake = Some(ShakeSetup::default());
        }
    }
    setup
}

/// The rigs a set of load-model states needs. Used while a test runs, so a
/// rig is at the table whenever its load is on even if the sheet did not
/// announce it.
fn setup_from(state: &RigState, mut setup: RigSetup) -> RigSetup {
    if let Some(w) = state.wind.first() {
        setup.fan.get_or_insert(FanSetup {
            direction: from_core(w.direction),
        });
    }
    if state.table.is_some() {
        setup.shake.get_or_insert_with(ShakeSetup::default);
    }
    if let Some(f) = state.flume.first() {
        setup.paddle.get_or_insert(PaddleSetup {
            direction: from_core(f.direction),
            rest: f.paddle_rest as f32,
            bed: f.bed as f32,
            ..Default::default()
        });
    }
    setup
}

/// The lab's picture of the load models' rig state at test time `time`.
pub fn to_lab(state: &RigState, time: f32) -> RigLive {
    RigLive {
        time,
        wind: state.wind.first().map(|w| WindDrive {
            fan_drive: w.fan_drive as f32,
            speed: w.speed as f32,
            direction: from_core(w.direction),
            gust: w.gust as f32,
        }),
        table: state.table.map(|t| TableDrive {
            displacement: from_core(t.displacement),
            velocity: from_core(t.velocity),
            acceleration: from_core(t.acceleration),
            drive: t.drive as f32,
        }),
        flume: state.flume.first().map(|f| FlumeDrive {
            bed: f.bed as f32,
            still_level: f.still_level as f32,
            depth: f.depth as f32,
            paddle_rest: f.paddle_rest as f32,
            paddle_position: f.paddle_position as f32,
            paddle_velocity: f.paddle_velocity as f32,
            wave_height: f.wave_height as f32,
            wave_period: f.wave_period as f32,
            direction: from_core(f.direction),
            current_speed: f.current_speed as f32,
        }),
        vehicles: state
            .vehicles
            .iter()
            .map(|v| VehicleDrive {
                name: v.name.clone(),
                forward: from_core(v.forward),
                speed: v.speed as f32,
                wheels: v
                    .wheels
                    .iter()
                    .map(|w| WheelDrive {
                        contact: from_core(w.contact),
                        bounce: w.bounce as f32,
                        lifted: w.lifted,
                    })
                    .collect(),
                braking: v.braking,
                fallen: v.fallen,
            })
            .collect(),
    }
}

/// Integrates the table's acceleration to its velocity and displacement
/// through a 0.25 Hz high-pass, as a shake table's controller does: a sine
/// that starts from rest would otherwise leave the table drifting away.
#[derive(Default, Clone, Copy)]
struct TableMotion {
    velocity: breakit_core::Vec3,
    displacement: breakit_core::Vec3,
    peak: f64,
    time: Option<f64>,
}

impl TableMotion {
    fn advance(&mut self, scenario: &bs::Scenario, to: f64) {
        let from = match self.time {
            Some(t) if to >= t && to - t < 1.0 => t,
            // A new test, or a jump: start from rest.
            _ => {
                *self = Self::default();
                to
            }
        };
        let steps = (((to - from) / (1.0 / 240.0)).ceil() as usize).max(1);
        let dt = (to - from) / steps as f64;
        for k in 0..steps {
            let a = Binding::ground_accel(scenario, from + (k as f64 + 0.5) * dt);
            let corner = std::f64::consts::TAU * 0.25;
            let restoring = self.velocity * (1.4 * corner) + self.displacement * corner * corner;
            self.velocity += (a - restoring) * dt;
            self.displacement += self.velocity * dt;
        }
        self.time = Some(to);
    }
}

/// The rig state of a running test.
///
/// The scenario run still plays its own events (convoys and a sine ground
/// shake) rather than a core `LoadSet`, so this builds the state from what
/// the run exposes. When `TestRun::rig_state` is there, this function is
/// that call.
fn rig_state(run: &TestRun, table: &mut TableMotion) -> RigState {
    let scenario = &run.scenario;
    let t = run.time();
    let mut state = RigState::default();

    let shakes = scenario
        .events
        .iter()
        .any(|e| matches!(e, bs::Event::GroundShake { .. }));
    if shakes {
        table.advance(scenario, t);
        let acceleration = Binding::ground_accel(scenario, t);
        table.peak = table.peak.max(acceleration.norm());
        state.table = Some(TableRig {
            displacement: table.displacement,
            velocity: table.velocity,
            acceleration,
            drive: if table.peak > 0.0 {
                acceleration.norm() / table.peak
            } else {
                0.0
            },
        });
    }

    for pose in run.vehicles() {
        let Some(bs::Event::Convoy {
            name, speed, axles, ..
        }) = scenario.events.get(pose.event)
        else {
            continue;
        };
        let wheels: Vec<WheelRig> = axles
            .iter()
            .map(|a| WheelRig {
                contact: pose.position - pose.dir * a.offset,
                force: a.load,
                on_structure: true,
                ..Default::default()
            })
            .collect();
        state.vehicles.push(VehicleRig {
            // Each vehicle of a convoy is its own model.
            name: format!("{name} {}", pose.index + 1),
            position: pose.position - pose.dir * (pose.length * 0.5),
            forward: pose.dir,
            speed: *speed,
            wheels,
            ..Default::default()
        });
    }
    state
}

/// Hero models in `assets/props/` replace the procedural rigs they are
/// named for (see `docs/rigs.md`).
fn load_hero_models(mut commands: Commands, assets: Res<AssetServer>) {
    commands.insert_resource(RigModels::from_folder(&assets, "props"));
}

fn call_rigs(sheets: Res<Sheets>, sim: Option<Res<Simulation>>, mut rigs: ResMut<LabRigs>) {
    // While a test runs, `drive_rigs` keeps the setup.
    if sim.is_some() {
        return;
    }
    let setup = setup_for(&sheets.current().def);
    if rigs.setup != setup {
        rigs.setup = setup;
    }
}

fn drive_rigs(
    time: Res<Time>,
    sheets: Res<Sheets>,
    sim: Option<Res<Simulation>>,
    mut rigs: ResMut<LabRigs>,
    mut table: Local<TableMotion>,
    mut clock: Local<f32>,
) {
    let Some(sim) = sim else {
        // Nothing runs: let the rigs wind down on their own clock.
        *clock += time.delta_secs();
        let idle = RigLive {
            time: *clock,
            ..default()
        };
        if rigs.live != idle {
            rigs.live = idle;
        }
        return;
    };
    let run = sim.run();
    let state = rig_state(run, &mut table);
    *clock = run.time() as f32;
    let setup = setup_from(&state, setup_for(&sheets.current().def));
    if rigs.setup != setup {
        rigs.setup = setup;
    }
    rigs.live = to_lab(&state, *clock);
}

#[cfg(test)]
mod tests {
    use super::*;
    use breakit_core::loads::water::FlumeRig;
    use breakit_core::loads::wind::WindRig;
    use breakit_core::Vec3 as V;

    #[test]
    fn the_quake_sheet_calls_the_shake_rig_and_the_bridge_does_not() {
        for s in bs::builtin() {
            let shakes = s
                .events
                .iter()
                .any(|e| matches!(e, bs::Event::GroundShake { .. }));
            assert_eq!(setup_for(&s).shake.is_some(), shakes, "{}", s.id);
            assert!(setup_for(&s).fan.is_none());
        }
    }

    #[test]
    fn a_load_that_is_on_brings_its_rig() {
        let state = RigState {
            wind: vec![WindRig {
                fan_drive: 0.5,
                speed: 12.0,
                direction: V::new(0.0, 0.0, 1.0),
                gust: 1.0,
            }],
            flume: vec![FlumeRig {
                bed: -3.2,
                paddle_rest: -13.6,
                direction: V::x(),
                ..Default::default()
            }],
            ..Default::default()
        };
        let setup = setup_from(&state, RigSetup::default());
        assert_eq!(setup.fan.unwrap().direction, Vec3::Z);
        let paddle = setup.paddle.unwrap();
        assert_eq!((paddle.rest, paddle.bed), (-13.6, -3.2));
        assert!(setup.shake.is_none());

        let live = to_lab(&state, 2.0);
        assert_eq!(live.wind.unwrap().speed, 12.0);
        assert_eq!(live.flume.unwrap().paddle_rest, -13.6);
        assert!(live.table.is_none() && live.vehicles.is_empty());
    }

    #[test]
    fn the_integrated_table_motion_matches_the_sine_it_is_driven_with() {
        let scenario = bs::builtin()
            .into_iter()
            .find(|s| {
                s.events
                    .iter()
                    .any(|e| matches!(e, bs::Event::GroundShake { .. }))
            })
            .expect("a built-in scenario shakes");
        let Some(bs::Event::GroundShake {
            start,
            duration,
            peak,
            frequency,
            ..
        }) = scenario
            .events
            .iter()
            .find(|e| matches!(e, bs::Event::GroundShake { .. }))
        else {
            unreachable!()
        };
        let mut m = TableMotion::default();
        let mut largest: f64 = 0.0;
        let mut t = *start;
        while t < start + duration {
            m.advance(&scenario, t);
            largest = largest.max(m.displacement.norm());
            t += 1.0 / 60.0;
        }
        // A steady sine of acceleration `a` at `f` Hz moves by a / (2 pi f)^2.
        let expect = peak / (std::f64::consts::TAU * frequency).powi(2);
        assert!(
            largest > 0.7 * expect && largest < 1.6 * expect,
            "table moved {largest}, a steady sine would move {expect}"
        );
    }
}
