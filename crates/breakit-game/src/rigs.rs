//! Calls the lab's load rigs for the current sheet and drives them from
//! the running test: fan wall, shake actuators and seismograph, flume
//! paddle, model vehicles. The rigs themselves are `breakit_lab::rigs`;
//! what they do comes from `breakit_core::loads::RigState`.

use bevy::prelude::*;
use breakit_core::loads::RigState;
use breakit_lab::rigs::{
    FanSetup, FlumeDrive, PaddleSetup, RigLive, RigSetup, ShakeSetup, TableDrive, VehicleDrive,
    WheelDrive, WindDrive,
};
use breakit_lab::{LabRigs, RigModels, RigSet};
use breakit_scenario as bs;
use breakit_scenario::Binding;

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

/// The rigs a scenario needs at the table, known before the test starts:
/// one per load case the sheet's events turn into.
pub fn setup_for(scenario: &bs::Scenario) -> RigSetup {
    let loads = Binding::load_set(scenario, scenario.settle.max(0.0));
    setup_from(&loads.rig_state(0.0), RigSetup::default())
}

/// The rigs a set of load-model states needs.
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

/// Hero models in `assets/props/` replace the procedural rigs they are
/// named for (see `docs/rigs.md`).
fn load_hero_models(mut commands: Commands, assets: Res<AssetServer>) {
    commands.insert_resource(RigModels::from_folder(&assets, "props"));
}

fn call_rigs(
    sheets: Res<Sheets>,
    sim: Option<Res<Simulation>>,
    mut rigs: ResMut<LabRigs>,
    mut known: Local<Option<(String, RigSetup)>>,
) {
    // While a test runs, `drive_rigs` keeps the setup.
    if sim.is_some() {
        return;
    }
    let def = &sheets.current().def;
    if known.as_ref().is_none_or(|(id, _)| *id != def.id) {
        *known = Some((def.id.clone(), setup_for(def)));
    }
    let Some((_, setup)) = known.as_ref() else {
        return;
    };
    if rigs.setup != *setup {
        rigs.setup = setup.clone();
    }
}

fn drive_rigs(
    time: Res<Time>,
    sim: Option<Res<Simulation>>,
    mut rigs: ResMut<LabRigs>,
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
    let state = run.rig_state();
    *clock = run.time() as f32;
    let setup = setup_from(&state, RigSetup::default());
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
    fn every_sheet_calls_the_rigs_of_its_loads() {
        for s in bs::builtin() {
            let has = |f: fn(&bs::Event) -> bool| s.events.iter().any(f);
            let setup = setup_for(&s);
            assert_eq!(
                setup.shake.is_some(),
                has(|e| matches!(e, bs::Event::GroundShake { .. })),
                "{}",
                s.id
            );
            assert_eq!(
                setup.fan.is_some(),
                has(|e| matches!(e, bs::Event::Wind { .. })),
                "{}",
                s.id
            );
            assert_eq!(
                setup.paddle.is_some(),
                has(|e| matches!(e, bs::Event::Water { .. })),
                "{}",
                s.id
            );
        }
        let jetty = bs::builtin()
            .into_iter()
            .find(|s| s.id == "flood_jetty")
            .expect("the flume sheet is built in");
        let paddle = setup_for(&jetty).paddle.unwrap();
        assert_eq!((paddle.rest, paddle.bed), (-13.6, -3.2));
        assert_eq!(paddle.direction, Vec3::X);
    }

    #[test]
    fn a_load_that_is_on_brings_its_rig() {
        let state = RigState {
            wind: vec![WindRig {
                fan_drive: 0.5,
                speed: 12.0,
                direction: V::new(0.0, 0.0, 1.0),
                gust: 1.0,
                ..Default::default()
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
}
