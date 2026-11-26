//! Scripted screenshots of the playground for headless checks: set
//! `BREAKIT_PLAYGROUND_CAPTURE=<dir>` and the game opens the playground
//! with the starter bridge over the ravine, plays wind and a truck, then
//! the shake table, then a dropped weight, saves a shot of each and exits.
//! E.g. `xvfb-run -a cargo run -p breakit-game`.

use std::path::PathBuf;

use bevy::app::AppExit;
use bevy::prelude::*;
use bevy::render::view::screenshot::{save_to_disk, Screenshot};
use breakit_lab::PlayTable;
use breakit_playground::{Probe, Trace};

use super::{Bench, Pick, PlayAction, TRACE_SECONDS};
use crate::analysis::Simulation;
use crate::camera::OrbitCamera;
use crate::flow::{Screen, UiAction};
use crate::scenario::Sheets;

pub fn plugin(app: &mut App) {
    if let Ok(dir) = std::env::var("BREAKIT_PLAYGROUND_CAPTURE") {
        let _ = std::fs::create_dir_all(&dir);
        app.insert_resource(Dir(PathBuf::from(dir)))
            .add_systems(Last, script);
    }
}

#[derive(Resource)]
struct Dir(PathBuf);

#[derive(Default)]
struct Script {
    frame: u32,
    since: u32,
    step: u32,
    t0: f64,
}

#[allow(clippy::too_many_arguments)]
fn script(
    mut commands: Commands,
    dir: Res<Dir>,
    mut s: Local<Script>,
    screen: Res<State<Screen>>,
    sheets: Res<Sheets>,
    mut table: ResMut<PlayTable>,
    mut bench: ResMut<Bench>,
    sim: Option<Res<Simulation>>,
    mut actions: MessageWriter<UiAction>,
    mut exit: MessageWriter<AppExit>,
    mut orbit: Single<&mut OrbitCamera>,
) {
    s.frame += 1;
    let waited = s.frame - s.since;
    let t = sim.as_ref().map(|x| x.time()).unwrap_or(0.0);
    let shot = |commands: &mut Commands, name: &str| {
        commands
            .spawn(Screenshot::primary_window())
            .observe(save_to_disk(dir.0.join(name)));
    };
    let next = |s: &mut Script| {
        s.step += 1;
        s.since = s.frame;
        s.t0 = t;
    };
    let mut play = |a| {
        actions.write(UiAction::Playground(a));
    };
    match s.step {
        0 if waited >= 60 => {
            if let Some(i) = sheets.all.iter().position(|x| x.playground) {
                actions.write(UiAction::SelectSheet(i));
            }
            next(&mut s);
        }
        1 if waited >= 10 => {
            actions.write(UiAction::OpenBrief);
            next(&mut s);
        }
        2 if waited >= 20 && *screen.get() == Screen::Build => {
            *table = PlayTable::builtin(0);
            orbit.focus = Vec3::new(0.0, 0.3, 0.0);
            orbit.distance = 13.0;
            orbit.yaw = 0.55;
            orbit.pitch = 0.32;
            actions.write(UiAction::RunTest);
            next(&mut s);
        }
        3 if waited >= 20 && *screen.get() == Screen::Playground => {
            bench.setup.wind.on = true;
            bench.setup.wind.speed = 40.0;
            bench.setup.traffic.mass = 30.0;
            bench.pick = Some(Pick::Member(3));
            bench.trace = Some(Trace::new(Probe::member(3)[0], TRACE_SECONDS));
            next(&mut s);
        }
        4 if t - s.t0 > 1.5 => {
            play(PlayAction::SendTruck);
            next(&mut s);
        }
        5 if t - s.t0 > 1.4 => {
            shot(&mut commands, "playground-1-wind-truck.png");
            next(&mut s);
        }
        6 if waited >= 5 => {
            bench.setup.wind.on = false;
            bench.setup.quake.on = true;
            bench.setup.quake.amplitude = 0.5;
            bench.setup.quake.frequency = 3.0;
            bench.deform = 2;
            bench.pick = Some(Pick::Node(3));
            bench.trace = Some(Trace::new(Probe::node(3)[1], TRACE_SECONDS));
            next(&mut s);
        }
        7 if t - s.t0 > 3.0 => {
            shot(&mut commands, "playground-2-quake.png");
            next(&mut s);
        }
        8 if waited >= 5 => {
            bench.setup.quake.on = false;
            bench.deform = 0;
            bench.setup.drop.mass = 10_000.0;
            bench.setup.drop.height = 3.0;
            play(PlayAction::DropWeight);
            next(&mut s);
        }
        9 if t - s.t0 > 0.9 => {
            shot(&mut commands, "playground-3-drop.png");
            next(&mut s);
        }
        10 if waited >= 5 => {
            play(PlayAction::NextTable);
            play(PlayAction::NextTable);
            next(&mut s);
        }
        11 if waited >= 10 => {
            bench.setup.water.on = true;
            bench.setup.water.level = 0.0;
            bench.setup.water.waves = breakit_playground::WaveKind::Regular;
            bench.setup.water.height = 1.0;
            bench.pick = None;
            bench.trace = None;
            orbit.distance = 22.0;
            next(&mut s);
        }
        12 if t - s.t0 > 6.0 => {
            shot(&mut commands, "playground-4-flume.png");
            next(&mut s);
        }
        13 if waited >= 20 => {
            exit.write(AppExit::Success);
        }
        _ => {}
    }
}
