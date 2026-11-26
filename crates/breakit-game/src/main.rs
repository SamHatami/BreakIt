//! BreakIt game shell.
//!
//! Build a frame in build mode (live static check, beams coloured by
//! utilisation), then press Space to run it in the real-time solver and
//! watch it hold or break.

mod analysis;
mod build;
mod camera;
mod capture;
mod design;
mod hud;
mod perf;
mod render;
mod test_scenario;
mod visuals;

use bevy::prelude::*;
use breakit_lab::{LabPlugin, PlayTable};

#[derive(States, Default, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Mode {
    #[default]
    Build,
    Simulate,
}

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "BreakIt".into(),
                ..default()
            }),
            ..default()
        }))
        .insert_resource(ClearColor(Color::srgb(0.1, 0.11, 0.13)))
        .add_plugins(LabPlugin)
        .insert_resource(design::Design::starter_bridge())
        .init_state::<Mode>()
        .add_plugins((
            camera::OrbitCameraPlugin,
            build::BuildPlugin,
            analysis::AnalysisPlugin,
            render::FrameRenderPlugin,
            visuals::VisualsPlugin,
            hud::HudPlugin,
            capture::CapturePlugin,
            test_scenario::TestScenarioPlugin,
            perf::PerfPlugin,
        ))
        .add_systems(Update, (toggle_mode, cycle_table))
        .run();
}

fn toggle_mode(
    keys: Res<ButtonInput<KeyCode>>,
    mode: Res<State<Mode>>,
    check: Res<test_scenario::ScenarioCheck>,
    mut next: ResMut<NextState<Mode>>,
) {
    match mode.get() {
        // The HUD lists what blocks the test.
        Mode::Build if keys.just_pressed(KeyCode::Space) && !check.blocks_test() => {
            next.set(Mode::Simulate)
        }
        Mode::Simulate if keys.any_just_pressed([KeyCode::Space, KeyCode::KeyR]) => {
            next.set(Mode::Build)
        }
        _ => {}
    }
}

/// T: next built-in play table (ravine, flat, flume, shake table) until
/// scenarios choose their own.
fn cycle_table(
    keys: Res<ButtonInput<KeyCode>>,
    mode: Res<State<Mode>>,
    mut table: ResMut<PlayTable>,
    mut index: Local<usize>,
) {
    if *mode.get() == Mode::Build && keys.just_pressed(KeyCode::KeyT) {
        *index += 1;
        *table = PlayTable::builtin(*index);
    }
}
