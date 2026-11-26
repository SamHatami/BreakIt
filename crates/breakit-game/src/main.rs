//! BreakIt game shell.
//!
//! Pick a sheet in the test register, read the brief, build a structure
//! (live static check, beams coloured by utilisation), run the test in the
//! real-time solver, read the report, revise. See `flow.rs` for the loop
//! and `ui/` for the screens.

// Release builds on Windows open no console window; logs go to the file
// set up in `release.rs`.
#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

mod analysis;
mod build;
mod camera;
mod capture;
mod design;
mod flow;
mod guides;
mod history;
mod perf;
mod playground;
mod release;
mod render;
mod rigs;
mod scenario;
mod sound;
mod testrun;
mod ui;
mod visuals;
mod water;

use bevy::prelude::*;
use breakit_lab::{LabPlugin, PlayTable};

/// Which physics runs: the static check while building (and behind the
/// register and brief), the real-time solver in a test and its report.
/// Computed from [`flow::Screen`].
#[derive(Default, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Mode {
    #[default]
    Build,
    Simulate,
}

fn main() {
    App::new()
        .add_plugins(
            DefaultPlugins
                .set(WindowPlugin {
                    primary_window: Some(Window {
                        title: release::version_line(),
                        ..default()
                    }),
                    ..default()
                })
                .set(release::log_plugin()),
        )
        .insert_resource(ClearColor(Color::srgb(0.1, 0.11, 0.13)))
        .add_plugins(LabPlugin)
        .insert_resource(design::Design::default())
        .add_plugins((
            flow::FlowPlugin,
            camera::OrbitCameraPlugin,
            build::BuildPlugin,
            analysis::AnalysisPlugin,
            testrun::TestRunPlugin,
            render::FrameRenderPlugin,
            rigs::RigsPlugin,
            visuals::VisualsPlugin,
            ui::GameUiPlugin,
            capture::CapturePlugin,
            guides::GuidesPlugin,
            perf::PerfPlugin,
            playground::PlaygroundPlugin,
            sound::SoundPlugin,
            water::WaterPlugin,
        ))
        .add_systems(Update, cycle_table.run_if(in_state(flow::Screen::Build)))
        .run();
}

/// T: next built-in play table (ravine, flat, flume, shake table) until
/// scenarios choose their own.
fn cycle_table(
    keys: Res<ButtonInput<KeyCode>>,
    mut table: ResMut<PlayTable>,
    mut index: Local<usize>,
) {
    if keys.just_pressed(KeyCode::KeyT) {
        *index += 1;
        *table = water::play_table(*index);
    }
}
