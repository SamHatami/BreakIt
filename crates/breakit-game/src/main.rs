//! BreakIt game shell.
//!
//! Pick a sheet in the test register, read the brief, build a structure
//! (live static check, beams coloured by utilisation), run the test in the
//! real-time solver, read the report, revise. See `flow.rs` for the loop
//! and `ui/` for the screens.

mod analysis;
mod build;
mod camera;
mod capture;
mod design;
mod flow;
mod history;
mod render;
mod scenario;
mod testrun;
mod ui;
mod visuals;

use bevy::prelude::*;

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
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "BreakIt".into(),
                ..default()
            }),
            ..default()
        }))
        .insert_resource(ClearColor(Color::srgb(0.1, 0.11, 0.13)))
        .insert_resource(design::Design::default())
        .add_plugins((
            flow::FlowPlugin,
            camera::OrbitCameraPlugin,
            build::BuildPlugin,
            analysis::AnalysisPlugin,
            testrun::TestRunPlugin,
            render::FrameRenderPlugin,
            visuals::VisualsPlugin,
            ui::GameUiPlugin,
            capture::CapturePlugin,
        ))
        .run();
}
