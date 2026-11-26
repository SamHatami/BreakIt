//! Scripted capture for headless checks: set `BREAKIT_CAPTURE=<dir>` and the
//! game screenshots build mode, switches to simulate, screenshots again,
//! pulls the camera back for a shot of the lab and exits. Used by agents and CI to see the game without a person at the
//! screen, e.g. `xvfb-run cargo run -p breakit-game`.

use std::path::PathBuf;

use bevy::app::AppExit;
use bevy::prelude::*;
use bevy::render::view::screenshot::{save_to_disk, Screenshot};

use crate::camera::OrbitCamera;
use crate::Mode;

pub struct CapturePlugin;

impl Plugin for CapturePlugin {
    fn build(&self, app: &mut App) {
        if let Ok(dir) = std::env::var("BREAKIT_CAPTURE") {
            app.insert_resource(CaptureDir(PathBuf::from(dir)))
                .add_systems(Update, capture_script);
        }
    }
}

#[derive(Resource)]
struct CaptureDir(PathBuf);

/// Frames to wait before the build shot (shaders compile on first frames).
const BUILD_SHOT: u32 = 90;
/// Frames of simulation before the second shot.
const SIM_SHOT: u32 = BUILD_SHOT + 180;
/// Frames after that for the pulled-back shot (the camera moves first).
const LAB_SHOT: u32 = SIM_SHOT + 40;

fn capture_script(
    mut commands: Commands,
    dir: Res<CaptureDir>,
    mut frame: Local<u32>,
    mut next: ResMut<NextState<Mode>>,
    mut exit: MessageWriter<AppExit>,
    mut orbit: Single<&mut OrbitCamera>,
) {
    *frame += 1;
    let shot = |commands: &mut Commands, name: &str| {
        let path = dir.0.join(name);
        commands
            .spawn(Screenshot::primary_window())
            .observe(save_to_disk(path));
    };
    match *frame {
        BUILD_SHOT => {
            shot(&mut commands, "build.png");
            next.set(Mode::Simulate);
        }
        SIM_SHOT => {
            shot(&mut commands, "simulate.png");
            orbit.distance = 70.0;
            orbit.pitch = 0.22;
            orbit.yaw = 0.75;
        }
        LAB_SHOT => shot(&mut commands, "lab.png"),
        f if f == LAB_SHOT + 30 => {
            exit.write(AppExit::Success);
        }
        _ => {}
    }
}
