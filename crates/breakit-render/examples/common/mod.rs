//! Shared by the examples: scene dressing and scripted screenshots.
//!
//! `--capture <dir> --at <seconds,seconds,...>` saves `<name>-<n>.png` at
//! those times and exits after the last, for headless runs
//! (`xvfb-run -a cargo run -p breakit-render --example gallery -- --capture out`).
#![allow(dead_code)]

use std::path::PathBuf;

use bevy::app::AppExit;
use bevy::prelude::*;
use bevy::render::view::screenshot::{save_to_disk, Screenshot};

pub fn arg(name: &str) -> Option<String> {
    let args: Vec<String> = std::env::args().collect();
    args.iter()
        .position(|a| a == name)
        .and_then(|i| args.get(i + 1).cloned())
}

#[derive(Resource)]
pub struct Capture {
    dir: PathBuf,
    name: &'static str,
    times: Vec<f32>,
    next: usize,
    exit_at: Option<f32>,
}

pub struct ExamplePlugin {
    pub name: &'static str,
    pub default_times: &'static [f32],
}

impl Plugin for ExamplePlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(ClearColor(Color::srgb(0.55, 0.66, 0.78)))
            .add_systems(Startup, dress_scene);
        if let Some(dir) = arg("--capture") {
            let times = arg("--at")
                .map(|s| s.split(',').filter_map(|t| t.parse().ok()).collect())
                .unwrap_or_else(|| self.default_times.to_vec());
            app.insert_resource(Capture {
                dir: dir.into(),
                name: self.name,
                times,
                next: 0,
                exit_at: None,
            })
            .add_systems(Update, capture);
        }
    }
}

/// Simulation clock for the examples: fixed steps when capturing, so
/// screenshots do not depend on how fast the machine renders.
pub fn example_dt(time: &Time, capturing: bool) -> f32 {
    if capturing {
        1.0 / 60.0
    } else {
        time.delta_secs().min(0.05)
    }
}

#[derive(Resource, Default)]
pub struct Clock(pub f32);

fn capture(
    mut commands: Commands,
    clock: Option<Res<Clock>>,
    mut cap: ResMut<Capture>,
    mut exit: MessageWriter<AppExit>,
) {
    let now = clock.map(|c| c.0).unwrap_or(0.0);
    if let Some(t) = cap.exit_at {
        if now > t {
            exit.write(AppExit::Success);
        }
        return;
    }
    if cap.next < cap.times.len() && now >= cap.times[cap.next] {
        let path = cap.dir.join(format!("{}-{}.png", cap.name, cap.next + 1));
        commands
            .spawn(Screenshot::primary_window())
            .observe(save_to_disk(path));
        cap.next += 1;
        if cap.next == cap.times.len() {
            cap.exit_at = Some(now + 0.5);
        }
    }
}

fn dress_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands.spawn((
        DirectionalLight {
            shadow_maps_enabled: true,
            illuminance: 7_000.0,
            ..default()
        },
        Transform::from_xyz(6.0, 14.0, 9.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
    commands.insert_resource(GlobalAmbientLight {
        brightness: 110.0,
        ..default()
    });
    commands.spawn((
        Mesh3d(meshes.add(Plane3d::new(Vec3::Y, Vec2::splat(400.0)))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.36, 0.4, 0.34),
            perceptual_roughness: 1.0,
            ..default()
        })),
    ));
}
