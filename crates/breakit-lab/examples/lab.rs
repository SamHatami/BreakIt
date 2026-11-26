//! The lab on its own, with a stand-in model on the table.
//!
//! `cargo run --release -p breakit-lab --example lab` then:
//! 1-4 table insert, V next camera view, F4 quality preset, B depth of field
//! (bokeh, gaussian, off), [ ] f-number.
//!
//! `-- --quality low|medium|high|ultra` starts on that preset.
//! `-- --capture <dir>` saves one screenshot per view and table and exits;
//! `--views play,hall` keeps only those views, `--tables` adds the play view
//! of the other tables.
//! `-- --bench` prints frame times for each quality preset.
//! `-- --rig wind|quake|flume|trucks` calls that load rig to the table and
//! runs it (R steps through them); the views `rig`, `rigs` and `park` look
//! at it.

use std::path::PathBuf;

use bevy::app::AppExit;
use bevy::post_process::dof::DepthOfFieldMode;
use bevy::prelude::*;
use bevy::render::view::screenshot::{save_to_disk, Screenshot};
use bevy::window::PresentMode;
use breakit_lab::rigs::{
    FanSetup, FlumeDrive, PaddleSetup, RigLive, RigSetup, ShakeSetup, TableDrive, VehicleDrive,
    WheelDrive, WindDrive,
};
use breakit_lab::{
    LabCamera, LabPlugin, LabQuality, LabRigs, LabSettings, LabStats, PlayTable, RigSet,
};

/// (focus, distance, yaw, pitch, label)
const VIEWS: [(Vec3, f32, f32, f32, &str); 8] = [
    (Vec3::new(0.0, 0.8, 0.0), 13.0, 0.6, 0.32, "play"),
    (Vec3::new(0.0, 0.5, 0.0), 9.0, -0.5, 0.12, "low"),
    (Vec3::new(0.0, 0.0, 0.0), 30.0, 0.9, 0.45, "table"),
    (Vec3::new(0.0, 0.0, 0.0), 75.0, 0.5, 0.25, "wide"),
    (Vec3::new(-240.0, 43.0, -220.0), 729.0, 1.08, 0.11, "hall"),
    (Vec3::new(-3.0, 1.5, 0.0), 24.0, 0.75, 0.3, "rig"),
    (Vec3::new(-6.0, 0.0, -4.0), 52.0, 0.55, 0.38, "rigs"),
    (Vec3::new(-31.0, -10.0, -24.0), 40.0, 2.5, 0.5, "park"),
];

/// A load rig run by the example, with made-up signals.
#[derive(Resource, Clone, Copy, Debug, Default, PartialEq, Eq)]
enum Demo {
    #[default]
    None,
    Wind,
    Quake,
    Flume,
    Trucks,
}

impl Demo {
    const ALL: [Demo; 5] = [
        Demo::None,
        Demo::Wind,
        Demo::Quake,
        Demo::Flume,
        Demo::Trucks,
    ];

    fn parse(name: &str) -> Option<Self> {
        Self::ALL
            .into_iter()
            .find(|d| format!("{d:?}").eq_ignore_ascii_case(name))
    }

    /// The built-in table the rig belongs with.
    fn table(self) -> usize {
        match self {
            Demo::None | Demo::Trucks => 0,
            Demo::Wind => 1,
            Demo::Flume => 2,
            Demo::Quake => 3,
        }
    }
}

#[derive(Resource)]
struct Script {
    capture: Option<PathBuf>,
    /// (view, table) pairs to capture.
    shots: Vec<(usize, usize)>,
    bench: bool,
    view: usize,
    frame: u32,
    step: usize,
    times: Vec<f32>,
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let capture = args
        .iter()
        .position(|a| a == "--capture")
        .and_then(|i| args.get(i + 1))
        .map(PathBuf::from);
    let bench = args.iter().any(|a| a == "--bench");
    let value = |flag: &str| {
        args.iter()
            .position(|a| a == flag)
            .and_then(|i| args.get(i + 1))
    };
    let quality = value("--quality")
        .and_then(|q| LabQuality::parse(q))
        .unwrap_or_default();
    let views: Vec<usize> = match value("--views") {
        Some(list) => list
            .split(',')
            .filter_map(|name| VIEWS.iter().position(|v| v.4 == name))
            .collect(),
        None => (0..VIEWS.len()).collect(),
    };
    let demo = value("--rig")
        .and_then(|r| Demo::parse(r))
        .unwrap_or_default();
    let mut shots: Vec<(usize, usize)> = views.into_iter().map(|v| (v, demo.table())).collect();
    if (value("--views").is_none() && demo == Demo::None) || args.iter().any(|a| a == "--tables") {
        shots.extend((1..4).map(|t| (0, t)));
    }
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "BreakIt lab".into(),
                resolution: (1600, 900).into(),
                present_mode: PresentMode::AutoNoVsync,
                ..default()
            }),
            ..default()
        }))
        .insert_resource(LabSettings::default().with_quality(quality))
        .add_plugins(LabPlugin)
        .insert_resource(Script {
            capture,
            shots,
            bench,
            view: 0,
            frame: 0,
            step: 0,
            times: Vec::new(),
        })
        .insert_resource(demo)
        .add_systems(Startup, setup)
        .add_systems(
            Update,
            (
                (keys, script, place_camera).chain(),
                run_rigs.before(RigSet),
            ),
        )
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut mats: ResMut<Assets<StandardMaterial>>,
) {
    commands.spawn((
        Camera3d::default(),
        LabCamera {
            focus_distance: 12.0,
        },
    ));
    // A stand-in truss so there is something in focus.
    let steel = mats.add(StandardMaterial {
        base_color: Color::srgb(0.75, 0.3, 0.2),
        perceptual_roughness: 0.5,
        ..default()
    });
    let tube = meshes.add(Cylinder::new(0.05, 1.0));
    let mut beam = |a: Vec3, b: Vec3| {
        let len = a.distance(b);
        commands.spawn((
            Mesh3d(tube.clone()),
            MeshMaterial3d(steel.clone()),
            Transform::from_translation((a + b) * 0.5)
                .with_rotation(Quat::from_rotation_arc(Vec3::Y, (b - a) / len))
                .with_scale(Vec3::new(1.0, len, 1.0)),
        ));
    };
    for i in 0..6 {
        let x = i as f32 - 3.0;
        beam(Vec3::new(x, 0.0, 0.0), Vec3::new(x + 1.0, 0.0, 0.0));
        beam(Vec3::new(x, 0.0, 0.0), Vec3::new(x + 0.5, 1.0, 0.0));
        beam(Vec3::new(x + 0.5, 1.0, 0.0), Vec3::new(x + 1.0, 0.0, 0.0));
        if i < 5 {
            beam(Vec3::new(x + 0.5, 1.0, 0.0), Vec3::new(x + 1.5, 1.0, 0.0));
        }
    }
}

fn keys(
    input: Res<ButtonInput<KeyCode>>,
    mut script: ResMut<Script>,
    mut table: ResMut<PlayTable>,
    mut settings: ResMut<LabSettings>,
    mut demo: ResMut<Demo>,
) {
    if input.just_pressed(KeyCode::KeyR) {
        let i = Demo::ALL.iter().position(|d| *d == *demo).unwrap_or(0);
        *demo = Demo::ALL[(i + 1) % Demo::ALL.len()];
        *table = PlayTable::builtin(demo.table());
    }
    for (k, i) in [
        (KeyCode::Digit1, 0),
        (KeyCode::Digit2, 1),
        (KeyCode::Digit3, 2),
        (KeyCode::Digit4, 3),
    ] {
        if input.just_pressed(k) {
            *table = PlayTable::builtin(i);
        }
    }
    if input.just_pressed(KeyCode::KeyV) {
        script.view = (script.view + 1) % VIEWS.len();
    }
    if input.just_pressed(KeyCode::KeyB) {
        settings.look.dof = match settings.look.dof {
            Some(DepthOfFieldMode::Bokeh) => Some(DepthOfFieldMode::Gaussian),
            Some(DepthOfFieldMode::Gaussian) => None,
            None => Some(DepthOfFieldMode::Bokeh),
        };
    }
    if input.just_pressed(KeyCode::BracketLeft) {
        settings.f_stops = (settings.f_stops / 1.4).max(0.5);
    }
    if input.just_pressed(KeyCode::BracketRight) {
        settings.f_stops = (settings.f_stops * 1.4).min(64.0);
    }
}

/// Capture: the chosen views on the default table, then the play view on
/// each other table. Bench: about 120 frames per quality preset.
fn script(
    mut commands: Commands,
    mut script: ResMut<Script>,
    mut table: ResMut<PlayTable>,
    mut settings: ResMut<LabSettings>,
    stats: Res<LabStats>,
    time: Res<Time<Real>>,
    demo: Res<Demo>,
    mut exit: MessageWriter<AppExit>,
) {
    script.frame += 1;
    if script.bench {
        let modes = LabQuality::ALL;
        if script.frame == 1 && script.step == 0 {
            settings.set_quality(modes[0]);
        }
        if script.frame > 30 {
            let dt = time.delta_secs() * 1000.0;
            script.times.push(dt);
        }
        if script.times.len() >= 120
            || (script.frame > 30
                && time.elapsed_secs() > 0.0
                && script.times.iter().sum::<f32>() > 6000.0)
        {
            let mut t = std::mem::take(&mut script.times);
            t.sort_by(f32::total_cmp);
            let avg = t.iter().sum::<f32>() / t.len() as f32;
            println!(
                "bench quality={} frames={} avg {:.1} ms  p50 {:.1}  p95 {:.1}  | hall {} tris in {} meshes, table {} tris, {} lamps ({} shadowed)",
                settings.quality.name(),
                t.len(),
                avg,
                t[t.len() / 2],
                t[t.len() * 95 / 100],
                stats.hall_triangles,
                stats.hall_meshes,
                stats.table_triangles,
                stats.point_lights,
                stats.shadowed_lamps
            );
            script.step += 1;
            script.frame = 0;
            if script.step >= modes.len() {
                exit.write(AppExit::Success);
            } else {
                settings.set_quality(modes[script.step]);
            }
        }
        return;
    }
    let Some(dir) = script.capture.clone() else {
        return;
    };
    // A rig needs time to roll in and spin up.
    let wait = if *demo == Demo::None { 40 } else { 70 };
    let Some(&(view, tbl)) = script.shots.get(script.step) else {
        // Give the last screenshot time to be written.
        if script.frame > 20 {
            exit.write(AppExit::Success);
        }
        return;
    };
    if script.frame == 1 {
        script.view = view;
        if tbl != demo.table() || script.step == 0 {
            *table = PlayTable::builtin(tbl);
        }
    }
    if script.frame == wait + if script.step == 0 { 60 } else { 0 } {
        let rig = match *demo {
            Demo::None => String::new(),
            d => format!("-{d:?}").to_lowercase(),
        };
        let name = format!(
            "{}-{}{rig}.png",
            VIEWS[view].4,
            table.insert.name().replace(' ', "-")
        );
        commands
            .spawn(Screenshot::primary_window())
            .observe(save_to_disk(dir.join(name)));
        script.step += 1;
        script.frame = 0;
    }
}

fn place_camera(script: Res<Script>, mut cam: Single<(&mut Transform, &mut LabCamera)>) {
    let (focus, dist, yaw, pitch, _) = VIEWS[script.view];
    let rot = Quat::from_euler(EulerRot::YXZ, yaw, -pitch, 0.0);
    *cam.0 = Transform::from_translation(focus + rot * Vec3::new(0.0, 0.0, dist))
        .looking_at(focus, Vec3::Y);
    cam.1.focus_distance = dist;
}

/// Calls the demo's rig to the table and drives it with made-up signals,
/// the way the game drives it from the load models.
fn run_rigs(demo: Res<Demo>, time: Res<Time>, mut rigs: ResMut<LabRigs>) {
    let t = time.elapsed_secs();
    let mut setup = RigSetup::default();
    let mut live = RigLive {
        time: t,
        ..default()
    };
    match *demo {
        Demo::None => {}
        Demo::Wind => {
            setup.fan = Some(FanSetup { direction: Vec3::X });
            let ramp = ((t - 4.0) / 3.0).clamp(0.0, 1.0);
            let gust = 4.0 * (t * 0.9).sin() * (t * 0.37).cos() * ramp;
            let speed = 25.0 * ramp + gust;
            live.wind = Some(WindDrive {
                fan_drive: (speed / 32.0).clamp(0.0, 1.0),
                speed,
                direction: Vec3::X,
                gust,
            });
        }
        Demo::Quake => {
            setup.shake = Some(ShakeSetup::default());
            // A 1.5 Hz table motion with a wandering envelope, 0.4 g peak.
            let w = std::f32::consts::TAU * 1.5;
            let env = 0.5 + 0.5 * (t * 0.6).sin();
            let a = 3.9 * env;
            live.table = Some(TableDrive {
                displacement: Vec3::X * (-a / (w * w) * (w * t).sin()),
                velocity: Vec3::X * (-a / w * (w * t).cos()),
                acceleration: Vec3::X * (a * (w * t).sin()),
                drive: env,
            });
        }
        Demo::Flume => {
            let paddle = PaddleSetup::default();
            setup.paddle = Some(paddle);
            let w = std::f32::consts::TAU / 2.6;
            let ramp = ((t - 4.0) / 3.0).clamp(0.0, 1.0);
            let stroke = 0.28 * ramp;
            live.flume = Some(FlumeDrive {
                bed: paddle.bed,
                still_level: -1.0,
                depth: -1.0 - paddle.bed,
                paddle_rest: paddle.rest,
                paddle_position: stroke * (w * t).sin(),
                paddle_velocity: stroke * w * (w * t).cos(),
                wave_height: 0.5 * ramp,
                wave_period: 2.6,
                direction: Vec3::X,
                current_speed: 0.0,
            });
        }
        Demo::Trucks => {
            let convoy: [(&str, &[f32]); 4] = [
                ("Loaded tipper", &[0.0, 3.4, 4.7]),
                ("Coil hauler", &[0.0, 3.8, 5.1]),
                ("Mobile crane", &[0.0, 1.6, 4.4, 5.8]),
                ("Delivery truck", &[0.0, 3.2]),
            ];
            for (k, (name, axles)) in convoy.into_iter().enumerate() {
                let speed = 2.5;
                let front = (10.0 + speed * t - 8.5 * k as f32).rem_euclid(34.0) - 14.0;
                let braking = (4.0..7.0).contains(&front);
                live.vehicles.push(VehicleDrive {
                    name: name.to_string(),
                    forward: Vec3::X,
                    speed,
                    wheels: axles
                        .iter()
                        .enumerate()
                        .map(|(n, a)| WheelDrive {
                            contact: Vec3::new(front - a, 0.0, 0.0),
                            bounce: 0.03 * (t * 7.0 + n as f32 * 1.3 + k as f32).sin(),
                            lifted: false,
                        })
                        .collect(),
                    braking,
                    fallen: false,
                });
            }
        }
    }
    if rigs.setup != setup {
        rigs.setup = setup;
    }
    rigs.live = live;
}
