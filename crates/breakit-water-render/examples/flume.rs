//! The wave flume on the lab's play table, with a stand-in pier in it.
//!
//! `cargo run --release -p breakit-water-render --example flume` then:
//! 1 regular waves, 2 a solitary surge, 3 a flood rising and draining,
//! V next camera view, F4 quality preset (lab and water together),
//! F drops a pontoon raft with cargo and three timber logs into the flume.
//!
//! `-- --program waves|surge|flood` picks what the rig does at the start,
//! `-- --quality low|medium|high|ultra` the preset, `-- --dx 0.15` the cell
//! size of the simulation, `-- --floats` drops the raft and the logs when
//! the stills or the clip start.
//! `-- --capture <dir>` saves one still per view after `--at <seconds>` of
//! simulated time and exits. `-- --clip <dir> --view beach --frames 60
//! --fps 15` saves numbered frames of one view for a clip.

use std::path::PathBuf;

use bevy::app::AppExit;
use bevy::prelude::*;
use bevy::render::view::screenshot::{save_to_disk, Screenshot};
use bevy::window::PresentMode;
use breakit_core::loads::{Geometry, View};
use breakit_core::section::Shape;
use breakit_core::{
    DynamicSim, LoadCase, LoadKind, LoadSet, Material, Section, SimConfig, Structure,
    Vec3 as CoreVec3, Window as LoadWindow,
};
use breakit_lab::{Flume, LabCamera, LabPlugin, LabQuality, LabSettings, PlayTable};
use breakit_water::{Config, Drive, FlumeSetup, Program, WaterSim};
use breakit_water_render::{LiveWater, WaterLook, WaterQuality, WaterRenderPlugin, WaterRenderSet};

/// (focus, distance, yaw, pitch, label)
const VIEWS: [(Vec3, f32, f32, f32, &str); 6] = [
    (Vec3::new(8.0, -0.9, 0.0), 9.5, 0.55, 0.3, "beach"),
    (Vec3::new(-1.0, -0.4, 0.0), 8.5, -0.5, 0.24, "pier"),
    (Vec3::new(6.0, -0.6, 0.0), 7.0, 1.35, 0.2, "low"),
    (Vec3::new(0.0, -0.5, 0.0), 27.0, 0.4, 0.5, "table"),
    (Vec3::new(2.0, -0.8, 0.0), 14.0, 0.0, 1.15, "top"),
    (Vec3::new(-7.0, -1.0, 0.0), 10.0, -0.5, 0.6, "raft"),
];

/// Pile positions (x, z) of the stand-in pier, m, and their radius.
const PILES: [(f32, f32); 6] = [
    (-2.0, -1.0),
    (-2.0, 1.0),
    (0.0, -1.0),
    (0.0, 1.0),
    (2.0, -1.0),
    (2.0, 1.0),
];
const PILE_RADIUS: f32 = 0.11;

/// Frames before the first shot: the lab needs a few to bake its
/// reflection maps.
const WARM_UP: u32 = 50;

#[derive(Clone, Copy, PartialEq)]
enum Rig {
    Waves,
    Surge,
    Flood,
}

#[derive(Resource)]
struct Script {
    rig: Rig,
    /// Simulated time the current rig program started at, s.
    since: f64,
    view: usize,
    capture: Option<PathBuf>,
    clip: Option<PathBuf>,
    /// Simulated seconds before the first still or clip frame.
    at: f64,
    frames: u32,
    fps: f64,
    frame: u32,
    shot: u32,
    still_level: f64,
    depth: f64,
    /// Drop the floats when the stills or the clip start.
    floats: bool,
}

/// Things floating in the flume: a pontoon raft and timber logs, simulated
/// by the structural solver with the water as their only load.
#[derive(Resource)]
struct Floats {
    structure: Structure,
    sim: DynamicSim,
    loads: LoadSet,
    geometry: Geometry,
    bars: Vec<Entity>,
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let value = |flag: &str| {
        args.iter()
            .position(|a| a == flag)
            .and_then(|i| args.get(i + 1))
    };
    let number =
        |flag: &str, default: f64| value(flag).and_then(|v| v.parse().ok()).unwrap_or(default);
    let quality = value("--quality")
        .and_then(|q| LabQuality::parse(q))
        .unwrap_or_default();
    let rig = match value("--program").map(String::as_str) {
        Some("surge") => Rig::Surge,
        Some("flood") => Rig::Flood,
        _ => Rig::Waves,
    };
    let view = value("--view")
        .and_then(|name| VIEWS.iter().position(|v| v.4 == name))
        .unwrap_or(0);
    let (width, height) = value("--size")
        .and_then(|s| s.split_once('x'))
        .and_then(|(w, h)| Some((w.parse().ok()?, h.parse().ok()?)))
        .unwrap_or((1600, 900));

    // The flume table with a beach, its water simulated instead of still.
    let flume = Flume {
        beach: Some((5.0, 0.4)),
        ..Flume::default()
    };
    let table = PlayTable::new(flume).without_still_water();
    let basin = table.water().expect("the flume holds water");
    let setup = FlumeSetup {
        x: (basin.min.x as f64, basin.max.x as f64),
        z: (basin.min.y as f64, basin.max.y as f64),
        dx: number("--dx", 0.2),
        bed: basin.bed as f64,
        still_level: basin.level as f64,
        beach: basin.beach.map(|(toe, slope)| (toe as f64, slope as f64)),
        outlet: false,
    };
    let sim = WaterSim::flume(setup, Config::default());

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "BreakIt wave flume".into(),
                resolution: (width, height).into(),
                present_mode: PresentMode::AutoNoVsync,
                ..default()
            }),
            ..default()
        }))
        .insert_resource(LabSettings::default().with_quality(quality))
        .add_plugins((LabPlugin, WaterRenderPlugin))
        .insert_resource(table)
        .insert_resource(water_look(quality))
        .insert_resource(LiveWater { sim, paddle: true })
        .insert_resource(Script {
            rig,
            since: 0.0,
            view,
            capture: value("--capture").map(PathBuf::from),
            clip: value("--clip").map(PathBuf::from),
            at: number("--at", 14.0),
            frames: number("--frames", 60.0) as u32,
            fps: number("--fps", 15.0),
            frame: 0,
            shot: 0,
            still_level: setup.still_level,
            depth: setup.still_level - setup.bed,
            floats: args.iter().any(|a| a == "--floats"),
        })
        .add_systems(Startup, setup_scene)
        .add_systems(
            Update,
            (
                keys,
                drop_floats,
                run_water,
                sync_floats,
                place_camera,
                shoot,
            )
                .chain()
                .before(WaterRenderSet),
        )
        .run();
}

fn water_look(q: LabQuality) -> WaterLook {
    match q {
        LabQuality::Low => WaterQuality::Low,
        LabQuality::Medium => WaterQuality::Medium,
        LabQuality::High => WaterQuality::High,
        LabQuality::Ultra => WaterQuality::Ultra,
    }
    .look()
}

fn setup_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut mats: ResMut<Assets<StandardMaterial>>,
) {
    commands.spawn((
        Camera3d::default(),
        LabCamera {
            focus_distance: 10.0,
        },
    ));
    // A stand-in pier: piles from the bed, a deck above the water.
    let steel = mats.add(StandardMaterial {
        base_color: Color::srgb(0.78, 0.33, 0.2),
        perceptual_roughness: 0.5,
        ..default()
    });
    let tube = meshes.add(Cylinder::new(1.0, 1.0));
    let mut bar = |a: Vec3, b: Vec3, r: f32| {
        let len = a.distance(b);
        commands.spawn((
            Mesh3d(tube.clone()),
            MeshMaterial3d(steel.clone()),
            Transform::from_translation((a + b) * 0.5)
                .with_rotation(Quat::from_rotation_arc(Vec3::Y, (b - a) / len))
                .with_scale(Vec3::new(r, len, r)),
        ));
    };
    let deck = 0.4;
    for (x, z) in PILES {
        bar(Vec3::new(x, -3.2, z), Vec3::new(x, deck, z), PILE_RADIUS);
    }
    for z in [-1.0, 1.0] {
        bar(Vec3::new(-2.6, deck, z), Vec3::new(2.6, deck, z), 0.08);
    }
    for x in [-2.0, 0.0, 2.0] {
        bar(Vec3::new(x, deck, -1.3), Vec3::new(x, deck, 1.3), 0.08);
    }
}

fn keys(
    keys: Res<ButtonInput<KeyCode>>,
    mut script: ResMut<Script>,
    settings: Res<LabSettings>,
    mut look: ResMut<WaterLook>,
    water: Res<LiveWater>,
) {
    for (key, rig) in [
        (KeyCode::Digit1, Rig::Waves),
        (KeyCode::Digit2, Rig::Surge),
        (KeyCode::Digit3, Rig::Flood),
    ] {
        if keys.just_pressed(key) {
            script.rig = rig;
            script.since = water.sim.solver.time();
        }
    }
    if keys.just_pressed(KeyCode::KeyV) {
        script.view = (script.view + 1) % VIEWS.len();
    }
    // The lab steps its own preset on F4; follow it.
    let want = water_look(settings.quality);
    if *look != want {
        *look = want;
    }
}

/// What the rig asks of the water at time `t` since its program started.
fn drive(script: &Script, t: f64) -> Drive {
    let program = match script.rig {
        Rig::Waves => Program::Regular {
            height: 0.7,
            period: 3.6,
            ramp: 5.0,
        },
        Rig::Surge => Program::Solitary {
            height: 1.1,
            at: 3.0,
        },
        Rig::Flood => Program::Still,
    };
    let (x, v) = program.motion(t, script.depth, 9.81);
    // The flood: up 1.3 m over 8 s, hold, and back down.
    let rise = if script.rig == Rig::Flood {
        let up = (t / 8.0).clamp(0.0, 1.0);
        let down = ((t - 16.0) / 8.0).clamp(0.0, 1.0);
        1.3 * (up * up * (3.0 - 2.0 * up) - down * down * (3.0 - 2.0 * down))
    } else {
        0.0
    };
    Drive {
        paddle_velocity: v,
        paddle_position: x,
        current: 0.0,
        still_level: script.still_level + rise,
    }
}

/// F, or the start of a scripted run with `--floats`: the raft is set on
/// the water and the logs are let go above it.
fn drop_floats(
    mut commands: Commands,
    keys: Res<ButtonInput<KeyCode>>,
    script: Res<Script>,
    floats: Option<Res<Floats>>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut mats: ResMut<Assets<StandardMaterial>>,
) {
    // In a clip, just after the first frame is taken.
    let scripted = script.floats && script.frame == WARM_UP + 3 && floats.is_none();
    if !(keys.just_pressed(KeyCode::KeyF) || scripted) {
        return;
    }
    if let Some(old) = floats {
        for e in &old.bars {
            commands.entity(*e).despawn();
        }
    }
    let level = script.still_level;
    let (steel, timber) = (Material::steel(), Material::timber());
    let mut s = Structure::new();
    // The raft: two thin-walled 813 mm pontoons tied by three tubes.
    let pontoon = Section::tube(0.4065, 0.006);
    let tie = Section::tube(0.057, 0.005);
    let side = |s: &mut Structure, z: f64| {
        s.add_line(
            CoreVec3::new(-10.0, level, z),
            CoreVec3::new(-7.0, level, z),
            2,
            &steel,
            &pontoon,
        )
    };
    let (left, right) = (side(&mut s, -0.9), side(&mut s, 0.9));
    for (a, b) in left.iter().zip(&right) {
        s.add_member(*a, *b, steel.clone(), tie.clone());
    }
    // The logs, let go at different heights.
    for (x, z, turn, height) in [
        (-5.6, -1.4, 0.3, 1.2),
        (-4.6, 0.2, -0.5, 2.0),
        (-5.2, 1.5, 1.2, 2.8),
    ] {
        let half = CoreVec3::new(f64::cos(turn), 0.12, f64::sin(turn));
        let mid = CoreVec3::new(x, level + height, z);
        s.add_line(mid - half, mid + half, 2, &timber, &Section::rect(0.2, 0.2));
    }
    let mut case = breakit_core::loads::water::Water::current(level, CoreVec3::zeros());
    case.bed = level - script.depth;
    let mut loads = LoadSet::new(vec![LoadCase::new(
        "Water",
        LoadWindow::new(0.0, None, 0.0),
        LoadKind::Water(case),
    )]);
    loads.prepare(&s);
    let mut sim = DynamicSim::new(&s, SimConfig::default());
    sim.add_point_mass(left[1], 300.0);
    sim.add_point_mass(right[1], 300.0);

    let tube = meshes.add(Cylinder::new(1.0, 1.0));
    let block = meshes.add(Cuboid::new(1.0, 1.0, 1.0));
    let paint = mats.add(StandardMaterial {
        base_color: Color::srgb(0.85, 0.62, 0.12),
        perceptual_roughness: 0.45,
        ..default()
    });
    let wood = mats.add(StandardMaterial {
        base_color: Color::srgb(0.62, 0.45, 0.27),
        perceptual_roughness: 0.8,
        ..default()
    });
    let bars = s
        .members
        .iter()
        .map(|m| {
            let (mesh, mat) = match m.section.shape {
                Shape::Tube { .. } => (tube.clone(), paint.clone()),
                Shape::Rect { .. } => (block.clone(), wood.clone()),
            };
            commands
                .spawn((Mesh3d(mesh), MeshMaterial3d(mat), Transform::IDENTITY))
                .id()
        })
        .collect();
    commands.insert_resource(Floats {
        geometry: Geometry::new(&s),
        structure: s,
        sim,
        loads,
        bars,
    });
}

/// Moves the drawn members with the simulated ones.
fn sync_floats(floats: Option<Res<Floats>>, mut bars: Query<&mut Transform>) {
    let Some(f) = floats else { return };
    for (m, e) in f.structure.members.iter().zip(&f.bars) {
        let at = |n: usize| {
            let p = f.sim.position(n);
            Vec3::new(p.x as f32, p.y as f32, p.z as f32)
        };
        let (a, b) = (at(m.a), at(m.b));
        let len = a.distance(b).max(1e-3);
        let size = match m.section.shape {
            Shape::Tube { ro, .. } => Vec3::new(ro as f32, len, ro as f32),
            Shape::Rect { b, h } => Vec3::new(b as f32, len, h as f32),
        };
        if let Ok(mut t) = bars.get_mut(*e) {
            *t = Transform::from_translation((a + b) * 0.5)
                .with_rotation(Quat::from_rotation_arc(Vec3::Y, (b - a) / len))
                .with_scale(size);
        }
    }
}

fn run_water(
    time: Res<Time>,
    mut script: ResMut<Script>,
    mut water: ResMut<LiveWater>,
    mut floats: Option<ResMut<Floats>>,
) {
    // Captures run on simulated time: a frame takes as long as it takes.
    let scripted = script.capture.is_some() || script.clip.is_some();
    let mut span = if !scripted {
        (time.delta_secs() as f64).min(0.05)
    } else if script.frame == 0 {
        script.at
    } else if script.clip.is_some() {
        1.0 / script.fps
    } else {
        0.0
    };
    script.frame += 1;
    while span > 1e-9 {
        let dt = span.min(1.0 / 60.0);
        let t = water.sim.solver.time() - script.since;
        let drive = drive(&script, t);
        // What floats slows the water and splashes into it...
        match floats.as_deref() {
            Some(f) => water.sim.couple(
                &View {
                    structure: &f.structure,
                    sim: &f.sim,
                    geometry: &f.geometry,
                    time: t,
                },
                1.0,
                2.0,
            ),
            None => water.sim.solver.clear_blockage(),
        }
        // ...and so do the piles, by their wet length.
        let field = water.sim.field();
        for (x, z) in PILES {
            if let Some((surface, bed)) = field.column(x as f64, z as f64) {
                let wet = surface - bed;
                let drag_area = 0.5 * 1.0 * 2.0 * PILE_RADIUS as f64 * wet;
                water
                    .sim
                    .solver
                    .add_blockage(x as f64, z as f64, drag_area, 0.0, 0.0);
            }
        }
        water.sim.advance(dt, drive);
        if let Some(f) = floats.as_deref_mut() {
            if let LoadKind::Water(w) = &mut f.loads.cases[0].kind {
                w.field = Some(water.sim.field());
            }
            f.loads.step(&f.structure, &mut f.sim, dt);
        }
        span -= dt;
    }
}

fn place_camera(script: Res<Script>, mut cam: Single<(&mut Transform, &mut LabCamera)>) {
    let (focus, dist, yaw, pitch, _) = VIEWS[script.view];
    let rot = Quat::from_euler(EulerRot::YXZ, yaw, -pitch, 0.0);
    *cam.0 = Transform::from_translation(focus + rot * Vec3::new(0.0, 0.0, dist))
        .looking_at(focus, Vec3::Y);
    cam.1.focus_distance = dist;
}

fn shoot(mut commands: Commands, mut script: ResMut<Script>, mut exit: MessageWriter<AppExit>) {
    const BETWEEN: u32 = 6;
    if let Some(dir) = script.clip.clone() {
        if script.frame <= WARM_UP {
            return;
        }
        if script.shot >= script.frames {
            if script.frame > WARM_UP + script.frames + 20 {
                exit.write(AppExit::Success);
            }
            return;
        }
        let name = format!("{}-{:04}.png", VIEWS[script.view].4, script.shot);
        commands
            .spawn(Screenshot::primary_window())
            .observe(save_to_disk(dir.join(name)));
        script.shot += 1;
        return;
    }
    let Some(dir) = script.capture.clone() else {
        return;
    };
    if script.frame < WARM_UP {
        return;
    }
    let step = (script.frame - WARM_UP) / BETWEEN;
    let phase = (script.frame - WARM_UP) % BETWEEN;
    if step as usize >= VIEWS.len() {
        if phase == BETWEEN - 1 && step as usize > VIEWS.len() + 2 {
            exit.write(AppExit::Success);
        }
        return;
    }
    script.view = step as usize;
    if phase == BETWEEN - 1 {
        let name = format!("{}.png", VIEWS[script.view].4);
        commands
            .spawn(Screenshot::primary_window())
            .observe(save_to_disk(dir.join(name)));
    }
}
