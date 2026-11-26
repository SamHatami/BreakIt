//! Thousands of members on the real solver: a long 3D truss bridge that is
//! loaded until it comes down.
//!
//! Prints the frame rate and where the CPU time goes (solver, visual scene)
//! once a second, so the renderer's share is visible.
//!
//! Run: `cargo run --release -p breakit-render --example stress -- [bays]`
//! (16 members per bay; 120 bays is about 1,900 members, 320 about 5,100).
//! `--no-sim` animates the members without the solver, to time rendering alone.

mod common;

use bevy::diagnostic::{DiagnosticsStore, FrameTimeDiagnosticsPlugin};
use bevy::prelude::*;
use breakit_core::{DynamicSim, Material, SimConfig, Structure, Support, Vec3 as V3};
use breakit_mesh::{MaterialKind, MemberStyle, Profile};
use breakit_render::{BreakitRenderPlugin, MemberFrame, RenderStats, StructureVisual};
use common::{example_dt, Clock, ExamplePlugin};

#[derive(Resource)]
struct Sim {
    sim: Option<DynamicSim>,
    rest: Vec<Vec3>,
    deck: Vec<usize>,
    solver_ms: f32,
    acc: f32,
}

fn main() {
    let bays: usize = std::env::args()
        .nth(1)
        .and_then(|a| a.parse().ok())
        .unwrap_or(120);
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "BreakIt stress test".into(),
                resolution: (1600u32, 900u32).into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins((
            BreakitRenderPlugin,
            FrameTimeDiagnosticsPlugin::default(),
            ExamplePlugin {
                name: "stress",
                default_times: &[1.0, 6.0],
            },
        ))
        .init_resource::<Clock>()
        .insert_resource(Bays(bays))
        .add_systems(Startup, setup)
        .add_systems(Update, (step, report).chain())
        .run();
}

#[derive(Resource)]
struct Bays(usize);

fn setup(mut commands: Commands, bays: Res<Bays>) {
    let bays = bays.0;
    // Mixed materials so every look and break is on screen: steel chords,
    // timber posts, concrete deck beams, composite bracing.
    let tube = Profile::Tube { r: 0.07, t: 0.007 };
    let post = Profile::Rect { b: 0.14, h: 0.14 };
    let beam = Profile::Rect { b: 0.2, h: 0.3 };
    let brace = Profile::Rect { b: 0.06, h: 0.1 };
    let mut s = Structure::new();
    let mut styles = Vec::new();
    let (w, hgt, bay, y0) = (3.0, 3.0, 2.0, 6.0);
    let mut chords = vec![Vec::new(); 4];
    for i in 0..=bays {
        let x = i as f64 * bay;
        for (c, (y, z)) in [(0.0, 0.0), (0.0, w), (hgt, 0.0), (hgt, w)]
            .iter()
            .enumerate()
        {
            chords[c].push(s.add_node(V3::new(x, y0 + *y, *z)));
        }
    }
    let mut add = |s: &mut Structure, a: usize, b: usize, kind: MaterialKind, profile: Profile| {
        let material = match kind {
            MaterialKind::Steel => Material::steel(),
            MaterialKind::Timber => Material::timber(),
            MaterialKind::Concrete => Material::concrete(),
            MaterialKind::ReinforcedConcrete => Material::reinforced_concrete(),
            MaterialKind::Composite => Material::cfrp(),
        };
        s.add_member(a, b, material, profile.section());
        styles.push(MemberStyle { profile, kind });
    };
    for i in 0..=bays {
        let n: Vec<usize> = (0..4).map(|c| chords[c][i]).collect();
        add(&mut s, n[0], n[1], MaterialKind::ReinforcedConcrete, beam);
        add(&mut s, n[2], n[3], MaterialKind::Steel, tube);
        add(&mut s, n[0], n[2], MaterialKind::Timber, post);
        add(&mut s, n[1], n[3], MaterialKind::Timber, post);
        add(&mut s, n[0], n[3], MaterialKind::Composite, brace);
        if i < bays {
            for c in 0..4 {
                add(
                    &mut s,
                    chords[c][i],
                    chords[c][i + 1],
                    MaterialKind::Steel,
                    tube,
                );
            }
            add(
                &mut s,
                chords[0][i],
                chords[2][i + 1],
                MaterialKind::Steel,
                tube,
            );
            add(
                &mut s,
                chords[1][i],
                chords[3][i + 1],
                MaterialKind::Steel,
                tube,
            );
            add(
                &mut s,
                chords[0][i],
                chords[1][i + 1],
                MaterialKind::Composite,
                brace,
            );
            add(
                &mut s,
                chords[2][i],
                chords[3][i + 1],
                MaterialKind::Composite,
                brace,
            );
        }
    }
    // A pier every 12 bays.
    for i in (0..=bays).step_by(12).chain([bays]) {
        for c in [0, 1] {
            s.add_support(Support::pinned(chords[c][i]));
        }
    }
    info!("{} nodes, {} members", s.nodes.len(), s.members.len());

    let rest: Vec<Vec3> = s
        .nodes
        .iter()
        .map(|p| Vec3::new(p.x as f32, p.y as f32, p.z as f32))
        .collect();
    let sim = (!std::env::args().any(|a| a == "--no-sim"))
        .then(|| DynamicSim::new(&s, SimConfig::default()));
    let deck: Vec<usize> = chords[0].iter().chain(&chords[1]).copied().collect();
    commands.insert_resource(StructureVisual::new(&s, &styles));
    commands.insert_resource(Sim {
        sim,
        rest,
        deck,
        solver_ms: 0.0,
        acc: 0.0,
    });

    let len = bays as f32 * bay as f32;
    let (eye, target) = match common::arg("--view").as_deref() {
        Some("close") => (
            Vec3::new(len * 0.5 - 9.0, y0 as f32 + 4.0, 13.0),
            Vec3::new(len * 0.5 + 3.0, y0 as f32, 1.5),
        ),
        _ => (
            Vec3::new(-14.0, y0 as f32 + 9.0, 24.0),
            Vec3::new(len * 0.22, y0 as f32 - 1.0, 0.0),
        ),
    };
    commands.spawn((
        Camera3d::default(),
        Transform::from_translation(eye).looking_at(target, Vec3::Y),
    ));
}

fn step(
    time: Res<Time>,
    mut clock: ResMut<Clock>,
    capture: Option<Res<common::Capture>>,
    mut state: ResMut<Sim>,
    mut visual: ResMut<StructureVisual>,
    mut frame: ResMut<MemberFrame>,
) {
    let dt = example_dt(&time, capture.is_some());
    clock.0 += dt;
    let t = clock.0;
    let state = &mut *state;
    match &mut state.sim {
        Some(sim) => {
            // A load on the middle spans that grows until something gives.
            let n = state.deck.len() / 2;
            let force = -(t - 1.0).max(0.0) as f64 * 60_000.0;
            for half in 0..2 {
                for k in n * 2 / 5..n * 3 / 5 {
                    sim.set_force(state.deck[half * n + k], V3::new(0.0, force, 0.0));
                }
            }
            let start = std::time::Instant::now();
            state.acc = (state.acc + dt).min(2.0 / 60.0);
            while state.acc >= 1.0 / 60.0 {
                sim.step(1.0 / 60.0);
                state.acc -= 1.0 / 60.0;
            }
            state.solver_ms = start.elapsed().as_secs_f32() * 1e3;
            frame.0 = visual.frame_from_sim(sim);
        }
        None => {
            // No solver: a travelling wave, every member bending every frame.
            let positions: Vec<Vec3> = state
                .rest
                .iter()
                .map(|p| *p + Vec3::Y * 0.4 * (p.x * 0.35 - t * 2.0).sin())
                .collect();
            let mut f = visual.frame_from_positions(&positions, &[]);
            f.node_rotations = state
                .rest
                .iter()
                .map(|p| Quat::from_rotation_z(0.14 * (p.x * 0.35 - t * 2.0).cos()))
                .collect();
            for (i, m) in f.members.iter_mut().enumerate() {
                m.utilization = 0.5 + 0.5 * (i as f32 * 0.37 + t).sin();
            }
            frame.0 = f;
        }
    }
}

fn report(
    time: Res<Time>,
    diagnostics: Res<DiagnosticsStore>,
    stats: Res<RenderStats>,
    state: Res<Sim>,
    mut last: Local<f32>,
) {
    if time.elapsed_secs() - *last < 1.0 {
        return;
    }
    *last = time.elapsed_secs();
    let get = |path| {
        diagnostics
            .get(path)
            .and_then(|d| d.smoothed())
            .unwrap_or(0.0)
    };
    info!(
        "{:.0} fps ({:.2} ms) | solver {:.2} ms | visual scene {:.3} ms | {} slots, {} remnants, {} debris, {} particles",
        get(&FrameTimeDiagnosticsPlugin::FPS),
        get(&FrameTimeDiagnosticsPlugin::FRAME_TIME),
        state.solver_ms,
        stats.scene_update_ms,
        stats.slots,
        stats.remnants,
        stats.debris,
        stats.particles
    );
}
