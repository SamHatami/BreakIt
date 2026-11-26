//! Every material, intact, damaged and broken, side by side.
//!
//! The members are driven by a synthetic solver frame, so this shows the
//! renderer alone: bending from node rotations, hinge creases, procedural
//! damage, and what a break looks like for each material. The right-hand
//! column breaks one second in.
//!
//! Run: `cargo run -p breakit-render --example gallery`

mod common;

use bevy::prelude::*;
use breakit_core::BreakCause;
use breakit_mesh::pose::rest_frame;
use breakit_mesh::state::VisualMember;
use breakit_mesh::{
    MaterialKind, MemberStyle, MemberVisualState, Profile, SceneConfig, VisualBreak, VisualFrame,
    VisualScene, VisualStructure,
};
use breakit_render::{BreakitRenderPlugin, MemberFrame, RenderSettings, StructureVisual};
use common::{example_dt, Clock, ExamplePlugin};

const SPAN: f32 = 3.2;
const COLS: usize = 3;
const BREAK_AT: f32 = 1.0;

fn rows() -> Vec<(MaterialKind, Profile)> {
    vec![
        (
            MaterialKind::Steel,
            Profile::IBeam {
                h: 0.3,
                b: 0.16,
                tw: 0.012,
                tf: 0.018,
            },
        ),
        (MaterialKind::Steel, Profile::Tube { r: 0.11, t: 0.012 }),
        (MaterialKind::Timber, Profile::Rect { b: 0.18, h: 0.26 }),
        (MaterialKind::Concrete, Profile::Rect { b: 0.28, h: 0.36 }),
        (
            MaterialKind::ReinforcedConcrete,
            Profile::Rect { b: 0.28, h: 0.36 },
        ),
        (
            MaterialKind::Composite,
            Profile::BoxSection {
                b: 0.16,
                h: 0.24,
                t: 0.03,
            },
        ),
    ]
}

fn node_pos(row: usize, col: usize, end: usize) -> Vec3 {
    Vec3::new(
        col as f32 * (SPAN + 1.3) + end as f32 * SPAN,
        1.6,
        row as f32 * 1.5,
    )
}

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "BreakIt material gallery".into(),
                resolution: (1600u32, 900u32).into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins((
            BreakitRenderPlugin,
            ExamplePlugin {
                name: "gallery",
                default_times: &[0.9, 1.12, 4.0],
            },
        ))
        .init_resource::<Clock>()
        .add_systems(Startup, setup)
        .add_systems(Update, (drive, toggle_stress))
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let rows = rows();
    let mut members = Vec::new();
    let pedestal = meshes.add(Cuboid::new(0.5, 1.0, 0.7));
    let stone = materials.add(StandardMaterial {
        base_color: Color::srgb(0.3, 0.31, 0.33),
        perceptual_roughness: 0.9,
        ..default()
    });
    for (r, (kind, profile)) in rows.iter().enumerate() {
        for c in 0..COLS {
            let i = members.len();
            let (a, b) = (node_pos(r, c, 0), node_pos(r, c, 1));
            members.push(VisualMember {
                a: 2 * i,
                b: 2 * i + 1,
                style: MemberStyle {
                    profile: *profile,
                    kind: *kind,
                },
                rest_frame: rest_frame(a, b),
                rest_length: SPAN,
                seed: (0.137 + i as f32 * 0.618).fract(),
            });
            for p in [a, b] {
                let h = p.y - profile.half_depth();
                commands.spawn((
                    Mesh3d(pedestal.clone()),
                    MeshMaterial3d(stone.clone()),
                    Transform::from_translation(Vec3::new(p.x, 0.5 * h, p.z))
                        .with_scale(Vec3::new(1.0, h, 1.0)),
                ));
            }
        }
    }
    let node_count = 2 * members.len();
    commands.insert_resource(StructureVisual::from_scene(VisualScene::new(
        VisualStructure {
            members,
            node_count,
            consumed_breaks: 0,
        },
        SceneConfig::default(),
    )));
    let centre = Vec3::new(
        (COLS as f32 * (SPAN + 1.3) - 1.3) * 0.5,
        1.2,
        (rows.len() as f32 - 1.0) * 0.75,
    );
    let eye = match common::arg("--view").as_deref() {
        Some("close") => (
            centre + Vec3::new(2.2, 2.6, 6.2),
            centre + Vec3::new(4.3, 0.1, -0.4),
        ),
        _ => (centre + Vec3::new(-1.5, 7.5, 10.5), centre),
    };
    commands.spawn((
        Camera3d::default(),
        Transform::from_translation(eye.0).looking_at(eye.1, Vec3::Y),
    ));
}

/// Column 0: unloaded. Column 1: heavily loaded and damaged. Column 2: the
/// same, then it breaks.
fn drive(
    time: Res<Time>,
    mut clock: ResMut<Clock>,
    capture: Option<Res<common::Capture>>,
    mut frame: ResMut<MemberFrame>,
    mut broke: Local<bool>,
) {
    let before = clock.0;
    clock.0 += example_dt(&time, capture.is_some());
    let rows = rows();
    let mut f = VisualFrame::default();
    let ramp = (clock.0 / (0.9 * BREAK_AT)).min(1.0);
    for (r, (kind, _)) in rows.iter().enumerate() {
        for c in 0..COLS {
            let i = r * COLS + c;
            let load = if c == 0 { 0.0 } else { ramp };
            let ductile = matches!(kind, MaterialKind::Steel | MaterialKind::ReinforcedConcrete);
            // Simply supported sag: the ends rotate towards each other.
            let theta = load * if ductile { 0.16 } else { 0.05 };
            // Once broken, the supports let the halves droop a little.
            let droop = if c == 2 && *broke { 0.12 } else { 0.0 };
            f.node_positions.push(node_pos(r, c, 0));
            f.node_positions.push(node_pos(r, c, 1));
            f.node_rotations.push(Quat::from_rotation_z(-theta - droop));
            f.node_rotations.push(Quat::from_rotation_z(theta + droop));
            let hinge = if *kind == MaterialKind::Steel {
                0.11 * load
            } else {
                0.0
            };
            f.members.push(MemberVisualState {
                utilization: 1.05 * load,
                // Hinge about local z at end a (and mirrored at b).
                hinge: [[0.0, 0.0], [-hinge, hinge]],
                hinge_damage: if hinge > 0.0 { [load, load] } else { [0.0; 2] },
                crack: [0.9 * load, 0.7 * load],
                crack_span: 0.55 * load,
                crush: if matches!(
                    kind,
                    MaterialKind::Concrete | MaterialKind::ReinforcedConcrete
                ) {
                    [0.0, 0.8 * load]
                } else {
                    [0.0; 2]
                },
                delamination: if *kind == MaterialKind::Composite {
                    0.8 * load
                } else {
                    0.0
                },
                yielded: ductile && load > 0.6,
                broken: c == 2 && clock.0 >= BREAK_AT,
            });
            if c == 2 && before < BREAK_AT && clock.0 >= BREAK_AT {
                f.breaks.push(VisualBreak {
                    member: i,
                    position: 0.42 + 0.03 * r as f32,
                    cause: BreakCause::Brittle,
                });
            }
        }
    }
    if clock.0 >= BREAK_AT {
        *broke = true;
    }
    frame.0 = f;
}

fn toggle_stress(keys: Res<ButtonInput<KeyCode>>, mut settings: ResMut<RenderSettings>) {
    if keys.just_pressed(KeyCode::KeyS) {
        settings.stress_view = !settings.stress_view;
    }
}
