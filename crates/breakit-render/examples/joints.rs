//! Every connection, close up: pinned, bolted, welded, monolithic, dowelled
//! and the steel shoe, each on a section it suits.
//!
//! The members sag under a synthetic load so pins turn in their clevises and
//! end plates open; one second in, the pinned and the bolted connection at
//! the right-hand end let go.
//!
//! Run: `cargo run -p breakit-render --example joints`
//! (`--view a|b` for close-ups of the front and back rows).

mod common;

use bevy::prelude::*;
use breakit_core::BreakCause;
use breakit_mesh::pose::rest_frame;
use breakit_mesh::state::VisualMember;
use breakit_mesh::{
    JointLook, MaterialKind, MemberStyle, MemberVisualState, Profile, SceneConfig, VisualBreak,
    VisualFrame, VisualScene, VisualStructure,
};
use breakit_render::{BreakitRenderPlugin, MemberFrame, StructureVisual};
use common::{example_dt, Clock, ExamplePlugin};

const SPAN: f32 = 1.5;
const PITCH: f32 = 2.3;
const PER_ROW: usize = 4;
const BREAK_AT: f32 = 1.0;

struct Specimen {
    kind: MaterialKind,
    profile: Profile,
    joint: JointLook,
    /// The connection at end b fails at `BREAK_AT`.
    fails: bool,
}

fn specimens() -> Vec<Specimen> {
    use JointLook as J;
    use MaterialKind as M;
    let ipe = Profile::IBeam {
        h: 0.2,
        b: 0.1,
        tw: 0.0056,
        tf: 0.0085,
    };
    let s = |kind, profile, joint, fails| Specimen {
        kind,
        profile,
        joint,
        fails,
    };
    vec![
        s(
            M::Steel,
            Profile::Tube { r: 0.07, t: 0.006 },
            J::Pinned,
            true,
        ),
        s(M::Steel, ipe, J::Bolted, true),
        s(M::Steel, ipe, J::Welded, false),
        s(
            M::Steel,
            Profile::BoxSection {
                b: 0.12,
                h: 0.12,
                t: 0.006,
            },
            J::Welded,
            false,
        ),
        s(
            M::ReinforcedConcrete,
            Profile::Rect { b: 0.2, h: 0.26 },
            J::Monolithic,
            false,
        ),
        s(
            M::Timber,
            Profile::Rect { b: 0.12, h: 0.2 },
            J::Dowelled,
            false,
        ),
        s(
            M::Timber,
            Profile::Rect { b: 0.12, h: 0.2 },
            J::Pinned,
            false,
        ),
        s(
            M::Composite,
            Profile::Tube { r: 0.06, t: 0.006 },
            J::Sleeve,
            false,
        ),
    ]
}

fn node_pos(i: usize, end: usize) -> Vec3 {
    Vec3::new(
        (i % PER_ROW) as f32 * PITCH + end as f32 * SPAN,
        1.1,
        (i / PER_ROW) as f32 * -2.2,
    )
}

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "BreakIt joints".into(),
                resolution: (1600u32, 900u32).into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins((
            BreakitRenderPlugin,
            ExamplePlugin {
                name: "joints",
                default_times: &[0.9, 2.5],
            },
        ))
        .init_resource::<Clock>()
        .add_systems(Startup, setup)
        .add_systems(Update, drive)
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let list = specimens();
    let pedestal = meshes.add(Cuboid::new(0.3, 1.0, 0.4));
    let stone = materials.add(StandardMaterial {
        base_color: Color::srgb(0.3, 0.31, 0.33),
        perceptual_roughness: 0.9,
        ..default()
    });
    let mut members = Vec::new();
    for (i, sp) in list.iter().enumerate() {
        let (a, b) = (node_pos(i, 0), node_pos(i, 1));
        members.push(VisualMember {
            a: 2 * i,
            b: 2 * i + 1,
            style: MemberStyle {
                profile: sp.profile,
                kind: sp.kind,
            },
            rest_frame: rest_frame(a, b),
            rest_length: SPAN,
            seed: (0.137 + i as f32 * 0.618).fract(),
        });
        for p in [a, b] {
            let h = p.y - 0.2;
            commands.spawn((
                Mesh3d(pedestal.clone()),
                MeshMaterial3d(stone.clone()),
                Transform::from_translation(Vec3::new(p.x, 0.5 * h, p.z))
                    .with_scale(Vec3::new(1.0, h, 1.0)),
            ));
        }
    }
    let structure = VisualStructure::from_members(members, 2 * list.len())
        .with_joints(list.iter().map(|s| [s.joint; 2]).collect());
    commands.insert_resource(StructureVisual::from_scene(VisualScene::new(
        structure,
        SceneConfig::default(),
    )));
    let width = (PER_ROW - 1) as f32 * PITCH + SPAN;
    let (eye, at) = match common::arg("--view").as_deref() {
        // The right-hand ends of the front row, where two connections fail.
        Some("a") => (Vec3::new(3.1, 1.75, 2.1), Vec3::new(2.6, 1.05, 0.0)),
        Some("b") => (Vec3::new(3.1, 1.75, -0.1), Vec3::new(2.6, 1.05, -2.2)),
        Some("c") => (Vec3::new(7.7, 1.75, 2.1), Vec3::new(7.2, 1.05, 0.0)),
        Some("d") => (Vec3::new(7.7, 1.75, -0.1), Vec3::new(7.2, 1.05, -2.2)),
        _ => (
            Vec3::new(0.5 * width - 0.6, 4.2, 5.2),
            Vec3::new(0.5 * width, 0.9, -1.0),
        ),
    };
    commands.spawn((
        Camera3d::default(),
        Transform::from_translation(eye).looking_at(at, Vec3::Y),
    ));
}

fn drive(
    time: Res<Time>,
    mut clock: ResMut<Clock>,
    capture: Option<Res<common::Capture>>,
    mut frame: ResMut<MemberFrame>,
) {
    let before = clock.0;
    clock.0 += example_dt(&time, capture.is_some());
    let ramp = (clock.0 / (0.9 * BREAK_AT)).min(1.0);
    let breaking = before < BREAK_AT && clock.0 >= BREAK_AT;
    let mut f = VisualFrame::default();
    for (i, sp) in specimens().iter().enumerate() {
        let broken = sp.fails && clock.0 >= BREAK_AT;
        // The nodes turn as if the member sagged between them; node b drops
        // away once its connection has gone.
        let theta = 0.12 * ramp;
        let drop = if broken {
            Vec3::new(0.25, -0.12, 0.0) * ((clock.0 - BREAK_AT) * 3.0).min(1.0)
        } else {
            Vec3::ZERO
        };
        f.node_positions.push(node_pos(i, 0));
        f.node_positions.push(node_pos(i, 1) + drop);
        f.node_rotations.push(Quat::from_rotation_z(-theta));
        f.node_rotations.push(Quat::from_rotation_z(theta));
        // A bolted end plate yields before the member: all of the rotation
        // opens the joint.
        let hinge = if sp.joint == JointLook::Bolted {
            theta
        } else {
            0.0
        };
        f.members.push(MemberVisualState {
            utilization: 0.9 * ramp,
            hinge: [[0.0, 0.0], [-hinge, hinge]],
            broken,
            ..default()
        });
        if sp.fails && breaking {
            f.breaks.push(VisualBreak {
                member: i,
                position: 0.95,
                cause: BreakCause::Connection,
            });
        }
    }
    frame.0 = f;
}
