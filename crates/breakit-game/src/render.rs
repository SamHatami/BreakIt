//! Draws the scene around the members: nodes as spheres, supports and loads
//! as gizmos. The hall, the play table and the lights come from
//! `breakit-lab`. The beams themselves are drawn by
//! `breakit-render` (see `visuals.rs`). Reads only [`Design`] (topology) and
//! [`FrameState`] (positions).

use bevy::prelude::*;

use crate::analysis::FrameState;
use crate::build::BuildTool;
use crate::design::{Design, SupportKind};
use crate::Mode;

pub struct FrameRenderPlugin;

impl Plugin for FrameRenderPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, setup_assets).add_systems(
            PostUpdate,
            (
                rebuild_visuals,
                update_visuals,
                highlight_nodes,
                draw_supports_and_loads,
            )
                .chain()
                .before(TransformSystems::Propagate),
        );
    }
}

#[derive(Resource)]
struct FrameAssets {
    node_mesh: Handle<Mesh>,
    node_material: Handle<StandardMaterial>,
    /// The node the cursor is on.
    node_hover: Handle<StandardMaterial>,
    /// The chain's last node in build mode.
    node_selected: Handle<StandardMaterial>,
}

/// Radius of a node sphere, metres.
const NODE_R: f32 = 0.08;

#[derive(Component)]
struct NodeVisual(usize);

/// Revision the spawned visuals were built for.
#[derive(Default)]
struct BuiltFor(Option<(u64, usize, usize)>);

fn setup_assets(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands.insert_resource(FrameAssets {
        node_mesh: meshes.add(Sphere::new(1.0)),
        node_material: materials.add(StandardMaterial {
            base_color: Color::srgb(0.85, 0.85, 0.88),
            metallic: 0.6,
            ..default()
        }),
        node_hover: materials.add(StandardMaterial {
            base_color: Color::srgb(0.35, 0.85, 1.0),
            emissive: LinearRgba::rgb(0.3, 1.2, 1.8),
            ..default()
        }),
        node_selected: materials.add(StandardMaterial {
            base_color: Color::srgb(1.0, 0.54, 0.24),
            emissive: LinearRgba::rgb(2.5, 0.9, 0.25),
            ..default()
        }),
    });
}

/// Respawns node entities when the design's topology changes.
fn rebuild_visuals(
    mut commands: Commands,
    design: Res<Design>,
    assets: Res<FrameAssets>,
    nodes: Query<Entity, With<NodeVisual>>,
    mut built: Local<BuiltFor>,
) {
    let key = (design.revision, design.nodes.len(), design.beams.len());
    if built.0 == Some(key) {
        return;
    }
    built.0 = Some(key);
    for e in &nodes {
        commands.entity(e).despawn();
    }
    for (i, &p) in design.nodes.iter().enumerate() {
        commands.spawn((
            Mesh3d(assets.node_mesh.clone()),
            MeshMaterial3d(assets.node_material.clone()),
            Transform::from_translation(p).with_scale(Vec3::splat(NODE_R)),
            NodeVisual(i),
        ));
    }
}

/// Moves nodes to the current positions.
fn update_visuals(
    design: Res<Design>,
    state: Res<FrameState>,
    mut nodes: Query<(&NodeVisual, &mut Transform)>,
) {
    for (n, mut tf) in &mut nodes {
        if let Some(&rest) = design.nodes.get(n.0) {
            tf.translation = state.positions.get(n.0).copied().unwrap_or(rest);
        }
    }
}

/// Build mode: the chain's last node glows in the accent, the hovered node
/// in the hover colour, both a little larger.
fn highlight_nodes(
    assets: Res<FrameAssets>,
    tool: Res<BuildTool>,
    mode: Res<State<Mode>>,
    mut nodes: Query<(
        &NodeVisual,
        &mut MeshMaterial3d<StandardMaterial>,
        &mut Transform,
    )>,
) {
    let building = *mode.get() == Mode::Build;
    for (n, mut mat, mut tf) in &mut nodes {
        let (want, r) = match (tool.selected, tool.hovered) {
            (Some(s), _) if building && s == n.0 => (&assets.node_selected, 1.7),
            (_, Some(h)) if building && h == n.0 => (&assets.node_hover, 1.5),
            _ => (&assets.node_material, 1.0),
        };
        if mat.0 != *want {
            mat.0 = want.clone();
        }
        tf.scale = Vec3::splat(NODE_R * r);
    }
}

fn draw_supports_and_loads(mut gizmos: Gizmos, design: Res<Design>, state: Res<FrameState>) {
    let pos = |i: usize| state.positions.get(i).copied().unwrap_or(design.nodes[i]);
    for (i, s) in design.supports.iter().enumerate() {
        let p = pos(i);
        match s {
            Some(SupportKind::Fixed) => gizmos.cube(
                Transform::from_translation(p - Vec3::Y * 0.15).with_scale(Vec3::splat(0.3)),
                Color::srgb(0.9, 0.5, 0.2),
            ),
            Some(SupportKind::Pinned) => {
                // A small pyramid under the node.
                let base = p - Vec3::Y * 0.3;
                let c = Color::srgb(0.9, 0.5, 0.2);
                let corners = [
                    base + Vec3::new(0.2, 0.0, 0.2),
                    base + Vec3::new(-0.2, 0.0, 0.2),
                    base + Vec3::new(-0.2, 0.0, -0.2),
                    base + Vec3::new(0.2, 0.0, -0.2),
                ];
                for k in 0..4 {
                    gizmos.line(p, corners[k], c);
                    gizmos.line(corners[k], corners[(k + 1) % 4], c);
                }
            }
            None => {}
        }
    }
    for (i, f) in design.loads.iter().enumerate() {
        if *f == Vec3::ZERO {
            continue;
        }
        // 1 m of arrow per 20 kN.
        let len = (f.length() / 20_000.0).clamp(0.3, 3.0);
        let tip = pos(i);
        gizmos.arrow(tip - f.normalize() * len, tip, Color::srgb(1.0, 0.3, 0.3));
    }
}
