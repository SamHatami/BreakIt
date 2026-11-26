//! Draws the scene around the members: nodes as spheres, supports and loads
//! as gizmos, ground and light. The beams themselves are drawn by
//! `breakit-render` (see `visuals.rs`). Reads only [`Design`] (topology) and
//! [`FrameState`] (positions).

use bevy::prelude::*;

use crate::analysis::FrameState;
use crate::design::{Design, SupportKind};

pub struct FrameRenderPlugin;

impl Plugin for FrameRenderPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, (setup_scene, setup_assets))
            .add_systems(
                PostUpdate,
                (rebuild_visuals, update_visuals, draw_supports_and_loads)
                    .chain()
                    .before(TransformSystems::Propagate),
            );
    }
}

#[derive(Resource)]
struct FrameAssets {
    node_mesh: Handle<Mesh>,
    node_material: Handle<StandardMaterial>,
}

#[derive(Component)]
struct NodeVisual(usize);

/// Revision the spawned visuals were built for.
#[derive(Default)]
struct BuiltFor(Option<(u64, usize, usize)>);

fn setup_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands.spawn((
        DirectionalLight {
            shadow_maps_enabled: true,
            illuminance: 8_000.0,
            ..default()
        },
        Transform::from_xyz(4.0, 10.0, 6.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
    commands.spawn((
        Mesh3d(meshes.add(Plane3d::new(Vec3::Y, Vec2::splat(30.0)))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.22, 0.24, 0.22),
            perceptual_roughness: 1.0,
            ..default()
        })),
        // Slightly below y = 0 so ground-level beams don't z-fight.
        Transform::from_xyz(0.0, -0.08, 0.0),
    ));
}

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
            Transform::from_translation(p).with_scale(Vec3::splat(0.08)),
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
