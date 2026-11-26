//! Draws the frame: beams as cylinders coloured by utilisation, nodes as
//! spheres, supports and loads as gizmos. Reads only [`Design`] (topology)
//! and [`FrameState`] (positions and member state).

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
    beam_mesh: Handle<Mesh>,
    node_mesh: Handle<Mesh>,
    node_material: Handle<StandardMaterial>,
}

#[derive(Component)]
struct BeamVisual {
    index: usize,
    material: Handle<StandardMaterial>,
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
        // Unit cylinder along Y; scaled per beam.
        beam_mesh: meshes.add(Cylinder::new(1.0, 1.0)),
        node_mesh: meshes.add(Sphere::new(1.0)),
        node_material: materials.add(StandardMaterial {
            base_color: Color::srgb(0.85, 0.85, 0.88),
            metallic: 0.6,
            ..default()
        }),
    });
}

/// Respawns beam and node entities when the design's topology changes.
fn rebuild_visuals(
    mut commands: Commands,
    design: Res<Design>,
    assets: Res<FrameAssets>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    beams: Query<(Entity, &BeamVisual)>,
    nodes: Query<Entity, With<NodeVisual>>,
    mut built: Local<BuiltFor>,
) {
    let key = (design.revision, design.nodes.len(), design.beams.len());
    if built.0 == Some(key) {
        return;
    }
    built.0 = Some(key);
    for (e, b) in &beams {
        materials.remove(&b.material);
        commands.entity(e).despawn();
    }
    for e in &nodes {
        commands.entity(e).despawn();
    }
    for i in 0..design.beams.len() {
        let material = materials.add(StandardMaterial {
            base_color: utilization_color(0.0),
            perceptual_roughness: 0.5,
            ..default()
        });
        commands.spawn((
            Mesh3d(assets.beam_mesh.clone()),
            MeshMaterial3d(material.clone()),
            Transform::default(),
            BeamVisual { index: i, material },
        ));
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

/// Moves beams and nodes to the current positions and recolours beams.
fn update_visuals(
    design: Res<Design>,
    state: Res<FrameState>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut beams: Query<(&BeamVisual, &mut Transform, &mut Visibility), Without<NodeVisual>>,
    mut nodes: Query<(&NodeVisual, &mut Transform), Without<BeamVisual>>,
) {
    let pos = |i: usize| state.positions.get(i).copied().unwrap_or(design.nodes[i]);
    for (b, mut tf, mut vis) in &mut beams {
        let Some(beam) = design.beams.get(b.index) else {
            continue;
        };
        let m = state.members.get(b.index).copied().unwrap_or_default();
        *vis = if m.broken {
            Visibility::Hidden
        } else {
            Visibility::Inherited
        };
        *tf = beam_transform(pos(beam.a), pos(beam.b), beam.kind.radius());
        if let Some(mut mat) = materials.get_mut(&b.material) {
            let c = utilization_color(m.utilization);
            if mat.base_color != c {
                mat.base_color = c;
            }
        }
    }
    for (n, mut tf) in &mut nodes {
        if n.0 < design.nodes.len() {
            tf.translation = pos(n.0);
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

/// Transform that stretches the unit Y cylinder between `a` and `b`.
pub fn beam_transform(a: Vec3, b: Vec3, radius: f32) -> Transform {
    let d = b - a;
    let len = d.length().max(1e-4);
    Transform {
        translation: (a + b) * 0.5,
        rotation: Quat::from_rotation_arc(Vec3::Y, d / len),
        scale: Vec3::new(radius, len, radius),
    }
}

/// Blue-grey when idle, through green and yellow to red at full capacity,
/// and magenta beyond it.
pub fn utilization_color(u: f32) -> Color {
    let u = u.max(0.0);
    if u > 1.0 {
        return Color::srgb(1.0, 0.0, 0.8);
    }
    let stops = [
        (0.0, Vec3::new(0.45, 0.55, 0.7)),
        (0.3, Vec3::new(0.2, 0.8, 0.3)),
        (0.7, Vec3::new(1.0, 0.85, 0.1)),
        (1.0, Vec3::new(1.0, 0.1, 0.05)),
    ];
    for w in stops.windows(2) {
        let (u0, c0) = w[0];
        let (u1, c1) = w[1];
        if u <= u1 {
            let t = (u - u0) / (u1 - u0);
            let c = c0.lerp(c1, t);
            return Color::srgb(c.x, c.y, c.z);
        }
    }
    unreachable!()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn beam_transform_spans_the_two_nodes() {
        let a = Vec3::new(1.0, 2.0, 3.0);
        let b = Vec3::new(4.0, 6.0, 3.0);
        let t = beam_transform(a, b, 0.05);
        // The unit cylinder's ends are at y = +-0.5.
        assert!(t.transform_point(Vec3::Y * 0.5).distance(b) < 1e-4);
        assert!(t.transform_point(-Vec3::Y * 0.5).distance(a) < 1e-4);
    }

    #[test]
    fn colours_change_with_utilization() {
        assert_ne!(utilization_color(0.0), utilization_color(1.0));
        assert_eq!(utilization_color(2.0), Color::srgb(1.0, 0.0, 0.8));
    }
}
