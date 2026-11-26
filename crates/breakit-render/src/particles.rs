//! Draws the fracture particles (sparks, dust, splinters, fibres) as one
//! mesh rebuilt each frame: a small octahedron per particle, stretched along
//! its velocity so sparks streak and splinters tumble. A few thousand
//! particles is a few hundred kilobytes of vertices; nothing is allocated
//! while no particle is alive.

use bevy::asset::RenderAssetUsages;
use bevy::camera::visibility::NoFrustumCulling;
use bevy::mesh::{Indices, PrimitiveTopology};
use bevy::prelude::*;

use crate::StructureVisual;

#[derive(Component)]
pub(crate) struct ParticleCloud;

pub(crate) fn build(app: &mut App) {
    app.add_systems(Startup, setup);
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let mut mesh = Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::default(),
    );
    fill(&mut mesh, Vec::new(), Vec::new(), Vec::new(), Vec::new());
    commands.spawn((
        Mesh3d(meshes.add(mesh)),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::WHITE,
            unlit: true,
            alpha_mode: AlphaMode::Blend,
            ..default()
        })),
        Transform::IDENTITY,
        NoFrustumCulling,
        bevy::light::NotShadowCaster,
        ParticleCloud,
    ));
}

fn fill(
    mesh: &mut Mesh,
    positions: Vec<[f32; 3]>,
    normals: Vec<[f32; 3]>,
    colors: Vec<[f32; 4]>,
    indices: Vec<u32>,
) {
    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions);
    mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, normals);
    mesh.insert_attribute(Mesh::ATTRIBUTE_COLOR, colors);
    mesh.insert_indices(Indices::U32(indices));
}

const OCTA: [[usize; 3]; 8] = [
    [0, 2, 4],
    [2, 1, 4],
    [1, 3, 4],
    [3, 0, 4],
    [2, 0, 5],
    [1, 2, 5],
    [3, 1, 5],
    [0, 3, 5],
];

pub(crate) fn sync_particles(
    visual: Option<Res<StructureVisual>>,
    cloud: Query<&Mesh3d, With<ParticleCloud>>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut was_empty: Local<bool>,
) {
    let Ok(handle) = cloud.single() else { return };
    let particles = visual
        .as_ref()
        .map(|v| &v.scene.particles.particles[..])
        .unwrap_or(&[]);
    let alive = particles.iter().filter(|p| p.alive()).count();
    if alive == 0 && *was_empty {
        return;
    }
    *was_empty = alive == 0;
    let mut positions = Vec::with_capacity(alive * 6);
    let mut normals = Vec::with_capacity(alive * 6);
    let mut colors = Vec::with_capacity(alive * 6);
    let mut indices = Vec::with_capacity(alive * 24);
    for p in particles.iter().filter(|p| p.alive()) {
        let size = p.current_size();
        let speed = p.velocity.length();
        let axis = if speed > 1e-3 {
            p.velocity / speed
        } else {
            Vec3::Y
        };
        // Glowing particles streak with speed; the rest are a little oblong.
        let stretch = if p.glow > 0.0 {
            1.0 + (speed * 0.04 / size.max(1e-4)).min(12.0)
        } else {
            1.6
        };
        let side = axis.any_orthonormal_vector();
        let up = axis.cross(side);
        let base = positions.len() as u32;
        let c = p.color * (1.0 + p.glow);
        // Dust (big, slow, barely falling) is a translucent puff that thins
        // out as it ages; everything else is solid.
        let alpha = if p.size > 0.05 {
            0.3 * (1.0 - p.age / p.life).clamp(0.0, 1.0)
        } else {
            1.0
        };
        for v in [side, -side, up, -up, axis * stretch, -axis * stretch] {
            positions.push((p.position + v * size).to_array());
            normals.push(v.normalize().to_array());
            colors.push([c.x, c.y, c.z, alpha]);
        }
        for t in OCTA {
            indices.extend(t.map(|i| base + i as u32));
        }
    }
    if let Some(mut mesh) = meshes.get_mut(&handle.0) {
        fill(&mut mesh, positions, normals, colors, indices);
    }
}
