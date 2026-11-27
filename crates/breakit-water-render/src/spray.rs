//! Draws the spray as one mesh rebuilt each frame: a small octahedron per
//! droplet, stretched along its velocity so fast drops streak.

use bevy::asset::RenderAssetUsages;
use bevy::camera::visibility::NoFrustumCulling;
use bevy::mesh::{Indices, PrimitiveTopology};
use bevy::prelude::*;
use breakit_water::Droplet;

/// Marks the spray mesh entity.
#[derive(Component)]
pub(crate) struct SprayCloud;

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

pub(crate) fn spawn(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
) -> Entity {
    let mut mesh = Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::default(),
    );
    fill(&mut mesh, &[], 0);
    commands
        .spawn((
            Mesh3d(meshes.add(mesh)),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgba(0.92, 0.96, 0.98, 0.6),
                perceptual_roughness: 0.25,
                reflectance: 0.4,
                alpha_mode: AlphaMode::Blend,
                ..default()
            })),
            Transform::IDENTITY,
            NoFrustumCulling,
            bevy::light::NotShadowCaster,
            SprayCloud,
        ))
        .id()
}

/// Rebuilds the spray mesh from the first `limit` droplets.
pub(crate) fn fill(mesh: &mut Mesh, droplets: &[Droplet], limit: usize) {
    let count = droplets.len().min(limit);
    let mut positions = Vec::with_capacity(count * 6);
    let mut normals = Vec::with_capacity(count * 6);
    let mut indices = Vec::with_capacity(count * 24);
    for d in &droplets[..count] {
        let p = Vec3::from_array(d.position);
        let v = Vec3::from_array(d.velocity);
        let speed = v.length();
        let axis = if speed > 1e-3 { v / speed } else { Vec3::Y };
        // Shrinks as it ages, streaks with speed.
        let size = d.size * (1.0 - 0.6 * (d.age / d.life).clamp(0.0, 1.0));
        let stretch = 1.0 + (speed * 0.006 / size.max(1e-4)).min(2.5);
        let side = axis.any_orthonormal_vector();
        let up = axis.cross(side);
        let base = positions.len() as u32;
        for corner in [side, -side, up, -up, axis * stretch, -axis * stretch] {
            positions.push((p + corner * size).to_array());
            normals.push(corner.normalize().to_array());
        }
        for t in OCTA {
            indices.extend(t.map(|i| base + i as u32));
        }
    }
    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions);
    mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, normals);
    mesh.insert_indices(Indices::U32(indices));
}
