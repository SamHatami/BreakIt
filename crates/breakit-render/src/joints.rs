//! Connection hardware and node hubs: plain rigid meshes, one entity per
//! part, placed each frame from the scene's joint poses.
//!
//! Every part of a (joint, profile, node size) combination shares one mesh
//! and one material, so Bevy batches all of a structure's bolts, plates and
//! pins into a handful of instanced draws. Per frame the CPU writes one
//! transform per part.

use std::collections::HashMap;

use bevy::prelude::*;

use breakit_mesh::joint::{joint_parts, node_hub, JointLook, PartLook, Side};
use breakit_mesh::MaterialKind;

use crate::{template_key, to_bevy_mesh, StructureVisual};

/// One rigid part of the joint with this index in `VisualScene::joints`.
#[derive(Component)]
pub(crate) struct JointPartVisual {
    joint: usize,
    side: Side,
    fuse: bool,
}

#[derive(Component)]
pub(crate) struct HubVisual(usize);

/// Surfaces of the hardware.
#[derive(Clone, Copy, PartialEq, Eq, Hash)]
enum Finish {
    /// Painted like the steelwork.
    Paint,
    /// Zinc plate and shoes on timber and composite.
    Galvanised,
    Fastener,
    Weld,
    Concrete,
    /// Cast steel node.
    Cast,
}

impl Finish {
    fn of(look: PartLook, member: MaterialKind) -> Self {
        match look {
            PartLook::Plate if member == MaterialKind::Steel => Finish::Paint,
            PartLook::Plate => Finish::Galvanised,
            PartLook::Fastener => Finish::Fastener,
            PartLook::Weld => Finish::Weld,
            PartLook::Concrete => Finish::Concrete,
        }
    }

    fn material(self) -> StandardMaterial {
        // Linear values, as the member shader writes its colours.
        let (c, metallic, perceptual_roughness) = match self {
            // The paint and the concrete of `member.wesl`.
            Finish::Paint => ([0.09, 0.16, 0.23], 0.1, 0.5),
            Finish::Concrete => ([0.5, 0.49, 0.46], 0.0, 0.95),
            Finish::Galvanised => ([0.33, 0.35, 0.37], 0.9, 0.45),
            Finish::Fastener => ([0.5, 0.5, 0.48], 1.0, 0.32),
            Finish::Weld => ([0.025, 0.023, 0.023], 0.8, 0.65),
            Finish::Cast => ([0.07, 0.08, 0.09], 0.35, 0.6),
        };
        let color = Color::linear_rgb(c[0], c[1], c[2]);
        StandardMaterial {
            base_color: color,
            metallic,
            perceptual_roughness,
            ..default()
        }
    }
}

type PartKey = (JointLook, [u32; 5], u32);
/// Side, surface, fuse and mesh of one part.
type PartMesh = (Side, PartLook, bool, Handle<Mesh>);

#[derive(Resource, Default)]
pub(crate) struct JointAssets {
    /// Meshes of each part of a joint, in `joint_parts` order.
    parts: HashMap<PartKey, Vec<PartMesh>>,
    hubs: [Option<Handle<Mesh>>; 2],
    materials: HashMap<Finish, Handle<StandardMaterial>>,
}

/// Spawns the hardware of a new structure and moves it every frame.
#[allow(clippy::type_complexity)]
pub(crate) fn sync_joints(
    mut commands: Commands,
    visual: Option<Res<StructureVisual>>,
    mut assets: ResMut<JointAssets>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut parts: Query<
        (Entity, &JointPartVisual, &mut Transform, &mut Visibility),
        Without<HubVisual>,
    >,
    mut hubs: Query<(Entity, &HubVisual, &mut Transform), Without<JointPartVisual>>,
    mut generation: Local<u64>,
) {
    let wanted = visual.as_ref().map(|v| v.generation).unwrap_or(0);
    if *generation != wanted {
        for (e, ..) in &parts {
            commands.entity(e).despawn();
        }
        for (e, ..) in &hubs {
            commands.entity(e).despawn();
        }
        *generation = wanted;
        let Some(visual) = &visual else {
            return;
        };
        let scene = &visual.scene;
        let assets = &mut *assets;
        let mut material = |finish: Finish| {
            assets
                .materials
                .entry(finish)
                .or_insert_with(|| materials.add(finish.material()))
                .clone()
        };
        for (i, j) in scene.joints.iter().enumerate() {
            let key = (
                j.look,
                template_key(&j.profile, false).profile,
                j.back.to_bits(),
            );
            let set = assets.parts.entry(key).or_insert_with(|| {
                joint_parts(j.look, &j.profile, j.back)
                    .iter()
                    .map(|p| (p.side, p.look, p.fuse, meshes.add(to_bevy_mesh(&p.mesh))))
                    .collect()
            });
            let (p, q) = scene
                .joint_states
                .get(i)
                .map(|s| s.pose.node)
                .unwrap_or_default();
            for (side, look, fuse, mesh) in set.iter() {
                commands.spawn((
                    Mesh3d(mesh.clone()),
                    MeshMaterial3d(material(Finish::of(*look, j.kind))),
                    Transform::from_translation(p).with_rotation(q),
                    JointPartVisual {
                        joint: i,
                        side: *side,
                        fuse: *fuse,
                    },
                ));
            }
        }
        for (i, h) in scene.hubs.iter().enumerate() {
            let mesh = assets.hubs[h.concrete as usize]
                .get_or_insert_with(|| meshes.add(to_bevy_mesh(&node_hub(h.concrete))))
                .clone();
            let finish = if h.concrete {
                Finish::Concrete
            } else {
                Finish::Cast
            };
            commands.spawn((
                Mesh3d(mesh),
                MeshMaterial3d(material(finish)),
                Transform::from_scale(Vec3::splat(h.radius)),
                HubVisual(i),
            ));
        }
        return;
    }
    let Some(visual) = &visual else {
        return;
    };
    let scene = &visual.scene;
    for (_, part, mut tf, mut visibility) in &mut parts {
        let Some(state) = scene.joint_states.get(part.joint) else {
            continue;
        };
        let (p, q) = match part.side {
            Side::Node => state.pose.node,
            Side::Member => state.pose.member,
        };
        if tf.translation != p || tf.rotation != q {
            tf.translation = p;
            tf.rotation = q;
        }
        let want = if part.fuse && state.failed {
            Visibility::Hidden
        } else {
            Visibility::Inherited
        };
        if *visibility != want {
            *visibility = want;
        }
    }
    for (_, hub, mut tf) in &mut hubs {
        if let Some(&(p, q)) = scene.hub_poses.get(hub.0) {
            if tf.translation != p || tf.rotation != q {
                tf.translation = p;
                tf.rotation = q;
            }
        }
    }
}
