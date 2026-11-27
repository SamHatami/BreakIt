//! The shake table's hardware: servo-hydraulic actuators on the table
//! surround that push the platform, an accelerometer on the platform, and
//! an instrument cart whose screen draws the table's acceleration.
//!
//! The platform itself is part of the play table (`table::ShakeTable`
//! builds it into `TableTop::platform`); it rides the table motion here.
//!
//! Actuator frame: the world's, origin in the middle of the platform.
//! Cart frame: origin at table-top level under the middle of the cart,
//! screen facing +z, reaching down to the hall floor.

use bevy::asset::RenderAssetUsages;
use bevy::light::NotShadowCaster;
use bevy::mesh::{Indices, PrimitiveTopology};
use bevy::prelude::*;

use super::hero::{ActuatorSpec, ShakeTableSpec};
use super::{
    in_parent, park, send, spawn_rig, Axis, LabRigs, Owner, Part, Rest, Rig, RigClock,
    RigMaterials, RigModel, RigModels, RigSlot, Role, ShakeSetup, Site,
};
use crate::palette::Mat;
use crate::scenery::Scenery;
use crate::{LabMaterials, MaterialMaker, ShakePlatform};

/// Push axis (towards the platform) and position across it for each
/// actuator: two along x from the -x side, two along z from the -z side.
pub fn actuator_axes() -> [(Vec3, f32); 4] {
    [
        (Vec3::X, -2.2),
        (Vec3::X, 2.2),
        (Vec3::Z, -2.2),
        (Vec3::Z, 2.2),
    ]
}

/// Height of the actuators' axis above the table top.
const AXIS_Y: f32 = 0.42;
/// From an actuator's base pin to the platform edge.
const REACH: f32 = 3.2;

/// Where the hero actuators go for a platform of edge `size`: one model
/// frame per actuator, base pin on the reaction bracket, rod end on the
/// platform's lug.
pub fn hero_fits(size: f32) -> Vec<Transform> {
    let p = size * 0.5;
    actuator_axes()
        .into_iter()
        .map(|(a, across)| {
            let base = a * -(p + REACH) + Vec3::Y * AXIS_Y + Vec3::Y.cross(a) * across;
            ActuatorSpec::fit(base, a, REACH)
        })
        .collect()
}

/// The procedural actuators for a platform of edge `size`. With `hero`
/// only what the hero actuators are mounted on: the reaction brackets and
/// the platform's lugs.
pub fn actuators(size: f32, hero: bool) -> RigModel {
    let p = size * 0.5;
    let mut fixed = Scenery::default();
    let mut riding = Scenery::default();
    let mut parts = Vec::new();
    let axis_y = AXIS_Y;
    for (k, (a, across)) in actuator_axes().into_iter().enumerate() {
        let side = Vec3::Y.cross(a);
        let at = |along: f32, up: f32, lateral: f32| {
            a * along + Vec3::Y * up + side * (across + lateral)
        };
        // Reaction bracket bolted to the surround, with gussets.
        fixed.m(Mat::Dark).cuboid(
            at(-(p + 3.3), 0.03, 0.0),
            Vec3::new(0.55, 0.03, 0.75),
            Quat::from_rotation_arc(Vec3::X, a),
        );
        fixed.m(Mat::MachineGrey).cuboid(
            at(-(p + 3.45), 0.5, 0.0),
            Vec3::new(0.16, 0.5, 0.6),
            Quat::from_rotation_arc(Vec3::X, a),
        );
        for s in [-0.45, 0.45] {
            fixed.m(Mat::MachineGrey).bar(
                at(-(p + 3.3), 0.85, s),
                at(-(p + 2.8), 0.2, s),
                0.08,
                0.3,
                Vec3::Y,
            );
        }
        let rot = Quat::from_rotation_arc(Vec3::X, a);
        if !hero {
            // Cylinder barrel, end caps, tie rods, trunnion saddle.
            let (b0, b1) = (at(-(p + 3.2), axis_y, 0.0), at(-(p + 0.95), axis_y, 0.0));
            fixed.m(Mat::Yellow).cylinder(b0, b1, 0.3, 20, true);
            for c in [b0, b1] {
                fixed.m(Mat::LightSteel).ring(c, a, 0.37, 0.37, 0.16, 20);
            }
            fixed.m(Mat::Dark).cuboid(
                at(-(p + 1.3), 0.08, 0.0),
                Vec3::new(0.2, 0.08, 0.42),
                Quat::from_rotation_arc(Vec3::X, a),
            );
            // Servo valve and its hoses, off towards the table edge.
            fixed.m(Mat::MachineBlue).cuboid(
                at(-(p + 2.3), axis_y + 0.42, 0.0),
                Vec3::new(0.3, 0.14, 0.22),
                Quat::from_rotation_arc(Vec3::X, a),
            );
            for (s, mat) in [(-0.1, Mat::Dark), (0.1, Mat::PipeRed)] {
                let top = at(-(p + 2.3), axis_y + 0.56, s);
                let knee = at(-(p + 2.9), axis_y + 0.75, s + 0.9 * across.signum());
                let down = at(-(p + 3.2), 0.1, s + 1.3 * across.signum());
                fixed.m(mat).cylinder(top, knee, 0.045, 6, false);
                fixed.m(mat).cylinder(knee, down, 0.045, 6, false);
            }

            // Rod: slides in the barrel, its clevis follows the platform lug.
            let mut rod = Scenery::default();
            rod.m(Mat::Stainless)
                .cylinder(Vec3::ZERO, Vec3::X * 2.9, 0.11, 12, false);
            rod.m(Mat::Dark)
                .cuboid(Vec3::X * 2.98, Vec3::new(0.14, 0.14, 0.2), Quat::IDENTITY);
            parts.push(Part::local(
                Role::Rod(k as u8),
                Transform::from_translation(at(-(p + 3.0), axis_y, 0.0)).with_rotation(rot),
                rod,
            ));
        }

        // Lug on the platform edge, with the clevis pin.
        riding.m(Mat::MachineBlue).cuboid(
            at(-(p - 0.3), 0.33, 0.0),
            Vec3::new(0.26, 0.33, 0.32),
            rot,
        );
        riding.m(Mat::Stainless).cylinder(
            at(-(p + 0.02), axis_y, -0.28),
            at(-(p + 0.02), axis_y, 0.28),
            0.06,
            8,
            true,
        );
    }
    // Accelerometer block in the platform's far corner, with its lead.
    let acc = Vec3::new(p - 0.5, 0.0, -(p - 0.5));
    riding.m(Mat::MachineOrange).aabb(
        acc + Vec3::new(-0.15, 0.0, -0.15),
        acc + Vec3::new(0.15, 0.2, 0.15),
    );
    riding.m(Mat::Dark).bar(
        acc + Vec3::new(0.15, 0.03, 0.0),
        acc + Vec3::new(0.5, 0.03, 0.0),
        0.03,
        0.03,
        Vec3::Y,
    );
    parts.push(Part::driven(Role::Platform, Vec3::ZERO, riding));
    parts.push(Part::fixed(fixed));
    RigModel { parts }
}

/// Size of the cart's screen (x, y) and the height of its centre.
const SCREEN: Vec2 = Vec2::new(9.0, 5.4);
const SCREEN_Y: f32 = 5.6;
const SCREEN_Z: f32 = 0.3;

/// The procedural instrument cart. `floor` is the hall floor in its frame.
pub fn cart(floor: f32) -> RigModel {
    let mut s = Scenery::default();
    let (hx, hz) = (5.0, 3.6);
    let top = 0.8;
    // Frame, shelves, castors, push handle.
    for x in [-hx, hx] {
        for z in [-hz, hz] {
            s.m(Mat::LightSteel).aabb(
                Vec3::new(x - 0.25, floor + 1.6, z - 0.25),
                Vec3::new(x + 0.25, top, z + 0.25),
            );
            s.m(Mat::Dark).cylinder(
                Vec3::new(x - 0.25, floor + 0.8, z),
                Vec3::new(x + 0.25, floor + 0.8, z),
                0.8,
                14,
                true,
            );
        }
    }
    for y in [floor + 1.8, floor + 9.5, top - 0.3] {
        s.m(Mat::MachineGrey).aabb(
            Vec3::new(-hx - 0.3, y, -hz - 0.3),
            Vec3::new(hx + 0.3, y + 0.3, hz + 0.3),
        );
    }
    for z in [-hz, hz] {
        s.m(Mat::LightSteel).cylinder(
            Vec3::new(hx, top, z),
            Vec3::new(hx + 1.2, top + 1.6, z),
            0.12,
            8,
            false,
        );
    }
    s.m(Mat::LightSteel).cylinder(
        Vec3::new(hx + 1.2, top + 1.6, -hz),
        Vec3::new(hx + 1.2, top + 1.6, hz),
        0.12,
        8,
        true,
    );
    // Signal conditioners and the data logger on the shelves.
    s.m(Mat::MachineBlue).aabb(
        Vec3::new(-4.2, floor + 9.8, -2.8),
        Vec3::new(1.0, floor + 13.0, 2.8),
    );
    s.m(Mat::MachineGrey).aabb(
        Vec3::new(1.6, floor + 9.8, -2.6),
        Vec3::new(4.4, floor + 12.0, 2.6),
    );
    s.m(Mat::Dark).aabb(
        Vec3::new(-4.0, floor + 2.1, -2.8),
        Vec3::new(4.0, floor + 6.5, 2.8),
    );
    for k in 0..6 {
        let x = -3.7 + k as f32 * 0.8;
        let mat = if k == 4 {
            Mat::IndicatorAmber
        } else {
            Mat::IndicatorGreen
        };
        s.m(mat).aabb(
            Vec3::new(x, floor + 12.2, 2.8),
            Vec3::new(x + 0.25, floor + 12.45, 2.86),
        );
    }
    // Monitor on a stand, a keyboard in front of it.
    s.m(Mat::Dark)
        .aabb(Vec3::new(-1.6, top, -1.4), Vec3::new(1.6, top + 0.15, 0.6));
    s.m(Mat::Dark)
        .aabb(Vec3::new(-0.3, top, -0.6), Vec3::new(0.3, SCREEN_Y, -0.2));
    s.m(Mat::MachineGrey).aabb(
        Vec3::new(
            -SCREEN.x * 0.5 - 0.35,
            SCREEN_Y - SCREEN.y * 0.5 - 0.35,
            -0.2,
        ),
        Vec3::new(
            SCREEN.x * 0.5 + 0.35,
            SCREEN_Y + SCREEN.y * 0.5 + 0.35,
            SCREEN_Z - 0.08,
        ),
    );
    s.m(Mat::TableDark).aabb(
        Vec3::new(-SCREEN.x * 0.5, SCREEN_Y - SCREEN.y * 0.5, SCREEN_Z - 0.1),
        Vec3::new(SCREEN.x * 0.5, SCREEN_Y + SCREEN.y * 0.5, SCREEN_Z - 0.04),
    );
    // Graticule.
    for k in 1..8 {
        let x = -SCREEN.x * 0.5 + SCREEN.x * k as f32 / 8.0;
        s.m(Mat::GridMinor).aabb(
            Vec3::new(x - 0.015, SCREEN_Y - SCREEN.y * 0.5, SCREEN_Z - 0.04),
            Vec3::new(x + 0.015, SCREEN_Y + SCREEN.y * 0.5, SCREEN_Z - 0.03),
        );
    }
    for k in 1..4 {
        let y = SCREEN_Y - SCREEN.y * 0.5 + SCREEN.y * k as f32 / 4.0;
        s.m(Mat::GridMinor).aabb(
            Vec3::new(-SCREEN.x * 0.5, y - 0.015, SCREEN_Z - 0.04),
            Vec3::new(SCREEN.x * 0.5, y + 0.015, SCREEN_Z - 0.03),
        );
    }
    s.m(Mat::Dark)
        .aabb(Vec3::new(-2.6, top, 1.2), Vec3::new(2.6, top + 0.18, 3.0));
    RigModel {
        parts: vec![Part::fixed(s)],
    }
}

/// Where the cart stands at the table: behind it, turned towards the
/// player's side.
pub fn cart_pose(half: Vec2) -> Transform {
    Transform::from_xyz(half.x * 0.4, 0.0, -(half.y + 6.5))
        .with_rotation(Quat::from_rotation_y(0.3))
}

/// Samples the trace keeps.
const TRACE_SAMPLES: usize = 240;
/// Test time between samples, s: the screen shows the last 8 s.
const TRACE_STEP: f32 = 1.0 / 30.0;

/// The seismograph's line on the cart screen.
#[derive(Component)]
pub(crate) struct Trace {
    /// Table acceleration along its strongest axis, m/s², oldest first.
    samples: std::collections::VecDeque<f32>,
    since: f32,
}

/// Keeps the cart and the actuators in step with the setup.
pub(crate) fn call(
    mut commands: Commands,
    rigs: Res<LabRigs>,
    site: Site,
    models: Res<RigModels>,
    extra: Res<RigMaterials>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut maker: MaterialMaker,
    cache: Option<ResMut<LabMaterials>>,
    existing: Query<(Entity, &Rig, &Transform)>,
    mut placed: Local<Option<(Option<ShakeSetup>, Vec2)>>,
) {
    let Some(mut cache) = cache else { return };
    let half = site.table_half();
    let park = park::cart_pose();
    let find = |slot| existing.iter().find(|(_, r, _)| r.0 == slot);
    let (cart_root, at) = match find(RigSlot::InstrumentCart) {
        Some((e, _, t)) => (e, *t),
        None => {
            let floor = site.floor();
            let root = spawn_rig(
                &mut commands,
                &mut meshes,
                &mut maker,
                &mut cache,
                &models,
                RigSlot::InstrumentCart,
                park,
                &[],
                |_| cart(floor),
            );
            spawn_trace(&mut commands, &mut meshes, &extra, root);
            *placed = Some((None, half));
            (root, park)
        }
    };
    let want = (rigs.setup.shake, half);
    if *placed == Some(want) {
        return;
    }
    *placed = Some(want);
    let to = match rigs.setup.shake {
        Some(_) => cart_pose(half),
        None => park,
    };
    send(&mut commands, cart_root, at, to, 0.0);

    // The actuators are bolted to the shake table insert: they come and go
    // with it.
    for slot in [RigSlot::ShakeActuators, RigSlot::ShakeTable] {
        if let Some((e, _, _)) = find(slot) {
            commands.entity(e).despawn();
        }
    }
    if let Some(setup) = rigs.setup.shake {
        spawn_rig(
            &mut commands,
            &mut meshes,
            &mut maker,
            &mut cache,
            &models,
            RigSlot::ShakeActuators,
            Transform::IDENTITY,
            &hero_fits(setup.platform),
            |hero| actuators(setup.platform, hero),
        );
        // The hero platform replaces the table insert's own (hidden in
        // `drive`); without the model there is nothing to add.
        if models.has(RigSlot::ShakeTable) {
            spawn_rig(
                &mut commands,
                &mut meshes,
                &mut maker,
                &mut cache,
                &models,
                RigSlot::ShakeTable,
                Transform::IDENTITY,
                &[ShakeTableSpec::fit(setup.platform)],
                |_| RigModel::default(),
            );
        }
    }
}

fn spawn_trace(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    extra: &RigMaterials,
    root: Entity,
) {
    let segments = TRACE_SAMPLES - 1;
    let mut indices = Vec::with_capacity(segments * 6);
    for k in 0..segments as u32 {
        let a = k * 4;
        indices.extend_from_slice(&[a, a + 1, a + 2, a, a + 2, a + 3]);
    }
    let verts = segments * 4;
    let mesh = Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::default(),
    )
    .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, vec![[0.0f32; 3]; verts])
    .with_inserted_attribute(Mesh::ATTRIBUTE_NORMAL, vec![[0.0f32, 0.0, 1.0]; verts])
    .with_inserted_indices(Indices::U32(indices));
    commands.spawn((
        Mesh3d(meshes.add(mesh)),
        MeshMaterial3d(extra.trace.clone()),
        Transform::from_xyz(0.0, SCREEN_Y, SCREEN_Z),
        bevy::camera::visibility::NoFrustumCulling,
        NotShadowCaster,
        Trace {
            samples: std::collections::VecDeque::from(vec![0.0; TRACE_SAMPLES]),
            since: 0.0,
        },
        ChildOf(root),
    ));
}

/// Moves the platform, everything that rides it, and the actuator rods.
pub(crate) fn drive(
    rigs: Res<LabRigs>,
    models: Res<RigModels>,
    owners: Query<&Rig>,
    parents: Query<&ChildOf>,
    globals: Query<&GlobalTransform>,
    mut platform: Query<(&mut Transform, &mut Visibility), (With<ShakePlatform>, Without<Role>)>,
    mut parts: Query<(Entity, &Role, &Rest, &Owner, Option<&Axis>, &mut Transform)>,
) {
    let offset = rigs.live.table.map_or(Vec3::ZERO, |t| t.displacement);
    // The hero platform stands in for the table insert's.
    let hero = models.has(RigSlot::ShakeTable) && rigs.setup.shake.is_some();
    for (mut at, mut vis) in &mut platform {
        if at.translation != offset {
            at.translation = offset;
        }
        vis.set_if_neq(if hero {
            Visibility::Hidden
        } else {
            Visibility::Inherited
        });
    }
    for (e, role, rest, owner, axis, mut at) in &mut parts {
        if !matches!(
            owners.get(owner.0).map(|r| r.0),
            Ok(RigSlot::ShakeActuators | RigSlot::ShakeTable)
        ) {
            continue;
        }
        let local = in_parent(&parents, &globals, e, offset);
        let to = match role {
            Role::Platform => rest.0.translation + local,
            Role::Rod(_) => {
                let axis = rest.0.rotation * axis.map_or(Vec3::X, |a| a.0);
                rest.0.translation + axis * local.dot(axis)
            }
            _ => continue,
        };
        if at.translation != to {
            at.translation = to;
        }
    }
}

/// Scrolls the seismograph: the table's acceleration along its strongest
/// axis over the last few seconds, scaled to the largest value on screen.
pub(crate) fn trace(
    rigs: Res<LabRigs>,
    clock: Res<RigClock>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut q: Query<(&mut Trace, &Mesh3d)>,
    mut drawn: Local<bool>,
) {
    let Ok((mut trace, mesh)) = q.single_mut() else {
        return;
    };
    let a = rigs.live.table.map_or(Vec3::ZERO, |t| t.acceleration);
    // Signed along whichever horizontal axis the motion mostly has.
    let value = if a.x.abs() >= a.z.abs() { a.x } else { a.z };
    trace.since += clock.dt;
    let mut changed = !*drawn;
    while trace.since >= TRACE_STEP {
        trace.since -= TRACE_STEP;
        trace.samples.pop_front();
        trace.samples.push_back(value);
        changed = true;
    }
    if !changed {
        return;
    }
    *drawn = true;
    let Some(mut mesh) = meshes.get_mut(&mesh.0) else {
        return;
    };
    // Full scale: at least 1 m/s², so noise does not fill the screen.
    let peak = trace.samples.iter().fold(1.0f32, |m, v| m.max(v.abs()));
    let point = |k: usize, v: f32| {
        Vec2::new(
            (k as f32 / (TRACE_SAMPLES - 1) as f32 - 0.5) * (SCREEN.x - 0.3),
            v / peak * (SCREEN.y * 0.5 - 0.3),
        )
    };
    let half = 0.03;
    let mut positions = Vec::with_capacity((TRACE_SAMPLES - 1) * 4);
    let mut prev = point(0, trace.samples[0]);
    for (k, &v) in trace.samples.iter().enumerate().skip(1) {
        let next = point(k, v);
        let dir = (next - prev).normalize_or(Vec2::X);
        let n = Vec2::new(-dir.y, dir.x) * half;
        // Overlap a little so the corners of a steep line stay closed.
        let (a, b) = (prev - dir * half, next + dir * half);
        for p in [a - n, b - n, b + n, a + n] {
            positions.push([p.x, p.y, 0.0]);
        }
        prev = next;
    }
    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::PlayTable;

    #[test]
    fn actuators_fit_on_the_surround() {
        let size = ShakeSetup::default().platform;
        let half = PlayTable::default().size * 0.5;
        let m = actuators(size, false);
        assert_eq!(
            m.parts
                .iter()
                .filter(|p| matches!(p.role, Some(Role::Rod(_))))
                .count(),
            4
        );
        let all = m.merged();
        for p in all.bufs.values().flat_map(|b| b.positions.iter()) {
            assert!(
                p[0].abs() <= half.x && p[2].abs() <= half.y,
                "{p:?} hangs over the table edge"
            );
            assert!(p[1] >= -1e-4, "{p:?} is below the playing surface");
        }
    }

    #[test]
    fn rods_point_at_the_platform() {
        for part in actuators(8.0, false).parts {
            if let Some(Role::Rod(k)) = part.role {
                let (axis, _) = actuator_axes()[k as usize];
                assert!((part.at.rotation * Vec3::X).distance(axis) < 1e-5);
                // The rod starts outside the platform and ends at its edge.
                assert!(part.at.translation.dot(axis) < -4.0);
            }
        }
    }

    #[test]
    fn the_cart_stands_on_the_floor_behind_the_table() {
        let floor = -17.2;
        let all = cart(floor).merged();
        let lo = all
            .bufs
            .values()
            .flat_map(|b| b.positions.iter())
            .map(|p| p[1])
            .fold(f32::MAX, f32::min);
        assert!((lo - floor).abs() < 1e-3);
        let half = Vec2::new(14.35, 8.35);
        assert!(cart_pose(half).translation.z < -half.y - 3.6);
    }
}
