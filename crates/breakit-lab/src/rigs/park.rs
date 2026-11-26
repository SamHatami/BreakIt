//! The rig park: the bay on the hall floor where the rigs wait. The fan
//! wall and the instrument cart roll from here to the table, the wave
//! paddle is lifted over from its trolley, and the spare model vehicles
//! stand in a row.
//!
//! Everything here is in world units on the hall floor, inside the patch
//! the hall's dressing keeps clear around the table.

use bevy::camera::visibility::RenderLayers;
use bevy::prelude::*;

use super::{truck, yaw_to, PaddleSetup, VehicleKind};
use crate::palette::Mat;
use crate::scenery::Scenery;
use crate::{LabMaterials, LabSettings, MaterialMaker, HALL_LAYER};

/// The bay, world x and z: (min, max).
pub const BAY: (Vec2, Vec2) = (Vec2::new(-43.5, -37.5), Vec2::new(-19.0, -12.5));

/// Where the fan wall parks: at the back of the bay, facing the table.
pub fn fan_pose() -> Transform {
    Transform::from_xyz(-28.0, 0.0, -32.8).with_rotation(yaw_to(Vec3::Z))
}

/// Where the instrument cart parks.
pub fn cart_pose() -> Transform {
    Transform::from_xyz(-24.5, 0.0, -24.5).with_rotation(Quat::from_rotation_y(0.9))
}

/// Middle of the paddle's trolley, world x and z, and its top above the
/// floor.
const TROLLEY: (Vec2, f32) = (Vec2::new(-39.0, -25.5), 1.6);

/// Where the paddle rests on its trolley: turned across the bay, the foot
/// of its plate on the trolley top.
pub fn paddle_pose(setup: &PaddleSetup, floor: f32) -> Transform {
    let turn = Quat::from_rotation_y(std::f32::consts::FRAC_PI_2);
    let foot = Vec3::new(setup.rest, setup.bed + 0.05, 0.0);
    let on = Vec3::new(TROLLEY.0.x, floor + TROLLEY.1, TROLLEY.0.y);
    Transform::from_translation(on - turn * foot).with_rotation(turn)
}

/// Where the spare vehicle `k` stands (its front axle), heading +x.
pub fn truck_spot(k: usize, floor: f32) -> Vec3 {
    Vec3::new(-24.0, floor, -14.6 - 2.3 * k as f32)
}

/// The bay's fixtures and the spare vehicles, in world units.
pub fn build(floor: f32) -> Scenery {
    let mut s = Scenery::default();
    let (lo, hi) = BAY;
    let y = floor + 0.04;
    // Bay outline with hatched corners.
    let line = 0.5;
    let rect = |s: &mut Scenery, a: Vec2, b: Vec2| {
        s.m(Mat::FloorPaint)
            .aabb(Vec3::new(a.x, floor, a.y), Vec3::new(b.x, y, b.y));
    };
    rect(&mut s, lo, Vec2::new(hi.x, lo.y + line));
    rect(&mut s, Vec2::new(lo.x, hi.y - line), hi);
    rect(&mut s, lo, Vec2::new(lo.x + line, hi.y));
    rect(&mut s, Vec2::new(hi.x - line, lo.y), hi);
    for k in 0..7 {
        // Chevrons along the side the rigs leave by.
        let z = lo.y + 2.0 + k as f32 * 3.5;
        s.m(Mat::FloorPaint).cuboid(
            Vec3::new(hi.x + 1.3, floor + 0.02, z),
            Vec3::new(1.2, 0.02, 0.25),
            Quat::from_rotation_y(0.7),
        );
    }
    // Guard rail along the back and the far side.
    let rail = |s: &mut Scenery, a: Vec3, b: Vec3| {
        let n = (a.distance(b) / 6.0).ceil() as i32;
        for k in 0..=n {
            let p = a.lerp(b, k as f32 / n as f32);
            s.m(Mat::Yellow)
                .cylinder(p, p + Vec3::Y * 10.5, 0.28, 10, true);
            s.m(Mat::Dark).cylinder(p, p + Vec3::Y * 0.2, 0.7, 10, true);
        }
        for h in [5.5, 10.2] {
            s.m(Mat::Yellow)
                .cylinder(a + Vec3::Y * h, b + Vec3::Y * h, 0.22, 8, false);
        }
    };
    rail(
        &mut s,
        Vec3::new(lo.x - 1.2, floor, lo.y - 1.0),
        Vec3::new(lo.x - 1.2, floor, hi.y),
    );
    rail(
        &mut s,
        Vec3::new(lo.x - 1.2, floor, lo.y - 1.0),
        Vec3::new(hi.x, floor, lo.y - 1.0),
    );

    // Trolley for the paddle: a low steel table on castors.
    let (c, top) = (Vec3::new(TROLLEY.0.x, floor, TROLLEY.0.y), TROLLEY.1);
    let half = Vec2::new(4.2, 2.2);
    s.m(Mat::MachineGrey).aabb(
        c + Vec3::new(-half.x, top - 0.3, -half.y),
        c + Vec3::new(half.x, top, half.y),
    );
    for x in [-half.x + 0.5, half.x - 0.5] {
        for z in [-half.y + 0.5, half.y - 0.5] {
            s.m(Mat::Dark).cylinder(
                c + Vec3::new(x, 0.55, z - 0.2),
                c + Vec3::new(x, 0.55, z + 0.2),
                0.55,
                12,
                true,
            );
            s.m(Mat::LightSteel).aabb(
                c + Vec3::new(x - 0.12, 0.55, z - 0.28),
                c + Vec3::new(x + 0.12, top - 0.3, z + 0.28),
            );
        }
    }

    // Test masses: stamped steel blocks with lifting eyes, stacked.
    let stack = Vec3::new(lo.x + 3.2, floor, hi.y - 4.0);
    for (k, (dx, dz, layer)) in [
        (0.0, 0.0, 0),
        (2.2, 0.0, 0),
        (0.0, 2.2, 0),
        (2.2, 2.2, 0),
        (0.6, 0.5, 1),
        (1.4, 1.9, 2),
    ]
    .into_iter()
    .enumerate()
    {
        let size = if layer == 2 { 0.7 } else { 1.0 };
        let base = stack + Vec3::new(dx, layer as f32 * 1.6, dz);
        let mat = if k % 2 == 0 {
            Mat::FloorSteel
        } else {
            Mat::Dark
        };
        s.m(mat).aabb(
            base + Vec3::new(-size, 0.0, -size),
            base + Vec3::new(size, 1.6, size),
        );
        s.m(Mat::Yellow)
            .ring(base + Vec3::Y * 1.9, Vec3::X, 0.35, 0.1, 0.1, 12);
    }

    // Spare vehicles, one of each kind that is not a plain box truck.
    let spares: [(VehicleKind, &[f32]); 3] = [
        (VehicleKind::Dump, &[0.0, 3.4, 4.7]),
        (VehicleKind::CoilHauler, &[0.0, 3.8, 5.1]),
        (VehicleKind::Crane, &[0.0, 1.6, 4.4, 5.8]),
    ];
    for (k, (kind, axles)) in spares.into_iter().enumerate() {
        let at = Transform::from_translation(truck_spot(k, floor));
        let model = truck::model(kind, axles, truck::DEFAULT_WIDTH).merged();
        for (mat, buf) in model.bufs {
            s.m(mat).append_transformed(&buf, &at);
        }
    }
    s
}

/// Spawns the bay once the lab's materials exist.
pub(crate) fn spawn_park(
    mut commands: Commands,
    settings: Res<LabSettings>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut maker: MaterialMaker,
    cache: Option<ResMut<LabMaterials>>,
    mut done: Local<bool>,
) {
    let Some(mut cache) = cache else { return };
    if *done {
        return;
    }
    *done = true;
    let root = commands
        .spawn((
            Transform::IDENTITY,
            Visibility::Inherited,
            Name::new("rig park"),
        ))
        .id();
    for (mat, buf) in build(settings.floor_y()).bufs {
        if buf.is_empty() {
            continue;
        }
        let material = maker.handle(&mut cache, mat, false);
        let mut e = commands.spawn((
            Mesh3d(meshes.add(buf.into_mesh())),
            Transform::IDENTITY,
            // Part of the hall: lit by the lamps, out of the daylight's
            // table-sized shadow maps.
            RenderLayers::layer(HALL_LAYER),
            ChildOf(root),
        ));
        crate::insert_material(&mut e, material);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rigs::{fan, shake};
    use crate::PlayTable;

    /// Footprint (x, z) of everything in `s` placed at `at`.
    fn footprint(s: &Scenery, at: &Transform) -> (Vec2, Vec2) {
        let mut lo = Vec2::splat(f32::MAX);
        let mut hi = Vec2::splat(f32::MIN);
        for p in s.bufs.values().flat_map(|b| b.positions.iter()) {
            let w = at.transform_point(Vec3::from_array(*p));
            lo = lo.min(Vec2::new(w.x, w.z));
            hi = hi.max(Vec2::new(w.x, w.z));
        }
        (lo, hi)
    }

    /// The patch of floor the hall keeps free of dressing around the table
    /// (see `table_area_is_clear_of_dressing`), world units.
    fn clear_patch() -> Vec2 {
        let s = LabSettings::default();
        PlayTable::default().size * 0.5 + Vec2::splat(1.5 * s.model_scale)
    }

    #[test]
    fn the_park_and_parked_rigs_stay_on_the_clear_floor_and_off_the_table() {
        let floor = LabSettings::default().floor_y();
        let clear = clear_patch();
        let table = PlayTable::default().size * 0.5 + Vec2::splat(0.35);
        let paddle = PaddleSetup::default();
        let parked = [
            ("park", build(floor), Transform::IDENTITY),
            ("fan wall", fan::model(floor).merged(), fan_pose()),
            ("cart", shake::cart(floor).merged(), cart_pose()),
            (
                "paddle",
                crate::rigs::paddle::model(&paddle).merged(),
                paddle_pose(&paddle, floor),
            ),
        ];
        for (name, s, at) in &parked {
            let (lo, hi) = footprint(s, at);
            assert!(
                lo.x > -clear.x && hi.x < clear.x && lo.y > -clear.y && hi.y < clear.y,
                "{name} leaves the clear floor: {lo} .. {hi}"
            );
            assert!(
                hi.x < -table.x || hi.y < -table.y,
                "{name} stands in the table: {lo} .. {hi}"
            );
        }
    }

    #[test]
    fn the_paddle_rests_on_its_trolley() {
        let floor = -17.2;
        let setup = PaddleSetup::default();
        let at = paddle_pose(&setup, floor);
        let all = crate::rigs::paddle::model(&setup).merged();
        let lo = all
            .bufs
            .values()
            .flat_map(|b| b.positions.iter())
            .map(|p| at.transform_point(Vec3::from_array(*p)).y)
            .fold(f32::MAX, f32::min);
        assert!((lo - (floor + TROLLEY.1)).abs() < 1e-3, "foot at {lo}");
    }
}
