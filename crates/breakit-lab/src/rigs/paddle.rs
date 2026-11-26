//! The wave flume's piston paddle: a plate hanging from a carriage that
//! drives itself along racked rails on the banks of the channel. The rails
//! are long, because a solitary wave is made with one push of several
//! metres.
//!
//! The water is not drawn here (`breakit-water` owns it); the plate's
//! downstream face is the water's upstream wall, at
//! `FlumeDrive::paddle_rest + paddle_position` along the wave direction.
//!
//! Rig frame: origin in the middle of the table top, +x the way the waves
//! travel.

use bevy::prelude::*;

use super::{
    park, send, spawn_rig, yaw_to, LabRigs, Owner, PaddleSetup, Part, Rest, Rig, RigModel,
    RigModels, RigSlot, Role, Site,
};
use crate::palette::Mat;
use crate::scenery::Scenery;
use crate::{LabMaterials, MaterialMaker};

/// Top of the plate above the playing surface.
pub const PLATE_TOP: f32 = 0.75;
/// How far the carriage can travel: a little back from rest, and far
/// enough forward for the single long push of a solitary wave.
pub const TRAVEL: (f32, f32) = (-0.3, 7.2);

/// The procedural paddle for a channel.
pub fn model(setup: &PaddleSetup) -> RigModel {
    let rest = setup.rest;
    let hw = setup.width * 0.5;
    let rail_z = hw + 0.6;
    let (r0, r1) = (rest + TRAVEL.0 - 0.45, rest + TRAVEL.1 + 0.6);
    let mut fixed = Scenery::default();

    // Rails with a rack on pedestals either side of the channel.
    for z in [-rail_z, rail_z] {
        fixed.m(Mat::Stainless).cylinder(
            Vec3::new(r0, 0.34, z),
            Vec3::new(r1, 0.34, z),
            0.08,
            10,
            true,
        );
        fixed.m(Mat::Dark).aabb(
            Vec3::new(r0, 0.16, z + z.signum() * 0.16 - 0.04),
            Vec3::new(r1, 0.24, z + z.signum() * 0.16 + 0.04),
        );
        let mut x = r0 + 0.2;
        while x < r1 {
            fixed.m(Mat::LightSteel).aabb(
                Vec3::new(x - 0.1, 0.0, z - 0.22),
                Vec3::new(x + 0.1, 0.26, z + 0.22),
            );
            x += 1.3;
        }
        // End stops with rubber buffers.
        for (x, toward) in [(r0, 1.0), (r1, -1.0)] {
            fixed.m(Mat::Yellow).aabb(
                Vec3::new(x - 0.07, 0.0, z - 0.2),
                Vec3::new(x + 0.07, 0.55, z + 0.2),
            );
            fixed.m(Mat::Dark).cylinder(
                Vec3::new(x, 0.34, z),
                Vec3::new(x + toward * 0.16, 0.34, z),
                0.1,
                10,
                true,
            );
        }
    }
    // Drive cabinet on the bank beside the rest position, cable to it.
    let cab = Vec3::new(rest + 0.2, 0.0, rail_z + 1.0);
    fixed.m(Mat::MachineGrey).aabb(
        cab + Vec3::new(-0.45, 0.0, -0.3),
        cab + Vec3::new(0.45, 1.1, 0.3),
    );
    fixed.m(Mat::Dark).aabb(
        cab + Vec3::new(-0.3, 0.6, -0.31),
        cab + Vec3::new(0.3, 0.95, -0.3),
    );
    fixed.m(Mat::Dark).bar(
        cab + Vec3::new(0.0, 0.05, 0.3),
        cab + Vec3::new(-0.6, 0.05, 2.0),
        0.08,
        0.08,
        Vec3::Y,
    );

    // Plate and carriage: move together.
    let mut plate = Scenery::default();
    plate.m(Mat::Stainless).aabb(
        Vec3::new(rest - 0.1, setup.bed + 0.05, -hw + 0.05),
        Vec3::new(rest, PLATE_TOP, hw - 0.05),
    );
    // Wiper seals down the sides and stiffeners on the dry face.
    for z in [-hw + 0.05, hw - 0.05] {
        plate.m(Mat::Dark).aabb(
            Vec3::new(rest - 0.06, setup.bed + 0.05, z - 0.04),
            Vec3::new(rest - 0.02, PLATE_TOP, z + 0.04),
        );
    }
    for z in [-hw * 0.5, 0.0, hw * 0.5] {
        plate.m(Mat::LightSteel).aabb(
            Vec3::new(rest - 0.3, setup.bed + 0.4, z - 0.04),
            Vec3::new(rest - 0.1, PLATE_TOP, z + 0.04),
        );
        plate.m(Mat::LightSteel).bar(
            Vec3::new(rest - 0.1, setup.bed + 0.6, z),
            Vec3::new(rest - 0.4, PLATE_TOP + 0.1, z),
            0.06,
            0.06,
            Vec3::Y,
        );
    }
    // Carriage beam, bearing blocks and the two pinion drives.
    plate.m(Mat::MachineOrange).aabb(
        Vec3::new(rest - 0.5, PLATE_TOP, -rail_z - 0.3),
        Vec3::new(rest, PLATE_TOP + 0.5, rail_z + 0.3),
    );
    for z in [-rail_z, rail_z] {
        plate.m(Mat::Dark).aabb(
            Vec3::new(rest - 0.45, 0.22, z - 0.2),
            Vec3::new(rest - 0.05, PLATE_TOP, z + 0.2),
        );
        let out = z.signum();
        plate.m(Mat::MachineBlue).aabb(
            Vec3::new(rest - 0.5, 0.5, z + out * 0.2),
            Vec3::new(rest - 0.05, 1.0, z + out * 0.75),
        );
        plate.m(Mat::Dark).cylinder(
            Vec3::new(rest - 0.27, 1.0, z + out * 0.48),
            Vec3::new(rest - 0.27, 1.45, z + out * 0.48),
            0.18,
            12,
            true,
        );
    }
    // Wave gauge staff on the wet face, read from the side.
    plate.m(Mat::Yellow).aabb(
        Vec3::new(rest, -1.4, hw - 0.3),
        Vec3::new(rest + 0.02, PLATE_TOP, hw - 0.12),
    );

    let beacon = cab + Vec3::Y * 1.1;
    let mut parts = vec![Part::driven(Role::Plate, Vec3::new(rest, 0.0, 0.0), plate)];
    for (role, mat) in [
        (Role::LampOn, Mat::IndicatorGreen),
        (Role::LampOff, Mat::IndicatorAmber),
    ] {
        let mut lamp = Scenery::default();
        lamp.m(mat)
            .cylinder(beacon, beacon + Vec3::Y * 0.22, 0.12, 10, true);
        parts.push(Part::driven(role, beacon, lamp));
    }
    parts.push(Part::fixed(fixed));
    RigModel { parts }
}

/// Where the paddle is mounted for waves along `direction`.
pub fn working_pose(setup: &PaddleSetup) -> Transform {
    Transform::from_rotation(yaw_to(setup.direction))
}

/// Builds the paddle for the setup, parks it, cranes it over to the table
/// and back.
#[allow(clippy::too_many_arguments)]
pub(crate) fn call(
    mut commands: Commands,
    rigs: Res<LabRigs>,
    site: Site,
    models: Res<RigModels>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut maker: MaterialMaker,
    cache: Option<ResMut<LabMaterials>>,
    existing: Query<(Entity, &Rig, &Transform)>,
    mut placed: Local<Option<Option<PaddleSetup>>>,
    mut built_for: Local<Option<PaddleSetup>>,
) {
    let Some(mut cache) = cache else { return };
    let found = existing.iter().find(|(_, r, _)| r.0 == RigSlot::WavePaddle);
    if found.is_some() && *placed == Some(rigs.setup.paddle) {
        return;
    }
    *placed = Some(rigs.setup.paddle);
    // The parked paddle is the one for the default flume; a scenario with
    // another channel gets its own.
    let shape = rigs.setup.paddle.or(*built_for).unwrap_or_default();
    let same_shape =
        built_for.is_some_and(|b| (b.rest, b.width, b.bed) == (shape.rest, shape.width, shape.bed));
    let park = park::paddle_pose(&shape, site.floor());
    let (root, at) = match found {
        Some((e, _, t)) if same_shape => (e, *t),
        other => {
            if let Some((e, _, _)) = other {
                commands.entity(e).despawn();
            }
            let root = spawn_rig(
                &mut commands,
                &mut meshes,
                &mut maker,
                &mut cache,
                &models,
                RigSlot::WavePaddle,
                park,
                || model(&shape),
            );
            (root, park)
        }
    };
    *built_for = Some(shape);
    let to = match rigs.setup.paddle {
        Some(p) => working_pose(&p),
        None => park,
    };
    send(&mut commands, root, at, to, 12.0);
}

pub(crate) fn drive(
    rigs: Res<LabRigs>,
    owners: Query<&Rig>,
    mut parts: Query<(&Role, &Rest, &Owner, &mut Transform, &mut Visibility)>,
) {
    let flume = rigs.live.flume.unwrap_or_default();
    let stroke = flume.paddle_position.clamp(TRAVEL.0, TRAVEL.1);
    let running = flume.paddle_velocity.abs() > 0.01;
    for (role, rest, owner, mut at, mut vis) in &mut parts {
        if owners.get(owner.0).map(|r| r.0) != Ok(RigSlot::WavePaddle) {
            continue;
        }
        match role {
            Role::Plate => {
                let to = rest.0.translation + rest.0.rotation * Vec3::X * stroke;
                if at.translation != to {
                    at.translation = to;
                }
            }
            Role::LampOn => {
                vis.set_if_neq(if running {
                    Visibility::Inherited
                } else {
                    Visibility::Hidden
                });
            }
            Role::LampOff => {
                vis.set_if_neq(if running {
                    Visibility::Hidden
                } else {
                    Visibility::Inherited
                });
            }
            _ => {}
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_plate_face_is_the_waters_upstream_wall() {
        let setup = PaddleSetup::default();
        let plate = model(&setup)
            .parts
            .into_iter()
            .find(|p| p.role == Some(Role::Plate))
            .unwrap();
        // Stainless is the plate itself; its downstream face sits on the
        // pivot, which is at the rest position.
        assert_eq!(plate.at.translation, Vec3::new(setup.rest, 0.0, 0.0));
        let steel = &plate.geom.bufs[&Mat::Stainless];
        let face = steel
            .positions
            .iter()
            .map(|p| p[0])
            .fold(f32::MIN, f32::max);
        assert!(face.abs() < 1e-5);
        let (lo, hi) = steel
            .positions
            .iter()
            .fold((f32::MAX, f32::MIN), |m, p| (m.0.min(p[1]), m.1.max(p[1])));
        assert!(lo > setup.bed && lo < setup.bed + 0.1);
        assert!((hi - PLATE_TOP).abs() < 1e-5);
        let wide = steel
            .positions
            .iter()
            .map(|p| p[2].abs())
            .fold(0.0, f32::max);
        assert!(wide < setup.width * 0.5);
    }

    #[test]
    fn the_rails_cover_the_travel_and_stay_on_the_table() {
        let setup = PaddleSetup::default();
        let half = crate::PlayTable::default().size * 0.5;
        let fixed = model(&setup).parts.pop().unwrap();
        assert!(fixed.role.is_none());
        let rail = &fixed.geom.bufs[&Mat::Stainless];
        let (lo, hi) = rail
            .positions
            .iter()
            .fold((f32::MAX, f32::MIN), |m, p| (m.0.min(p[0]), m.1.max(p[0])));
        assert!(lo <= setup.rest + TRAVEL.0 - 0.45 && hi >= setup.rest + TRAVEL.1);
        assert!(lo >= -half.x - 0.35 && hi <= half.x);
        for p in fixed.geom.bufs.values().flat_map(|b| b.positions.iter()) {
            assert!(p[1] >= -1e-4 && p[2].abs() <= half.y);
        }
    }
}
