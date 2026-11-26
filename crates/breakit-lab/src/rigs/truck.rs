//! The heavy model vehicles: a box truck, a dump truck, a coil hauler and
//! a mobile crane. Each is built around the axles the load model gives it
//! and follows its wheel contacts: wheels sit on the deck where the tyres
//! press, the body rides above on the suspension's bounce.
//!
//! Vehicle frame: origin on the ground under the middle of the front
//! axle, +x forward, +y up.

use bevy::prelude::*;

use super::{
    spawn_rig, LabRigs, Owner, Part, Rest, RigClock, RigModel, RigModels, RigSlot, Role,
    VehicleDrive,
};
use crate::palette::Mat;
use crate::scenery::Scenery;
use crate::{LabMaterials, MaterialMaker};

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum VehicleKind {
    #[default]
    Box,
    Dump,
    CoilHauler,
    Crane,
}

impl VehicleKind {
    pub const ALL: [VehicleKind; 4] = [
        VehicleKind::Box,
        VehicleKind::Dump,
        VehicleKind::CoilHauler,
        VehicleKind::Crane,
    ];

    /// The model for a vehicle called `name` in a scenario: "tipper" or
    /// "dump" is a dump truck, "coil" a coil hauler, "crane" a mobile
    /// crane, anything else a box truck.
    pub fn from_name(name: &str) -> Self {
        let n = name.to_ascii_lowercase();
        if n.contains("tipper") || n.contains("dump") {
            VehicleKind::Dump
        } else if n.contains("coil") {
            VehicleKind::CoilHauler
        } else if n.contains("crane") {
            VehicleKind::Crane
        } else {
            VehicleKind::Box
        }
    }
}

/// Body width of a vehicle when the scenario does not say, world units.
pub const DEFAULT_WIDTH: f32 = 1.3;

/// Wheel radius for a vehicle of `width`.
pub fn wheel_radius(width: f32) -> f32 {
    0.34 * width
}

/// A procedural vehicle. `axles` are the distances of each axle behind the
/// front one (the first is 0), `width` the body width.
pub fn model(kind: VehicleKind, axles: &[f32], width: f32) -> RigModel {
    let w = width;
    let hw = w * 0.5;
    let r = wheel_radius(w);
    let last = axles.iter().copied().fold(0.0, f32::max).max(2.0 * r);
    let tail = -(last + 1.5 * r);
    let nose = 2.3 * r;
    let frame_top = 1.5 * r;
    let mut b = Scenery::default();

    // Ladder frame, bumper, fuel tank, mudguards.
    for z in [-0.28 * w, 0.28 * w] {
        b.m(Mat::Dark).aabb(
            Vec3::new(tail, 0.95 * r, z - 0.05 * w),
            Vec3::new(nose - 0.1 * r, frame_top, z + 0.05 * w),
        );
    }
    b.m(Mat::Dark).aabb(
        Vec3::new(nose - 0.15 * r, 0.6 * r, -hw),
        Vec3::new(nose + 0.1 * r, 1.25 * r, hw),
    );
    b.m(Mat::Stainless).cylinder(
        Vec3::new(-1.6 * r, 0.95 * r, hw - 0.14 * w),
        Vec3::new(-(last - 1.4 * r).max(2.4 * r), 0.95 * r, hw - 0.14 * w),
        0.3 * r,
        12,
        true,
    );
    for &a in axles {
        b.m(Mat::Dark).aabb(
            Vec3::new(-a - 1.15 * r, 2.02 * r, -hw),
            Vec3::new(-a + 1.15 * r, 2.1 * r, hw),
        );
    }

    let cab_colour = match kind {
        VehicleKind::Box => Mat::MachineBlue,
        VehicleKind::Dump => Mat::Yellow,
        VehicleKind::CoilHauler => Mat::MachineGreen,
        VehicleKind::Crane => Mat::Yellow,
    };
    let cab_top = frame_top + 1.15 * w;
    let cab_back = -r;
    if kind != VehicleKind::Crane {
        cab(&mut b, cab_colour, cab_back, nose, frame_top, cab_top, w);
    }
    let bed_front = cab_back - 0.25 * r;
    let bed = frame_top + 0.08 * w;
    match kind {
        VehicleKind::Box => {
            let top = cab_top + 0.25 * w;
            b.m(Mat::MachineGrey).aabb(
                Vec3::new(tail, bed, -hw * 1.04),
                Vec3::new(bed_front, top, hw * 1.04),
            );
            // Corner rails, side ribs and the rear door posts.
            let mut x = tail;
            while x <= bed_front + 1e-3 {
                for z in [-hw * 1.04, hw * 1.04] {
                    b.m(Mat::LightSteel).aabb(
                        Vec3::new(x - 0.03 * w, bed, z - 0.015 * w),
                        Vec3::new(x + 0.03 * w, top, z + 0.015 * w),
                    );
                }
                x += (bed_front - tail) / ((bed_front - tail) / (0.7 * w)).round().max(1.0);
            }
            for y in [bed, top] {
                b.m(Mat::LightSteel).aabb(
                    Vec3::new(tail - 0.01 * w, y - 0.03 * w, -hw * 1.06),
                    Vec3::new(bed_front + 0.01 * w, y + 0.03 * w, hw * 1.06),
                );
            }
            b.m(Mat::LightSteel).aabb(
                Vec3::new(tail - 0.015 * w, bed, -0.02 * w),
                Vec3::new(tail, top, 0.02 * w),
            );
        }
        VehicleKind::Dump => {
            let wall = 0.62 * w;
            let t = 0.05 * w;
            // Tub: floor, sides, headboard with a cab shield, tailgate.
            b.m(Mat::MachineOrange)
                .aabb(Vec3::new(tail, bed, -hw), Vec3::new(bed_front, bed + t, hw));
            for z in [-hw, hw - t] {
                b.m(Mat::MachineOrange).aabb(
                    Vec3::new(tail, bed, z),
                    Vec3::new(bed_front, bed + wall, z + t),
                );
            }
            b.m(Mat::MachineOrange).aabb(
                Vec3::new(bed_front - t, bed, -hw),
                Vec3::new(bed_front, cab_top + 0.12 * w, hw),
            );
            b.m(Mat::MachineOrange).aabb(
                Vec3::new(bed_front - t, cab_top + 0.06 * w, -hw),
                Vec3::new(nose - 0.9 * r, cab_top + 0.12 * w, hw),
            );
            b.m(Mat::MachineOrange).cuboid(
                Vec3::new(tail - 0.08 * w, bed + wall * 0.5, 0.0),
                Vec3::new(t * 0.5, wall * 0.52, hw),
                Quat::from_rotation_z(0.25),
            );
            // Ribs.
            let n = (((bed_front - tail) / (0.55 * w)).round() as i32).max(2);
            for k in 0..=n {
                let x = tail + (bed_front - tail) * k as f32 / n as f32;
                for z in [-hw - 0.03 * w, hw] {
                    b.m(Mat::MachineOrange).aabb(
                        Vec3::new(x - 0.03 * w, bed, z),
                        Vec3::new(x + 0.03 * w, bed + wall, z + 0.03 * w),
                    );
                }
            }
            // A heaped load of aggregate.
            let len = bed_front - tail;
            let heaps = ((len / (0.6 * w)).floor() as i32).max(2);
            for k in 0..heaps {
                let x = tail + len * (k as f32 + 0.5) / heaps as f32;
                let side = if k % 2 == 0 { -0.08 } else { 0.08 };
                b.m(Mat::Concrete).sphere(
                    Vec3::new(x, bed + wall * 0.72, side * w),
                    0.42 * w,
                    12,
                    true,
                );
            }
            b.m(Mat::Concrete).aabb(
                Vec3::new(tail + t, bed + t, -hw + t),
                Vec3::new(bed_front - t, bed + wall * 0.74, hw - t),
            );
        }
        VehicleKind::CoilHauler => {
            b.m(Mat::Timber).aabb(
                Vec3::new(tail, bed, -hw),
                Vec3::new(bed_front, bed + 0.06 * w, hw),
            );
            b.m(Mat::LightSteel).aabb(
                Vec3::new(tail, bed - 0.05 * w, -hw - 0.01 * w),
                Vec3::new(bed_front, bed, hw + 0.01 * w),
            );
            // Headboard.
            b.m(Mat::LightSteel).aabb(
                Vec3::new(bed_front - 0.05 * w, bed, -hw),
                Vec3::new(bed_front, cab_top - 0.1 * w, hw),
            );
            // Steel coils on timber cradles, eye to the side, chained down.
            let deck = bed + 0.06 * w;
            let len = bed_front - 0.1 * w - tail;
            let cr = 0.45 * w;
            let n = ((len / (2.0 * cr + 0.25 * w)).floor() as i32).max(1);
            for k in 0..n {
                let x = tail + len * (k as f32 + 0.5) / n as f32;
                let c = Vec3::new(x, deck + cr + 0.04 * w, 0.0);
                let half = 0.36 * w;
                b.m(Mat::Stainless)
                    .cylinder(c - Vec3::Z * half, c + Vec3::Z * half, cr, 28, false);
                for s in [-1.0, 1.0] {
                    b.m(Mat::Stainless).annulus(
                        c + Vec3::Z * half * s,
                        Vec3::Z * s,
                        cr,
                        0.4 * cr,
                        28,
                    );
                }
                b.m(Mat::Dark).cylinder(
                    c - Vec3::Z * half,
                    c + Vec3::Z * half,
                    0.4 * cr,
                    20,
                    false,
                );
                for s in [-1.0, 1.0] {
                    b.m(Mat::Timber).cuboid(
                        Vec3::new(x + s * cr * 0.8, deck + 0.09 * w, 0.0),
                        Vec3::new(0.14 * w, 0.06 * w, hw * 0.9),
                        Quat::from_rotation_z(-s * 0.6),
                    );
                }
                for z in [-0.18 * w, 0.18 * w] {
                    b.m(Mat::Dark).ring(
                        c + Vec3::Z * z,
                        Vec3::Z,
                        cr + 0.02 * w,
                        0.02 * w,
                        0.035 * w,
                        28,
                    );
                }
            }
        }
        VehicleKind::Crane => {
            let deck = frame_top + 0.28 * w;
            // Carrier deck and outrigger boxes.
            b.m(Mat::Yellow)
                .aabb(Vec3::new(tail, frame_top, -hw), Vec3::new(nose, deck, hw));
            for x in [tail + 0.5 * w, -0.9 * w.min(last * 0.4)] {
                b.m(Mat::Dark).aabb(
                    Vec3::new(x - 0.14 * w, frame_top - 0.2 * w, -hw - 0.02 * w),
                    Vec3::new(x + 0.14 * w, frame_top + 0.05 * w, hw + 0.02 * w),
                );
            }
            // Low driver's cab across the nose; the boom stows above it.
            cab(
                &mut b,
                Mat::Yellow,
                nose - 1.25 * w,
                nose,
                deck,
                deck + 0.72 * w,
                w,
            );
            // Slewing ring, superstructure, operator's cab, counterweight.
            let slew = Vec3::new(-last * 0.55, deck, 0.0);
            b.m(Mat::Dark)
                .cylinder(slew, slew + Vec3::Y * 0.12 * w, 0.4 * w, 20, true);
            let sup = deck + 0.12 * w;
            b.m(Mat::Yellow).aabb(
                Vec3::new(slew.x - 0.95 * w, sup, -0.3 * w),
                Vec3::new(slew.x + 0.5 * w, sup + 0.5 * w, 0.3 * w),
            );
            b.m(Mat::Dark).aabb(
                Vec3::new(slew.x - 1.2 * w, sup, -0.46 * w),
                Vec3::new(slew.x - 0.75 * w, sup + 0.6 * w, 0.46 * w),
            );
            b.m(Mat::Yellow).aabb(
                Vec3::new(slew.x - 0.1 * w, sup, 0.1 * w),
                Vec3::new(slew.x + 0.6 * w, sup + 0.75 * w, 0.48 * w),
            );
            b.m(Mat::Dark).aabb(
                Vec3::new(slew.x + 0.05 * w, sup + 0.3 * w, 0.485 * w),
                Vec3::new(slew.x + 0.5 * w, sup + 0.68 * w, 0.49 * w),
            );
            b.m(Mat::Dark).aabb(
                Vec3::new(slew.x + 0.6 * w, sup + 0.3 * w, 0.14 * w),
                Vec3::new(slew.x + 0.61 * w, sup + 0.68 * w, 0.44 * w),
            );
            // Telescopic boom stowed forward over the driver's cab, its
            // lift cylinder, and the hook block lashed to the nose.
            let foot = Vec3::new(slew.x - 0.65 * w, sup + 0.62 * w, -0.08 * w);
            let head = Vec3::new(nose + 0.5 * w, sup + 1.0 * w, -0.08 * w);
            let mid = foot.lerp(head, 0.62);
            b.m(Mat::Yellow).bar(foot, mid, 0.3 * w, 0.34 * w, Vec3::Y);
            b.m(Mat::LightSteel)
                .bar(mid, foot.lerp(head, 0.86), 0.24 * w, 0.27 * w, Vec3::Y);
            b.m(Mat::Yellow)
                .bar(foot.lerp(head, 0.86), head, 0.18 * w, 0.21 * w, Vec3::Y);
            b.m(Mat::Stainless).cylinder(
                Vec3::new(slew.x + 0.3 * w, sup + 0.4 * w, -0.08 * w),
                foot.lerp(head, 0.4) - Vec3::Y * 0.15 * w,
                0.06 * w,
                10,
                false,
            );
            b.m(Mat::Dark).cylinder(
                head - Vec3::Y * 0.1 * w,
                Vec3::new(head.x - 0.05 * w, deck + 0.35 * w, -0.08 * w),
                0.012 * w,
                6,
                false,
            );
            b.m(Mat::Yellow).aabb(
                Vec3::new(head.x - 0.15 * w, deck + 0.1 * w, -0.2 * w),
                Vec3::new(head.x + 0.05 * w, deck + 0.38 * w, 0.04 * w),
            );
            // Beacon.
            b.m(Mat::IndicatorAmber).cylinder(
                Vec3::new(slew.x + 0.25 * w, sup + 0.75 * w, 0.3 * w),
                Vec3::new(slew.x + 0.25 * w, sup + 0.85 * w, 0.3 * w),
                0.05 * w,
                10,
                true,
            );
        }
    }
    // Tail lamps: dim lenses on the body, bright ones that show when braking.
    let mut brake = Scenery::default();
    for z in [-0.36 * w, 0.36 * w] {
        b.m(Mat::PipeRed).aabb(
            Vec3::new(tail - 0.01 * w, 1.0 * r, z - 0.07 * w),
            Vec3::new(tail, 1.35 * r, z + 0.07 * w),
        );
        brake.m(Mat::IndicatorRed).aabb(
            Vec3::new(tail - 0.02 * w, 1.03 * r, z - 0.06 * w),
            Vec3::new(tail - 0.01 * w, 1.32 * r, z + 0.06 * w),
        );
    }

    let mut parts = vec![
        Part::driven(Role::Body, Vec3::ZERO, b),
        // The brake lamps ride on the body: same pivot, same motion.
        Part::driven(Role::BrakeLight, Vec3::ZERO, brake),
    ];
    for (n, &a) in axles.iter().enumerate() {
        // Rear axles carry twin tyres.
        let tyre = if n == 0 { 0.2 * w } else { 0.3 * w };
        parts.push(Part::local(
            Role::Wheel(n as u8),
            Transform::from_xyz(-a, r, 0.0),
            wheels(r, hw, tyre),
        ));
    }
    RigModel { parts }
}

/// A forward-control cab between `back` and `front`, from `bottom` to
/// `top`, `width` wide.
fn cab(s: &mut Scenery, colour: Mat, back: f32, front: f32, bottom: f32, top: f32, width: f32) {
    let hw = width * 0.5;
    let h = top - bottom;
    let belt = bottom + h * 0.48;
    s.m(colour)
        .aabb(Vec3::new(back, bottom, -hw), Vec3::new(front, belt, hw));
    // Upper cab, raked windscreen.
    s.m(colour).aabb(
        Vec3::new(back, belt, -hw),
        Vec3::new(front - h * 0.16, top, hw),
    );
    let lean = Quat::from_rotation_z(0.3);
    let glass = Vec3::new(front - h * 0.1, (belt + top) * 0.5, 0.0);
    s.m(colour)
        .cuboid(glass, Vec3::new(h * 0.03, (top - belt) * 0.55, hw), lean);
    s.m(Mat::Dark).cuboid(
        glass + lean * Vec3::X * h * 0.03,
        Vec3::new(h * 0.008, (top - belt) * 0.42, hw * 0.86),
        lean,
    );
    for z in [-hw - 0.004, hw + 0.004] {
        s.m(Mat::Dark).aabb(
            Vec3::new(back + h * 0.12, belt + h * 0.06, z - 0.004),
            Vec3::new(front - h * 0.28, top - h * 0.08, z + 0.004),
        );
        // Mirror.
        s.m(Mat::Dark).aabb(
            Vec3::new(
                front - h * 0.24,
                belt + h * 0.05,
                z + z.signum() * hw * 0.2 - 0.02,
            ),
            Vec3::new(
                front - h * 0.2,
                belt + h * 0.3,
                z + z.signum() * hw * 0.2 + 0.02,
            ),
        );
    }
    // Grille and headlamps.
    s.m(Mat::LightSteel).aabb(
        Vec3::new(front, bottom + h * 0.14, -hw * 0.55),
        Vec3::new(front + 0.01, belt - h * 0.08, hw * 0.55),
    );
    for z in [-hw * 0.78, hw * 0.78] {
        s.m(Mat::WindowWarm).aabb(
            Vec3::new(front, bottom + h * 0.1, z - hw * 0.14),
            Vec3::new(front + 0.012, bottom + h * 0.24, z + hw * 0.14),
        );
    }
    // Exhaust stack behind the cab.
    s.m(Mat::Stainless).cylinder(
        Vec3::new(back - h * 0.06, bottom, hw * 0.75),
        Vec3::new(back - h * 0.06, top + h * 0.08, hw * 0.75),
        h * 0.035,
        8,
        true,
    );
}

/// The wheels of one axle, about the axle centre: tyres, rims, and wheel
/// nuts and a paint mark so the rolling shows.
fn wheels(r: f32, half_width: f32, tyre: f32) -> Scenery {
    let mut s = Scenery::default();
    for side in [-1.0f32, 1.0] {
        let outer = Vec3::Z * side * half_width;
        let inner = Vec3::Z * side * (half_width - tyre);
        s.m(Mat::TableDark).cylinder(inner, outer, r, 24, false);
        s.m(Mat::TableDark)
            .annulus(outer, Vec3::Z * side, r, 0.6 * r, 24);
        s.m(Mat::TableDark)
            .annulus(inner, -Vec3::Z * side, r, 0.3 * r, 24);
        let face = outer - Vec3::Z * side * 0.04 * r;
        s.m(Mat::LightSteel).disc(face, Vec3::Z * side, 0.6 * r, 20);
        s.m(Mat::LightSteel)
            .cylinder(face, outer + Vec3::Z * side * 0.03 * r, 0.2 * r, 12, true);
        for k in 0..6 {
            let a = k as f32 / 6.0 * std::f32::consts::TAU;
            let p = face + Vec3::new(a.cos(), a.sin(), 0.0) * 0.38 * r;
            s.m(Mat::Dark)
                .cylinder(p, p + Vec3::Z * side * 0.05 * r, 0.05 * r, 6, true);
        }
        // Tyre-fitter's chalk mark.
        s.m(Mat::Yellow).cuboid(
            outer + Vec3::Z * side * 0.002 + Vec3::Y * 0.8 * r,
            Vec3::new(0.05 * r, 0.14 * r, 0.002),
            Quat::IDENTITY,
        );
    }
    // Axle.
    s.m(Mat::Dark).cylinder(
        -Vec3::Z * (half_width - tyre),
        Vec3::Z * (half_width - tyre),
        0.18 * r,
        8,
        false,
    );
    s
}

/// A vehicle in the scene: which entry of `RigLive::vehicles` it shows.
#[derive(Component, Clone, Debug)]
pub(crate) struct Truck {
    index: usize,
    name: String,
    axles: Vec<f32>,
    width: f32,
    /// Distance rolled, world units.
    rolled: f32,
}

/// Axle offsets behind the front axle, from the wheel contacts.
pub fn axle_offsets(v: &VehicleDrive) -> Vec<f32> {
    let Some(front) = v.wheels.first() else {
        return Vec::new();
    };
    let f = v.forward.normalize_or(Vec3::X);
    v.wheels
        .iter()
        .map(|w| (front.contact - w.contact).dot(f).max(0.0))
        .collect()
}

/// The pose of a vehicle whose front tyre touches at `front`, heading
/// along `forward` (pitch included).
pub fn pose(front: Vec3, forward: Vec3) -> Transform {
    let x = forward.normalize_or(Vec3::X);
    let z = x.cross(Vec3::Y).normalize_or(Vec3::Z);
    let y = z.cross(x);
    Transform::from_translation(front).with_rotation(Quat::from_mat3(&Mat3::from_cols(x, y, z)))
}

/// Spawns, moves and removes the vehicles to match `RigLive::vehicles`.
pub(crate) fn drive(
    mut commands: Commands,
    rigs: Res<LabRigs>,
    clock: Res<RigClock>,
    models: Res<RigModels>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut maker: MaterialMaker,
    cache: Option<ResMut<LabMaterials>>,
    mut trucks: Query<(Entity, &mut Truck, &mut Transform), Without<Role>>,
    mut parts: Query<(&Role, &Rest, &Owner, &mut Transform, &mut Visibility)>,
) {
    let Some(mut cache) = cache else { return };
    let live = &rigs.live.vehicles;
    let mut seen = vec![false; live.len()];
    for (e, mut truck, mut at) in &mut trucks {
        let v = live.get(truck.index).filter(|v| {
            v.name == truck.name && v.wheels.len() == truck.axles.len() && !v.wheels.is_empty()
        });
        let Some(v) = v else {
            commands.entity(e).despawn();
            continue;
        };
        seen[truck.index] = true;
        truck.rolled += v.speed * clock.dt;
        *at = pose(v.wheels[0].contact, v.forward);
    }
    for (i, v) in live.iter().enumerate() {
        if seen[i] || v.wheels.is_empty() {
            continue;
        }
        let kind = VehicleKind::from_name(&v.name);
        let axles = axle_offsets(v);
        let width = DEFAULT_WIDTH;
        let root = spawn_rig(
            &mut commands,
            &mut meshes,
            &mut maker,
            &mut cache,
            &models,
            RigSlot::Truck(kind),
            pose(v.wheels[0].contact, v.forward),
            || model(kind, &axles, width),
        );
        commands.entity(root).insert(Truck {
            index: i,
            name: v.name.clone(),
            axles,
            width,
            rolled: 0.0,
        });
    }

    for (role, rest, owner, mut at, mut vis) in &mut parts {
        let Ok((_, truck, root)) = trucks.get(owner.0) else {
            continue;
        };
        let Some(v) = live.get(truck.index) else {
            continue;
        };
        let r = wheel_radius(truck.width);
        match *role {
            Role::Wheel(n) => {
                let Some(wheel) = v.wheels.get(n as usize) else {
                    continue;
                };
                // The tyre sits where the load model says it presses, so
                // each axle follows the deck under it.
                let local = root.rotation.inverse() * (wheel.contact - root.translation);
                at.translation = Vec3::new(local.x, local.y + r, rest.0.translation.z);
                at.rotation = Quat::from_rotation_z(-truck.rolled / r);
            }
            Role::Body | Role::BrakeLight => {
                let front = v.wheels.first().map_or(0.0, |w| w.bounce);
                let rear = v.wheels.last().map_or(0.0, |w| w.bounce);
                let base = truck.axles.last().copied().unwrap_or(0.0).max(1e-3);
                let pitch = -((rear - front) / base).clamp(-0.5, 0.5).asin();
                at.translation = rest.0.translation + Vec3::Y * front;
                at.rotation = rest.0.rotation * Quat::from_rotation_z(pitch);
                if *role == Role::BrakeLight {
                    vis.set_if_neq(if v.braking {
                        Visibility::Inherited
                    } else {
                        Visibility::Hidden
                    });
                }
            }
            _ => {}
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rigs::WheelDrive;

    #[test]
    fn names_pick_the_model() {
        assert_eq!(VehicleKind::from_name("Loaded tipper"), VehicleKind::Dump);
        assert_eq!(
            VehicleKind::from_name("Coil hauler 2"),
            VehicleKind::CoilHauler
        );
        assert_eq!(VehicleKind::from_name("Mobile CRANE"), VehicleKind::Crane);
        assert_eq!(VehicleKind::from_name("Delivery truck"), VehicleKind::Box);
    }

    #[test]
    fn every_kind_has_a_wheel_set_per_axle_and_stands_on_the_ground() {
        let axles = [0.0, 3.4, 4.7];
        for kind in VehicleKind::ALL {
            let m = model(kind, &axles, DEFAULT_WIDTH);
            let wheels = m
                .parts
                .iter()
                .filter(|p| matches!(p.role, Some(Role::Wheel(_))))
                .count();
            assert_eq!(wheels, axles.len(), "{kind:?}");
            assert!(m.triangles() > 500, "{kind:?}");
            let all = m.merged();
            let lo = all
                .bufs
                .values()
                .flat_map(|b| b.positions.iter())
                .map(|p| p[1])
                .fold(f32::MAX, f32::min);
            assert!(lo.abs() < 1e-4, "{kind:?} lowest point {lo}");
        }
    }

    #[test]
    fn axles_come_from_the_wheel_contacts() {
        let v = VehicleDrive {
            forward: Vec3::X,
            wheels: [2.0, -1.2, -2.5]
                .iter()
                .map(|&x| WheelDrive {
                    contact: Vec3::new(x, 0.0, 0.3),
                    ..Default::default()
                })
                .collect(),
            ..Default::default()
        };
        let a = axle_offsets(&v);
        assert!((a[1] - 3.2).abs() < 1e-5 && (a[2] - 4.5).abs() < 1e-5);
    }

    #[test]
    fn pose_keeps_the_vehicle_upright_and_on_its_heading() {
        let f = Vec3::new(1.0, -0.1, 0.5).normalize();
        let t = pose(Vec3::new(1.0, 2.0, 3.0), f);
        assert!((t.rotation * Vec3::X).distance(f) < 1e-5);
        assert!((t.rotation * Vec3::Y).y > 0.9);
        // No roll: the axle stays level.
        assert!((t.rotation * Vec3::Z).y.abs() < 1e-5);
    }
}
