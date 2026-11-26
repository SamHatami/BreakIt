//! The fan wall: a 5 by 4 array of ducted fans on a cart, with a smoke
//! rake and a wire of wool tufts in front of it and a control console
//! beside it.
//!
//! Rig frame: origin at table-top level under the middle of the outlet
//! face, +x downstream, +y up. The cart reaches down to the hall floor.

use std::f32::consts::{FRAC_PI_2, TAU};

use bevy::asset::RenderAssetUsages;
use bevy::light::NotShadowCaster;
use bevy::mesh::{Indices, PrimitiveTopology};
use bevy::prelude::*;

use super::{
    park, send, spawn_rig, yaw_to, FanSetup, LabRigs, Owner, Part, Rest, Rig, RigClock,
    RigMaterials, RigModel, RigModels, RigSlot, Role, Site,
};
use crate::palette::Mat;
use crate::scenery::{Rng, Scenery};
use crate::{LabMaterials, MaterialMaker};

pub const COLS: usize = 5;
pub const ROWS: usize = 4;
/// Pitch of the fans, world units.
pub const CELL: f32 = 2.8;
/// Bottom of the array above the table top.
pub const BASE: f32 = -0.3;
/// Clear distance from the table rim to the outlet face.
pub const STANDOFF: f32 = 2.6;
/// Number of tufts on the wire.
pub const TUFTS: usize = 13;

/// Centre of fan `i` on the outlet plane.
pub fn fan_centre(i: usize) -> Vec3 {
    let (c, r) = (i % COLS, i / COLS);
    Vec3::new(
        0.0,
        BASE + (r as f32 + 0.5) * CELL,
        (c as f32 - (COLS as f32 - 1.0) * 0.5) * CELL,
    )
}

/// The procedural fan wall. `floor` is the hall floor in the rig frame.
pub fn model(floor: f32) -> RigModel {
    let mut fixed = Scenery::default();
    let half_w = COLS as f32 * CELL * 0.5;
    let top = BASE + ROWS as f32 * CELL;
    let depth = 1.7;

    // Housing: a dark plenum with a yellow frame between the fans.
    fixed.m(Mat::Dark).aabb(
        Vec3::new(-depth + 0.1, BASE + 0.05, -half_w + 0.05),
        Vec3::new(-0.75, top - 0.05, half_w - 0.05),
    );
    let bar = 0.22;
    for r in 0..=ROWS {
        let y = BASE + r as f32 * CELL;
        fixed.m(Mat::Yellow).aabb(
            Vec3::new(-depth, y - bar * 0.5, -half_w - bar * 0.5),
            Vec3::new(0.0, y + bar * 0.5, half_w + bar * 0.5),
        );
    }
    for c in 0..=COLS {
        let z = -half_w + c as f32 * CELL;
        fixed.m(Mat::Yellow).aabb(
            Vec3::new(-depth, BASE, z - bar * 0.5),
            Vec3::new(0.0, top, z + bar * 0.5),
        );
    }
    // Side and top sheets.
    for z in [-half_w - bar * 0.5, half_w + bar * 0.5] {
        fixed.m(Mat::MachineGrey).aabb(
            Vec3::new(-depth, BASE, z - 0.03),
            Vec3::new(-0.1, top, z + 0.03),
        );
    }
    fixed.m(Mat::MachineGrey).aabb(
        Vec3::new(-depth, top + bar * 0.5, -half_w),
        Vec3::new(-0.1, top + bar * 0.5 + 0.05, half_w),
    );

    let mut parts = Vec::new();
    for i in 0..COLS * ROWS {
        let c = fan_centre(i);
        // Duct, guard grille, motor.
        fixed
            .m(Mat::LightSteel)
            .ring(c - Vec3::X * 0.4, Vec3::X, CELL * 0.5 - 0.14, 0.07, 0.8, 32);
        for k in 1..=4 {
            fixed.m(Mat::LightSteel).ring(
                c - Vec3::X * 0.03,
                Vec3::X,
                (CELL * 0.5 - 0.16) * k as f32 / 4.0,
                0.035,
                0.035,
                28,
            );
        }
        for k in 0..8 {
            let a = k as f32 / 8.0 * TAU;
            let d = Vec3::new(0.0, a.cos(), a.sin());
            fixed.m(Mat::LightSteel).bar(
                c - Vec3::X * 0.03 + d * 0.12,
                c - Vec3::X * 0.03 + d * (CELL * 0.5 - 0.16),
                0.03,
                0.03,
                Vec3::X,
            );
        }
        fixed.m(Mat::MachineBlue).cylinder(
            c - Vec3::X * (depth + 0.55),
            c - Vec3::X * (depth - 0.2),
            0.42,
            16,
            true,
        );
        fixed.m(Mat::Dark).cylinder(
            c - Vec3::X * (depth + 0.7),
            c - Vec3::X * (depth + 0.55),
            0.3,
            12,
            true,
        );

        // Rotor: hub, spinner and five pitched blades.
        let pivot = c - Vec3::X * 0.5;
        let mut rotor = Scenery::default();
        rotor.m(Mat::Dark).cylinder(
            pivot - Vec3::X * 0.16,
            pivot + Vec3::X * 0.12,
            0.3,
            16,
            true,
        );
        rotor
            .m(Mat::Dark)
            .sphere(pivot + Vec3::X * 0.1, 0.24, 12, false);
        for b in 0..5 {
            let a = b as f32 / 5.0 * TAU + i as f32 * 0.37;
            let around = Quat::from_rotation_x(a);
            let pitch = Quat::from_rotation_y(0.6);
            rotor.m(Mat::LightSteel).cuboid(
                pivot + around * Vec3::new(0.0, 0.72, 0.0),
                Vec3::new(0.02, 0.48, 0.2),
                around * pitch,
            );
        }
        parts.push(Part::driven(Role::Rotor(i as u8), pivot, rotor));
    }

    // Stand: columns down to the cart, braced.
    let deck = floor + 2.4;
    let (xb, xf) = (-depth + 0.15, -0.25);
    for z in [-half_w + 0.3, 0.0, half_w - 0.3] {
        for x in [xb, xf] {
            fixed.m(Mat::FrameSteel).aabb(
                Vec3::new(x - 0.3, deck, z - 0.3),
                Vec3::new(x + 0.3, BASE, z + 0.3),
            );
        }
        fixed.m(Mat::FrameSteel).bar(
            Vec3::new(-3.2, deck + 0.2, z),
            Vec3::new(xb, BASE - 1.0, z),
            0.2,
            0.2,
            Vec3::Y,
        );
    }
    for x in [xb, xf] {
        for (za, zb) in [(-half_w + 0.3, 0.0), (0.0, half_w - 0.3)] {
            fixed.m(Mat::LightSteel).bar(
                Vec3::new(x, deck + 0.3, za),
                Vec3::new(x, BASE - 0.3, zb),
                0.1,
                0.1,
                Vec3::X,
            );
            fixed.m(Mat::LightSteel).bar(
                Vec3::new(x, BASE - 0.3, za),
                Vec3::new(x, deck + 0.3, zb),
                0.1,
                0.1,
                Vec3::X,
            );
        }
    }
    // Cart: deck, ballast, drive cabinet, cable reel, castors, push bar.
    fixed.m(Mat::MachineGrey).aabb(
        Vec3::new(-3.6, deck - 0.5, -half_w - 0.3),
        Vec3::new(0.4, deck, half_w + 0.3),
    );
    fixed.m(Mat::Yellow).aabb(
        Vec3::new(-3.65, deck - 0.5, -half_w - 0.35),
        Vec3::new(0.45, deck - 0.38, half_w + 0.35),
    );
    for z in [-half_w + 1.5, half_w - 1.5] {
        fixed.m(Mat::Concrete).aabb(
            Vec3::new(-3.3, deck, z - 1.1),
            Vec3::new(-2.1, deck + 1.0, z + 1.1),
        );
    }
    fixed.m(Mat::MachineGreen).aabb(
        Vec3::new(-3.3, deck, -1.2),
        Vec3::new(-2.3, deck + 3.2, 1.2),
    );
    fixed.m(Mat::Dark).cylinder(
        Vec3::new(-2.9, deck + 0.9, 2.2),
        Vec3::new(-2.9, deck + 0.9, 3.0),
        0.85,
        20,
        true,
    );
    for x in [-3.1, -1.6, -0.1] {
        for z in [-half_w + 0.3, half_w - 0.3] {
            fixed.m(Mat::Dark).cylinder(
                Vec3::new(x, floor + 0.9, z - 0.3),
                Vec3::new(x, floor + 0.9, z + 0.3),
                0.9,
                16,
                true,
            );
            fixed.m(Mat::LightSteel).aabb(
                Vec3::new(x - 0.2, floor + 0.9, z - 0.4),
                Vec3::new(x + 0.2, deck - 0.5, z + 0.4),
            );
        }
    }
    for z in [-half_w + 0.6, half_w - 0.6] {
        fixed.m(Mat::LightSteel).cylinder(
            Vec3::new(-3.5, deck, z),
            Vec3::new(-4.3, deck + 2.4, z),
            0.08,
            8,
            false,
        );
    }
    fixed.m(Mat::LightSteel).cylinder(
        Vec3::new(-4.3, deck + 2.4, -half_w + 0.6),
        Vec3::new(-4.3, deck + 2.4, half_w - 0.6),
        0.08,
        8,
        true,
    );

    // Smoke rake: four manifolds with a nozzle per streamer.
    let rake_x = 0.75;
    for z in [-half_w + 0.4, half_w - 0.4] {
        fixed.m(Mat::Stainless).cylinder(
            Vec3::new(rake_x, BASE - 0.2, z),
            Vec3::new(rake_x, top - 0.6, z),
            0.06,
            8,
            true,
        );
        fixed.m(Mat::Stainless).cylinder(
            Vec3::new(0.0, BASE - 0.2, z),
            Vec3::new(rake_x, BASE - 0.2, z),
            0.06,
            8,
            false,
        );
    }
    for y in streamer_heights() {
        fixed.m(Mat::Stainless).cylinder(
            Vec3::new(rake_x, y, -half_w + 0.4),
            Vec3::new(rake_x, y, half_w - 0.4),
            0.045,
            8,
            false,
        );
    }
    for p in streamer_starts() {
        fixed
            .m(Mat::Stainless)
            .frustum(Vec3::new(rake_x, p.y, p.z), p, 0.05, 0.02, 8, false);
    }

    // Tuft wire between two arms, just short of the table rim.
    let (wx, wy) = (STANDOFF - 0.5, 1.3);
    for z in [-half_w + 0.2, half_w - 0.2] {
        fixed.m(Mat::LightSteel).bar(
            Vec3::new(0.0, BASE + 0.1, z),
            Vec3::new(wx, wy, z),
            0.05,
            0.05,
            Vec3::Y,
        );
    }
    fixed.m(Mat::LightSteel).cylinder(
        Vec3::new(wx, wy, -half_w + 0.2),
        Vec3::new(wx, wy, half_w - 0.2),
        0.012,
        6,
        false,
    );
    for k in 0..TUFTS {
        let z = (k as f32 / (TUFTS - 1) as f32 - 0.5) * (2.0 * half_w - 1.2);
        let pivot = Vec3::new(wx, wy, z);
        let mut tuft = Scenery::default();
        tuft.m(Mat::PipeRed).aabb(
            pivot + Vec3::new(-0.012, -0.75, -0.02),
            pivot + Vec3::new(0.012, 0.0, 0.02),
        );
        parts.push(Part::driven(Role::Tuft(k as u8), pivot, tuft));
    }

    // Running / standby beacons on top of the housing.
    let beacon = Vec3::new(-depth * 0.5, top + bar * 0.5 + 0.05, half_w - 0.8);
    fixed
        .m(Mat::Dark)
        .cylinder(beacon, beacon + Vec3::Y * 0.25, 0.16, 10, true);
    for (role, mat) in [
        (Role::LampOn, Mat::IndicatorGreen),
        (Role::LampOff, Mat::IndicatorAmber),
    ] {
        let mut lamp = Scenery::default();
        lamp.m(mat).cylinder(
            beacon + Vec3::Y * 0.25,
            beacon + Vec3::Y * 0.55,
            0.13,
            10,
            true,
        );
        parts.push(Part::driven(role, beacon, lamp));
    }

    // On the far side from where the player usually looks from.
    console(&mut fixed, floor, Vec3::new(-1.0, 0.0, -(half_w + 7.5)));
    parts.push(Part::fixed(fixed));
    RigModel { parts }
}

/// A pedestal console with a screen, a speed dial and an emergency stop.
/// Real size: about 0.5 m square and 1 m high.
fn console(s: &mut Scenery, floor: f32, at: Vec3) {
    let base = Vec3::new(at.x, floor, at.z);
    let h = 19.0;
    s.m(Mat::MachineGrey).aabb(
        base + Vec3::new(-3.5, 0.0, -4.5),
        base + Vec3::new(3.5, h, 4.5),
    );
    s.m(Mat::Dark).aabb(
        base + Vec3::new(-3.9, 0.0, -4.9),
        base + Vec3::new(3.9, 1.2, 4.9),
    );
    // Sloped desk facing downstream, where the operator can see the model.
    let tilt = Quat::from_rotation_z(-0.45);
    let desk = base + Vec3::new(0.8, h + 1.6, 0.0);
    s.m(Mat::MachineGrey)
        .cuboid(desk, Vec3::new(4.2, 0.9, 5.0), tilt);
    let up = tilt * Vec3::Y;
    let along = tilt * Vec3::X;
    s.m(Mat::Dark).cuboid(
        desk + up * 0.92 - along * 0.6 - Vec3::Z * 1.4,
        Vec3::new(2.6, 0.04, 2.8),
        tilt,
    );
    for k in 0..3 {
        s.m(Mat::IndicatorGreen).cuboid(
            desk + up * 0.98 - along * (0.6 + k as f32 * 0.8 - 0.8) - Vec3::Z * 1.4,
            Vec3::new(0.05, 0.01, 2.0 - k as f32 * 0.5),
            tilt,
        );
    }
    // Emergency stop: yellow collar, red mushroom.
    let stop = desk + up * 0.9 + Vec3::Z * 3.2;
    s.m(Mat::Yellow)
        .cylinder(stop, stop + up * 0.35, 1.0, 16, true);
    s.m(Mat::PipeRed)
        .cylinder(stop + up * 0.35, stop + up * 0.8, 0.7, 16, true);
    s.m(Mat::PipeRed).sphere(stop + up * 0.8, 0.7, 14, true);
    // Speed dial and a key switch.
    let dial = desk + up * 0.9 + along * 2.6 - Vec3::Z * 1.0;
    s.m(Mat::Dark)
        .cylinder(dial, dial + up * 0.5, 0.6, 14, true);
    let key = desk + up * 0.9 + along * 2.6 + Vec3::Z * 1.2;
    s.m(Mat::Stainless)
        .cylinder(key, key + up * 0.3, 0.35, 12, true);
    // Cable to the cart along the floor.
    s.m(Mat::Dark).bar(
        base + Vec3::new(0.0, 0.2, 4.5),
        Vec3::new(-2.0, floor + 0.2, at.z + 8.5),
        0.35,
        0.35,
        Vec3::Y,
    );
}

fn streamer_heights() -> [f32; 4] {
    [BASE + 1.3, BASE + 3.9, BASE + 6.5, BASE + 9.1]
}

/// Nozzle tips of the smoke rake, rig frame.
pub fn streamer_starts() -> Vec<Vec3> {
    let mut out = Vec::new();
    for (r, y) in streamer_heights().into_iter().enumerate() {
        let n = 9;
        for k in 0..n {
            // Rows are staggered so the streamers do not hide each other.
            let z = (k as f32 - (n as f32 - 1.0) * 0.5) * 1.5 + if r % 2 == 1 { 0.4 } else { 0.0 };
            out.push(Vec3::new(1.05, y, z));
        }
    }
    out
}

/// Where the fan wall stands to blow along `direction` over a table of
/// half size `half` (rim included).
pub fn working_pose(direction: Vec3, half: Vec2) -> Transform {
    let d = Vec3::new(direction.x, 0.0, direction.z).normalize_or(Vec3::X);
    let reach = d.x.abs() * half.x + d.z.abs() * half.y;
    Transform::from_translation(-d * (reach + STANDOFF)).with_rotation(yaw_to(d))
}

/// The streamers' mesh entity.
#[derive(Component)]
pub(crate) struct Streamers {
    /// Per streamer: phases of its wander.
    phases: Vec<[f32; 4]>,
    starts: Vec<Vec3>,
}

/// Points along a streamer.
const STREAMER_POINTS: usize = 64;
/// Spacing of the points, world units.
const STREAMER_STEP: f32 = 0.5;

/// Keeps the fan wall in the scene and sends it to the table or the park
/// as the setup changes.
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
    mut placed: Local<Option<(Option<FanSetup>, Vec2)>>,
) {
    let Some(mut cache) = cache else { return };
    let half = site.table_half();
    let park = park::fan_pose();
    let found = existing.iter().find(|(_, r, _)| r.0 == RigSlot::FanWall);
    let (root, at) = match found {
        Some((e, _, t)) => (e, *t),
        None => {
            let floor = site.floor();
            let root = spawn_rig(
                &mut commands,
                &mut meshes,
                &mut maker,
                &mut cache,
                &models,
                RigSlot::FanWall,
                park,
                || model(floor),
            );
            spawn_effects(&mut commands, &mut meshes, &extra, root);
            *placed = Some((None, half));
            (root, park)
        }
    };
    let want = (rigs.setup.fan, half);
    if *placed == Some(want) {
        return;
    }
    *placed = Some(want);
    let to = match rigs.setup.fan {
        Some(f) => working_pose(f.direction, half),
        None => park,
    };
    send(&mut commands, root, at, to, 0.0);
}

/// Blur discs and the smoke streamers: effects, so they are there with a
/// hero model too.
fn spawn_effects(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    extra: &RigMaterials,
    root: Entity,
) {
    let disc = meshes.add(Circle::new(CELL * 0.5 - 0.2));
    for i in 0..COLS * ROWS {
        let at = Transform::from_translation(fan_centre(i) - Vec3::X * 0.3)
            .with_rotation(Quat::from_rotation_y(FRAC_PI_2));
        commands.spawn((
            Mesh3d(disc.clone()),
            MeshMaterial3d(extra.blur.clone()),
            at,
            Role::Blur(i as u8),
            Rest(at),
            NotShadowCaster,
            ChildOf(root),
        ));
    }
    let starts = streamer_starts();
    let mut rng = Rng::new(0x51DE_CA57);
    let phases = starts
        .iter()
        .map(|_| std::array::from_fn(|_| rng.range(0.0, TAU)))
        .collect();
    let n = starts.len();
    let verts = n * STREAMER_POINTS * 4;
    let mut indices = Vec::new();
    for j in 0..n {
        for k in 0..STREAMER_POINTS - 1 {
            let a = ((j * STREAMER_POINTS + k) * 4) as u32;
            for strip in [0, 2] {
                let (p, q) = (a + strip, a + strip + 4);
                indices.extend_from_slice(&[p, p + 1, q, p + 1, q + 1, q]);
            }
        }
    }
    let mesh = Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::default(),
    )
    .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, vec![[0.0f32; 3]; verts])
    .with_inserted_attribute(Mesh::ATTRIBUTE_NORMAL, vec![[0.0f32, 1.0, 0.0]; verts])
    .with_inserted_attribute(Mesh::ATTRIBUTE_COLOR, vec![[1.0f32, 1.0, 1.0, 0.0]; verts])
    .with_inserted_indices(Indices::U32(indices));
    commands.spawn((
        Mesh3d(meshes.add(mesh)),
        MeshMaterial3d(extra.smoke.clone()),
        Transform::IDENTITY,
        Visibility::Hidden,
        // The mesh is rewritten every frame; its bounds would be stale.
        bevy::camera::visibility::NoFrustumCulling,
        NotShadowCaster,
        Streamers { phases, starts },
        ChildOf(root),
    ));
}

/// What the fans are doing, smoothed: rotors have inertia and smoke hangs
/// in the air for a moment.
#[derive(Default)]
pub(crate) struct FanMotion {
    /// Rotor speed, rad/s.
    spin: f32,
    angle: f32,
    /// Smoke density 0..1.
    smoke: f32,
}

/// Rotor speed at full drive, rad/s. Slower than a real fan on purpose:
/// above a few turns a second the blades only alias; the blur disc takes
/// over from there.
const FULL_SPIN: f32 = 26.0;

pub(crate) fn drive(
    rigs: Res<LabRigs>,
    clock: Res<RigClock>,
    extra: Res<RigMaterials>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    owners: Query<&Rig>,
    mut parts: Query<(&Role, &Rest, &Owner, &mut Transform, &mut Visibility)>,
    mut motion: Local<FanMotion>,
) {
    let wind = rigs.live.wind.unwrap_or_default();
    let drive = wind.fan_drive.clamp(0.0, 1.0);
    let dt = clock.dt;
    motion.spin += (drive * FULL_SPIN - motion.spin) * (1.0 - (-dt / 0.7).exp());
    motion.angle = (motion.angle + motion.spin * dt) % (TAU * 64.0);
    let level = motion.spin / FULL_SPIN;
    if let Some(mut m) = materials.get_mut(&extra.blur) {
        let alpha = 0.5 * (level * 1.6).min(1.0).powi(2);
        if (m.base_color.alpha() - alpha).abs() > 1e-3 {
            m.base_color.set_alpha(alpha);
        }
    }
    let running = drive > 0.02;
    let t = rigs.live.time;
    for (role, rest, owner, mut at, mut vis) in &mut parts {
        if owners.get(owner.0).map(|r| r.0) != Ok(RigSlot::FanWall) {
            continue;
        }
        match *role {
            Role::Rotor(i) => {
                // Alternate fans are a touch out of step, as real ones are.
                let a = motion.angle * (1.0 + 0.013 * (i % 4) as f32);
                at.rotation = rest.0.rotation * Quat::from_rotation_x(a);
            }
            Role::Tuft(i) => {
                let k = i as f32;
                let lift = (wind.speed * wind.speed / 40.0).atan();
                let flutter = (wind.speed / 6.0).min(1.0)
                    * (0.1 * (t * (9.0 + k * 0.7) + k * 2.1).sin()
                        + 0.06 * (t * (17.0 + k) + k).sin());
                at.rotation = rest.0.rotation * Quat::from_rotation_z(lift + flutter);
            }
            Role::LampOn => {
                vis.set_if_neq(show(running));
            }
            Role::LampOff => {
                vis.set_if_neq(show(!running));
            }
            _ => {}
        }
    }
}

fn show(on: bool) -> Visibility {
    if on {
        Visibility::Inherited
    } else {
        Visibility::Hidden
    }
}

/// Rewrites the smoke streamers: lines of smoke carried downstream at the
/// wind speed, wandering more the further they have travelled.
pub(crate) fn streamers(
    rigs: Res<LabRigs>,
    clock: Res<RigClock>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut q: Query<(&Streamers, &Mesh3d, &mut Visibility)>,
    mut motion: Local<FanMotion>,
) {
    let Ok((streamers, mesh, mut vis)) = q.single_mut() else {
        return;
    };
    let wind = rigs.live.wind.unwrap_or_default();
    let target = if wind.fan_drive > 0.05 { 1.0 } else { 0.0 };
    motion.smoke += (target - motion.smoke) * (1.0 - (-clock.dt / 0.9).exp());
    if motion.smoke < 0.01 {
        vis.set_if_neq(Visibility::Hidden);
        return;
    }
    vis.set_if_neq(Visibility::Inherited);
    let Some(mut mesh) = meshes.get_mut(&mesh.0) else {
        return;
    };
    let n = streamers.starts.len() * STREAMER_POINTS * 4;
    let mut positions = Vec::with_capacity(n);
    let mut colors = Vec::with_capacity(n);
    let length = (STREAMER_POINTS - 1) as f32 * STREAMER_STEP;
    let travelled = wind.speed.max(2.0) * rigs.live.time;
    let gusty = 1.0 + (wind.gust.abs() / wind.speed.max(1.0)).min(1.0) * 2.0;
    for (start, ph) in streamers.starts.iter().zip(&streamers.phases) {
        for k in 0..STREAMER_POINTS {
            let s = k as f32 * STREAMER_STEP;
            // The pattern is frozen into the air and carried along.
            let u = s - travelled;
            let wander = 0.005 * s * gusty;
            let dy = wander * ((0.9 * u + ph[0]).sin() + 0.5 * (2.3 * u + ph[1]).sin());
            let dz = wander * ((0.7 * u + ph[2]).sin() + 0.5 * (1.9 * u + ph[3]).sin());
            let c = *start + Vec3::new(s, dy, dz);
            let w = 0.018 + 0.0035 * s;
            let alpha =
                0.42 * motion.smoke * (s / 1.5).min(1.0) * (1.0 - s / length).max(0.0).powf(1.5);
            for off in [Vec3::Y * w, -Vec3::Y * w, Vec3::Z * w, -Vec3::Z * w] {
                positions.push((c + off).to_array());
                colors.push([1.0, 1.0, 1.0, alpha]);
            }
        }
    }
    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions);
    mesh.insert_attribute(Mesh::ATTRIBUTE_COLOR, colors);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_wall_stands_upstream_and_blows_at_the_table() {
        let half = Vec2::new(14.35, 8.35);
        for d in [Vec3::X, Vec3::NEG_Z, Vec3::new(-1.0, 0.0, 0.0)] {
            let pose = working_pose(d, half);
            // Outlet face clear of the rim, on the upstream side.
            assert!(pose.translation.dot(d) < 0.0);
            let reach = d.x.abs() * half.x + d.z.abs() * half.y;
            assert!((-pose.translation.dot(d) - reach - STANDOFF).abs() < 1e-4);
            assert!((pose.rotation * Vec3::X).distance(d) < 1e-5);
        }
    }

    #[test]
    fn the_cart_stands_on_the_floor_and_the_fans_are_above_the_table() {
        let floor = -17.2;
        let m = model(floor);
        assert_eq!(
            m.parts
                .iter()
                .filter(|p| matches!(p.role, Some(Role::Rotor(_))))
                .count(),
            COLS * ROWS
        );
        let all = m.merged();
        let (mut lo, mut downstream) = (f32::MAX, f32::MIN);
        for p in all.bufs.values().flat_map(|b| b.positions.iter()) {
            lo = lo.min(p[1]);
            // The console stands beside the table, clear of its end.
            if p[2].abs() < 8.4 {
                downstream = downstream.max(p[0]);
            } else {
                assert!(p[2] < -9.0, "{p:?} is neither wall nor console");
            }
        }
        assert!((lo - floor).abs() < 1e-3, "lowest point {lo}");
        // Nothing in front of the table reaches its rim.
        assert!(downstream < STANDOFF);
        assert!(fan_centre(0).y > 0.0);
    }
}
