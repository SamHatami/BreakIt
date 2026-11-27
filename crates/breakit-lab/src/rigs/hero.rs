//! The hero models in `assets/props/`: where each one goes and which of
//! its nodes move.
//!
//! The files are built into the crate, so a release needs no asset folder.
//! To change a model, overwrite its file (same name, same node names and
//! frame) and rebuild. What the code relies on in each file is written
//! down here and in `docs/rigs.md`:
//!
//! - Units are metres, +y up, the model stands on y = 0.
//! - `industrial_fan_wall`: blows along +z, rotors `fan_rotor_1..9` spin
//!   about their z. Real lab hardware: drawn `model_scale` times larger.
//! - `hydraulic_actuator`: `barrel` along +y from its base pin at the
//!   origin, `piston_rod` slides along y. Fitted to the stroke it has to
//!   cover at the shake platform.
//! - `shake_table`: `table_platen` rides the table motion over
//!   `shake_table_base`. Fitted to the platform of the table insert.
//! - `model_dump_truck`, `model_coil_hauler`, `model_mobile_crane`: +x
//!   forward, `chassis` with the wheels as children, wheels turn about z.
//!   Fitted to the axles of the vehicle they show.
//! - `overhead_gantry_crane`: spans along x, stands on the hall floor.

use std::f32::consts::FRAC_PI_2;
use std::path::{Path, PathBuf};

use bevy::asset::io::embedded::EmbeddedAssetRegistry;
use bevy::camera::visibility::RenderLayers;
use bevy::prelude::*;
use bevy::world_serialization::WorldAssetRoot;

use super::{RigModels, RigSlot, Role, VehicleKind};
use crate::{LabSettings, HALL_LAYER};

macro_rules! prop {
    ($stem:literal) => {
        (
            $stem,
            include_bytes!(concat!(
                env!("CARGO_MANIFEST_DIR"),
                "/../../assets/props/",
                $stem,
                ".glb"
            )) as &'static [u8],
        )
    };
}

/// File stem and contents of every model.
const FILES: [(&str, &[u8]); 7] = [
    prop!("industrial_fan_wall"),
    prop!("hydraulic_actuator"),
    prop!("shake_table"),
    prop!("model_dump_truck"),
    prop!("model_coil_hauler"),
    prop!("model_mobile_crane"),
    prop!("overhead_gantry_crane"),
];

/// File stem of the gantry crane: a prop of the hall, not a rig.
pub const GANTRY: &str = "overhead_gantry_crane";

/// Whether the hero models are used. Insert `HeroProps(false)` before the
/// lab starts to get the procedural rigs.
#[derive(Resource, Clone, Copy, Debug, PartialEq, Eq)]
pub struct HeroProps(pub bool);

impl Default for HeroProps {
    fn default() -> Self {
        Self(true)
    }
}

fn scene_path(stem: &str) -> String {
    format!("embedded://breakit_lab/props/{stem}.glb#Scene0")
}

/// Registers the model files as embedded assets.
pub(crate) fn embed(app: &mut App) {
    // No asset server (a headless test): nothing to load them with.
    let Some(registry) = app.world().get_resource::<EmbeddedAssetRegistry>() else {
        return;
    };
    for (stem, bytes) in FILES {
        registry.insert_asset(
            PathBuf::from(format!("assets/props/{stem}.glb")),
            Path::new(&format!("breakit_lab/props/{stem}.glb")),
            bytes,
        );
    }
}

/// Loads the model of every slot that has one.
pub(crate) fn load(
    on: Res<HeroProps>,
    assets: Option<Res<AssetServer>>,
    mut models: ResMut<RigModels>,
) {
    let Some(assets) = assets.filter(|_| on.0) else {
        return;
    };
    for slot in RigSlot::ALL {
        let stem = slot.file_stem();
        if FILES.iter().any(|(s, _)| *s == stem) {
            models
                .0
                .entry(slot)
                .or_insert_with(|| assets.load(scene_path(stem)));
        }
    }
}

/// The role and axis of the node called `name` in the model of `slot`,
/// for the node names the files in `assets/props/` use.
pub fn alias(slot: RigSlot, name: &str) -> Option<(Role, Vec3)> {
    match slot {
        RigSlot::FanWall => {
            let n: u8 = name.strip_prefix("fan_rotor_")?.parse().ok()?;
            Some((Role::Rotor(n.checked_sub(1)?), Vec3::Z))
        }
        RigSlot::ShakeActuators => (name == "piston_rod").then_some((Role::Rod(0), Vec3::Y)),
        RigSlot::ShakeTable => (name == "table_platen").then_some((Role::Platform, Vec3::Y)),
        RigSlot::Truck(kind) => {
            if name == "chassis" {
                return Some((Role::Body, Vec3::Z));
            }
            let axle = match name {
                "wheel_fl" | "wheel_fr" | "wheel_front_left" | "wheel_front_right" => 0,
                "wheel_rl" | "wheel_rr" | "wheel_rear_left_1" | "wheel_rear_right_1" => 1,
                "wheel_rear_left_2" | "wheel_rear_right_2" => 2,
                _ => return None,
            };
            let axles = TruckSpec::of(kind)?.axles.len();
            (axle < axles).then_some((Role::Wheel(axle as u8), Vec3::Z))
        }
        _ => None,
    }
}

/// The fan wall model, in its own metres.
pub struct FanSpec;

impl FanSpec {
    /// z of the outlet grille.
    pub const OUTLET: f32 = 0.2;
    /// x of the fan columns and y of the rows.
    pub const COLS: [f32; 3] = [-0.8, 0.0, 0.8];
    pub const ROWS: [f32; 3] = [0.66, 1.46, 2.26];
    /// z of the rotor plane and the blade tip radius.
    pub const ROTOR_Z: f32 = -0.08;
    pub const RADIUS: f32 = 0.315;
    /// Half width of the array of ducts and the top of the frame.
    pub const HALF_WIDTH: f32 = 1.16;
    pub const TOP: f32 = 2.7;
    /// How far the cart reaches behind the origin (the push handle), and
    /// how far the middle of its footprint is off the origin along x (the
    /// control cabinet is on one side).
    pub const BACK: f32 = 0.52;
    pub const OFF_CENTRE: f32 = 0.08;

    /// The model's frame in the rig's frame (origin at table height in the
    /// middle of the outlet face, +x downstream) for a hall drawn `scale`
    /// times larger with its floor at `floor`.
    pub fn fit(scale: f32, floor: f32) -> Transform {
        Transform {
            translation: Vec3::new(-Self::OUTLET * scale, floor, 0.0),
            // The model blows along +z.
            rotation: Quat::from_rotation_y(FRAC_PI_2),
            scale: Vec3::splat(scale),
        }
    }
}

/// The actuator model, in its own metres.
pub struct ActuatorSpec;

impl ActuatorSpec {
    /// Base pin to rod-end pin along y with the rod at mid stroke.
    pub const LENGTH: f32 = 1.5;

    /// The model's frame for an actuator whose base pin is at `base` and
    /// that pushes along `axis` (horizontal, unit) over `length`.
    pub fn fit(base: Vec3, axis: Vec3, length: f32) -> Transform {
        // Barrel along the push axis, valve block (model +x) on top.
        let rotation = Quat::from_mat3(&Mat3::from_cols(Vec3::Y, axis, Vec3::Y.cross(axis)));
        Transform {
            translation: base,
            rotation,
            scale: Vec3::splat(length / Self::LENGTH),
        }
    }
}

/// The shake table model, in its own metres.
pub struct ShakeTableSpec;

impl ShakeTableSpec {
    /// Edge of the platen and the height of its top face.
    pub const PLATEN: f32 = 2.9;
    pub const TOP: f32 = 0.54;

    /// The model's frame for a platform of edge `size` flush with the
    /// table top.
    pub fn fit(size: f32) -> Transform {
        let scale = size / Self::PLATEN;
        Transform {
            translation: Vec3::new(0.0, -Self::TOP * scale, 0.0),
            rotation: Quat::IDENTITY,
            scale: Vec3::splat(scale),
        }
    }
}

/// A truck model, in its own metres.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TruckSpec {
    /// x of each axle, front first.
    pub axles: &'static [f32],
    /// Tyre radius.
    pub radius: f32,
    /// Width over the tyres.
    pub width: f32,
}

impl TruckSpec {
    pub fn of(kind: VehicleKind) -> Option<Self> {
        let axles: &[f32] = match kind {
            VehicleKind::Box => return None,
            VehicleKind::Dump => &[0.21, -0.21],
            VehicleKind::CoilHauler => &[0.27, -0.16, -0.34],
            VehicleKind::Crane => &[0.25, -0.25],
        };
        Some(Self {
            axles,
            radius: 0.085,
            width: 0.44,
        })
    }

    /// First to last axle.
    pub fn wheelbase(&self) -> f32 {
        self.axles[0] - self.axles[self.axles.len() - 1]
    }

    /// The scale at which the model spans a vehicle whose last axle is
    /// `wheelbase` behind its first. A single axle says nothing: the model
    /// is then as wide as `width`.
    pub fn scale_for(&self, wheelbase: f32, width: f32) -> f32 {
        if wheelbase > 1e-3 {
            wheelbase / self.wheelbase()
        } else {
            width / self.width
        }
    }

    /// The model's frame in the vehicle's (origin on the ground under the
    /// front axle, +x forward).
    pub fn fit(&self, scale: f32) -> Transform {
        Transform {
            translation: Vec3::new(-self.axles[0] * scale, 0.0, 0.0),
            rotation: Quat::IDENTITY,
            scale: Vec3::splat(scale),
        }
    }

    /// Distance of model axle `k` behind the front one at `scale`.
    pub fn offset(&self, k: usize, scale: f32) -> f32 {
        (self.axles[0] - self.axles[k.min(self.axles.len() - 1)]) * scale
    }
}

/// A hero model that belongs to the hall: lit by the lamps like the rest
/// of it.
#[derive(Component)]
pub(crate) struct HallProp;

/// Where the gantry crane stands, lab metres: (x, z) of the middle of its
/// span. It spans the hall across z, clear of the overhead crane's bridge
/// and of the floor round the table.
pub const GANTRY_AT: Vec2 = Vec2::new(-19.0, 0.0);

pub(crate) fn spawn_gantry(
    mut commands: Commands,
    on: Res<HeroProps>,
    settings: Res<LabSettings>,
    assets: Option<Res<AssetServer>>,
) {
    let Some(assets) = assets.filter(|_| on.0) else {
        return;
    };
    commands.spawn((
        WorldAssetRoot(assets.load(scene_path(GANTRY))),
        Transform {
            translation: settings.lab_to_world(Vec3::new(GANTRY_AT.x, 0.0, GANTRY_AT.y)),
            rotation: Quat::from_rotation_y(FRAC_PI_2),
            scale: Vec3::splat(settings.model_scale),
        },
        Visibility::Inherited,
        HallProp,
        Name::new("gantry crane"),
    ));
}

/// Puts the meshes of hall props on the hall's render layer once their
/// scene has spawned.
pub(crate) fn light_hall_props(
    mut commands: Commands,
    meshes: Query<Entity, Added<Mesh3d>>,
    parents: Query<&ChildOf>,
    props: Query<(), With<HallProp>>,
) {
    for e in &meshes {
        if parents.iter_ancestors(e).any(|a| props.contains(a)) {
            commands.entity(e).insert(RenderLayers::layer(HALL_LAYER));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Node names and translations in a `.glb`, read from its JSON chunk.
    fn nodes(bytes: &[u8]) -> Vec<(String, Vec3)> {
        let len = u32::from_le_bytes(bytes[12..16].try_into().unwrap()) as usize;
        let json = std::str::from_utf8(&bytes[20..20 + len]).unwrap();
        // The list of node objects (the scene's `nodes` is a list of numbers).
        let start = json.find("\"nodes\":[{").expect("the file has nodes");
        let list = &json[start..];
        let mut depth = 0;
        let mut end = list.len();
        for (i, c) in list.char_indices() {
            match c {
                '[' => depth += 1,
                ']' => {
                    depth -= 1;
                    if depth == 0 {
                        end = i;
                        break;
                    }
                }
                _ => {}
            }
        }
        let mut out = Vec::new();
        for node in list[..end].split("\"name\"").skip(1) {
            let name = node.split('"').nth(1).unwrap().to_string();
            let at = node.find("\"translation\"").map_or(Vec3::ZERO, |i| {
                let v: Vec<f32> = node[i..]
                    .split('[')
                    .nth(1)
                    .unwrap()
                    .split(']')
                    .next()
                    .unwrap()
                    .split(',')
                    .map(|x| x.trim().parse().unwrap())
                    .collect();
                Vec3::new(v[0], v[1], v[2])
            });
            out.push((name, at));
        }
        out
    }

    fn file(stem: &str) -> &'static [u8] {
        FILES.iter().find(|(s, _)| *s == stem).unwrap().1
    }

    #[test]
    fn every_file_is_a_binary_gltf_and_belongs_to_a_slot_or_the_hall() {
        for (stem, bytes) in FILES {
            assert_eq!(&bytes[..4], b"glTF", "{stem}");
            assert!(
                stem == GANTRY || RigSlot::ALL.iter().any(|s| s.file_stem() == stem),
                "{stem} is not used"
            );
        }
    }

    #[test]
    fn the_truck_files_have_the_axles_the_code_expects() {
        for kind in [
            VehicleKind::Dump,
            VehicleKind::CoilHauler,
            VehicleKind::Crane,
        ] {
            let slot = RigSlot::Truck(kind);
            let spec = TruckSpec::of(kind).unwrap();
            let mut wheels = vec![0; spec.axles.len()];
            let mut body = false;
            for (name, at) in nodes(file(slot.file_stem())) {
                match alias(slot, &name) {
                    Some((Role::Wheel(k), _)) => {
                        wheels[k as usize] += 1;
                        assert!(
                            (at.x - spec.axles[k as usize]).abs() < 1e-3,
                            "{name} is at {at}"
                        );
                        assert!((at.y - spec.radius).abs() < 1e-3, "{name} is at {at}");
                    }
                    Some((Role::Body, _)) => body = true,
                    _ => {}
                }
            }
            assert!(body, "{kind:?} has no chassis");
            assert!(wheels.iter().all(|n| *n == 2), "{kind:?}: {wheels:?}");
        }
    }

    #[test]
    fn the_fan_wall_file_has_its_nine_rotors_where_the_effects_go() {
        let mut seen = [false; 9];
        for (name, at) in nodes(file(RigSlot::FanWall.file_stem())) {
            if let Some((Role::Rotor(i), _)) = alias(RigSlot::FanWall, &name) {
                let i = i as usize;
                seen[i] = true;
                let want = Vec3::new(FanSpec::COLS[i % 3], FanSpec::ROWS[i / 3], FanSpec::ROTOR_Z);
                assert!(at.distance(want) < 1e-3, "{name} is at {at}");
            }
        }
        assert_eq!(seen, [true; 9]);
    }

    #[test]
    fn the_shake_files_have_their_moving_nodes() {
        let rod = nodes(file("hydraulic_actuator"))
            .into_iter()
            .find(|(n, _)| alias(RigSlot::ShakeActuators, n).is_some());
        assert!(rod.is_some());
        let platen = nodes(file("shake_table"))
            .into_iter()
            .find(|(n, _)| alias(RigSlot::ShakeTable, n).is_some());
        assert!(platen.is_some());
    }

    #[test]
    fn the_fan_wall_blows_along_the_rig_x_and_stands_on_the_floor() {
        let fit = FanSpec::fit(10.0, -8.6);
        let outlet = fit.transform_point(Vec3::new(0.0, 0.0, FanSpec::OUTLET));
        assert!(outlet.distance(Vec3::new(0.0, -8.6, 0.0)) < 1e-4);
        assert!((fit.rotation * Vec3::Z).distance(Vec3::X) < 1e-5);
    }

    #[test]
    fn an_actuator_lies_along_its_push_axis_with_the_valve_on_top() {
        for axis in [Vec3::X, Vec3::Z] {
            let fit = ActuatorSpec::fit(Vec3::new(1.0, 0.4, 2.0), axis, 3.0);
            let tip = fit.transform_point(Vec3::Y * ActuatorSpec::LENGTH);
            assert!(tip.distance(Vec3::new(1.0, 0.4, 2.0) + axis * 3.0) < 1e-4);
            assert!((fit.rotation * Vec3::X).distance(Vec3::Y) < 1e-5);
        }
    }

    #[test]
    fn a_truck_spans_the_axles_it_is_given() {
        let spec = TruckSpec::of(VehicleKind::CoilHauler).unwrap();
        let s = spec.scale_for(5.1, 1.3);
        assert!((spec.offset(2, s) - 5.1).abs() < 1e-4);
        assert_eq!(spec.offset(0, s), 0.0);
        let fit = spec.fit(s);
        let front = fit.transform_point(Vec3::new(spec.axles[0], 0.0, 0.0));
        assert!(front.length() < 1e-4);
    }
}
