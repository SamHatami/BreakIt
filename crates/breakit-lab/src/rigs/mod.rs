//! The visible load rigs: what the forces of a scenario come from.
//!
//! A fan wall on a cart, the shake table's actuators with an instrument
//! cart and its seismograph, the wave flume's piston paddle and the heavy
//! model trucks. They wait in the rig park and roll (or are lifted) to the
//! table when a scenario calls for them.
//!
//! The lab does not know the solver. Whoever runs the test writes
//! [`LabRigs`]: `setup` says which rigs the scenario needs, `live` is this
//! frame's state of each (fan drive, table motion, paddle stroke, wheel
//! contacts). `breakit-game/src/rigs.rs` copies it from
//! `breakit_core::loads::RigState`; field for field the two match.
//!
//! Every rig is a root entity with one child per moving [`Role`] (rotor,
//! wheel, rod, ...). The geometry here is procedural; a hero model can
//! replace a rig's geometry through [`RigModels`], and its nodes are driven
//! by the same systems when they carry the role's name (see
//! [`Role::from_node_name`] and `docs/rigs.md`).

pub mod fan;
pub mod paddle;
pub mod park;
pub mod shake;
pub mod truck;

use std::collections::BTreeMap;

use bevy::light::NotShadowCaster;
use bevy::prelude::*;
use bevy::world_serialization::{WorldAsset, WorldAssetRoot};

use crate::palette::{self, Mat};
use crate::scenery::Scenery;
use crate::{insert_material, LabMaterials, LabSettings, MaterialMaker, PlayTable};

pub use truck::VehicleKind;

/// Which rigs the scenario needs and how they are driven right now.
#[derive(Resource, Clone, Debug, Default, PartialEq)]
pub struct LabRigs {
    /// Set when a scenario is chosen; rigs roll in or back to the park when
    /// it changes.
    pub setup: RigSetup,
    /// This frame's state. The default is everything at rest.
    pub live: RigLive,
}

/// The rigs a scenario calls for.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct RigSetup {
    pub fan: Option<FanSetup>,
    pub shake: Option<ShakeSetup>,
    pub paddle: Option<PaddleSetup>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FanSetup {
    /// Direction the wind blows, horizontal.
    pub direction: Vec3,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ShakeSetup {
    /// Edge length of the platform, world units (see `table::ShakeTable`).
    pub platform: f32,
}

impl Default for ShakeSetup {
    fn default() -> Self {
        Self {
            platform: crate::table::ShakeTable::default().size,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PaddleSetup {
    /// Direction the waves travel, horizontal.
    pub direction: Vec3,
    /// Rest position of the paddle face along `direction` from the world
    /// origin, world units.
    pub rest: f32,
    /// Channel width, world units.
    pub width: f32,
    /// Channel bed, world y.
    pub bed: f32,
}

impl Default for PaddleSetup {
    fn default() -> Self {
        let flume = crate::table::Flume::default();
        Self {
            direction: Vec3::X,
            rest: -PlayTable::default().size.x * 0.5 + 0.4,
            width: flume.width,
            bed: -flume.depth,
        }
    }
}

/// State of every rig at one instant. Mirrors
/// `breakit_core::loads::RigState` in `f32` and world units.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct RigLive {
    /// Test time, s. Rotors, wheels and smoke advance with it, so they slow
    /// down and stop with the simulation.
    pub time: f32,
    pub wind: Option<WindDrive>,
    pub table: Option<TableDrive>,
    pub flume: Option<FlumeDrive>,
    pub vehicles: Vec<VehicleDrive>,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct WindDrive {
    /// Fan power 0..1.
    pub fan_drive: f32,
    /// Speed at the reference point including the gust, m/s.
    pub speed: f32,
    pub direction: Vec3,
    /// Current gust, m/s.
    pub gust: f32,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct TableDrive {
    /// m.
    pub displacement: Vec3,
    /// m/s.
    pub velocity: Vec3,
    /// m/s².
    pub acceleration: Vec3,
    /// Actuator demand 0..1.
    pub drive: f32,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FlumeDrive {
    /// Bed level, m.
    pub bed: f32,
    /// Still water level, world y, m.
    pub still_level: f32,
    /// Still water depth above the bed, m.
    pub depth: f32,
    /// Rest position of the paddle face along `direction`, m.
    pub paddle_rest: f32,
    /// Paddle displacement from rest along `direction`, m.
    pub paddle_position: f32,
    /// m/s.
    pub paddle_velocity: f32,
    /// Target wave height, m.
    pub wave_height: f32,
    /// Wave period, s.
    pub wave_period: f32,
    pub direction: Vec3,
    /// m/s.
    pub current_speed: f32,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct WheelDrive {
    /// Tyre contact point, world.
    pub contact: Vec3,
    /// Body displacement above this axle's static ride height, m.
    pub bounce: f32,
    pub lifted: bool,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct VehicleDrive {
    /// Picks the model: see [`VehicleKind::from_name`].
    pub name: String,
    /// Unit vector from the rear to the front wheel contact.
    pub forward: Vec3,
    /// m/s.
    pub speed: f32,
    /// One per axle, front first.
    pub wheels: Vec<WheelDrive>,
    pub braking: bool,
    pub fallen: bool,
}

/// What a part of a rig does. Systems drive parts by role, so a hero
/// model's nodes move like the procedural ones once they carry the role.
#[derive(Component, Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Role {
    /// Fan rotor `n`, spins about its local x.
    Rotor(u8),
    /// Disc in front of rotor `n` that fades in as the blades blur.
    Blur(u8),
    /// The sprung body of a vehicle.
    Body,
    /// Wheel set of axle `n` (front is 0), rolls about its local z.
    Wheel(u8),
    /// Shown only while braking.
    BrakeLight,
    /// The flume's piston plate and carriage, slides along local x.
    Plate,
    /// Actuator rod `n`, slides along its local x.
    Rod(u8),
    /// The part of a shake rig that rides the platform.
    Platform,
    /// Shown while the rig runs / while it idles.
    LampOn,
    LampOff,
    /// Wool tuft `n`, hangs from its pivot and streams out along local x.
    Tuft(u8),
}

impl Role {
    /// The role of a model node called `name`: `rotor.3`, `wheel.0`,
    /// `rod.1`, `tuft.12`, `blur.3`, `body`, `plate`, `platform`,
    /// `brake_light`, `lamp_on`, `lamp_off`. Case and anything after a
    /// second dot are ignored, so `Wheel.1.L` and `wheel.1.R` both roll
    /// with axle 1.
    pub fn from_node_name(name: &str) -> Option<Role> {
        let lower = name.to_ascii_lowercase();
        let mut it = lower.split('.');
        let head = it.next()?;
        let n = it.next().and_then(|s| s.parse::<u8>().ok());
        Some(match (head, n) {
            ("rotor", Some(n)) => Role::Rotor(n),
            ("blur", Some(n)) => Role::Blur(n),
            ("wheel", Some(n)) => Role::Wheel(n),
            ("rod", Some(n)) => Role::Rod(n),
            ("tuft", Some(n)) => Role::Tuft(n),
            ("body", _) => Role::Body,
            ("plate", _) => Role::Plate,
            ("platform", _) => Role::Platform,
            ("brake_light", _) => Role::BrakeLight,
            ("lamp_on", _) => Role::LampOn,
            ("lamp_off", _) => Role::LampOff,
            _ => return None,
        })
    }
}

/// Where a driven part sits when its rig is at rest, relative to its
/// parent. Drives are applied on top of it.
#[derive(Component, Clone, Copy, Debug)]
pub struct Rest(pub Transform);

/// The rigs a hero model can replace.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum RigSlot {
    FanWall,
    InstrumentCart,
    ShakeActuators,
    WavePaddle,
    Truck(VehicleKind),
}

impl RigSlot {
    /// File stem of the slot's model under `assets/props/`.
    pub fn file_stem(self) -> &'static str {
        match self {
            RigSlot::FanWall => "fan_wall",
            RigSlot::InstrumentCart => "instrument_cart",
            RigSlot::ShakeActuators => "shake_actuators",
            RigSlot::WavePaddle => "wave_paddle",
            RigSlot::Truck(VehicleKind::Box) => "truck_box",
            RigSlot::Truck(VehicleKind::Dump) => "truck_dump",
            RigSlot::Truck(VehicleKind::CoilHauler) => "truck_coil",
            RigSlot::Truck(VehicleKind::Crane) => "truck_crane",
        }
    }

    pub const ALL: [RigSlot; 8] = [
        RigSlot::FanWall,
        RigSlot::InstrumentCart,
        RigSlot::ShakeActuators,
        RigSlot::WavePaddle,
        RigSlot::Truck(VehicleKind::Box),
        RigSlot::Truck(VehicleKind::Dump),
        RigSlot::Truck(VehicleKind::CoilHauler),
        RigSlot::Truck(VehicleKind::Crane),
    ];
}

/// Hero models that replace the procedural geometry of a rig. A slot with
/// a model spawns the model's scene under the rig root instead; its nodes
/// are driven by name (see [`Role::from_node_name`]). The model is
/// authored in the rig's frame, in world units (`docs/rigs.md` lists the
/// frames).
#[derive(Resource, Clone, Debug, Default)]
pub struct RigModels(pub BTreeMap<RigSlot, Handle<WorldAsset>>);

impl RigModels {
    /// Loads `assets/<folder>/<slot>.glb` for every slot whose file exists.
    pub fn from_folder(assets: &AssetServer, folder: &str) -> Self {
        let mut out = BTreeMap::new();
        for slot in RigSlot::ALL {
            let file = format!("{folder}/{}.glb", slot.file_stem());
            if std::path::Path::new("assets").join(&file).exists() {
                out.insert(slot, assets.load(format!("{file}#Scene0")));
            }
        }
        Self(out)
    }
}

/// A rig's root entity.
#[derive(Component, Clone, Copy, Debug, PartialEq, Eq)]
pub struct Rig(pub RigSlot);

/// Moves a rig between the park and its place at the table.
#[derive(Component, Clone, Copy, Debug)]
pub struct Travel {
    pub from: Transform,
    pub to: Transform,
    /// 0..1.
    pub progress: f32,
    /// Seconds for the whole move.
    pub duration: f32,
    /// Height of the lift in the middle of the move (table-mounted rigs are
    /// craned over, carts roll and use 0).
    pub hoist: f32,
}

impl Travel {
    pub fn arrived(&self) -> bool {
        self.progress >= 1.0
    }

    /// The pose at the current progress.
    pub fn pose(&self) -> Transform {
        let t = self.progress.clamp(0.0, 1.0);
        let s = t * t * (3.0 - 2.0 * t);
        let mut out = Transform {
            translation: self.from.translation.lerp(self.to.translation, s),
            rotation: self.from.rotation.slerp(self.to.rotation, s),
            scale: Vec3::ONE,
        };
        out.translation.y += self.hoist * (std::f32::consts::PI * s).sin();
        out
    }
}

/// The rig a driven part belongs to.
#[derive(Component, Clone, Copy, Debug, PartialEq, Eq)]
pub struct Owner(pub Entity);

/// One piece of a procedural rig that moves as one.
pub struct Part {
    /// `None`: fixed to the rig.
    pub role: Option<Role>,
    /// The part's frame in the rig's frame.
    pub at: Transform,
    /// Geometry in the part's frame.
    pub geom: Scenery,
}

impl Part {
    pub fn fixed(geom: Scenery) -> Self {
        Self {
            role: None,
            at: Transform::IDENTITY,
            geom,
        }
    }

    /// A part whose `geom` is given in the rig's frame and that turns or
    /// slides about `pivot`.
    pub fn driven(role: Role, pivot: Vec3, geom: Scenery) -> Self {
        let mut moved = Scenery::default();
        for (mat, buf) in geom.bufs {
            moved.m(mat).append(&buf, -pivot);
        }
        Self {
            role: Some(role),
            at: Transform::from_translation(pivot),
            geom: moved,
        }
    }

    /// A part whose `geom` is given in its own frame `at`.
    pub fn local(role: Role, at: Transform, geom: Scenery) -> Self {
        Self {
            role: Some(role),
            at,
            geom,
        }
    }
}

/// The geometry of a rig as parts.
#[derive(Default)]
pub struct RigModel {
    pub parts: Vec<Part>,
}

impl RigModel {
    pub fn triangles(&self) -> usize {
        self.parts.iter().map(|p| p.geom.triangles()).sum()
    }

    /// Everything merged, in the rig's frame: for parked display copies.
    pub fn merged(self) -> Scenery {
        let mut out = Scenery::default();
        for p in self.parts {
            for (mat, buf) in p.geom.bufs {
                out.m(mat).append_transformed(&buf, &p.at);
            }
        }
        out
    }
}

/// Materials the rigs need beyond the lab's palette.
#[derive(Resource, Clone)]
pub(crate) struct RigMaterials {
    /// Motion blur of the fan blades; alpha follows the fan drive.
    pub blur: Handle<StandardMaterial>,
    /// Smoke streamers (vertex alpha).
    pub smoke: Handle<StandardMaterial>,
    /// Seismograph trace.
    pub trace: Handle<StandardMaterial>,
}

impl FromWorld for RigMaterials {
    fn from_world(world: &mut World) -> Self {
        let mut mats = world.resource_mut::<Assets<StandardMaterial>>();
        let see_through = |color: Color| StandardMaterial {
            base_color: color,
            alpha_mode: AlphaMode::Blend,
            unlit: true,
            double_sided: true,
            cull_mode: None,
            ..default()
        };
        Self {
            blur: mats.add(see_through(Color::srgba(0.16, 0.17, 0.19, 0.0))),
            smoke: mats.add(see_through(Color::srgba(0.8, 0.82, 0.84, 1.0))),
            trace: mats.add(StandardMaterial {
                base_color: Color::srgb(0.3, 1.0, 0.5),
                emissive: LinearRgba::rgb(0.3, 3.0, 0.8),
                ..default()
            }),
        }
    }
}

/// Spawns `scenery` as mesh children of `parent`, one per material.
pub(crate) fn spawn_meshes(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    maker: &mut MaterialMaker,
    cache: &mut LabMaterials,
    parent: Entity,
    scenery: Scenery,
) {
    for (mat, buf) in scenery.bufs {
        if buf.is_empty() {
            continue;
        }
        // Rigs are real-size hardware: patterned in lab metres.
        let material = maker.handle(cache, mat, false);
        let mut e = commands.spawn((
            Mesh3d(meshes.add(buf.into_mesh())),
            Transform::IDENTITY,
            ChildOf(parent),
        ));
        insert_material(&mut e, material);
        if palette::blended(mat) || emissive(mat) {
            e.insert(NotShadowCaster);
        }
    }
}

fn emissive(mat: Mat) -> bool {
    matches!(
        mat,
        Mat::IndicatorGreen | Mat::IndicatorRed | Mat::IndicatorAmber | Mat::WindowWarm
    )
}

/// Spawns a rig root at `pose` with the slot's hero model if there is one,
/// its procedural `model` otherwise.
#[allow(clippy::too_many_arguments)]
pub(crate) fn spawn_rig(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    maker: &mut MaterialMaker,
    cache: &mut LabMaterials,
    models: &RigModels,
    slot: RigSlot,
    pose: Transform,
    model: impl FnOnce() -> RigModel,
) -> Entity {
    let root = commands
        .spawn((
            Rig(slot),
            pose,
            Visibility::Inherited,
            Name::new(format!("rig {}", slot.file_stem())),
        ))
        .id();
    if let Some(scene) = models.0.get(&slot) {
        commands.spawn((WorldAssetRoot(scene.clone()), ChildOf(root)));
        return root;
    }
    for part in model().parts {
        let mut e = commands.spawn((part.at, Visibility::Inherited, ChildOf(root)));
        if let Some(role) = part.role {
            e.insert((role, Rest(part.at), Owner(root)));
        }
        let id = e.id();
        spawn_meshes(commands, meshes, maker, cache, id, part.geom);
    }
    root
}

/// Gives the named nodes of a hero model their roles once its scene has
/// spawned.
fn bind_model_nodes(
    mut commands: Commands,
    named: Query<(Entity, &Name, &Transform), (Added<Name>, Without<Role>, Without<Rig>)>,
    parents: Query<&ChildOf>,
    rigs: Query<(), With<Rig>>,
) {
    for (e, name, at) in &named {
        let Some(role) = Role::from_node_name(name.as_str()) else {
            continue;
        };
        if let Some(rig) = parents.iter_ancestors(e).find(|&a| rigs.contains(a)) {
            commands.entity(e).insert((role, Rest(*at), Owner(rig)));
        }
    }
}

fn travel(
    time: Res<Time>,
    mut commands: Commands,
    mut rigs: Query<(Entity, &mut Travel, &mut Transform)>,
) {
    for (e, mut tr, mut at) in &mut rigs {
        tr.progress = (tr.progress + time.delta_secs() / tr.duration.max(1e-3)).min(1.0);
        *at = tr.pose();
        if tr.arrived() {
            commands.entity(e).remove::<Travel>();
        }
    }
}

/// Sends a rig from where it is to `to`.
pub(crate) fn send(commands: &mut Commands, e: Entity, from: Transform, to: Transform, hoist: f32) {
    if from.translation.distance(to.translation) < 1e-3
        && from.rotation.angle_between(to.rotation) < 1e-3
    {
        return;
    }
    let distance = from.translation.distance(to.translation);
    commands.entity(e).insert(Travel {
        from,
        to,
        progress: 0.0,
        duration: (distance / 14.0).clamp(1.2, 4.0),
        hoist,
    });
}

/// Seconds of test time since the previous frame, for things that
/// integrate (rotor angle, wheel roll). Zero when the test is paused or
/// has been reset.
#[derive(Resource, Clone, Copy, Debug, Default)]
pub(crate) struct RigClock {
    last: f32,
    pub dt: f32,
}

fn tick(rigs: Res<LabRigs>, mut clock: ResMut<RigClock>) {
    let dt = rigs.live.time - clock.last;
    // A long frame still counts; a jump back or far ahead is a new test.
    clock.dt = if (0.0..0.5).contains(&dt) { dt } else { 0.0 };
    clock.last = rigs.live.time;
}

/// Systems that drive the rigs from [`LabRigs`], after it has been written.
#[derive(SystemSet, Debug, Clone, PartialEq, Eq, Hash)]
pub struct RigSet;

pub(crate) fn build(app: &mut App) {
    app.init_resource::<LabRigs>()
        .init_resource::<RigModels>()
        .init_resource::<RigMaterials>()
        .init_resource::<RigClock>()
        .add_systems(
            Update,
            (
                tick,
                park::spawn_park,
                (fan::call, shake::call, paddle::call),
                travel,
                bind_model_nodes,
                (
                    fan::drive,
                    fan::streamers,
                    shake::drive,
                    shake::trace,
                    paddle::drive,
                    truck::drive,
                ),
            )
                .chain()
                .in_set(RigSet),
        );
}

/// Yaw that turns a rig's local +x to `direction` (horizontal).
pub(crate) fn yaw_to(direction: Vec3) -> Quat {
    let d = Vec3::new(direction.x, 0.0, direction.z).normalize_or(Vec3::X);
    Quat::from_rotation_y((-d.z).atan2(d.x))
}

/// Settings and table every rig is placed against.
#[derive(bevy::ecs::system::SystemParam)]
pub(crate) struct Site<'w> {
    pub settings: Res<'w, LabSettings>,
    pub table: Res<'w, PlayTable>,
}

impl Site<'_> {
    /// Hall floor, world y.
    pub fn floor(&self) -> f32 {
        self.settings.floor_y()
    }

    /// Half size of the table including its rim, x and z.
    pub fn table_half(&self) -> Vec2 {
        self.table.size * 0.5 + Vec2::splat(0.35)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn node_names_map_to_roles() {
        assert_eq!(Role::from_node_name("rotor.3"), Some(Role::Rotor(3)));
        assert_eq!(Role::from_node_name("Wheel.1.L"), Some(Role::Wheel(1)));
        assert_eq!(Role::from_node_name("body"), Some(Role::Body));
        assert_eq!(Role::from_node_name("Body.001"), Some(Role::Body));
        assert_eq!(Role::from_node_name("rotor"), None);
        assert_eq!(Role::from_node_name("chassis"), None);
    }

    #[test]
    fn yaw_points_local_x_along_the_direction() {
        for d in [Vec3::X, Vec3::Z, Vec3::NEG_X, Vec3::new(1.0, 0.0, -1.0)] {
            let got = yaw_to(d) * Vec3::X;
            assert!(got.distance(d.normalize()) < 1e-5, "{d} gave {got}");
        }
    }

    #[test]
    fn travel_starts_and_ends_on_its_poses_and_lifts_between() {
        let mut t = Travel {
            from: Transform::from_xyz(0.0, 0.0, 0.0),
            to: Transform::from_xyz(10.0, 0.0, 0.0),
            progress: 0.0,
            duration: 2.0,
            hoist: 3.0,
        };
        assert_eq!(t.pose().translation, Vec3::ZERO);
        t.progress = 0.5;
        assert!((t.pose().translation - Vec3::new(5.0, 3.0, 0.0)).length() < 1e-4);
        t.progress = 1.0;
        assert!((t.pose().translation - Vec3::new(10.0, 0.0, 0.0)).length() < 1e-4);
    }

    #[test]
    fn every_slot_has_its_own_file() {
        let mut stems: Vec<_> = RigSlot::ALL.iter().map(|s| s.file_stem()).collect();
        stems.sort_unstable();
        stems.dedup();
        assert_eq!(stems.len(), RigSlot::ALL.len());
    }
}
