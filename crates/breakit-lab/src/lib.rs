//! The lab around the play table.
//!
//! BreakIt is played on a model table in a structures and hydraulics test
//! hall. The structure on the table is full size as far as the solver is
//! concerned (SI units, a 6 m bridge is 6 world units), and the hall is
//! drawn [`LabSettings::model_scale`] times larger, so the table reads as a
//! 1:20 model in a real hall. Depth of field with a sensor scaled the same
//! way gives the out-of-focus background of a photo of a real model.
//!
//! Everything is generated in code: no assets to author or re-export.
//! The hall is merged into one static mesh per material (a couple of dozen
//! draw calls, nothing per frame). The play table is rebuilt when
//! [`PlayTable`] changes; scenarios supply a [`TableInsert`].

pub mod envmap;
pub mod geom;
pub mod hall;
pub mod look;
pub mod palette;
pub mod props;
pub mod scenery;
pub mod surface;
pub mod table;

use std::collections::BTreeMap;
use std::time::Instant;

use bevy::camera::visibility::RenderLayers;
use bevy::light::NotShadowCaster;
use bevy::prelude::*;

pub use hall::HallSpec;
pub use look::{AntiAlias, LabQuality, Look, HALL_LAYER};
pub use palette::Mat;
pub use scenery::{Anchor, Scenery};
pub use surface::{LabMaterial, LabSurface, LabWater, SurfaceKind};
pub use table::{Flat, Flume, PlayTable, Ravine, ShakeTable, TableInsert, TableTop};

#[derive(Resource, Clone, Debug)]
pub struct LabSettings {
    /// World units per lab metre: 20 means the table is a 1:20 model.
    pub model_scale: f32,
    /// Where the table stands in the hall (lab metres, x and z).
    pub table_at: Vec2,
    /// Height of the playing surface above the hall floor, lab metres.
    pub table_height: f32,
    pub hall: HallSpec,
    /// Seed for the dressing.
    pub seed: u64,
    /// The preset `look` was last set from. Change it with
    /// [`LabSettings::set_quality`].
    pub quality: LabQuality,
    /// Lights, reflections and the camera's post stack. Any knob can be
    /// set by hand; the lab re-fits itself when it changes.
    pub look: Look,
    /// Steps through the quality presets. `None` leaves it to the game.
    pub quality_key: Option<KeyCode>,
    /// Lens f-number for the depth of field.
    pub f_stops: f32,
}

impl Default for LabSettings {
    fn default() -> Self {
        Self {
            model_scale: 20.0,
            table_at: Vec2::new(4.0, 5.0),
            table_height: 0.86,
            hall: HallSpec::default(),
            seed: 0xB2EA_4C17,
            quality: LabQuality::default(),
            look: LabQuality::default().look(),
            quality_key: Some(KeyCode::F4),
            f_stops: 2.0,
        }
    }
}

impl LabSettings {
    pub fn with_quality(mut self, quality: LabQuality) -> Self {
        self.set_quality(quality);
        self
    }

    /// Switches to a preset, replacing every knob in `look`.
    pub fn set_quality(&mut self, quality: LabQuality) {
        self.quality = quality;
        self.look = quality.look();
    }

    /// Lab metres to world units.
    pub fn lab_to_world(&self, p: Vec3) -> Vec3 {
        (p - self.lab_origin()) * self.model_scale
    }

    /// The lab point that is the world origin: the middle of the table top.
    pub fn lab_origin(&self) -> Vec3 {
        Vec3::new(self.table_at.x, self.table_height, self.table_at.y)
    }

    /// The hall floor, in world units.
    pub fn floor_y(&self) -> f32 {
        -self.table_height * self.model_scale
    }
}

/// Put this on a 3D camera to give it the lab's lens: the hall's
/// reflections and ambient light, ambient occlusion, bloom, grade, haze and
/// depth of field focused at `focus_distance` (world units, typically the
/// orbit distance). What exactly is on follows `LabSettings::look`.
#[derive(Component, Clone, Copy, Debug)]
pub struct LabCamera {
    pub focus_distance: f32,
}

/// What the lab costs, for read-outs and benchmarks.
#[derive(Resource, Clone, Debug, Default)]
pub struct LabStats {
    pub hall_triangles: usize,
    pub hall_meshes: usize,
    pub table_triangles: usize,
    pub table_meshes: usize,
    /// Lamps that light the scene (the rest only glow).
    pub point_lights: usize,
    /// Lamps that also cast shadows.
    pub shadowed_lamps: usize,
    /// Name of the quality preset in use.
    pub quality: &'static str,
    /// CPU time to paint and filter the reflection maps, ms (once per
    /// reflection size).
    pub environment_bake_ms: f32,
    /// CPU time to generate the hall and dressing, ms (once, at startup).
    pub hall_build_ms: f32,
    /// CPU time of the last table rebuild, ms.
    pub table_build_ms: f32,
    pub table_name: String,
}

/// Where things are in the hall, world units. The scenery is merged into
/// a few meshes, so this is how sound or scenario rigs find, say, the
/// power units or the crane hook.
#[derive(Resource, Clone, Debug, Default)]
pub struct LabAnchors(pub Vec<(Anchor, Vec3)>);

impl LabAnchors {
    pub fn of(&self, kind: Anchor) -> impl Iterator<Item = Vec3> + '_ {
        self.0.iter().filter(move |a| a.0 == kind).map(|a| a.1)
    }
}

pub struct LabPlugin;

impl Plugin for LabPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<LabSettings>()
            .init_resource::<PlayTable>()
            .init_resource::<LabStats>()
            .init_resource::<LabAnchors>()
            .init_resource::<look::LabEnvironment>()
            .init_resource::<GlobalAmbientLight>()
            .add_systems(Startup, spawn_hall);
        surface::build(app);
        app
            .add_systems(
                Update,
                (
                    rebuild_table,
                    (
                        look::quality_key,
                        look::light_hall,
                        look::bake_environment,
                        look::lab_lens,
                    )
                        .chain(),
                ),
            );
    }
}

#[derive(Component)]
struct HallPart;

#[derive(Component)]
struct TablePart;

/// A scenery material: procedural where the material has a look, plain
/// `StandardMaterial` otherwise.
#[derive(Clone)]
enum LabHandle {
    Plain(Handle<StandardMaterial>),
    Surface(Handle<LabMaterial>),
}

/// One material per (scenery material, on the table or in the hall).
#[derive(Resource, Default)]
struct LabMaterials(BTreeMap<(Mat, bool), LabHandle>);

#[derive(bevy::ecs::system::SystemParam)]
struct MaterialMaker<'w> {
    plain: ResMut<'w, Assets<StandardMaterial>>,
    surfaces: ResMut<'w, Assets<LabMaterial>>,
    settings: Res<'w, LabSettings>,
    water: Res<'w, LabWater>,
}

impl MaterialMaker<'_> {
    fn handle(&mut self, cache: &mut LabMaterials, mat: Mat, on_table: bool) -> LabHandle {
        if let Some(h) = cache.0.get(&(mat, on_table)) {
            return h.clone();
        }
        let double_sided = |m: &mut StandardMaterial| {
            if !palette::blended(mat) {
                // Scenery quads are not wound consistently on purpose: it
                // is cheaper to draw both sides than to care.
                m.double_sided = true;
                m.cull_mode = None;
            }
        };
        let h = match surface::kind_of(mat) {
            Some(kind) => {
                let mut m = surface::lab_material(mat, kind, on_table, &self.settings, &self.water);
                double_sided(&mut m.base);
                LabHandle::Surface(self.surfaces.add(m))
            }
            None => {
                let mut m = palette::material(mat);
                double_sided(&mut m);
                LabHandle::Plain(self.plain.add(m))
            }
        };
        cache.0.insert((mat, on_table), h.clone());
        h
    }
}

fn insert_material(e: &mut EntityCommands, h: LabHandle) {
    match h {
        LabHandle::Plain(h) => e.insert(MeshMaterial3d(h)),
        LabHandle::Surface(h) => e.insert(MeshMaterial3d(h)),
    };
}

/// The hall and its dressing, in lab metres.
pub fn build_hall(settings: &LabSettings) -> Scenery {
    let mut s = hall::build(&settings.hall);
    s.merge(props::build(&settings.hall, settings.seed));
    s
}

fn spawn_hall(
    mut commands: Commands,
    settings: Res<LabSettings>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut maker: MaterialMaker,
    mut stats: ResMut<LabStats>,
) {
    let t0 = Instant::now();
    let scenery = build_hall(&settings);
    stats.hall_build_ms = t0.elapsed().as_secs_f32() * 1000.0;
    stats.hall_triangles = scenery.triangles();
    stats.hall_meshes = scenery.bufs.len();
    commands.insert_resource(LabAnchors(
        scenery
            .anchors
            .iter()
            .map(|&(kind, p)| (kind, settings.lab_to_world(p)))
            .collect(),
    ));

    let mut cache = LabMaterials::default();
    let k = settings.model_scale;
    let root = Transform {
        translation: -settings.lab_origin() * k,
        scale: Vec3::splat(k),
        ..default()
    };
    for (mat, buf) in scenery.bufs {
        if buf.is_empty() {
            continue;
        }
        let material = maker.handle(&mut cache, mat, false);
        let mut e = commands.spawn((
            Mesh3d(meshes.add(buf.into_mesh())),
            root,
            // Seen by lab cameras, and it casts in the lamps' shadow maps
            // but not in the daylight's (see `look::HALL_LAYER`).
            RenderLayers::layer(HALL_LAYER),
            HallPart,
            Name::new(format!("lab {mat:?}")),
        ));
        insert_material(&mut e, material);
    }
    commands.insert_resource(cache);
}

#[allow(clippy::too_many_arguments)]
fn rebuild_table(
    mut commands: Commands,
    table: Res<PlayTable>,
    settings: Res<LabSettings>,
    parts: Query<Entity, With<TablePart>>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut maker: MaterialMaker,
    cache: Option<ResMut<LabMaterials>>,
    mut stats: ResMut<LabStats>,
) {
    if !table.is_changed() {
        return;
    }
    let Some(mut cache) = cache else {
        return;
    };
    for e in &parts {
        commands.entity(e).despawn();
    }
    let t0 = Instant::now();
    let scenery = table::build(&table, settings.floor_y());
    stats.table_build_ms = t0.elapsed().as_secs_f32() * 1000.0;
    stats.table_triangles = scenery.triangles();
    stats.table_meshes = scenery.bufs.len();
    stats.table_name = table.insert.name().to_string();
    for (mat, buf) in scenery.bufs {
        if buf.is_empty() {
            continue;
        }
        let material = maker.handle(&mut cache, mat, true);
        let mut e = commands.spawn((
            Mesh3d(meshes.add(buf.into_mesh())),
            Transform::IDENTITY,
            TablePart,
            Name::new(format!("table {mat:?}")),
        ));
        insert_material(&mut e, material);
        if palette::blended(mat) {
            e.insert(NotShadowCaster);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn table_area_is_clear_of_dressing() {
        let s = LabSettings::default();
        let hall = build_hall(&s);
        let t = PlayTable::default();
        // Table footprint in lab metres, plus walking room.
        let half = t.size * 0.5 / s.model_scale + Vec2::splat(1.5);
        for (mat, buf) in &hall.bufs {
            if matches!(
                mat,
                Mat::Concrete | Mat::Dark | Mat::FloorPaint | Mat::LampGlow | Mat::LampHousing
            ) {
                continue; // floor, joints and things overhead
            }
            for p in &buf.positions {
                let d = Vec2::new(p[0], p[2]) - s.table_at;
                let low = p[1] < s.hall.crane_rail - 1.0;
                assert!(
                    !(low && d.x.abs() < half.x && d.y.abs() < half.y),
                    "{mat:?} at {p:?} stands where the table goes"
                );
            }
        }
    }

    #[test]
    fn anchors_are_inside_the_hall() {
        let s = LabSettings::default();
        let hall = build_hall(&s);
        let (hx, hz) = s.hall.half();
        assert!(hall.anchors.iter().any(|a| a.0 == Anchor::CraneHook));
        for (kind, p) in &hall.anchors {
            assert!(
                p.x.abs() < hx && p.z.abs() < hz && p.y > 0.0,
                "{kind:?} at {p}"
            );
        }
    }

    #[test]
    fn world_origin_is_the_table_top() {
        let s = LabSettings::default();
        assert_eq!(s.lab_to_world(s.lab_origin()), Vec3::ZERO);
        assert!((s.floor_y() + 17.2).abs() < 1e-4);
    }
}
