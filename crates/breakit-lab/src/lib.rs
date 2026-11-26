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

pub mod geom;
pub mod hall;
pub mod palette;
pub mod props;
pub mod scenery;
pub mod table;

use std::collections::BTreeMap;
use std::time::Instant;

use bevy::camera::Projection;
use bevy::core_pipeline::tonemapping::Tonemapping;
use bevy::light::{CascadeShadowConfigBuilder, NotShadowCaster};
use bevy::pbr::{DistanceFog, FogFalloff};
use bevy::post_process::bloom::Bloom;
use bevy::post_process::dof::{DepthOfField, DepthOfFieldMode};
use bevy::prelude::*;

pub use hall::HallSpec;
pub use palette::Mat;
pub use scenery::{Anchor, Scenery};
pub use table::{Flat, Flume, PlayTable, Ravine, ShakeTable, TableInsert, TableTop};

/// Super 35 sensor height, Bevy's default for depth of field, m.
const SENSOR_HEIGHT: f32 = 0.018_66;

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
    /// Real point lights under the high-bay lamps (otherwise they only
    /// glow). The most expensive part of the hall on a GPU.
    pub lamp_lights: bool,
    /// Depth of field on lab cameras. `None` turns it off.
    pub dof: Option<DepthOfFieldMode>,
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
            lamp_lights: true,
            dof: Some(DepthOfFieldMode::Bokeh),
            f_stops: 2.0,
        }
    }
}

impl LabSettings {
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

/// Put this on a 3D camera to give it the lab's lens: HDR, bloom,
/// tonemapping, haze and depth of field focused at `focus_distance`
/// (world units, typically the orbit distance).
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
    pub point_lights: usize,
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
            .insert_resource(GlobalAmbientLight {
                color: Color::srgb(0.82, 0.88, 0.95),
                brightness: 110.0,
                ..default()
            })
            .add_systems(Startup, spawn_hall)
            .add_systems(Update, (rebuild_table, lab_lens));
    }
}

#[derive(Component)]
struct HallPart;

#[derive(Component)]
struct TablePart;

#[derive(Resource)]
struct LabMaterials(BTreeMap<Mat, Handle<StandardMaterial>>);

fn material_handle(
    cache: &mut LabMaterials,
    materials: &mut Assets<StandardMaterial>,
    mat: Mat,
) -> Handle<StandardMaterial> {
    cache
        .0
        .entry(mat)
        .or_insert_with(|| {
            let mut m = palette::material(mat);
            if !palette::blended(mat) {
                // Scenery quads are not wound consistently on purpose: it
                // is cheaper to draw both sides than to care.
                m.double_sided = true;
                m.cull_mode = None;
            }
            materials.add(m)
        })
        .clone()
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
    mut materials: ResMut<Assets<StandardMaterial>>,
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

    let mut cache = LabMaterials(BTreeMap::new());
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
        commands.spawn((
            Mesh3d(meshes.add(buf.into_mesh())),
            MeshMaterial3d(material_handle(&mut cache, &mut materials, mat)),
            root,
            NotShadowCaster,
            HallPart,
            Name::new(format!("lab {mat:?}")),
        ));
    }
    commands.insert_resource(cache);

    // Daylight through the roof lights: the one shadowed light, its
    // cascades sized for the table, not the hall.
    commands.spawn((
        DirectionalLight {
            color: Color::srgb(0.95, 0.97, 1.0),
            illuminance: 2_500.0,
            shadow_maps_enabled: true,
            ..default()
        },
        CascadeShadowConfigBuilder {
            num_cascades: 3,
            minimum_distance: 0.5,
            first_cascade_far_bound: 25.0,
            maximum_distance: 140.0,
            ..default()
        }
        .build(),
        Transform::from_xyz(-3.0, 10.0, 4.0).looking_at(Vec3::ZERO, Vec3::Y),
        HallPart,
    ));

    // High-bay lamps. Intensities are those of real 20 klm fittings,
    // scaled with the square of the model scale so the hall is lit as a
    // hall would be.
    let mut lights = 0;
    if settings.lamp_lights {
        for p in settings.hall.lamp_positions() {
            commands.spawn((
                PointLight {
                    color: Color::srgb(1.0, 0.95, 0.88),
                    intensity: 20_000.0 * k * k * 8.0,
                    range: 30.0 * k,
                    radius: 0.3 * k,
                    shadow_maps_enabled: false,
                    ..default()
                },
                Transform::from_translation(settings.lab_to_world(p - Vec3::Y * 0.05)),
                HallPart,
            ));
            lights += 1;
        }
    }
    stats.point_lights = lights;
}

#[allow(clippy::too_many_arguments)]
fn rebuild_table(
    mut commands: Commands,
    table: Res<PlayTable>,
    settings: Res<LabSettings>,
    parts: Query<Entity, With<TablePart>>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
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
        let mut e = commands.spawn((
            Mesh3d(meshes.add(buf.into_mesh())),
            MeshMaterial3d(material_handle(&mut cache, &mut materials, mat)),
            Transform::IDENTITY,
            TablePart,
            Name::new(format!("table {mat:?}")),
        ));
        if palette::blended(mat) {
            e.insert(NotShadowCaster);
        }
    }
}

/// Applies the lab lens to new lab cameras and keeps the focus on target.
fn lab_lens(
    mut commands: Commands,
    settings: Res<LabSettings>,
    mut cams: Query<(Entity, &LabCamera, Option<&mut DepthOfField>, Has<Bloom>)>,
) {
    let k = settings.model_scale;
    for (e, cam, dof, has_bloom) in &mut cams {
        if !has_bloom {
            commands.entity(e).insert((
                Bloom::NATURAL,
                Tonemapping::TonyMcMapface,
                Projection::Perspective(PerspectiveProjection {
                    far: 6.0 * settings.hall.length() * k,
                    ..default()
                }),
                DistanceFog {
                    color: Color::srgba(0.55, 0.6, 0.66, 0.4),
                    falloff: FogFalloff::Linear {
                        start: 12.0 * k,
                        end: 70.0 * k,
                    },
                    ..default()
                },
            ));
        }
        match (settings.dof, dof) {
            (None, Some(_)) => {
                commands.entity(e).remove::<DepthOfField>();
            }
            (Some(mode), Some(mut d)) => {
                d.mode = mode;
                d.focal_distance = cam.focus_distance;
                d.aperture_f_stops = settings.f_stops;
                d.sensor_height = SENSOR_HEIGHT * k;
            }
            (Some(mode), None) => {
                commands.entity(e).insert(DepthOfField {
                    mode,
                    focal_distance: cam.focus_distance,
                    sensor_height: SENSOR_HEIGHT * k,
                    aperture_f_stops: settings.f_stops,
                    max_depth: 200.0 * k,
                    ..default()
                });
            }
            (None, None) => {}
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
