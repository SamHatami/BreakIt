//! The lab's look: lights, reflections and the camera's post stack, behind
//! one quality setting.
//!
//! The direction is a photograph of an architect's model in a working test
//! hall: cool daylight from the roof lights, warm high-bay lamps, a lens
//! with real depth of field, a restrained film grade. Nothing glows that
//! would not glow in a real hall.
//!
//! [`LabQuality`] picks a [`Look`]; every knob in it can also be set by hand
//! through `LabSettings::look`. Changing either at run time re-lights the
//! hall and re-fits the cameras, so presets can be compared on real hardware
//! with the frame-time read-out open.

use bevy::anti_alias::contrast_adaptive_sharpening::ContrastAdaptiveSharpening;
use bevy::anti_alias::fxaa::Fxaa;
use bevy::anti_alias::smaa::{Smaa, SmaaPreset};
use bevy::anti_alias::taa::TemporalAntiAliasing;
use bevy::camera::visibility::RenderLayers;
use bevy::camera::{Exposure, Projection};
use bevy::core_pipeline::tonemapping::Tonemapping;
use bevy::light::{
    CascadeShadowConfigBuilder, DirectionalLightShadowMap, EnvironmentMapLight, PointLightShadowMap,
};
use bevy::pbr::{
    ContactShadows, DistanceFog, FogFalloff, ScreenSpaceAmbientOcclusion,
    ScreenSpaceAmbientOcclusionQualityLevel,
};
use bevy::post_process::bloom::{Bloom, BloomPrefilter};
use bevy::post_process::dof::{DepthOfField, DepthOfFieldMode};
use bevy::post_process::effect_stack::Vignette;
use bevy::prelude::*;
use bevy::render::view::{ColorGrading, ColorGradingGlobal, ColorGradingSection};

use crate::envmap::{self, HallRadiance};
use crate::{LabCamera, LabSettings, LabStats};

/// Super 35 sensor height, Bevy's default for depth of field, m.
const SENSOR_HEIGHT: f32 = 0.018_66;

/// Render layer of the hall and its dressing. The play table, the structure
/// and everything else stay on layer 0. Lab cameras see both. The daylight
/// only takes shadows from layer 0 (the roof would otherwise shade the whole
/// hall), the lamps from both.
pub const HALL_LAYER: usize = 1;

/// How much the lab is allowed to cost. Each step up adds things a GPU
/// pays for per pixel; none of them touch the solver's CPU budget.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub enum LabQuality {
    /// Integrated graphics: no ambient occlusion, cheap blur, few lamps.
    Low,
    /// Ambient occlusion and edge anti-aliasing.
    Medium,
    /// The intended look.
    #[default]
    High,
    /// High plus temporal anti-aliasing and more shadowed lamps.
    Ultra,
}

impl LabQuality {
    pub const ALL: [LabQuality; 4] = [
        LabQuality::Low,
        LabQuality::Medium,
        LabQuality::High,
        LabQuality::Ultra,
    ];

    pub fn name(self) -> &'static str {
        match self {
            LabQuality::Low => "low",
            LabQuality::Medium => "medium",
            LabQuality::High => "high",
            LabQuality::Ultra => "ultra",
        }
    }

    pub fn parse(s: &str) -> Option<Self> {
        Self::ALL.into_iter().find(|q| q.name() == s)
    }

    pub fn next(self) -> Self {
        Self::ALL[(self as usize + 1) % Self::ALL.len()]
    }

    pub fn look(self) -> Look {
        use ScreenSpaceAmbientOcclusionQualityLevel as Ao;
        match self {
            LabQuality::Low => Look {
                anti_alias: AntiAlias::Fxaa,
                ambient_occlusion: None,
                contact_shadows: false,
                bloom: true,
                dof: Some(DepthOfFieldMode::Gaussian),
                vignette: true,
                shadow_map: 1024,
                cascades: 2,
                lamp_lights: 6,
                lamp_shadows: 0,
                reflections: 32,
                surface_detail: false,
            },
            LabQuality::Medium => Look {
                anti_alias: AntiAlias::Smaa,
                ambient_occlusion: Some(Ao::Medium),
                contact_shadows: false,
                bloom: true,
                dof: Some(DepthOfFieldMode::Gaussian),
                vignette: true,
                shadow_map: 2048,
                cascades: 3,
                lamp_lights: 12,
                lamp_shadows: 0,
                reflections: 64,
                surface_detail: true,
            },
            LabQuality::High => Look {
                anti_alias: AntiAlias::Smaa,
                ambient_occlusion: Some(Ao::High),
                contact_shadows: true,
                bloom: true,
                dof: Some(DepthOfFieldMode::Bokeh),
                vignette: true,
                shadow_map: 2048,
                cascades: 3,
                lamp_lights: 20,
                lamp_shadows: 4,
                reflections: 128,
                surface_detail: true,
            },
            LabQuality::Ultra => Look {
                anti_alias: AntiAlias::Taa,
                ambient_occlusion: Some(Ao::Ultra),
                contact_shadows: true,
                bloom: true,
                dof: Some(DepthOfFieldMode::Bokeh),
                vignette: true,
                shadow_map: 4096,
                cascades: 4,
                lamp_lights: 20,
                lamp_shadows: 8,
                reflections: 256,
                surface_detail: true,
            },
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AntiAlias {
    Off,
    /// One cheap pass.
    Fxaa,
    /// Edge-based, sharp, nothing temporal: thin members stay put.
    Smaa,
    /// Temporal, with sharpening. Smoothest, and it hides the noise of the
    /// ambient occlusion, but fast-moving members can leave faint trails.
    Taa,
}

/// Every knob of the look. Start from [`LabQuality::look`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Look {
    pub anti_alias: AntiAlias,
    /// Screen-space ambient occlusion: darkens creases and where things
    /// stand on the floor.
    pub ambient_occlusion: Option<ScreenSpaceAmbientOcclusionQualityLevel>,
    /// Short screen-space shadow rays from the daylight, for the last
    /// centimetre where a member meets the table.
    pub contact_shadows: bool,
    pub bloom: bool,
    /// Depth of field. `None` turns it off.
    pub dof: Option<DepthOfFieldMode>,
    pub vignette: bool,
    /// Edge of each shadow map, texels.
    pub shadow_map: usize,
    /// Cascades of the daylight's shadow.
    pub cascades: usize,
    /// How many high-bay lamps really light the scene, nearest the table
    /// first. The rest only glow.
    pub lamp_lights: usize,
    /// How many of those cast shadows.
    pub lamp_shadows: usize,
    /// Edge of the reflection cube map, texels. 0 falls back to flat
    /// ambient light and no reflections.
    pub reflections: usize,
    /// Procedural surfaces (worn concrete, grime, spangle, foam cells,
    /// moving water). Off, the same materials are drawn plain.
    pub surface_detail: bool,
}

/// Marks what [`light_hall`] spawned.
#[derive(Component)]
pub(crate) struct HallLight;

/// The settings a camera or the lights were last fitted with.
#[derive(Component, Clone, PartialEq)]
pub(crate) struct Fitted(Look, f32);

#[derive(Resource, Default)]
pub(crate) struct LabEnvironment {
    size: usize,
    maps: Option<(Handle<Image>, Handle<Image>)>,
}

/// Environment light scale: radiance units of `envmap` to cd/m².
const ENV_INTENSITY: f32 = 120.0;
/// The lamps are driven harder than real 20 klm fittings would be next to
/// daylight, so their warm pools read against the cool light from the roof:
/// an overcast afternoon, not noon.
const LAMP_GAIN: f32 = 40.0;
/// Exposure of the lab cameras. A real hall under these lamps sits around
/// EV 8.
const EV100: f32 = 9.2;

/// F4 (by default) steps through the quality presets.
pub(crate) fn quality_key(keys: Res<ButtonInput<KeyCode>>, mut settings: ResMut<LabSettings>) {
    if settings.quality_key.is_some_and(|k| keys.just_pressed(k)) {
        let q = settings.quality.next();
        settings.set_quality(q);
        info!("lab quality: {}", q.name());
    }
}

/// Spawns the hall's lights, again whenever the look changes.
pub(crate) fn light_hall(
    mut commands: Commands,
    settings: Res<LabSettings>,
    old: Query<Entity, With<HallLight>>,
    mut stats: ResMut<LabStats>,
    mut fitted: Local<Option<Look>>,
    mut ambient: ResMut<GlobalAmbientLight>,
) {
    let look = settings.look;
    if *fitted == Some(look) {
        return;
    }
    *fitted = Some(look);
    for e in &old {
        commands.entity(e).despawn();
    }
    let k = settings.model_scale;

    commands.insert_resource(DirectionalLightShadowMap {
        size: look.shadow_map,
    });
    commands.insert_resource(PointLightShadowMap {
        size: look.shadow_map,
    });
    // With reflections on, the environment map is the ambient light.
    *ambient = GlobalAmbientLight {
        color: Color::srgb(0.82, 0.88, 0.95),
        brightness: if look.reflections > 0 { 0.0 } else { 110.0 },
        ..default()
    };

    // Daylight through the roof lights: the key light on the table, cool
    // and from high up. Its cascades are sized for the table, and only
    // layer 0 (table and structure) casts into them.
    commands.spawn((
        DirectionalLight {
            color: Color::srgb(0.90, 0.95, 1.0),
            illuminance: 1_300.0,
            shadow_maps_enabled: true,
            contact_shadows_enabled: look.contact_shadows,
            ..default()
        },
        CascadeShadowConfigBuilder {
            num_cascades: look.cascades.max(1),
            minimum_distance: 0.5,
            first_cascade_far_bound: 22.0,
            maximum_distance: 160.0,
            overlap_proportion: 0.25,
        }
        .build(),
        Transform::from_xyz(-3.5, 10.0, 4.5).looking_at(Vec3::ZERO, Vec3::Y),
        RenderLayers::layer(0),
        HallLight,
    ));

    // High-bay lamps: warm, shining down. Those nearest the table come
    // first, so a low setting still lights where the camera looks. Real
    // 20 klm fittings, scaled with the square of the model scale so the
    // hall is lit as a hall would be.
    let mut lamps = settings.hall.lamp_positions();
    let table = Vec3::new(settings.table_at.x, 0.0, settings.table_at.y);
    lamps.sort_by(|a, b| {
        let d = |p: &Vec3| Vec2::new(p.x - table.x, p.z - table.z).length_squared();
        d(a).total_cmp(&d(b))
    });
    let lit = look.lamp_lights.min(lamps.len());
    // Fewer lamps each carry more, so the hall keeps its level.
    let share = (lamps.len() as f32 / lit.max(1) as f32).sqrt();
    for (i, p) in lamps.iter().take(lit).enumerate() {
        let at = settings.lab_to_world(*p - Vec3::Y * 0.05);
        commands.spawn((
            SpotLight {
                color: Color::srgb(1.0, 0.92, 0.80),
                intensity: 20_000.0 * k * k * LAMP_GAIN * share,
                range: 34.0 * k,
                radius: 0.3 * k,
                outer_angle: 0.95,
                inner_angle: 0.2,
                shadow_maps_enabled: i < look.lamp_shadows,
                shadow_depth_bias: 3.0,
                shadow_normal_bias: 2.4,
                shadow_map_near_z: 1.5 * k,
                ..default()
            },
            Transform::from_translation(at).looking_at(at - Vec3::Y, Vec3::X),
            RenderLayers::from_layers(&[0, HALL_LAYER]),
            HallLight,
        ));
    }
    stats.point_lights = lit;
    stats.shadowed_lamps = look.lamp_shadows.min(lit);
    stats.quality = settings.quality.name();
}

/// Bakes the reflection maps when the look asks for a new size.
pub(crate) fn bake_environment(
    settings: Res<LabSettings>,
    mut env: ResMut<LabEnvironment>,
    mut images: ResMut<Assets<Image>>,
    mut stats: ResMut<LabStats>,
) {
    let size = settings.look.reflections;
    if env.size == size && (size == 0 || env.maps.is_some()) {
        return;
    }
    env.size = size;
    if size == 0 {
        env.maps = None;
        return;
    }
    let t0 = std::time::Instant::now();
    let maps = envmap::bake(
        &HallRadiance::new(&settings.hall, settings.lab_origin()),
        size,
    );
    env.maps = Some((
        images.add(maps.diffuse_image()),
        images.add(maps.specular_image()),
    ));
    stats.environment_bake_ms = t0.elapsed().as_secs_f32() * 1000.0;
}

type Stack = (
    Bloom,
    Vignette,
    DepthOfField,
    ScreenSpaceAmbientOcclusion,
    ContactShadows,
    EnvironmentMapLight,
    Fxaa,
    Smaa,
    TemporalAntiAliasing,
    ContrastAdaptiveSharpening,
);

/// Fits lab cameras with the look, again whenever it changes, and keeps the
/// focus on target.
#[allow(clippy::type_complexity)]
pub(crate) fn lab_lens(
    mut commands: Commands,
    settings: Res<LabSettings>,
    env: Res<LabEnvironment>,
    mut cams: Query<(
        Entity,
        &LabCamera,
        Option<&mut DepthOfField>,
        Option<&mut ScreenSpaceAmbientOcclusion>,
        Option<&Fitted>,
        Has<EnvironmentMapLight>,
    )>,
) {
    let k = settings.model_scale;
    let look = settings.look;
    let want = Fitted(look, settings.f_stops);
    for (e, cam, dof, ao, fitted, has_env) in &mut cams {
        // The maps are baked a frame after the look changes.
        let env_ready = look.reflections == 0 || env.maps.is_some();
        let env_stale = (look.reflections > 0) != has_env;
        if fitted != Some(&want) || (env_ready && env_stale) {
            let mut c = commands.entity(e);
            c.remove::<Stack>();
            c.insert((
                want.clone(),
                Msaa::Off,
                Exposure { ev100: EV100 },
                Tonemapping::TonyMcMapface,
                grade(),
                Projection::Perspective(PerspectiveProjection {
                    far: 6.0 * settings.hall.length() * k,
                    ..default()
                }),
                RenderLayers::from_layers(&[0, HALL_LAYER]),
                // Haze: the far end of the hall goes a little pale and
                // cool, which is most of what makes it read as large.
                DistanceFog {
                    color: Color::srgba(0.40, 0.45, 0.52, 0.36),
                    falloff: FogFalloff::Linear {
                        start: 12.0 * k,
                        end: 70.0 * k,
                    },
                    ..default()
                },
            ));
            if look.bloom {
                // Only what is far brighter than a lit surface blooms:
                // lamp faces, roof lights, indicator lamps, hot glints.
                c.insert(Bloom {
                    intensity: 0.11,
                    low_frequency_boost: 0.55,
                    low_frequency_boost_curvature: 0.9,
                    high_pass_frequency: 1.0,
                    prefilter: BloomPrefilter {
                        threshold: 1.1,
                        threshold_softness: 0.6,
                    },
                    ..Bloom::NATURAL
                });
            }
            if look.vignette {
                c.insert(Vignette {
                    intensity: 0.32,
                    radius: 0.82,
                    smoothness: 0.75,
                    roundness: 0.8,
                    color: Color::srgb(0.01, 0.012, 0.02),
                    ..default()
                });
            }
            if let Some(quality_level) = look.ambient_occlusion {
                let radius = ao_radius(cam.focus_distance);
                c.insert(ScreenSpaceAmbientOcclusion {
                    quality_level,
                    radius,
                    constant_object_thickness: 0.4 * radius,
                });
            }
            if look.contact_shadows {
                c.insert(ContactShadows {
                    linear_steps: 12,
                    thickness: 0.12,
                    length: 0.35,
                });
            }
            match look.anti_alias {
                AntiAlias::Off => {}
                AntiAlias::Fxaa => {
                    c.insert(Fxaa::default());
                }
                AntiAlias::Smaa => {
                    c.insert(Smaa {
                        preset: SmaaPreset::High,
                    });
                }
                AntiAlias::Taa => {
                    c.insert((
                        TemporalAntiAliasing::default(),
                        ContrastAdaptiveSharpening {
                            enabled: true,
                            sharpening_strength: 0.35,
                            denoise: false,
                        },
                    ));
                }
            }
            if let Some((diffuse, specular)) = env.maps.clone().filter(|_| look.reflections > 0) {
                c.insert(EnvironmentMapLight {
                    diffuse_map: diffuse,
                    specular_map: specular,
                    intensity: ENV_INTENSITY,
                    ..default()
                });
            }
            if let Some(mode) = look.dof {
                c.insert(lens(mode, cam.focus_distance, settings.f_stops, k));
            }
        } else {
            if let Some(mut d) = dof {
                if d.focal_distance != cam.focus_distance {
                    d.focal_distance = cam.focus_distance;
                }
            }
            if let Some(mut ao) = ao {
                let radius = ao_radius(cam.focus_distance);
                if (ao.radius - radius).abs() > 0.02 * radius {
                    ao.radius = radius;
                    ao.constant_object_thickness = 0.4 * radius;
                }
            }
        }
    }
}

/// Ambient occlusion reach, world units, for a camera focused `focus` away.
///
/// The structure is full size and the hall is 20 times that, so no single
/// reach suits both. It follows what the camera is looking at: about 0.8 m
/// up close, where it sits in the joints of the structure, growing to 12
/// (0.6 m of hall) when the camera pulls back, where it grounds machinery on
/// the floor. A reach that stays small at a distance would also sink below
/// the precision of the occlusion pass's depth buffer and turn to speckle.
fn ao_radius(focus: f32) -> f32 {
    (0.07 * focus).clamp(0.6, 12.0)
}

fn lens(mode: DepthOfFieldMode, focus: f32, f_stops: f32, k: f32) -> DepthOfField {
    DepthOfField {
        mode,
        focal_distance: focus,
        // A sensor scaled with the model: the blur of a real camera
        // looking at a real 1:20 model.
        sensor_height: SENSOR_HEIGHT * k,
        aperture_f_stops: f_stops,
        max_depth: 200.0 * k,
        ..default()
    }
}

/// The grade: a touch of contrast, shadows left a little cool and open,
/// highlights a little warm. Enough to tie the picture together, not
/// enough to notice.
fn grade() -> ColorGrading {
    ColorGrading {
        global: ColorGradingGlobal {
            exposure: 0.0,
            temperature: 0.02,
            post_saturation: 1.04,
            ..default()
        },
        shadows: ColorGradingSection {
            saturation: 0.92,
            contrast: 1.04,
            lift: 0.004,
            ..default()
        },
        midtones: ColorGradingSection {
            contrast: 1.06,
            ..default()
        },
        highlights: ColorGradingSection {
            saturation: 1.03,
            gain: 1.03,
            ..default()
        },
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn presets_only_get_dearer() {
        let looks: Vec<Look> = LabQuality::ALL.iter().map(|q| q.look()).collect();
        for w in looks.windows(2) {
            assert!(w[0].shadow_map <= w[1].shadow_map);
            assert!(w[0].cascades <= w[1].cascades);
            assert!(w[0].lamp_lights <= w[1].lamp_lights);
            assert!(w[0].lamp_shadows <= w[1].lamp_shadows);
            assert!(w[0].reflections <= w[1].reflections);
        }
        for l in &looks {
            assert!(l.lamp_shadows <= l.lamp_lights);
            assert!(l.shadow_map.is_power_of_two() && l.reflections.is_power_of_two());
        }
        assert!(looks[0].ambient_occlusion.is_none());
    }

    #[test]
    fn quality_names_round_trip_and_cycle() {
        let mut q = LabQuality::Low;
        for _ in 0..LabQuality::ALL.len() {
            assert_eq!(LabQuality::parse(q.name()), Some(q));
            q = q.next();
        }
        assert_eq!(q, LabQuality::Low);
    }
}
