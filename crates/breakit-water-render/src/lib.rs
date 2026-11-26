//! Draws the water of `breakit-water`: the simulated surface with
//! refraction, depth colour, reflections, caustics and foam, and the spray.
//!
//! Put the simulation in the [`LiveWater`] resource and step it yourself
//! (before [`WaterRenderSet`]); [`WaterRenderPlugin`] draws whatever is in
//! it each frame and removes its entities when the resource goes away.
//! [`WaterLook`] holds the quality knobs; [`WaterQuality`] gives the four
//! presets that match the lab's.
//!
//! The surface is a mesh rebuilt on the CPU from the solver's cells each
//! frame (`surface.rs`): at the game's grid that is 17 k vertices and well
//! under a millisecond. Everything that makes it look like water happens
//! in `water.wesl`.

mod spray;
pub mod surface;

use bevy::asset::{embedded_asset, RenderAssetUsages};
use bevy::camera::visibility::NoFrustumCulling;
use bevy::mesh::{Indices, PrimitiveTopology};
use bevy::pbr::{
    ExtendedMaterial, MaterialExtension, MaterialPlugin, ScreenSpaceTransmission,
    ScreenSpaceTransmissionQuality,
};
use bevy::prelude::*;
use bevy::render::render_resource::AsBindGroup;
use bevy::shader::ShaderRef;
use breakit_water::WaterSim;

use surface::{lattice_indices, SurfaceBuffers};

const SHADER_PATH: &str = "embedded://breakit_water_render/water.wesl";

/// Radiance of the light scattered back out of deep water, in the units
/// of `StandardMaterial::emissive`, tuned against the lab's lighting.
const SCATTER: f32 = 40.0;

/// The material of the water surface.
pub type WaterMaterial = ExtendedMaterial<StandardMaterial, WaterSurface>;

/// The water to draw. Whoever owns the scenario inserts it, advances
/// `sim` each frame and removes it when the table has no water.
#[derive(Resource)]
pub struct LiveWater {
    /// The simulation.
    pub sim: WaterSim,
    /// True if the low-x edge of the grid is a paddle face: the edge of
    /// the surface then follows the paddle's displacement.
    pub paddle: bool,
}

/// The systems that draw the water, in `Update`. Step the simulation
/// before this set.
#[derive(SystemSet, Debug, Clone, PartialEq, Eq, Hash)]
pub struct WaterRenderSet;

/// Quality presets, matching the lab's.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum WaterQuality {
    /// Alpha-blended surface, no ripples, no spray.
    Low,
    /// Refraction and ripples, modest spray.
    Medium,
    /// Adds caustics and a finer surface.
    #[default]
    High,
    /// The finest surface and the most spray.
    Ultra,
}

impl WaterQuality {
    /// The knobs of this preset.
    pub fn look(self) -> WaterLook {
        match self {
            WaterQuality::Low => WaterLook {
                upsample: 1,
                refraction: false,
                caustics: false,
                detail: false,
                droplets: 0,
            },
            WaterQuality::Medium => WaterLook {
                upsample: 1,
                refraction: true,
                caustics: false,
                detail: true,
                droplets: 800,
            },
            WaterQuality::High => WaterLook {
                upsample: 2,
                refraction: true,
                caustics: true,
                detail: true,
                droplets: 2500,
            },
            WaterQuality::Ultra => WaterLook {
                upsample: 3,
                refraction: true,
                caustics: true,
                detail: true,
                droplets: 4000,
            },
        }
    }
}

/// How the water is drawn.
#[derive(Resource, Clone, Copy, Debug, PartialEq)]
pub struct WaterLook {
    /// Surface vertices per solver cell, along each axis.
    pub upsample: usize,
    /// Screen-space refraction with depth colour (true) or plain alpha
    /// blending (false).
    pub refraction: bool,
    /// Caustics on the bed.
    pub caustics: bool,
    /// Ripples and broken-up foam.
    pub detail: bool,
    /// Most spray droplets drawn.
    pub droplets: usize,
}

impl Default for WaterLook {
    fn default() -> Self {
        WaterQuality::default().look()
    }
}

/// Uniforms of `water.wesl`, in the order of its `WaterLook` struct.
#[derive(Asset, AsBindGroup, Reflect, Debug, Clone, Default)]
pub struct WaterSurface {
    /// x: detail, y: caustics, z: refraction (each 0 or 1), w: the
    /// simulation's time, s, which moves the ripples and caustics (so a
    /// paused or slowed test pauses or slows them too).
    #[uniform(100)]
    pub params: Vec4,
    /// rgb: deep-water colour; a: its opacity when blending, or the
    /// radiance of its scattered light when refracting.
    #[uniform(100)]
    pub tint: Vec4,
}

impl MaterialExtension for WaterSurface {
    fn fragment_shader() -> ShaderRef {
        SHADER_PATH.into()
    }

    // The surface is see-through: it stays out of the depth prepass (so
    // ambient occlusion and depth of field see the bed) and casts no shadow.
    fn enable_prepass() -> bool {
        false
    }

    fn enable_shadows() -> bool {
        false
    }
}

/// The material for `look`.
pub fn water_material(look: &WaterLook) -> WaterMaterial {
    let flag = |on: bool| if on { 1.0 } else { 0.0 };
    let base = if look.refraction {
        StandardMaterial {
            base_color: Color::WHITE,
            perceptual_roughness: 0.035,
            // Water reflects 2 % head-on.
            reflectance: 0.35,
            specular_transmission: 1.0,
            ior: 1.333,
            thickness: 1.0,
            // Clear flume water: a green-blue cast that builds over metres.
            attenuation_color: Color::srgb(0.48, 0.80, 0.84),
            attenuation_distance: 3.2,
            ..default()
        }
    } else {
        StandardMaterial {
            base_color: Color::srgba(0.16, 0.36, 0.46, 0.7),
            perceptual_roughness: 0.06,
            reflectance: 0.35,
            alpha_mode: AlphaMode::Blend,
            ..default()
        }
    };
    ExtendedMaterial {
        base,
        extension: WaterSurface {
            params: Vec4::new(
                flag(look.detail),
                flag(look.caustics),
                flag(look.refraction),
                0.0,
            ),
            tint: if look.refraction {
                Vec4::new(0.05, 0.19, 0.21, SCATTER)
            } else {
                Vec4::new(0.05, 0.16, 0.2, 0.78)
            },
        },
    }
}

/// Draws [`LiveWater`].
pub struct WaterRenderPlugin;

impl Plugin for WaterRenderPlugin {
    fn build(&self, app: &mut App) {
        embedded_asset!(app, "water.wesl");
        app.add_plugins(MaterialPlugin::<WaterMaterial>::default())
            .init_resource::<WaterLook>()
            .init_resource::<Drawn>()
            .add_systems(
                Update,
                (sync_entities, sync_surface, sync_spray, sync_cameras)
                    .chain()
                    .in_set(WaterRenderSet),
            );
    }
}

/// Marks the surface mesh entity.
#[derive(Component)]
struct SurfaceMesh;

/// What is spawned for the current water, and the scratch buffers.
#[derive(Resource, Default)]
struct Drawn {
    surface: Option<Entity>,
    spray: Option<Entity>,
    /// Lattice size and look the entities were built for.
    built: Option<((usize, usize), WaterLook)>,
    buffers: SurfaceBuffers,
}

fn lattice(water: &LiveWater, look: &WaterLook) -> (usize, usize) {
    let g = water.sim.solver.grid;
    let up = look.upsample.max(1);
    (g.nx * up + 1, g.nz * up + 1)
}

fn sync_entities(
    mut commands: Commands,
    water: Option<Res<LiveWater>>,
    look: Res<WaterLook>,
    mut drawn: ResMut<Drawn>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<WaterMaterial>>,
    mut plain: ResMut<Assets<StandardMaterial>>,
) {
    let want = water.as_ref().map(|w| (lattice(w, &look), *look));
    if drawn.built == want {
        return;
    }
    for e in [drawn.surface.take(), drawn.spray.take()]
        .into_iter()
        .flatten()
    {
        commands.entity(e).despawn();
    }
    drawn.built = want;
    let Some(((vx, vz), look)) = want else {
        return;
    };
    let mut mesh = Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::default(),
    );
    let n = vx * vz;
    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, vec![[0.0f32; 3]; n]);
    mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, vec![[0.0f32, 1.0, 0.0]; n]);
    mesh.insert_attribute(Mesh::ATTRIBUTE_UV_0, vec![[0.0f32; 2]; n]);
    mesh.insert_attribute(Mesh::ATTRIBUTE_COLOR, vec![[0.0f32; 4]; n]);
    mesh.insert_indices(Indices::U32(lattice_indices(vx, vz)));
    drawn.surface = Some(
        commands
            .spawn((
                Mesh3d(meshes.add(mesh)),
                MeshMaterial3d(materials.add(water_material(&look))),
                Transform::IDENTITY,
                // The vertices move every frame; the bounds would be stale.
                NoFrustumCulling,
                bevy::light::NotShadowCaster,
                SurfaceMesh,
            ))
            .id(),
    );
    if look.droplets > 0 {
        drawn.spray = Some(spray::spawn(&mut commands, &mut meshes, &mut plain));
    }
}

fn sync_surface(
    water: Option<Res<LiveWater>>,
    look: Res<WaterLook>,
    mut drawn: ResMut<Drawn>,
    surface: Query<(&Mesh3d, &MeshMaterial3d<WaterMaterial>), With<SurfaceMesh>>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<WaterMaterial>>,
) {
    let (Some(water), Ok((handle, material))) = (water, surface.single()) else {
        return;
    };
    if let Some(mut m) = materials.get_mut(&material.0) {
        // Wrapped so the shader's f32 keeps its precision over a long test.
        m.extension.params.w = (water.sim.solver.time() % 3600.0) as f32;
    }
    let field = water.sim.field();
    let shift = if water.paddle {
        water.sim.drive().paddle_position as f32
    } else {
        0.0
    };
    let buffers = &mut drawn.buffers;
    buffers.fill(&field, water.sim.foam(), look.upsample, shift);
    let Some(mut mesh) = meshes.get_mut(&handle.0) else {
        return;
    };
    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, buffers.positions.clone());
    mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, buffers.normals.clone());
    mesh.insert_attribute(Mesh::ATTRIBUTE_UV_0, buffers.uvs.clone());
    mesh.insert_attribute(Mesh::ATTRIBUTE_COLOR, buffers.colors.clone());
}

fn sync_spray(
    water: Option<Res<LiveWater>>,
    look: Res<WaterLook>,
    cloud: Query<&Mesh3d, With<spray::SprayCloud>>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut was_empty: Local<bool>,
) {
    let (Some(water), Ok(handle)) = (water, cloud.single()) else {
        return;
    };
    let droplets = water.sim.droplets();
    if droplets.is_empty() && *was_empty {
        return;
    }
    *was_empty = droplets.is_empty();
    if let Some(mut mesh) = meshes.get_mut(&handle.0) {
        spray::fill(&mut mesh, droplets, look.droplets);
    }
}

/// Refraction needs the camera to keep a copy of what is behind the
/// water; give every 3D camera that setting while water is drawn.
fn sync_cameras(
    mut commands: Commands,
    water: Option<Res<LiveWater>>,
    look: Res<WaterLook>,
    cameras: Query<(Entity, Has<ScreenSpaceTransmission>), With<Camera3d>>,
) {
    if water.is_none() || !look.refraction {
        return;
    }
    for (e, has) in &cameras {
        if !has {
            commands.entity(e).insert(ScreenSpaceTransmission {
                steps: 1,
                quality: ScreenSpaceTransmissionQuality::Medium,
            });
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn presets_step_up() {
        let all = [
            WaterQuality::Low,
            WaterQuality::Medium,
            WaterQuality::High,
            WaterQuality::Ultra,
        ];
        for pair in all.windows(2) {
            let (a, b) = (pair[0].look(), pair[1].look());
            assert!(b.upsample >= a.upsample && b.droplets >= a.droplets);
            assert!(b.refraction >= a.refraction && b.caustics >= a.caustics);
        }
        assert!(!WaterQuality::Low.look().refraction);
    }

    #[test]
    fn refraction_uses_transmission_and_blending_uses_alpha() {
        let m = water_material(&WaterQuality::High.look());
        assert!(m.base.specular_transmission > 0.0);
        assert_eq!(m.extension.params.z, 1.0);
        let m = water_material(&WaterQuality::Low.look());
        assert_eq!(m.base.specular_transmission, 0.0);
        assert!(matches!(m.base.alpha_mode, AlphaMode::Blend));
    }
}
