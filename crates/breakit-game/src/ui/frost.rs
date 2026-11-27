//! Frosted backdrops: every UI panel darkens and blurs the scene behind
//! it, so its text reads over a bright hall or a busy structure.
//!
//! Two halves:
//!
//! - A render pass on the game camera, after its post-processing and
//!   before the UI is drawn, copies the frame into a quarter-size texture
//!   ([`FROST_IMAGE`]) and blurs it (a 4x box downsample, then a
//!   separable Gaussian). The scene itself is not touched.
//! - Panels carry [`MaterialNode<FrostMaterial>`] (see
//!   [`super::widgets::panel`]). The material samples that texture at the
//!   panel's screen position, darkens it with the theme's tint and draws
//!   the rounded corners and a hairline edge.
//!
//! The panel material and both blur textures have fixed handles, so a
//! panel can be spawned anywhere without looking anything up, and the
//! render world finds the textures the same way.

use bevy::asset::{embedded_asset, load_embedded_asset, uuid_handle, RenderAssetUsages};
use bevy::core_pipeline::schedule::{Core3d, Core3dSystems};
use bevy::core_pipeline::FullscreenShader;
use bevy::image::ImageSampler;
use bevy::prelude::*;
use bevy::render::extract_component::{ExtractComponent, ExtractComponentPlugin};
use bevy::render::render_asset::RenderAssets;
use bevy::render::render_resource::binding_types::{sampler, texture_2d};
use bevy::render::render_resource::*;
use bevy::render::renderer::{RenderContext, RenderDevice, ViewQuery};
use bevy::render::texture::GpuImage;
use bevy::render::view::ViewTarget;
use bevy::render::{RenderApp, RenderStartup};
use bevy::shader::{Shader, ShaderRef};
use bevy::window::{PrimaryWindow, WindowResized};

use super::theme::Themes;
use crate::camera::OrbitCamera;

/// The blurred scene the panels sample.
pub const FROST_IMAGE: Handle<Image> = uuid_handle!("6c1f3f0e-8a52-4a8e-9a43-1f6f0b8c2d11");
/// Scratch texture for the horizontal blur.
const FROST_SCRATCH: Handle<Image> = uuid_handle!("2b7d5e64-0c3a-4f5e-8f8e-61c9a7d3b402");
/// The one panel material; its tint follows the theme.
pub const FROST_MATERIAL: Handle<FrostMaterial> =
    uuid_handle!("9e0a4c7b-5d21-4b6f-a3d8-7c2e19f04a53");

/// The blur texture is this many times smaller than the window.
const DOWNSAMPLE: u32 = 4;
const FORMAT: TextureFormat = TextureFormat::Rgba16Float;

pub struct FrostPlugin;

impl Plugin for FrostPlugin {
    fn build(&self, app: &mut App) {
        embedded_asset!(app, "frost.wesl");
        embedded_asset!(app, "frost_blur.wesl");
        app.add_plugins((
            UiMaterialPlugin::<FrostMaterial>::default(),
            ExtractComponentPlugin::<FrostSource>::default(),
        ))
        .add_systems(Startup, setup_frost)
        .add_systems(Update, (tag_camera, resize_frost, follow_theme));
        let Some(render_app) = app.get_sub_app_mut(RenderApp) else {
            return;
        };
        render_app
            .add_systems(RenderStartup, init_blur_pipeline)
            .add_systems(
                Core3d,
                frost_blur
                    .after(Core3dSystems::PostProcess)
                    .before(bevy::ui_render::ui_pass),
            );
    }
}

/// Marks the camera whose frame the panels blur.
#[derive(Component, Clone, Copy, Default, ExtractComponent)]
#[extract_app(RenderApp)]
pub struct FrostSource;

/// A panel's backdrop: the blurred scene under a tint, rounded like the
/// node, with a hairline edge.
#[derive(Asset, TypePath, AsBindGroup, Debug, Clone)]
pub struct FrostMaterial {
    /// Colour laid over the blurred scene; alpha is how much of it.
    #[uniform(0)]
    pub tint: LinearRgba,
    /// The hairline along the panel's edge; alpha is its strength.
    #[uniform(0)]
    pub edge: LinearRgba,
    #[texture(1)]
    #[sampler(2)]
    pub blur: Handle<Image>,
}

impl UiMaterial for FrostMaterial {
    fn fragment_shader() -> ShaderRef {
        "embedded://breakit_game/ui/frost.wesl".into()
    }

    /// Under the node's own background and border.
    fn stack_z_offset() -> f32 {
        -0.05
    }
}

fn blur_image(window: UVec2) -> Image {
    let size = (window / DOWNSAMPLE).max(UVec2::splat(16));
    let mut image = Image::new_target_texture(size.x, size.y, FORMAT, None);
    image.asset_usage = RenderAssetUsages::RENDER_WORLD;
    image.sampler = ImageSampler::linear();
    image
}

fn window_size(window: &Window) -> UVec2 {
    UVec2::new(window.physical_width(), window.physical_height()).max(UVec2::ONE)
}

fn setup_frost(
    mut images: ResMut<Assets<Image>>,
    mut materials: ResMut<Assets<FrostMaterial>>,
    themes: Res<Themes>,
    window: Query<&Window, With<PrimaryWindow>>,
) {
    let size = window
        .single()
        .map(window_size)
        .unwrap_or(UVec2::new(1280, 720));
    let _ = images.insert(&FROST_IMAGE, blur_image(size));
    let _ = images.insert(&FROST_SCRATCH, blur_image(size));
    let t = themes.current();
    let _ = materials.insert(
        &FROST_MATERIAL,
        FrostMaterial {
            tint: t.backdrop.into(),
            edge: t.backdrop_edge.into(),
            blur: FROST_IMAGE,
        },
    );
}

fn tag_camera(
    mut commands: Commands,
    cams: Query<Entity, (With<OrbitCamera>, Without<FrostSource>)>,
) {
    for e in &cams {
        commands.entity(e).insert(FrostSource);
    }
}

/// Keeps the blur textures at a quarter of the window.
fn resize_frost(
    mut resized: MessageReader<WindowResized>,
    window: Query<&Window, With<PrimaryWindow>>,
    mut images: ResMut<Assets<Image>>,
    mut materials: ResMut<Assets<FrostMaterial>>,
) {
    if resized.read().count() == 0 {
        return;
    }
    let Ok(window) = window.single() else {
        return;
    };
    let size = window_size(window);
    let want = (size / DOWNSAMPLE).max(UVec2::splat(16));
    if images.get(&FROST_IMAGE).is_some_and(|i| i.size() == want) {
        return;
    }
    let _ = images.insert(&FROST_IMAGE, blur_image(size));
    let _ = images.insert(&FROST_SCRATCH, blur_image(size));
    // Rebind the panels to the new texture.
    let _ = materials.get_mut(&FROST_MATERIAL);
}

fn follow_theme(themes: Res<Themes>, mut materials: ResMut<Assets<FrostMaterial>>) {
    if !themes.is_changed() {
        return;
    }
    let t = themes.current();
    if let Some(mut m) = materials.get_mut(&FROST_MATERIAL) {
        m.tint = t.backdrop.into();
        m.edge = t.backdrop_edge.into();
    }
}

// ------------------------------------------------------------ render world

#[derive(Resource)]
struct BlurPipeline {
    layout: BindGroupLayoutDescriptor,
    sampler: Sampler,
    /// Downsample, horizontal and vertical blur.
    passes: [CachedRenderPipelineId; 3],
}

fn init_blur_pipeline(
    mut commands: Commands,
    device: Res<RenderDevice>,
    fullscreen: Res<FullscreenShader>,
    assets: Res<AssetServer>,
    cache: Res<PipelineCache>,
) {
    let layout = BindGroupLayoutDescriptor::new(
        "frost_blur_layout",
        &BindGroupLayoutEntries::sequential(
            ShaderStages::FRAGMENT,
            (
                texture_2d(TextureSampleType::Float { filterable: true }),
                sampler(SamplerBindingType::Filtering),
            ),
        ),
    );
    let sampler = device.create_sampler(&SamplerDescriptor {
        mag_filter: FilterMode::Linear,
        min_filter: FilterMode::Linear,
        ..default()
    });
    let shader: Handle<Shader> = load_embedded_asset!(assets.as_ref(), "frost_blur.wesl");
    let pass = |entry: &'static str| {
        cache.queue_render_pipeline(RenderPipelineDescriptor {
            label: Some(format!("frost_{entry}").into()),
            layout: vec![layout.clone()],
            vertex: fullscreen.to_vertex_state(),
            fragment: Some(FragmentState {
                shader: shader.clone(),
                entry_point: Some(entry.into()),
                targets: vec![Some(ColorTargetState {
                    format: FORMAT,
                    blend: None,
                    write_mask: ColorWrites::ALL,
                })],
                ..default()
            }),
            ..default()
        })
    };
    let passes = [pass("down"), pass("blur_h"), pass("blur_v")];
    commands.insert_resource(BlurPipeline {
        layout,
        sampler,
        passes,
    });
}

/// Frame -> quarter size -> horizontal blur -> vertical blur, ending in
/// [`FROST_IMAGE`] for the UI pass that follows.
fn frost_blur(
    view: ViewQuery<&ViewTarget, With<FrostSource>>,
    pipeline: Option<Res<BlurPipeline>>,
    cache: Res<PipelineCache>,
    images: Res<RenderAssets<GpuImage>>,
    mut ctx: RenderContext,
) {
    let target = view.into_inner();
    let Some(pipeline) = pipeline else {
        return;
    };
    let (Some(out), Some(scratch)) = (images.get(&FROST_IMAGE), images.get(&FROST_SCRATCH)) else {
        return;
    };
    let Some(passes) = pipeline
        .passes
        .iter()
        .map(|id| cache.get_render_pipeline(*id))
        .collect::<Option<Vec<_>>>()
    else {
        return;
    };
    let layout = cache.get_bind_group_layout(&pipeline.layout);
    // With MSAA the resolved frame; otherwise the frame itself.
    let source = target
        .sampled_main_texture_view()
        .unwrap_or(target.main_texture_view());
    let steps = [
        (source, &out.texture_view),
        (&out.texture_view, &scratch.texture_view),
        (&scratch.texture_view, &out.texture_view),
    ];
    for ((src, dst), pass) in steps.into_iter().zip(passes) {
        let bind = ctx.render_device().create_bind_group(
            Some("frost_blur"),
            &layout,
            &BindGroupEntries::sequential((src, &pipeline.sampler)),
        );
        let mut rp = ctx
            .command_encoder()
            .begin_render_pass(&RenderPassDescriptor {
                label: Some("frost_blur"),
                color_attachments: &[Some(RenderPassColorAttachment {
                    view: dst,
                    depth_slice: None,
                    resolve_target: None,
                    ops: Operations::default(),
                })],
                depth_stencil_attachment: None,
                timestamp_writes: None,
                occlusion_query_set: None,
                multiview_mask: None,
            });
        rp.set_pipeline(pass);
        rp.set_bind_group(0, &bind, &[]);
        rp.draw(0..3, 0..1);
    }
}
