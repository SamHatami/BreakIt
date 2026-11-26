//! Bevy renderer for BreakIt structures.
//!
//! Members are never re-meshed. Each (profile, detail) pair has one template
//! mesh, every member is an instance of it, and a vertex shader bends each
//! instance along its member's curve from a 176-byte record in one storage
//! buffer (`member.wesl`). Per frame the CPU only rewrites that buffer, so
//! the cost of a member does not depend on how many vertices it has, and
//! Bevy batches all members sharing a template into one draw.
//!
//! # Use
//!
//! ```ignore
//! app.add_plugins(BreakitRenderPlugin);
//! // Whenever the design changes:
//! commands.insert_resource(StructureVisual::new(&structure, &styles));
//! // Every frame, after physics:
//! frame.0 = visual.frame_from_sim(&sim);            // simulate mode
//! frame.0 = visual.frame_from_positions(&pos, &u);  // build mode
//! ```
//!
//! The plugin's systems run in `PostUpdate` in [`BreakitRenderSet`].

use std::collections::HashMap;

use bevy::asset::{embedded_asset, RenderAssetUsages};
use bevy::camera::visibility::NoFrustumCulling;
use bevy::mesh::{Indices, MeshTag, PrimitiveTopology};
use bevy::pbr::{ExtendedMaterial, MaterialExtension};
use bevy::prelude::*;
use bevy::render::render_resource::AsBindGroup;
use bevy::render::storage::ShaderBuffer;
use bevy::shader::ShaderRef;

use breakit_core::{DynamicSim, Structure};
use breakit_mesh::fracture::PiecePart;
use breakit_mesh::mesh::{member_template, MeshData, TemplateOptions};
use breakit_mesh::pose::{flags, GpuMember};
use breakit_mesh::{
    MaterialKind, MemberStyle, Profile, SceneConfig, VisualFrame, VisualScene, VisualStructure,
};

pub use breakit_mesh;

mod particles;

const SHADER_PATH: &str = "embedded://breakit_render/member.wesl";

pub struct BreakitRenderPlugin;

/// The renderer's systems, in `PostUpdate`.
#[derive(SystemSet, Debug, Clone, PartialEq, Eq, Hash)]
pub struct BreakitRenderSet;

impl Plugin for BreakitRenderPlugin {
    fn build(&self, app: &mut App) {
        embedded_asset!(app, "member.wesl");
        app.add_plugins(MaterialPlugin::<MemberMaterial>::default())
            .init_resource::<MemberFrame>()
            .init_resource::<RenderSettings>()
            .init_resource::<RenderStats>()
            .add_systems(Startup, setup_assets)
            .add_systems(
                PostUpdate,
                (
                    advance_scene,
                    sync_instances,
                    sync_debris,
                    particles::sync_particles,
                )
                    .chain()
                    .in_set(BreakitRenderSet)
                    .before(TransformSystems::Propagate),
            );
        particles::build(app);
    }
}

/// Material for every member: `StandardMaterial` lighting with the member
/// shader's deformation and procedural surface.
pub type MemberMaterial = ExtendedMaterial<StandardMaterial, MemberExtension>;

#[derive(Asset, AsBindGroup, Reflect, Debug, Clone, Default)]
pub struct MemberExtension {
    /// One `GpuMember` per slot, indexed by each instance's `MeshTag`.
    #[storage(100, read_only)]
    pub members: Handle<ShaderBuffer>,
}

impl MaterialExtension for MemberExtension {
    fn vertex_shader() -> ShaderRef {
        SHADER_PATH.into()
    }
    fn fragment_shader() -> ShaderRef {
        SHADER_PATH.into()
    }
    fn prepass_vertex_shader() -> ShaderRef {
        SHADER_PATH.into()
    }
}

/// The structure being drawn. Insert a new one whenever the design changes.
#[derive(Resource)]
pub struct StructureVisual {
    pub scene: VisualScene,
    generation: u64,
}

impl StructureVisual {
    /// One style per member of `structure`.
    pub fn new(structure: &Structure, styles: &[MemberStyle]) -> Self {
        Self::from_scene(VisualScene::new(
            VisualStructure::new(structure, styles),
            SceneConfig::default(),
        ))
    }

    pub fn from_scene(scene: VisualScene) -> Self {
        use std::sync::atomic::{AtomicU64, Ordering};
        static NEXT: AtomicU64 = AtomicU64::new(1);
        Self {
            scene,
            generation: NEXT.fetch_add(1, Ordering::Relaxed),
        }
    }

    /// This frame's state from the real-time solver (simulate mode).
    pub fn frame_from_sim(&mut self, sim: &DynamicSim) -> VisualFrame {
        self.scene.structure.capture(sim)
    }

    /// Straight members at the given node positions (build mode).
    pub fn frame_from_positions(&self, positions: &[Vec3], utilization: &[f32]) -> VisualFrame {
        self.scene
            .structure
            .frame_from_positions(positions, utilization)
    }
}

/// Solver output for this frame. Overwrite it after physics has run.
#[derive(Resource, Default)]
pub struct MemberFrame(pub VisualFrame);

#[derive(Resource, Debug, Clone, Default)]
pub struct RenderSettings {
    /// Colour members by utilisation instead of showing their material.
    pub stress_view: bool,
    /// Members to highlight (hover, selection).
    pub highlighted: Vec<usize>,
}

/// What the renderer did last frame, for HUDs and benchmarks.
#[derive(Resource, Debug, Default, Clone, Copy)]
pub struct RenderStats {
    pub slots: usize,
    pub remnants: usize,
    pub debris: usize,
    pub particles: usize,
    /// CPU time spent packing members and stepping debris and particles.
    pub scene_update_ms: f32,
}

#[derive(Resource)]
struct MemberAssets {
    material: Handle<MemberMaterial>,
    buffer: Handle<ShaderBuffer>,
    templates: HashMap<TemplateKey, Handle<Mesh>>,
    debris_materials: HashMap<(MaterialKind, bool), Handle<StandardMaterial>>,
}

#[derive(Clone, Copy, PartialEq, Eq, Hash)]
struct TemplateKey {
    profile: [u32; 5],
    detailed: bool,
}

fn template_key(profile: &Profile, detailed: bool) -> TemplateKey {
    let b = |x: f32| x.to_bits();
    let profile = match *profile {
        Profile::Rect { b: w, h } => [0, b(w), b(h), 0, 0],
        Profile::Round { r } => [1, b(r), 0, 0, 0],
        Profile::Tube { r, t } => [2, b(r), b(t), 0, 0],
        Profile::BoxSection { b: w, h, t } => [3, b(w), b(h), b(t), 0],
        Profile::IBeam { h, b: w, tw, tf } => [4, b(h), b(w), b(tw), b(tf)],
    };
    TemplateKey { profile, detailed }
}

/// A member instance; `MeshTag` holds the same slot index for the shader.
#[derive(Component)]
struct MemberInstance {
    key: TemplateKey,
}

#[derive(Component)]
struct DebrisVisual(u64);

/// Convert engine-free mesh data to a Bevy mesh.
pub fn to_bevy_mesh(data: &MeshData) -> Mesh {
    let mut mesh = Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::default(),
    );
    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, data.positions.clone());
    mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, data.normals.clone());
    mesh.insert_attribute(Mesh::ATTRIBUTE_UV_0, data.uvs.clone());
    mesh.insert_indices(Indices::U32(data.indices.clone()));
    mesh
}

/// A member template as a Bevy mesh. UV0 is (s, perimeter coordinate), with
/// the perimeter coordinate set to -1 on end caps; UV1 is the section point.
fn template_mesh(profile: &Profile, detailed: bool) -> Mesh {
    let opts = if detailed {
        TemplateOptions::detailed()
    } else {
        TemplateOptions::default()
    };
    let data = member_template(profile, opts);
    let uv0: Vec<[f32; 2]> = data
        .uvs
        .iter()
        .zip(&data.normals)
        .map(|(uv, n)| if n[0].abs() > 0.5 { [uv[0], -1.0] } else { *uv })
        .collect();
    let uv1: Vec<[f32; 2]> = data.positions.iter().map(|p| [p[1], p[2]]).collect();
    let mut mesh = to_bevy_mesh(&data);
    mesh.insert_attribute(Mesh::ATTRIBUTE_UV_0, uv0);
    mesh.insert_attribute(Mesh::ATTRIBUTE_UV_1, uv1);
    mesh
}

fn setup_assets(
    mut commands: Commands,
    mut buffers: ResMut<Assets<ShaderBuffer>>,
    mut materials: ResMut<Assets<MemberMaterial>>,
) {
    let buffer = buffers.add(ShaderBuffer::from(vec![hidden_member()]));
    let material = materials.add(ExtendedMaterial {
        base: StandardMaterial {
            perceptual_roughness: 0.6,
            ..default()
        },
        extension: MemberExtension {
            members: buffer.clone(),
        },
    });
    commands.insert_resource(MemberAssets {
        material,
        buffer,
        templates: HashMap::new(),
        debris_materials: HashMap::new(),
    });
}

fn hidden_member() -> GpuMember {
    GpuMember {
        info: Vec4::new(1.0, 0.0, 0.0, flags::HIDDEN as f32),
        ..Default::default()
    }
}

/// Advances the visual scene to this frame and uploads the member records.
fn advance_scene(
    time: Res<Time>,
    visual: Option<ResMut<StructureVisual>>,
    frame: Res<MemberFrame>,
    settings: Res<RenderSettings>,
    assets: Res<MemberAssets>,
    mut buffers: ResMut<Assets<ShaderBuffer>>,
    mut stats: ResMut<RenderStats>,
) {
    let Some(mut visual) = visual else {
        if let Some(mut buffer) = buffers.get_mut(&assets.buffer) {
            buffer.clear();
            buffer.extend_from_slice(&[hidden_member()]);
        }
        return;
    };
    let scene = &mut visual.scene;
    if frame.0.node_positions.len() < scene.structure.node_count {
        // No frame for this structure yet.
        return;
    }
    let start = std::time::Instant::now();
    let base = if settings.stress_view {
        flags::STRESS_VIEW
    } else {
        0
    };
    scene.update(&frame.0, time.delta_secs().min(0.1), base);
    for &i in &settings.highlighted {
        if let Some(g) = scene.gpu.get_mut(i) {
            g.info.w = (g.info.w as u32 | flags::HIGHLIGHT) as f32;
        }
    }
    if let Some(mut buffer) = buffers.get_mut(&assets.buffer) {
        buffer.clear();
        if scene.gpu.is_empty() {
            buffer.extend_from_slice(&[hidden_member()]);
        } else {
            buffer.extend_from_slice(&scene.gpu);
        }
    }
    *stats = RenderStats {
        slots: scene.slots.len(),
        remnants: scene.remnants.len(),
        debris: scene.debris.bodies.len(),
        particles: scene.particles.alive_count(),
        scene_update_ms: start.elapsed().as_secs_f32() * 1e3,
    };
}

/// Keeps one instance entity per slot, with the right template.
fn sync_instances(
    mut commands: Commands,
    visual: Option<Res<StructureVisual>>,
    mut assets: ResMut<MemberAssets>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut instances: Query<(Entity, &MeshTag, &mut MemberInstance, &mut Mesh3d)>,
    mut synced: Local<(u64, u64, usize)>,
) {
    let Some(visual) = visual else {
        if synced.0 != 0 {
            for (e, ..) in &instances {
                commands.entity(e).despawn();
            }
            *synced = (0, 0, 0);
        }
        return;
    };
    let scene = &visual.scene;
    let state = (visual.generation, scene.slots_revision, scene.slots.len());
    if *synced == state {
        return;
    }
    let material = assets.material.clone();
    let templates = &mut assets.templates;
    let mut template = |profile: &Profile, detailed: bool| {
        let key = template_key(profile, detailed);
        let handle = templates
            .entry(key)
            .or_insert_with(|| meshes.add(template_mesh(profile, detailed)))
            .clone();
        (key, handle)
    };
    let mut have = vec![false; scene.slots.len()];
    let new_structure = synced.0 != state.0;
    for (e, tag, mut inst, mut mesh) in &mut instances {
        let slot = tag.value as usize;
        if new_structure || slot >= scene.slots.len() {
            commands.entity(e).despawn();
            continue;
        }
        have[slot] = true;
        let s = &scene.slots[slot];
        let (key, handle) = template(&s.profile, s.detailed);
        if inst.key != key {
            inst.key = key;
            mesh.0 = handle;
        }
    }
    for (slot, s) in scene.slots.iter().enumerate() {
        if have[slot] {
            continue;
        }
        let (key, handle) = template(&s.profile, s.detailed);
        commands.spawn((
            Mesh3d(handle),
            MeshMaterial3d(material.clone()),
            MeshTag::new(slot as u32),
            MemberInstance { key },
            // The shader places members in world space itself, so the
            // entity's transform and bounds say nothing about where it is.
            Transform::IDENTITY,
            NoFrustumCulling,
        ));
    }
    *synced = state;
}

fn debris_color(kind: MaterialKind, part: PiecePart) -> (Color, f32, f32) {
    match (kind, part) {
        (_, PiecePart::Rebar) | (MaterialKind::Steel, _) => {
            (Color::srgb(0.3, 0.33, 0.37), 0.8, 0.5)
        }
        (MaterialKind::Timber, _) => (Color::srgb(0.72, 0.56, 0.36), 0.0, 0.9),
        (MaterialKind::Composite, _) => (Color::srgb(0.06, 0.06, 0.07), 0.1, 0.5),
        _ => (Color::srgb(0.58, 0.57, 0.54), 0.0, 0.95),
    }
}

/// Spawns, moves and removes loose debris pieces.
fn sync_debris(
    mut commands: Commands,
    visual: Option<ResMut<StructureVisual>>,
    mut assets: ResMut<MemberAssets>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut pieces: Query<(Entity, &DebrisVisual, &mut Transform, &Mesh3d)>,
    mut generation: Local<u64>,
) {
    let Some(mut visual) = visual else {
        for (e, _, _, mesh) in &pieces {
            meshes.remove(&mesh.0);
            commands.entity(e).despawn();
        }
        return;
    };
    let fresh = *generation != visual.generation;
    *generation = visual.generation;
    let scene = &mut visual.scene;
    for (id, piece) in scene.spawned.drain(..) {
        let key = (piece.kind, piece.part == PiecePart::Rebar);
        let material = assets
            .debris_materials
            .entry(key)
            .or_insert_with(|| {
                let (base_color, metallic, perceptual_roughness) =
                    debris_color(piece.kind, piece.part);
                materials.add(StandardMaterial {
                    base_color,
                    metallic,
                    perceptual_roughness,
                    ..default()
                })
            })
            .clone();
        commands.spawn((
            Mesh3d(meshes.add(to_bevy_mesh(&piece.mesh))),
            MeshMaterial3d(material),
            Transform::from_translation(piece.position).with_rotation(piece.rotation),
            DebrisVisual(id),
        ));
    }
    scene.debris.removed.clear();
    let live: HashMap<u64, (Vec3, Quat, bool)> = scene
        .debris
        .bodies
        .iter()
        .map(|b| (b.id, (b.position, b.rotation, b.asleep)))
        .collect();
    for (e, d, mut tf, mesh) in &mut pieces {
        match live.get(&d.0) {
            Some(&(p, r, asleep)) if !fresh => {
                if !asleep || tf.translation != p {
                    tf.translation = p;
                    tf.rotation = r;
                }
            }
            _ => {
                meshes.remove(&mesh.0);
                commands.entity(e).despawn();
            }
        }
    }
}
