//! The sheet's site drawn as things, not hints: foundations as concrete
//! blocks, the road (or footpath) a bridge carries, the equipment a
//! structure must hold, and short labels that say what each one is while
//! building.
//!
//! - Foundations: a concrete block per anchor, with an orange pin across it
//!   (pinned: the structure may turn on it) or an orange plate on it
//!   (fixed). Supports only count on these.
//! - Road: approach roads on the banks, and over the gap a road surface
//!   laid on the deck nodes the player builds. It follows those nodes in
//!   the test, so the trucks visibly drive on it, and a stretch drops out
//!   when the members under it break. While building, the lane shows where
//!   road nodes count and red stretches where they are too far apart.
//! - Equipment: each payload as a box of its real size standing on its
//!   feet: see-through while building, solid in the test, riding the feet
//!   and falling off when the run says it was lost.
//! - Labels: world-anchored captions over each of these while building.

use bevy::asset::RenderAssetUsages;
use bevy::mesh::{Indices, PrimitiveTopology};
use bevy::prelude::*;
use breakit_lab::{LabRigs, PlayTable};
use breakit_scenario as bs;
use breakit_scenario::{Binding, PlayMode};

use crate::analysis::{FrameState, Simulation};
use crate::design::Design;
use crate::flow::Screen;
use crate::scenario::Sheets;
use crate::ui::theme::Swatch;
use crate::ui::widgets::caps;

pub struct SitePlugin;

impl Plugin for SitePlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<RoadLayout>()
            .add_systems(Startup, setup_assets)
            .add_systems(
                PostUpdate,
                (
                    spawn_site,
                    update_road,
                    update_equipment,
                    ride_table,
                    draw_lane_guide,
                    place_labels,
                )
                    .chain()
                    .before(TransformSystems::Propagate),
            );
    }
}

/// Road width either side of the lane line is the deck's capture width,
/// capped so a wide capture strip doesn't make a motorway.
const MAX_HALF_WIDTH: f32 = 1.4;
/// Road slab thickness, m.
const SLAB: f32 = 0.12;
/// The road surface sits this far above the deck nodes, about the top of a
/// chord.
const SLAB_LIFT: f32 = 0.04;
/// Length of the approach road laid on each bank, m.
const APPROACH: f32 = 5.0;
/// A deck stretch longer than this times its built length has torn.
const TORN_STRETCH: f32 = 1.3;

const ASPHALT: [f32; 4] = [0.13, 0.135, 0.14, 1.0];
const PAINT: [f32; 4] = [0.86, 0.86, 0.82, 1.0];
const TIMBER: [f32; 4] = [0.48, 0.36, 0.24, 1.0];
const ACCENT: Color = Color::srgb(1.0, 0.54, 0.24);
const GAP: Color = Color::srgb(1.0, 0.48, 0.4);
const LANE_OK: Color = Color::srgba(0.95, 0.95, 0.9, 0.8);

#[derive(Resource)]
struct SiteAssets {
    concrete: Handle<StandardMaterial>,
    accent: Handle<StandardMaterial>,
    road: Handle<StandardMaterial>,
    ghost: Handle<StandardMaterial>,
    solid: Handle<StandardMaterial>,
    block: Handle<Mesh>,
    pin: Handle<Mesh>,
    plate: Handle<Mesh>,
    cube: Handle<Mesh>,
}

fn setup_assets(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands.insert_resource(SiteAssets {
        concrete: materials.add(StandardMaterial {
            base_color: Color::srgb(0.66, 0.65, 0.62),
            perceptual_roughness: 0.9,
            ..default()
        }),
        accent: materials.add(StandardMaterial {
            base_color: ACCENT,
            perceptual_roughness: 0.5,
            ..default()
        }),
        road: materials.add(StandardMaterial {
            base_color: Color::WHITE,
            perceptual_roughness: 0.85,
            ..default()
        }),
        ghost: materials.add(StandardMaterial {
            base_color: ACCENT.with_alpha(0.12),
            alpha_mode: AlphaMode::Blend,
            unlit: true,
            double_sided: true,
            cull_mode: None,
            ..default()
        }),
        solid: materials.add(StandardMaterial {
            base_color: Color::srgb(0.32, 0.42, 0.5),
            perceptual_roughness: 0.6,
            metallic: 0.2,
            ..default()
        }),
        block: meshes.add(Cuboid::new(0.44, 0.3, 0.44)),
        pin: meshes.add(Cylinder::new(0.06, 0.4)),
        plate: meshes.add(Cuboid::new(0.34, 0.04, 0.34)),
        cube: meshes.add(Cuboid::new(1.0, 1.0, 1.0)),
    });
}

/// Everything spawned for the current sheet.
#[derive(Component)]
struct SitePart;

/// A foundation part: on the shake table it rides the platform.
#[derive(Component)]
struct Rides {
    rest: Vec3,
}

/// The road mesh, rebuilt as the deck moves.
#[derive(Component)]
struct RoadMesh;

/// One payload's box, by payload index.
#[derive(Component)]
struct EquipmentBox {
    index: usize,
    /// Box size, m.
    size: Vec3,
    /// Box centre above the footprint centre, m.
    lift: f32,
    /// Feet as built.
    rest_feet: Vec<Vec3>,
    /// Falling after the run lost it: velocity, m/s.
    falling: Option<Vec3>,
}

/// A caption pinned to a world point while building.
#[derive(Component)]
struct SiteLabel {
    at: Vec3,
}

/// Which sheet and play mode the spawned site belongs to.
#[derive(Default, PartialEq)]
struct SpawnedFor(Option<(String, PlayMode)>);

fn shows_site(screen: &Screen) -> bool {
    matches!(screen, Screen::Build | Screen::Test | Screen::Report)
}

fn v3(p: [f64; 3]) -> Vec3 {
    Vec3::new(p[0] as f32, p[1] as f32, p[2] as f32)
}

/// What a sheet's lane carries, for the road's look and its label.
fn lane_kind(def: &bs::Scenario) -> (&'static str, [f32; 4]) {
    let trucks = def
        .events
        .iter()
        .any(|e| matches!(e, bs::Event::Convoy { .. }));
    if trucks {
        ("Road: the trucks drive here", ASPHALT)
    } else {
        ("Footpath: people walk here", TIMBER)
    }
}

/// Anchors of one kind closer than this to a neighbour share a label.
const LABEL_CLUSTER: f64 = 1.6;

/// Anchors grouped for labels: same kind, chained by distance.
fn anchor_groups(anchors: &[bs::Anchor]) -> Vec<Vec<usize>> {
    let near = |a: &bs::Anchor, b: &bs::Anchor| {
        a.kind == b.kind
            && (0..3)
                .map(|k| (a.position[k] - b.position[k]).powi(2))
                .sum::<f64>()
                .sqrt()
                <= LABEL_CLUSTER
    };
    let mut group_of: Vec<Option<usize>> = vec![None; anchors.len()];
    let mut groups: Vec<Vec<usize>> = Vec::new();
    for i in 0..anchors.len() {
        if group_of[i].is_some() {
            continue;
        }
        let g = groups.len();
        let mut members = vec![i];
        group_of[i] = Some(g);
        let mut k = 0;
        while k < members.len() {
            let m = members[k];
            for j in 0..anchors.len() {
                if group_of[j].is_none() && near(&anchors[m], &anchors[j]) {
                    group_of[j] = Some(g);
                    members.push(j);
                }
            }
            k += 1;
        }
        groups.push(members);
    }
    groups
}

#[allow(clippy::too_many_arguments)]
fn spawn_site(
    mut commands: Commands,
    sheets: Res<Sheets>,
    screen: Res<State<Screen>>,
    assets: Res<SiteAssets>,
    mut meshes: ResMut<Assets<Mesh>>,
    parts: Query<Entity, With<SitePart>>,
    mut labels: Query<&mut Visibility, With<SiteLabel>>,
    mut spawned: Local<SpawnedFor>,
) {
    let sheet = sheets.current();
    let show = shows_site(screen.get()) && sheet.mode != PlayMode::Sandbox;
    let key = show.then(|| (sheet.def.id.clone(), sheet.mode));
    let building = *screen.get() == Screen::Build;
    for mut v in &mut labels {
        let want = if building {
            Visibility::Inherited
        } else {
            Visibility::Hidden
        };
        if *v != want {
            *v = want;
        }
    }
    if spawned.0 == key {
        return;
    }
    spawned.0 = key;
    for e in &parts {
        commands.entity(e).despawn();
    }
    if !show {
        return;
    }
    let def = &sheet.def;

    // Foundations.
    for a in &def.anchors {
        let p = v3(a.position);
        commands.spawn((
            Mesh3d(assets.block.clone()),
            MeshMaterial3d(assets.concrete.clone()),
            Transform::from_translation(p - Vec3::Y * 0.13),
            Rides {
                rest: p - Vec3::Y * 0.13,
            },
            SitePart,
        ));
        let mark = match a.kind {
            bs::AnchorKind::Pinned => (
                Mesh3d(assets.pin.clone()),
                Transform::from_translation(p + Vec3::Y * 0.02)
                    .with_rotation(Quat::from_rotation_x(std::f32::consts::FRAC_PI_2)),
            ),
            bs::AnchorKind::Fixed => (
                Mesh3d(assets.plate.clone()),
                Transform::from_translation(p + Vec3::Y * 0.03),
            ),
        };
        commands.spawn((
            mark.0,
            MeshMaterial3d(assets.accent.clone()),
            mark.1,
            SitePart,
        ));
    }
    for group in anchor_groups(&def.anchors) {
        let a = &def.anchors[group[0]];
        let centre = group
            .iter()
            .map(|&j| v3(def.anchors[j].position))
            .sum::<Vec3>()
            / group.len() as f32;
        let what = match (a.kind, group.len()) {
            (bs::AnchorKind::Pinned, 1) => "Foundation (pin): support here".to_string(),
            (bs::AnchorKind::Pinned, n) => format!("{n} foundations (pins): support here"),
            (bs::AnchorKind::Fixed, _) => "Foundation (fixed): support here".to_string(),
        };
        // Below the foundations and out from the middle of the site, clear
        // of the structure's own labels.
        let site_mid = v3(def.limits.volume.min).lerp(v3(def.limits.volume.max), 0.5);
        let out = Vec3::new(centre.x - site_mid.x, 0.0, centre.z - site_mid.z).normalize_or_zero();
        spawn_label(&mut commands, centre + out * 0.6 - Vec3::Y * 0.45, &what);
    }

    // The build area's box is drawn by `guides.rs`; say what it is.
    let (lo, hi) = (v3(def.limits.volume.min), v3(def.limits.volume.max));
    spawn_label(
        &mut commands,
        Vec3::new((lo.x + hi.x) * 0.5, hi.y, hi.z),
        "Build area: every node goes inside the box",
    );

    // The road: one mesh, rebuilt by `update_road`.
    if let Some(d) = &def.deck {
        commands.spawn((
            Mesh3d(meshes.add(empty_mesh())),
            MeshMaterial3d(assets.road.clone()),
            Transform::default(),
            RoadMesh,
            SitePart,
        ));
        let (what, _) = lane_kind(def);
        // On the far approach, where the road is clear of the structure.
        let (a, b) = (v3(d.from), v3(d.to));
        let at = b + (b - a).normalize_or(Vec3::X) * 3.0;
        spawn_label(&mut commands, at + Vec3::Y * 0.3, what);
    }

    // Equipment.
    for (i, p) in def.payloads.iter().enumerate() {
        let size = p
            .size
            .map(v3)
            .unwrap_or_else(|| Vec3::splat((p.mass as f32 / 1000.0).cbrt().clamp(0.8, 1.8)));
        let feet: Vec<Vec3> = p.foot_positions().into_iter().map(v3).collect();
        let lift = (p.height as f32 - size.y * 0.5).max(0.0) + size.y * 0.5;
        let base = feet.iter().sum::<Vec3>() / feet.len() as f32;
        commands.spawn((
            Mesh3d(assets.cube.clone()),
            MeshMaterial3d(assets.ghost.clone()),
            Transform::from_translation(base + Vec3::Y * lift).with_scale(size),
            EquipmentBox {
                index: i,
                size,
                lift,
                rest_feet: feet,
                falling: None,
            },
            SitePart,
        ));
        let label = format!("{}, {}: carry its feet", p.name, tonnes(p.mass));
        spawn_label(
            &mut commands,
            base + Vec3::Y * (lift + size.y * 0.5 + 0.3),
            &label,
        );
    }
}

fn tonnes(kg: f64) -> String {
    if kg >= 1000.0 {
        format!("{:.1} t", kg / 1000.0).replace(".0 t", " t")
    } else {
        format!("{kg:.0} kg")
    }
}

fn spawn_label(commands: &mut Commands, at: Vec3, s: &str) {
    commands
        .spawn((
            Node {
                position_type: PositionType::Absolute,
                padding: UiRect::axes(Val::Px(8.0), Val::Px(4.0)),
                ..default()
            },
            BackgroundColor(Color::srgba(0.04, 0.05, 0.06, 0.62)),
            GlobalZIndex(-1),
            Visibility::Hidden,
            SiteLabel { at },
            SitePart,
        ))
        .with_children(|l| {
            l.spawn(caps(s, Swatch::Ink));
        });
}

fn empty_mesh() -> Mesh {
    Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::default(),
    )
    .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, Vec::<[f32; 3]>::new())
    .with_inserted_attribute(Mesh::ATTRIBUTE_NORMAL, Vec::<[f32; 3]>::new())
    .with_inserted_attribute(Mesh::ATTRIBUTE_COLOR, Vec::<[f32; 4]>::new())
    .with_inserted_indices(Indices::U32(Vec::new()))
}

/// Where the current design's deck stations are, cached per design
/// revision.
#[derive(Resource, Default)]
struct RoadLayout {
    key: Option<(String, u64, usize, usize)>,
    /// Lane start and unit direction (horizontal), and its length.
    start: Vec3,
    dir: Vec3,
    length: f32,
    half_width: f32,
    max_gap: f32,
    /// Per station: distance along the lane, its nodes, their mean rest
    /// position, and the members joining it to the next station.
    stations: Vec<(f32, Vec<usize>, Vec3, Vec<usize>)>,
}

impl RoadLayout {
    fn refresh(&mut self, def: &bs::Scenario, design: &Design) {
        let key = (
            def.id.clone(),
            design.revision,
            design.nodes.len(),
            design.beams.len(),
        );
        if self.key.as_ref() == Some(&key) {
            return;
        }
        self.key = Some(key);
        self.stations.clear();
        let Some(d) = &def.deck else { return };
        let (a, b) = (v3(d.from), v3(d.to));
        self.start = a;
        self.length = (b - a).length();
        self.dir = (b - a).normalize_or(Vec3::X);
        self.half_width = (d.width as f32).min(MAX_HALF_WIDTH);
        self.max_gap = d.max_gap as f32;
        if design.beams.is_empty() {
            return;
        }
        let binding = Binding::new(def, &design.to_structure());
        let Some(lane) = binding.deck else { return };
        for (k, st) in lane.stations.iter().enumerate() {
            let nodes: Vec<usize> = std::iter::once(st.node)
                .chain(st.others.iter().copied())
                .filter(|&n| n < design.nodes.len())
                .collect();
            if nodes.is_empty() {
                continue;
            }
            let rest = nodes.iter().map(|&n| design.nodes[n]).sum::<Vec3>() / nodes.len() as f32;
            let joins = lane.segments.get(k).cloned().unwrap_or_default();
            self.stations.push((st.s as f32, nodes, rest, joins));
        }
    }

    fn point(&self, s: f32) -> Vec3 {
        self.start + self.dir * s
    }

    /// Stretches of the lane (from, to, distance along) longer than the
    /// sheet allows, counting from the lane ends.
    fn gaps(&self) -> Vec<(f32, f32)> {
        let mut edges = vec![0.0];
        edges.extend(self.stations.iter().map(|s| s.0));
        edges.push(self.length);
        edges
            .windows(2)
            .filter(|w| w[1] - w[0] > self.max_gap + 1e-3)
            .map(|w| (w[0], w[1]))
            .collect()
    }
}

/// Lays the road on the deck nodes where they are now.
#[allow(clippy::too_many_arguments)]
fn update_road(
    sheets: Res<Sheets>,
    design: Res<Design>,
    state: Res<FrameState>,
    table: Res<PlayTable>,
    mut layout: ResMut<RoadLayout>,
    mut meshes: ResMut<Assets<Mesh>>,
    road: Query<&Mesh3d, With<RoadMesh>>,
) {
    let Ok(handle) = road.single() else {
        return;
    };
    let def = &sheets.current().def;
    layout.refresh(def, &design);
    let Some(mut mesh) = meshes.get_mut(&handle.0) else {
        return;
    };
    let mut out = Slabs::default();
    let (_, top) = lane_kind(def);
    let side = layout.dir.cross(Vec3::Y).normalize_or(Vec3::Z);
    let hw = layout.half_width;

    // Approach roads on the banks, on the ground at each lane end.
    let ends = [
        (layout.point(0.0), -layout.dir),
        (layout.point(layout.length), layout.dir),
    ];
    let half = table.size * 0.5;
    for (p, out_dir) in ends {
        if !on_ground(&table, p) {
            continue;
        }
        // To the table edge, at most APPROACH.
        let mut len = APPROACH;
        for (axis, lim) in [
            (out_dir.x, half.x - p.x.abs()),
            (out_dir.z, half.y - p.z.abs()),
        ] {
            if axis.abs() > 1e-3 {
                len = len.min(lim.max(0.0) / axis.abs());
            }
        }
        if len > 0.05 {
            let y = p.y + 0.02;
            let far = p + out_dir * len;
            out.slab(
                Vec3::new(p.x, y, p.z),
                Vec3::new(far.x, y, far.z),
                side,
                hw,
                0.04,
                top,
            );
        }
    }

    // The deck itself, from station to station.
    let live = |n: usize| state.positions.get(n).copied().unwrap_or(design.nodes[n]);
    let pts: Vec<Vec3> = layout
        .stations
        .iter()
        .map(|(s, nodes, rest, _)| {
            let now = nodes.iter().map(|&n| live(n)).sum::<Vec3>() / nodes.len() as f32;
            let at = layout.point(*s);
            // The lane point carried by the stations' movement, at deck
            // height.
            Vec3::new(at.x, rest.y, at.z) + (now - *rest) + Vec3::Y * SLAB_LIFT
        })
        .collect();
    let broken = |m: &usize| state.members.get(*m).is_some_and(|s| s.broken);
    for (k, w) in pts.windows(2).enumerate() {
        let (s0, s1) = (layout.stations[k].0, layout.stations[k + 1].0);
        let built = (s1 - s0).max(1e-3);
        if built > layout.max_gap + 1e-3 {
            continue;
        }
        let joins = &layout.stations[k].3;
        let torn = !joins.is_empty() && joins.iter().all(broken);
        if torn || (w[1] - w[0]).length() > built * TORN_STRETCH {
            continue;
        }
        out.slab(w[0], w[1], side, hw, SLAB, top);
    }
    // From the banks to the first and last station, while they reach.
    if let (Some(first), Some(last)) = (pts.first(), pts.last()) {
        let s_first = layout.stations[0].0;
        let s_last = layout.stations[layout.stations.len() - 1].0;
        let a = layout.point(0.0) + Vec3::Y * SLAB_LIFT;
        let b = layout.point(layout.length) + Vec3::Y * SLAB_LIFT;
        if s_first > 0.02 && s_first <= layout.max_gap {
            out.slab(a, *first, side, hw, SLAB, top);
        }
        let tail = layout.length - s_last;
        if tail > 0.02 && tail <= layout.max_gap {
            out.slab(*last, b, side, hw, SLAB, top);
        }
    }
    out.write(&mut mesh);
}

/// True when `p` is on (or just above) solid table ground.
fn on_ground(table: &PlayTable, p: Vec3) -> bool {
    table.contact_boxes().iter().any(|(lo, hi)| {
        p.x >= lo.x - 0.6
            && p.x <= hi.x + 0.6
            && p.z >= lo.z - 0.6
            && p.z <= hi.z + 0.6
            && (p.y - hi.y).abs() < 0.1
    })
}

/// Triangle soup with flat normals and vertex colours.
#[derive(Default)]
struct Slabs {
    pos: Vec<[f32; 3]>,
    nrm: Vec<[f32; 3]>,
    col: Vec<[f32; 4]>,
    idx: Vec<u32>,
}

impl Slabs {
    fn quad(&mut self, q: [Vec3; 4], c: [f32; 4]) {
        let n = (q[1] - q[0]).cross(q[3] - q[0]).normalize_or(Vec3::Y);
        let base = self.pos.len() as u32;
        for p in q {
            self.pos.push(p.to_array());
            self.nrm.push(n.to_array());
            self.col.push(c);
        }
        self.idx
            .extend_from_slice(&[base, base + 1, base + 2, base, base + 2, base + 3]);
    }

    /// A road slab from `a` to `b` (top surface centre line), `hw` either
    /// side along `side`, `depth` thick, with painted edge lines.
    fn slab(&mut self, a: Vec3, b: Vec3, side: Vec3, hw: f32, depth: f32, top: [f32; 4]) {
        let (sa, sb) = (side * hw, side * hw);
        let down = Vec3::Y * depth;
        let ta = [a - sa, a + sa];
        let tb = [b - sb, b + sb];
        // Top, wound to face up.
        let up_quad = [ta[0], tb[0], tb[1], ta[1]];
        let n = (up_quad[1] - up_quad[0]).cross(up_quad[3] - up_quad[0]);
        let up_quad = if n.y >= 0.0 {
            up_quad
        } else {
            [ta[0], ta[1], tb[1], tb[0]]
        };
        self.quad(up_quad, top);
        // Bottom and sides.
        let bot = [ta[0] - down, ta[1] - down, tb[1] - down, tb[0] - down];
        let flip = |q: [Vec3; 4]| [q[0], q[3], q[2], q[1]];
        self.quad(if n.y >= 0.0 { flip(bot) } else { bot }, top);
        let darker = [top[0] * 0.7, top[1] * 0.7, top[2] * 0.7, 1.0];
        for i in 0..2 {
            let q = [ta[i], tb[i], tb[i] - down, ta[i] - down];
            let outward = if i == 0 { -side } else { side };
            let qn = (q[1] - q[0]).cross(q[3] - q[0]);
            self.quad(if qn.dot(outward) >= 0.0 { q } else { flip(q) }, darker);
        }
        // Edge lines a hair above the surface.
        let lift = Vec3::Y * 0.004;
        for s in [-1.0, 1.0] {
            let o = side * (hw - 0.12) * s;
            let w = side * 0.04;
            let q = [
                a + o - w + lift,
                b + o - w + lift,
                b + o + w + lift,
                a + o + w + lift,
            ];
            let qn = (q[1] - q[0]).cross(q[3] - q[0]);
            self.quad(if qn.y >= 0.0 { q } else { flip(q) }, PAINT);
        }
    }

    fn write(self, mesh: &mut Mesh) {
        mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, self.pos);
        mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, self.nrm);
        mesh.insert_attribute(Mesh::ATTRIBUTE_COLOR, self.col);
        mesh.insert_indices(Indices::U32(self.idx));
    }
}

/// Equipment boxes: ghosts on their feet while building; in the test they
/// ride the feet, and fall once the run has lost them.
#[allow(clippy::too_many_arguments)]
fn update_equipment(
    time: Res<Time>,
    screen: Res<State<Screen>>,
    sim: Option<Res<Simulation>>,
    state: Res<FrameState>,
    design: Res<Design>,
    sheets: Res<Sheets>,
    table: Res<PlayTable>,
    rigs: Res<LabRigs>,
    assets: Res<SiteAssets>,
    mut boxes: Query<(
        &mut EquipmentBox,
        &mut Transform,
        &mut MeshMaterial3d<StandardMaterial>,
    )>,
) {
    let building = *screen.get() == Screen::Build;
    let run = sim.as_deref().and_then(|s| s.run());
    let feet_nodes = sheets.current().load_points.len();
    for (mut b, mut tf, mut mat) in &mut boxes {
        let want = if building {
            &assets.ghost
        } else {
            &assets.solid
        };
        if mat.0 != *want {
            mat.0 = want.clone();
        }
        if building || run.is_none() {
            b.falling = None;
            let base = b.rest_feet.iter().sum::<Vec3>() / b.rest_feet.len() as f32;
            *tf = Transform::from_translation(base + Vec3::Y * b.lift).with_scale(b.size);
            continue;
        }
        let lost = run.is_some_and(|r| r.equipment().get(b.index).is_some_and(|e| e.lost));
        if lost {
            let dt = time.delta_secs().min(0.05);
            let floor = ground_below(&table, tf.translation) + b.size.y * 0.5;
            let v = b.falling.get_or_insert(Vec3::ZERO);
            *v += Vec3::Y * -9.81 * dt;
            tf.translation += *v * dt;
            if tf.translation.y < floor {
                tf.translation.y = floor;
                *v = Vec3::ZERO;
            }
            continue;
        }
        // Feet are the payload's locked nodes, in order, at the start of
        // the design.
        let first: usize = sheets.current().def.payloads[..b.index]
            .iter()
            .map(|p| p.foot_positions().len())
            .sum();
        let n = b.rest_feet.len();
        if first + n > feet_nodes {
            continue;
        }
        let now: Vec<Vec3> = (first..first + n)
            .map(|i| state.positions.get(i).copied().unwrap_or(design.nodes[i]))
            .collect();
        let now_c = now.iter().sum::<Vec3>() / n as f32;
        let rot = if n >= 3 {
            let normal = |p: &[Vec3]| (p[1] - p[0]).cross(p[2] - p[0]).normalize_or(Vec3::Y);
            let (r, w) = (normal(&b.rest_feet), normal(&now));
            let (r, w) = if r.y < 0.0 { (-r, -w) } else { (r, w) };
            Quat::from_rotation_arc(r, w)
        } else {
            Quat::IDENTITY
        };
        let offset = table_offset(&rigs, sim.as_deref());
        *tf = Transform::from_translation(now_c + offset + rot * (Vec3::Y * b.lift))
            .with_rotation(rot)
            .with_scale(b.size);
    }
}

/// The shake platform's offset now: the solver works in the table's frame,
/// the screen shows the structure riding it (as `visuals.rs` does).
fn table_offset(rigs: &LabRigs, sim: Option<&Simulation>) -> Vec3 {
    match (rigs.live.table, sim) {
        (Some(t), Some(_)) => t.displacement,
        _ => Vec3::ZERO,
    }
}

fn ride_table(
    rigs: Res<LabRigs>,
    sim: Option<Res<Simulation>>,
    mut parts: Query<(&Rides, &mut Transform)>,
) {
    let offset = table_offset(&rigs, sim.as_deref());
    for (r, mut tf) in &mut parts {
        let want = r.rest + offset;
        if tf.translation != want {
            tf.translation = want;
        }
    }
}

/// Height of the table's solid ground under `p` (or far below).
fn ground_below(table: &PlayTable, p: Vec3) -> f32 {
    table
        .contact_boxes()
        .iter()
        .filter(|(lo, hi)| p.x >= lo.x && p.x <= hi.x && p.z >= lo.z && p.z <= hi.z)
        .map(|(_, hi)| hi.y)
        .filter(|&y| y <= p.y + 0.5)
        .fold(-10.0, f32::max)
}

/// While building: the lane's strip (where a node counts as a road node),
/// red where road nodes are too far apart, and a mark at each payload foot.
fn draw_lane_guide(
    mut gizmos: Gizmos,
    screen: Res<State<Screen>>,
    sheets: Res<Sheets>,
    layout: Res<RoadLayout>,
) {
    let sheet = sheets.current();
    if *screen.get() != Screen::Build || sheet.mode == PlayMode::Sandbox {
        return;
    }
    let def = &sheet.def;
    if def.deck.is_some() {
        let side = layout.dir.cross(Vec3::Y).normalize_or(Vec3::Z) * layout.half_width;
        let (a, b) = (layout.point(0.0), layout.point(layout.length));
        // The strip's outline and a dashed centre line.
        for s in [-1.0, 1.0] {
            gizmos.line(a + side * s, b + side * s, LANE_OK.with_alpha(0.35));
        }
        gizmos.line(a - side, a + side, LANE_OK);
        gizmos.line(b - side, b + side, LANE_OK);
        let dash = 0.25;
        let n = (layout.length / (2.0 * dash)).ceil() as i32;
        for k in 0..n {
            let s0 = k as f32 * 2.0 * dash;
            let s1 = (s0 + dash).min(layout.length);
            gizmos.line(layout.point(s0), layout.point(s1), LANE_OK);
        }
        for (s0, s1) in layout.gaps() {
            let lift = Vec3::Y * 0.06;
            let (p0, p1) = (layout.point(s0) + lift, layout.point(s1) + lift);
            gizmos.line(p0, p1, GAP);
            gizmos.line(p0 - side * 0.3, p1 - side * 0.3, GAP);
            gizmos.line(p0 + side * 0.3, p1 + side * 0.3, GAP);
        }
    }
    for p in &def.payloads {
        for f in p.foot_positions() {
            let at = v3(f);
            gizmos.circle(
                Isometry3d::new(at, Quat::from_rotation_x(std::f32::consts::FRAC_PI_2)),
                0.18,
                ACCENT,
            );
        }
    }
}

/// Moves each label to its world point on screen; hides it behind the
/// camera.
fn place_labels(
    screen: Res<State<Screen>>,
    camera: Single<(&Camera, &GlobalTransform), With<Camera3d>>,
    mut labels: Query<(&SiteLabel, &mut Node, &ComputedNode, &mut Visibility)>,
) {
    if *screen.get() != Screen::Build {
        return;
    }
    let (cam, at) = *camera;
    for (l, mut node, computed, mut vis) in &mut labels {
        let Ok(p) = cam.world_to_viewport(at, l.at) else {
            *vis = Visibility::Hidden;
            continue;
        };
        let size = computed.size() * computed.inverse_scale_factor();
        node.left = Val::Px(p.x - size.x * 0.5);
        node.top = Val::Px(p.y - size.y * 0.5);
        if *vis == Visibility::Hidden {
            *vis = Visibility::Inherited;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::analysis::compute_static;
    use crate::design::BeamKind;
    use crate::scenario::catalogue;

    /// Builds `beams` (end points) on sheet `id` and runs build mode's
    /// static check.
    fn check(id: &str, beams: &[(Vec3, Vec3)]) -> FrameState {
        let s = catalogue().into_iter().find(|s| s.id == id).unwrap();
        let mut d = s.initial_design();
        for &(a, b) in beams {
            let (a, b) = (d.add_node(a), d.add_node(b));
            d.add_beam(a, b, BeamKind::part("steel-tube-m"));
        }
        compute_static(&d, 1.0, &s.static_loads(&d), &s.static_springs(&d))
    }

    #[test]
    fn a_frame_only_the_water_holds_is_not_shown_flying_off() {
        // Hangs into the flume from the pump house's feet, on no foundation.
        let st = check(
            "F-01",
            &[
                (Vec3::new(-1.0, 1.0, -0.5), Vec3::new(1.0, 1.0, -0.5)),
                (Vec3::new(-1.0, 1.0, -0.5), Vec3::new(-1.0, -2.0, -0.5)),
            ],
        );
        assert!(st.error.is_some());
        assert!(st
            .positions
            .iter()
            .all(|p| p.is_finite() && p.length() < 20.0));
    }

    #[test]
    fn a_jetty_on_its_footings_still_solves() {
        let st = check(
            "F-01",
            &[
                (Vec3::new(-1.5, -3.2, -1.5), Vec3::new(-1.0, 1.0, -0.5)),
                (Vec3::new(1.5, -3.2, -1.5), Vec3::new(1.0, 1.0, -0.5)),
                (Vec3::new(-1.0, 1.0, -0.5), Vec3::new(1.0, 1.0, -0.5)),
            ],
        );
        assert!(st.error.is_none(), "{:?}", st.error);
    }

    #[test]
    fn anchor_labels_group_each_bank() {
        let s = catalogue().into_iter().find(|s| s.id == "B-01").unwrap();
        let groups = anchor_groups(&s.def.anchors);
        assert_eq!(groups.len(), 2);
        assert!(groups.iter().all(|g| g.len() == 3));
    }
}
