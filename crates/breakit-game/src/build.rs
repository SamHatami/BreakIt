//! Build mode: place nodes on a snapping grid, join them with beams, set
//! supports and loads, select members.
//!
//! Input arrives as [`UiAction`]s (keys and buttons, see `flow.rs` for the
//! map) plus left clicks on the scene, which act by the current [`Tool`]:
//! - Draw (B): click empty grid to add a node, a node to pick it; with a
//!   node picked, each click adds a beam from it, so clicks chain beams.
//!   Clicking a beam selects it for the inspector.
//! - Support (S): click a node to cycle none, pinned, fixed.
//! - Delete (X): click a node or a beam to delete it.
//! - Load (L, sandbox only): click a node to cycle its load.
//!
//! Fixed scenario nodes (anchors, load points) can't be deleted or have
//! their support changed.

use bevy::prelude::*;
use bevy::window::PrimaryWindow;

use crate::analysis::DeflectionScale;
use crate::camera::OrbitCamera;
use crate::design::{catalog, BeamKind, Design};
use crate::flow::{FailurePins, Overlay, Screen, UiAction};
use crate::history::History;
use crate::scenario::{Scenario, Sheets};
use crate::ui::pointer::Pointer;

pub struct BuildPlugin;

impl Plugin for BuildPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<BuildTool>()
            .init_resource::<History>()
            .add_systems(
                Update,
                (
                    update_cursor,
                    build_actions,
                    build_clicks,
                    observe_history,
                    draw_build_guides,
                )
                    .chain()
                    .run_if(in_state(Screen::Build)),
            )
            .add_systems(Update, frame_view);
    }
}

/// Grid spacing for snapping, metres.
pub const GRID: f32 = 0.5;
/// How close, in screen pixels, the cursor must be to pick a node.
const PICK_RADIUS_PX: f32 = 14.0;
/// How close, in screen pixels, the cursor must be to pick a beam.
const BEAM_PICK_PX: f32 = 8.0;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum BuildPlane {
    /// Horizontal plane at `y = level`.
    Ground,
    /// Vertical plane at `z = level`, the plane a bridge elevation is drawn in.
    #[default]
    Wall,
}

impl BuildPlane {
    pub fn normal(self) -> Dir3 {
        match self {
            BuildPlane::Ground => Dir3::Y,
            BuildPlane::Wall => Dir3::Z,
        }
    }

    pub fn origin(self, level: f32) -> Vec3 {
        match self {
            BuildPlane::Ground => Vec3::new(0.0, level, 0.0),
            BuildPlane::Wall => Vec3::new(0.0, 0.0, level),
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            BuildPlane::Ground => "XZ",
            BuildPlane::Wall => "XY",
        }
    }

    pub fn axis(self) -> &'static str {
        match self {
            BuildPlane::Ground => "y",
            BuildPlane::Wall => "z",
        }
    }
}

/// What a left click on the scene does.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum Tool {
    #[default]
    Draw,
    Support,
    Delete,
    Load,
}

impl Tool {
    pub fn label(self) -> &'static str {
        match self {
            Tool::Draw => "Draw",
            Tool::Support => "Support",
            Tool::Delete => "Delete",
            Tool::Load => "Load",
        }
    }

    pub fn key(self) -> &'static str {
        match self {
            Tool::Draw => "B",
            Tool::Support => "S",
            Tool::Delete => "X",
            Tool::Load => "L",
        }
    }

    pub fn hint(self) -> &'static str {
        match self {
            Tool::Draw => "Click to place a node, clicks chain beams. Click a beam to inspect it. Esc ends the chain.",
            Tool::Support => "Click a node to cycle its support: pinned, fixed, none.",
            Tool::Delete => "Click a node or a beam to delete it. Fixed sheet nodes stay.",
            Tool::Load => "Click a node to cycle its load: 10, 50, 100, 200 kN, none.",
        }
    }
}

#[derive(Resource, Debug, Default)]
pub struct BuildTool {
    pub tool: Tool,
    pub kind: BeamKind,
    pub plane: BuildPlane,
    /// Plane offset along its normal, metres.
    pub level: f32,
    /// Start of the beam chain being drawn.
    pub selected: Option<usize>,
    /// Beam shown in the inspector.
    pub selected_beam: Option<usize>,
    /// Node under the cursor, if any.
    pub hovered: Option<usize>,
    /// Beam under the cursor, if no node is.
    pub hovered_beam: Option<usize>,
    /// Snapped point on the build plane under the cursor.
    pub cursor: Option<Vec3>,
    /// The sheet allows the load tool.
    pub loads_allowed: bool,
    /// Catalog parts the sheet allows. A member may be any size or grade of
    /// an allowed part's material.
    pub allowed: Vec<BeamKind>,
}

impl BuildTool {
    /// Fresh tool state for a sheet.
    pub fn reset_for(&mut self, s: &Scenario) {
        *self = BuildTool {
            loads_allowed: s.sandbox,
            allowed: s.allowed.clone(),
            kind: s.allowed.first().copied().unwrap_or_default(),
            ..Default::default()
        };
    }

    /// Whether the sheet allows this member's material.
    pub fn allows(&self, k: BeamKind) -> bool {
        self.allowed
            .iter()
            .any(|a| a.material_kind() == k.material_kind())
    }

    /// Material classes of the allowed parts, in palette order (keys 1-4).
    pub fn classes(&self) -> Vec<breakit_core::MaterialKind> {
        let mut out = Vec::new();
        for k in &self.allowed {
            if !out.contains(&k.material_kind()) {
                out.push(k.material_kind());
            }
        }
        out
    }

    /// The member the palette and inspector act on: the selected beam's,
    /// else the one the next beam gets.
    pub fn current(&self, design: &Design) -> BeamKind {
        self.selected_beam
            .and_then(|b| design.beams.get(b))
            .map(|b| b.kind)
            .unwrap_or(self.kind)
    }

    /// Ends the chain, else drops the selected beam. Returns whether there
    /// was anything to clear.
    pub fn deselect(&mut self) -> bool {
        if self.selected.take().is_some() {
            return true;
        }
        self.selected_beam.take().is_some()
    }
}

pub fn snap(p: Vec3) -> Vec3 {
    (p / GRID).round() * GRID
}

/// Distance from `p` to segment `a`-`b`, 2D.
pub fn segment_distance(p: Vec2, a: Vec2, b: Vec2) -> f32 {
    let ab = b - a;
    let t = if ab.length_squared() > 0.0 {
        ((p - a).dot(ab) / ab.length_squared()).clamp(0.0, 1.0)
    } else {
        0.0
    };
    p.distance(a + ab * t)
}

fn update_cursor(
    window: Single<&Window, With<PrimaryWindow>>,
    camera: Single<(&Camera, &GlobalTransform), With<OrbitCamera>>,
    design: Res<Design>,
    pointer: Res<Pointer>,
    mut tool: ResMut<BuildTool>,
) {
    let (camera, cam_tf) = *camera;
    tool.hovered = None;
    tool.hovered_beam = None;
    tool.cursor = None;
    let Some(px) = window.cursor_position() else {
        return;
    };
    if pointer.over_ui {
        return;
    }

    let screen: Vec<Option<Vec2>> = design
        .nodes
        .iter()
        .map(|&p| camera.world_to_viewport(cam_tf, p).ok())
        .collect();

    // Nearest node in screen space wins.
    tool.hovered = screen
        .iter()
        .enumerate()
        .filter_map(|(i, s)| {
            let d = s.as_ref()?.distance(px);
            (d <= PICK_RADIUS_PX).then_some((i, d))
        })
        .min_by(|a, b| a.1.total_cmp(&b.1))
        .map(|(i, _)| i);

    if tool.hovered.is_none() {
        tool.hovered_beam = design
            .beams
            .iter()
            .enumerate()
            .filter_map(|(i, b)| {
                let d = segment_distance(px, screen[b.a]?, screen[b.b]?);
                (d <= BEAM_PICK_PX).then_some((i, d))
            })
            .min_by(|a, b| a.1.total_cmp(&b.1))
            .map(|(i, _)| i);
    }

    let Ok(ray) = camera.viewport_to_world(cam_tf, px) else {
        return;
    };
    let origin = tool.plane.origin(tool.level);
    let plane = InfinitePlane3d::new(tool.plane.normal());
    if let Some(t) = ray.intersect_plane(origin, plane) {
        let mut p = snap(ray.get_point(t));
        // Keep the point exactly on the plane after snapping.
        match tool.plane {
            BuildPlane::Ground => p.y = tool.level,
            BuildPlane::Wall => p.z = tool.level,
        }
        tool.cursor = Some(p);
    }
}

/// Sets the member for the next beam and for the selected one, if the
/// sheet allows its material.
fn set_kind(design: &mut Design, tool: &mut BuildTool, k: BeamKind) {
    if !tool.allows(k) {
        return;
    }
    tool.kind = k;
    if let Some(b) = tool.selected_beam {
        design.set_beam_kind(b, k);
    }
}

#[allow(clippy::too_many_arguments)]
fn build_actions(
    mut actions: MessageReader<UiAction>,
    mut design: ResMut<Design>,
    mut tool: ResMut<BuildTool>,
    mut scale: ResMut<DeflectionScale>,
    mut history: ResMut<History>,
    sheets: Res<Sheets>,
) {
    for a in actions.read() {
        match *a {
            UiAction::SetTool(t) => {
                if t != Tool::Load || tool.loads_allowed {
                    tool.tool = t;
                    tool.selected = None;
                }
            }
            UiAction::SetMaterial(k) => set_kind(&mut design, &mut tool, k),
            UiAction::MaterialTab(n) => {
                let class = tool.classes().get(n as usize).copied();
                let current = tool.current(&design);
                if let Some(c) = class.filter(|c| *c != current.material_kind()) {
                    if let Some(k) = tool.allowed.iter().find(|k| k.material_kind() == c) {
                        let k = *k;
                        set_kind(&mut design, &mut tool, k);
                    }
                }
            }
            UiAction::StepSize(step) => {
                let k = tool.current(&design);
                if let Some((f, size, grade)) = catalog().family_of(&k.0) {
                    let next = size as i32 + step as i32;
                    if next >= 0 && (next as usize) < f.sizes.len() {
                        let mut c = f.choice(next as usize, grade);
                        c.joints = k.0.joints;
                        set_kind(&mut design, &mut tool, BeamKind(c));
                    }
                }
            }
            UiAction::CycleGrade => {
                let k = tool.current(&design);
                if let Some((f, size, grade)) = catalog().family_of(&k.0) {
                    let mut c = f.choice(size, (grade + 1) % f.grades.len());
                    c.joints = k.0.joints;
                    set_kind(&mut design, &mut tool, BeamKind(c));
                }
            }
            UiAction::CycleJoint(end) => {
                if let Some(b) = tool.selected_beam.filter(|&b| b < design.beams.len()) {
                    let end = end as usize & 1;
                    let beam = design.beams[b];
                    let allowed = catalog().joints_for(&beam.kind.0);
                    // Automatic, then each allowed type, then automatic again.
                    let next = match beam.joints[end] {
                        None => allowed.first().copied(),
                        Some(j) => allowed
                            .iter()
                            .position(|a| *a == j)
                            .and_then(|i| allowed.get(i + 1))
                            .copied(),
                    };
                    design.set_joint(b, end, next);
                }
            }
            UiAction::DeleteSelected => {
                if let Some(b) = tool.selected_beam.take() {
                    design.remove_beam(b);
                } else if let Some(h) = tool.hovered {
                    delete_node(&mut design, &mut tool, h);
                }
            }
            UiAction::Undo => {
                history.undo(&mut design);
                tool.selected = None;
                tool.selected_beam = None;
            }
            UiAction::Redo => {
                history.redo(&mut design);
                tool.selected = None;
                tool.selected_beam = None;
            }
            UiAction::Clear => {
                let mut fresh = sheets.current().initial_design();
                fresh.revision = design.revision + 1;
                *design = fresh;
                tool.selected = None;
                tool.selected_beam = None;
            }
            UiAction::CyclePlane => {
                tool.plane = match tool.plane {
                    BuildPlane::Ground => BuildPlane::Wall,
                    BuildPlane::Wall => BuildPlane::Ground,
                };
                tool.level = 0.0;
            }
            UiAction::PlaneUp => tool.level += GRID,
            UiAction::PlaneDown => tool.level -= GRID,
            UiAction::ToggleDeflection => {
                scale.0 = if scale.0 > 0.0 { 0.0 } else { 50.0 };
            }
            _ => {}
        }
    }
}

fn delete_node(design: &mut Design, tool: &mut BuildTool, h: usize) {
    if design.remove_node(h) {
        tool.selected = match tool.selected {
            Some(s) if s == h => None,
            Some(s) if s > h => Some(s - 1),
            s => s,
        };
        tool.selected_beam = None;
        tool.hovered = None;
    }
}

fn build_clicks(
    buttons: Res<ButtonInput<MouseButton>>,
    pointer: Res<Pointer>,
    overlay: Res<Overlay>,
    mut design: ResMut<Design>,
    mut tool: ResMut<BuildTool>,
) {
    if !buttons.just_pressed(MouseButton::Left) || pointer.over_ui || *overlay != Overlay::None {
        return;
    }
    match tool.tool {
        Tool::Draw => {
            if let (None, Some(b)) = (tool.hovered, tool.hovered_beam) {
                tool.selected = None;
                tool.selected_beam = Some(b);
                return;
            }
            let target = match (tool.hovered, tool.cursor) {
                (Some(h), _) => Some(h),
                (None, Some(p)) => Some(design.add_node(p)),
                (None, None) => None,
            };
            if let Some(t) = target {
                if let Some(s) = tool.selected {
                    if s != t {
                        if let Some(b) = design.add_beam(s, t, tool.kind) {
                            tool.selected_beam = Some(b);
                        }
                    }
                }
                tool.selected = Some(t);
            }
        }
        Tool::Support => {
            if let Some(h) = tool.hovered {
                design.cycle_support(h);
            }
        }
        Tool::Delete => {
            if let Some(h) = tool.hovered {
                let mut t = std::mem::take(&mut *tool);
                delete_node(&mut design, &mut t, h);
                *tool = t;
            } else if let Some(b) = tool.hovered_beam {
                design.remove_beam(b);
                tool.selected_beam = None;
            }
        }
        Tool::Load => {
            if let Some(h) = tool.hovered {
                design.cycle_load(h);
            }
        }
    }
    if tool.selected_beam.is_some_and(|b| b >= design.beams.len()) {
        tool.selected_beam = None;
    }
}

fn observe_history(
    design: Res<Design>,
    mut history: ResMut<History>,
    mut pins: ResMut<FailurePins>,
) {
    history.observe(&design);
    if !pins.pins.is_empty() && pins.beam_count != design.beams.len() {
        pins.pins.clear();
    }
}

/// F: frame the selection, or the whole structure.
fn frame_view(
    mut actions: MessageReader<UiAction>,
    design: Res<Design>,
    tool: Res<BuildTool>,
    sheets: Res<Sheets>,
    mut cam: Single<&mut OrbitCamera>,
) {
    // Also frame a sheet when it opens.
    let asked = actions.read().any(|a| *a == UiAction::FrameView) || sheets.is_changed();
    if !asked || design.nodes.is_empty() {
        return;
    }
    let pts: Vec<Vec3> = match (tool.selected_beam, tool.selected) {
        (Some(b), _) if b < design.beams.len() => {
            vec![
                design.nodes[design.beams[b].a],
                design.nodes[design.beams[b].b],
            ]
        }
        (_, Some(n)) if n < design.nodes.len() => vec![design.nodes[n]],
        _ => design.nodes.clone(),
    };
    let min = pts.iter().copied().fold(Vec3::splat(f32::MAX), Vec3::min);
    let max = pts.iter().copied().fold(Vec3::splat(f32::MIN), Vec3::max);
    cam.focus = (min + max) * 0.5;
    cam.distance = ((max - min).length() * 1.6).clamp(6.0, 120.0);
}

fn draw_build_guides(mut gizmos: Gizmos, design: Res<Design>, tool: Res<BuildTool>) {
    // On the table top the table's own grid shows the snap points; keep the
    // build-plane grid faint there so the two don't fight.
    let on_table = tool.plane == BuildPlane::Ground && tool.level == 0.0;
    let grid_color = Color::srgba(0.6, 0.65, 0.7, if on_table { 0.08 } else { 0.25 });
    let rot = match tool.plane {
        BuildPlane::Ground => Quat::from_rotation_x(std::f32::consts::FRAC_PI_2),
        BuildPlane::Wall => Quat::IDENTITY,
    };
    gizmos.grid(
        Isometry3d::new(tool.plane.origin(tool.level), rot),
        UVec2::splat(40),
        Vec2::splat(GRID),
        grid_color,
    );

    let accent = Color::srgb(0.88, 0.65, 0.15);
    // Fixed sheet nodes get a ring so the player sees what to connect.
    for (i, &p) in design.nodes.iter().enumerate() {
        if design.is_locked(i) {
            gizmos.circle(
                Isometry3d::new(p, Quat::IDENTITY),
                0.2,
                Color::srgb(0.85, 0.85, 0.8),
            );
        }
    }
    if let Some(b) = tool.selected_beam.and_then(|b| design.beams.get(b)) {
        let (a, c) = (design.nodes[b.a], design.nodes[b.b]);
        let off = Vec3::new(0.0, 0.0, 0.0);
        gizmos.line(a + off, c + off, accent);
        gizmos.sphere(Isometry3d::from_translation(a), 0.12, accent);
        gizmos.sphere(Isometry3d::from_translation(c), 0.12, accent);
    }
    if let Some(s) = tool.selected {
        let p = design.nodes[s];
        gizmos.sphere(Isometry3d::from_translation(p), 0.15, accent);
        let to = tool.hovered.map(|h| design.nodes[h]).or(tool.cursor);
        if let Some(to) = to {
            gizmos.line(p, to, accent);
        }
    }
    if let Some(h) = tool.hovered {
        gizmos.sphere(
            Isometry3d::from_translation(design.nodes[h]),
            0.18,
            Color::srgb(0.3, 0.8, 1.0),
        );
    } else if let Some(b) = tool.hovered_beam.and_then(|b| design.beams.get(b)) {
        gizmos.line(
            design.nodes[b.a],
            design.nodes[b.b],
            Color::srgb(0.3, 0.8, 1.0),
        );
    } else if let Some(c) = tool.cursor {
        gizmos.sphere(
            Isometry3d::from_translation(c),
            0.08,
            Color::srgb(0.8, 0.8, 0.8),
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn snapping_rounds_to_grid() {
        assert_eq!(snap(Vec3::new(0.26, -0.24, 1.74)), Vec3::new(0.5, 0.0, 1.5));
    }

    #[test]
    fn segment_distance_clamps_to_the_ends() {
        let (a, b) = (Vec2::ZERO, Vec2::new(10.0, 0.0));
        assert_eq!(segment_distance(Vec2::new(5.0, 3.0), a, b), 3.0);
        assert_eq!(segment_distance(Vec2::new(-4.0, 3.0), a, b), 5.0);
    }

    #[test]
    fn escape_ends_the_chain_before_dropping_the_beam() {
        let mut t = BuildTool {
            selected: Some(1),
            selected_beam: Some(0),
            ..Default::default()
        };
        assert!(t.deselect());
        assert_eq!(t.selected_beam, Some(0));
        assert!(t.deselect());
        assert!(!t.deselect());
    }
}
