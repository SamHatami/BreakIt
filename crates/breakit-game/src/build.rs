//! Build mode: place nodes in 3D, join them with beams, set supports and
//! loads, select members.
//!
//! Input arrives as [`UiAction`]s (keys and buttons, see `flow.rs` for the
//! map) plus mouse clicks on the scene. A left click acts by the current
//! [`Tool`]:
//! - Draw (B): click empty space to add a node, a node to pick it; with a
//!   node picked, each click adds a beam from it, so clicks chain beams.
//!   Clicking a beam selects it for the inspector.
//! - Support (S): click a node to cycle none, pinned, fixed.
//! - Delete (X): click a node or a beam to delete it.
//! - Load (L, sandbox only): click a node to cycle its load.
//!
//! A right click (without dragging, which orbits) always cancels: it ends
//! the chain, else drops the selection, else goes back to Draw.
//!
//! Where a click lands ([`aim`]): an existing node or a foundation under
//! the cursor wins, in any direction; with Shift held the next node slides
//! along the x, y or z axis through the chain's last node; otherwise it is
//! the grid point on the work plane. The work plane passes through the
//! chain's last node and turns to face the camera (Tab locks it to the
//! floor, front or side plane; Q / E step it).
//!
//! Fixed scenario nodes (anchors, load points) can't be deleted or have
//! their support changed. [`aids`] draws the previews, the snap feedback
//! and the hint next to the cursor.

pub mod aids;
pub mod aim;
mod demo;

use bevy::input::mouse::AccumulatedMouseMotion;
use bevy::prelude::*;
use bevy::window::PrimaryWindow;

use crate::analysis::DeflectionScale;
use crate::camera::OrbitCamera;
use crate::design::{catalog, BeamKind, Design};
use crate::flow::{FailurePins, Overlay, Screen, UiAction};
use crate::history::History;
use crate::scenario::{Scenario, Sheets};
use crate::ui::pointer::Pointer;

pub use aim::{snap, Axis, BuildPlane, PlaneMode};

pub struct BuildPlugin;

impl Plugin for BuildPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<BuildTool>()
            .init_resource::<History>()
            .add_plugins((aids::BuildAidsPlugin, demo::BuildDemoPlugin))
            .add_systems(
                Update,
                (
                    update_cursor,
                    build_actions,
                    right_click_cancel,
                    build_clicks,
                    observe_history,
                    aids::update_guide,
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
/// A right press that moves less than this, in pixels, is a click (cancel)
/// rather than an orbit drag.
const CLICK_SLOP_PX: f32 = 5.0;

/// What the cursor is aimed at.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum Aim {
    /// Nothing under the cursor (over the UI, or the plane is edge-on).
    #[default]
    None,
    /// An existing node.
    Node(usize),
    /// A sheet foundation with no node on it yet (index into the
    /// design's anchors).
    Anchor(usize),
    /// A member, when no node is near.
    Beam(usize),
    /// A grid point on the work plane.
    Grid,
    /// A point on the axis through the chain's last node (Shift).
    Axis(Axis),
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
            Tool::Draw => "Click to place a node, clicks chain beams. Click a beam to inspect it. Right-click ends the chain.",
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
    /// The work plane in use this frame.
    pub plane: BuildPlane,
    /// Auto (faces the camera) or locked by Tab.
    pub plane_mode: PlaneMode,
    /// The work plane's coordinate along its normal, metres: where it
    /// meets `base`, plus `offset`.
    pub level: f32,
    /// Point the work plane passes through: the chain's last node, else
    /// where the last chain was.
    pub base: Vec3,
    /// Q / E steps of the plane away from `base`, metres.
    pub offset: f32,
    /// What the cursor is aimed at.
    pub aim: Aim,
    /// The aimed point lies outside the sheet's build area.
    pub outside: bool,
    /// The sheet's build area (challenge sheets).
    pub volume: Option<(Vec3, Vec3)>,
    /// Next step for the player, shown under the scene.
    pub guide: String,
    /// Start of the beam chain being drawn.
    pub selected: Option<usize>,
    /// Beam shown in the inspector.
    pub selected_beam: Option<usize>,
    /// Node under the cursor, if any.
    pub hovered: Option<usize>,
    /// Beam under the cursor, if no node is.
    pub hovered_beam: Option<usize>,
    /// Where a click would put a node: the aimed node, foundation, axis
    /// or grid point.
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
        let volume = (s.mode != breakit_scenario::PlayMode::Sandbox).then(|| {
            let (lo, hi) = (s.def.limits.volume.min, s.def.limits.volume.max);
            (
                Vec3::new(lo[0] as f32, lo[1] as f32, lo[2] as f32),
                Vec3::new(hi[0] as f32, hi[1] as f32, hi[2] as f32),
            )
        });
        *self = BuildTool {
            loads_allowed: s.sandbox,
            allowed: s.allowed.clone(),
            kind: s.allowed.first().copied().unwrap_or_default(),
            // Start the work plane on the first foundation, so the front
            // plane is the bridge's and the floor plane is its footing.
            base: central_anchor(s),
            volume,
            ..Default::default()
        };
    }

    /// Whether a point is inside the sheet's build area.
    pub fn inside(&self, p: Vec3) -> bool {
        self.volume
            .is_none_or(|(lo, hi)| p.cmpge(lo - 1e-3).all() && p.cmple(hi + 1e-3).all())
    }

    /// Tool-bar label of the plane button.
    pub fn plane_button(&self) -> String {
        match self.plane_mode {
            PlaneMode::Auto => format!("Plane auto {}", self.plane.label()),
            PlaneMode::Locked(p) => format!("Plane {}", p.label()),
        }
    }

    /// One-line read-out of the work plane.
    pub fn plane_readout(&self) -> String {
        let how = match self.plane_mode {
            PlaneMode::Auto => "turns with the view",
            PlaneMode::Locked(_) => "locked",
        };
        format!(
            "{} plane {} at {} {:+.1} m, {how}   Tab change, Q / E move   snap {:.1} m",
            self.plane.name(),
            self.plane.label(),
            self.plane.axis(),
            self.level,
            GRID
        )
    }

    /// Right click: ends the chain, else drops the selected beam, else goes
    /// back to the draw tool. Returns whether it did anything.
    pub fn cancel(&mut self) -> bool {
        if self.deselect() {
            return true;
        }
        if self.tool != Tool::Draw {
            self.tool = Tool::Draw;
            return true;
        }
        false
    }

    /// Moves the work plane onto the chain's last node when that changes,
    /// and works out its level.
    fn follow_chain(&mut self, design: &Design, last: &mut Option<usize>) {
        if self.selected != *last {
            *last = self.selected;
            if let Some(p) = self.selected.and_then(|s| design.nodes.get(s)) {
                self.base = *p;
                self.offset = 0.0;
            }
        }
        self.level = self.plane.level_of(self.base) + self.offset;
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

/// The foundation nearest the middle of them all, where the work plane
/// starts.
fn central_anchor(s: &Scenario) -> Vec3 {
    if s.anchors.is_empty() {
        return Vec3::ZERO;
    }
    let mid = s.anchors.iter().map(|a| a.0).sum::<Vec3>() / s.anchors.len() as f32;
    s.anchors
        .iter()
        .map(|a| a.0)
        .min_by(|a, b| a.distance(mid).total_cmp(&b.distance(mid)))
        .unwrap_or(Vec3::ZERO)
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

#[allow(clippy::too_many_arguments)]
fn update_cursor(
    window: Single<&Window, With<PrimaryWindow>>,
    camera: Single<(&Camera, &GlobalTransform), With<OrbitCamera>>,
    keys: Res<ButtonInput<KeyCode>>,
    design: Res<Design>,
    pointer: Res<Pointer>,
    overlay: Res<Overlay>,
    mut tool: ResMut<BuildTool>,
    mut last_chain: Local<Option<usize>>,
) {
    let (camera, cam_tf) = *camera;
    let forward = *cam_tf.forward();
    tool.plane = tool.plane_mode.resolve(forward, tool.plane);
    tool.follow_chain(&design, &mut last_chain);

    tool.hovered = None;
    tool.hovered_beam = None;
    tool.cursor = None;
    tool.aim = Aim::None;
    tool.outside = false;
    let Some(px) = window.cursor_position() else {
        return;
    };
    if pointer.over_ui || *overlay != Overlay::None {
        return;
    }
    let to_screen = |p: Vec3| camera.world_to_viewport(cam_tf, p).ok();

    let screen: Vec<Option<Vec2>> = design.nodes.iter().map(|&p| to_screen(p)).collect();
    let nearest = |pts: &mut dyn Iterator<Item = (usize, Option<Vec2>)>, r: f32| {
        pts.filter_map(|(i, s)| {
            let d = s?.distance(px);
            (d <= r).then_some((i, d))
        })
        .min_by(|a, b| a.1.total_cmp(&b.1))
        .map(|(i, _)| i)
    };

    // An existing node under the cursor wins, wherever it is in 3D.
    tool.hovered = nearest(&mut screen.iter().copied().enumerate(), PICK_RADIUS_PX);
    if let Some(h) = tool.hovered {
        tool.aim = Aim::Node(h);
        tool.cursor = Some(design.nodes[h]);
        return;
    }
    // Then a foundation nothing stands on yet.
    let free_anchor = |&(i, (p, _)): &(usize, &(Vec3, _))| {
        design
            .node_at(*p, 0.05)
            .is_none()
            .then_some((i, to_screen(*p)))
    };
    let anchor = nearest(
        &mut design
            .anchors
            .iter()
            .enumerate()
            .filter_map(|e| free_anchor(&e)),
        PICK_RADIUS_PX * 1.4,
    );
    if let Some(a) = anchor {
        tool.aim = Aim::Anchor(a);
        tool.cursor = Some(design.anchors[a].0);
        return;
    }

    let Ok(ray) = camera.viewport_to_world(cam_tf, px) else {
        return;
    };
    let (origin, dir) = (ray.origin, *ray.direction);
    let shift = keys.any_pressed([KeyCode::ShiftLeft, KeyCode::ShiftRight]);
    let chain = tool
        .selected
        .filter(|_| tool.tool == Tool::Draw)
        .and_then(|s| design.nodes.get(s).copied());

    // Shift with a chain: slide along an axis through its last node, so
    // columns go straight up and beams straight out of the plane.
    if let (true, Some(start)) = (shift, chain) {
        if let Some((axis, p)) = aim::axis_lock(start, origin, dir, px, to_screen) {
            tool.aim = Aim::Axis(axis);
            tool.cursor = Some(p);
            tool.outside = !tool.inside(p);
            return;
        }
    }

    // A member under the cursor, for the inspector or the delete tool;
    // while drawing a chain the grid point is the better target.
    if chain.is_none() {
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
        if let Some(b) = tool.hovered_beam {
            tool.aim = Aim::Beam(b);
            return;
        }
    }

    if let Some(p) = aim::on_plane(origin, dir, tool.plane, tool.level) {
        tool.aim = Aim::Grid;
        tool.cursor = Some(p);
        tool.outside = !tool.inside(p);
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
                tool.plane_mode = tool.plane_mode.next();
                tool.offset = 0.0;
            }
            UiAction::PlaneUp => tool.offset += GRID,
            UiAction::PlaneDown => tool.offset -= GRID,
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

/// A right click that did not drag cancels; a drag orbits the camera.
fn right_click_cancel(
    buttons: Res<ButtonInput<MouseButton>>,
    keys: Res<ButtonInput<KeyCode>>,
    motion: Res<AccumulatedMouseMotion>,
    pointer: Res<Pointer>,
    overlay: Res<Overlay>,
    mut tool: ResMut<BuildTool>,
    mut dragged: Local<f32>,
) {
    if buttons.just_pressed(MouseButton::Right) {
        *dragged = 0.0;
    }
    if buttons.pressed(MouseButton::Right) {
        *dragged += motion.delta.length();
    }
    let shift = keys.any_pressed([KeyCode::ShiftLeft, KeyCode::ShiftRight]);
    if buttons.just_released(MouseButton::Right)
        && *dragged < CLICK_SLOP_PX
        && !shift
        && !pointer.over_ui
        && *overlay == Overlay::None
    {
        tool.cancel();
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
            if let Aim::Beam(b) = tool.aim {
                tool.selected = None;
                tool.selected_beam = Some(b);
                return;
            }
            let target = match (tool.aim, tool.cursor) {
                (Aim::Node(h), _) => Some(h),
                (Aim::None | Aim::Beam(_), _) | (_, None) => None,
                (_, Some(p)) => Some(design.add_node(p)),
            };
            match target {
                Some(t) => {
                    if let Some(s) = tool.selected {
                        if s != t {
                            if let Some(b) = design.add_beam(s, t, tool.kind) {
                                tool.selected_beam = Some(b);
                            }
                        }
                    }
                    tool.selected = Some(t);
                }
                // A click on nothing ends the chain.
                None if tool.aim == Aim::None => {
                    tool.selected = None;
                }
                None => {}
            }
        }
        Tool::Support => match (tool.aim, tool.cursor) {
            (Aim::Node(h), _) if aids::support_allowed(&design, h) => design.cycle_support(h),
            // A foundation with nothing on it: put a node there, which
            // takes the foundation's support.
            (Aim::Anchor(_), Some(p)) => {
                design.add_node(p);
            }
            _ => {}
        },
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
    // Close enough that the structure fills the middle of the screen
    // between the side panels.
    cam.distance = ((max - min).length() * 1.2).clamp(4.0, 120.0);
}

#[cfg(test)]
mod tests {
    use super::*;

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

    #[test]
    fn right_click_cancels_chain_then_selection_then_tool() {
        let mut t = BuildTool {
            tool: Tool::Support,
            selected: Some(2),
            selected_beam: Some(1),
            ..Default::default()
        };
        assert!(t.cancel());
        assert_eq!((t.selected, t.selected_beam), (None, Some(1)));
        assert!(t.cancel());
        assert_eq!(t.selected_beam, None);
        assert_eq!(t.tool, Tool::Support);
        assert!(t.cancel());
        assert_eq!(t.tool, Tool::Draw);
        assert!(!t.cancel());
    }

    #[test]
    fn work_plane_follows_the_chain() {
        let mut d = Design::default();
        let a = d.add_node(Vec3::new(1.0, 2.0, 3.0));
        let mut t = BuildTool {
            plane: BuildPlane::Wall,
            offset: 0.5,
            ..Default::default()
        };
        let mut last = None;
        t.follow_chain(&d, &mut last);
        assert_eq!(t.level, 0.5);
        t.selected = Some(a);
        t.follow_chain(&d, &mut last);
        assert_eq!((t.base, t.level), (d.nodes[a], 3.0));
        t.plane = BuildPlane::Ground;
        t.follow_chain(&d, &mut last);
        assert_eq!(t.level, 2.0);
        // The plane stays where the chain ended.
        t.selected = None;
        t.follow_chain(&d, &mut last);
        assert_eq!(t.level, 2.0);
    }
}
