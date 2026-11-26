//! Build mode: place nodes on a snapping grid, join them with beams, and set
//! supports and loads.
//!
//! Controls (also listed in the HUD):
//! - Left click: on empty grid, add a node; on a node, select it. With a node
//!   selected, the click also adds a beam from it, so clicks chain beams.
//! - Esc: deselect. Tab: switch build plane (ground / wall). Q / E: move the
//!   plane down / up one grid step.
//! - S: cycle support (none, pinned, fixed) on the hovered node.
//! - F: cycle the hovered node's load (0, 10, 50, 100, 200 kN down).
//! - X or Delete: delete the hovered node and its beams.
//! - M: switch beam material. C: clear everything. D: toggle exaggerated
//!   deflection view.

use bevy::prelude::*;
use bevy::window::PrimaryWindow;

use crate::analysis::DeflectionScale;
use crate::camera::OrbitCamera;
use crate::design::{BeamKind, Design};
use crate::Mode;

pub struct BuildPlugin;

impl Plugin for BuildPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<BuildTool>().add_systems(
            Update,
            (update_cursor, build_input, draw_build_guides)
                .chain()
                .run_if(in_state(Mode::Build)),
        );
    }
}

/// Grid spacing for snapping, metres.
pub const GRID: f32 = 0.5;
/// How close, in screen pixels, the cursor must be to pick a node.
const PICK_RADIUS_PX: f32 = 14.0;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum BuildPlane {
    /// Horizontal plane at `y = level`.
    #[default]
    Ground,
    /// Vertical plane at `z = level`.
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
            BuildPlane::Ground => "ground (XZ)",
            BuildPlane::Wall => "wall (XY)",
        }
    }
}

#[derive(Resource, Debug, Default)]
pub struct BuildTool {
    pub kind: BeamKind,
    pub plane: BuildPlane,
    /// Plane offset along its normal, metres.
    pub level: f32,
    pub selected: Option<usize>,
    /// Node under the cursor, if any.
    pub hovered: Option<usize>,
    /// Snapped point on the build plane under the cursor.
    pub cursor: Option<Vec3>,
}

pub fn snap(p: Vec3) -> Vec3 {
    (p / GRID).round() * GRID
}

fn update_cursor(
    window: Single<&Window, With<PrimaryWindow>>,
    camera: Single<(&Camera, &GlobalTransform), With<OrbitCamera>>,
    design: Res<Design>,
    mut tool: ResMut<BuildTool>,
) {
    let (camera, cam_tf) = *camera;
    tool.hovered = None;
    tool.cursor = None;
    let Some(px) = window.cursor_position() else {
        return;
    };

    // Nearest node in screen space wins.
    tool.hovered = design
        .nodes
        .iter()
        .enumerate()
        .filter_map(|(i, &p)| {
            let s = camera.world_to_viewport(cam_tf, p).ok()?;
            let d = s.distance(px);
            (d <= PICK_RADIUS_PX).then_some((i, d))
        })
        .min_by(|a, b| a.1.total_cmp(&b.1))
        .map(|(i, _)| i);

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

#[allow(clippy::too_many_arguments)]
fn build_input(
    buttons: Res<ButtonInput<MouseButton>>,
    keys: Res<ButtonInput<KeyCode>>,
    mut design: ResMut<Design>,
    mut tool: ResMut<BuildTool>,
    mut scale: ResMut<DeflectionScale>,
) {
    if keys.just_pressed(KeyCode::Escape) {
        tool.selected = None;
    }
    if keys.just_pressed(KeyCode::Tab) {
        tool.plane = match tool.plane {
            BuildPlane::Ground => BuildPlane::Wall,
            BuildPlane::Wall => BuildPlane::Ground,
        };
        tool.level = 0.0;
    }
    if keys.just_pressed(KeyCode::KeyE) {
        tool.level += GRID;
    }
    if keys.just_pressed(KeyCode::KeyQ) {
        tool.level -= GRID;
    }
    if keys.just_pressed(KeyCode::KeyM) {
        tool.kind = tool.kind.next();
    }
    if keys.just_pressed(KeyCode::KeyD) {
        scale.0 = if scale.0 > 0.0 { 0.0 } else { 50.0 };
    }
    if keys.just_pressed(KeyCode::KeyC) {
        *design = Design {
            revision: design.revision + 1,
            ..Default::default()
        };
        tool.selected = None;
        return;
    }

    if let Some(h) = tool.hovered {
        if keys.just_pressed(KeyCode::KeyS) {
            design.cycle_support(h);
        }
        if keys.just_pressed(KeyCode::KeyF) {
            design.cycle_load(h);
        }
        if keys.any_just_pressed([KeyCode::KeyX, KeyCode::Delete]) {
            design.remove_node(h);
            tool.selected = match tool.selected {
                Some(s) if s == h => None,
                Some(s) if s > h => Some(s - 1),
                s => s,
            };
            tool.hovered = None;
            return;
        }
    }

    if buttons.just_pressed(MouseButton::Left) {
        let target = match (tool.hovered, tool.cursor) {
            (Some(h), _) => Some(h),
            (None, Some(p)) => Some(design.add_node(p)),
            (None, None) => None,
        };
        if let Some(t) = target {
            if let Some(s) = tool.selected {
                if s != t {
                    design.add_beam(s, t, tool.kind);
                }
            }
            tool.selected = Some(t);
        }
    }
}

fn draw_build_guides(mut gizmos: Gizmos, design: Res<Design>, tool: Res<BuildTool>) {
    let grid_color = Color::srgba(0.6, 0.65, 0.7, 0.25);
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

    if let Some(s) = tool.selected {
        let p = design.nodes[s];
        gizmos.sphere(
            Isometry3d::from_translation(p),
            0.15,
            Color::srgb(1.0, 0.85, 0.2),
        );
        let to = tool.hovered.map(|h| design.nodes[h]).or(tool.cursor);
        if let Some(to) = to {
            gizmos.line(p, to, Color::srgb(1.0, 0.85, 0.2));
        }
    }
    if let Some(h) = tool.hovered {
        gizmos.sphere(
            Isometry3d::from_translation(design.nodes[h]),
            0.18,
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
}
