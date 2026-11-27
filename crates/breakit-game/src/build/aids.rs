//! What build mode shows the player while they work: the work plane, the
//! preview of the next beam, snap feedback, the selection drawn over the
//! members, the hint next to the cursor and the next step under the scene.
//!
//! The previews and the selection are drawn on top of the scene (gizmos
//! with a depth bias of -1), so a member never hides its own highlight.

use bevy::gizmos::config::{GizmoConfig, GizmoLineConfig, GizmoLineStyle};
use bevy::prelude::*;
use bevy::window::PrimaryWindow;

use super::{Aim, BuildPlane, BuildTool, PlaneMode, Tool, GRID};
use crate::analysis::FrameState;
use crate::camera::OrbitCamera;
use crate::design::{Design, SupportKind};
use crate::flow::{Overlay, Screen};
use crate::ui::theme::{Swatch, TextRole};
use crate::ui::widgets::text;

pub struct BuildAidsPlugin;

impl Plugin for BuildAidsPlugin {
    fn build(&self, app: &mut App) {
        app.insert_gizmo_config(
            OnTop,
            GizmoConfig {
                depth_bias: -1.0,
                line: GizmoLineConfig {
                    width: 3.0,
                    ..default()
                },
                ..default()
            },
        )
        .insert_gizmo_config(
            OnTopThin,
            GizmoConfig {
                depth_bias: -1.0,
                line: GizmoLineConfig {
                    width: 1.5,
                    style: GizmoLineStyle::Dashed {
                        gap_scale: 3.0,
                        line_scale: 5.0,
                    },
                    ..default()
                },
                ..default()
            },
        )
        .add_systems(OnEnter(Screen::Build), spawn_cursor_tag)
        .add_systems(
            Update,
            (draw_build_guides, update_cursor_tag)
                .after(super::build_clicks)
                .run_if(in_state(Screen::Build)),
        );
    }
}

/// Gizmos drawn over everything: selection, previews, snap marks.
#[derive(Default, Reflect, GizmoConfigGroup)]
pub struct OnTop;

/// Thin dashed guides over everything: axis lines, drop lines.
#[derive(Default, Reflect, GizmoConfigGroup)]
pub struct OnTopThin;

/// Selection and the next beam.
const ACCENT: Color = Color::srgb(1.0, 0.54, 0.24);
/// What the cursor is over.
const HOVER: Color = Color::srgb(0.35, 0.85, 1.0);
const FAIL: Color = Color::srgb(1.0, 0.36, 0.3);
/// Sheet foundations not built on yet.
const FOUNDATION: Color = Color::srgb(0.95, 0.78, 0.25);

fn support_name(k: SupportKind) -> &'static str {
    match k {
        SupportKind::Pinned => "pinned",
        SupportKind::Fixed => "fixed",
    }
}

/// Whether the support tool may change this node: sheets with foundations
/// only take supports on them.
pub fn support_allowed(design: &Design, node: usize) -> bool {
    if design.is_locked(node) {
        return false;
    }
    design.anchors.is_empty()
        || design
            .anchors
            .iter()
            .any(|(a, _)| a.distance(design.nodes[node]) <= 0.05)
}

/// The two lines next to the cursor: what a click does here, and a detail.
/// The flag marks a click that won't work.
pub fn cursor_hint(tool: &BuildTool, design: &Design) -> Option<(String, String, bool)> {
    let chain = tool.selected.and_then(|s| design.nodes.get(s).copied());
    let len = |to: Vec3| chain.map(|c| c.distance(to));
    let coords = |p: Vec3| format!("x {:+.1}   y {:+.1}   z {:+.1} m", p.x, p.y, p.z);
    let s = |a: &str, b: String, fail: bool| Some((a.to_string(), b, fail));
    match tool.tool {
        Tool::Draw => match tool.aim {
            Aim::Node(h) if tool.selected == Some(h) => s(
                "Chain starts here",
                "click where the beam goes, right-click to stop".into(),
                false,
            ),
            Aim::Node(h) => match len(design.nodes[h]) {
                Some(l) => s(
                    &format!("Connect  {l:.1} m beam"),
                    "to this node".into(),
                    false,
                ),
                None if design.is_locked(h) => s(
                    "Payload point",
                    "click to start a beam from it".into(),
                    false,
                ),
                None => s(
                    "Start from this node",
                    "then click where the beam goes".into(),
                    false,
                ),
            },
            Aim::Anchor(a) => {
                let kind = support_name(design.anchors[a].1);
                match len(design.anchors[a].0) {
                    Some(l) => s(
                        &format!("Beam to foundation  {l:.1} m"),
                        format!("the node there gets a {kind} support"),
                        false,
                    ),
                    None => s(
                        "Foundation: start here",
                        format!("a node here gets a {kind} support"),
                        false,
                    ),
                }
            }
            Aim::Beam(b) => s(
                &format!("Select member {}", b + 1),
                design.beams[b].kind.designation(),
                false,
            ),
            Aim::Axis(_) | Aim::Grid if tool.outside => s(
                "Outside the build area",
                "nodes must stay inside the outlined box".into(),
                true,
            ),
            Aim::Axis(axis) => {
                let p = tool.cursor?;
                s(
                    &format!("Beam  {:.1} m  {}", len(p).unwrap_or(0.0), axis.label()),
                    format!("{}   release Shift for the plane", coords(p)),
                    false,
                )
            }
            Aim::Grid => {
                let p = tool.cursor?;
                match len(p) {
                    Some(l) if l < 1e-3 => None,
                    Some(l) => s(
                        &format!("Beam  {l:.1} m"),
                        format!("{}   Shift: straight up or out", coords(p)),
                        false,
                    ),
                    None => s("Place a node", coords(p), false),
                }
            }
            Aim::None => None,
        },
        Tool::Support => match tool.aim {
            Aim::Node(h) if design.is_locked(h) => {
                s("Payload point", "the sheet sets its support".into(), true)
            }
            Aim::Node(h) if !support_allowed(design, h) => s(
                "No foundation here",
                "supports only go on the yellow foundations".into(),
                true,
            ),
            Aim::Node(h) => match design.supports[h] {
                None => s(
                    "Click: pinned support",
                    "holds the node in place, lets it turn".into(),
                    false,
                ),
                Some(SupportKind::Pinned) => s(
                    "Click: fixed support",
                    "holds the node and stops it turning".into(),
                    false,
                ),
                Some(SupportKind::Fixed) => s(
                    "Click: remove the support",
                    "the node is free again".into(),
                    false,
                ),
            },
            Aim::Anchor(a) => s(
                "Click: node on this foundation",
                format!("it gets a {} support", support_name(design.anchors[a].1)),
                false,
            ),
            _ => None,
        },
        Tool::Delete => match tool.aim {
            Aim::Node(h) if design.is_locked(h) => {
                s("Sheet node", "it can't be deleted".into(), true)
            }
            Aim::Node(h) => {
                let n = design.beams.iter().filter(|b| b.a == h || b.b == h).count();
                let what = match n {
                    0 => "it has no beams".to_string(),
                    1 => "and its beam".to_string(),
                    n => format!("and its {n} beams"),
                };
                s("Click: delete node", what, false)
            }
            Aim::Beam(b) => s(
                &format!("Click: delete member {}", b + 1),
                design.beams[b].kind.designation(),
                false,
            ),
            _ => None,
        },
        Tool::Load => match tool.aim {
            Aim::Node(h) => {
                let kn = design.loads[h].length() / 1000.0;
                let now = if kn > 0.0 {
                    format!("now {kn:.0} kN")
                } else {
                    "no load yet".into()
                };
                s("Click: next load", now, false)
            }
            _ => None,
        },
    }
}

/// The next step, shown under the scene.
pub fn guide_text(tool: &BuildTool, design: &Design) -> String {
    let anchors = !design.anchors.is_empty();
    match tool.tool {
        Tool::Draw if tool.selected.is_some() => "Click to add the next beam, or click a node to connect to it. \
             Hold Shift to go straight up or out of the plane. Right-click to stop."
            .into(),
        Tool::Draw if design.beams.is_empty() && anchors => {
            "Start at a yellow foundation: click it, then click where the next node goes. \
             Each click adds a beam."
                .into()
        }
        Tool::Draw if design.beams.is_empty() => {
            "Click to place a node, then click again to add a beam from it.".into()
        }
        Tool::Draw => match tool.selected_beam {
            Some(b) if b < design.beams.len() => format!(
                "Member {} selected: change its part on the left, Del deletes it. \
                 Right-click to deselect.",
                b + 1
            ),
            _ => "Click a node to build on from it. Right-drag to orbit: \
                  the work plane turns to face you, so you can build in 3D."
                .into(),
        },
        Tool::Support if anchors => {
            "Click a node on a foundation to cycle its support: pinned, fixed, none. \
             Right-click to go back to drawing."
                .into()
        }
        Tool::Support => {
            "Click a node to cycle its support: pinned, fixed, none. Right-click to go back to drawing."
                .into()
        }
        t => format!("{} Right-click to go back to drawing.", t.hint()),
    }
}

pub fn update_guide(design: Res<Design>, mut tool: ResMut<BuildTool>) {
    let g = guide_text(&tool, &design);
    if tool.guide != g {
        tool.guide = g;
    }
}

/// Screen-constant size: `px` pixels at the point's distance.
fn px_size(cam: Vec3, p: Vec3, px: f32) -> f32 {
    cam.distance(p) * px * 0.0011
}

/// A ring that faces the camera.
fn ring<C: GizmoConfigGroup>(g: &mut Gizmos<C>, cam: Vec3, p: Vec3, r: f32, color: Color) {
    let to_cam = (cam - p).normalize_or(Vec3::Z);
    g.circle(
        Isometry3d::new(p, Quat::from_rotation_arc(Vec3::Z, to_cam)),
        r,
        color,
    );
}

/// A box hugging a member, drawn over it.
fn member_box<C: GizmoConfigGroup>(g: &mut Gizmos<C>, a: Vec3, b: Vec3, thick: f32, color: Color) {
    let d = b - a;
    let l = d.length();
    if l < 1e-4 {
        return;
    }
    let dir = d / l;
    let up = if dir.y.abs() > 0.99 { Vec3::X } else { Vec3::Y };
    g.cube(
        Transform::from_translation((a + b) * 0.5)
            .looking_to(dir, up)
            .with_scale(Vec3::new(thick, thick, l + thick)),
        color,
    );
}

/// How thick a member's outline is: its section plus a margin.
fn outline_size(design: &Design, beam: usize) -> f32 {
    let p = design.beams[beam].kind.style().profile;
    2.0 * p.half_depth().max(p.half_width()) + 0.08
}

#[allow(clippy::too_many_arguments)]
fn draw_build_guides(
    mut gizmos: Gizmos,
    mut top: Gizmos<OnTop>,
    mut thin: Gizmos<OnTopThin>,
    design: Res<Design>,
    state: Res<FrameState>,
    tool: Res<BuildTool>,
    overlay: Res<Overlay>,
    time: Res<Time>,
    camera: Single<&GlobalTransform, With<OrbitCamera>>,
) {
    let cam = camera.translation();
    let pos = |i: usize| {
        state
            .positions
            .get(i)
            .copied()
            .filter(|_| state.positions.len() == design.nodes.len())
            .unwrap_or(design.nodes[i])
    };

    // The work plane: a patch of grid around the cursor (or the plane's
    // base), so it reads as a sheet in space rather than a floor.
    let n = tool.plane.normal();
    let centre = tool
        .cursor
        .filter(|_| tool.aim == Aim::Grid)
        .unwrap_or(tool.base);
    let centre = super::snap(centre - n * centre.dot(n)) + n * tool.level;
    let on_table = tool.plane == BuildPlane::Ground && tool.level.abs() < 1e-3;
    let rot = match tool.plane {
        BuildPlane::Ground => Quat::from_rotation_x(std::f32::consts::FRAC_PI_2),
        BuildPlane::Wall => Quat::IDENTITY,
        BuildPlane::Side => Quat::from_rotation_y(std::f32::consts::FRAC_PI_2),
    };
    let alpha = if on_table { 0.08 } else { 0.18 };
    gizmos.grid(
        Isometry3d::new(centre, rot),
        UVec2::splat(16),
        Vec2::splat(GRID),
        Color::srgba(0.65, 0.75, 0.85, alpha),
    );
    if tool.plane_mode != PlaneMode::Auto {
        gizmos.rect(
            Isometry3d::new(centre, rot),
            Vec2::splat(16.0 * GRID),
            Color::srgba(0.65, 0.75, 0.85, alpha + 0.2),
        );
    }

    // Foundations nothing stands on yet pulse until the first beam.
    let pulse = 1.0 + 0.15 * (time.elapsed_secs() * 3.0).sin();
    for (i, (p, _)) in design.anchors.iter().enumerate() {
        if design.node_at(*p, 0.05).is_some() {
            continue;
        }
        let r = px_size(cam, *p, 16.0) * if design.beams.is_empty() { pulse } else { 1.0 };
        let c = if tool.aim == Aim::Anchor(i) {
            HOVER
        } else {
            FOUNDATION
        };
        ring(&mut top, cam, *p, r, c);
    }
    // Fixed sheet nodes get a ring so the player sees what to connect.
    for (i, _) in design.nodes.iter().enumerate() {
        if design.is_locked(i) {
            ring(
                &mut top,
                cam,
                pos(i),
                px_size(cam, pos(i), 13.0),
                Color::srgb(0.9, 0.9, 0.85),
            );
        }
    }

    if *overlay != Overlay::None {
        return;
    }

    let deleting = tool.tool == Tool::Delete;
    let hover = if deleting { FAIL } else { HOVER };

    // The selected member, outlined over the scene.
    if let Some(b) = tool.selected_beam.filter(|&b| b < design.beams.len()) {
        let m = design.beams[b];
        member_box(
            &mut top,
            pos(m.a),
            pos(m.b),
            outline_size(&design, b),
            ACCENT,
        );
    }
    if let Some(b) = tool.hovered_beam.filter(|&b| b < design.beams.len()) {
        if tool.selected_beam != Some(b) {
            let m = design.beams[b];
            member_box(
                &mut top,
                pos(m.a),
                pos(m.b),
                outline_size(&design, b),
                hover,
            );
        }
    }

    // The chain's last node, and the beam the next click adds.
    if let Some(s) = tool.selected.filter(|&s| s < design.nodes.len()) {
        let p = pos(s);
        let r = px_size(cam, p, 11.0);
        top.sphere(Isometry3d::from_translation(p), r * 0.6, ACCENT);
        ring(&mut top, cam, p, r * 1.4, ACCENT);
        if let Some(to) = tool.cursor.filter(|_| tool.tool == Tool::Draw) {
            let bad = tool.outside || to.distance(p) < 1e-3;
            top.line(p, to, if bad { FAIL } else { ACCENT });
            if let Aim::Axis(axis) = tool.aim {
                let d = axis.dir() * 40.0;
                thin.line(p - d, p + d, axis.color());
            }
        }
    }

    match tool.aim {
        Aim::Node(h) => {
            let p = pos(h);
            let c = match tool.tool {
                Tool::Support if !support_allowed(&design, h) => FAIL,
                Tool::Delete if design.is_locked(h) => FAIL,
                _ => hover,
            };
            ring(&mut top, cam, p, px_size(cam, p, 15.0), c);
            if tool.tool == Tool::Support && c != FAIL {
                support_ghost(&mut top, p, design.supports[h]);
            }
        }
        Aim::Anchor(a) => {
            let p = design.anchors[a].0;
            ring(&mut top, cam, p, px_size(cam, p, 22.0), HOVER);
            top.sphere(Isometry3d::from_translation(p), px_size(cam, p, 5.0), HOVER);
        }
        Aim::Grid | Aim::Axis(_) if tool.tool == Tool::Draw => {
            let Some(p) = tool.cursor else { return };
            let c = if tool.outside { FAIL } else { Color::WHITE };
            let r = px_size(cam, p, 7.0);
            let (u, v) = tool.plane.axes();
            top.line(p - u * r, p + u * r, c);
            top.line(p - v * r, p + v * r, c);
            ring(&mut top, cam, p, r, c);
            // A drop line to the table top shows how high the point is.
            if p.y.abs() > 1e-3 {
                let foot = Vec3::new(p.x, 0.0, p.z);
                thin.line(p, foot, Color::srgba(1.0, 1.0, 1.0, 0.6));
                top.circle(
                    Isometry3d::new(foot, Quat::from_rotation_x(std::f32::consts::FRAC_PI_2)),
                    r * 0.6,
                    Color::srgba(1.0, 1.0, 1.0, 0.6),
                );
            }
        }
        _ => {}
    }

    // The build area's edges, where the cursor leaves it.
    if tool.outside {
        if let Some((lo, hi)) = tool.volume {
            top.cube(
                Transform::from_translation((lo + hi) * 0.5).with_scale(hi - lo),
                FAIL.with_alpha(0.7),
            );
        }
    }
}

/// What the support tool's click would make, drawn at the node.
fn support_ghost(g: &mut Gizmos<OnTop>, p: Vec3, now: Option<SupportKind>) {
    let c = ACCENT.with_alpha(0.9);
    match now {
        // Next: pinned. A pyramid under the node.
        None => {
            let base = p - Vec3::Y * 0.3;
            let k = [(0.2, 0.2), (-0.2, 0.2), (-0.2, -0.2), (0.2, -0.2)];
            let corners = k.map(|(x, z)| base + Vec3::new(x, 0.0, z));
            for i in 0..4 {
                g.line(p, corners[i], c);
                g.line(corners[i], corners[(i + 1) % 4], c);
            }
        }
        // Next: fixed. A block under the node.
        Some(SupportKind::Pinned) => g.cube(
            Transform::from_translation(p - Vec3::Y * 0.15).with_scale(Vec3::splat(0.3)),
            c,
        ),
        // Next: none. A cross through the node.
        Some(SupportKind::Fixed) => {
            let r = 0.25;
            g.line(p + Vec3::new(-r, -r, 0.0), p + Vec3::new(r, r, 0.0), FAIL);
            g.line(p + Vec3::new(-r, r, 0.0), p + Vec3::new(r, -r, 0.0), FAIL);
        }
    }
}

/// The hint box that follows the cursor.
#[derive(Component)]
struct CursorTag;

#[derive(Component)]
struct CursorTagLine(u8);

fn spawn_cursor_tag(mut commands: Commands) {
    commands
        .spawn((
            CursorTag,
            Node {
                position_type: PositionType::Absolute,
                flex_direction: FlexDirection::Column,
                row_gap: Val::Px(2.0),
                padding: UiRect::axes(Val::Px(10.0), Val::Px(6.0)),
                ..default()
            },
            crate::ui::theme::Skin::fill(Swatch::Scrim),
            Visibility::Hidden,
            GlobalZIndex(50),
            DespawnOnExit(Screen::Build),
        ))
        .with_children(|c| {
            c.spawn((
                text("", TextRole::Body, Swatch::SceneText),
                CursorTagLine(0),
            ));
            c.spawn((text("", TextRole::Label, Swatch::Muted), CursorTagLine(1)));
        });
}

fn update_cursor_tag(
    window: Single<&Window, With<PrimaryWindow>>,
    tool: Res<BuildTool>,
    design: Res<Design>,
    mut tag: Query<(&mut Node, &mut Visibility), With<CursorTag>>,
    mut lines: Query<(&CursorTagLine, &mut Text, &mut crate::ui::theme::Ink)>,
) {
    let Ok((mut node, mut vis)) = tag.single_mut() else {
        return;
    };
    let hint = window
        .cursor_position()
        .and_then(|px| Some((px, cursor_hint(&tool, &design)?)));
    let Some((px, (head, detail, fail))) = hint else {
        if *vis != Visibility::Hidden {
            *vis = Visibility::Hidden;
        }
        return;
    };
    *vis = Visibility::Inherited;
    // Below right of the cursor, flipped near the window's edges.
    let (w, h) = (window.width(), window.height());
    let x = if px.x > w - 340.0 {
        px.x - 330.0
    } else {
        px.x + 22.0
    };
    let y = if px.y > h - 140.0 {
        px.y - 70.0
    } else {
        px.y + 22.0
    };
    node.left = Val::Px(x);
    node.top = Val::Px(y);
    for (line, mut t, mut ink) in &mut lines {
        let (s, role, swatch) = match line.0 {
            0 => (
                &head,
                TextRole::Body,
                if fail {
                    Swatch::Fail
                } else {
                    Swatch::SceneText
                },
            ),
            _ => (&detail, TextRole::Label, Swatch::Muted),
        };
        if t.0 != *s {
            t.0 = s.clone();
        }
        let want = crate::ui::theme::Ink::new(role, swatch);
        if *ink != want {
            *ink = want;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::design::BeamKind;

    fn bridge() -> Design {
        let mut d = Design {
            anchors: vec![
                (Vec3::new(-3.0, 0.0, 0.0), SupportKind::Pinned),
                (Vec3::new(3.0, 0.0, 0.0), SupportKind::Pinned),
            ],
            ..Default::default()
        };
        d.add_locked_node(Vec3::new(0.0, 0.0, 0.0), None);
        d
    }

    #[test]
    fn first_step_points_at_the_foundations() {
        let d = bridge();
        let t = BuildTool::default();
        assert!(guide_text(&t, &d).contains("foundation"));
        let t = BuildTool {
            aim: Aim::Anchor(0),
            ..Default::default()
        };
        let (head, detail, fail) = cursor_hint(&t, &d).unwrap();
        assert!(head.contains("Foundation"));
        assert!(detail.contains("pinned"));
        assert!(!fail);
    }

    #[test]
    fn hints_give_the_beam_length_and_flag_bad_clicks() {
        let mut d = bridge();
        let a = d.add_node(Vec3::new(-3.0, 0.0, 0.0));
        let t = BuildTool {
            selected: Some(a),
            aim: Aim::Grid,
            cursor: Some(Vec3::new(-3.0, 2.0, 0.0)),
            ..Default::default()
        };
        assert!(cursor_hint(&t, &d).unwrap().0.contains("2.0 m"));
        let t = BuildTool { outside: true, ..t };
        assert!(cursor_hint(&t, &d).unwrap().2);

        // Supports only go on foundations when the sheet has them.
        let free = d.add_node(Vec3::new(0.0, 1.0, 0.0));
        let t = BuildTool {
            tool: Tool::Support,
            aim: Aim::Node(free),
            ..Default::default()
        };
        assert!(cursor_hint(&t, &d).unwrap().2);
        assert!(support_allowed(&d, a));
        assert!(!support_allowed(&d, free));
        assert!(!support_allowed(&d, 0), "payload nodes keep theirs");
    }

    #[test]
    fn guide_follows_the_chain_and_selection() {
        let mut d = bridge();
        let a = d.add_node(Vec3::new(-3.0, 0.0, 0.0));
        let b = d.add_beam(a, 0, BeamKind::default()).unwrap();
        let t = BuildTool {
            selected: Some(0),
            ..Default::default()
        };
        assert!(guide_text(&t, &d).contains("Right-click to stop"));
        let t = BuildTool {
            selected_beam: Some(b),
            ..Default::default()
        };
        assert!(guide_text(&t, &d).contains("Member 1 selected"));
    }
}
