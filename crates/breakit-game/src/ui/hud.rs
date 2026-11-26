//! In-game HUDs: build mode (sheet, read-outs, tools, inspector, parts
//! drawer, run button, world tags) and the test run (sheet with live load
//! cases, event log, slow-motion banner, chart recorder).

use bevy::ecs::hierarchy::ChildSpawnerCommands;
use bevy::prelude::*;

use super::pointer::BlocksPointer;
use super::screens::{kn, mm, pct};
use super::theme::{ramp_band, Ink, Skin, Swatch, TextRole};
use super::widgets::*;
use super::{GameView, View, ViewKey};
use crate::analysis::FrameState;
use crate::build::Tool;
use crate::camera::OrbitCamera;
use crate::design::{catalog, joint_class, Design};
use crate::flow::{Screen, UiAction};
use crate::scenario::{money, Sheets, TestEvent, TestRecord, CHART_COLUMNS};
use breakit_parts::JointSpec;

fn view(kind: View, node: Node) -> impl Bundle {
    (node, kind, ViewKey::default())
}

/// Distance of the HUD from the screen edges.
const EDGE: f32 = 40.0;

pub fn spawn_build_hud(mut commands: Commands) {
    commands
        .spawn((layer(), DespawnOnExit(Screen::Build)))
        .with_children(|l| {
            l.spawn(view(View::BuildTags, layer()));
            l.spawn(view(View::BuildFooter, layer()));
            l.spawn(view(
                View::BuildSheet,
                at(Some(EDGE), Some(32.0), None, None),
            ));
            l.spawn(view(
                View::BuildReadouts,
                at(None, Some(32.0), Some(EDGE), None),
            ));
            l.spawn(view(
                View::BuildInspector,
                at(None, Some(232.0), Some(EDGE), None),
            ));
            l.spawn(view(
                View::BuildTools,
                Node {
                    flex_direction: FlexDirection::Row,
                    justify_content: JustifyContent::Center,
                    ..at(Some(0.0), None, Some(0.0), Some(30.0))
                },
            ));
            l.spawn(view(
                View::BuildDrawer,
                at(Some(EDGE), None, None, Some(32.0)),
            ));
        });
}

pub fn spawn_test_hud(mut commands: Commands) {
    commands
        .spawn((layer(), DespawnOnExit(Screen::Test)))
        .with_children(|l| {
            l.spawn(view(View::TestTags, layer()));
            l.spawn(view(
                View::TestSheet,
                at(Some(EDGE), Some(32.0), None, None),
            ));
            l.spawn(view(View::TestLog, at(None, Some(32.0), Some(EDGE), None)));
            l.spawn(view(
                View::TestBanner,
                Node {
                    flex_direction: FlexDirection::Column,
                    align_items: AlignItems::Center,
                    ..at(Some(0.0), Some(36.0), Some(0.0), None)
                },
            ));
            // Test clock, bottom left.
            l.spawn(Node {
                align_items: AlignItems::Baseline,
                column_gap: Val::Px(10.0),
                flex_direction: FlexDirection::Row,
                ..at(Some(EDGE), None, None, Some(96.0))
            })
            .with_children(|r| {
                r.spawn((
                    text("00.00", TextRole::MonoLarge, Swatch::InstrumentInk),
                    TimeText,
                ));
                r.spawn((
                    text("", TextRole::Heading, Swatch::InstrumentMuted),
                    TotalText,
                ));
            });
            l.spawn(view(
                View::TestControls,
                Node {
                    flex_direction: FlexDirection::Row,
                    align_items: AlignItems::Center,
                    column_gap: Val::Px(20.0),
                    ..at(None, None, Some(EDGE), Some(104.0))
                },
            ));
            // The timeline: load-case bands over the worst-member trace.
            l.spawn(instrument(Node {
                height: Val::Px(52.0),
                flex_direction: FlexDirection::Column,
                ..at(Some(EDGE), None, Some(EDGE), Some(32.0))
            }))
            .with_children(|ins| {
                ins.spawn((
                    Node {
                        flex_grow: 1.0,
                        flex_direction: FlexDirection::Column,
                        ..default()
                    },
                    Skin::fill(Swatch::Chart).square(),
                    ChartArea,
                ));
            });
        });
}

/// The paper strip of the chart recorder; filled by `update_test_readouts`
/// once the sheet is known.
#[derive(Component)]
pub struct ChartArea;
#[derive(Component)]
pub struct ChartBar(usize);
#[derive(Component)]
pub struct Playhead;
#[derive(Component)]
pub struct TimeText;
#[derive(Component)]
pub struct TotalText;

/// A label pinned to a point of the structure.
#[derive(Component, Clone, Copy, Debug)]
pub enum WorldTag {
    Beam(usize),
}

// ---------------------------------------------------------------- build

fn static_sag(v: &GameView) -> Option<f32> {
    let s = v.sheets.current();
    if v.state.displacements.is_empty() {
        return None;
    }
    s.sag_nodes(&v.design)
        .into_iter()
        .filter_map(|i| v.state.displacements.get(i))
        .map(|d| -d.y)
        .reduce(f32::max)
}

fn worst(v: &GameView) -> Option<(usize, f32)> {
    v.state
        .members
        .iter()
        .enumerate()
        .map(|(i, m)| (i, m.utilization))
        .max_by(|a, b| a.1.total_cmp(&b.1))
}

pub fn build_sheet_key(v: &GameView) -> String {
    let s = v.sheets.current();
    format!(
        "{} {} {:?} {:.0} {} {} {:?}",
        v.sheets.current,
        s.is_connected(&v.design),
        static_sag(v).map(|x| (x * 1000.0).round()),
        v.cost() / 10.0,
        v.revisions.of(s.id).len(),
        v.state.error.is_some(),
        v.check.violations
    )
}

pub fn build_sheet(p: &mut ChildSpawnerCommands, v: &GameView) {
    let s = v.sheets.current();
    let rev = v.revisions.next_letter(s.id);
    p.spawn(panel(Node {
        width: Val::Px(380.0),
        flex_direction: FlexDirection::Column,
        ..default()
    }))
    .with_children(|b| {
        sheet_head(b, &format!("{}  rev {rev}", s.id), &s.title);
        if s.sandbox {
            kv(
                b,
                "Nodes / beams",
                text(
                    format!("{} / {}", v.design.nodes.len(), v.design.beams.len()),
                    TextRole::Mono,
                    Swatch::Ink,
                ),
            );
            kv(
                b,
                "Length",
                text(
                    format!("{:.1} m", v.design.total_length()),
                    TextRole::Mono,
                    Swatch::Ink,
                ),
            );
            let sc = match &v.state.error {
                Some(e) => e.clone(),
                None => format!("sag {}", mm(v.state.max_displacement)),
            };
            kv(b, "Static check", text(sc, TextRole::Mono, Swatch::Ink));
            return;
        }
        // What blocks the test, in the scenario's own words.
        if v.check.violations.is_empty() {
            check_row(b, Status::Ok, "Meets the sheet's limits", "ready");
        }
        for vio in v.check.violations.iter().take(4) {
            check_row(b, Status::Failed, &vio.to_string(), "");
        }
        if v.check.violations.len() > 4 {
            check_row(
                b,
                Status::Failed,
                &format!("and {} more", v.check.violations.len() - 4),
                "",
            );
        }
        check_row(
            b,
            Status::Pending,
            &format!("Survive {:.1} s of {}", s.def.duration, s.load_summary()),
            "on test",
        );
        if let Some(budget) = s.budget {
            let c = v.cost();
            check_row(
                b,
                if c <= budget {
                    Status::Ok
                } else {
                    Status::Failed
                },
                &format!("Cost under {}", money(budget)),
                &money(c),
            );
        }
        if let Some(max) = s.def.limits.max_mass {
            let m = v.mass();
            check_row(
                b,
                if m as f64 <= max {
                    Status::Ok
                } else {
                    Status::Failed
                },
                &format!("Mass under {max:.0} kg"),
                &format!("{m:.0} kg"),
            );
        }
        if let Some(l) = s.sag_limit {
            let (st, val) = match (static_sag(v), &v.state.error) {
                (_, Some(_)) => (Status::Failed, "unstable".to_string()),
                (Some(x), None) => (
                    if x <= l { Status::Ok } else { Status::Pending },
                    format!("{} static", mm(x)),
                ),
                (None, None) => (Status::Pending, "-".into()),
            };
            check_row(b, st, &format!("Deck sag under {}", mm(l)), &val);
        }
        b.spawn(Node {
            column_gap: Val::Px(10.0),
            margin: UiRect::top(Val::Px(8.0)),
            ..default()
        })
        .with_children(|f| {
            f.spawn(caps("Brief", Swatch::Muted));
            f.spawn(caps("F1", Swatch::Accent));
        });
    });
}

pub fn readouts_key(v: &GameView) -> String {
    let w = worst(v).map(|(i, u)| (i, (u * 100.0).round() as i32));
    format!(
        "{} {:?} {:.0} {:?} {:?}",
        (v.cost() / 10.0).round(),
        w,
        v.mass(),
        v.state.error,
        v.sheets.current().budget
    )
}

pub fn build_readouts(p: &mut ChildSpawnerCommands, v: &GameView) {
    let s = v.sheets.current();
    let cost = v.cost();
    p.spawn(panel(Node {
        width: Val::Px(280.0),
        flex_direction: FlexDirection::Column,
        align_items: AlignItems::Stretch,
        ..default()
    }))
    .with_children(|b| {
        b.spawn(Node {
            justify_content: JustifyContent::FlexEnd,
            ..default()
        })
        .with_child(caps("Cost", Swatch::Muted));
        b.spawn(Node {
            justify_content: JustifyContent::FlexEnd,
            align_items: AlignItems::Baseline,
            column_gap: Val::Px(8.0),
            ..default()
        })
        .with_children(|r| {
            let over = s.budget.is_some_and(|b| cost > b);
            r.spawn(text(
                money_number(cost),
                TextRole::Stamp,
                if over { Swatch::Fail } else { Swatch::Ink },
            ));
            if let Some(b) = s.budget {
                r.spawn(text(
                    format!("/ {}", money(b)),
                    TextRole::Body,
                    Swatch::Muted,
                ));
            }
        });
        if let Some(budget) = s.budget {
            let fill = if cost > budget {
                Swatch::Fail
            } else {
                Swatch::Ink
            };
            meter(b, cost / budget, fill, s.bonus_budget.map(|x| x / budget));
            if let Some(bonus) = s.bonus_budget {
                b.spawn(Node {
                    justify_content: JustifyContent::FlexEnd,
                    margin: UiRect::bottom(Val::Px(10.0)),
                    ..default()
                })
                .with_child(caps(
                    &format!("Full marks under {}", money(bonus)),
                    Swatch::Muted,
                ));
            }
        }
        kv(
            b,
            "Mass",
            text(format!("{:.0} kg", v.mass()), TextRole::Body, Swatch::Ink),
        );
        match worst(v) {
            Some((i, u)) => kv(
                b,
                "Worst member",
                tag_bundle(format!("M{} {}", i + 1, pct(u)), u),
            ),
            None => kv(b, "Worst member", text("-", TextRole::Body, Swatch::Muted)),
        }
        let check = match &v.state.error {
            Some(_) => text("unstable: add supports", TextRole::Body, Swatch::Fail),
            None if v.design.beams.is_empty() => {
                text("nothing built", TextRole::Body, Swatch::Muted)
            }
            None => text("solved", TextRole::Body, Swatch::Ink),
        };
        kv(b, "Static check", check);
    });
}

/// A cost without its unit, for the large read-out.
fn money_number(x: f32) -> String {
    money(x).trim_end_matches(" kr").to_string()
}

/// A utilisation read-out: a band-coloured dot and the number.
fn tag_bundle(s: String, u: f32) -> impl Bundle {
    let band = ramp_band(u);
    (
        Node {
            align_items: AlignItems::Center,
            column_gap: Val::Px(8.0),
            ..default()
        },
        children![
            (
                Node {
                    width: Val::Px(8.0),
                    height: Val::Px(8.0),
                    ..default()
                },
                Skin::fill(Swatch::Ramp(band)).pill(),
            ),
            text(s, TextRole::Body, Swatch::Ink),
        ],
    )
}

/// A tag pinned to a member in the scene: dot and number, no box.
fn member_tag(s: String, u: f32) -> impl Bundle {
    tag_bundle(s, u)
}

/// One text tool in the bottom bar: label, key under it, an accent
/// underline when active.
fn tool_item(
    p: &mut ChildSpawnerCommands,
    label: &str,
    key: &str,
    action: UiAction,
    on: bool,
    enabled: bool,
) {
    let mut e = p.spawn((
        Node {
            flex_direction: FlexDirection::Column,
            align_items: AlignItems::Center,
            padding: UiRect::new(Val::Px(14.0), Val::Px(14.0), Val::Px(8.0), Val::Px(6.0)),
            row_gap: Val::Px(3.0),
            ..default()
        },
        Skin::fill(Swatch::Clear),
        super::pointer::Clickable(action),
        BlocksPointer,
        Outline::new(Val::Px(1.0), Val::Px(0.0), Color::NONE),
    ));
    if !enabled {
        e.insert(super::pointer::Disabled);
    }
    e.with_children(|t| {
        let fg = match (on, enabled) {
            (true, _) => Swatch::Ink,
            (false, true) => Swatch::Muted,
            (false, false) => Swatch::Rule,
        };
        t.spawn(text(label, TextRole::Body, fg));
        t.spawn(caps(
            key,
            if enabled { Swatch::Muted } else { Swatch::Rule },
        ));
        t.spawn((
            Node {
                width: Val::Percent(100.0),
                height: Val::Px(2.0),
                margin: UiRect::top(Val::Px(4.0)),
                ..default()
            },
            Skin::fill(if on { Swatch::Accent } else { Swatch::Clear }).square(),
        ));
    });
}

fn bar_separator(p: &mut ChildSpawnerCommands) {
    p.spawn((
        Node {
            width: Val::Px(1.0),
            height: Val::Px(28.0),
            margin: UiRect::axes(Val::Px(10.0), Val::Px(0.0)),
            ..default()
        },
        Skin::fill(Swatch::Rule).square(),
    ));
}

pub fn build_tools(p: &mut ChildSpawnerCommands, v: &GameView) {
    p.spawn(row(2.0)).with_children(|r| {
        let mut tools = vec![Tool::Draw, Tool::Support, Tool::Delete];
        if v.tool.loads_allowed {
            tools.push(Tool::Load);
        }
        for t in tools {
            tool_item(
                r,
                t.label(),
                t.key(),
                UiAction::SetTool(t),
                v.tool.tool == t,
                true,
            );
        }
        bar_separator(r);
        tool_item(
            r,
            "Undo",
            "Ctrl Z",
            UiAction::Undo,
            false,
            v.history.can_undo(),
        );
        tool_item(
            r,
            "Redo",
            "Ctrl Y",
            UiAction::Redo,
            false,
            v.history.can_redo(),
        );
        bar_separator(r);
        tool_item(
            r,
            &format!("Plane {}", v.tool.plane.label()),
            "Tab",
            UiAction::CyclePlane,
            false,
            true,
        );
    });
}

pub fn inspector_key(v: &GameView) -> String {
    let b = v.tool.selected_beam;
    let m = b.and_then(|b| v.state.members.get(b)).map(|m| {
        (
            (m.utilization * 100.0).round(),
            m.axial.map(|a| (a / 100.0).round()),
            m.joints.map(|j| (j * 100.0).round()),
        )
    });
    format!("{b:?} {} {m:?}", v.design.revision)
}

/// A label / value row the player can click, with its key hint. Quiet text
/// on the scene; the hover outline is the only frame.
fn kv_action(
    p: &mut ChildSpawnerCommands,
    label: &str,
    value: impl FnOnce(&mut ChildSpawnerCommands),
    key: &str,
    action: UiAction,
) {
    p.spawn((
        Node {
            justify_content: JustifyContent::SpaceBetween,
            align_items: AlignItems::Center,
            padding: UiRect::vertical(Val::Px(3.0)),
            column_gap: Val::Px(12.0),
            ..default()
        },
        super::pointer::Clickable(action),
        BlocksPointer,
        Outline::new(Val::Px(1.0), Val::Px(2.0), Color::NONE),
    ))
    .with_children(|r| {
        r.spawn(text(label, TextRole::Body, Swatch::Muted));
        r.spawn(row(10.0)).with_children(|v| {
            value(v);
            v.spawn(caps(key, Swatch::Muted));
        });
    });
}

/// Clickable arrow for stepping through sizes.
fn step_arrow(p: &mut ChildSpawnerCommands, glyph: &str, action: UiAction, enabled: bool) {
    let mut e = p.spawn((
        Node {
            padding: UiRect::horizontal(Val::Px(4.0)),
            ..default()
        },
        text(
            glyph,
            TextRole::Body,
            if enabled { Swatch::Ink } else { Swatch::Rule },
        ),
        BlocksPointer,
    ));
    if enabled {
        e.insert((
            super::pointer::Clickable(action),
            Outline::new(Val::Px(1.0), Val::Px(2.0), Color::NONE),
        ));
    }
}

/// The value and utilisation of one end joint.
fn joint_value(p: &mut ChildSpawnerCommands, j: JointSpec, auto: bool, util: f32) {
    let name = if auto {
        format!("{} (auto)", j.label())
    } else {
        j.label().to_string()
    };
    p.spawn(text(name, TextRole::Body, Swatch::Ink));
    if matches!(j, JointSpec::Monolithic | JointSpec::Rigid) {
        // Full-strength joints have no capacity of their own.
        p.spawn(text("full", TextRole::Label, Swatch::Muted));
    } else {
        p.spawn(tag_bundle(pct(util), util));
    }
}

pub fn build_inspector(p: &mut ChildSpawnerCommands, v: &GameView) {
    let Some(i) = v.tool.selected_beam.filter(|&i| i < v.design.beams.len()) else {
        return;
    };
    let beam = v.design.beams[i];
    let kind = beam.kind;
    let joints = v.design.joints(i);
    let len = v.design.beam_length(i);
    let m = v.state.members.get(i).copied().unwrap_or_default();
    let family = catalog().family_of(&kind.0);
    let costs = &v.sheets.costs;
    let joint_cost: f32 = joints
        .iter()
        .map(|j| costs.joints.price(joint_class(*j)) as f32)
        .sum();
    p.spawn(Node {
        width: Val::Px(280.0),
        flex_direction: FlexDirection::Column,
        ..default()
    })
    .with_children(|b| {
        b.spawn(caps(&format!("Member {}", i + 1), Swatch::Accent));
        b.spawn((
            text(kind.designation(), TextRole::Heading, Swatch::Ink),
            Node {
                margin: UiRect::top(Val::Px(4.0)),
                ..default()
            },
        ));
        b.spawn((
            text(
                family
                    .map(|(f, ..)| f.name.as_str())
                    .unwrap_or(kind.label()),
                TextRole::Label,
                Swatch::Muted,
            ),
            Node {
                margin: UiRect::bottom(Val::Px(6.0)),
                ..default()
            },
        ));
        if let Some((f, size, grade)) = family {
            b.spawn(Node {
                justify_content: JustifyContent::SpaceBetween,
                align_items: AlignItems::Center,
                padding: UiRect::vertical(Val::Px(3.0)),
                ..default()
            })
            .with_children(|r| {
                r.spawn(text("Size", TextRole::Body, Swatch::Muted));
                r.spawn(row(6.0)).with_children(|s| {
                    step_arrow(s, "<", UiAction::StepSize(-1), size > 0);
                    s.spawn(text(&f.sizes[size].name, TextRole::Body, Swatch::Ink));
                    step_arrow(s, ">", UiAction::StepSize(1), size + 1 < f.sizes.len());
                    s.spawn(caps("[ ]", Swatch::Muted));
                });
            });
            let g = kind.0.material.grade_label();
            if f.grades.len() > 1 {
                kv_action(
                    b,
                    "Grade",
                    |v| {
                        v.spawn(text(g, TextRole::Body, Swatch::Ink));
                    },
                    "G",
                    UiAction::CycleGrade,
                );
            } else {
                let _ = grade;
                kv(b, "Grade", text(g, TextRole::Body, Swatch::Ink));
            }
        }
        for (end, name) in ["End A", "End B"].into_iter().enumerate() {
            kv_action(
                b,
                name,
                |v| joint_value(v, joints[end], beam.joints[end].is_none(), m.joints[end]),
                "",
                UiAction::CycleJoint(end as u8),
            );
        }
        b.spawn(Node {
            height: Val::Px(6.0),
            ..default()
        });
        kv(
            b,
            "Length",
            text(format!("{len:.2} m"), TextRole::Body, Swatch::Ink),
        );
        if let Some(a) = m.axial {
            let sense = if a >= 0.0 { "tension" } else { "compression" };
            kv(
                b,
                "Axial",
                text(
                    format!("{} {sense}", kn(a.abs())),
                    TextRole::Body,
                    Swatch::Ink,
                ),
            );
        }
        kv(b, "Section", tag_bundle(pct(m.utilization), m.utilization));
        kv(
            b,
            "Cost / mass",
            text(
                format!(
                    "{} / {:.0} kg",
                    money(len * kind.cost_per_metre(costs) + joint_cost),
                    len * kind.mass_per_metre()
                ),
                TextRole::Body,
                Swatch::Ink,
            ),
        );
        b.spawn(Node {
            height: Val::Px(8.0),
            ..default()
        });
        kv_action(b, "Delete member", |_| {}, "Del", UiAction::DeleteSelected);
    });
}

/// Palette tab names for the material classes.
fn class_label(k: breakit_core::MaterialKind) -> &'static str {
    use breakit_core::MaterialKind::*;
    match k {
        Steel => "Steel",
        Timber => "Timber",
        Concrete => "Concrete",
        Composite => "Carbon",
    }
}

pub fn drawer_key(v: &GameView) -> String {
    format!(
        "{:?} {} {:?} {}",
        v.tool.kind,
        v.tool.allowed.len(),
        v.tool.selected_beam,
        v.design.revision
    )
}

pub fn build_drawer(p: &mut ChildSpawnerCommands, v: &GameView) {
    // With a member selected, the palette shows and swaps that member's part.
    let current = v.tool.current(&v.design);
    let class = current.material_kind();
    let sel_beam = v.tool.selected_beam.filter(|&i| i < v.design.beams.len());
    p.spawn(column(10.0)).with_children(|d| {
        d.spawn(Node {
            margin: UiRect::bottom(Val::Px(4.0)),
            ..default()
        })
        .with_children(|c| ramp_legend(c, 120.0));
        // Material tabs, keys 1-4.
        d.spawn(row(18.0)).with_children(|r| {
            for (n, c) in v.tool.classes().into_iter().enumerate() {
                let active = c == class;
                r.spawn((
                    column(3.0),
                    super::pointer::Clickable(UiAction::MaterialTab(n as u8)),
                    BlocksPointer,
                    Outline::new(Val::Px(1.0), Val::Px(2.0), Color::NONE),
                ))
                .with_children(|t| {
                    t.spawn(row(6.0)).with_children(|l| {
                        l.spawn(caps(
                            class_label(c),
                            if active { Swatch::Ink } else { Swatch::Muted },
                        ));
                        l.spawn(caps(&(n + 1).to_string(), Swatch::Muted));
                    });
                    t.spawn((
                        Node {
                            height: Val::Px(2.0),
                            ..default()
                        },
                        Skin::fill(if active {
                            Swatch::Accent
                        } else {
                            Swatch::Clear
                        })
                        .square(),
                    ));
                });
            }
        });
        // The parts of the active material, two columns.
        d.spawn(Node {
            flex_direction: FlexDirection::Row,
            flex_wrap: FlexWrap::Wrap,
            width: Val::Px(360.0),
            row_gap: Val::Px(8.0),
            ..default()
        })
        .with_children(|g| {
            for k in v.tool.allowed.iter().filter(|k| k.material_kind() == class) {
                let selected = k.same_section(current);
                g.spawn((
                    Node {
                        width: Val::Px(180.0),
                        flex_direction: FlexDirection::Row,
                        column_gap: Val::Px(8.0),
                        ..default()
                    },
                    super::pointer::Clickable(UiAction::SetMaterial(*k)),
                    BlocksPointer,
                    Outline::new(Val::Px(1.0), Val::Px(2.0), Color::NONE),
                ))
                .with_children(|card| {
                    card.spawn((
                        Node {
                            width: Val::Px(2.0),
                            ..default()
                        },
                        Skin::fill(if selected {
                            Swatch::Accent
                        } else {
                            Swatch::Clear
                        })
                        .square(),
                    ));
                    card.spawn(column(2.0)).with_children(|c| {
                        c.spawn(text(
                            k.label(),
                            TextRole::Body,
                            if selected { Swatch::Ink } else { Swatch::Muted },
                        ));
                        c.spawn(text(
                            format!(
                                "{}  {:.0} kr/m",
                                k.designation(),
                                k.cost_per_metre(&v.sheets.costs)
                            ),
                            TextRole::Label,
                            Swatch::Muted,
                        ));
                    });
                });
            }
        });
        let note = match sel_beam {
            Some(i) => format!("{}  applies to member {}", current.designation(), i + 1),
            None => format!("next beam  {}", current.designation()),
        };
        d.spawn(text(note, TextRole::Label, Swatch::Muted));
    });
}

pub fn build_footer(p: &mut ChildSpawnerCommands, v: &GameView) {
    // Tool hint and plane read-out, centred above the tool bar.
    p.spawn(Node {
        align_items: AlignItems::Center,
        row_gap: Val::Px(4.0),
        ..at(Some(0.0), None, Some(0.0), Some(104.0))
    })
    .with_children(|c| {
        c.spawn(text(v.tool.tool.hint(), TextRole::Body, Swatch::SceneText));
        let defl = if v.scale.0 > 0.0 {
            format!("   deflection x{:.0}", v.scale.0)
        } else {
            String::new()
        };
        c.spawn(text(
            format!(
                "plane {} at {} {:+.1} m   snap {:.1} m{defl}",
                v.tool.plane.label(),
                v.tool.plane.axis(),
                v.tool.level,
                crate::build::GRID
            ),
            TextRole::Label,
            Swatch::Muted,
        ));
    });
    // Run button, bottom right.
    p.spawn(Node {
        align_items: AlignItems::FlexEnd,
        row_gap: Val::Px(10.0),
        ..at(None, None, Some(EDGE), Some(32.0))
    })
    .with_children(|c| {
        let s = v.sheets.current();
        let last = v.revisions.of(s.id).last().map(|r| {
            let what = if r.passed {
                "passed".to_string()
            } else {
                format!("held {:.1} s", r.held)
            };
            format!("Rev {} {what}", r.letter)
        });
        c.spawn(text(
            match last {
                Some(l) => format!(
                    "{l}   /   {} load cases, {:.1} s",
                    s.load_cases.len(),
                    s.total_duration()
                ),
                None => format!(
                    "{} load cases, {:.1} s",
                    s.load_cases.len(),
                    s.total_duration()
                ),
            },
            TextRole::Label,
            Swatch::Muted,
        ));
        let e = button(
            c,
            "Run test",
            Some("Space"),
            UiAction::RunTest,
            ButtonKind::Primary,
        );
        c.commands().entity(e).insert(Node {
            padding: UiRect::axes(Val::Px(24.0), Val::Px(12.0)),
            column_gap: Val::Px(14.0),
            align_items: AlignItems::Center,
            ..default()
        });
    });
}

pub fn build_tags_key(v: &GameView) -> String {
    format!(
        "{:?} {:?} {:?}",
        v.state.revision,
        top_members(&v.state),
        v.pins.pins
    )
}

/// The three most used members over half capacity.
fn top_members(state: &FrameState) -> Vec<(usize, i32)> {
    let mut m: Vec<(usize, f32)> = state
        .members
        .iter()
        .enumerate()
        .filter(|(_, m)| m.utilization >= 0.5)
        .map(|(i, m)| (i, m.utilization))
        .collect();
    m.sort_by(|a, b| b.1.total_cmp(&a.1));
    m.truncate(3);
    m.into_iter()
        .map(|(i, u)| (i, (u * 100.0).round() as i32))
        .collect()
}

pub fn build_tags(p: &mut ChildSpawnerCommands, v: &GameView) {
    for (i, u) in top_members(&v.state) {
        p.spawn((
            Node {
                position_type: PositionType::Absolute,
                ..default()
            },
            WorldTag::Beam(i),
            children![member_tag(format!("{}  {}%", i + 1, u), u as f32 / 100.0)],
        ));
    }
    for (i, label) in &v.pins.pins {
        p.spawn((
            Node {
                position_type: PositionType::Absolute,
                padding: UiRect::axes(Val::Px(6.0), Val::Px(1.0)),
                ..default()
            },
            WorldTag::Beam(*i),
            Children::spawn_one(text(label.clone(), TextRole::Body, Swatch::Fail)),
        ));
    }
}

// ---------------------------------------------------------------- test

fn current_case(rec: &TestRecord) -> Option<usize> {
    rec.events.iter().rev().find_map(|e| match e.event {
        TestEvent::CaseStarted { case } => Some(case),
        _ => None,
    })
}

pub fn test_sheet_key(v: &GameView) -> String {
    format!(
        "{:?} {:?} {} {:.0}",
        current_case(&v.record),
        v.record.first_break,
        v.record.events.len(),
        v.record.peak_sag * 1000.0
    )
}

pub fn test_sheet(p: &mut ChildSpawnerCommands, v: &GameView) {
    let s = v.sheets.current();
    let rec = &v.record;
    let cur = current_case(rec);
    let starts = s.case_starts();
    let broke_in = rec.first_break.and_then(|t| s.case_at(t));
    p.spawn(panel(Node {
        width: Val::Px(380.0),
        flex_direction: FlexDirection::Column,
        ..default()
    }))
    .with_children(|b| {
        let rev = v.revisions.next_letter(s.id);
        sheet_head(b, &format!("{} test  rev {rev}", s.id), &s.title);
        for (i, c) in s.load_cases.iter().enumerate() {
            let (st, val) = match (broke_in, cur) {
                (Some(bc), _) if bc == i => (
                    Status::Failed,
                    format!("broke {:.2} s", rec.first_break.unwrap()),
                ),
                (Some(bc), _) if i > bc => (Status::Pending, "-".into()),
                (_, Some(cc)) if i < cc => (Status::Ok, "held".into()),
                (_, Some(cc)) if i == cc => (Status::Running, format!("from {:.0} s", starts[i])),
                _ => (Status::Pending, format!("at {:.0} s", starts[i])),
            };
            check_row(b, st, &c.name, &val);
        }
        if let Some(l) = s.sag_limit {
            let st = if rec.peak_sag > l {
                Status::Failed
            } else {
                Status::Ok
            };
            check_row(
                b,
                st,
                &format!("Deck sag under {}", mm(l)),
                &format!("peak {}", mm(rec.peak_sag)),
            );
        }
    });
}

pub fn test_log(p: &mut ChildSpawnerCommands, v: &GameView) {
    let s = v.sheets.current();
    p.spawn(panel(Node {
        width: Val::Px(320.0),
        flex_direction: FlexDirection::Column,
        row_gap: Val::Px(6.0),
        ..default()
    }))
    .with_children(|b| {
        let events: Vec<_> = v.record.events.iter().rev().take(8).collect();
        for e in events.into_iter().rev() {
            let (what, fail) = match &e.event {
                TestEvent::CaseStarted { case } => (s.load_cases[*case].name.clone(), false),
                TestEvent::FirstYield { member, state } => (
                    format!("Member {} first damage: {}", member + 1, state),
                    false,
                ),
                TestEvent::Broke { member, cause } => {
                    (format!("Member {} {}", member + 1, cause), true)
                }
                TestEvent::Collapse { reason } => (format!("Collapse: {reason}"), true),
                TestEvent::Passed => ("Test passed".to_string(), false),
                TestEvent::ProofHeld { factor } => (format!("Proof test held x{factor:.1}"), false),
            };
            b.spawn(Node {
                column_gap: Val::Px(14.0),
                ..default()
            })
            .with_children(|r| {
                r.spawn((
                    text(format!("{:.2}", e.time), TextRole::Body, Swatch::Muted),
                    Node {
                        width: Val::Px(44.0),
                        flex_shrink: 0.0,
                        ..default()
                    },
                ));
                r.spawn(text(
                    what,
                    TextRole::Body,
                    if fail { Swatch::Fail } else { Swatch::Ink },
                ));
            });
        }
    });
}

pub fn test_banner(p: &mut ChildSpawnerCommands, v: &GameView) {
    let msg = if v.slowmo.active() {
        Some(("Slow motion  /  first break", "press . to resume speed"))
    } else if v.clock.paused {
        Some(("Paused", "Space to resume"))
    } else {
        None
    };
    let Some((head, tail)) = msg else { return };
    p.spawn(caps(head, Swatch::Accent));
    p.spawn((
        text(tail, TextRole::Label, Swatch::Muted),
        Node {
            margin: UiRect::top(Val::Px(4.0)),
            ..default()
        },
    ));
}

/// A text control in the test bar: label and key, accent when on.
fn text_control(p: &mut ChildSpawnerCommands, label: &str, key: &str, action: UiAction, on: bool) {
    p.spawn((
        Node {
            column_gap: Val::Px(6.0),
            align_items: AlignItems::Baseline,
            padding: UiRect::axes(Val::Px(2.0), Val::Px(4.0)),
            ..default()
        },
        Skin::fill(Swatch::Clear),
        super::pointer::Clickable(action),
        BlocksPointer,
        Outline::new(Val::Px(1.0), Val::Px(2.0), Color::NONE),
    ))
    .with_children(|k| {
        k.spawn(text(
            label,
            TextRole::Body,
            if on { Swatch::Accent } else { Swatch::Ink },
        ));
        if !key.is_empty() {
            k.spawn(caps(key, Swatch::Muted));
        }
    });
}

pub fn test_controls(p: &mut ChildSpawnerCommands, v: &GameView) {
    let pause = if v.clock.paused { "Run" } else { "Pause" };
    text_control(p, pause, "Space", UiAction::SimPause, false);
    let s = v.clock.speed;
    let speed = if s < 1.0 {
        format!("1/{:.0}x", 1.0 / s)
    } else {
        format!("{s:.0}x")
    };
    p.spawn(text(speed, TextRole::Body, Swatch::Muted));
    text_control(p, "Slower", ",", UiAction::SimSlower, false);
    text_control(p, "Faster", ".", UiAction::SimFaster, false);
    text_control(p, "Stress", "H", UiAction::ToggleHeatmap, v.heat.0);
    text_control(p, "Abort", "Esc", UiAction::AbortTest, false);
}

pub fn test_tags(p: &mut ChildSpawnerCommands, v: &GameView) {
    let Some(t) = v.record.first_break else {
        return;
    };
    let Some((m, cause)) = v.record.events.iter().find_map(|e| match &e.event {
        TestEvent::Broke { member, cause } => Some((*member, cause.clone())),
        _ => None,
    }) else {
        return;
    };
    let kind = v
        .design
        .beams
        .get(m)
        .map(|b| b.kind.label())
        .unwrap_or("member");
    p.spawn((
        Node {
            position_type: PositionType::Absolute,
            flex_direction: FlexDirection::Column,
            row_gap: Val::Px(2.0),
            width: Val::Px(260.0),
            ..default()
        },
        WorldTag::Beam(m),
    ))
    .with_children(|c| {
        c.spawn(text(
            format!("Member {} broke", m + 1),
            TextRole::Heading,
            Swatch::Fail,
        ));
        c.spawn(text(
            format!("{kind}, {cause}, at {t:.2} s"),
            TextRole::Body,
            Swatch::Ink,
        ));
    });
}

/// Test time, chart bars and playhead, every frame.
#[allow(clippy::too_many_arguments)]
pub fn update_test_readouts(
    mut commands: Commands,
    sheets: Res<Sheets>,
    rec: Res<TestRecord>,
    sim: Option<Res<crate::analysis::Simulation>>,
    area: Query<(Entity, Option<&Children>), With<ChartArea>>,
    mut bars: Query<(&ChartBar, &mut Node, &mut Skin), Without<Playhead>>,
    mut head: Query<&mut Node, (With<Playhead>, Without<ChartBar>)>,
    mut time_text: Query<&mut Text, (With<TimeText>, Without<TotalText>)>,
    mut total_text: Query<&mut Text, (With<TotalText>, Without<TimeText>)>,
) {
    let Ok((area, children)) = area.single() else {
        return;
    };
    let s = sheets.current();
    let total = s.total_duration().max(1e-3);
    for mut tx in &mut total_text {
        let v = format!("/ {total:.1} s");
        if tx.0 != v {
            tx.0 = v;
        }
    }
    if children.is_none_or(|c| c.is_empty()) {
        build_chart(&mut commands, area, s);
        return;
    }
    let t = sim.map(|s| s.time() as f32).unwrap_or(0.0);
    for mut tx in &mut time_text {
        let v = format!("{t:05.2}");
        if tx.0 != v {
            tx.0 = v;
        }
    }
    let now = ((t / total) * CHART_COLUMNS as f32) as usize;
    for (bar, mut node, mut skin) in &mut bars {
        let u = rec.chart.get(bar.0).copied().unwrap_or(0.0);
        let h = Val::Percent((u / 1.2).clamp(0.0, 1.0) * 100.0);
        if node.height != h {
            node.height = h;
        }
        let want = if bar.0 <= now {
            Swatch::Ramp(ramp_band(u))
        } else {
            Swatch::Clear
        };
        if skin.bg != want {
            skin.bg = want;
        }
    }
    for mut n in &mut head {
        n.left = Val::Percent((t / total).clamp(0.0, 1.0) * 100.0);
    }
}

fn build_chart(commands: &mut Commands, area: Entity, s: &crate::scenario::Scenario) {
    commands.entity(area).with_children(|a| {
        // Load-case labels at their place in time; gaps are self weight.
        let total = s.total_duration().max(1e-3);
        a.spawn(Node {
            height: Val::Px(16.0),
            flex_shrink: 0.0,
            ..default()
        })
        .with_children(|bands| {
            for c in &s.load_cases {
                bands
                    .spawn(Node {
                        position_type: PositionType::Absolute,
                        left: Val::Percent(c.start.max(0.0) / total * 100.0),
                        width: Val::Percent((c.end - c.start.max(0.0)).max(0.0) / total * 100.0),
                        height: Val::Percent(100.0),
                        column_gap: Val::Px(6.0),
                        overflow: Overflow::clip(),
                        ..default()
                    })
                    .with_children(|b| {
                        b.spawn((
                            Node {
                                width: Val::Px(1.0),
                                height: Val::Percent(100.0),
                                flex_shrink: 0.0,
                                ..default()
                            },
                            Skin::fill(Swatch::Muted).square(),
                        ));
                        b.spawn((
                            Text::new(super::widgets::plain(c.name.clone())),
                            Ink::new(TextRole::Label, Swatch::Muted),
                        ));
                    });
            }
        });
        // Bars, the 100 % line, the playhead.
        a.spawn((Node {
            flex_grow: 1.0,
            align_items: AlignItems::FlexEnd,
            column_gap: Val::Px(1.0),
            margin: UiRect::top(Val::Px(4.0)),
            ..default()
        },))
            .with_children(|plot| {
                plot.spawn((
                    Node {
                        position_type: PositionType::Absolute,
                        left: Val::Px(0.0),
                        right: Val::Px(0.0),
                        bottom: Val::Px(0.0),
                        height: Val::Px(1.0),
                        ..default()
                    },
                    Skin::fill(Swatch::Rule).square(),
                ));
                for i in 0..CHART_COLUMNS {
                    plot.spawn((
                        Node {
                            flex_grow: 1.0,
                            flex_basis: Val::Px(0.0),
                            height: Val::Percent(0.0),
                            ..default()
                        },
                        Skin::fill(Swatch::Clear).square(),
                        ChartBar(i),
                    ));
                }
                plot.spawn((
                    Node {
                        position_type: PositionType::Absolute,
                        left: Val::Px(0.0),
                        right: Val::Px(0.0),
                        bottom: Val::Percent(100.0 / 1.2),
                        height: Val::Px(1.0),
                        ..default()
                    },
                    Skin::fill(Swatch::Fail).square(),
                ));
                plot.spawn((
                    Node {
                        position_type: PositionType::Absolute,
                        top: Val::Px(-4.0),
                        bottom: Val::Px(-4.0),
                        width: Val::Px(1.0),
                        left: Val::Percent(0.0),
                        ..default()
                    },
                    Skin::fill(Swatch::Accent).square(),
                    Playhead,
                ));
            });
    });
}

/// Keeps world tags next to their member, on screen.
pub fn place_world_tags(
    camera: Query<(&Camera, &GlobalTransform), With<OrbitCamera>>,
    design: Res<Design>,
    state: Res<FrameState>,
    mut tags: Query<(&WorldTag, &mut Node, &mut Visibility)>,
) {
    let Ok((camera, tf)) = camera.single() else {
        return;
    };
    let pos = |n: usize| state.positions.get(n).or(design.nodes.get(n)).copied();
    for (tag, mut node, mut vis) in &mut tags {
        let world = match *tag {
            WorldTag::Beam(b) => design
                .beams
                .get(b)
                .and_then(|m| Some((pos(m.a)? + pos(m.b)?) * 0.5)),
        };
        match world.and_then(|w| camera.world_to_viewport(tf, w).ok()) {
            Some(px) => {
                node.left = Val::Px(px.x + 14.0);
                node.top = Val::Px(px.y - 30.0);
                *vis = Visibility::Inherited;
            }
            None => *vis = Visibility::Hidden,
        }
    }
}
