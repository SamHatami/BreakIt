//! In-game HUDs: build mode (sheet, read-outs, tools, inspector, parts
//! drawer, run button, world tags) and the test run (sheet with live load
//! cases, event log, slow-motion banner, chart recorder).

use bevy::ecs::hierarchy::ChildSpawnerCommands;
use bevy::prelude::*;

use super::screens::{kn, mm, pct};
use super::theme::{ramp_band, Ink, Skin, Swatch, TextRole, Weight};
use super::widgets::*;
use super::{GameView, View, ViewKey};
use crate::analysis::FrameState;
use crate::build::Tool;
use crate::camera::OrbitCamera;
use crate::design::{BeamKind, Design};
use crate::flow::{Screen, UiAction};
use crate::scenario::{money, Sheets, TestEvent, TestRecord, CHART_COLUMNS};

fn view(kind: View, node: Node) -> impl Bundle {
    (node, kind, ViewKey::default())
}

pub fn spawn_build_hud(mut commands: Commands) {
    commands
        .spawn((layer(), DespawnOnExit(Screen::Build)))
        .with_children(|l| {
            l.spawn(view(View::BuildTags, layer()));
            l.spawn(view(View::BuildFooter, layer()));
            l.spawn(view(
                View::BuildSheet,
                at(Some(20.0), Some(20.0), None, None),
            ));
            l.spawn(view(
                View::BuildReadouts,
                at(None, Some(20.0), Some(20.0), None),
            ));
            l.spawn(view(
                View::BuildTools,
                at(Some(20.0), Some(230.0), None, None),
            ));
            l.spawn(view(
                View::BuildInspector,
                at(None, Some(196.0), Some(20.0), None),
            ));
            l.spawn(view(
                View::BuildDrawer,
                at(Some(92.0), None, None, Some(20.0)),
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
                at(Some(20.0), Some(20.0), None, None),
            ));
            l.spawn(view(View::TestLog, at(None, Some(20.0), Some(20.0), None)));
            l.spawn(view(
                View::TestBanner,
                Node {
                    justify_content: JustifyContent::Center,
                    align_items: AlignItems::FlexStart,
                    // Between the sheet and the log, so it covers neither.
                    ..at(Some(370.0), Some(20.0), Some(300.0), None)
                },
            ));
            // The chart recorder: built once, its read-outs update in place.
            l.spawn(instrument(Node {
                height: Val::Px(160.0),
                flex_direction: FlexDirection::Row,
                ..at(Some(20.0), None, Some(20.0), Some(20.0))
            }))
            .with_children(|ins| {
                ins.spawn(view(
                    View::TestControls,
                    Node {
                        width: Val::Px(230.0),
                        flex_direction: FlexDirection::Column,
                        padding: UiRect::axes(Val::Px(12.0), Val::Px(8.0)),
                        row_gap: Val::Px(5.0),
                        ..default()
                    },
                ));
                ins.spawn((
                    Node {
                        flex_grow: 1.0,
                        flex_direction: FlexDirection::Column,
                        margin: UiRect::all(Val::Px(10.0)),
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

/// A label pinned to a point of the structure.
#[derive(Component, Clone, Copy, Debug)]
pub enum WorldTag {
    Beam(usize),
}

// ---------------------------------------------------------------- build

fn static_sag(v: &GameView) -> Option<f32> {
    let s = v.sheets.current();
    if v.state.displacements.is_empty() || s.load_points.is_empty() {
        return None;
    }
    (0..s.load_points.len())
        .filter_map(|i| v.state.displacements.get(s.load_point_node(i)))
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
        "{} {} {:?} {:.0} {} {}",
        v.sheets.current,
        s.is_connected(&v.design),
        static_sag(v).map(|x| (x * 1000.0).round()),
        v.design.cost() / 10.0,
        v.revisions.of(s.id).len(),
        v.state.error.is_some()
    )
}

pub fn build_sheet(p: &mut ChildSpawnerCommands, v: &GameView) {
    let s = v.sheets.current();
    let rev = v.revisions.next_letter(s.id);
    p.spawn(panel(Node {
        width: Val::Px(330.0),
        flex_direction: FlexDirection::Column,
        ..default()
    }))
    .with_children(|b| {
        title_strip(
            b,
            &format!("{}  {}", s.id, s.title),
            &format!("rev {rev}"),
            Swatch::Panel,
            Swatch::Ink,
        );
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
        b.spawn((
            text("To pass", TextRole::Label, Swatch::Muted),
            Node {
                margin: UiRect::new(Val::Px(10.0), Val::Px(10.0), Val::Px(6.0), Val::Px(2.0)),
                ..default()
            },
        ));
        let connected = s.is_connected(&v.design);
        check_row(
            b,
            if connected {
                Status::Ok
            } else {
                Status::Pending
            },
            &format!("Span {:.0} m, all fixed nodes joined", s.span),
            if connected { "ok" } else { "open" },
        );
        check_row(
            b,
            Status::Pending,
            &format!("Survive LC1 to LC{}", s.load_cases.len()),
            "on test",
        );
        if let Some(l) = s.sag_limit {
            let (st, val) = match (static_sag(v), &v.state.error) {
                (_, Some(_)) => (Status::Failed, "unstable".to_string()),
                (Some(x), None) => (
                    if x <= l { Status::Ok } else { Status::Failed },
                    format!("{} static", mm(x)),
                ),
                (None, None) => (Status::Pending, "-".into()),
            };
            check_row(b, st, &format!("Sag under {}", mm(l)), &val);
        }
        if let Some(budget) = s.budget {
            let c = v.design.cost();
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
        rule(b, Swatch::Ink);
        b.spawn(Node {
            justify_content: JustifyContent::SpaceBetween,
            align_items: AlignItems::Center,
            padding: UiRect::axes(Val::Px(10.0), Val::Px(6.0)),
            column_gap: Val::Px(8.0),
            ..default()
        })
        .with_children(|f| {
            let bonus = s.bonus_budget.map(money).unwrap_or_default();
            f.spawn(text(
                format!("Bonus: under {bonus}, no member over 80%"),
                TextRole::Label,
                Swatch::Muted,
            ));
            key_cap(f, "F1");
        });
    });
}

pub fn readouts_key(v: &GameView) -> String {
    let w = worst(v).map(|(i, u)| (i, (u * 100.0).round() as i32));
    format!(
        "{} {:?} {:.0} {:?} {:?}",
        (v.design.cost() / 10.0).round(),
        w,
        v.design.mass(),
        v.state.error,
        v.sheets.current().budget
    )
}

pub fn build_readouts(p: &mut ChildSpawnerCommands, v: &GameView) {
    let s = v.sheets.current();
    let cost = v.design.cost();
    p.spawn(panel(Node {
        width: Val::Px(300.0),
        flex_direction: FlexDirection::Column,
        ..default()
    }))
    .with_children(|b| {
        b.spawn(Node {
            justify_content: JustifyContent::SpaceBetween,
            align_items: AlignItems::FlexEnd,
            padding: UiRect::new(Val::Px(10.0), Val::Px(10.0), Val::Px(8.0), Val::Px(2.0)),
            ..default()
        })
        .with_children(|r| {
            r.spawn(text("Cost", TextRole::Label, Swatch::Muted));
            let budget = s
                .budget
                .map(|b| format!(" / {}", money(b)))
                .unwrap_or_default();
            r.spawn(text(
                format!("{}{budget}", money(cost)),
                TextRole::Mono,
                Swatch::Ink,
            ));
        });
        if let Some(budget) = s.budget {
            let fill = if cost > budget {
                Swatch::Fail
            } else {
                Swatch::Ink
            };
            meter(b, cost / budget, fill, s.bonus_budget.map(|x| x / budget));
        }
        match worst(v) {
            Some((i, u)) => kv(
                b,
                "Worst member",
                tag_bundle(format!("M{} {}", i + 1, pct(u)), u),
            ),
            None => kv(b, "Worst member", text("-", TextRole::Mono, Swatch::Muted)),
        }
        kv(
            b,
            "Mass",
            text(
                format!("{:.0} kg", v.design.mass()),
                TextRole::Mono,
                Swatch::Ink,
            ),
        );
        let check = match &v.state.error {
            Some(_) => text(
                "unstable: add supports or bracing",
                TextRole::Body,
                Swatch::Fail,
            ),
            None if v.design.beams.is_empty() => {
                text("nothing built", TextRole::Body, Swatch::Muted)
            }
            None => text("solved", TextRole::Mono, Swatch::Ink),
        };
        kv(b, "Static check", check);
    });
}

/// A small chip filled with the utilisation band colour, with its number.
fn tag_bundle(s: String, u: f32) -> impl Bundle {
    let band = ramp_band(u);
    let fg = if band == 0 || band == 3 {
        Swatch::OnFail
    } else {
        Swatch::OnAccent
    };
    (
        Node {
            padding: UiRect::axes(Val::Px(6.0), Val::Px(1.0)),
            ..default()
        },
        Skin::boxed(Swatch::Ramp(band), Swatch::Ink, Weight::Thin).square(),
        Children::spawn_one(text(s, TextRole::Mono, fg)),
    )
}

pub fn build_tools(p: &mut ChildSpawnerCommands, v: &GameView) {
    p.spawn(column(6.0)).with_children(|c| {
        let mut tools = vec![Tool::Draw, Tool::Support, Tool::Delete];
        if v.tool.loads_allowed {
            tools.push(Tool::Load);
        }
        for t in tools {
            let selected = v.tool.tool == t;
            let (bg, fg, w) = if selected {
                (Swatch::Accent, Swatch::OnAccent, Weight::Strong)
            } else {
                (Swatch::Panel, Swatch::Ink, Weight::Thin)
            };
            c.spawn((
                Node {
                    width: Val::Px(56.0),
                    height: Val::Px(50.0),
                    flex_direction: FlexDirection::Column,
                    align_items: AlignItems::Center,
                    justify_content: JustifyContent::Center,
                    row_gap: Val::Px(2.0),
                    ..default()
                },
                Skin::boxed(bg, Swatch::Ink, w),
                super::pointer::Clickable(UiAction::SetTool(t)),
                Outline::new(Val::Px(2.0), Val::Px(1.0), Color::NONE),
            ))
            .with_children(|tile| {
                tile.spawn(text(t.label(), TextRole::Label, fg));
                tile.spawn(text(t.key(), TextRole::Mono, fg));
            });
        }
        c.spawn(Node {
            height: Val::Px(6.0),
            ..default()
        });
        c.spawn(row(4.0)).with_children(|r| {
            for (label, action, ok) in [
                ("Undo", UiAction::Undo, v.history.can_undo()),
                ("Redo", UiAction::Redo, v.history.can_redo()),
            ] {
                let kind = if ok {
                    ButtonKind::Secondary
                } else {
                    ButtonKind::Disabled
                };
                let e = button(r, label, None, action, kind);
                r.commands().entity(e).insert(Node {
                    padding: UiRect::axes(Val::Px(4.0), Val::Px(3.0)),
                    min_height: Val::Px(24.0),
                    ..default()
                });
            }
        });
    });
}

pub fn inspector_key(v: &GameView) -> String {
    let b = v.tool.selected_beam;
    let m = b.and_then(|b| v.state.members.get(b)).map(|m| {
        (
            (m.utilization * 100.0).round(),
            m.axial.map(|a| (a / 100.0).round()),
        )
    });
    format!("{b:?} {} {m:?}", v.design.revision)
}

pub fn build_inspector(p: &mut ChildSpawnerCommands, v: &GameView) {
    let Some(i) = v.tool.selected_beam.filter(|&i| i < v.design.beams.len()) else {
        return;
    };
    let beam = v.design.beams[i];
    let len = v.design.beam_length(i);
    let m = v.state.members.get(i).copied().unwrap_or_default();
    p.spawn(panel(Node {
        width: Val::Px(300.0),
        flex_direction: FlexDirection::Column,
        ..default()
    }))
    .with_children(|b| {
        title_strip(
            b,
            &format!("Member M{}", i + 1),
            "selected",
            Swatch::Accent,
            Swatch::OnAccent,
        );
        kv(
            b,
            "Part",
            text(beam.kind.label(), TextRole::Body, Swatch::Ink),
        );
        kv(
            b,
            "Section",
            text(section_label(beam.kind), TextRole::Mono, Swatch::Ink),
        );
        kv(
            b,
            "Length",
            text(format!("{len:.2} m"), TextRole::Mono, Swatch::Ink),
        );
        if let Some(a) = m.axial {
            let sense = if a >= 0.0 { "tension" } else { "compression" };
            kv(
                b,
                "Axial",
                text(
                    format!("{} {sense}", kn(a.abs())),
                    TextRole::Mono,
                    Swatch::Ink,
                ),
            );
        }
        kv(
            b,
            "Utilisation",
            tag_bundle(pct(m.utilization), m.utilization),
        );
        meter(
            b,
            m.utilization / 1.2,
            Swatch::Ramp(ramp_band(m.utilization)),
            Some(1.0 / 1.2),
        );
        kv(
            b,
            "Cost / mass",
            text(
                format!(
                    "{} / {:.0} kg",
                    money(len * beam.kind.cost_per_metre()),
                    len * beam.kind.mass_per_metre()
                ),
                TextRole::Mono,
                Swatch::Ink,
            ),
        );
        rule(b, Swatch::Ink);
        b.spawn(Node {
            column_gap: Val::Px(6.0),
            padding: UiRect::all(Val::Px(8.0)),
            flex_wrap: FlexWrap::Wrap,
            row_gap: Val::Px(6.0),
            ..default()
        })
        .with_children(|r| {
            for (n, k) in BeamKind::ALL.iter().enumerate() {
                if !v.tool.allowed.contains(k) {
                    continue;
                }
                let kind = if *k == beam.kind {
                    ButtonKind::Selected
                } else {
                    ButtonKind::Secondary
                };
                button(
                    r,
                    k.label(),
                    Some(&(n + 1).to_string()),
                    UiAction::SetMaterial(*k),
                    kind,
                );
            }
            button(
                r,
                "Delete",
                Some("Del"),
                UiAction::DeleteSelected,
                ButtonKind::Secondary,
            );
        });
    });
}

fn section_label(k: BeamKind) -> &'static str {
    match k {
        BeamKind::SteelTube => "tube 100 x 5",
        BeamKind::Timber => "100 x 150",
    }
}

pub fn build_drawer(p: &mut ChildSpawnerCommands, v: &GameView) {
    // With a member selected, the drawer shows and swaps that member's part.
    let sel_beam = v.tool.selected_beam.filter(|&i| i < v.design.beams.len());
    let sel_kind = sel_beam.map(|i| v.design.beams[i].kind);
    p.spawn(panel(Node {
        flex_direction: FlexDirection::Row,
        ..default()
    }))
    .with_children(|d| {
        d.spawn(Node {
            flex_direction: FlexDirection::Column,
            justify_content: JustifyContent::Center,
            padding: UiRect::axes(Val::Px(12.0), Val::Px(8.0)),
            width: Val::Px(110.0),
            ..default()
        })
        .with_children(|c| {
            c.spawn(text("Parts", TextRole::Label, Swatch::Muted));
            let note = match sel_beam {
                Some(i) => format!("applies to M{}", i + 1),
                None => "for new beams".to_string(),
            };
            c.spawn(text(note, TextRole::Label, Swatch::Muted));
        });
        for (n, k) in BeamKind::ALL.iter().enumerate() {
            if !v.tool.allowed.contains(k) {
                continue;
            }
            let selected = sel_kind.unwrap_or(v.tool.kind) == *k;
            d.spawn((
                Node {
                    width: Val::Px(170.0),
                    flex_direction: FlexDirection::Column,
                    padding: UiRect::axes(Val::Px(10.0), Val::Px(8.0)),
                    row_gap: Val::Px(3.0),
                    border: UiRect::left(Val::Px(1.0)),
                    ..default()
                },
                Skin::boxed(
                    if selected {
                        Swatch::Raised
                    } else {
                        Swatch::Panel
                    },
                    if selected {
                        Swatch::Accent
                    } else {
                        Swatch::Rule
                    },
                    if selected {
                        Weight::Strong
                    } else {
                        Weight::Thin
                    },
                )
                .square(),
                super::pointer::Clickable(UiAction::SetMaterial(*k)),
                Outline::new(Val::Px(2.0), Val::Px(-2.0), Color::NONE),
            ))
            .with_children(|card| {
                card.spawn(row(8.0)).with_children(|r| {
                    r.spawn(text(k.label(), TextRole::Heading, Swatch::Ink));
                    key_cap(r, &(n + 1).to_string());
                });
                card.spawn(text(section_label(*k), TextRole::Mono, Swatch::Muted));
                card.spawn(text(
                    format!(
                        "{:.1} kg/m  {:.0} kr/m",
                        k.mass_per_metre(),
                        k.cost_per_metre()
                    ),
                    TextRole::Mono,
                    Swatch::Muted,
                ));
            });
        }
    });
}

pub fn build_footer(p: &mut ChildSpawnerCommands, v: &GameView) {
    // Hint, legend and plane read-out, over the scene.
    p.spawn(Node {
        flex_direction: FlexDirection::Column,
        row_gap: Val::Px(6.0),
        ..at(Some(92.0), None, None, Some(132.0))
    })
    .with_children(|c| {
        c.spawn(text(v.tool.tool.hint(), TextRole::Body, Swatch::SceneText));
        c.spawn(row(14.0)).with_children(|r| {
            ramp_legend(r, 220.0);
            let defl = if v.scale.0 > 0.0 {
                format!("  deflection x{:.0}", v.scale.0)
            } else {
                String::new()
            };
            r.spawn(text(
                format!(
                    "plane {} at {} {:+.1} m  snap {:.1} m{defl}",
                    v.tool.plane.label(),
                    v.tool.plane.axis(),
                    v.tool.level,
                    crate::build::GRID
                ),
                TextRole::Mono,
                Swatch::SceneText,
            ));
        });
    });
    // Run button.
    p.spawn(Node {
        flex_direction: FlexDirection::Column,
        align_items: AlignItems::FlexEnd,
        row_gap: Val::Px(6.0),
        ..at(None, None, Some(20.0), Some(20.0))
    })
    .with_children(|c| {
        let s = v.sheets.current();
        if let Some(r) = v.revisions.of(s.id).last() {
            let what = if r.passed {
                "passed".to_string()
            } else {
                format!("held {:.1} s", r.held)
            };
            c.spawn(text(
                format!("last test: rev {} {what}", r.letter),
                TextRole::Mono,
                Swatch::SceneText,
            ));
        }
        let e = button(
            c,
            "RUN TEST",
            Some("Space"),
            UiAction::RunTest,
            ButtonKind::Primary,
        );
        c.commands().entity(e).insert(Node {
            width: Val::Px(300.0),
            height: Val::Px(60.0),
            padding: UiRect::axes(Val::Px(18.0), Val::Px(8.0)),
            align_items: AlignItems::Center,
            justify_content: JustifyContent::SpaceBetween,
            ..default()
        });
        c.spawn(text(
            format!(
                "LC1 to LC{}, {:.0} s",
                s.load_cases.len(),
                s.total_duration()
            ),
            TextRole::Mono,
            Swatch::SceneText,
        ));
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
            tag_bundle(format!("M{} {}%", i + 1, u), u as f32 / 100.0),
            Node {
                position_type: PositionType::Absolute,
                padding: UiRect::axes(Val::Px(6.0), Val::Px(1.0)),
                ..default()
            },
            WorldTag::Beam(i),
        ));
    }
    for (i, label) in &v.pins.pins {
        p.spawn((
            Node {
                position_type: PositionType::Absolute,
                padding: UiRect::axes(Val::Px(6.0), Val::Px(1.0)),
                ..default()
            },
            Skin::boxed(Swatch::Panel, Swatch::Fail, Weight::Strong).square(),
            WorldTag::Beam(*i),
            Children::spawn_one(text(label.clone(), TextRole::Mono, Swatch::Fail)),
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
        v.record.completed,
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
        width: Val::Px(330.0),
        flex_direction: FlexDirection::Column,
        ..default()
    }))
    .with_children(|b| {
        let rev = v.revisions.next_letter(s.id);
        title_strip(
            b,
            &format!("{} test", s.id),
            &format!("rev {rev}"),
            Swatch::Panel,
            Swatch::Ink,
        );
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
            check_row(b, st, &format!("LC{} {}", i + 1, c.name), &val);
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
                &format!("Sag under {}", mm(l)),
                &format!("peak {}", mm(rec.peak_sag)),
            );
        }
    });
}

pub fn test_log(p: &mut ChildSpawnerCommands, v: &GameView) {
    let s = v.sheets.current();
    p.spawn(panel(Node {
        width: Val::Px(280.0),
        flex_direction: FlexDirection::Column,
        ..default()
    }))
    .with_children(|b| {
        title_strip(b, "Event log", "s", Swatch::Panel, Swatch::Ink);
        let events: Vec<_> = v.record.events.iter().rev().take(8).collect();
        for e in events.into_iter().rev() {
            let (what, fail) = match &e.event {
                TestEvent::CaseStarted { case } => (
                    format!("LC{} {}", case + 1, s.load_cases[*case].name),
                    false,
                ),
                TestEvent::FirstYield {
                    member,
                    utilization,
                } => (
                    format!("M{} first yield {}", member + 1, pct(*utilization)),
                    false,
                ),
                TestEvent::Broke { member, cause } => (format!("M{} {}", member + 1, cause), true),
            };
            rule(b, Swatch::Rule);
            b.spawn((
                Node {
                    padding: UiRect::axes(Val::Px(10.0), Val::Px(3.0)),
                    column_gap: Val::Px(10.0),
                    ..default()
                },
                Skin::fill(if fail { Swatch::Fail } else { Swatch::Clear }).square(),
            ))
            .with_children(|r| {
                let fg = if fail { Swatch::OnFail } else { Swatch::Ink };
                r.spawn((
                    text(
                        format!("{:5.2}", e.time),
                        TextRole::Mono,
                        if fail { fg } else { Swatch::Muted },
                    ),
                    Node {
                        width: Val::Px(44.0),
                        ..default()
                    },
                ));
                r.spawn(text(what, TextRole::Body, fg));
            });
        }
    });
}

pub fn test_banner(p: &mut ChildSpawnerCommands, v: &GameView) {
    let msg = if v.slowmo.active() {
        Some(("SLOW-MO 1/4x", "auto, first break  |  . to resume speed"))
    } else if v.clock.paused {
        Some(("PAUSED", "Space to resume"))
    } else {
        None
    };
    let Some((head, tail)) = msg else { return };
    p.spawn((
        Node { ..default() },
        Skin::boxed(Swatch::Panel, Swatch::Ink, Weight::Strong).square(),
    ))
    .with_children(|b| {
        b.spawn((
            Node {
                padding: UiRect::axes(Val::Px(12.0), Val::Px(6.0)),
                ..default()
            },
            Skin::fill(Swatch::Instrument).square(),
        ))
        .with_child(text(head, TextRole::Mono, Swatch::Accent));
        b.spawn(Node {
            padding: UiRect::axes(Val::Px(12.0), Val::Px(6.0)),
            ..default()
        })
        .with_child(text(tail, TextRole::Body, Swatch::Ink));
    });
}

pub fn test_controls(p: &mut ChildSpawnerCommands, v: &GameView) {
    p.spawn(row(8.0)).with_children(|r| {
        r.spawn(text("Test time", TextRole::Label, Swatch::InstrumentMuted));
        r.spawn(text("Esc abort", TextRole::Label, Swatch::InstrumentMuted));
    });
    p.spawn(Node {
        align_items: AlignItems::Baseline,
        column_gap: Val::Px(6.0),
        ..default()
    })
    .with_children(|r| {
        r.spawn((
            text("00.00", TextRole::MonoLarge, Swatch::InstrumentInk),
            TimeText,
        ));
        r.spawn(text(
            format!("/ {:.1} s", v.sheets.current().total_duration()),
            TextRole::Mono,
            Swatch::InstrumentMuted,
        ));
    });
    p.spawn(row(4.0)).with_children(|r| {
        let pause = if v.clock.paused { "Run" } else { "Pause" };
        for (label, key, a, on) in [
            (pause, "Spc", UiAction::SimPause, false),
            ("<<", ",", UiAction::SimSlower, false),
            (">>", ".", UiAction::SimFaster, false),
            ("Heat", "H", UiAction::ToggleHeatmap, v.heat.0),
        ] {
            r.spawn((
                Node {
                    flex_direction: FlexDirection::Column,
                    align_items: AlignItems::Center,
                    padding: UiRect::axes(Val::Px(6.0), Val::Px(3.0)),
                    min_width: Val::Px(38.0),
                    ..default()
                },
                Skin::boxed(
                    Swatch::Instrument,
                    if on {
                        Swatch::Accent
                    } else {
                        Swatch::InstrumentMuted
                    },
                    Weight::Thin,
                ),
                super::pointer::Clickable(a),
                Outline::new(Val::Px(2.0), Val::Px(1.0), Color::NONE),
            ))
            .with_children(|k| {
                k.spawn(text(
                    label,
                    TextRole::Mono,
                    if on {
                        Swatch::Accent
                    } else {
                        Swatch::InstrumentInk
                    },
                ));
                k.spawn(text(key, TextRole::Label, Swatch::InstrumentMuted));
            });
        }
    });
    p.spawn(row(0.0)).with_children(|r| {
        for s in crate::analysis::SPEEDS {
            let on = (s - v.clock.speed).abs() < 1e-3;
            let label = if s < 1.0 {
                format!("1/{:.0}", 1.0 / s)
            } else {
                format!("{s:.0}x")
            };
            r.spawn((
                Node {
                    padding: UiRect::axes(Val::Px(4.0), Val::Px(1.0)),
                    ..default()
                },
                Skin::fill(if on { Swatch::Accent } else { Swatch::Clear }).square(),
            ))
            .with_child(text(
                label,
                TextRole::Label,
                if on {
                    Swatch::OnAccent
                } else {
                    Swatch::InstrumentMuted
                },
            ));
        }
    });
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
            width: Val::Px(230.0),
            ..default()
        },
        Skin::boxed(Swatch::Panel, Swatch::Fail, Weight::Strong).square(),
        WorldTag::Beam(m),
    ))
    .with_children(|c| {
        c.spawn((
            Node {
                justify_content: JustifyContent::SpaceBetween,
                padding: UiRect::axes(Val::Px(8.0), Val::Px(3.0)),
                ..default()
            },
            Skin::fill(Swatch::Fail).square(),
        ))
        .with_children(|h| {
            h.spawn(text("FRACTURE", TextRole::Heading, Swatch::OnFail));
            h.spawn(text(format!("t {t:.2} s"), TextRole::Mono, Swatch::OnFail));
        });
        c.spawn(Node {
            padding: UiRect::axes(Val::Px(8.0), Val::Px(5.0)),
            ..default()
        })
        .with_child(text(
            format!("M{} {kind}: {cause}", m + 1),
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
    mut time_text: Query<&mut Text, With<TimeText>>,
) {
    let Ok((area, children)) = area.single() else {
        return;
    };
    let s = sheets.current();
    let total = s.total_duration().max(1e-3);
    if children.is_none_or(|c| c.is_empty()) {
        build_chart(&mut commands, area, s);
        return;
    }
    let t = sim.map(|s| s.sim.time() as f32).unwrap_or(0.0);
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
        // Load-case bands.
        a.spawn(Node {
            height: Val::Px(18.0),
            flex_shrink: 0.0,
            ..default()
        })
        .with_children(|bands| {
            for (i, c) in s.load_cases.iter().enumerate() {
                bands
                    .spawn((
                        Node {
                            flex_grow: c.duration,
                            flex_basis: Val::Px(0.0),
                            padding: UiRect::axes(Val::Px(6.0), Val::Px(2.0)),
                            border: UiRect::right(Val::Px(1.0)),
                            overflow: Overflow::clip(),
                            ..default()
                        },
                        Skin::boxed(Swatch::Chart, Swatch::Ink, Weight::Thin).square(),
                    ))
                    .with_child((
                        Text::new(format!("LC{} {}", i + 1, c.name)),
                        Ink::new(TextRole::Label, Swatch::Ink),
                    ));
            }
        });
        // Bars, 100% line, playhead.
        a.spawn(Node {
            flex_grow: 1.0,
            align_items: AlignItems::FlexEnd,
            ..default()
        })
        .with_children(|plot| {
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
                Text::new("100%"),
                Ink::new(TextRole::Label, Swatch::Fail),
                Node {
                    position_type: PositionType::Absolute,
                    left: Val::Px(4.0),
                    bottom: Val::Percent(100.0 / 1.2),
                    ..default()
                },
            ));
            plot.spawn((
                Node {
                    position_type: PositionType::Absolute,
                    top: Val::Px(0.0),
                    bottom: Val::Px(0.0),
                    width: Val::Px(2.0),
                    left: Val::Percent(0.0),
                    ..default()
                },
                Skin::fill(Swatch::Ink).square(),
                Playhead,
            ));
            plot.spawn((
                Text::new("worst member utilisation"),
                Ink::new(TextRole::Label, Swatch::Muted),
                Node {
                    position_type: PositionType::Absolute,
                    right: Val::Px(8.0),
                    top: Val::Px(6.0),
                    ..default()
                },
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
