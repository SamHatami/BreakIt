//! Full screens and overlays: test register, brief, test report, pause,
//! brief overlay and controls card.

use bevy::ecs::hierarchy::ChildSpawnerCommands;
use bevy::prelude::*;

use super::theme::{Skin, Swatch, TextRole, Weight};
use super::widgets::*;
use super::{GameView, View, ViewKey};
use crate::flow::{unlocked, Overlay, Screen, UiAction};
use crate::scenario::{money, Rig, Scenario, TestEvent};

pub fn spawn_register(mut commands: Commands) {
    commands.spawn((
        Node {
            padding: UiRect::all(Val::Px(36.0)),
            column_gap: Val::Px(36.0),
            align_items: AlignItems::FlexStart,
            ..layer()
        },
        DespawnOnExit(Screen::Register),
        View::Register,
        ViewKey::default(),
    ));
}

pub fn spawn_brief(mut commands: Commands) {
    commands.spawn((
        Node {
            justify_content: JustifyContent::Center,
            align_items: AlignItems::Center,
            ..layer()
        },
        Skin::fill(Swatch::Scrim),
        DespawnOnExit(Screen::Brief),
        View::Brief,
        ViewKey::default(),
    ));
}

pub fn spawn_report(mut commands: Commands) {
    commands.spawn((
        Node {
            justify_content: JustifyContent::Center,
            align_items: AlignItems::Center,
            ..layer()
        },
        Skin::fill(Swatch::Scrim),
        DespawnOnExit(Screen::Report),
        View::Report,
        ViewKey::default(),
    ));
}

pub fn kn(newtons: f32) -> String {
    format!("{:.0} kN", newtons / 1000.0)
}

pub fn mm(metres: f32) -> String {
    format!("{:.0} mm", metres * 1000.0)
}

pub fn pct(u: f32) -> String {
    format!("{:.0}%", u * 100.0)
}

/// What a sheet loads, for the register and the brief: `3 x 40 kN`.
fn load_summary(s: &Scenario) -> String {
    let Some(c) = s.heaviest_case() else {
        return if s.sandbox {
            "your own".into()
        } else {
            "self weight".into()
        };
    };
    let loads = &s.load_cases[c].loads;
    let max = loads.iter().map(|l| l.1).fold(0.0, f32::max);
    format!("{} x {}", loads.len(), kn(max))
}

pub fn register_key(v: &GameView) -> String {
    format!(
        "{} {}",
        v.sheets.current,
        v.revisions.0.iter().map(|r| r.1.len()).sum::<usize>()
    )
}

pub fn build_register(p: &mut ChildSpawnerCommands, v: &GameView) {
    let sheets = &v.sheets;
    let cur = sheets.current();
    let passed = sheets
        .all
        .iter()
        .filter(|s| !s.sandbox && v.revisions.passed(s.id))
        .count();
    let total = sheets.all.iter().filter(|s| !s.sandbox).count();

    // The register sheet.
    p.spawn(panel(Node {
        width: Val::Px(620.0),
        flex_direction: FlexDirection::Column,
        ..default()
    }))
    .with_children(|s| {
        s.spawn(Node {
            justify_content: JustifyContent::SpaceBetween,
            align_items: AlignItems::FlexEnd,
            padding: UiRect::axes(Val::Px(16.0), Val::Px(12.0)),
            ..default()
        })
        .with_children(|h| {
            h.spawn(column(2.0)).with_children(|c| {
                c.spawn(text(
                    "BreakIt structures lab",
                    TextRole::Label,
                    Swatch::Muted,
                ));
                c.spawn(text("Test register", TextRole::Title, Swatch::Ink));
            });
            h.spawn(column(2.0)).with_children(|c| {
                c.spawn(text("Passed", TextRole::Label, Swatch::Muted));
                c.spawn(text(
                    format!("{passed} / {total}"),
                    TextRole::Heading,
                    Swatch::Ink,
                ));
            });
        });
        rule(s, Swatch::Ink);
        // Rig tabs.
        s.spawn(Node {
            column_gap: Val::Px(0.0),
            ..default()
        })
        .with_children(|tabs| {
            for rig in Rig::ALL {
                let kind = if rig == cur.rig {
                    ButtonKind::Selected
                } else {
                    ButtonKind::Secondary
                };
                button(tabs, rig.label(), None, UiAction::SelectRig(rig), kind);
            }
        });
        rule(s, Swatch::Ink);
        // Column heads.
        sheet_row(
            s,
            ["No.", "Sheet", "Span", "Best", "Revs"],
            Swatch::Muted,
            None,
            TextRole::Label,
        );
        for (i, sc) in sheets
            .all
            .iter()
            .enumerate()
            .filter(|(_, sc)| sc.rig == cur.rig)
        {
            let revs = v.revisions.of(sc.id);
            let open = unlocked(sheets, &v.revisions, i);
            let best = revs
                .iter()
                .filter(|r| r.passed)
                .min_by(|a, b| a.cost.total_cmp(&b.cost))
                .map(|r| format!("{} {}", r.letter, money(r.cost)))
                .unwrap_or_else(|| if open { "-".into() } else { "locked".into() });
            let title = if open {
                sc.title.to_string()
            } else {
                format!("{}  (locked)", sc.title)
            };
            let span = if sc.sandbox {
                "-".into()
            } else {
                format!("{:.0} m", sc.span)
            };
            let color = if open { Swatch::Ink } else { Swatch::Muted };
            let selected = i == sheets.current;
            rule(s, Swatch::Rule);
            sheet_row(
                s,
                [sc.id, &title, &span, &best, &revs.len().to_string()],
                if selected { Swatch::OnAccent } else { color },
                Some((UiAction::SelectSheet(i), selected)),
                TextRole::Body,
            );
        }
        rule(s, Swatch::Ink);
        s.spawn(Node {
            column_gap: Val::Px(14.0),
            padding: UiRect::axes(Val::Px(16.0), Val::Px(10.0)),
            align_items: AlignItems::Center,
            ..default()
        })
        .with_children(|h| {
            let theme = format!("theme: {}", v.theme.current().name);
            for (k, what) in [
                ("Up/Down", "sheet"),
                ("Enter", "open"),
                ("F10", theme.as_str()),
            ] {
                key_cap(h, k);
                h.spawn(text(what, TextRole::Body, Swatch::Muted));
            }
        });
    });

    // The preview card for the selected sheet.
    let open = unlocked(sheets, &v.revisions, sheets.current);
    p.spawn(panel(Node {
        width: Val::Px(420.0),
        flex_direction: FlexDirection::Column,
        margin: UiRect::top(Val::Px(260.0)),
        ..default()
    }))
    .with_children(|c| {
        title_strip(c, cur.title, cur.id, Swatch::Panel, Swatch::Ink);
        c.spawn(text(
            format!("  {} rig", cur.rig.label()),
            TextRole::Label,
            Swatch::Muted,
        ));
        let span = if cur.sandbox {
            "free".into()
        } else {
            format!("{:.1} m", cur.span)
        };
        kv(c, "Span", text(span, TextRole::Mono, Swatch::Ink));
        kv(
            c,
            "Load",
            text(load_summary(cur), TextRole::Mono, Swatch::Ink),
        );
        kv(
            c,
            "Budget",
            text(
                cur.budget.map(money).unwrap_or("none".into()),
                TextRole::Mono,
                Swatch::Ink,
            ),
        );
        kv(
            c,
            "Parts",
            text(parts_label(cur), TextRole::Body, Swatch::Ink),
        );
        rule(c, Swatch::Ink);
        c.spawn(Node {
            justify_content: JustifyContent::SpaceBetween,
            align_items: AlignItems::Center,
            padding: UiRect::all(Val::Px(10.0)),
            ..default()
        })
        .with_children(|f| {
            let last = v.revisions.of(cur.id).last();
            let note = match last {
                Some(r) if r.passed => format!("last: rev {} passed", r.letter),
                Some(r) => format!("last: rev {} held {:.1} s", r.letter, r.held),
                None => "not tested yet".into(),
            };
            f.spawn(text(note, TextRole::Mono, Swatch::Muted));
            let (label, kind) = match (open, cur.sandbox) {
                (false, _) => ("LOCKED", ButtonKind::Disabled),
                (true, true) => ("OPEN BENCH", ButtonKind::Primary),
                (true, false) => ("OPEN BRIEF", ButtonKind::Primary),
            };
            button(f, label, Some("Enter"), UiAction::OpenBrief, kind);
        });
    });
}

fn parts_label(s: &Scenario) -> String {
    s.allowed
        .iter()
        .map(|k| k.label())
        .collect::<Vec<_>>()
        .join(", ")
}

/// A register row: five cells, clickable when it has an action.
fn sheet_row(
    p: &mut ChildSpawnerCommands,
    cells: [&str; 5],
    color: Swatch,
    action: Option<(UiAction, bool)>,
    role: TextRole,
) {
    let widths = [64.0, 300.0, 64.0, 120.0, 40.0];
    let mut e = p.spawn(Node {
        padding: UiRect::axes(Val::Px(16.0), Val::Px(7.0)),
        align_items: AlignItems::Center,
        ..default()
    });
    if let Some((a, selected)) = action {
        e.insert((
            super::pointer::Clickable(a),
            Outline::new(Val::Px(2.0), Val::Px(-2.0), Color::NONE),
            if selected {
                Skin::fill(Swatch::Accent).square()
            } else {
                Skin::fill(Swatch::Clear).square()
            },
        ));
    }
    e.with_children(|r| {
        for (i, c) in cells.iter().enumerate() {
            let role = if i == 0 || i >= 2 {
                TextRole::Mono
            } else {
                role
            };
            r.spawn((
                text(*c, role, color),
                Node {
                    width: Val::Px(widths[i]),
                    ..default()
                },
            ));
        }
    });
}

/// The brief: one sheet, no scrolling. `overlay` adds a Close button
/// instead of Back / Start.
pub fn build_brief(p: &mut ChildSpawnerCommands, v: &GameView, overlay: bool) {
    let s = v.sheets.current();
    let sheet_no = v
        .sheets
        .all
        .iter()
        .filter(|x| x.rig == s.rig)
        .position(|x| x.id == s.id)
        .unwrap_or(0)
        + 1;
    let of = v.sheets.all.iter().filter(|x| x.rig == s.rig).count();
    p.spawn(panel(Node {
        width: Val::Px(860.0),
        flex_direction: FlexDirection::Column,
        ..default()
    }))
    .with_children(|b| {
        b.spawn(Node {
            justify_content: JustifyContent::SpaceBetween,
            align_items: AlignItems::FlexEnd,
            padding: UiRect::axes(Val::Px(16.0), Val::Px(12.0)),
            ..default()
        })
        .with_children(|h| {
            h.spawn(column(2.0)).with_children(|c| {
                c.spawn(text(
                    format!("{}  |  {} rig", s.id, s.rig.label()),
                    TextRole::Label,
                    Swatch::Muted,
                ));
                c.spawn(text(s.title, TextRole::Title, Swatch::Ink));
            });
            h.spawn(text(format!("sheet {sheet_no} of {of}"), TextRole::Mono, Swatch::Muted));
        });
        rule(b, Swatch::Ink);
        b.spawn(Node::default()).with_children(|cols| {
            // Left: the site and the load cases.
            cols.spawn(Node {
                flex_direction: FlexDirection::Column,
                flex_grow: 1.0,
                padding: UiRect::all(Val::Px(16.0)),
                row_gap: Val::Px(14.0),
                ..default()
            })
            .with_children(|l| {
                l.spawn(text("Site", TextRole::Label, Swatch::Muted));
                site_drawing(l, s);
                l.spawn(text(
                    format!(
                        "Span {:.1} m between pinned anchors. {} load point(s) on the deck line. Join them all.",
                        s.span,
                        s.load_points.len()
                    ),
                    TextRole::Body,
                    Swatch::Ink,
                ));
                l.spawn(text("Load cases, in test order", TextRole::Label, Swatch::Muted));
                l.spawn(column(0.0)).with_children(|t| {
                    for (i, c) in s.load_cases.iter().enumerate() {
                        rule(t, Swatch::Rule);
                        t.spawn(Node {
                            padding: UiRect::axes(Val::Px(0.0), Val::Px(4.0)),
                            column_gap: Val::Px(12.0),
                            ..default()
                        })
                        .with_children(|r| {
                            r.spawn((text(format!("LC{}", i + 1), TextRole::Mono, Swatch::Muted), Node { width: Val::Px(40.0), ..default() }));
                            r.spawn((text(c.name, TextRole::Body, Swatch::Ink), Node { width: Val::Px(200.0), ..default() }));
                            let loads = if c.loads.is_empty() {
                                "-".to_string()
                            } else {
                                format!("{} x {}", c.loads.len(), kn(c.loads[0].1))
                            };
                            r.spawn((text(loads, TextRole::Mono, Swatch::Ink), Node { width: Val::Px(110.0), ..default() }));
                            r.spawn(text(format!("{:.0} s", c.duration), TextRole::Mono, Swatch::Muted));
                        });
                    }
                });
            });
            cols.spawn((
                Node {
                    width: Val::Px(1.0),
                    ..default()
                },
                Skin::fill(Swatch::Ink).square(),
            ));
            // Right: what passes.
            cols.spawn(Node {
                flex_direction: FlexDirection::Column,
                width: Val::Px(300.0),
                padding: UiRect::all(Val::Px(16.0)),
                row_gap: Val::Px(6.0),
                ..default()
            })
            .with_children(|r| {
                r.spawn(text("To pass", TextRole::Label, Swatch::Muted));
                r.spawn(column(0.0)).with_children(|c| {
                    check_row(c, Status::Pending, &format!("Survive LC1 to LC{}", s.load_cases.len()), "");
                    if let Some(l) = s.sag_limit {
                        check_row(c, Status::Pending, &format!("Sag under {}", mm(l)), "");
                    }
                    if let Some(b) = s.budget {
                        check_row(c, Status::Pending, &format!("Cost under {}", money(b)), "");
                    }
                });
                r.spawn(Node { height: Val::Px(8.0), ..default() });
                r.spawn(text("Bonus grade", TextRole::Label, Swatch::Muted));
                r.spawn(column(0.0)).with_children(|c| {
                    if let Some(b) = s.bonus_budget {
                        check_row(c, Status::Pending, &format!("Cost under {}", money(b)), "");
                    }
                    check_row(c, Status::Pending, "No member over 80%", "");
                });
                r.spawn(Node { height: Val::Px(8.0), ..default() });
                r.spawn(text("Parts allowed", TextRole::Label, Swatch::Muted));
                r.spawn(text(parts_label(s), TextRole::Body, Swatch::Ink));
            });
        });
        rule(b, Swatch::Ink);
        b.spawn(Node {
            justify_content: JustifyContent::SpaceBetween,
            align_items: AlignItems::Center,
            padding: UiRect::all(Val::Px(10.0)),
            ..default()
        })
        .with_children(|f| {
            if overlay {
                f.spawn(text("This sheet stays on F1.", TextRole::Body, Swatch::Muted));
                button(f, "Close", Some("F1"), UiAction::CloseOverlay, ButtonKind::Secondary);
            } else {
                button(f, "Back", Some("Esc"), UiAction::BackToRegister, ButtonKind::Secondary);
                button(f, "START BUILD", Some("Enter"), UiAction::StartBuild, ButtonKind::Primary);
            }
        });
    });
}

/// The site in elevation: anchors as filled blocks, load points as marks
/// on the deck line, laid out to scale.
fn site_drawing(p: &mut ChildSpawnerCommands, s: &Scenario) {
    let w = 480.0;
    let xs: Vec<f32> = s
        .anchors
        .iter()
        .map(|a| a.0.x)
        .chain(s.load_points.iter().map(|l| l.x))
        .collect();
    let (min, max) = xs
        .iter()
        .fold((f32::MAX, f32::MIN), |(a, b), &x| (a.min(x), b.max(x)));
    let span = (max - min).max(1.0);
    let x = |v: f32| 20.0 + (v - min) / span * (w - 40.0);
    p.spawn((
        Node {
            width: Val::Px(w),
            height: Val::Px(84.0),
            ..default()
        },
        Skin::boxed(Swatch::Raised, Swatch::Rule, Weight::Thin).square(),
    ))
    .with_children(|d| {
        // Deck line.
        d.spawn((
            Node {
                width: Val::Px(x(max) - x(min)),
                height: Val::Px(1.0),
                ..at(Some(x(min)), Some(44.0), None, None)
            },
            Skin::fill(Swatch::Muted).square(),
        ));
        for a in &s.anchors {
            d.spawn((
                Node {
                    width: Val::Px(14.0),
                    height: Val::Px(22.0),
                    ..at(Some(x(a.0.x) - 7.0), Some(46.0), None, None)
                },
                Skin::fill(Swatch::Ink).square(),
            ));
        }
        for l in &s.load_points {
            d.spawn((
                Node {
                    width: Val::Px(2.0),
                    height: Val::Px(26.0),
                    ..at(Some(x(l.x) - 1.0), Some(16.0), None, None)
                },
                Skin::fill(Swatch::Fail).square(),
            ));
        }
        d.spawn((
            text(format!("{:.1} m", s.span), TextRole::Mono, Swatch::Ink),
            at(Some(w / 2.0 - 20.0), Some(64.0), None, None),
        ));
    });
}

pub fn build_report(p: &mut ChildSpawnerCommands, v: &GameView) {
    let s = v.sheets.current();
    let last = &v.last;
    let rec = &last.record;
    p.spawn(panel(Node {
        width: Val::Px(920.0),
        flex_direction: FlexDirection::Column,
        ..default()
    }))
    .with_children(|b| {
        // Title block.
        b.spawn(Node {
            align_items: AlignItems::Stretch,
            ..default()
        })
        .with_children(|h| {
            let cell = |w: f32| Node {
                width: Val::Px(w),
                flex_direction: FlexDirection::Column,
                padding: UiRect::axes(Val::Px(12.0), Val::Px(8.0)),
                row_gap: Val::Px(2.0),
                ..default()
            };
            h.spawn(cell(90.0))
                .with_child(text(s.id, TextRole::Heading, Swatch::Ink));
            h.spawn(Node {
                flex_grow: 1.0,
                ..cell(0.0)
            })
            .with_children(|c| {
                c.spawn(text("Test report", TextRole::Label, Swatch::Muted));
                c.spawn(text(s.title, TextRole::Title, Swatch::Ink));
            });
            h.spawn(cell(110.0)).with_children(|c| {
                c.spawn(text("Revision", TextRole::Label, Swatch::Muted));
                c.spawn(text(
                    last.letter.to_string(),
                    TextRole::Heading,
                    Swatch::Ink,
                ));
            });
            h.spawn(cell(140.0)).with_children(|c| {
                c.spawn(text("Cost", TextRole::Label, Swatch::Muted));
                c.spawn(text(money(rec.cost), TextRole::Mono, Swatch::Ink));
            });
        });
        rule(b, Swatch::Ink);
        b.spawn(Node::default()).with_children(|cols| {
            cols.spawn(Node {
                flex_direction: FlexDirection::Column,
                flex_grow: 1.0,
                padding: UiRect::all(Val::Px(16.0)),
                row_gap: Val::Px(14.0),
                ..default()
            })
            .with_children(|l| {
                // Stamp.
                let (word, c) = if last.passed {
                    ("PASSED", Swatch::Ink)
                } else {
                    ("FAILED", Swatch::Fail)
                };
                l.spawn((
                    Node {
                        padding: UiRect::axes(Val::Px(16.0), Val::Px(4.0)),
                        align_self: AlignSelf::FlexStart,
                        ..default()
                    },
                    Skin::boxed(Swatch::Clear, c, Weight::Strong).square(),
                    UiTransform::from_rotation(Rot2::degrees(-4.0)),
                ))
                .with_child(text(word, TextRole::Stamp, c));
                l.spawn(text(
                    "Criteria, measured against required",
                    TextRole::Label,
                    Swatch::Muted,
                ));
                l.spawn(column(0.0)).with_children(|t| {
                    report_row(
                        t,
                        ["Criterion", "Measured", "Required", ""],
                        Swatch::Muted,
                        TextRole::Label,
                    );
                    for c in &last.checks {
                        let mark = if c.ok { "OK" } else { "X" };
                        let col = if c.ok {
                            Swatch::Ink
                        } else if c.bonus {
                            Swatch::Muted
                        } else {
                            Swatch::Fail
                        };
                        rule(t, Swatch::Rule);
                        report_row(
                            t,
                            [&c.label, &c.measured, &c.required, mark],
                            col,
                            TextRole::Body,
                        );
                    }
                });
                l.spawn(text("Failure sequence", TextRole::Label, Swatch::Muted));
                l.spawn(Node {
                    column_gap: Val::Px(6.0),
                    flex_wrap: FlexWrap::Wrap,
                    row_gap: Val::Px(6.0),
                    ..default()
                })
                .with_children(|seq| {
                    let steps: Vec<_> = rec
                        .events
                        .iter()
                        .filter(|e| !matches!(e.event, TestEvent::CaseStarted { .. }))
                        .take(5)
                        .collect();
                    if steps.is_empty() {
                        seq.spawn(text(
                            "Nothing yielded or broke.",
                            TextRole::Body,
                            Swatch::Ink,
                        ));
                    }
                    for e in steps {
                        let (what, swatch) = match &e.event {
                            TestEvent::FirstYield { member, .. } => {
                                (format!("M{} yields", member + 1), Swatch::Raised)
                            }
                            TestEvent::Broke { member, cause } => {
                                (format!("M{} {}", member + 1, cause), Swatch::Fail)
                            }
                            TestEvent::CaseStarted { .. } => continue,
                        };
                        let fg = if swatch == Swatch::Fail {
                            Swatch::OnFail
                        } else {
                            Swatch::Ink
                        };
                        seq.spawn((
                            Node {
                                flex_direction: FlexDirection::Column,
                                padding: UiRect::axes(Val::Px(8.0), Val::Px(4.0)),
                                ..default()
                            },
                            Skin::boxed(swatch, Swatch::Ink, Weight::Thin),
                        ))
                        .with_children(|c| {
                            c.spawn(text(format!("{:.2} s", e.time), TextRole::Mono, fg));
                            c.spawn(text(what, TextRole::Body, fg));
                        });
                    }
                });
                l.spawn(text(why(v), TextRole::Body, Swatch::Ink));
            });
            cols.spawn((
                Node {
                    width: Val::Px(1.0),
                    ..default()
                },
                Skin::fill(Swatch::Ink).square(),
            ));
            cols.spawn(Node {
                flex_direction: FlexDirection::Column,
                width: Val::Px(280.0),
                padding: UiRect::all(Val::Px(16.0)),
                row_gap: Val::Px(6.0),
                ..default()
            })
            .with_children(|r| {
                r.spawn(text("Revisions", TextRole::Label, Swatch::Muted));
                r.spawn(column(0.0)).with_children(|t| {
                    for rev in v.revisions.of(s.id) {
                        rule(t, Swatch::Rule);
                        let current = rev.letter == last.letter;
                        t.spawn((
                            Node {
                                justify_content: JustifyContent::SpaceBetween,
                                padding: UiRect::axes(Val::Px(6.0), Val::Px(4.0)),
                                ..default()
                            },
                            Skin::fill(if current {
                                Swatch::Accent
                            } else {
                                Swatch::Clear
                            })
                            .square(),
                        ))
                        .with_children(|row| {
                            let fg = if current {
                                Swatch::OnAccent
                            } else {
                                Swatch::Ink
                            };
                            row.spawn(text(rev.letter.to_string(), TextRole::Mono, fg));
                            row.spawn(text(
                                if rev.passed {
                                    "passed".to_string()
                                } else {
                                    format!("held {:.1} s", rev.held)
                                },
                                TextRole::Body,
                                fg,
                            ));
                            row.spawn(text(money(rev.cost), TextRole::Mono, fg));
                        });
                    }
                });
                let broken = rec.broken_members();
                if !broken.is_empty() {
                    r.spawn(Node {
                        height: Val::Px(10.0),
                        ..default()
                    });
                    r.spawn(text(
                        "Revise pins these in build",
                        TextRole::Label,
                        Swatch::Muted,
                    ));
                    for m in broken.iter().take(4) {
                        let kind = v
                            .design
                            .beams
                            .get(*m)
                            .map(|b| b.kind.label())
                            .unwrap_or("member");
                        r.spawn(text(
                            format!("M{} {}", m + 1, kind),
                            TextRole::Body,
                            Swatch::Fail,
                        ));
                    }
                }
            });
        });
        rule(b, Swatch::Ink);
        b.spawn(Node {
            column_gap: Val::Px(10.0),
            align_items: AlignItems::Center,
            padding: UiRect::all(Val::Px(10.0)),
            ..default()
        })
        .with_children(|f| {
            let has_next = v
                .sheets
                .all
                .iter()
                .skip(v.sheets.current + 1)
                .any(|x| x.rig == s.rig);
            let (revise, next) = if last.passed {
                (
                    ButtonKind::Secondary,
                    if has_next {
                        ButtonKind::Primary
                    } else {
                        ButtonKind::Disabled
                    },
                )
            } else {
                (ButtonKind::Primary, ButtonKind::Disabled)
            };
            button(f, "REVISE", Some("R"), UiAction::Revise, revise);
            button(
                f,
                "Replay",
                Some("P"),
                UiAction::Replay,
                ButtonKind::Secondary,
            );
            button(f, "Next sheet", Some("N"), UiAction::NextSheet, next);
            f.spawn(Node {
                flex_grow: 1.0,
                ..default()
            });
            button(
                f,
                "Register",
                Some("Esc"),
                UiAction::BackToRegister,
                ButtonKind::Secondary,
            );
        });
    });
}

fn report_row(p: &mut ChildSpawnerCommands, cells: [&str; 4], color: Swatch, role: TextRole) {
    let widths = [220.0, 160.0, 210.0, 36.0];
    p.spawn(Node {
        padding: UiRect::axes(Val::Px(0.0), Val::Px(4.0)),
        ..default()
    })
    .with_children(|r| {
        for (i, c) in cells.iter().enumerate() {
            let role = if i == 0 || role == TextRole::Label {
                role
            } else {
                TextRole::Mono
            };
            r.spawn((
                text(*c, role, color),
                Node {
                    width: Val::Px(widths[i]),
                    ..default()
                },
            ));
        }
    });
}

/// One plain sentence on why it failed, from the solver's events.
fn why(v: &GameView) -> String {
    let s = v.sheets.current();
    let rec = &v.last.record;
    let first = rec.events.iter().find_map(|e| match &e.event {
        TestEvent::Broke { member, cause } => Some((e.time, *member, cause.clone())),
        _ => None,
    });
    if let Some((t, m, cause)) = first {
        let case = s
            .case_at(t)
            .map(|c| s.load_cases[c].name)
            .unwrap_or("the test");
        let kind = v
            .design
            .beams
            .get(m)
            .map(|b| b.kind.label())
            .unwrap_or("member");
        return format!(
            "Why: M{} ({kind}) failed by {cause} at {t:.2} s, during \"{case}\". Strengthen that member or give the load another path.",
            m + 1
        );
    }
    if let Some(l) = s.sag_limit {
        if rec.peak_sag > l {
            return format!(
                "Why: nothing broke, but the deck sagged {} against {} allowed. A deeper truss or stiffer chords cut the sag.",
                mm(rec.peak_sag),
                mm(l)
            );
        }
    }
    if v.last.passed {
        "It held. Try a cheaper revision for the bonus grade.".into()
    } else {
        "Over budget. Same structure with lighter members?".into()
    }
}

pub fn build_overlay(p: &mut ChildSpawnerCommands, v: &GameView) {
    let o = *v.overlay;
    if o == Overlay::None {
        return;
    }
    p.spawn((
        Node {
            justify_content: JustifyContent::Center,
            align_items: AlignItems::Center,
            ..layer()
        },
        Skin::fill(Swatch::Scrim),
        super::pointer::BlocksPointer,
    ))
    .with_children(|l| match o {
        Overlay::Brief => build_brief(l, v, true),
        Overlay::Pause => {
            l.spawn(panel(Node {
                width: Val::Px(360.0),
                flex_direction: FlexDirection::Column,
                ..default()
            }))
            .with_children(|b| {
                title_strip(
                    b,
                    "Paused",
                    v.sheets.current().id,
                    Swatch::Panel,
                    Swatch::Ink,
                );
                b.spawn(Node {
                    flex_direction: FlexDirection::Column,
                    row_gap: Val::Px(8.0),
                    padding: UiRect::all(Val::Px(14.0)),
                    ..default()
                })
                .with_children(|c| {
                    button(
                        c,
                        "RESUME",
                        Some("Esc"),
                        UiAction::CloseOverlay,
                        ButtonKind::Primary,
                    );
                    if !v.sheets.current().sandbox {
                        button(
                            c,
                            "Brief",
                            Some("F1"),
                            UiAction::ToggleBrief,
                            ButtonKind::Secondary,
                        );
                    }
                    button(
                        c,
                        "Controls",
                        Some("/"),
                        UiAction::ToggleControls,
                        ButtonKind::Secondary,
                    );
                    button(
                        c,
                        "Back to register",
                        Some("Q"),
                        UiAction::BackToRegister,
                        ButtonKind::Secondary,
                    );
                });
            });
        }
        Overlay::Controls => {
            l.spawn(panel(Node {
                width: Val::Px(520.0),
                flex_direction: FlexDirection::Column,
                ..default()
            }))
            .with_children(|b| {
                title_strip(b, "Controls", "build", Swatch::Panel, Swatch::Ink);
                for (k, what) in CONTROLS {
                    rule(b, Swatch::Rule);
                    b.spawn(Node {
                        padding: UiRect::axes(Val::Px(12.0), Val::Px(4.0)),
                        column_gap: Val::Px(12.0),
                        ..default()
                    })
                    .with_children(|r| {
                        r.spawn((
                            text(k, TextRole::Mono, Swatch::Ink),
                            Node {
                                width: Val::Px(150.0),
                                ..default()
                            },
                        ));
                        r.spawn(text(what, TextRole::Body, Swatch::Ink));
                    });
                }
                rule(b, Swatch::Ink);
                b.spawn(Node {
                    padding: UiRect::all(Val::Px(10.0)),
                    ..default()
                })
                .with_children(|f| {
                    button(
                        f,
                        "Close",
                        Some("Esc"),
                        UiAction::CloseOverlay,
                        ButtonKind::Secondary,
                    );
                });
            });
        }
        Overlay::None => {}
    });
}

/// The controls card. Keep in step with `flow::keyboard_actions`.
const CONTROLS: [(&str, &str); 15] = [
    ("RMB drag", "orbit"),
    ("MMB / Shift+RMB", "pan"),
    ("Wheel", "zoom"),
    ("F", "frame selection, or everything"),
    ("LMB", "use the tool: place, chain, select"),
    ("B S X L", "draw, support, delete, load (sandbox)"),
    ("1 2", "steel, timber (also the selected beam)"),
    ("Del", "delete selected beam / hovered node"),
    ("Ctrl+Z / Ctrl+Y", "undo / redo"),
    ("Tab, Q / E", "drawing plane, plane down / up"),
    ("D", "exaggerate deflection"),
    ("C", "clear to the sheet's start"),
    ("Space", "run test; in a test: pause"),
    (", .", "test slower / faster; H heatmap"),
    ("F1 / Esc", "brief / pause or back"),
];
