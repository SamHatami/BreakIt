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

/// What a sheet loads, for the register and the brief.
fn load_summary(s: &Scenario) -> String {
    s.load_summary()
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
                c.spawn(caps("BreakIt structures lab", Swatch::Accent));
                c.spawn(text("Test register", TextRole::Title, Swatch::Ink));
            });
            h.spawn(column(2.0)).with_children(|c| {
                c.spawn(caps("Passed", Swatch::Muted));
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
                color,
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
        title_strip(c, &cur.title, cur.id, Swatch::Panel, Swatch::Ink);
        c.spawn((
            caps(&format!("{} rig", cur.rig.label()), Swatch::Muted),
            Node {
                margin: UiRect::bottom(Val::Px(8.0)),
                ..default()
            },
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
                (false, _) => ("Locked", ButtonKind::Disabled),
                (true, true) if cur.playground => ("Open playground", ButtonKind::Primary),
                (true, true) => ("Open bench", ButtonKind::Primary),
                (true, false) => ("Open brief", ButtonKind::Primary),
            };
            button(f, label, Some("Enter"), UiAction::OpenBrief, kind);
        });
    });
}

fn parts_label(s: &Scenario) -> String {
    let mut classes: Vec<&str> = Vec::new();
    for k in &s.allowed {
        let c = match k.material_kind() {
            breakit_core::MaterialKind::Steel => "steel",
            breakit_core::MaterialKind::Timber => "timber",
            breakit_core::MaterialKind::Concrete => "concrete",
            breakit_core::MaterialKind::Composite => "carbon fibre",
        };
        if !classes.contains(&c) {
            classes.push(c);
        }
    }
    let list = classes.join(", ");
    let mut it = list.chars();
    match it.next() {
        Some(f) => format!(
            "{}{} members, any standard size",
            f.to_uppercase(),
            it.as_str()
        ),
        None => "None".into(),
    }
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
                Skin::fill(Swatch::Raised)
            } else {
                Skin::fill(Swatch::Clear)
            },
        ));
    }
    e.with_children(|r| {
        for (i, c) in cells.iter().enumerate() {
            let role = if i == 0 && role != TextRole::Label {
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
                c.spawn(caps(
                    &format!("{}  /  {} rig", s.id, s.rig.label()),
                    Swatch::Accent,
                ));
                c.spawn(text(s.title.clone(), TextRole::Title, Swatch::Ink));
            });
            h.spawn(text(
                format!("sheet {sheet_no} of {of}"),
                TextRole::Mono,
                Swatch::Muted,
            ));
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
                l.spawn(caps("Site", Swatch::Muted));
                site_drawing(l, s);
                l.spawn(text(s.def.brief.clone(), TextRole::Body, Swatch::Ink));
                l.spawn(caps("Load cases, in test order", Swatch::Muted));
                l.spawn(column(0.0)).with_children(|t| {
                    for (i, c) in s.load_cases.iter().enumerate() {
                        rule(t, Swatch::Rule);
                        t.spawn(Node {
                            padding: UiRect::axes(Val::Px(0.0), Val::Px(4.0)),
                            column_gap: Val::Px(12.0),
                            ..default()
                        })
                        .with_children(|r| {
                            r.spawn((
                                text(format!("LC{}", i + 1), TextRole::Mono, Swatch::Muted),
                                Node {
                                    width: Val::Px(40.0),
                                    ..default()
                                },
                            ));
                            r.spawn((
                                text(c.name.as_str(), TextRole::Body, Swatch::Ink),
                                Node {
                                    width: Val::Px(200.0),
                                    ..default()
                                },
                            ));
                            r.spawn((
                                text(c.load.as_str(), TextRole::Mono, Swatch::Ink),
                                Node {
                                    width: Val::Px(110.0),
                                    ..default()
                                },
                            ));
                            r.spawn(text(
                                format!("{:.1} to {:.1} s", c.start, c.end),
                                TextRole::Mono,
                                Swatch::Muted,
                            ));
                        });
                    }
                });
            });
            cols.spawn((
                Node {
                    width: Val::Px(1.0),
                    ..default()
                },
                Skin::fill(Swatch::Rule).square(),
            ));
            // Right: what passes.
            cols.spawn(Node {
                flex_direction: FlexDirection::Column,
                width: Val::Px(300.0),
                flex_shrink: 0.0,
                padding: UiRect::all(Val::Px(16.0)),
                row_gap: Val::Px(6.0),
                ..default()
            })
            .with_children(|r| {
                r.spawn(caps("To pass", Swatch::Muted));
                r.spawn(column(0.0)).with_children(|c| {
                    check_row(
                        c,
                        Status::Pending,
                        &format!("Survive {:.1} s of loads", s.def.duration),
                        "",
                    );
                    if let Some(b) = s.budget {
                        check_row(c, Status::Pending, &format!("Cost under {}", money(b)), "");
                    }
                    if let Some(m) = s.def.limits.max_mass {
                        check_row(c, Status::Pending, &format!("Mass under {m:.0} kg"), "");
                    }
                });
                r.spawn(Node {
                    height: Val::Px(8.0),
                    ..default()
                });
                r.spawn(caps("Full marks", Swatch::Muted));
                r.spawn(column(0.0)).with_children(|c| {
                    if let Some(b) = s.bonus_budget {
                        check_row(c, Status::Pending, &format!("Cost under {}", money(b)), "");
                    }
                    let sc = &s.def.scoring;
                    check_row(
                        c,
                        Status::Pending,
                        &format!("Mass under {:.0} kg", sc.target_mass),
                        "",
                    );
                    check_row(
                        c,
                        Status::Pending,
                        &format!(
                            "Safety margin {:.2} to {:.1}",
                            sc.margin_band[0], sc.margin_band[1]
                        ),
                        "",
                    );
                    for o in &s.def.objectives {
                        check_row(c, Status::Pending, &crate::scenario::objective_label(o), "");
                    }
                });
                r.spawn(Node {
                    height: Val::Px(8.0),
                    ..default()
                });
                r.spawn(caps("Parts allowed", Swatch::Muted));
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
                f.spawn(text(
                    "This sheet stays on F1.",
                    TextRole::Body,
                    Swatch::Muted,
                ));
                button(
                    f,
                    "Close",
                    Some("F1"),
                    UiAction::CloseOverlay,
                    ButtonKind::Secondary,
                );
            } else {
                button(
                    f,
                    "Back",
                    Some("Esc"),
                    UiAction::BackToRegister,
                    ButtonKind::Secondary,
                );
                button(
                    f,
                    "Start build",
                    Some("Enter"),
                    UiAction::StartBuild,
                    ButtonKind::Primary,
                );
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
            h.spawn(Node {
                flex_grow: 1.0,
                ..cell(0.0)
            })
            .with_children(|c| {
                c.spawn(caps(&format!("{}  test report", s.id), Swatch::Accent));
                c.spawn(text(s.title.clone(), TextRole::Title, Swatch::Ink));
            });
            h.spawn(cell(110.0)).with_children(|c| {
                c.spawn(caps("Revision", Swatch::Muted));
                c.spawn(text(
                    last.letter.to_string(),
                    TextRole::Heading,
                    Swatch::Ink,
                ));
            });
            h.spawn(cell(140.0)).with_children(|c| {
                c.spawn(caps("Cost", Swatch::Muted));
                c.spawn(text(money(rec.cost), TextRole::Heading, Swatch::Ink));
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
                    ("Passed", Swatch::Ok)
                } else {
                    ("Failed", Swatch::Fail)
                };
                l.spawn(Node {
                    column_gap: Val::Px(40.0),
                    align_items: AlignItems::FlexStart,
                    ..default()
                })
                .with_children(|top| {
                    top.spawn(column(2.0)).with_children(|g| {
                        g.spawn(caps("Result", Swatch::Muted));
                        g.spawn(text(word, TextRole::Stamp, c));
                    });
                    // Grade: score, stars, and the proof test's read-outs.
                    if let Some(r) = last.report.as_ref().filter(|r| r.passed && r.ranked) {
                        top.spawn(column(2.0)).with_children(|g| {
                            g.spawn(caps("Grade", Swatch::Muted));
                            g.spawn(row(6.0)).with_children(|st| {
                                for i in 0..3 {
                                    st.spawn((
                                        Node {
                                            width: Val::Px(10.0),
                                            height: Val::Px(10.0),
                                            ..default()
                                        },
                                        Skin::fill(if i < r.stars {
                                            Swatch::Accent
                                        } else {
                                            Swatch::Rule
                                        })
                                        .pill(),
                                    ));
                                }
                                st.spawn((
                                    text(
                                        format!("{} / 1000", r.score),
                                        TextRole::Heading,
                                        Swatch::Ink,
                                    ),
                                    Node {
                                        margin: UiRect::left(Val::Px(8.0)),
                                        ..default()
                                    },
                                ));
                            });
                            g.spawn(text(
                                format!("safety margin {:.2}", r.safety_margin),
                                TextRole::Body,
                                Swatch::Muted,
                            ));
                        });
                        if let Some(f) = r.reserve_factor {
                            top.spawn(column(2.0)).with_children(|g| {
                                g.spawn(caps("Proof test", Swatch::Muted));
                                g.spawn(text(
                                    format!("held to x{f:.1}"),
                                    TextRole::Heading,
                                    Swatch::Ink,
                                ));
                                let warn = r
                                    .warning_time
                                    .map(|w| format!("{w:.1} s warning before collapse"))
                                    .unwrap_or_else(|| "no warning before collapse".into());
                                g.spawn(text(warn, TextRole::Body, Swatch::Muted));
                            });
                        }
                    }
                });
                l.spawn(column(0.0)).with_children(|t| {
                    report_row(
                        t,
                        ["Criterion", "Measured", "Required", ""],
                        Swatch::Muted,
                        TextRole::Label,
                    );
                    for c in &last.checks {
                        let mark = if c.ok {
                            "ok"
                        } else if c.bonus {
                            "-"
                        } else {
                            "no"
                        };
                        let col = if c.ok {
                            Swatch::Ink
                        } else if c.bonus {
                            Swatch::Muted
                        } else {
                            Swatch::Fail
                        };
                        report_row(
                            t,
                            [&c.label, &c.measured, &c.required, mark],
                            col,
                            TextRole::Body,
                        );
                    }
                });
                // The test's own failures; after a pass, how the proof test
                // broke it.
                let test_end = s.def.duration as f32;
                let in_proof = last.passed;
                l.spawn(caps(
                    if in_proof {
                        "Proof test: what gave way first"
                    } else {
                        "Failure sequence"
                    },
                    Swatch::Muted,
                ));
                l.spawn(Node {
                    column_gap: Val::Px(28.0),
                    flex_wrap: FlexWrap::Wrap,
                    row_gap: Val::Px(6.0),
                    ..default()
                })
                .with_children(|seq| {
                    let steps: Vec<_> = rec
                        .events
                        .iter()
                        .filter(|e| !matches!(e.event, TestEvent::CaseStarted { .. }))
                        .filter(|e| (e.time > test_end) == in_proof)
                        .take(5)
                        .collect();
                    if steps.is_empty() {
                        seq.spawn(text(
                            if in_proof {
                                "Nothing gave way."
                            } else {
                                "Nothing yielded or broke."
                            },
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
                            TestEvent::Collapse { .. } => ("collapse".to_string(), Swatch::Fail),
                            _ => continue,
                        };
                        let fg = if swatch == Swatch::Fail {
                            Swatch::Fail
                        } else {
                            Swatch::Ink
                        };
                        seq.spawn(column(1.0)).with_children(|c| {
                            c.spawn(text(
                                format!("{:.2} s", e.time),
                                TextRole::Label,
                                Swatch::Muted,
                            ));
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
                Skin::fill(Swatch::Rule).square(),
            ));
            cols.spawn(Node {
                flex_direction: FlexDirection::Column,
                width: Val::Px(280.0),
                padding: UiRect::all(Val::Px(16.0)),
                row_gap: Val::Px(6.0),
                ..default()
            })
            .with_children(|r| {
                r.spawn(caps("Revisions", Swatch::Muted));
                r.spawn(column(0.0)).with_children(|t| {
                    for rev in v.revisions.of(s.id) {
                        let current = rev.letter == last.letter;
                        t.spawn((
                            Node {
                                justify_content: JustifyContent::SpaceBetween,
                                padding: UiRect::axes(Val::Px(8.0), Val::Px(5.0)),
                                ..default()
                            },
                            Skin::fill(if current {
                                Swatch::Raised
                            } else {
                                Swatch::Clear
                            }),
                        ))
                        .with_children(|row| {
                            let fg = if current { Swatch::Ink } else { Swatch::Muted };
                            row.spawn(text(
                                rev.letter.to_string(),
                                TextRole::Body,
                                if current { Swatch::Accent } else { fg },
                            ));
                            row.spawn(text(
                                if rev.passed {
                                    "passed".to_string()
                                } else {
                                    format!("held {:.1} s", rev.held)
                                },
                                TextRole::Body,
                                fg,
                            ));
                            row.spawn(text(money(rev.cost), TextRole::Body, fg));
                        });
                    }
                });
                let test_end = s.def.duration as f32;
                let broken: Vec<usize> = rec
                    .events
                    .iter()
                    .filter_map(|e| match e.event {
                        TestEvent::Broke { member, .. } if (e.time > test_end) == last.passed => {
                            Some(member)
                        }
                        _ => None,
                    })
                    .collect();
                if !broken.is_empty() {
                    r.spawn(Node {
                        height: Val::Px(10.0),
                        ..default()
                    });
                    r.spawn(caps("Revise pins these in build", Swatch::Muted));
                    for m in broken.iter().take(4) {
                        let kind = v
                            .design
                            .beams
                            .get(*m)
                            .map(|b| b.kind.label())
                            .unwrap_or("member");
                        r.spawn(text(
                            format!("Member {} {}", m + 1, kind),
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
            button(f, "Revise", Some("R"), UiAction::Revise, revise);
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
    let widths = [200.0, 170.0, 250.0, 30.0];
    p.spawn(Node {
        padding: UiRect::axes(Val::Px(0.0), Val::Px(3.0)),
        column_gap: Val::Px(12.0),
        ..default()
    })
    .with_children(|r| {
        for (i, c) in cells.iter().enumerate() {
            let color = if i > 0 && i < 3 && role != TextRole::Label {
                Swatch::Muted
            } else {
                color
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
    let test_end = s.def.duration as f32;
    let first = rec.events.iter().find_map(|e| match &e.event {
        TestEvent::Broke { member, cause } if e.time <= test_end && !v.last.passed => {
            Some((e.time, *member, cause.clone()))
        }
        _ => None,
    });
    if let Some((t, m, cause)) = first {
        let case = s
            .case_at(t)
            .map(|c| s.load_cases[c].name.clone())
            .unwrap_or_else(|| "the test".into());
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
    let report = v.last.report.as_ref();
    if let Some(f) = report.and_then(|r| r.failure.as_ref()) {
        return format!("Why: {f}. Stiffen the structure or give the load another path.");
    }
    match report {
        Some(r) if r.passed && r.stars < 3 => {
            "It held. A lighter, cheaper revision that still holds scores higher.".into()
        }
        Some(r) if r.passed => "It held, and nothing to spare. Well judged.".into(),
        _ => "It did not pass.".into(),
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
                        "Resume",
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
const CONTROLS: [(&str, &str); 16] = [
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
    ("T / F3", "next play table / frame time"),
    ("Space", "run test; in a test: pause"),
    (", .", "test slower / faster; H heatmap"),
    ("F1 / Esc", "brief / pause or back"),
];
