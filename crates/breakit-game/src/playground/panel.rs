//! The playground's screen: the load bench on the left, view switches,
//! the inspector and the graph on the right, time controls along the
//! bottom, and reaction values beside the supports.
//!
//! Same pattern as `ui`: each panel is rebuilt when its key (a summary of
//! what it shows) changes, and fast numbers (clock, inspector values, graph
//! bars, reaction tags) update in place every frame. Hairline look: type
//! straight on the scene, the accent only for what is on.

use bevy::ecs::hierarchy::ChildSpawnerCommands;
use bevy::ecs::system::SystemParam;
use bevy::prelude::*;
use breakit_lab::PlayTable;
use breakit_playground::{Param, Probe, QuakeMotion, WaveKind};

use super::{Bench, Load, Pick, PlayAction, Show, TRACE_SECONDS};
use crate::analysis::{SimClock, Simulation};
use crate::camera::OrbitCamera;
use crate::design::{from_core, Design};
use crate::flow::{Screen, UiAction};
use crate::testrun::Heatmap;
use crate::ui::pointer::{BlocksPointer, Clickable};
use crate::ui::theme::{Skin, Swatch, TextRole};
use crate::ui::widgets::{at, button, caps, column, layer, row, rule, text, ButtonKind};

/// Distance of the panels from the screen edges, px.
const EDGE: f32 = 32.0;
/// Columns in the graph.
const GRAPH_COLUMNS: usize = 120;
/// Graph size, px.
const GRAPH_W: f32 = 300.0;
const GRAPH_H: f32 = 84.0;
/// How long a status note stays, s.
const NOTE_SECONDS: f64 = 4.0;

/// A playground panel.
#[derive(Component, Clone, Copy, Debug, PartialEq, Eq)]
pub enum PlayView {
    Bench,
    View,
    Inspector,
    Controls,
    Tags,
}

#[derive(Component, Default)]
pub struct PlayKey(Option<String>);

/// A number updated every frame.
#[derive(Component, Clone, Copy, Debug, PartialEq, Eq)]
pub enum Readout {
    Clock,
    Speed,
    Field(usize),
    GraphNow,
    GraphRange,
}

#[derive(Component, Clone, Copy, Debug)]
pub struct GraphBar(usize);

/// Reaction value beside a support node.
#[derive(Component, Clone, Copy, Debug)]
pub struct ReactionTag(usize);

#[derive(SystemParam)]
pub struct Ctx<'w> {
    bench: Res<'w, Bench>,
    clock: Res<'w, SimClock>,
    heat: Res<'w, Heatmap>,
    design: Res<'w, Design>,
    table: Res<'w, PlayTable>,
    time: Res<'w, Time<Real>>,
}

impl Ctx<'_> {
    fn note(&self) -> Option<&str> {
        let (msg, at) = self.bench.note.as_ref()?;
        (self.time.elapsed_secs_f64() - at < NOTE_SECONDS).then_some(msg.as_str())
    }
}

pub fn spawn(mut commands: Commands) {
    let panel = |v: PlayView, node: Node| (node, v, PlayKey::default());
    commands
        .spawn((layer(), DespawnOnExit(Screen::Playground)))
        .with_children(|l| {
            l.spawn(panel(PlayView::Tags, layer()));
            l.spawn((
                panel(
                    PlayView::Bench,
                    Node {
                        width: Val::Px(300.0),
                        ..at(Some(EDGE), Some(EDGE), None, None)
                    },
                ),
                BlocksPointer,
            ));
            l.spawn((
                Node {
                    width: Val::Px(GRAPH_W),
                    row_gap: Val::Px(14.0),
                    ..at(None, Some(EDGE), Some(EDGE), None)
                },
                BlocksPointer,
            ))
            .with_children(|r| {
                r.spawn(panel(PlayView::View, column(2.0)));
                r.spawn(panel(PlayView::Inspector, column(2.0)));
            });
            l.spawn((
                panel(
                    PlayView::Controls,
                    Node {
                        flex_direction: FlexDirection::Column,
                        row_gap: Val::Px(6.0),
                        ..at(Some(EDGE), None, Some(EDGE), Some(20.0))
                    },
                ),
                BlocksPointer,
            ));
        });
}

fn key(v: PlayView, c: &Ctx) -> String {
    let b = &c.bench;
    match v {
        PlayView::Bench => format!("{:?} {} {}", b.setup, b.point_tool, c.table.insert.name()),
        PlayView::View => format!("{} {} {} {}", c.heat.0, b.forces, b.reactions, b.deform),
        PlayView::Inspector => format!("{:?} {:?}", b.pick, b.trace.as_ref().map(|t| t.probe)),
        PlayView::Controls => format!(
            "{} {} {:?} {}",
            c.clock.paused,
            c.clock.speed,
            c.note(),
            c.table.insert.name()
        ),
        PlayView::Tags => format!("{} {}", b.reactions, c.design.revision),
    }
}

fn build(v: PlayView, p: &mut ChildSpawnerCommands, c: &Ctx) {
    match v {
        PlayView::Bench => bench(p, c),
        PlayView::View => view(p, c),
        PlayView::Inspector => inspector(p, c),
        PlayView::Controls => controls(p, c),
        PlayView::Tags => tags(p, c),
    }
}

/// Rebuilds every panel whose key changed.
pub fn refresh(
    mut commands: Commands,
    ctx: Ctx,
    mut roots: Query<(Entity, &PlayView, &mut PlayKey)>,
) {
    for (e, v, mut k) in &mut roots {
        let want = key(*v, &ctx);
        if k.0.as_ref() == Some(&want) {
            continue;
        }
        k.0 = Some(want);
        let mut ec = commands.entity(e);
        ec.despawn_children();
        ec.with_children(|p| build(*v, p, &ctx));
    }
}

fn act(a: PlayAction) -> Clickable {
    Clickable(UiAction::Playground(a))
}

/// A small clickable word: `-`, `+`, ON / OFF, a mode name.
fn chip(p: &mut ChildSpawnerCommands, label: &str, a: PlayAction, on: bool) {
    p.spawn((
        Node {
            padding: UiRect::axes(Val::Px(7.0), Val::Px(1.0)),
            ..default()
        },
        Skin::fill(Swatch::Clear),
        act(a),
        Outline::new(Val::Px(1.0), Val::Px(0.0), Color::NONE),
    ))
    .with_children(|c| {
        let color = if on { Swatch::Accent } else { Swatch::Muted };
        c.spawn(caps(label, color));
    });
}

/// Label, then `-  value  +`.
fn stepper(p: &mut ChildSpawnerCommands, c: &Ctx, param: Param, label: &str) {
    p.spawn(Node {
        align_items: AlignItems::Center,
        min_height: Val::Px(20.0),
        ..default()
    })
    .with_children(|r| {
        r.spawn((
            text(label, TextRole::Body, Swatch::Muted),
            Node {
                flex_grow: 1.0,
                ..default()
            },
        ));
        chip(r, "-", PlayAction::Step(param, -1), false);
        r.spawn((
            Node {
                width: Val::Px(84.0),
                justify_content: JustifyContent::Center,
                ..default()
            },
            children![text(
                param.display(&c.bench.setup),
                TextRole::Mono,
                Swatch::Ink
            )],
        ));
        chip(r, "+", PlayAction::Step(param, 1), false);
    });
}

/// Label, then a clickable word.
fn choice(p: &mut ChildSpawnerCommands, label: &str, value: &str, a: PlayAction, on: bool) {
    p.spawn(Node {
        align_items: AlignItems::Center,
        min_height: Val::Px(20.0),
        ..default()
    })
    .with_children(|r| {
        r.spawn((
            text(label, TextRole::Body, Swatch::Muted),
            Node {
                flex_grow: 1.0,
                ..default()
            },
        ));
        chip(r, value, a, on);
    });
}

/// A section head: name, key, and an on/off switch if it has one.
fn head(p: &mut ChildSpawnerCommands, name: &str, key: &str, toggle: Option<(PlayAction, bool)>) {
    p.spawn(Node {
        align_items: AlignItems::Center,
        margin: UiRect::top(Val::Px(8.0)),
        column_gap: Val::Px(8.0),
        ..default()
    })
    .with_children(|r| {
        r.spawn((
            text(name, TextRole::Heading, Swatch::Ink),
            Node {
                flex_grow: 1.0,
                ..default()
            },
        ));
        r.spawn(caps(key, Swatch::Muted));
        if let Some((a, on)) = toggle {
            chip(r, if on { "on" } else { "off" }, a, on);
        }
    });
}

fn bench(p: &mut ChildSpawnerCommands, c: &Ctx) {
    let s = &c.bench.setup;
    p.spawn(caps(
        &format!("Playground  {}", c.table.insert.name()),
        Swatch::Accent,
    ));
    p.spawn(text("Load bench", TextRole::Title, Swatch::Ink));

    head(
        p,
        "Wind",
        "1",
        Some((PlayAction::Toggle(Load::Wind), s.wind.on)),
    );
    if s.wind.on {
        stepper(p, c, Param::WindSpeed, "Speed");
        stepper(p, c, Param::WindHeading, "Towards");
        let g = if s.wind.gusts { "on" } else { "off" };
        choice(p, "Gusts", g, PlayAction::Toggle(Load::Gusts), s.wind.gusts);
        if s.wind.gusts {
            stepper(p, c, Param::Turbulence, "Turbulence");
        }
    }

    head(
        p,
        "Shake table",
        "2",
        Some((PlayAction::Toggle(Load::Quake), s.quake.on)),
    );
    if s.quake.on {
        let q = &s.quake;
        choice(
            p,
            "Motion",
            q.motion.label(),
            PlayAction::NextQuakeMotion,
            true,
        );
        stepper(p, c, Param::QuakeAmplitude, "Peak");
        let f = match q.motion {
            QuakeMotion::Harmonic => "Frequency",
            QuakeMotion::Synthetic => "Soil frequency",
            QuakeMotion::Sweep => "Sweep up to",
        };
        stepper(p, c, Param::QuakeFrequency, f);
        stepper(p, c, Param::QuakeHeading, "Direction");
        if q.motion == QuakeMotion::Synthetic {
            choice(
                p,
                "Record",
                &format!("#{}  new", q.seed),
                PlayAction::NewRecord,
                false,
            );
        }
    }

    head(
        p,
        "Water",
        "3",
        Some((PlayAction::Toggle(Load::Water), s.water.on)),
    );
    if s.water.on {
        let w = &s.water;
        stepper(p, c, Param::WaterLevel, "Level");
        stepper(p, c, Param::Current, "Current");
        choice(
            p,
            "Waves",
            w.waves.label(),
            PlayAction::NextWaves,
            w.waves != WaveKind::Calm,
        );
        if w.waves != WaveKind::Calm {
            stepper(p, c, Param::WaveHeight, "Height");
        }
        if w.waves == WaveKind::Regular {
            stepper(p, c, Param::WavePeriod, "Period");
        }
    }

    head(p, "Traffic", "V", None);
    stepper(p, c, Param::TruckMass, "Truck");
    stepper(p, c, Param::TruckSpeed, "Speed");
    stepper(p, c, Param::TruckAxles, "Axles");
    choice(p, "", "send truck", PlayAction::SendTruck, true);

    head(p, "Drop weight", "B", None);
    stepper(p, c, Param::DropMass, "Mass");
    stepper(p, c, Param::DropHeight, "Height");
    let target = match c.bench.pick {
        Some(Pick::Node(n)) => format!("drop on node {n}"),
        Some(Pick::Member(m)) => format!("drop on member {m}"),
        None => "drop in the middle".into(),
    };
    choice(p, "", &target, PlayAction::DropWeight, true);

    head(p, "Gravity", "", None);
    stepper(p, c, Param::Gravity, "Gravity");

    head(p, "Point loads", "P", None);
    stepper(p, c, Param::PointLoad, "Size");
    let n = s.point_loads.len();
    p.spawn(row(6.0)).with_children(|r| {
        let place = if c.bench.point_tool {
            "click nodes"
        } else {
            "place"
        };
        chip(r, place, PlayAction::PointTool, c.bench.point_tool);
        if n > 0 {
            chip(r, &format!("clear {n}"), PlayAction::ClearPointLoads, false);
        }
    });
}

fn view(p: &mut ChildSpawnerCommands, c: &Ctx) {
    p.spawn(caps("View", Swatch::Muted));
    let b = &c.bench;
    let rows = [
        (
            "Stress colours",
            "H",
            PlayAction::Show(Show::Heatmap),
            c.heat.0,
        ),
        (
            "Applied forces",
            "A",
            PlayAction::Show(Show::Forces),
            b.forces,
        ),
        (
            "Reactions",
            "X",
            PlayAction::Show(Show::Reactions),
            b.reactions,
        ),
    ];
    for (label, k, a, on) in rows {
        p.spawn(row(8.0)).with_children(|r| {
            r.spawn((
                text(label, TextRole::Body, Swatch::Ink),
                Node {
                    flex_grow: 1.0,
                    ..default()
                },
            ));
            r.spawn(caps(k, Swatch::Muted));
            chip(r, if on { "on" } else { "off" }, a, on);
        });
    }
    let d = b.deform_scale();
    p.spawn(row(8.0)).with_children(|r| {
        r.spawn((
            text("Deformation", TextRole::Body, Swatch::Ink),
            Node {
                flex_grow: 1.0,
                ..default()
            },
        ));
        r.spawn(caps("D", Swatch::Muted));
        let label = if d > 1.0 {
            format!("x{d:.0}")
        } else {
            "true".into()
        };
        chip(r, &label, PlayAction::NextDeform, d > 1.0);
    });
}

/// Inspector rows for a pick: labels; values come from [`fields`].
fn field_labels(pick: Pick) -> &'static [&'static str] {
    match pick {
        Pick::Member(_) => &[
            "Utilisation",
            "Axial force",
            "End moment",
            "State",
            "Damage",
            "Plastic strain",
            "Joints a / b",
        ],
        Pick::Node(_) => &[
            "Displacement",
            "Vertical",
            "Speed",
            "Applied force",
            "Reaction",
            "Mass",
        ],
    }
}

/// Current inspector values for a pick.
fn fields(pick: Pick, sim: &Simulation) -> Vec<String> {
    let Some(live) = sim.live() else {
        return Vec::new();
    };
    let s = live.sim();
    match pick {
        Pick::Member(m) if m < live.structure().members.len() => {
            let d = s.damage(m);
            let moment = d
                .moment
                .iter()
                .flatten()
                .map(|v| v.abs())
                .fold(0.0, f64::max);
            vec![
                if s.is_broken(m) {
                    "broken".into()
                } else {
                    format!("{:.0} %", s.utilization(m) * 100.0)
                },
                format!(
                    "{:.1} kN {}",
                    d.axial.abs() / 1000.0,
                    if d.axial >= 0.0 { "tension" } else { "compr." }
                ),
                format!("{:.2} kNm", moment / 1000.0),
                format!("{:?}", d.state).to_lowercase(),
                format!("{:.0} %", d.damage * 100.0),
                format!("{:.2} %", d.plastic_strain * 100.0),
                format!("{:.0} / {:.0} %", d.joint[0] * 100.0, d.joint[1] * 100.0),
            ]
        }
        Pick::Node(n) if n < s.node_count() => {
            let u = s.displacement(n);
            let f = live
                .applied_forces()
                .get(n)
                .map(|f| f.norm())
                .unwrap_or(0.0);
            let r = live
                .reactions()
                .into_iter()
                .find(|(k, _)| *k == n)
                .map(|(_, r)| format!("{:.1} kN", r.norm() / 1000.0))
                .unwrap_or_else(|| "free".into());
            vec![
                format!("{:.1} mm", u.norm() * 1000.0),
                format!("{:+.1} mm", u.y * 1000.0),
                format!("{:.2} m/s", s.velocity(n).norm()),
                format!("{:.1} kN", f / 1000.0),
                r,
                format!("{:.0} kg", s.node_mass(n)),
            ]
        }
        _ => Vec::new(),
    }
}

fn inspector(p: &mut ChildSpawnerCommands, c: &Ctx) {
    p.spawn(caps("Inspector", Swatch::Muted));
    let Some(pick) = c.bench.pick else {
        p.spawn(text(
            "Click a member or node to read it live and graph it.",
            TextRole::Body,
            Swatch::Muted,
        ));
        return;
    };
    let (title, sub) = match pick {
        Pick::Member(m) => (
            format!("Member {m}"),
            c.design
                .beams
                .get(m)
                .map(|b| b.kind.designation())
                .unwrap_or_default(),
        ),
        Pick::Node(n) => (
            format!("Node {n}"),
            c.design
                .nodes
                .get(n)
                .map(|p| format!("at {:.1}, {:.1}, {:.1} m", p.x, p.y, p.z))
                .unwrap_or_default(),
        ),
    };
    p.spawn(row(8.0)).with_children(|r| {
        r.spawn((
            text(title, TextRole::Heading, Swatch::Ink),
            Node {
                flex_grow: 1.0,
                ..default()
            },
        ));
        chip(r, "close", PlayAction::Unpick, false);
    });
    p.spawn(text(sub, TextRole::Body, Swatch::Muted));
    for (i, label) in field_labels(pick).iter().enumerate() {
        p.spawn(Node {
            justify_content: JustifyContent::SpaceBetween,
            min_height: Val::Px(18.0),
            ..default()
        })
        .with_children(|r| {
            r.spawn(text(*label, TextRole::Body, Swatch::Muted));
            r.spawn((text("-", TextRole::Mono, Swatch::Ink), Readout::Field(i)));
        });
    }
    rule(p, Swatch::Rule);
    let probe = c.bench.trace.as_ref().map(|t| t.probe);
    p.spawn(row(8.0)).with_children(|r| {
        let label = probe.map(Probe::label).unwrap_or_default();
        r.spawn((
            text(label, TextRole::Body, Swatch::Ink),
            Node {
                flex_grow: 1.0,
                ..default()
            },
        ));
        r.spawn(caps("G", Swatch::Muted));
        chip(r, "next", PlayAction::NextProbe, false);
    });
    p.spawn(row(8.0)).with_children(|r| {
        r.spawn((text("", TextRole::Mono, Swatch::Ink), Readout::GraphNow));
        r.spawn((text("", TextRole::Body, Swatch::Muted), Readout::GraphRange));
    });
    // The graph: bars from the zero line, newest on the right.
    p.spawn((
        Node {
            width: Val::Px(GRAPH_W),
            height: Val::Px(GRAPH_H),
            margin: UiRect::top(Val::Px(4.0)),
            ..default()
        },
        Skin::fill(Swatch::Clear),
    ))
    .with_children(|g| {
        let w = GRAPH_W / GRAPH_COLUMNS as f32;
        for i in 0..GRAPH_COLUMNS {
            g.spawn((
                Node {
                    position_type: PositionType::Absolute,
                    left: Val::Px(i as f32 * w),
                    width: Val::Px((w - 0.5).max(1.0)),
                    top: Val::Percent(100.0),
                    height: Val::Px(0.0),
                    ..default()
                },
                Skin::fill(Swatch::Accent).square(),
                GraphBar(i),
            ));
        }
        g.spawn((
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
    });
    p.spawn(caps(&format!("last {TRACE_SECONDS:.0} s"), Swatch::Muted));
}

fn controls(p: &mut ChildSpawnerCommands, c: &Ctx) {
    if let Some(n) = c.note() {
        p.spawn(text(n, TextRole::Body, Swatch::SceneText));
    }
    p.spawn(Node {
        align_items: AlignItems::Center,
        column_gap: Val::Px(10.0),
        ..default()
    })
    .with_children(|r| {
        r.spawn((text("", TextRole::MonoLarge, Swatch::Ink), Readout::Clock));
        r.spawn((
            Node {
                width: Val::Px(110.0),
                ..default()
            },
            children![(caps("", Swatch::Accent), Readout::Speed)],
        ));
        let paused = c.clock.paused;
        let (label, kind) = if paused {
            ("Run", ButtonKind::Primary)
        } else {
            ("Pause", ButtonKind::Secondary)
        };
        let a = |x| UiAction::Playground(x);
        button(r, label, Some("Space"), a(PlayAction::Pause), kind);
        button(
            r,
            "Step",
            Some("N"),
            a(PlayAction::StepFrame),
            ButtonKind::Secondary,
        );
        button(
            r,
            "Slower",
            Some(","),
            a(PlayAction::Slower),
            ButtonKind::Secondary,
        );
        button(
            r,
            "Faster",
            Some("."),
            a(PlayAction::Faster),
            ButtonKind::Secondary,
        );
        button(
            r,
            "Restart",
            Some("R"),
            a(PlayAction::Restart),
            ButtonKind::Secondary,
        );
        button(
            r,
            c.table.insert.name(),
            Some("T"),
            a(PlayAction::NextTable),
            ButtonKind::Secondary,
        );
        button(
            r,
            "Save",
            Some("F5"),
            a(PlayAction::Save(1)),
            ButtonKind::Secondary,
        );
        button(
            r,
            "Load",
            Some("F9"),
            a(PlayAction::Load(1)),
            ButtonKind::Secondary,
        );
        button(
            r,
            "Build",
            Some("Esc"),
            a(PlayAction::Exit),
            ButtonKind::Secondary,
        );
    });
}

fn tags(p: &mut ChildSpawnerCommands, c: &Ctx) {
    if !c.bench.reactions {
        return;
    }
    for (n, s) in c.design.supports.iter().enumerate() {
        if s.is_some() {
            p.spawn((
                Node {
                    position_type: PositionType::Absolute,
                    ..default()
                },
                ReactionTag(n),
                Visibility::Hidden,
                children![text("", TextRole::Mono, Swatch::SceneText)],
            ));
        }
    }
}

/// Numbers that change every frame.
pub fn update_readouts(
    bench: Res<Bench>,
    clock: Res<SimClock>,
    sim: Option<Res<Simulation>>,
    camera: Query<(&Camera, &GlobalTransform), With<OrbitCamera>>,
    mut texts: Query<(&Readout, &mut Text)>,
    mut bars: Query<(&GraphBar, &mut Node), Without<ReactionTag>>,
    mut tags: Query<(&ReactionTag, &mut Node, &mut Visibility, &Children), Without<GraphBar>>,
    mut tag_text: Query<&mut Text, Without<Readout>>,
) {
    let Some(sim) = sim else { return };
    let Some(live) = sim.live() else { return };
    let values = bench.pick.map(|p| fields(p, &sim)).unwrap_or_default();
    let trace = bench.trace.as_ref();
    let set = |t: &mut Text, v: String| {
        if t.0 != v {
            t.0 = v;
        }
    };
    for (r, mut t) in &mut texts {
        match r {
            Readout::Clock => set(&mut t, format!("{:06.2}", live.time())),
            Readout::Speed => {
                let v = if clock.paused {
                    "s  paused".to_string()
                } else if (clock.speed - 1.0).abs() < 1e-3 {
                    "s  real time".into()
                } else {
                    format!("s  x{}", clock.speed)
                };
                set(&mut t, v);
            }
            Readout::Field(i) => set(&mut t, values.get(*i).cloned().unwrap_or_default()),
            Readout::GraphNow => {
                let v = trace
                    .and_then(|tr| Some(format!("{:.1} {}", tr.last()?, tr.probe.unit())))
                    .unwrap_or_default();
                set(&mut t, v);
            }
            Readout::GraphRange => {
                let v = trace
                    .map(|tr| {
                        let (lo, hi) = tr.range();
                        format!("  range {lo:.1} to {hi:.1}")
                    })
                    .unwrap_or_default();
                set(&mut t, v);
            }
        }
    }

    if let Some(tr) = trace {
        let cols = tr.columns(GRAPH_COLUMNS);
        let (lo, hi) = tr.range();
        let span = (hi - lo).max(1.0);
        for (bar, mut node) in &mut bars {
            let (top, h) = match cols.get(bar.0).copied().flatten() {
                Some(v) if v >= 0.0 => ((hi - v) / span, v / span),
                Some(v) => (hi / span, -v / span),
                None => (hi / span, 0.0),
            };
            let (top, h) = (
                Val::Percent(top as f32 * 100.0),
                Val::Percent(h as f32 * 100.0),
            );
            if node.top != top || node.height != h {
                node.top = top;
                node.height = h;
            }
        }
    }

    let Ok((cam, cam_tf)) = camera.single() else {
        return;
    };
    let reactions = live.reactions();
    for (tag, mut node, mut vis, children) in &mut tags {
        let Some((_, f)) = reactions.iter().find(|(n, _)| *n == tag.0) else {
            *vis = Visibility::Hidden;
            continue;
        };
        let p = from_core(live.sim().position(tag.0));
        match cam.world_to_viewport(cam_tf, p).ok() {
            Some(px) => {
                node.left = Val::Px(px.x + 10.0);
                node.top = Val::Px(px.y + 8.0);
                *vis = Visibility::Inherited;
            }
            None => *vis = Visibility::Hidden,
        }
        for c in children.iter() {
            if let Ok(mut t) = tag_text.get_mut(c) {
                let v = format!("R {:.1} kN", f.norm() / 1000.0);
                if t.0 != v {
                    t.0 = v;
                }
            }
        }
    }
}
