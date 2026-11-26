//! Building blocks the screens are made of. Each returns a bundle or
//! spawns a small subtree; none of them choose colours or sizes, which come
//! from the theme through [`Skin`] and [`Ink`].
//!
//! Text is plain ASCII on purpose: Bevy's built-in font is a subset, and
//! symbols like check marks or arrows may be missing from it.

use bevy::ecs::hierarchy::ChildSpawnerCommands;
use bevy::prelude::*;

use super::pointer::{BlocksPointer, Clickable, Disabled};
use super::theme::{Ink, Skin, Swatch, TextRole, Weight};
use crate::flow::UiAction;

/// A text node.
pub fn text(s: impl Into<String>, role: TextRole, color: Swatch) -> impl Bundle {
    (Text::new(plain(s.into())), Ink::new(role, color))
}

/// The built-in font only has ASCII: spell out the symbols other crates'
/// text uses (report rows, limit sentences).
pub fn plain(s: String) -> String {
    if s.is_ascii() {
        return s;
    }
    s.replace('\u{2264}', "<=")
        .replace('\u{2265}', ">=")
        .replace('\u{00d7}', "x")
        .replace(['\u{2013}', '\u{2014}'], "-")
        .replace('\u{00b2}', "2")
        .chars()
        .map(|c| if c.is_ascii() { c } else { '?' })
        .collect()
}

/// A panel: theme panel ground, strong ink border, keeps clicks off the
/// scene.
pub fn panel(node: Node) -> impl Bundle {
    (
        node,
        Skin::boxed(Swatch::Panel, Swatch::Ink, Weight::Strong),
        BlocksPointer,
    )
}

/// Same as [`panel`] in the instrument's colours.
pub fn instrument(node: Node) -> impl Bundle {
    (
        node,
        Skin::boxed(Swatch::Instrument, Swatch::Ink, Weight::Strong),
        BlocksPointer,
    )
}

pub fn column(gap: f32) -> Node {
    Node {
        flex_direction: FlexDirection::Column,
        row_gap: Val::Px(gap),
        ..default()
    }
}

pub fn row(gap: f32) -> Node {
    Node {
        flex_direction: FlexDirection::Row,
        column_gap: Val::Px(gap),
        align_items: AlignItems::Center,
        ..default()
    }
}

/// A full-screen layer for a screen or an overlay.
pub fn layer() -> Node {
    Node {
        position_type: PositionType::Absolute,
        left: Val::Px(0.0),
        right: Val::Px(0.0),
        top: Val::Px(0.0),
        bottom: Val::Px(0.0),
        ..default()
    }
}

/// Absolutely placed node, offsets in px; `None` leaves a side free.
pub fn at(left: Option<f32>, top: Option<f32>, right: Option<f32>, bottom: Option<f32>) -> Node {
    let v = |o: Option<f32>| o.map(Val::Px).unwrap_or(Val::Auto);
    Node {
        position_type: PositionType::Absolute,
        left: v(left),
        top: v(top),
        right: v(right),
        bottom: v(bottom),
        flex_direction: FlexDirection::Column,
        ..default()
    }
}

/// A title strip across the top of a panel: heading on the left, an ID
/// or value on the right.
pub fn title_strip(p: &mut ChildSpawnerCommands, left: &str, right: &str, bg: Swatch, fg: Swatch) {
    p.spawn((
        Node {
            justify_content: JustifyContent::SpaceBetween,
            align_items: AlignItems::Center,
            padding: UiRect::axes(Val::Px(10.0), Val::Px(6.0)),
            column_gap: Val::Px(12.0),
            ..default()
        },
        Skin::fill(bg).square(),
    ))
    .with_children(|s| {
        s.spawn(text(left, TextRole::Heading, fg));
        s.spawn(text(right, TextRole::Mono, fg));
    });
    rule(p, Swatch::Ink);
}

/// A horizontal rule.
pub fn rule(p: &mut ChildSpawnerCommands, color: Swatch) {
    p.spawn((
        Node {
            height: Val::Px(1.0),
            width: Val::Percent(100.0),
            ..default()
        },
        Skin::fill(color).square(),
    ));
}

/// A label / value row with a rule above it.
pub fn kv(p: &mut ChildSpawnerCommands, label: &str, value: impl Bundle) {
    rule(p, Swatch::Rule);
    p.spawn(Node {
        justify_content: JustifyContent::SpaceBetween,
        align_items: AlignItems::Center,
        padding: UiRect::axes(Val::Px(10.0), Val::Px(4.0)),
        column_gap: Val::Px(12.0),
        ..default()
    })
    .with_children(|r| {
        r.spawn(text(label, TextRole::Body, Swatch::Ink));
        r.spawn(value);
    });
}

/// A key cap: `Space`, `F1`.
pub fn key_cap(p: &mut ChildSpawnerCommands, key: &str) {
    p.spawn((
        Node {
            padding: UiRect::axes(Val::Px(5.0), Val::Px(1.0)),
            ..default()
        },
        Skin::boxed(Swatch::Raised, Swatch::Ink, Weight::Thin),
    ))
    .with_child(text(key, TextRole::Mono, Swatch::Ink));
}

/// How a button looks.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ButtonKind {
    /// The one primary action on screen: accent fill.
    Primary,
    Secondary,
    /// Selected state of a toggle or tab.
    Selected,
    Disabled,
}

/// A button with a label and its key.
pub fn button(
    p: &mut ChildSpawnerCommands,
    label: &str,
    key: Option<&str>,
    action: UiAction,
    kind: ButtonKind,
) -> Entity {
    let (bg, fg, w) = match kind {
        ButtonKind::Primary => (Swatch::Accent, Swatch::OnAccent, Weight::Strong),
        ButtonKind::Secondary => (Swatch::Raised, Swatch::Ink, Weight::Thin),
        ButtonKind::Selected => (Swatch::Ink, Swatch::Panel, Weight::Thin),
        ButtonKind::Disabled => (Swatch::Panel, Swatch::Muted, Weight::Thin),
    };
    let border = if kind == ButtonKind::Disabled {
        Swatch::Rule
    } else {
        Swatch::Ink
    };
    let mut e = p.spawn((
        Node {
            padding: UiRect::axes(Val::Px(12.0), Val::Px(7.0)),
            column_gap: Val::Px(10.0),
            align_items: AlignItems::Center,
            justify_content: JustifyContent::SpaceBetween,
            min_height: Val::Px(34.0),
            ..default()
        },
        Skin::boxed(bg, border, w),
        Clickable(action),
        Outline::new(Val::Px(2.0), Val::Px(1.0), Color::NONE),
    ));
    if kind == ButtonKind::Disabled {
        e.insert(Disabled);
    }
    e.with_children(|b| {
        let role = if kind == ButtonKind::Primary {
            TextRole::Heading
        } else {
            TextRole::Body
        };
        b.spawn(text(label, role, fg));
        if let Some(k) = key {
            key_cap(b, k);
        }
    });
    e.id()
}

/// A small filled or empty box used as a checkbox read-out.
pub fn status_box(p: &mut ChildSpawnerCommands, state: Status) {
    let skin = match state {
        Status::Ok => Skin::fill(Swatch::Ok).square(),
        Status::Pending => Skin::boxed(Swatch::Clear, Swatch::Ink, Weight::Thin).square(),
        Status::Running => Skin::fill(Swatch::Accent).square(),
        Status::Failed => Skin::fill(Swatch::Fail).square(),
    };
    p.spawn((
        Node {
            width: Val::Px(10.0),
            height: Val::Px(10.0),
            flex_shrink: 0.0,
            ..default()
        },
        skin,
    ));
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Status {
    Ok,
    Pending,
    Running,
    Failed,
}

/// A criterion row: status box, label, value on the right.
pub fn check_row(p: &mut ChildSpawnerCommands, status: Status, label: &str, value: &str) {
    rule(p, Swatch::Rule);
    let (lc, vc) = if status == Status::Failed {
        (Swatch::Fail, Swatch::Fail)
    } else {
        (Swatch::Ink, Swatch::Muted)
    };
    p.spawn(Node {
        align_items: AlignItems::Center,
        padding: UiRect::axes(Val::Px(10.0), Val::Px(4.0)),
        column_gap: Val::Px(8.0),
        ..default()
    })
    .with_children(|r| {
        status_box(r, status);
        r.spawn((
            text(label, TextRole::Body, lc),
            Node {
                flex_grow: 1.0,
                ..default()
            },
        ));
        r.spawn(text(value, TextRole::Mono, vc));
    });
}

/// The utilisation legend: four bands with their limits.
pub fn ramp_legend(p: &mut ChildSpawnerCommands, width: f32) {
    p.spawn(column(2.0)).with_children(|c| {
        c.spawn(Node {
            width: Val::Px(width),
            height: Val::Px(8.0),
            ..default()
        })
        .with_children(|bar| {
            for (i, grow) in [(0u8, 50.0), (1, 30.0), (2, 20.0), (3, 22.0)] {
                bar.spawn((
                    Node {
                        flex_grow: grow,
                        ..default()
                    },
                    Skin::fill(Swatch::Ramp(i)).square(),
                ));
            }
        });
        c.spawn(Node {
            width: Val::Px(width),
            ..default()
        })
        .with_children(|l| {
            for (label, grow) in [("0", 50.0), ("50", 30.0), ("80", 20.0), ("100%", 22.0)] {
                l.spawn((
                    text(label, TextRole::Label, Swatch::SceneText),
                    Node {
                        flex_grow: grow,
                        flex_basis: Val::Px(0.0),
                        ..default()
                    },
                ));
            }
        });
    });
}

/// Fills `p` with a horizontal bar at `fraction` of its width.
pub fn meter(p: &mut ChildSpawnerCommands, fraction: f32, fill: Swatch, marker: Option<f32>) {
    p.spawn((
        Node {
            height: Val::Px(10.0),
            margin: UiRect::axes(Val::Px(10.0), Val::Px(4.0)),
            ..default()
        },
        Skin::boxed(Swatch::Raised, Swatch::Ink, Weight::Strong).square(),
    ))
    .with_children(|m| {
        m.spawn((
            Node {
                width: Val::Percent(fraction.clamp(0.0, 1.0) * 100.0),
                height: Val::Percent(100.0),
                ..default()
            },
            Skin::fill(fill).square(),
        ));
        if let Some(x) = marker {
            m.spawn((
                Node {
                    position_type: PositionType::Absolute,
                    left: Val::Percent(x.clamp(0.0, 1.0) * 100.0),
                    top: Val::Px(-4.0),
                    bottom: Val::Px(-4.0),
                    width: Val::Px(2.0),
                    ..default()
                },
                Skin::fill(Swatch::Muted).square(),
            ));
        }
    });
}
