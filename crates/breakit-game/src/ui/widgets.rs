//! Building blocks the screens are made of. Each returns a bundle or
//! spawns a small subtree; none of them choose colours or sizes, which come
//! from the theme through [`Skin`] and [`Ink`].
//!
//! Text goes through [`plain`], which keeps only what the bundled font
//! draws.

use bevy::ecs::hierarchy::ChildSpawnerCommands;
use bevy::prelude::*;

use super::pointer::{BlocksPointer, Clickable, Disabled};
use super::theme::{Ink, Skin, Swatch, TextRole, Weight};
use crate::flow::UiAction;

/// A text node.
pub fn text(s: impl Into<String>, role: TextRole, color: Swatch) -> impl Bundle {
    (Text::new(plain(s.into())), Ink::new(role, color))
}

/// The bundled UI font covers Latin text and the few symbols other
/// crates' strings use (<=, >=, x, dashes, superscript 2). Anything else is
/// replaced so it never renders as a missing glyph.
pub fn plain(s: String) -> String {
    if s.is_ascii() {
        return s;
    }
    s.chars()
        .map(|c| {
            if c.is_ascii() || KNOWN.contains(&c) || c.is_alphabetic() {
                c
            } else {
                '?'
            }
        })
        .collect()
}

const KNOWN: [char; 9] = [
    '\u{2264}', '\u{2265}', '\u{00d7}', '\u{2013}', '\u{2014}', '\u{00b2}', '\u{00b7}', '\u{2192}',
    '\u{00b0}',
];

/// Inner margin of a [`panel`] whose node sets none.
pub const PANEL_PADDING: UiRect = UiRect {
    left: Val::Px(18.0),
    right: Val::Px(18.0),
    top: Val::Px(14.0),
    bottom: Val::Px(14.0),
};

/// Left plus right [`PANEL_PADDING`], px.
pub const PANEL_PADDING_X: f32 = 36.0;

/// A group of read-outs on a frosted backdrop (the blurred, darkened
/// scene, see [`super::frost`]) over the theme's panel ground. Keeps
/// clicks off the scene.
pub fn panel(mut node: Node) -> impl Bundle {
    if node.padding == UiRect::DEFAULT {
        node.padding = PANEL_PADDING;
    }
    (
        node,
        Skin::fill(Swatch::Panel),
        MaterialNode(super::frost::FROST_MATERIAL),
        BlocksPointer,
    )
}

/// Same as [`panel`] in the instrument's colours.
pub fn instrument(mut node: Node) -> impl Bundle {
    if node.padding == UiRect::DEFAULT {
        node.padding = PANEL_PADDING;
    }
    (
        node,
        Skin::fill(Swatch::Instrument),
        MaterialNode(super::frost::FROST_MATERIAL),
        BlocksPointer,
    )
}

/// The head of a sheet: a small accent kicker (ID, revision) over a
/// light title.
pub fn sheet_head(p: &mut ChildSpawnerCommands, kicker: &str, title: &str) {
    p.spawn(Node {
        flex_direction: FlexDirection::Column,
        row_gap: Val::Px(2.0),
        margin: UiRect::bottom(Val::Px(12.0)),
        ..default()
    })
    .with_children(|h| {
        h.spawn(caps(kicker, Swatch::Accent));
        h.spawn(text(title, TextRole::Title, Swatch::Ink));
    });
}

/// Small tracked capitals: section labels, key hints.
pub fn caps(s: &str, color: Swatch) -> impl Bundle {
    (
        text(s.to_uppercase(), TextRole::Label, color),
        bevy::text::LetterSpacing::Px(1.4),
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

/// A title line in a panel: heading on the left, a quiet ID or value on
/// the right.
pub fn title_strip(p: &mut ChildSpawnerCommands, left: &str, right: &str, bg: Swatch, fg: Swatch) {
    let muted = if fg == Swatch::Ink { Swatch::Muted } else { fg };
    p.spawn((
        Node {
            justify_content: JustifyContent::SpaceBetween,
            align_items: AlignItems::Baseline,
            padding: UiRect::new(Val::Px(0.0), Val::Px(0.0), Val::Px(2.0), Val::Px(8.0)),
            column_gap: Val::Px(12.0),
            ..default()
        },
        Skin::fill(if bg == Swatch::Panel {
            Swatch::Clear
        } else {
            bg
        })
        .square(),
    ))
    .with_children(|s| {
        s.spawn(text(left, TextRole::Heading, fg));
        s.spawn(caps(right, muted));
    });
}

/// A horizontal hairline.
pub fn rule(p: &mut ChildSpawnerCommands, color: Swatch) {
    p.spawn((
        Node {
            height: Val::Px(1.0),
            width: Val::Percent(100.0),
            margin: UiRect::vertical(Val::Px(6.0)),
            ..default()
        },
        Skin::fill(if color == Swatch::Ink {
            Swatch::Rule
        } else {
            color
        })
        .square(),
    ));
}

/// A label / value row.
pub fn kv(p: &mut ChildSpawnerCommands, label: &str, value: impl Bundle) {
    p.spawn(Node {
        justify_content: JustifyContent::SpaceBetween,
        align_items: AlignItems::Center,
        padding: UiRect::vertical(Val::Px(3.0)),
        column_gap: Val::Px(16.0),
        ..default()
    })
    .with_children(|r| {
        r.spawn(text(label, TextRole::Body, Swatch::Muted));
        r.spawn(value);
    });
}

/// A key hint: `SPACE`, `F1`. Quiet text, no cap.
pub fn key_cap(p: &mut ChildSpawnerCommands, key: &str) {
    p.spawn(caps(key, Swatch::Muted));
}

/// How a button looks.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ButtonKind {
    /// The one primary action on screen: accent pill.
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
    let (skin, fg) = match kind {
        ButtonKind::Primary => (Skin::fill(Swatch::Accent).pill(), Swatch::OnAccent),
        ButtonKind::Secondary => (Skin::fill(Swatch::Raised), Swatch::Ink),
        ButtonKind::Selected => (
            Skin::boxed(Swatch::Raised, Swatch::Accent, Weight::Thin),
            Swatch::Ink,
        ),
        ButtonKind::Disabled => (Skin::fill(Swatch::Clear), Swatch::Muted),
    };
    let mut e = p.spawn((
        Node {
            padding: UiRect::axes(Val::Px(14.0), Val::Px(7.0)),
            column_gap: Val::Px(12.0),
            align_items: AlignItems::Center,
            justify_content: JustifyContent::SpaceBetween,
            min_height: Val::Px(32.0),
            ..default()
        },
        skin,
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
            let c = if kind == ButtonKind::Primary {
                Swatch::OnAccent
            } else {
                Swatch::Muted
            };
            b.spawn(caps(k, c));
        }
    });
    e.id()
}

/// A small dot used as a status read-out.
pub fn status_box(p: &mut ChildSpawnerCommands, state: Status) {
    let c = match state {
        Status::Ok => Swatch::Ok,
        Status::Pending => Swatch::Rule,
        Status::Running => Swatch::Accent,
        Status::Failed => Swatch::Fail,
    };
    p.spawn((
        Node {
            width: Val::Px(6.0),
            height: Val::Px(6.0),
            flex_shrink: 0.0,
            margin: UiRect::right(Val::Px(4.0)),
            ..default()
        },
        Skin::fill(c).pill(),
    ));
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Status {
    Ok,
    Pending,
    Running,
    Failed,
}

/// A criterion row: status dot, label, value on the right.
pub fn check_row(p: &mut ChildSpawnerCommands, status: Status, label: &str, value: &str) {
    let (lc, vc) = if status == Status::Failed {
        (Swatch::Fail, Swatch::Fail)
    } else {
        (Swatch::Ink, Swatch::Muted)
    };
    p.spawn(Node {
        align_items: AlignItems::Center,
        padding: UiRect::vertical(Val::Px(3.0)),
        column_gap: Val::Px(8.0),
        ..default()
    })
    .with_children(|r| {
        status_box(r, status);
        r.spawn((
            text(label, TextRole::Body, lc),
            Node {
                flex_grow: 1.0,
                flex_shrink: 1.0,
                ..default()
            },
        ));
        r.spawn(text(value, TextRole::Body, vc));
    });
}

/// A 2 px bar at `fraction` of its width, with an optional tick.
pub fn meter(p: &mut ChildSpawnerCommands, fraction: f32, fill: Swatch, marker: Option<f32>) {
    p.spawn((
        Node {
            height: Val::Px(2.0),
            margin: UiRect::vertical(Val::Px(8.0)),
            ..default()
        },
        Skin::fill(Swatch::Rule).square(),
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
                    width: Val::Px(1.0),
                    ..default()
                },
                Skin::fill(Swatch::Accent).square(),
            ));
        }
    });
}
