//! Mouse over the UI: which button is under the cursor, clicks on it, and
//! whether the cursor is over a panel (so the scene ignores the click).
//!
//! A small hit test over laid-out nodes, instead of Bevy's `Button` /
//! `Interaction`, which 0.20 deprecates in favour of the picking-based
//! widgets this crate doesn't enable.

use bevy::prelude::*;
use bevy::window::PrimaryWindow;

use crate::flow::UiAction;

/// A node that sends an action when clicked.
#[derive(Component, Clone, Copy, Debug)]
pub struct Clickable(pub UiAction);

/// Shown but not clickable right now (greyed by its screen).
#[derive(Component, Clone, Copy, Debug)]
pub struct Disabled;

/// A panel that keeps clicks away from the scene.
#[derive(Component, Clone, Copy, Debug, Default)]
pub struct BlocksPointer;

#[derive(Resource, Debug, Default)]
pub struct Pointer {
    /// Cursor is over a panel or a button.
    pub over_ui: bool,
    pub hovered: Option<Entity>,
}

/// Hit-tests panels and buttons, sends the clicked button's action, and
/// outlines the hovered button.
#[allow(clippy::type_complexity)]
pub fn pointer_system(
    window: Single<&Window, With<PrimaryWindow>>,
    buttons: Res<ButtonInput<MouseButton>>,
    themes: Res<super::theme::Themes>,
    nodes: Query<(
        Entity,
        &ComputedNode,
        &UiGlobalTransform,
        &InheritedVisibility,
        Option<&Clickable>,
        Has<BlocksPointer>,
        Has<Disabled>,
    )>,
    mut outlines: Query<&mut Outline, With<Clickable>>,
    mut pointer: ResMut<Pointer>,
    mut out: MessageWriter<UiAction>,
) {
    let cursor = window.physical_cursor_position();
    let mut over = false;
    let mut best: Option<(Entity, f32, UiAction, bool)> = None;
    if let Some(p) = cursor {
        for (e, node, tf, vis, click, blocks, disabled) in &nodes {
            if !vis.get() || node.size().cmple(Vec2::ZERO).any() {
                continue;
            }
            if !(click.is_some() || blocks) || !node.contains_point(*tf, p) {
                continue;
            }
            over = true;
            if let Some(c) = click {
                // Innermost (smallest) clickable wins.
                let area = node.size().x * node.size().y;
                if best.is_none_or(|b| area < b.1) {
                    best = Some((e, area, c.0, disabled));
                }
            }
        }
    }
    pointer.over_ui = over;
    let hovered = best.filter(|b| !b.3).map(|b| b.0);
    if pointer.hovered != hovered {
        let accent = themes.current().accent;
        for (e, color) in [(pointer.hovered, Color::NONE), (hovered, accent)] {
            if let Some(mut o) = e.and_then(|e| outlines.get_mut(e).ok()) {
                o.color = color;
                o.width = Val::Px(2.0);
            }
        }
        pointer.hovered = hovered;
    }
    if buttons.just_pressed(MouseButton::Left) {
        if let Some((_, _, action, false)) = best {
            out.write(action);
        }
    }
}
