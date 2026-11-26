//! The game UI: screens, HUDs and overlays.
//!
//! Layout and behaviour live here; the look lives in [`theme`]. Each panel
//! is a [`View`]: a root node whose children are rebuilt from game state
//! whenever the view's key (a short summary of what it shows) changes.
//! Fast-changing read-outs (test time, chart bars, world tags) update in
//! place instead, see [`hud`].

pub mod hud;
pub mod pointer;
pub mod screens;
pub mod theme;
pub mod widgets;

use bevy::ecs::hierarchy::ChildSpawnerCommands;
use bevy::ecs::system::SystemParam;
use bevy::prelude::*;

use crate::analysis::{DeflectionScale, FrameState, SimClock};
use crate::build::BuildTool;
use crate::design::Design;
use crate::flow::{FailurePins, LastTest, Overlay, Revisions, Screen};
use crate::history::History;
use crate::scenario::{Sheets, TestRecord};
use crate::testrun::{Heatmap, SlowMo};

pub struct GameUiPlugin;

impl Plugin for GameUiPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<theme::Themes>()
            .init_resource::<pointer::Pointer>()
            .add_systems(
                PreUpdate,
                pointer::pointer_system.after(crate::flow::keyboard_actions),
            )
            .add_systems(Startup, (theme::load_fonts, spawn_overlay_layer))
            .add_systems(OnEnter(Screen::Register), screens::spawn_register)
            .add_systems(OnEnter(Screen::Brief), screens::spawn_brief)
            .add_systems(OnEnter(Screen::Build), hud::spawn_build_hud)
            .add_systems(OnEnter(Screen::Test), hud::spawn_test_hud)
            .add_systems(
                OnEnter(Screen::Report),
                screens::spawn_report.after(crate::flow::finish_test),
            )
            .add_systems(
                PostUpdate,
                (
                    refresh_views,
                    hud::update_test_readouts,
                    hud::place_world_tags,
                    theme::apply_theme,
                )
                    .chain()
                    .before(bevy::ui::UiSystems::Prepare),
            );
    }
}

/// Everything the UI reads, in one parameter.
#[derive(SystemParam)]
pub struct GameView<'w> {
    pub design: Res<'w, Design>,
    pub state: Res<'w, FrameState>,
    pub sheets: Res<'w, Sheets>,
    pub revisions: Res<'w, Revisions>,
    pub tool: Res<'w, BuildTool>,
    pub last: Res<'w, LastTest>,
    pub pins: Res<'w, FailurePins>,
    pub record: Res<'w, TestRecord>,
    pub clock: Res<'w, SimClock>,
    pub slowmo: Res<'w, SlowMo>,
    pub heat: Res<'w, Heatmap>,
    pub history: Res<'w, History>,
    pub overlay: Res<'w, Overlay>,
    pub scale: Res<'w, DeflectionScale>,
    pub theme: Res<'w, theme::Themes>,
    pub check: Res<'w, crate::scenario::ScenarioCheck>,
}

impl GameView<'_> {
    /// The design's cost from the scenario cost model.
    pub fn cost(&self) -> f32 {
        self.check.bill.total() as f32
    }

    /// The design's mass, kg.
    pub fn mass(&self) -> f32 {
        self.check.bill.mass as f32
    }
}

/// A panel rebuilt from game state.
#[derive(Component, Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum View {
    Register,
    Brief,
    Report,
    Overlay,
    BuildSheet,
    BuildReadouts,
    BuildTools,
    BuildInspector,
    BuildDrawer,
    BuildFooter,
    BuildTags,
    TestSheet,
    TestLog,
    TestBanner,
    TestControls,
    TestTags,
}

/// Key the view was last built for.
#[derive(Component, Default)]
pub struct ViewKey(Option<String>);

impl View {
    fn key(self, v: &GameView) -> String {
        match self {
            View::Register => screens::register_key(v),
            View::Brief => format!("{}", v.sheets.current),
            View::Report => format!(
                "{} {}",
                v.last.letter,
                v.revisions.of(v.sheets.current().id).len()
            ),
            View::Overlay => format!("{:?} {}", *v.overlay, v.sheets.current),
            View::BuildSheet => hud::build_sheet_key(v),
            View::BuildReadouts => hud::readouts_key(v),
            View::BuildTools => format!(
                "{:?} {:?} {} {} {}",
                v.tool.tool,
                v.tool.plane,
                v.tool.loads_allowed,
                v.history.can_undo(),
                v.history.can_redo()
            ),
            View::BuildInspector => hud::inspector_key(v),
            View::BuildDrawer => format!(
                "{:?} {:?} {:?} {}",
                v.tool.kind, v.tool.allowed, v.tool.selected_beam, v.design.revision
            ),
            View::BuildFooter => format!(
                "{:?} {:?} {} {} {}",
                v.tool.tool,
                v.tool.plane,
                v.tool.level,
                v.scale.0,
                v.revisions.of(v.sheets.current().id).len()
            ),
            View::BuildTags => hud::build_tags_key(v),
            View::TestSheet => hud::test_sheet_key(v),
            View::TestLog => format!("{}", v.record.events.len()),
            View::TestBanner => format!("{} {}", v.slowmo.active(), v.clock.paused),
            View::TestControls => format!("{} {} {}", v.clock.speed, v.clock.paused, v.heat.0),
            View::TestTags => format!("{:?} {}", v.record.first_break, v.slowmo.active()),
        }
    }

    fn build(self, p: &mut ChildSpawnerCommands, v: &GameView) {
        match self {
            View::Register => screens::build_register(p, v),
            View::Brief => screens::build_brief(p, v, false),
            View::Report => screens::build_report(p, v),
            View::Overlay => screens::build_overlay(p, v),
            View::BuildSheet => hud::build_sheet(p, v),
            View::BuildReadouts => hud::build_readouts(p, v),
            View::BuildTools => hud::build_tools(p, v),
            View::BuildInspector => hud::build_inspector(p, v),
            View::BuildDrawer => hud::build_drawer(p, v),
            View::BuildFooter => hud::build_footer(p, v),
            View::BuildTags => hud::build_tags(p, v),
            View::TestSheet => hud::test_sheet(p, v),
            View::TestLog => hud::test_log(p, v),
            View::TestBanner => hud::test_banner(p, v),
            View::TestControls => hud::test_controls(p, v),
            View::TestTags => hud::test_tags(p, v),
        }
    }
}

/// Rebuilds every view whose key changed.
fn refresh_views(
    mut commands: Commands,
    view: GameView,
    mut roots: Query<(Entity, &View, &mut ViewKey)>,
) {
    for (e, kind, mut key) in &mut roots {
        let k = kind.key(&view);
        if key.0.as_ref() == Some(&k) {
            continue;
        }
        key.0 = Some(k);
        let mut ec = commands.entity(e);
        ec.despawn_children();
        ec.with_children(|p| kind.build(p, &view));
    }
}

/// The overlay layer lives for the whole game, above every screen.
fn spawn_overlay_layer(mut commands: Commands) {
    // Soft gradients along the screen edges, under every screen's UI.
    for (edge, h) in [
        (theme::EdgeScrim::Top, 260.0),
        (theme::EdgeScrim::Bottom, 280.0),
    ] {
        commands.spawn((edge.node(h), edge, GlobalZIndex(-10), Pickable::IGNORE));
    }
    commands.spawn((
        widgets::layer(),
        GlobalZIndex(10),
        View::Overlay,
        ViewKey::default(),
    ));
}
