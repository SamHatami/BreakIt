//! The player loop: register, brief, build, test, report, and back.
//!
//! [`Screen`] is the one state the game is in; [`Mode`] (which physics
//! runs) is computed from it. Everything the player can do, by key or by
//! click, arrives as a [`UiAction`] message, so the input map lives in one
//! place ([`keyboard_actions`]) and buttons just emit the same actions.

use bevy::prelude::*;

use crate::build::{BuildTool, Tool};
use crate::design::{BeamKind, Design};
use crate::history::History;
use crate::scenario::{Check, Report, Rig, ScenarioCheck, Sheets, TestOutcome, TestRecord};
use crate::Mode;

#[derive(States, Default, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Screen {
    /// Pick a sheet.
    #[default]
    Register,
    Brief,
    Build,
    /// The test run.
    Test,
    /// The test report, over the frozen scene.
    Report,
}

impl ComputedStates for Mode {
    type SourceStates = Screen;

    fn compute(s: Screen) -> Option<Self> {
        Some(match s {
            Screen::Test | Screen::Report => Mode::Simulate,
            _ => Mode::Build,
        })
    }
}

/// A sheet laid over the current screen.
#[derive(Resource, Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Overlay {
    #[default]
    None,
    Pause,
    Brief,
    Controls,
}

/// Everything the player can ask for.
#[derive(Message, Clone, Copy, Debug, PartialEq)]
pub enum UiAction {
    SelectSheet(usize),
    SelectRig(Rig),
    SheetUp,
    SheetDown,
    OpenBrief,
    StartBuild,
    BackToRegister,
    RunTest,
    AbortTest,
    Revise,
    Replay,
    NextSheet,
    /// Esc: end the chain, deselect, or open the pause sheet.
    Back,
    ToggleBrief,
    ToggleControls,
    CloseOverlay,
    SetTool(Tool),
    SetMaterial(BeamKind),
    DeleteSelected,
    Undo,
    Redo,
    Clear,
    FrameView,
    CyclePlane,
    PlaneUp,
    PlaneDown,
    ToggleDeflection,
    SimPause,
    SimSlower,
    SimFaster,
    ToggleHeatmap,
    CycleTheme,
}

/// One tested revision of a sheet.
#[derive(Clone, Debug, PartialEq)]
pub struct Revision {
    pub letter: char,
    pub passed: bool,
    pub held: f32,
    pub cost: f32,
}

/// Every test result, per sheet id.
#[derive(Resource, Default, Debug)]
pub struct Revisions(pub Vec<(&'static str, Vec<Revision>)>);

impl Revisions {
    pub fn of(&self, id: &str) -> &[Revision] {
        self.0
            .iter()
            .find(|(k, _)| *k == id)
            .map(|(_, v)| v.as_slice())
            .unwrap_or(&[])
    }

    pub fn push(&mut self, id: &'static str, r: Revision) {
        match self.0.iter_mut().find(|(k, _)| *k == id) {
            Some((_, v)) => v.push(r),
            None => self.0.push((id, vec![r])),
        }
    }

    pub fn passed(&self, id: &str) -> bool {
        self.of(id).iter().any(|r| r.passed)
    }

    /// Letter the next test of this sheet will get.
    pub fn next_letter(&self, id: &str) -> char {
        letter(self.of(id).len())
    }
}

pub fn letter(i: usize) -> char {
    (b'A' + (i % 26) as u8) as char
}

/// Whether a sheet can be opened: the first of each rig and the sandbox
/// always; the others once the sheet before them passed.
pub fn unlocked(sheets: &Sheets, revisions: &Revisions, i: usize) -> bool {
    let s = &sheets.all[i];
    if s.sandbox {
        return true;
    }
    let prev = sheets.all[..i].iter().rev().find(|p| p.rig == s.rig);
    match prev {
        None => true,
        Some(p) => revisions.passed(p.id),
    }
}

/// Result of the last finished test, shown by the report and pinned in
/// build mode after Revise.
#[derive(Resource, Default, Debug)]
pub struct LastTest {
    pub record: TestRecord,
    /// The scenario run's graded report.
    pub report: Option<Report>,
    /// The report's criterion rows.
    pub checks: Vec<Check>,
    pub passed: bool,
    pub letter: char,
}

/// Failed members shown in build mode after Revise, until the topology
/// changes.
#[derive(Resource, Default, Debug)]
pub struct FailurePins {
    pub pins: Vec<(usize, String)>,
    pub beam_count: usize,
}

pub struct FlowPlugin;

impl Plugin for FlowPlugin {
    fn build(&self, app: &mut App) {
        app.init_state::<Screen>()
            .add_computed_state::<Mode>()
            .add_message::<UiAction>()
            .init_resource::<Overlay>()
            .init_resource::<Sheets>()
            .init_resource::<Revisions>()
            .init_resource::<LastTest>()
            .init_resource::<FailurePins>()
            .init_resource::<ScenarioCheck>()
            .add_systems(Startup, load_first_sheet)
            .add_systems(Update, (update_check, sync_table))
            .add_systems(PreUpdate, keyboard_actions)
            .add_systems(Update, apply_flow_actions)
            .add_systems(OnEnter(Screen::Report), finish_test)
            .add_systems(OnExit(Screen::Build), close_overlay)
            .add_systems(OnEnter(Screen::Build), close_overlay);
    }
}

fn load_first_sheet(
    sheets: Res<Sheets>,
    mut design: ResMut<Design>,
    mut history: ResMut<History>,
    mut tool: ResMut<BuildTool>,
) {
    open_sheet(&sheets, &mut design, &mut history, &mut tool);
}

/// Replaces the design with the current sheet's starting design.
pub fn open_sheet(
    sheets: &Sheets,
    design: &mut Design,
    history: &mut History,
    tool: &mut BuildTool,
) {
    let mut d = sheets.current().initial_design();
    d.revision = design.revision + 1;
    *design = d;
    history.reset(design);
    tool.reset_for(sheets.current());
}

/// Keeps the pre-test check (bill and blocking limits) current.
fn update_check(design: Res<Design>, sheets: Res<Sheets>, mut check: ResMut<ScenarioCheck>) {
    let key = (design.revision, sheets.current);
    if check.key == Some(key) {
        return;
    }
    let (bill, violations) = ScenarioCheck::compute(sheets.current(), &design, &sheets.costs);
    *check = ScenarioCheck {
        bill,
        violations,
        key: Some(key),
    };
}

/// Puts the sheet's play table in the lab.
fn sync_table(
    sheets: Res<Sheets>,
    mut table: ResMut<breakit_lab::PlayTable>,
    mut shown: Local<Option<Rig>>,
) {
    let rig = sheets.current().rig;
    if *shown != Some(rig) {
        *shown = Some(rig);
        *table = breakit_lab::PlayTable::builtin(rig.table());
    }
}

fn close_overlay(mut overlay: ResMut<Overlay>) {
    *overlay = Overlay::None;
}

/// The input map. Keys turn into the same actions the buttons send.
pub fn keyboard_actions(
    keys: Res<ButtonInput<KeyCode>>,
    screen: Res<State<Screen>>,
    overlay: Res<Overlay>,
    mut out: MessageWriter<UiAction>,
) {
    use KeyCode as K;
    let ctrl = keys.any_pressed([K::ControlLeft, K::ControlRight, K::SuperLeft, K::SuperRight]);
    let shift = keys.any_pressed([K::ShiftLeft, K::ShiftRight]);
    let mut send = |a| {
        out.write(a);
    };
    if keys.just_pressed(K::F10) {
        send(UiAction::CycleTheme);
    }
    if *overlay != Overlay::None {
        if keys.just_pressed(K::Escape) {
            send(UiAction::CloseOverlay);
        }
        if keys.just_pressed(K::F1) {
            send(UiAction::ToggleBrief);
        }
        if keys.just_pressed(K::Slash) {
            send(UiAction::ToggleControls);
        }
        if *overlay == Overlay::Pause && keys.just_pressed(K::KeyQ) {
            send(UiAction::BackToRegister);
        }
        return;
    }
    match screen.get() {
        Screen::Register => {
            if keys.just_pressed(K::ArrowUp) {
                send(UiAction::SheetUp);
            }
            if keys.just_pressed(K::ArrowDown) {
                send(UiAction::SheetDown);
            }
            if keys.any_just_pressed([K::Enter, K::NumpadEnter]) {
                send(UiAction::OpenBrief);
            }
        }
        Screen::Brief => {
            if keys.any_just_pressed([K::Enter, K::NumpadEnter, K::Space]) {
                send(UiAction::StartBuild);
            }
            if keys.just_pressed(K::Escape) {
                send(UiAction::BackToRegister);
            }
        }
        Screen::Build => {
            if ctrl && keys.just_pressed(K::KeyZ) {
                send(if shift {
                    UiAction::Redo
                } else {
                    UiAction::Undo
                });
                return;
            }
            if ctrl && keys.just_pressed(K::KeyY) {
                send(UiAction::Redo);
                return;
            }
            if ctrl {
                return;
            }
            let map = [
                (K::Space, UiAction::RunTest),
                (K::Escape, UiAction::Back),
                (K::F1, UiAction::ToggleBrief),
                (K::Slash, UiAction::ToggleControls),
                (K::KeyB, UiAction::SetTool(Tool::Draw)),
                (K::KeyS, UiAction::SetTool(Tool::Support)),
                (K::KeyX, UiAction::SetTool(Tool::Delete)),
                (K::KeyL, UiAction::SetTool(Tool::Load)),
                (K::Delete, UiAction::DeleteSelected),
                (K::Backspace, UiAction::DeleteSelected),
                (K::Digit1, UiAction::SetMaterial(BeamKind::ALL[0])),
                (K::Digit2, UiAction::SetMaterial(BeamKind::ALL[1])),
                (K::KeyF, UiAction::FrameView),
                (K::Tab, UiAction::CyclePlane),
                (K::KeyE, UiAction::PlaneUp),
                (K::KeyQ, UiAction::PlaneDown),
                (K::KeyD, UiAction::ToggleDeflection),
                (K::KeyC, UiAction::Clear),
            ];
            for (k, a) in map {
                if keys.just_pressed(k) {
                    send(a);
                }
            }
        }
        Screen::Test => {
            let map = [
                (K::Space, UiAction::SimPause),
                (K::Comma, UiAction::SimSlower),
                (K::Period, UiAction::SimFaster),
                (K::KeyH, UiAction::ToggleHeatmap),
                (K::Escape, UiAction::AbortTest),
                (K::F1, UiAction::ToggleBrief),
                (K::KeyF, UiAction::FrameView),
            ];
            for (k, a) in map {
                if keys.just_pressed(k) {
                    send(a);
                }
            }
        }
        Screen::Report => {
            let map = [
                (K::KeyR, UiAction::Revise),
                (K::KeyP, UiAction::Replay),
                (K::KeyN, UiAction::NextSheet),
                (K::Escape, UiAction::BackToRegister),
                (K::Enter, UiAction::Revise),
            ];
            for (k, a) in map {
                if keys.just_pressed(k) {
                    send(a);
                }
            }
        }
    }
}

/// Screen changes, overlays and sheet selection.
#[allow(clippy::too_many_arguments)]
fn apply_flow_actions(
    mut actions: MessageReader<UiAction>,
    screen: Res<State<Screen>>,
    mut next: ResMut<NextState<Screen>>,
    mut overlay: ResMut<Overlay>,
    mut sheets: ResMut<Sheets>,
    revisions: Res<Revisions>,
    last: Res<LastTest>,
    mut pins: ResMut<FailurePins>,
    mut design: ResMut<Design>,
    mut history: ResMut<History>,
    mut tool: ResMut<BuildTool>,
    mut themes: ResMut<crate::ui::theme::Themes>,
    check: Res<ScenarioCheck>,
) {
    let screen = *screen.get();
    for a in actions.read() {
        match (*a, screen) {
            (UiAction::CycleTheme, _) => themes.cycle(),
            (UiAction::SelectSheet(i), Screen::Register) => {
                if i < sheets.all.len() && i != sheets.current {
                    sheets.current = i;
                    open_sheet(&sheets, &mut design, &mut history, &mut tool);
                }
            }
            (UiAction::SelectRig(r), Screen::Register) => {
                if sheets.current().rig != r {
                    if let Some(i) = sheets.all.iter().position(|s| s.rig == r) {
                        sheets.current = i;
                        open_sheet(&sheets, &mut design, &mut history, &mut tool);
                    }
                }
            }
            (UiAction::SheetUp | UiAction::SheetDown, Screen::Register) => {
                let rig = sheets.current().rig;
                let same: Vec<usize> = (0..sheets.all.len())
                    .filter(|&i| sheets.all[i].rig == rig)
                    .collect();
                let pos = same.iter().position(|&i| i == sheets.current).unwrap_or(0);
                let to = if *a == UiAction::SheetUp {
                    pos.checked_sub(1)
                } else {
                    (pos + 1 < same.len()).then_some(pos + 1)
                };
                if let Some(p) = to {
                    sheets.current = same[p];
                    open_sheet(&sheets, &mut design, &mut history, &mut tool);
                }
            }
            (UiAction::OpenBrief, Screen::Register) => {
                if unlocked(&sheets, &revisions, sheets.current) {
                    next.set(if sheets.current().sandbox {
                        Screen::Build
                    } else {
                        Screen::Brief
                    });
                }
            }
            (UiAction::StartBuild, Screen::Brief) => next.set(Screen::Build),
            (UiAction::BackToRegister, _) => {
                *overlay = Overlay::None;
                next.set(Screen::Register);
            }
            (UiAction::RunTest, Screen::Build) => {
                // The build HUD lists what blocks the test.
                if !design.beams.is_empty() && !check.blocks_test() {
                    next.set(Screen::Test);
                }
            }
            (UiAction::AbortTest, Screen::Test) => next.set(Screen::Build),
            (UiAction::Revise, Screen::Report) => {
                // Pin what broke in the test, or after a pass what the
                // proof test broke first.
                let test_end = sheets.current().def.duration as f32;
                pins.pins = last
                    .record
                    .broken_members()
                    .into_iter()
                    .filter_map(|m| {
                        let t = last.record.events.iter().find_map(|e| match e.event {
                            crate::scenario::TestEvent::Broke { member, .. }
                                if member == m && (e.time > test_end) == last.passed =>
                            {
                                Some(e.time)
                            }
                            _ => None,
                        })?;
                        let when = if t > test_end { "proof" } else { "broke" };
                        Some((m, format!("{}: {when} {:.1} s", last.letter, t)))
                    })
                    .collect();
                pins.beam_count = design.beams.len();
                next.set(Screen::Build);
            }
            (UiAction::Replay, Screen::Report) => next.set(Screen::Test),
            (UiAction::NextSheet, Screen::Report) => {
                let i = sheets.current;
                let rig = sheets.current().rig;
                if last.passed {
                    if let Some(n) = (i + 1..sheets.all.len()).find(|&n| sheets.all[n].rig == rig) {
                        sheets.current = n;
                        open_sheet(&sheets, &mut design, &mut history, &mut tool);
                        pins.pins.clear();
                        next.set(Screen::Brief);
                    }
                }
            }
            (UiAction::Back, Screen::Build) => {
                if !tool.deselect() {
                    *overlay = Overlay::Pause;
                }
            }
            (UiAction::ToggleBrief, Screen::Build | Screen::Test) => {
                if !sheets.current().sandbox {
                    *overlay = toggle(*overlay, Overlay::Brief);
                }
            }
            (UiAction::ToggleControls, Screen::Build) => {
                *overlay = toggle(*overlay, Overlay::Controls);
            }
            (UiAction::CloseOverlay, _) => *overlay = Overlay::None,
            _ => {}
        }
    }
}

fn toggle(cur: Overlay, to: Overlay) -> Overlay {
    if cur == to {
        Overlay::None
    } else {
        to
    }
}

/// Judges the run that just ended and files it as a revision.
pub fn finish_test(
    sheets: Res<Sheets>,
    record: Res<TestRecord>,
    outcome: Res<TestOutcome>,
    mut revisions: ResMut<Revisions>,
    mut last: ResMut<LastTest>,
) {
    let s = sheets.current();
    let report = outcome.0.clone();
    let ok = report.as_ref().is_some_and(|r| r.passed);
    if let Some(r) = &report {
        info!("test report\n{}", r.summary());
    }
    let letter = revisions.next_letter(s.id);
    revisions.push(
        s.id,
        Revision {
            letter,
            passed: ok,
            held: record.first_break.unwrap_or(s.def.duration as f32),
            cost: record.cost,
        },
    );
    *last = LastTest {
        record: record.clone(),
        checks: report
            .as_ref()
            .map(|r| r.criteria.clone())
            .unwrap_or_default(),
        report,
        passed: ok,
        letter,
    };
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn later_sheets_unlock_when_the_one_before_passes() {
        let mut sheets = Sheets::default();
        // A second bridge sheet after the first.
        let mut second = sheets.all[0].clone();
        second.id = "B-02";
        sheets.all.insert(1, second);
        let mut revs = Revisions::default();
        assert!(unlocked(&sheets, &revs, 0));
        assert!(!unlocked(&sheets, &revs, 1));
        let tower = sheets
            .all
            .iter()
            .position(|s| s.rig == Rig::Towers)
            .unwrap();
        assert!(unlocked(&sheets, &revs, tower));
        let sandbox = sheets.all.iter().position(|s| s.sandbox).unwrap();
        assert!(unlocked(&sheets, &revs, sandbox));
        revs.push(
            sheets.all[0].id,
            Revision {
                letter: 'A',
                passed: true,
                held: 7.0,
                cost: 1.0,
            },
        );
        assert!(unlocked(&sheets, &revs, 1));
        assert_eq!(revs.next_letter(sheets.all[0].id), 'B');
    }

    #[test]
    fn physics_mode_follows_the_screen() {
        assert_eq!(Mode::compute(Screen::Register), Some(Mode::Build));
        assert_eq!(Mode::compute(Screen::Build), Some(Mode::Build));
        assert_eq!(Mode::compute(Screen::Test), Some(Mode::Simulate));
        assert_eq!(Mode::compute(Screen::Report), Some(Mode::Simulate));
    }
}
