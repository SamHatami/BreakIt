//! The test run on screen: mirrors the scenario run's timeline into the
//! chart recorder and the log, slows down on the first break, and opens
//! the report once the outcome has played out. The loads and the grading
//! are `breakit_scenario::TestRun`'s.

use bevy::prelude::*;

use crate::analysis::{BreakitSet, FrameState, SimClock, Simulation, SPEEDS};
use crate::flow::{Screen, UiAction};
use crate::scenario::{Sheets, TestOutcome, TestRecord, TimedEvent, CHART_COLUMNS};

pub struct TestRunPlugin;

impl Plugin for TestRunPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<TestRecord>()
            .init_resource::<TestOutcome>()
            .init_resource::<Heatmap>()
            .init_resource::<SlowMo>()
            .add_systems(OnEnter(Screen::Test), begin_record)
            .add_systems(OnEnter(Screen::Report), freeze)
            .add_systems(
                Update,
                (test_actions, record.after(BreakitSet::Physics)).run_if(in_state(Screen::Test)),
            );
    }
}

/// Utilisation colours on the members during the test (H).
#[derive(Resource, Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct Heatmap(pub bool);

/// Automatic slow motion after the first break: real seconds left, and the
/// speed to go back to.
#[derive(Resource, Debug, Default, Clone, Copy, PartialEq)]
pub struct SlowMo {
    pub remaining: f32,
    pub restore: f32,
}

impl SlowMo {
    pub fn active(&self) -> bool {
        self.remaining > 0.0
    }
}

/// Slow motion speed and how long it lasts, real seconds.
const SLOWMO_SPEED: f32 = 0.25;
const SLOWMO_SECONDS: f32 = 2.5;
fn begin_record(
    mut rec: ResMut<TestRecord>,
    mut outcome: ResMut<TestOutcome>,
    check: Res<crate::scenario::ScenarioCheck>,
    mut slowmo: ResMut<SlowMo>,
    mut heat: ResMut<Heatmap>,
) {
    *rec = TestRecord {
        cost: check.bill.total() as f32,
        chart: vec![0.0; CHART_COLUMNS],
        ..Default::default()
    };
    *slowmo = SlowMo::default();
    heat.0 = false;
    outcome.0 = None;
}

fn freeze(mut clock: ResMut<SimClock>) {
    clock.paused = true;
}

fn test_actions(
    mut actions: MessageReader<UiAction>,
    mut clock: ResMut<SimClock>,
    mut heat: ResMut<Heatmap>,
    mut slowmo: ResMut<SlowMo>,
) {
    for a in actions.read() {
        match a {
            UiAction::SimPause => clock.paused = !clock.paused,
            UiAction::SimSlower | UiAction::SimFaster => {
                // Taking the controls cancels automatic slow motion.
                *slowmo = SlowMo::default();
                let i = SPEEDS
                    .iter()
                    .position(|s| (*s - clock.speed).abs() < 1e-3)
                    .unwrap_or(3);
                let j = if *a == UiAction::SimSlower {
                    i.saturating_sub(1)
                } else {
                    (i + 1).min(SPEEDS.len() - 1)
                };
                clock.speed = SPEEDS[j];
                clock.paused = false;
            }
            UiAction::ToggleHeatmap => heat.0 = !heat.0,
            _ => {}
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn record(
    time: Res<Time>,
    sheets: Res<Sheets>,
    state: Res<FrameState>,
    sim: Option<Res<Simulation>>,
    mut rec: ResMut<TestRecord>,
    mut clock: ResMut<SimClock>,
    mut slowmo: ResMut<SlowMo>,
    mut next: ResMut<NextState<Screen>>,
    mut outcome: ResMut<TestOutcome>,
) {
    let Some(sim) = sim else { return };
    let s = sheets.current();
    let run = sim.run();
    let t = run.time() as f32;
    let total = s.total_duration();

    // The chart: worst member per column, test and proof test alike.
    let util = state.max_utilization();
    if total > 0.0 && t >= 0.0 {
        let col = ((t / total) * CHART_COLUMNS as f32) as usize;
        if let Some(c) = rec.chart.get_mut(col.min(CHART_COLUMNS - 1)) {
            *c = c.max(util);
        }
    }

    // Read-outs from the run.
    let m = run.measured();
    rec.peak_utilization = m.peak_utilization as f32;
    rec.peak_sag = m
        .peak_deck_sag
        .or_else(|| m.payload_drift.iter().copied().reduce(f64::max))
        .unwrap_or(0.0) as f32;
    let first_break = m.first_break.as_ref().map(|b| b.0 as f32);
    if rec.first_break.is_none() && first_break.is_some() {
        *slowmo = SlowMo {
            remaining: SLOWMO_SECONDS,
            restore: clock.speed,
        };
        clock.speed = SLOWMO_SPEED;
    }
    rec.first_break = first_break;
    rec.events = run
        .timeline()
        .iter()
        .filter_map(|e| TimedEvent::from_entry(s, e))
        .collect();

    // Slow motion runs on real time.
    if slowmo.active() && !clock.paused {
        slowmo.remaining -= time.delta_secs();
        if !slowmo.active() {
            clock.speed = slowmo.restore;
        }
    }

    if run.is_done() {
        outcome.0 = Some(run.report());
        next.set(Screen::Report);
    }
}
