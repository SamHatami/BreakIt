//! The test run: applies each load case at its time, records what happens
//! for the chart recorder and the report, slows down on the first break,
//! and ends the run.

use bevy::prelude::*;
use breakit_core::Vec3 as CoreVec3;

use crate::analysis::{cause_label, BreakitSet, FrameState, SimClock, Simulation, SPEEDS};
use crate::design::{to_core, Design};
use crate::flow::{Screen, UiAction};
use crate::scenario::{Sheets, TestEvent, TestRecord, TimedEvent, CHART_COLUMNS};

pub struct TestRunPlugin;

impl Plugin for TestRunPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<TestRecord>()
            .init_resource::<Heatmap>()
            .init_resource::<SlowMo>()
            .add_systems(OnEnter(Screen::Test), begin_record)
            .add_systems(OnEnter(Screen::Report), freeze)
            .add_systems(
                Update,
                (
                    test_actions,
                    apply_load_cases.before(BreakitSet::Physics),
                    record.after(BreakitSet::Physics),
                )
                    .run_if(in_state(Screen::Test)),
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
/// Simulated seconds the run continues after the first break, so the
/// collapse plays out before the report.
const AFTER_BREAK: f32 = 3.0;

/// Index of the current load case in the record (applied once per change).
#[derive(Default)]
struct Applied(Option<usize>);

fn begin_record(
    mut rec: ResMut<TestRecord>,
    design: Res<Design>,
    mut slowmo: ResMut<SlowMo>,
    mut heat: ResMut<Heatmap>,
) {
    *rec = TestRecord {
        cost: design.cost(),
        chart: vec![0.0; CHART_COLUMNS],
        ..Default::default()
    };
    *slowmo = SlowMo::default();
    heat.0 = false;
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

fn apply_load_cases(
    sheets: Res<Sheets>,
    design: Res<Design>,
    sim: Option<ResMut<Simulation>>,
    mut rec: ResMut<TestRecord>,
    mut applied: Local<Applied>,
) {
    let Some(mut sim) = sim else { return };
    let s = sheets.current();
    let t = sim.sim.time() as f32;
    if t == 0.0 {
        applied.0 = None;
    }
    let case = s.case_at(t);
    if case == applied.0 || case.is_none() {
        return;
    }
    applied.0 = case;
    let c = case.unwrap();
    let mut forces: Vec<CoreVec3> = design.loads.iter().map(|&f| to_core(f)).collect();
    for (n, f) in s.case_forces(c) {
        if let Some(slot) = forces.get_mut(n) {
            *slot += to_core(f);
        }
    }
    for (n, f) in forces.into_iter().enumerate() {
        sim.sim.set_force(n, f);
    }
    rec.events.push(TimedEvent {
        time: t,
        event: TestEvent::CaseStarted { case: c },
    });
}

#[allow(clippy::too_many_arguments)]
fn record(
    time: Res<Time>,
    sheets: Res<Sheets>,
    design: Res<Design>,
    state: Res<FrameState>,
    sim: Option<Res<Simulation>>,
    mut rec: ResMut<TestRecord>,
    mut clock: ResMut<SimClock>,
    mut slowmo: ResMut<SlowMo>,
    mut next: ResMut<NextState<Screen>>,
) {
    let Some(sim) = sim else { return };
    let s = sheets.current();
    let t = sim.sim.time() as f32;
    let total = s.total_duration();

    // Utilisation and sag.
    let util = state.max_utilization();
    rec.peak_utilization = rec.peak_utilization.max(util);
    // Sag stops counting at the first break: debris falling is not sag.
    let intact = rec.first_break.is_none();
    for i in (0..s.load_points.len()).filter(|_| intact) {
        let n = s.load_point_node(i);
        if let (Some(rest), Some(now)) = (design.nodes.get(n), state.positions.get(n)) {
            rec.peak_sag = rec.peak_sag.max(rest.y - now.y);
        }
    }
    if total > 0.0 {
        let col = ((t / total) * CHART_COLUMNS as f32) as usize;
        if let Some(c) = rec.chart.get_mut(col.min(CHART_COLUMNS - 1)) {
            *c = c.max(util);
        }
    }
    let yielded = rec
        .events
        .iter()
        .any(|e| matches!(e.event, TestEvent::FirstYield { .. }));
    if !yielded {
        if let Some((m, u)) = state
            .members
            .iter()
            .enumerate()
            .filter(|(_, m)| !m.broken && m.utilization >= 1.0)
            .map(|(i, m)| (i, m.utilization))
            .next()
        {
            rec.events.push(TimedEvent {
                time: t,
                event: TestEvent::FirstYield {
                    member: m,
                    utilization: u,
                },
            });
        }
    }

    // Breaks.
    let known = rec.broken_members().len();
    let events = sim.sim.break_events();
    for e in &events[known.min(events.len())..] {
        rec.events.push(TimedEvent {
            time: e.time as f32,
            event: TestEvent::Broke {
                member: e.member,
                cause: cause_label(&e.cause),
            },
        });
        if rec.first_break.is_none() {
            rec.first_break = Some(e.time as f32);
            *slowmo = SlowMo {
                remaining: SLOWMO_SECONDS,
                restore: clock.speed,
            };
            clock.speed = SLOWMO_SPEED;
        }
    }

    // Slow motion runs on real time.
    if slowmo.active() && !clock.paused {
        slowmo.remaining -= time.delta_secs();
        if !slowmo.active() {
            clock.speed = slowmo.restore;
        }
    }

    // End of the run.
    let done = t >= total || rec.first_break.is_some_and(|b| t >= b + AFTER_BREAK);
    if done {
        rec.completed = t >= total && rec.first_break.is_none();
        next.set(Screen::Report);
    }
}
