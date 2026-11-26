//! A test run: the scenario's loads driving the real-time solver, with the
//! read-outs the report needs recorded as it goes.
//!
//! The game steps a [`TestRun`] once per physics frame and draws
//! [`TestRun::sim`]; [`evaluate`] does the same headless, as fast as it
//! can, for tests and for scoring.

use serde::{Deserialize, Serialize};

use breakit_core::{DamageState, DynamicSim, NodeLoad, SimConfig, Structure, Vec3};

use crate::cost::{Bill, CostModel};
use crate::limits::{check_limits, PlayMode, Violation};
use crate::loads::{event_window, Binding, VehiclePose};
use crate::report::Report;
use crate::scenario::Scenario;

/// Something that happened during the run, for the report timeline and the
/// test HUD.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct TimelineEntry {
    /// Test time, s (negative while self weight settles).
    pub time: f64,
    pub what: Happening,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum Happening {
    EventStarted {
        event: usize,
        name: String,
    },
    EventEnded {
        event: usize,
        name: String,
    },
    /// First member to yield, crack or crush. `state` is the damage state
    /// in words.
    FirstDamage {
        member: usize,
        state: String,
    },
    Break {
        member: usize,
        cause: String,
    },
    Collapse {
        reason: String,
    },
    /// Survived the full test.
    Passed,
    ProofTestStarted,
    /// The proof test reached its top load factor without collapse.
    ProofTestHeld {
        factor: f64,
    },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Phase {
    /// Self weight ramping in, before the clock starts.
    Settling,
    Test,
    ProofTest,
    /// The outcome is known; physics keeps running for the aftermath.
    Decided,
}

/// What the run has measured so far. All maxima cover settling and the
/// test proper, not the proof test.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Measured {
    pub peak_utilization: f64,
    /// Largest downward deck displacement, m.
    pub peak_deck_sag: Option<f64>,
    /// Largest displacement of each payload's node, m.
    pub payload_drift: Vec<f64>,
    pub first_damage: Option<f64>,
    pub first_break: Option<(f64, usize, String)>,
    pub broken: usize,
    /// `Some` once the test is decided.
    pub passed: Option<bool>,
    pub failure: Option<String>,
    pub collapse_time: Option<f64>,
    /// Proof test: load factor at collapse, or the top factor if it held.
    pub reserve_factor: Option<f64>,
    pub proof_held: bool,
    /// Proof test: first damage at or after the proof test started.
    pub proof_first_damage: Option<f64>,
    pub proof_collapse: Option<f64>,
}

/// Seconds of physics kept running after the outcome, for the collapse to
/// play out on screen.
pub const AFTERMATH: f64 = 3.0;

pub struct TestRun {
    pub scenario: Scenario,
    pub mode: PlayMode,
    pub bill: Bill,
    sim: DynamicSim,
    binding: Binding,
    watched: Vec<usize>,
    gravity: Vec3,
    clock: f64,
    phase: Phase,
    decided_at: Option<f64>,
    measured: Measured,
    timeline: Vec<TimelineEntry>,
    seen_breaks: usize,
    event_live: Vec<(bool, bool)>,
}

impl TestRun {
    /// Checks the limits and sets up the run. `bill` is the structure's
    /// cost (from [`CostModel`], with the player's joint kinds); `extra` are
    /// player loads, applied only in sandbox mode.
    pub fn new(
        scenario: &Scenario,
        structure: &Structure,
        extra: &[NodeLoad],
        bill: Bill,
        mode: PlayMode,
        config: SimConfig,
    ) -> Result<Self, Vec<Violation>> {
        let violations = check_limits(scenario, structure, &bill, mode);
        if !violations.is_empty() {
            return Err(violations);
        }
        let gravity = config.gravity;
        let mut sim = DynamicSim::new(structure, config);
        let mut binding = Binding::new(scenario, structure);
        for (p, n) in scenario.payloads.iter().zip(&binding.payload_nodes) {
            if let Some(n) = *n {
                sim.add_point_mass(n, p.mass);
            }
        }
        binding.node_mass = (0..structure.nodes.len())
            .map(|i| sim.node_mass(i))
            .collect();
        if mode == PlayMode::Sandbox {
            binding.extra = extra.iter().map(|l| (l.node, l.force)).collect();
        }
        let mut watched: Vec<usize> = binding
            .deck
            .iter()
            .flat_map(|d| d.nodes())
            .chain(binding.payload_nodes.iter().flatten().copied())
            .filter(|&n| binding.node_mass[n] > 0.0)
            .collect();
        if watched.is_empty() {
            watched = (0..structure.nodes.len())
                .filter(|&n| binding.node_mass[n] > 0.0)
                .collect();
        }
        watched.sort_unstable();
        watched.dedup();
        let measured = Measured {
            peak_deck_sag: binding.deck.as_ref().map(|_| 0.0),
            payload_drift: vec![0.0; scenario.payloads.len()],
            ..Default::default()
        };
        let settle = scenario.settle.max(0.0);
        let mut run = Self {
            scenario: scenario.clone(),
            mode,
            bill,
            sim,
            binding,
            watched,
            gravity,
            clock: -settle,
            phase: if settle > 0.0 {
                Phase::Settling
            } else {
                Phase::Test
            },
            decided_at: None,
            measured,
            timeline: Vec::new(),
            seen_breaks: 0,
            event_live: vec![(false, false); scenario.events.len()],
        };
        run.apply_loads();
        Ok(run)
    }

    pub fn sim(&self) -> &DynamicSim {
        &self.sim
    }
    pub fn binding(&self) -> &Binding {
        &self.binding
    }
    /// Test time, s. Negative while settling.
    pub fn time(&self) -> f64 {
        self.clock
    }
    pub fn phase(&self) -> Phase {
        self.phase
    }
    pub fn measured(&self) -> &Measured {
        &self.measured
    }
    pub fn timeline(&self) -> &[TimelineEntry] {
        &self.timeline
    }
    /// Current scenario load factor (above 1 only in the proof test).
    pub fn load_factor(&self) -> f64 {
        Binding::factor_at(&self.scenario, self.clock)
    }
    pub fn vehicles(&self) -> Vec<VehiclePose> {
        self.binding.vehicles_at(&self.scenario, self.clock)
    }
    /// The outcome is known (the run may still be playing out).
    pub fn is_decided(&self) -> bool {
        self.phase == Phase::Decided
    }
    /// Decided and the aftermath has played out.
    pub fn is_done(&self) -> bool {
        self.decided_at.is_some_and(|t| self.clock - t >= AFTERMATH)
    }

    pub fn report(&self) -> Report {
        Report::new(self)
    }

    fn apply_loads(&mut self) {
        let settle = self.scenario.settle;
        let g = if self.clock < 0.0 && settle > 0.0 {
            (1.0 + self.clock / settle).clamp(0.0, 1.0)
        } else {
            1.0
        };
        self.sim.config.gravity = self.gravity * g;
        if self.phase == Phase::Decided {
            // Loads stay as they were at the moment of the outcome.
            return;
        }
        let f = self.binding.forces_at(&self.scenario, self.clock);
        for (n, f) in f.into_iter().enumerate() {
            self.sim.set_force(n, f);
        }
    }

    /// Advances one physics frame.
    pub fn step(&mut self, dt: f64) {
        self.apply_loads();
        self.sim.step(dt);
        self.clock += dt;
        if self.phase == Phase::Decided {
            return;
        }
        if self.phase == Phase::Settling && self.clock >= 0.0 {
            self.phase = Phase::Test;
        }
        self.track_events();
        self.record();
    }

    /// Steps until decided or `max_time` of test time passes.
    pub fn run_to_outcome(&mut self, dt: f64) {
        let end = self.scenario.duration
            + self
                .scenario
                .overload
                .map_or(0.0, |o| (o.max_factor - 1.0) / o.rate.max(1e-6) + 2.0)
            + 1.0;
        while !self.is_decided() && self.clock < end {
            self.step(dt);
        }
        if !self.is_decided() {
            // Proof test never finished; count what it reached.
            self.measured.reserve_factor = Some(self.load_factor());
            self.measured.proof_held = true;
            self.decide();
        }
    }

    fn push(&mut self, what: Happening) {
        self.timeline.push(TimelineEntry {
            time: self.clock,
            what,
        });
    }

    fn track_events(&mut self) {
        let deck_len = self.binding.deck.as_ref().map_or(0.0, |d| d.length);
        let in_test = self.phase == Phase::Test;
        for i in 0..self.scenario.events.len() {
            let (a, b) = event_window(&self.scenario.events[i], deck_len);
            let name = self.scenario.events[i].name().to_string();
            let (started, ended) = self.event_live[i];
            if !started && self.clock >= a && in_test {
                self.event_live[i].0 = true;
                self.push(Happening::EventStarted {
                    event: i,
                    name: name.clone(),
                });
            }
            if started && !ended && self.clock >= b.min(self.scenario.duration) {
                self.event_live[i].1 = true;
                self.push(Happening::EventEnded { event: i, name });
            }
        }
    }

    fn record(&mut self) {
        let proof = self.phase == Phase::ProofTest;
        let n_members = self.sim.damage_all().len();

        // Breaks.
        let events = self.sim.break_events()[self.seen_breaks..].to_vec();
        self.seen_breaks += events.len();
        for e in events {
            let cause = format!("{:?}", e.cause).to_lowercase();
            if self.measured.first_break.is_none() && !proof {
                self.measured.first_break = Some((self.clock, e.member, cause.clone()));
            }
            self.measured.broken += 1;
            self.push(Happening::Break {
                member: e.member,
                cause,
            });
        }

        // Damage and utilisation.
        let mut damaged = None;
        let mut peak: f64 = 0.0;
        for (i, d) in self.sim.damage_all().enumerate() {
            if d.state == DamageState::Broken {
                continue;
            }
            peak = peak.max(d.utilization);
            if d.state != DamageState::Intact && damaged.is_none() {
                damaged = Some((i, d.state));
            }
        }
        if let Some((member, state)) = damaged {
            if self.measured.first_damage.is_none() && !proof {
                self.measured.first_damage = Some(self.clock);
                self.push(Happening::FirstDamage {
                    member,
                    state: format!("{state:?}").to_lowercase(),
                });
            }
            if proof && self.measured.proof_first_damage.is_none() {
                self.measured.proof_first_damage = Some(self.clock);
            }
        }
        if !proof {
            self.measured.peak_utilization = self.measured.peak_utilization.max(peak);
        }

        // Displacements and collapse.
        let mut collapse = None;
        let limit = self.scenario.collapse.displacement;
        for &n in &self.watched {
            let u = self.sim.displacement(n);
            if !u.iter().all(|x| x.is_finite()) || u.norm() > limit {
                collapse = Some(format!("node {n} moved more than {limit:.2} m"));
                break;
            }
        }
        if !proof {
            if let (Some(sag), Some(deck)) = (&mut self.measured.peak_deck_sag, &self.binding.deck)
            {
                for n in deck.nodes() {
                    *sag = sag.max(-self.sim.displacement(n).y);
                }
            }
            for (k, n) in self.binding.payload_nodes.iter().enumerate() {
                if let Some(n) = *n {
                    let d = self.sim.displacement(n).norm();
                    self.measured.payload_drift[k] = self.measured.payload_drift[k].max(d);
                }
            }
        }
        let broken_frac = self.measured.broken as f64 / n_members.max(1) as f64;
        if collapse.is_none() && broken_frac > self.scenario.collapse.max_broken_fraction {
            collapse = Some(format!("{:.0}% of members broke", broken_frac * 100.0));
        }

        if let Some(reason) = collapse {
            self.push(Happening::Collapse {
                reason: reason.clone(),
            });
            if proof {
                self.measured.proof_collapse = Some(self.clock);
                self.measured.reserve_factor = Some(self.load_factor());
            } else {
                self.measured.passed = Some(false);
                self.measured.failure = Some(reason);
                self.measured.collapse_time = Some(self.clock);
            }
            self.decide();
            return;
        }

        if self.phase == Phase::Test && self.clock >= self.scenario.duration {
            self.measured.passed = Some(true);
            self.push(Happening::Passed);
            match self.scenario.overload {
                Some(_) => {
                    self.phase = Phase::ProofTest;
                    self.push(Happening::ProofTestStarted);
                }
                None => self.decide(),
            }
            return;
        }

        if let (true, Some(o)) = (proof, self.scenario.overload) {
            let top_since = self.scenario.duration + (o.max_factor - 1.0) / o.rate.max(1e-6);
            if self.clock >= top_since + 1.0 {
                self.measured.reserve_factor = Some(o.max_factor);
                self.measured.proof_held = true;
                self.push(Happening::ProofTestHeld {
                    factor: o.max_factor,
                });
                self.decide();
            }
        }
    }

    fn decide(&mut self) {
        self.phase = Phase::Decided;
        self.decided_at = Some(self.clock);
    }
}

/// Physics frame used by [`evaluate`], s.
pub const EVAL_DT: f64 = 1.0 / 60.0;

/// Runs a scenario headless and grades it.
pub fn evaluate(
    scenario: &Scenario,
    structure: &Structure,
    extra: &[NodeLoad],
    costs: &CostModel,
    mode: PlayMode,
    config: SimConfig,
) -> Result<Report, Vec<Violation>> {
    let bill = costs.estimate(structure);
    let mut run = TestRun::new(scenario, structure, extra, bill, mode, config)?;
    run.run_to_outcome(EVAL_DT);
    Ok(run.report())
}
