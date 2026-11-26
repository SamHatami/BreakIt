//! A test run: the scenario's loads driving the real-time solver, with the
//! read-outs the report needs recorded as it goes.
//!
//! The game steps a [`TestRun`] once per physics frame and draws
//! [`TestRun::sim`]; [`evaluate`] does the same headless, as fast as it
//! can, for tests and for scoring.

use serde::{Deserialize, Serialize};

use breakit_core::loads::traffic::VehicleRig;
use breakit_core::loads::{LoadKind, LoadSet, RigState};
use breakit_core::{DamageState, DynamicSim, NodeLoad, SimConfig, Structure, Vec3};

use crate::cost::{Bill, CostModel};
use crate::equipment::{Equipment, EquipmentEvent};
use crate::limits::{check_limits, PlayMode, Violation};
use crate::loads::{event_window, Binding};
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
    /// A bolted item tore its bolts out and now rests on friction.
    BoltsFailed {
        payload: usize,
        name: String,
    },
    /// An item tipped or slid off. Its mass has left the structure; it is
    /// at `position` (centre of mass) moving at `velocity`.
    PayloadLost {
        payload: usize,
        name: String,
        /// kg.
        mass: f64,
        /// Contact radius for a falling-object impactor, m.
        radius: f64,
        position: [f64; 3],
        velocity: [f64; 3],
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
    /// Largest movement of each payload's footprint centre, m (same as
    /// `payloads[k].drift`).
    pub payload_drift: Vec<f64>,
    pub payloads: Vec<PayloadMeasured>,
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

/// Read-outs for one payload over the test (not the proof test).
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct PayloadMeasured {
    pub tilt_deg: f64,
    /// Largest downward movement of a foot, m.
    pub sag: f64,
    /// Largest movement of the footprint centre, m.
    pub drift: f64,
    /// Smallest foot reaction over its static share; below 0 a foot lifted.
    pub min_reaction: f64,
    pub bolts_failed: bool,
    /// Test time it came off.
    pub lost: Option<f64>,
}

/// Seconds of physics kept running after the outcome, for the collapse to
/// play out on screen.
pub const AFTERMATH: f64 = 3.0;

pub struct TestRun {
    pub scenario: Scenario,
    pub mode: PlayMode,
    pub bill: Bill,
    sim: DynamicSim,
    structure: Structure,
    binding: Binding,
    /// The scenario's traffic and ground motion as core load cases, in
    /// solver time.
    loads: LoadSet,
    watched: Vec<usize>,
    gravity: Vec3,
    clock: f64,
    phase: Phase,
    decided_at: Option<f64>,
    measured: Measured,
    timeline: Vec<TimelineEntry>,
    seen_breaks: usize,
    event_live: Vec<(bool, bool)>,
    /// Per payload, in scenario order.
    equipment: Vec<Equipment>,
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
        let mut equipment = Vec::new();
        for (p, feet) in scenario.payloads.iter().zip(&binding.payload_feet) {
            let feet = feet.clone().expect("limits check attaches every payload");
            for &n in &feet {
                sim.add_point_mass(n, p.mass / feet.len() as f64);
            }
            equipment.push(Equipment::new(p, feet, &sim));
        }
        binding.node_mass = (0..structure.nodes.len())
            .map(|i| sim.node_mass(i))
            .collect();
        if mode == PlayMode::Sandbox {
            binding.extra = extra.iter().map(|l| (l.node, l.force)).collect();
        }
        let mut watched: Vec<usize> = binding
            .deck_nodes()
            .into_iter()
            .chain(binding.payload_feet.iter().flatten().flatten().copied())
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
            payloads: vec![PayloadMeasured::default(); scenario.payloads.len()],
            ..Default::default()
        };
        let settle = scenario.settle.max(0.0);
        let mut loads = Binding::load_set(scenario, settle);
        loads.prepare(structure);
        sim.config.gravity = if settle > 0.0 { Vec3::zeros() } else { gravity };
        Ok(Self {
            scenario: scenario.clone(),
            mode,
            bill,
            sim,
            structure: structure.clone(),
            binding,
            loads,
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
            equipment,
        })
    }

    pub fn sim(&self) -> &DynamicSim {
        &self.sim
    }
    pub fn binding(&self) -> &Binding {
        &self.binding
    }
    /// The core load cases driving the run. Their times are solver time
    /// ([`DynamicSim::time`]), which runs `settle` seconds ahead of
    /// [`Self::time`].
    pub fn loads(&self) -> &LoadSet {
        &self.loads
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
    /// Every vehicle of the scenario's convoys as it is now, for drawing.
    pub fn vehicles(&self) -> Vec<VehicleRig> {
        self.loads.rig_state(self.sim.time()).vehicles
    }
    /// Every rig (vehicles, shake table, fan wall, flume, impactors) as it
    /// is now, for drawing.
    pub fn rig_state(&self) -> RigState {
        self.loads.rig_state(self.sim.time())
    }
    /// Shake table displacement now, m: add it to every simulated position
    /// (and to the table) when drawing.
    pub fn table_offset(&self) -> Vec3 {
        self.loads.table_offset(self.sim.time())
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

    fn apply_loads(&mut self, dt: f64) {
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
        // The proof test swaps the traffic for a uniform stand-in (in
        // `deck_forces`) and raises everything by the load factor.
        let proof = self.scenario.overload.is_some() && self.clock > self.scenario.duration;
        self.loads.factor = self.load_factor();
        for c in &mut self.loads.cases {
            if matches!(c.kind, LoadKind::Traffic(_)) {
                c.factor = if proof { 0.0 } else { 1.0 };
            }
        }
        let centres: Vec<Vec3> = self
            .equipment
            .iter()
            .map(|e| e.centre(&self.sim) + Vec3::y() * e.height)
            .collect();
        let wind = Binding::payload_wind(&self.scenario, &self.loads, &centres, self.sim.time());
        for (e, w) in self.equipment.iter_mut().zip(wind) {
            e.wind = if e.lost { Vec3::zeros() } else { w };
        }
        self.loads.apply(&self.structure, &mut self.sim, dt);
        let mut f = self.binding.deck_forces(&self.scenario, self.clock);
        for (a, b) in f.iter_mut().zip(self.loads.forces()) {
            *a += b;
        }
        for e in &self.equipment {
            e.add_forces(&mut f);
        }
        for (n, f) in f.into_iter().enumerate() {
            self.sim.set_force(n, f);
        }
    }

    /// Advances one physics frame.
    pub fn step(&mut self, dt: f64) {
        self.apply_loads(dt);
        self.sim.step(dt);
        self.clock += dt;
        self.update_equipment(dt);
        if self.phase == Phase::Decided {
            return;
        }
        if self.phase == Phase::Settling && self.clock >= 0.0 {
            self.phase = Phase::Test;
        }
        self.track_events();
        self.record();
    }

    fn update_equipment(&mut self, dt: f64) {
        let ground = Binding::ground_accel(&self.loads, self.sim.time());
        let g = self.sim.config.gravity;
        let measuring = matches!(self.phase, Phase::Settling | Phase::Test);
        for k in 0..self.equipment.len() {
            let built: Vec<Vec3> = self.equipment[k]
                .feet
                .iter()
                .map(|&n| self.structure.nodes[n])
                .collect();
            let peaks = (
                self.equipment[k].peak_tilt,
                self.equipment[k].peak_sag,
                self.equipment[k].peak_drift,
            );
            let events = self.equipment[k].update(&mut self.sim, &built, ground, g, dt);
            let e = &mut self.equipment[k];
            if !measuring {
                // Read-outs cover the test proper, not the proof test.
                (e.peak_tilt, e.peak_sag, e.peak_drift) = peaks;
            }
            let pm = &mut self.measured.payloads[k];
            pm.tilt_deg = e.peak_tilt.to_degrees();
            pm.sag = e.peak_sag;
            pm.drift = e.peak_drift;
            pm.min_reaction = e.min_reaction;
            self.measured.payload_drift[k] = e.peak_drift;
            let name = self.scenario.payloads[k].name.clone();
            for ev in events {
                match ev {
                    EquipmentEvent::BoltsFailed => {
                        self.measured.payloads[k].bolts_failed = true;
                        self.push(Happening::BoltsFailed {
                            payload: k,
                            name: name.clone(),
                        });
                    }
                    EquipmentEvent::Lost { position, velocity } => {
                        self.measured.payloads[k].lost = Some(self.clock);
                        for &n in &self.equipment[k].feet {
                            self.binding.node_mass[n] = self.sim.node_mass(n);
                        }
                        let p = &self.scenario.payloads[k];
                        let (mass, radius) = (p.mass, p.radius());
                        self.push(Happening::PayloadLost {
                            payload: k,
                            name: name.clone(),
                            mass,
                            radius,
                            position: [position.x, position.y, position.z],
                            velocity: [velocity.x, velocity.y, velocity.z],
                        });
                    }
                }
            }
        }
    }

    /// The first critical payload lost, or strict limit broken, as a
    /// failure reason. `at_end` also checks the strict limits.
    fn payload_failure(&self, at_end: bool) -> Option<String> {
        for (p, m) in self.scenario.payloads.iter().zip(&self.measured.payloads) {
            if p.critical && m.lost.is_some() {
                return Some(format!("the {} came off", p.name));
            }
            if !(at_end && p.limits.strict) {
                continue;
            }
            let l = &p.limits;
            if let Some(t) = l.tilt_deg.filter(|&t| m.tilt_deg > t) {
                return Some(format!(
                    "the {} tilted {:.1}° (limit {t:.1}°)",
                    p.name, m.tilt_deg
                ));
            }
            if let Some(d) = l.sag.filter(|&d| m.sag > d) {
                return Some(format!(
                    "the {} sank {:.0} mm (limit {:.0} mm)",
                    p.name,
                    m.sag * 1000.0,
                    d * 1000.0
                ));
            }
            if let Some(d) = l.drift.filter(|&d| m.drift > d) {
                return Some(format!(
                    "the {} moved {:.0} mm (limit {:.0} mm)",
                    p.name,
                    m.drift * 1000.0,
                    d * 1000.0
                ));
            }
        }
        None
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
        let deck_len = self.binding.deck_length;
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
            if let Some(sag) = &mut self.measured.peak_deck_sag {
                for n in self.binding.deck_nodes() {
                    *sag = sag.max(-self.sim.displacement(n).y);
                }
            }
        }
        let broken_frac = self.measured.broken as f64 / n_members.max(1) as f64;
        if collapse.is_none() && broken_frac > self.scenario.collapse.max_broken_fraction {
            collapse = Some(format!("{:.0}% of members broke", broken_frac * 100.0));
        }

        if collapse.is_none() && !proof {
            collapse = self.payload_failure(false);
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
            if let Some(reason) = self.payload_failure(true) {
                self.push(Happening::Collapse {
                    reason: reason.clone(),
                });
                self.measured.passed = Some(false);
                self.measured.failure = Some(reason);
                self.measured.collapse_time = Some(self.clock);
                self.decide();
                return;
            }
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
