//! The game's view of the test sheets. The scenarios themselves (limits,
//! loads, the test run and the grading) live in the engine-free
//! `breakit-scenario` crate; this module turns them into what the screens
//! show and the build tool needs: anchors, load points, load-case bands,
//! the live pre-test check and the static preview loads.
//!
//! Payload points come first in a sheet's design and are locked, so their
//! node indices never move while the player edits. Anchors are not nodes:
//! a node placed on one gets that foundation's support.

use bevy::math::Vec3;
use bevy::prelude::Resource;
use breakit_core::NodeLoad;
use breakit_scenario as bs;
use breakit_scenario::{check_limits, Bill, Binding, CostModel, PlayMode, Violation};

use crate::analysis::GRAVITY;
use crate::design::{to_core, BeamKind, Design, SupportKind};

pub use breakit_scenario::Criterion as Check;
pub use breakit_scenario::Report;

/// Register tab a sheet sits under.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Rig {
    Bridges,
    Towers,
    Sandbox,
}

impl Rig {
    pub const ALL: [Rig; 3] = [Rig::Bridges, Rig::Towers, Rig::Sandbox];

    pub fn label(self) -> &'static str {
        match self {
            Rig::Bridges => "Bridges",
            Rig::Towers => "Towers",
            Rig::Sandbox => "Sandbox",
        }
    }

    /// The built-in play table this rig sits on (see `breakit_lab::PlayTable::builtin`).
    pub fn table(self) -> usize {
        match self {
            Rig::Bridges => 0,
            Rig::Towers => 3,
            Rig::Sandbox => 1,
        }
    }

    fn from_group(group: &str) -> Self {
        match group {
            "Towers" => Rig::Towers,
            _ => Rig::Bridges,
        }
    }
}

/// One band of the test timeline: a scenario event (or the proof test)
/// from `start` to `end`, seconds of test time.
#[derive(Clone, Debug, PartialEq)]
pub struct LoadCase {
    pub name: String,
    /// What it puts on the structure, e.g. `2 x 7.5 t` or `0.4 g`.
    pub load: String,
    pub start: f32,
    pub end: f32,
}

/// A sheet: a scenario played in a mode, with what the screens need from
/// it worked out once.
#[derive(Clone, Debug)]
pub struct Scenario {
    pub def: bs::Scenario,
    pub mode: PlayMode,
    /// Sheet number, e.g. "B-01".
    pub id: &'static str,
    pub title: String,
    pub rig: Rig,
    /// Length of the deck, or the width of the build area, metres.
    pub span: f32,
    pub anchors: Vec<(Vec3, SupportKind)>,
    /// Payload positions; locked nodes at the start of the design.
    pub load_points: Vec<Vec3>,
    /// Scenario events in time order, then the proof test if any.
    pub load_cases: Vec<LoadCase>,
    pub budget: Option<f32>,
    /// Cost for full marks.
    pub bonus_budget: Option<f32>,
    /// Deck sag objective, metres.
    pub sag_limit: Option<f32>,
    pub allowed: Vec<BeamKind>,
    /// Free play: the player places supports and loads, no limits.
    pub sandbox: bool,
}

fn v3(p: [f64; 3]) -> Vec3 {
    Vec3::new(p[0] as f32, p[1] as f32, p[2] as f32)
}

fn event_load(e: &bs::Event) -> String {
    match e {
        bs::Event::Convoy { count, axles, .. } => {
            let t = axles.iter().map(|a| a.load).sum::<f64>() / 9.81 / 1000.0;
            format!("{count} x {t:.1} t")
        }
        bs::Event::DeckLoad { per_metre, .. } => format!("{:.0} kN/m", per_metre / 1000.0),
        bs::Event::GroundShake { peak, .. } => format!("{:.1} g", peak / 9.81),
    }
}

/// A bonus objective in words, for the brief.
pub fn objective_label(o: &bs::Objective) -> String {
    match o {
        bs::Objective::DeckDeflection { limit } => {
            format!("Deck sag under {:.0} mm", limit * 1000.0)
        }
        bs::Objective::PayloadDrift { payload, limit } => {
            format!("{payload} moves under {:.0} mm", limit * 1000.0)
        }
        bs::Objective::NoBreaks => "Nothing breaks".into(),
        bs::Objective::GracefulFailure { warning } => {
            format!("Proof test: {warning:.0} s warning before collapse")
        }
        bs::Objective::Reserve { factor } => format!("Proof test: holds x{factor:.1}"),
    }
}

fn material(k: BeamKind) -> bs::MaterialClass {
    k.material().kind.into()
}

impl Scenario {
    pub fn new(def: bs::Scenario, mode: PlayMode) -> Self {
        let sandbox = mode == PlayMode::Sandbox;
        let id: &'static str = if sandbox {
            "S-00"
        } else {
            Box::leak(def.sheet.clone().into_boxed_str())
        };
        let deck_len = def
            .deck
            .as_ref()
            .map(|d| (v3(d.to) - v3(d.from)).length())
            .unwrap_or(0.0);
        let span = if deck_len > 0.0 {
            deck_len
        } else {
            let v = &def.limits.volume;
            (v.max[0] - v.min[0]) as f32
        };
        let duration = def.duration as f32;
        let mut load_cases: Vec<LoadCase> = def
            .events
            .iter()
            .map(|e| {
                let (a, b) = bs::loads::event_window(e, deck_len as f64);
                LoadCase {
                    name: e.name().to_string(),
                    load: event_load(e),
                    start: a as f32,
                    end: (b as f32).min(duration),
                }
            })
            .collect();
        load_cases.sort_by(|a, b| a.start.total_cmp(&b.start));
        if let Some(o) = def.overload.filter(|_| !sandbox) {
            let end = duration + ((o.max_factor - 1.0) / o.rate.max(1e-6)) as f32 + 1.0;
            load_cases.push(LoadCase {
                name: "Proof test".into(),
                load: format!("up to x{:.0}", o.max_factor),
                start: duration,
                end,
            });
        }
        let allowed = crate::design::catalog()
            .parts
            .iter()
            .map(|p| BeamKind(p.choice))
            .filter(|k| {
                sandbox
                    || def
                        .limits
                        .materials
                        .as_ref()
                        .is_none_or(|m| m.contains(&material(*k)))
            })
            .collect();
        let sag_limit = def.objectives.iter().find_map(|o| match o {
            bs::Objective::DeckDeflection { limit } => Some(*limit as f32),
            _ => None,
        });
        Self {
            id,
            title: if sandbox {
                "Free bench".into()
            } else {
                def.title.clone()
            },
            rig: if sandbox {
                Rig::Sandbox
            } else {
                Rig::from_group(&def.group)
            },
            span,
            anchors: def
                .anchors
                .iter()
                .map(|a| {
                    let s = match a.kind {
                        bs::AnchorKind::Pinned => SupportKind::Pinned,
                        bs::AnchorKind::Fixed => SupportKind::Fixed,
                    };
                    (v3(a.position), s)
                })
                .collect(),
            load_points: def.payloads.iter().map(|p| v3(p.at)).collect(),
            load_cases,
            budget: def.limits.budget.filter(|_| !sandbox).map(|b| b as f32),
            bonus_budget: (!sandbox).then_some(def.scoring.target_cost as f32),
            sag_limit,
            allowed,
            sandbox,
            mode,
            def,
        }
    }

    /// What the sheet loads, for the register: payloads, else the
    /// heaviest event.
    pub fn load_summary(&self) -> String {
        if self.sandbox {
            return "your own".into();
        }
        if let Some(p) = self.def.payloads.first() {
            return format!("{:.0} t {}", p.mass / 1000.0, p.name);
        }
        self.load_cases
            .first()
            .map(|c| c.load.clone())
            .unwrap_or_else(|| "self weight".into())
    }

    /// Test time the chart covers: the test, then the proof test.
    pub fn total_duration(&self) -> f32 {
        self.load_cases
            .iter()
            .map(|c| c.end)
            .fold(self.def.duration as f32, f32::max)
    }

    /// The band running at test time `t`: the latest started one still
    /// going, else the latest started. `None` before the first and after
    /// the end.
    pub fn case_at(&self, t: f32) -> Option<usize> {
        if t < 0.0 || t >= self.total_duration() {
            return None;
        }
        let started = || {
            self.load_cases
                .iter()
                .enumerate()
                .filter(|(_, c)| c.start <= t)
        };
        started()
            .rfind(|(_, c)| t < c.end)
            .or_else(|| started().next_back())
            .map(|(i, _)| i)
    }

    pub fn case_starts(&self) -> Vec<f32> {
        self.load_cases.iter().map(|c| c.start).collect()
    }

    /// The design the player starts from: locked payload nodes, and the
    /// anchors that give a node placed on them its support.
    pub fn initial_design(&self) -> Design {
        let mut d = if self.sandbox {
            Design::starter_bridge()
        } else {
            let mut d = Design::default();
            for &p in &self.load_points {
                d.add_locked_node(p, None);
            }
            d
        };
        if !self.sandbox {
            d.anchors = self.anchors.clone();
        }
        d
    }

    /// Nodes whose sag the sheet cares about: deck nodes on the lane, else
    /// the payload nodes.
    pub fn sag_nodes(&self, design: &Design) -> Vec<usize> {
        let Some(d) = &self.def.deck else {
            return (0..self.load_points.len()).collect();
        };
        let (a, b) = (v3(d.from), v3(d.to));
        let len = (b - a).length().max(1e-6);
        let dir = (b - a) / len;
        design
            .nodes
            .iter()
            .enumerate()
            .filter(|(_, p)| {
                let r = **p - a;
                let along = r.dot(dir);
                let off = r - dir * along;
                (-0.01..=len + 0.01).contains(&along)
                    && off.y.abs() <= d.height as f32
                    && Vec3::new(off.x, 0.0, off.z).length() <= d.width as f32
            })
            .map(|(i, _)| i)
            .collect()
    }

    /// Every payload node has a beam, and they are all joined into one
    /// structure.
    pub fn is_connected(&self, d: &Design) -> bool {
        let fixed = self.load_points.len();
        if fixed == 0 || d.nodes.len() < fixed {
            return !d.beams.is_empty() && d.nodes.len() >= fixed;
        }
        let mut parent: Vec<usize> = (0..d.nodes.len()).collect();
        fn root(p: &mut [usize], mut i: usize) -> usize {
            while p[i] != i {
                p[i] = p[p[i]];
                i = p[i];
            }
            i
        }
        for b in &d.beams {
            let (ra, rb) = (root(&mut parent, b.a), root(&mut parent, b.b));
            parent[ra] = rb;
        }
        let r0 = root(&mut parent, 0);
        (0..fixed).all(|i| root(&mut parent, i) == r0)
            && (0..fixed).all(|i| d.beams.iter().any(|b| b.a == i || b.b == i))
    }

    /// Loads for build mode's static check: payload weights, the moment of
    /// the test with the most load on the structure, and for ground
    /// shaking each payload's peak inertia force.
    pub fn static_loads(&self, design: &Design) -> Vec<NodeLoad> {
        if design.beams.is_empty() {
            return Vec::new();
        }
        let s = design.to_structure();
        let binding = Binding::new(&self.def, &s);
        let mut out = Vec::new();
        let mut best: (f64, Vec<breakit_core::Vec3>) = (0.0, Vec::new());
        let steps = (self.def.duration * 10.0).ceil() as usize;
        for k in 0..=steps {
            let f = binding.forces_at(&self.def, k as f64 * 0.1);
            let total: f64 = f.iter().map(|x| x.norm()).sum();
            if total > best.0 {
                best = (total, f);
            }
        }
        for (n, f) in best.1.into_iter().enumerate() {
            if f.norm() > 0.0 {
                out.push(NodeLoad::force(n, f));
            }
        }
        let shake = self
            .def
            .events
            .iter()
            .filter_map(|e| match e {
                bs::Event::GroundShake {
                    peak, direction, ..
                } => Some(v3(*direction).normalize_or_zero() * *peak as f32),
                _ => None,
            })
            .max_by(|a, b| a.length().total_cmp(&b.length()))
            .unwrap_or(Vec3::ZERO);
        for (p, n) in self.def.payloads.iter().zip(&binding.payload_nodes) {
            if let Some(n) = *n {
                let m = p.mass as f32;
                out.push(NodeLoad::force(
                    n,
                    to_core(Vec3::new(0.0, -m * GRAVITY, 0.0) + shake * m),
                ));
            }
        }
        out
    }
}

/// The live pre-test check for the current design: its bill and what
/// blocks the test.
#[derive(Resource, Debug, Default, Clone)]
pub struct ScenarioCheck {
    pub bill: Bill,
    pub violations: Vec<Violation>,
    pub key: Option<(u64, usize)>,
}

impl ScenarioCheck {
    pub fn blocks_test(&self) -> bool {
        !self.violations.is_empty()
    }

    pub fn compute(s: &Scenario, design: &Design, costs: &CostModel) -> (Bill, Vec<Violation>) {
        let structure = design.to_structure();
        let bill = design.bill(&structure, costs);
        let violations = if design.beams.is_empty() {
            vec![Violation::NoSupports]
        } else {
            check_limits(&s.def, &structure, &bill, s.mode)
        };
        (bill, violations)
    }
}

/// Something worth telling the player during a test run, from the run's
/// timeline.
#[derive(Clone, Debug, PartialEq)]
pub enum TestEvent {
    CaseStarted { case: usize },
    FirstYield { member: usize, state: String },
    Broke { member: usize, cause: String },
    Collapse { reason: String },
    Passed,
    ProofHeld { factor: f32 },
}

#[derive(Clone, Debug, PartialEq)]
pub struct TimedEvent {
    pub time: f32,
    pub event: TestEvent,
}

impl TimedEvent {
    /// The UI's reading of a run timeline entry, given the sheet's bands.
    pub fn from_entry(s: &Scenario, e: &bs::TimelineEntry) -> Option<Self> {
        let time = e.time as f32;
        let event = match &e.what {
            bs::Happening::EventStarted { name, .. } => TestEvent::CaseStarted {
                case: s.load_cases.iter().position(|c| &c.name == name)?,
            },
            bs::Happening::ProofTestStarted => TestEvent::CaseStarted {
                case: s.load_cases.iter().position(|c| c.name == "Proof test")?,
            },
            bs::Happening::FirstDamage { member, state } => TestEvent::FirstYield {
                member: *member,
                state: state.clone(),
            },
            bs::Happening::Break { member, cause } => TestEvent::Broke {
                member: *member,
                cause: cause.clone(),
            },
            bs::Happening::Collapse { reason } => TestEvent::Collapse {
                reason: reason.clone(),
            },
            bs::Happening::Passed => TestEvent::Passed,
            bs::Happening::ProofTestHeld { factor } => TestEvent::ProofHeld {
                factor: *factor as f32,
            },
            bs::Happening::EventEnded { .. } => return None,
        };
        Some(Self { time, event })
    }
}

/// The graded report of the run that just ended, handed from the test
/// screen to the report screen.
#[derive(Resource, Debug, Default, Clone)]
pub struct TestOutcome(pub Option<Report>);

/// What the UI records of a test run, for the chart recorder and the log.
#[derive(Resource, Clone, Debug, Default, PartialEq)]
pub struct TestRecord {
    pub events: Vec<TimedEvent>,
    /// Largest deck sag (or payload drift), metres.
    pub peak_sag: f32,
    /// Highest member utilisation over the test (not the proof test).
    pub peak_utilization: f32,
    /// Time of the first break in the test, if any.
    pub first_break: Option<f32>,
    pub cost: f32,
    /// Worst utilisation per chart column (see [`CHART_COLUMNS`]).
    pub chart: Vec<f32>,
}

/// Number of columns in the test-run chart recorder.
pub const CHART_COLUMNS: usize = 160;

impl TestRecord {
    pub fn broken_members(&self) -> Vec<usize> {
        self.events
            .iter()
            .filter_map(|e| match e.event {
                TestEvent::Broke { member, .. } => Some(member),
                _ => None,
            })
            .collect()
    }
}

/// Formats an amount with a thin thousands gap: `2 100 kr`.
pub fn money(v: f32) -> String {
    let n = v.round().max(0.0) as u64;
    let s = n.to_string();
    let mut out = String::new();
    for (i, ch) in s.chars().enumerate() {
        if i > 0 && (s.len() - i).is_multiple_of(3) {
            out.push(' ');
        }
        out.push(ch);
    }
    out + " kr"
}

/// Which sheet is open, every sheet in the register, and the cost model.
#[derive(Resource, Clone, Debug)]
pub struct Sheets {
    pub all: Vec<Scenario>,
    pub current: usize,
    pub costs: CostModel,
}

impl Default for Sheets {
    fn default() -> Self {
        Self {
            all: catalogue(),
            current: 0,
            costs: CostModel::default(),
        }
    }
}

impl Sheets {
    pub fn current(&self) -> &Scenario {
        &self.all[self.current]
    }
}

/// The built-in scenarios as sheets, then a sandbox bench on the first
/// bridge.
pub fn catalogue() -> Vec<Scenario> {
    let defs = bs::builtin();
    let mut out: Vec<Scenario> = defs
        .iter()
        .cloned()
        .map(|d| Scenario::new(d, PlayMode::Challenge))
        .collect();
    if let Some(first) = defs.into_iter().next() {
        out.push(Scenario::new(first, PlayMode::Sandbox));
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use breakit_core::SimConfig;

    fn bridge() -> Scenario {
        catalogue().into_iter().find(|s| s.id == "B-01").unwrap()
    }

    #[test]
    fn catalogue_has_each_scenario_and_a_sandbox() {
        let all = catalogue();
        assert!(all.iter().any(|s| s.rig == Rig::Bridges));
        assert!(all.iter().any(|s| s.rig == Rig::Towers));
        assert!(all.last().unwrap().sandbox);
    }

    #[test]
    fn bridge_bands_run_in_order_then_the_proof_test() {
        let s = bridge();
        assert!(s.load_cases.windows(2).all(|w| w[0].start <= w[1].start));
        assert_eq!(s.load_cases.last().unwrap().name, "Proof test");
        assert_eq!(s.case_at(0.0), None.or(s.case_at(0.0)));
        assert_eq!(
            s.case_at(s.def.duration as f32 + 0.5),
            Some(s.load_cases.len() - 1)
        );
        assert_eq!(s.case_at(s.total_duration() + 1.0), None);
    }

    #[test]
    fn anchors_support_nodes_placed_on_them() {
        let s = bridge();
        let mut d = s.initial_design();
        let (p, k) = s.anchors[1];
        let n = d.add_node(p);
        assert_eq!(d.supports[n], Some(k));
        let m = d.add_node(p + Vec3::X);
        assert_eq!(d.supports[m], None);
    }

    #[test]
    fn starter_bridge_passes_the_bridge_sheet_and_previews_loads() {
        let s = bridge();
        let mut d = Design::starter_bridge();
        d.loads.iter_mut().for_each(|l| *l = Vec3::ZERO);
        let (_, v) = ScenarioCheck::compute(&s, &d, &CostModel::default());
        assert_eq!(v, vec![]);
        assert!(!s.static_loads(&d).is_empty());
        let r = bs::evaluate(
            &s.def,
            &d.to_structure(),
            &[],
            &CostModel::default(),
            PlayMode::Challenge,
            SimConfig::default(),
        )
        .unwrap();
        assert!(r.passed);
        assert!(r.criteria.iter().any(|c| !c.bonus));
    }

    #[test]
    fn tower_previews_the_tank_weight() {
        let s = catalogue()
            .into_iter()
            .find(|s| s.rig == Rig::Towers)
            .unwrap();
        let mut d = s.initial_design();
        assert_eq!(d.nodes.len(), 1);
        let base = d.add_node(Vec3::new(-1.0, 0.0, -1.0));
        assert!(d.supports[base].is_some());
        d.add_beam(base, 0, BeamKind::part("steel-tube-m"));
        let loads = s.static_loads(&d);
        assert!(loads.iter().any(|l| l.node == 0 && l.force.y < -30_000.0));
    }

    #[test]
    fn money_groups_thousands() {
        assert_eq!(money(41_200.0), "41 200 kr");
        assert_eq!(money(950.0), "950 kr");
    }
}
