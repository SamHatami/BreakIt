//! Scenarios ("sheets"): the site, the loads the test applies over time,
//! the budget and the pass criteria, and the judging of a test run.
//!
//! Pure data and maths, no Bevy systems, so it is unit tested headless.
//! A sheet's fixed nodes come first in the design (anchors, then load
//! points) and are locked, so their indices never move while the player
//! edits.

use bevy::math::Vec3;
use bevy::prelude::Resource;

use crate::design::{BeamKind, Design, SupportKind};

/// The test rig a sheet runs on. The register groups sheets by rig.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Rig {
    TrafficBridge,
    Sandbox,
}

impl Rig {
    pub const ALL: [Rig; 2] = [Rig::TrafficBridge, Rig::Sandbox];

    pub fn label(self) -> &'static str {
        match self {
            Rig::TrafficBridge => "Traffic bridge",
            Rig::Sandbox => "Sandbox",
        }
    }
}

/// One step of the test: these downward loads, in newtons, on these load
/// points (indices into [`Scenario::load_points`]), for this long. Loads
/// not listed are zero during the case.
#[derive(Clone, Debug, PartialEq)]
pub struct LoadCase {
    pub name: &'static str,
    pub duration: f32,
    pub loads: Vec<(usize, f32)>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Scenario {
    pub id: &'static str,
    pub title: &'static str,
    pub rig: Rig,
    /// Clear span the structure must cross, metres (for the brief).
    pub span: f32,
    pub anchors: Vec<(Vec3, SupportKind)>,
    pub load_points: Vec<Vec3>,
    pub load_cases: Vec<LoadCase>,
    /// Cost limit; `None` in the sandbox.
    pub budget: Option<f32>,
    /// Cost for the bonus grade.
    pub bonus_budget: Option<f32>,
    /// Largest allowed sag at any load point, metres.
    pub sag_limit: Option<f32>,
    pub allowed: Vec<BeamKind>,
    /// Free play: the player places supports and loads.
    pub sandbox: bool,
}

impl Scenario {
    pub fn total_duration(&self) -> f32 {
        self.load_cases.iter().map(|c| c.duration).sum()
    }

    /// Index of the load case running at test time `t`, or `None` once the
    /// test is over.
    pub fn case_at(&self, t: f32) -> Option<usize> {
        let mut end = 0.0;
        for (i, c) in self.load_cases.iter().enumerate() {
            end += c.duration;
            if t < end {
                return Some(i);
            }
        }
        None
    }

    /// Start time of each load case.
    pub fn case_starts(&self) -> Vec<f32> {
        let mut t = 0.0;
        self.load_cases
            .iter()
            .map(|c| {
                let s = t;
                t += c.duration;
                s
            })
            .collect()
    }

    /// Design node of load point `i`.
    pub fn load_point_node(&self, i: usize) -> usize {
        self.anchors.len() + i
    }

    /// The fixed nodes, before the player adds anything.
    pub fn initial_design(&self) -> Design {
        if self.sandbox {
            return Design::starter_bridge();
        }
        let mut d = Design::default();
        for &(p, s) in &self.anchors {
            d.add_locked_node(p, Some(s));
        }
        for &p in &self.load_points {
            d.add_locked_node(p, None);
        }
        d
    }

    /// Node forces (world frame, newtons) for load case `case`.
    pub fn case_forces(&self, case: usize) -> Vec<(usize, Vec3)> {
        let Some(c) = self.load_cases.get(case) else {
            return Vec::new();
        };
        (0..self.load_points.len())
            .map(|i| {
                let f = c
                    .loads
                    .iter()
                    .filter(|(p, _)| *p == i)
                    .map(|(_, f)| *f)
                    .sum::<f32>();
                (self.load_point_node(i), Vec3::new(0.0, -f, 0.0))
            })
            .collect()
    }

    /// The load case with the largest total load, which build mode's static
    /// check previews.
    pub fn heaviest_case(&self) -> Option<usize> {
        self.load_cases
            .iter()
            .enumerate()
            .map(|(i, c)| (i, c.loads.iter().map(|l| l.1).sum::<f32>()))
            .filter(|(_, total)| *total > 0.0)
            .max_by(|a, b| a.1.total_cmp(&b.1))
            .map(|(i, _)| i)
    }

    /// Every fixed node has a beam, and they are all joined into one
    /// structure.
    pub fn is_connected(&self, d: &Design) -> bool {
        let fixed = self.anchors.len() + self.load_points.len();
        if fixed == 0 {
            return !d.beams.is_empty();
        }
        if d.nodes.len() < fixed {
            return false;
        }
        // Union-find over the design's beams.
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
}

/// Something worth telling the player during a test run.
#[derive(Clone, Debug, PartialEq)]
pub enum TestEvent {
    CaseStarted { case: usize },
    FirstYield { member: usize, utilization: f32 },
    Broke { member: usize, cause: String },
}

#[derive(Clone, Debug, PartialEq)]
pub struct TimedEvent {
    pub time: f32,
    pub event: TestEvent,
}

/// What a test run measured.
#[derive(Resource, Clone, Debug, Default, PartialEq)]
pub struct TestRecord {
    pub events: Vec<TimedEvent>,
    /// Largest sag at any load point over the run, metres.
    pub peak_sag: f32,
    /// Highest member utilisation over the run.
    pub peak_utilization: f32,
    /// Time of the first break, if any.
    pub first_break: Option<f32>,
    /// The run reached the end of its last load case.
    pub completed: bool,
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

/// One pass criterion, measured against what was required.
#[derive(Clone, Debug, PartialEq)]
pub struct Check {
    pub label: String,
    pub measured: String,
    pub required: String,
    pub ok: bool,
    /// Bonus checks don't decide pass or fail.
    pub bonus: bool,
}

/// Judges a finished test run.
pub fn judge(s: &Scenario, r: &TestRecord) -> Vec<Check> {
    let mut out = Vec::new();
    let total = s.total_duration();
    out.push(Check {
        label: format!("Survive {}", case_span_label(s)),
        measured: match r.first_break {
            Some(t) => format!("broke at {t:.2} s"),
            None if r.completed => "held".into(),
            None => "stopped early".into(),
        },
        required: format!("{total:.1} s, nothing breaks"),
        ok: r.first_break.is_none() && r.completed,
        bonus: false,
    });
    if let Some(limit) = s.sag_limit {
        out.push(Check {
            label: "Peak sag at load points".into(),
            measured: format!("{:.0} mm", r.peak_sag * 1000.0),
            required: format!("under {:.0} mm", limit * 1000.0),
            ok: r.peak_sag <= limit,
            bonus: false,
        });
    }
    if let Some(budget) = s.budget {
        out.push(Check {
            label: "Cost".into(),
            measured: money(r.cost),
            required: format!("under {}", money(budget)),
            ok: r.cost <= budget,
            bonus: false,
        });
    }
    if let Some(bonus) = s.bonus_budget {
        out.push(Check {
            label: "Bonus: cost".into(),
            measured: money(r.cost),
            required: format!("under {}", money(bonus)),
            ok: r.cost <= bonus,
            bonus: true,
        });
    }
    if !s.sandbox {
        out.push(Check {
            label: "Bonus: no member over 80%".into(),
            measured: format!("peak {:.0}%", r.peak_utilization * 100.0),
            required: "80%".into(),
            ok: r.peak_utilization <= 0.8,
            bonus: true,
        });
    }
    out
}

pub fn passed(checks: &[Check]) -> bool {
    checks.iter().filter(|c| !c.bonus).all(|c| c.ok)
}

fn case_span_label(s: &Scenario) -> String {
    match s.load_cases.len() {
        0 => "the test".into(),
        1 => "LC1".into(),
        n => format!("LC1 to LC{n}"),
    }
}

/// Formats an amount with a thin thousands gap: `41 200 kr`.
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

/// Which sheet is open, and every sheet in the register.
#[derive(Resource, Clone, Debug)]
pub struct Sheets {
    pub all: Vec<Scenario>,
    pub current: usize,
}

impl Default for Sheets {
    fn default() -> Self {
        Self {
            all: catalogue(),
            current: 0,
        }
    }
}

impl Sheets {
    pub fn current(&self) -> &Scenario {
        &self.all[self.current]
    }
}

const KN: f32 = 1000.0;

/// The built-in sheets.
pub fn catalogue() -> Vec<Scenario> {
    let steel_timber = vec![BeamKind::SteelTube, BeamKind::Timber];
    vec![
        Scenario {
            id: "B-01",
            title: "Plank crossing",
            rig: Rig::TrafficBridge,
            span: 4.0,
            anchors: vec![
                (Vec3::new(-2.0, 0.0, 0.0), SupportKind::Pinned),
                (Vec3::new(2.0, 0.0, 0.0), SupportKind::Pinned),
            ],
            load_points: vec![Vec3::new(0.0, 0.0, 0.0)],
            load_cases: vec![
                LoadCase {
                    name: "Self weight",
                    duration: 2.0,
                    loads: vec![],
                },
                LoadCase {
                    name: "Handcart 2 t",
                    duration: 5.0,
                    loads: vec![(0, 20.0 * KN)],
                },
            ],
            budget: Some(6_000.0),
            bonus_budget: Some(4_500.0),
            sag_limit: Some(0.02),
            allowed: steel_timber.clone(),
            sandbox: false,
        },
        Scenario {
            id: "B-02",
            title: "Footbridge over flume",
            rig: Rig::TrafficBridge,
            span: 6.0,
            anchors: vec![
                (Vec3::new(-3.0, 0.0, 0.0), SupportKind::Pinned),
                (Vec3::new(3.0, 0.0, 0.0), SupportKind::Pinned),
            ],
            load_points: vec![
                Vec3::new(-1.0, 0.0, 0.0),
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
            ],
            load_cases: vec![
                LoadCase {
                    name: "Self weight",
                    duration: 2.0,
                    loads: vec![],
                },
                LoadCase {
                    name: "One vehicle",
                    duration: 4.0,
                    loads: vec![(1, 40.0 * KN)],
                },
                LoadCase {
                    name: "Three vehicles",
                    duration: 6.0,
                    loads: vec![(0, 40.0 * KN), (1, 40.0 * KN), (2, 40.0 * KN)],
                },
            ],
            budget: Some(14_000.0),
            bonus_budget: Some(10_000.0),
            sag_limit: Some(0.024),
            allowed: steel_timber.clone(),
            sandbox: false,
        },
        Scenario {
            id: "B-03",
            title: "Two-lane road bridge",
            rig: Rig::TrafficBridge,
            span: 10.0,
            anchors: vec![
                (Vec3::new(-5.0, 0.0, 0.0), SupportKind::Pinned),
                (Vec3::new(5.0, 0.0, 0.0), SupportKind::Pinned),
            ],
            load_points: (-2..=2).map(|i| Vec3::new(i as f32, 0.0, 0.0)).collect(),
            load_cases: vec![
                LoadCase {
                    name: "Self weight",
                    duration: 2.0,
                    loads: vec![],
                },
                LoadCase {
                    name: "One lorry",
                    duration: 4.0,
                    loads: vec![(1, 60.0 * KN), (2, 60.0 * KN)],
                },
                LoadCase {
                    name: "Queue, both lanes",
                    duration: 8.0,
                    loads: (0..5).map(|i| (i, 60.0 * KN)).collect(),
                },
            ],
            budget: Some(30_000.0),
            bonus_budget: Some(24_000.0),
            sag_limit: Some(0.04),
            allowed: steel_timber.clone(),
            sandbox: false,
        },
        Scenario {
            id: "S-00",
            title: "Free bench",
            rig: Rig::Sandbox,
            span: 0.0,
            anchors: vec![],
            load_points: vec![],
            load_cases: vec![LoadCase {
                name: "Gravity and your loads",
                duration: 20.0,
                loads: vec![],
            }],
            budget: None,
            bonus_budget: None,
            sag_limit: None,
            allowed: steel_timber,
            sandbox: true,
        },
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    fn footbridge() -> Scenario {
        catalogue().into_iter().find(|s| s.id == "B-02").unwrap()
    }

    #[test]
    fn load_cases_run_back_to_back() {
        let s = footbridge();
        assert_eq!(s.total_duration(), 12.0);
        assert_eq!(s.case_at(0.0), Some(0));
        assert_eq!(s.case_at(2.5), Some(1));
        assert_eq!(s.case_at(11.9), Some(2));
        assert_eq!(s.case_at(12.0), None);
        assert_eq!(s.case_starts(), vec![0.0, 2.0, 6.0]);
        assert_eq!(s.heaviest_case(), Some(2));
    }

    #[test]
    fn initial_design_locks_anchors_and_load_points() {
        let s = footbridge();
        let d = s.initial_design();
        assert_eq!(d.nodes.len(), 5);
        assert!(d.locked.iter().all(|l| *l));
        assert_eq!(d.supports[0], Some(SupportKind::Pinned));
        assert_eq!(d.nodes[s.load_point_node(1)], Vec3::ZERO);
        let f = s.case_forces(1);
        assert_eq!(f[1], (3, Vec3::new(0.0, -40_000.0, 0.0)));
        assert_eq!(f[0].1, Vec3::ZERO);
    }

    #[test]
    fn connectivity_needs_every_fixed_node_joined() {
        let s = footbridge();
        let mut d = s.initial_design();
        assert!(!s.is_connected(&d));
        // Chain the bottom chord: anchor, three load points, anchor.
        for (a, b) in [(0, 2), (2, 3), (3, 4)] {
            d.add_beam(a, b, BeamKind::SteelTube);
        }
        assert!(!s.is_connected(&d));
        d.add_beam(4, 1, BeamKind::SteelTube);
        assert!(s.is_connected(&d));
    }

    #[test]
    fn judging_separates_pass_criteria_from_bonus() {
        let s = footbridge();
        let mut r = TestRecord {
            completed: true,
            peak_sag: 0.01,
            peak_utilization: 0.9,
            cost: 12_000.0,
            ..Default::default()
        };
        let checks = judge(&s, &r);
        assert!(passed(&checks));
        assert!(checks.iter().any(|c| c.bonus && !c.ok));
        r.first_break = Some(7.2);
        assert!(!passed(&judge(&s, &r)));
    }

    #[test]
    fn money_groups_thousands() {
        assert_eq!(money(41_200.4), "41 200 kr");
        assert_eq!(money(950.0), "950 kr");
        assert_eq!(money(1_000_000.0), "1 000 000 kr");
    }
}
