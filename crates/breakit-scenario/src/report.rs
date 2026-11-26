//! Grading a finished run: pass or fail, the scored parts, stars, and the
//! report rows the results screen shows.

use std::fmt::Write as _;

use serde::{Deserialize, Serialize};

use crate::cost::Bill;
use crate::limits::PlayMode;
use crate::run::{Measured, TestRun, TimelineEntry};
use crate::scenario::{Objective, Scenario};

/// One row of the report: what was asked, what was measured.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Criterion {
    pub label: String,
    pub measured: String,
    pub required: String,
    pub ok: bool,
    /// Scored or bonus rows; `false` for the rows a pass needs.
    pub bonus: bool,
}

/// Each scored part, 0..1.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct ScoreParts {
    pub cost: f64,
    pub mass: f64,
    pub margin: f64,
    pub objectives: f64,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct BreakRecord {
    pub time: f64,
    pub member: usize,
    pub cause: String,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Report {
    pub scenario: String,
    pub sheet: String,
    pub title: String,
    pub mode: PlayMode,
    pub passed: bool,
    pub failure: Option<String>,
    pub bill: Bill,
    pub budget: Option<f64>,
    pub target_cost: f64,
    pub max_mass: Option<f64>,
    pub target_mass: f64,
    /// Highest utilisation of any intact member during the test, 1 = at
    /// capacity.
    pub peak_utilization: f64,
    /// 1 / peak utilisation.
    pub safety_margin: f64,
    pub peak_deck_sag: Option<f64>,
    pub payload_drift: Vec<(String, f64)>,
    pub first_damage: Option<f64>,
    pub first_break: Option<BreakRecord>,
    pub collapse_time: Option<f64>,
    /// Proof test load factor reached.
    pub reserve_factor: Option<f64>,
    /// Proof test: seconds from first damage to collapse.
    pub warning_time: Option<f64>,
    pub criteria: Vec<Criterion>,
    pub parts: ScoreParts,
    /// 0..1000, 0 on a fail.
    pub score: u32,
    /// 0 on a fail, 1 to 3 on a pass.
    pub stars: u8,
    /// Sandbox runs are not ranked.
    pub ranked: bool,
    pub timeline: Vec<TimelineEntry>,
}

/// Full marks at or under `target`, falling linearly to a quarter at the
/// cap; without a cap, `target / value`.
fn under_target(value: f64, target: f64, cap: Option<f64>) -> f64 {
    if value <= target {
        return 1.0;
    }
    match cap {
        Some(cap) if cap > target => {
            (1.0 - 0.75 * (value - target) / (cap - target)).clamp(0.0, 1.0)
        }
        _ => (target / value).clamp(0.0, 1.0),
    }
}

/// Full marks inside the band; below it falls to 0 at a margin of 1 (the
/// structure reached capacity), above it falls as `hi / margin`.
fn margin_score(margin: f64, [lo, hi]: [f64; 2]) -> f64 {
    if margin < lo {
        ((margin - 1.0) / (lo - 1.0).max(1e-9)).clamp(0.0, 1.0)
    } else if margin > hi {
        (hi / margin).clamp(0.0, 1.0)
    } else {
        1.0
    }
}

fn objective_row(o: &Objective, s: &Scenario, m: &Measured) -> Criterion {
    let mm = |x: f64| format!("{:.1} mm", x * 1000.0);
    let (label, measured, required, ok) = match o {
        Objective::DeckDeflection { limit } => {
            let sag = m.peak_deck_sag.unwrap_or(0.0);
            (
                "Deck sag".into(),
                mm(sag),
                format!("≤ {}", mm(*limit)),
                sag <= *limit,
            )
        }
        Objective::PayloadDrift { payload, limit } => {
            let k = s.payloads.iter().position(|p| &p.name == payload);
            let d = k
                .and_then(|k| m.payload_drift.get(k).copied())
                .unwrap_or(f64::INFINITY);
            (
                format!("{payload} movement"),
                if d.is_finite() { mm(d) } else { "n/a".into() },
                format!("≤ {}", mm(*limit)),
                d <= *limit,
            )
        }
        Objective::NoBreaks => (
            "No broken members".into(),
            match &m.first_break {
                Some((t, member, _)) => format!("member {member} broke at {t:.1} s"),
                None => "none".into(),
            },
            "0".into(),
            m.first_break.is_none() && m.passed == Some(true),
        ),
        Objective::GracefulFailure { warning } => {
            let w = warning_time(m);
            let measured = match (m.proof_held, w) {
                (true, _) => "did not fail".into(),
                (false, Some(w)) => format!("{w:.1} s warning"),
                _ => "not tested".into(),
            };
            (
                "Fails with warning".into(),
                measured,
                format!("≥ {warning:.1} s from first damage to collapse"),
                m.proof_held || w.is_some_and(|w| w >= *warning),
            )
        }
        Objective::Reserve { factor } => {
            let r = m.reserve_factor.unwrap_or(0.0);
            (
                "Reserve strength".into(),
                if m.reserve_factor.is_some() {
                    format!("× {r:.2}")
                } else {
                    "not tested".into()
                },
                format!("≥ × {factor:.2}"),
                r >= *factor,
            )
        }
    };
    Criterion {
        label,
        measured,
        required,
        ok,
        bonus: true,
    }
}

fn warning_time(m: &Measured) -> Option<f64> {
    match (m.proof_first_damage, m.proof_collapse) {
        (Some(d), Some(c)) => Some(c - d),
        (None, Some(_)) => Some(0.0),
        _ => None,
    }
}

impl Report {
    pub fn new(run: &TestRun) -> Self {
        let s = &run.scenario;
        let m = run.measured();
        let bill = run.bill.clone();
        let passed = m.passed == Some(true);
        let peak = m.peak_utilization;
        let margin = if peak > 0.0 {
            1.0 / peak
        } else {
            f64::INFINITY
        };
        let sc = &s.scoring;

        let mut criteria = vec![Criterion {
            label: "Survives the test".into(),
            measured: match (&m.failure, m.collapse_time) {
                (Some(f), Some(t)) => format!("collapsed at {t:.1} s: {f}"),
                _ if passed => "held".into(),
                _ => "unfinished".into(),
            },
            required: format!("{:.0} s", s.duration),
            ok: passed,
            bonus: false,
        }];
        if let Some(b) = s.limits.budget {
            criteria.push(Criterion {
                label: "Budget".into(),
                measured: format!("{:.0} kr", bill.total()),
                required: format!("≤ {b:.0} kr"),
                ok: bill.total() <= b,
                bonus: false,
            });
        }
        criteria.push(Criterion {
            label: "Cost".into(),
            measured: format!("{:.0} kr", bill.total()),
            required: format!("≤ {:.0} kr for full marks", sc.target_cost),
            ok: bill.total() <= sc.target_cost,
            bonus: true,
        });
        criteria.push(Criterion {
            label: "Weight".into(),
            measured: format!("{:.0} kg", bill.mass),
            required: format!("≤ {:.0} kg for full marks", sc.target_mass),
            ok: bill.mass <= sc.target_mass,
            bonus: true,
        });
        criteria.push(Criterion {
            label: "Safety margin".into(),
            measured: if margin.is_finite() {
                format!("{margin:.2}")
            } else {
                "unloaded".into()
            },
            required: format!("{:.2} to {:.2}", sc.margin_band[0], sc.margin_band[1]),
            ok: (sc.margin_band[0]..=sc.margin_band[1]).contains(&margin),
            bonus: true,
        });
        let objective_rows: Vec<Criterion> = s
            .objectives
            .iter()
            .map(|o| objective_row(o, s, m))
            .collect();
        let met = objective_rows.iter().filter(|c| c.ok).count();
        criteria.extend(objective_rows);

        let parts = ScoreParts {
            cost: under_target(bill.total(), sc.target_cost, s.limits.budget),
            mass: under_target(bill.mass, sc.target_mass, s.limits.max_mass),
            margin: margin_score(margin, sc.margin_band),
            objectives: if s.objectives.is_empty() {
                1.0
            } else {
                met as f64 / s.objectives.len() as f64
            },
        };
        let w = sc.weights;
        let wsum = (w.cost + w.mass + w.margin + w.objectives).max(1e-9);
        let frac = (w.cost * parts.cost
            + w.mass * parts.mass
            + w.margin * parts.margin
            + w.objectives * parts.objectives)
            / wsum;
        let (score, stars) = if passed {
            let stars = 1 + (frac >= sc.stars[0]) as u8 + (frac >= sc.stars[1]) as u8;
            ((frac * 1000.0).round() as u32, stars)
        } else {
            (0, 0)
        };

        Report {
            scenario: s.id.clone(),
            sheet: s.sheet.clone(),
            title: s.title.clone(),
            mode: run.mode,
            passed,
            failure: m.failure.clone(),
            budget: s.limits.budget,
            target_cost: sc.target_cost,
            max_mass: s.limits.max_mass,
            target_mass: sc.target_mass,
            peak_utilization: peak,
            safety_margin: margin,
            peak_deck_sag: m.peak_deck_sag,
            payload_drift: s
                .payloads
                .iter()
                .map(|p| p.name.clone())
                .zip(m.payload_drift.iter().copied())
                .collect(),
            first_damage: m.first_damage,
            first_break: m
                .first_break
                .as_ref()
                .map(|(t, member, cause)| BreakRecord {
                    time: *t,
                    member: *member,
                    cause: cause.clone(),
                }),
            collapse_time: m.collapse_time,
            reserve_factor: m.reserve_factor,
            warning_time: warning_time(m),
            criteria,
            parts,
            score,
            stars,
            ranked: run.mode == PlayMode::Challenge,
            timeline: run.timeline().to_vec(),
            bill,
        }
    }

    /// Plain-text summary for logs and the placeholder HUD.
    pub fn summary(&self) -> String {
        let mut s = String::new();
        let _ = writeln!(
            s,
            "{} {}: {}",
            self.sheet,
            self.title,
            if self.passed { "PASS" } else { "FAIL" }
        );
        if self.passed {
            let _ = writeln!(
                s,
                "score {} / 1000   {}{}",
                self.score,
                "*".repeat(self.stars as usize),
                if self.ranked {
                    ""
                } else {
                    "   (sandbox, unranked)"
                }
            );
        }
        for c in &self.criteria {
            let _ = writeln!(
                s,
                "[{}] {:<20} {:<24} {}",
                if c.ok { "x" } else { " " },
                c.label,
                c.measured,
                c.required
            );
        }
        if let Some(b) = &self.first_break {
            let _ = writeln!(
                s,
                "first break: member {} ({}) at {:.2} s",
                b.member, b.cause, b.time
            );
        }
        s
    }
}
