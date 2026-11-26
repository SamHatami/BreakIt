//! The pre-test check: hard limits that block the test.

use std::fmt;

use serde::{Deserialize, Serialize};

use breakit_core::Structure;

use crate::cost::Bill;
use crate::loads::Binding;
use crate::scenario::{AnchorKind, MaterialClass, Scenario};

/// Challenge mode enforces every limit; sandbox drops the caps (volume,
/// anchors, budget, mass, materials) and keeps only what the test needs to
/// run at all.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum PlayMode {
    #[default]
    Challenge,
    Sandbox,
}

/// One reason the test cannot run. `Display` gives a plain sentence for
/// the UI.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum Violation {
    NoSupports,
    OutOfVolume {
        node: usize,
    },
    SupportOffAnchor {
        node: usize,
    },
    /// A support restrains rotation at an anchor that can only take a pin.
    AnchorTooWeak {
        node: usize,
        anchor: String,
    },
    OverBudget {
        cost: f64,
        budget: f64,
    },
    OverMass {
        mass: f64,
        max: f64,
    },
    MaterialNotAllowed {
        member: usize,
        material: MaterialClass,
    },
    NoDeck,
    DeckGap {
        at: f64,
        gap: f64,
        max: f64,
    },
    PayloadUnattached {
        payload: String,
    },
}

impl Violation {
    /// Whether sandbox mode still enforces it.
    pub fn blocks_sandbox(&self) -> bool {
        matches!(
            self,
            Violation::NoSupports
                | Violation::NoDeck
                | Violation::DeckGap { .. }
                | Violation::PayloadUnattached { .. }
        )
    }
}

impl fmt::Display for Violation {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Violation::NoSupports => write!(f, "The structure is not supported anywhere."),
            Violation::OutOfVolume { node } => {
                write!(f, "Node {node} is outside the build area.")
            }
            Violation::SupportOffAnchor { node } => {
                write!(f, "The support at node {node} is not on a foundation.")
            }
            Violation::AnchorTooWeak { node, anchor } => write!(
                f,
                "Foundation {anchor} only takes a pinned support, but node {node} is fixed."
            ),
            Violation::OverBudget { cost, budget } => write!(
                f,
                "Over budget: costs {cost:.0} kr, the budget is {budget:.0} kr."
            ),
            Violation::OverMass { mass, max } => write!(
                f,
                "Too heavy: weighs {mass:.0} kg, the limit is {max:.0} kg."
            ),
            Violation::MaterialNotAllowed { member, material } => write!(
                f,
                "Member {member} is {}, which this test does not allow.",
                material.label()
            ),
            Violation::NoDeck => write!(f, "Nothing carries the deck."),
            Violation::DeckGap { at, gap, max } => write!(
                f,
                "The deck has a {gap:.1} m gap at {at:.1} m along it; wheels need a node every {max:.1} m."
            ),
            Violation::PayloadUnattached { payload } => {
                write!(f, "Nothing holds the {payload}: each of its feet needs a node of its own.")
            }
        }
    }
}

/// Everything that stops this structure being tested, in a stable order.
pub fn check_limits(
    scenario: &Scenario,
    s: &Structure,
    bill: &Bill,
    mode: PlayMode,
) -> Vec<Violation> {
    let mut out = Vec::new();
    let lim = &scenario.limits;
    if s.supports.is_empty() {
        out.push(Violation::NoSupports);
    }
    for (i, p) in s.nodes.iter().enumerate() {
        if !lim.volume.contains([p.x, p.y, p.z], 1e-6) {
            out.push(Violation::OutOfVolume { node: i });
        }
    }
    for sup in &s.supports {
        let p = s.nodes[sup.node];
        let anchor = scenario.anchors.iter().find(|a| {
            let d = dist(a.position, [p.x, p.y, p.z]);
            d <= a.tolerance
        });
        match anchor {
            None => out.push(Violation::SupportOffAnchor { node: sup.node }),
            Some(a) => {
                let fixes_rotation = sup.fixed[3..].iter().any(|&r| r);
                if a.kind == AnchorKind::Pinned && fixes_rotation {
                    out.push(Violation::AnchorTooWeak {
                        node: sup.node,
                        anchor: a.name.clone(),
                    });
                }
            }
        }
    }
    if let Some(budget) = lim.budget {
        if bill.total() > budget {
            out.push(Violation::OverBudget {
                cost: bill.total(),
                budget,
            });
        }
    }
    if let Some(max) = lim.max_mass {
        if bill.mass > max {
            out.push(Violation::OverMass {
                mass: bill.mass,
                max,
            });
        }
    }
    if let Some(allowed) = &lim.materials {
        for (i, m) in s.members.iter().enumerate() {
            let c = MaterialClass::from(m.material.kind);
            if !allowed.contains(&c) {
                out.push(Violation::MaterialNotAllowed {
                    member: i,
                    material: c,
                });
            }
        }
    }
    let binding = Binding::new(scenario, s);
    if let Some(spec) = &scenario.deck {
        match binding.largest_gap() {
            None => out.push(Violation::NoDeck),
            Some((gap, at)) if gap > spec.max_gap => out.push(Violation::DeckGap {
                at,
                gap,
                max: spec.max_gap,
            }),
            Some(_) => {}
        }
    }
    for (p, n) in scenario.payloads.iter().zip(&binding.payload_feet) {
        if n.is_none() {
            out.push(Violation::PayloadUnattached {
                payload: p.name.clone(),
            });
        }
    }
    if mode == PlayMode::Sandbox {
        out.retain(Violation::blocks_sandbox);
    }
    out
}

fn dist(a: [f64; 3], b: [f64; 3]) -> f64 {
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
}
