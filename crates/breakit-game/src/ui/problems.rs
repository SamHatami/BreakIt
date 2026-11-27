//! What stops the test, said so the player can act on it: what is wrong,
//! where it is, how to fix it, and the places to mark in the scene.
//!
//! The scenario's [`Violation`]s say *which* rule a design breaks; this
//! module adds the numbers and names that make the sentence specific (how
//! far out of the build area, which foundation is nearest, which members
//! cost the most) and turns the static solver's bare "unstable" into the
//! part of the structure that is loose.

use bevy::math::Vec3;
use breakit_scenario::Violation;

use super::GameView;
use crate::design::{joint_class, Design};
use crate::scenario::{money, Scenario};

/// Something in the scene a problem points at.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Spot {
    Node(usize),
    Beam(usize),
    Point(Vec3),
}

/// One reason the test cannot run, in the player's terms.
#[derive(Clone, Debug, PartialEq)]
pub struct Problem {
    /// Short headline: what is wrong.
    pub title: String,
    /// Where and by how much.
    pub detail: String,
    /// What to do about it.
    pub fix: String,
    /// Places to mark in the scene.
    pub at: Vec<Spot>,
}

impl Problem {
    fn new(title: impl Into<String>, detail: impl Into<String>, fix: impl Into<String>) -> Self {
        Self {
            title: title.into(),
            detail: detail.into(),
            fix: fix.into(),
            at: Vec::new(),
        }
    }

    fn at(mut self, spots: impl IntoIterator<Item = Spot>) -> Self {
        self.at.extend(spots);
        self
    }
}

fn v3(p: [f64; 3]) -> Vec3 {
    Vec3::new(p[0] as f32, p[1] as f32, p[2] as f32)
}

/// A length for a sentence: "35 cm" under a metre, "1.6 m" above.
fn len(m: f32) -> String {
    if m.abs() < 1.0 {
        format!("{:.0} cm", m * 100.0)
    } else {
        format!("{m:.1} m")
    }
}

/// Everything that blocks the test now, in the order the player should
/// deal with it: an empty or floating structure first, then the
/// scenario's limits.
pub fn problems(v: &GameView) -> Vec<Problem> {
    let s = v.sheets.current();
    let d = &v.design;
    let mut out: Vec<Problem> = v
        .check
        .violations
        .iter()
        .map(|vio| describe(vio, s, d, v))
        .collect();
    if let Some(p) = v.state.error.as_ref().and_then(|_| unstable(d)) {
        if !v.check.violations.contains(&Violation::NoSupports) {
            out.insert(0, p);
        }
    }
    out
}

/// Every problem's spots, numbered from 1 like the list.
pub fn spots(problems: &[Problem]) -> Vec<(usize, Spot)> {
    problems
        .iter()
        .enumerate()
        .flat_map(|(i, p)| p.at.iter().map(move |s| (i + 1, *s)))
        .collect()
}

fn anchor_spots(s: &Scenario) -> impl Iterator<Item = Spot> + '_ {
    s.anchors.iter().map(|(p, _)| Spot::Point(*p))
}

fn describe(vio: &Violation, s: &Scenario, d: &Design, v: &GameView) -> Problem {
    let def = &s.def;
    match vio {
        Violation::NoSupports if d.beams.is_empty() => Problem::new(
            "Nothing built yet",
            "The test needs a structure that stands on the sheet's foundations.",
            "Pick Draw (B), click a foundation marker to start, then click to chain beams.",
        )
        .at(anchor_spots(s)),
        Violation::NoSupports => Problem::new(
            "The structure stands on nothing",
            "None of its nodes sits on a foundation, so it would fall.",
            "Move a node onto a foundation marker (it becomes a support), or draw a beam down to one.",
        )
        .at(anchor_spots(s)),
        Violation::OutOfVolume { node } => {
            let p = d.nodes.get(*node).copied().unwrap_or_default();
            let (min, max) = (v3(def.limits.volume.min), v3(def.limits.volume.max));
            let mut over = Vec::new();
            for (axis, (lo, hi)) in [("left", "right"), ("low", "high"), ("far back", "far forward")]
                .into_iter()
                .enumerate()
            {
                let (x, a, b) = (p[axis], min[axis], max[axis]);
                if x < a {
                    over.push(format!("{} too {lo}", len(a - x)));
                } else if x > b {
                    over.push(format!("{} too {hi}", len(x - b)));
                }
            }
            let how = if over.is_empty() {
                "just outside".to_string()
            } else {
                over.join(" and ")
            };
            Problem::new(
                format!("Node {} is outside the build area", node + 1),
                format!(
                    "It is {how}. The build area runs {} wide, {} high and {} deep.",
                    len(max.x - min.x),
                    len(max.y - min.y),
                    len(max.z - min.z)
                ),
                "Delete it (X) or move it back inside.",
            )
            .at([Spot::Node(*node)])
        }
        Violation::SupportOffAnchor { node } => {
            let p = d.nodes.get(*node).copied().unwrap_or_default();
            let nearest = def
                .anchors
                .iter()
                .map(|a| (a, v3(a.position).distance(p)))
                .min_by(|a, b| a.1.total_cmp(&b.1));
            let detail = match nearest {
                Some((a, dist)) => format!(
                    "Supports only hold on a foundation. The nearest, {}, is {} away.",
                    a.name,
                    len(dist)
                ),
                None => "Supports only hold on a foundation, and this sheet marks none.".into(),
            };
            let mut prob = Problem::new(
                format!("Support at node {} is off the foundations", node + 1),
                detail,
                "Click the node with Support (S) until it shows none, or move it onto a foundation marker.",
            )
            .at([Spot::Node(*node)]);
            if let Some((a, _)) = nearest {
                prob = prob.at([Spot::Point(v3(a.position))]);
            }
            prob
        }
        Violation::AnchorTooWeak { node, anchor } => Problem::new(
            format!("Foundation {anchor} can only take a pin"),
            format!(
                "Node {} is fixed there, so it would twist the foundation off.",
                node + 1
            ),
            "Click the node twice with Support (S): fixed, none, pinned.",
        )
        .at([Spot::Node(*node)]),
        Violation::OverBudget { cost, budget } => {
            let top = priciest(d, v, |i| beam_cost(d, v, i));
            let names = top
                .iter()
                .map(|&(i, c)| format!("member {} ({})", i + 1, money(c)))
                .collect::<Vec<_>>()
                .join(", ");
            Problem::new(
                format!("Over budget by {}", money((cost - budget) as f32)),
                format!(
                    "It costs {} of {}. Most expensive: {names}.",
                    money(*cost as f32),
                    money(*budget as f32)
                ),
                "Select a member and pick a lighter part, or step its size down with [.",
            )
            .at(top.iter().map(|&(i, _)| Spot::Beam(i)))
        }
        Violation::OverMass { mass, max } => {
            let top = priciest(d, v, |i| d.beam_length(i) * d.beams[i].kind.mass_per_metre());
            let names = top
                .iter()
                .map(|&(i, m)| format!("member {} ({m:.0} kg)", i + 1))
                .collect::<Vec<_>>()
                .join(", ");
            Problem::new(
                format!("Too heavy by {:.0} kg", mass - max),
                format!("It weighs {mass:.0} kg of {max:.0} kg. Heaviest: {names}."),
                "Use smaller sections, timber or carbon, or fewer members.",
            )
            .at(top.iter().map(|&(i, _)| Spot::Beam(i)))
        }
        Violation::MaterialNotAllowed { member, material } => {
            let allowed = def
                .limits
                .materials
                .as_ref()
                .map(|m| m.iter().map(|c| c.label()).collect::<Vec<_>>().join(" or "))
                .unwrap_or_default();
            Problem::new(
                format!("Member {} is {}", member + 1, material.label()),
                format!("This sheet only allows {allowed}."),
                format!("Select it and pick a {allowed} part from the parts list."),
            )
            .at([Spot::Beam(*member)])
        }
        Violation::NoDeck => {
            let deck = def.deck.as_ref();
            let gap = deck.map(|k| len(k.max_gap as f32)).unwrap_or_default();
            let mut p = Problem::new(
                "Nothing carries the road",
                "The vehicles drive along the road line between the two marked ends, and no node is on it yet.",
                format!("Build a deck along that line: nodes at most {gap} apart, joined by beams."),
            );
            if let Some(k) = deck {
                p = p.at([Spot::Point(v3(k.from)), Spot::Point(v3(k.to))]);
            }
            p
        }
        Violation::DeckGap { at, gap, max } => {
            let mut p = Problem::new(
                format!("The road has a {} gap", len(*gap as f32)),
                format!(
                    "From {} to {} along the road there is no node, and a wheel falls through any gap over {}.",
                    len(*at as f32),
                    len((at + gap) as f32),
                    len(*max as f32)
                ),
                format!(
                    "Add deck nodes in that stretch, at most {} apart.",
                    len(*max as f32)
                ),
            );
            if let Some(k) = &def.deck {
                let (a, b) = (v3(k.from), v3(k.to));
                let dir = (b - a).normalize_or_zero();
                let mid = a + dir * (*at + gap * 0.5) as f32;
                p = p.at([Spot::Point(mid)]);
            }
            p
        }
        Violation::PayloadUnattached { payload } => {
            let feet: Vec<Vec3> = def
                .payloads
                .iter()
                .filter(|p| &p.name == payload)
                .flat_map(|p| {
                    let at = v3(p.at);
                    if p.feet.is_empty() {
                        vec![at]
                    } else {
                        p.feet.iter().map(|f| at + v3(*f)).collect()
                    }
                })
                .collect();
            let tol = def
                .payloads
                .iter()
                .find(|p| &p.name == payload)
                .map(|p| len(p.tolerance as f32))
                .unwrap_or_default();
            Problem::new(
                format!("Nothing holds the {payload}"),
                format!(
                    "It stands on {} feet (marked), and each needs a node within {tol}.",
                    feet.len()
                ),
                "Build up to each marked foot and connect a node to it.",
            )
            .at(feet.into_iter().map(Spot::Point))
        }
    }
}

/// The three members with the largest `value`.
fn priciest(d: &Design, _v: &GameView, value: impl Fn(usize) -> f32) -> Vec<(usize, f32)> {
    let mut all: Vec<(usize, f32)> = (0..d.beams.len()).map(|i| (i, value(i))).collect();
    all.sort_by(|a, b| b.1.total_cmp(&a.1));
    all.truncate(3);
    all
}

fn beam_cost(d: &Design, v: &GameView, i: usize) -> f32 {
    let costs = &v.sheets.costs;
    let joints: f32 = d
        .joints(i)
        .iter()
        .map(|j| costs.joints.price(joint_class(*j)) as f32)
        .sum();
    d.beam_length(i) * d.beams[i].kind.cost_per_metre(costs) + joints
}

/// Why the static solve failed, found from the design's shape: nodes on
/// no beam, pieces that reach no support, or (when every piece is held)
/// a frame that can fold.
pub fn unstable(d: &Design) -> Option<Problem> {
    let n = d.nodes.len();
    if n == 0 || d.beams.is_empty() {
        return None;
    }
    let mut degree = vec![0usize; n];
    let mut parent: Vec<usize> = (0..n).collect();
    fn find(p: &mut [usize], mut i: usize) -> usize {
        while p[i] != i {
            p[i] = p[p[i]];
            i = p[i];
        }
        i
    }
    for b in &d.beams {
        degree[b.a] += 1;
        degree[b.b] += 1;
        let (ra, rb) = (find(&mut parent, b.a), find(&mut parent, b.b));
        parent[ra] = rb;
    }
    let loose: Vec<usize> = (0..n)
        .filter(|&i| degree[i] == 0 && d.supports[i].is_none())
        .collect();
    if !loose.is_empty() {
        let list = node_list(&loose);
        return Some(
            Problem::new(
                if loose.len() == 1 {
                    format!("Node {list} is on no beam")
                } else {
                    format!("Nodes {list} are on no beam")
                },
                "A node needs at least one beam, or the solver cannot hold it in place.",
                "Delete it (X), or draw a beam to it (B).",
            )
            .at(loose.iter().map(|&i| Spot::Node(i))),
        );
    }
    let mut held = vec![false; n];
    for i in 0..n {
        if d.supports[i].is_some() {
            let r = find(&mut parent, i);
            held[r] = true;
        }
    }
    let floating: Vec<usize> = (0..n)
        .filter(|&i| degree[i] > 0 && !held[find(&mut parent, i)])
        .collect();
    if !floating.is_empty() {
        let beams: Vec<usize> = d
            .beams
            .iter()
            .enumerate()
            .filter(|(_, b)| floating.contains(&b.a))
            .map(|(i, _)| i)
            .collect();
        return Some(
            Problem::new(
                format!(
                    "{} not connected to a support",
                    plural(beams.len(), "member is", "members are")
                ),
                format!(
                    "Nodes {} hang in the air: nothing links them to a foundation.",
                    node_list(&floating)
                ),
                "Join that piece to the rest with a beam, or delete it.",
            )
            .at(beams.iter().map(|&i| Spot::Beam(i))),
        );
    }
    // Every piece reaches a support, so it can move as a mechanism: a
    // frame of pinned bars without triangles, or a single pinned support.
    let supports = d.supports.iter().filter(|s| s.is_some()).count();
    let fix = if supports < 2 {
        "Add a second support on another foundation, or make this one fixed (S)."
    } else {
        "Add diagonals so every panel is a triangle; rectangles of pinned bars fold."
    };
    Some(Problem::new(
        "The structure can fold or swing",
        "It is a mechanism: some part can move without bending any member.",
        fix,
    ))
}

fn plural(n: usize, one: &str, many: &str) -> String {
    if n == 1 {
        format!("1 {one}")
    } else {
        format!("{n} {many}")
    }
}

/// "3, 4 and 9" (1-based), shortened after six.
fn node_list(nodes: &[usize]) -> String {
    let mut names: Vec<String> = nodes.iter().take(6).map(|i| (i + 1).to_string()).collect();
    if nodes.len() > 6 {
        names.push(format!("{} more", nodes.len() - 6));
    }
    match names.len() {
        0 => String::new(),
        1 => names.remove(0),
        _ => {
            let last = names.pop().unwrap_or_default();
            format!("{} and {last}", names.join(", "))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::design::{BeamKind, SupportKind};

    fn bar(d: &mut Design, a: Vec3, b: Vec3) {
        let (a, b) = (d.add_node(a), d.add_node(b));
        d.add_beam(a, b, BeamKind::default());
    }

    #[test]
    fn a_stray_node_is_named() {
        let mut d = Design::default();
        bar(&mut d, Vec3::ZERO, Vec3::X);
        d.set_support(0, Some(SupportKind::Fixed));
        d.add_node(Vec3::Y * 3.0);
        let p = unstable(&d).expect("a problem");
        assert_eq!(p.title, "Node 3 is on no beam");
        assert_eq!(p.at, vec![Spot::Node(2)]);
    }

    #[test]
    fn a_floating_piece_points_at_its_members() {
        let mut d = Design::default();
        bar(&mut d, Vec3::ZERO, Vec3::X);
        d.set_support(0, Some(SupportKind::Fixed));
        bar(&mut d, Vec3::new(0.0, 2.0, 0.0), Vec3::new(1.0, 2.0, 0.0));
        let p = unstable(&d).expect("a problem");
        assert_eq!(p.title, "1 member is not connected to a support");
        assert_eq!(p.at, vec![Spot::Beam(1)]);
    }

    #[test]
    fn a_held_frame_that_folds_is_a_mechanism() {
        let mut d = Design::default();
        bar(&mut d, Vec3::ZERO, Vec3::X);
        d.set_support(0, Some(SupportKind::Pinned));
        let p = unstable(&d).expect("a problem");
        assert!(p.fix.contains("second support"));
    }

    #[test]
    fn node_lists_read_as_english() {
        assert_eq!(node_list(&[2]), "3");
        assert_eq!(node_list(&[0, 1, 4]), "1, 2 and 5");
    }

    #[test]
    fn lengths_pick_their_unit() {
        assert_eq!(len(0.35), "35 cm");
        assert_eq!(len(1.6), "1.6 m");
    }
}
