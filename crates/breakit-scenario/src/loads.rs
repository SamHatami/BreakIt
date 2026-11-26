//! Turning a scenario's loads into nodal forces on one particular structure.
//!
//! [`Binding`] finds which of the player's nodes are deck nodes and which
//! carry each payload. [`Binding::forces_at`] is then a pure function from
//! test time to one force per node, so the game, the headless evaluator
//! and the UI's chart recorder all see the same loads.

use breakit_core::{Structure, Vec3};

use crate::scenario::{Deck, Event, Scenario};

fn v(p: [f64; 3]) -> Vec3 {
    Vec3::new(p[0], p[1], p[2])
}

/// Nodes at one distance along the lane. Loads at a station split evenly
/// between its nodes (e.g. the two trusses of a box bridge).
#[derive(Clone, Debug, PartialEq)]
pub struct Station {
    /// Distance along the lane from `Deck::from`, m.
    pub s: f64,
    pub nodes: Vec<usize>,
}

/// The deck nodes of one structure, sorted along the lane.
#[derive(Clone, Debug, PartialEq)]
pub struct DeckNodes {
    pub origin: Vec3,
    pub dir: Vec3,
    pub length: f64,
    pub stations: Vec<Station>,
}

/// Stations closer than this along the lane merge into one, m.
const STATION_MERGE: f64 = 0.05;

impl DeckNodes {
    pub fn find(deck: &Deck, s: &Structure) -> Self {
        let origin = v(deck.from);
        let span = v(deck.to) - origin;
        let length = span.norm();
        let dir = span / length.max(1e-9);
        let mut on: Vec<(f64, usize)> = s
            .nodes
            .iter()
            .enumerate()
            .filter_map(|(i, p)| {
                let d = p - origin;
                let st = d.dot(&dir);
                let off = d - dir * st;
                let (vertical, sideways) = (off.y.abs(), (off - Vec3::y() * off.y).norm());
                (sideways <= deck.width
                    && vertical <= deck.height
                    && st >= -STATION_MERGE
                    && st <= length + STATION_MERGE)
                    .then_some((st.clamp(0.0, length), i))
            })
            .collect();
        on.sort_by(|a, b| a.0.total_cmp(&b.0));
        let mut stations: Vec<Station> = Vec::new();
        for (st, n) in on {
            match stations.last_mut() {
                Some(last) if st - last.s <= STATION_MERGE => last.nodes.push(n),
                _ => stations.push(Station {
                    s: st,
                    nodes: vec![n],
                }),
            }
        }
        Self {
            origin,
            dir,
            length,
            stations,
        }
    }

    pub fn nodes(&self) -> impl Iterator<Item = usize> + '_ {
        self.stations.iter().flat_map(|s| s.nodes.iter().copied())
    }

    /// The largest unsupported stretch of lane and where it starts, counting
    /// the stretches from the lane ends to the first and last deck nodes.
    pub fn largest_gap(&self) -> (f64, f64) {
        let mut edges = vec![0.0];
        edges.extend(self.stations.iter().map(|s| s.s));
        edges.push(self.length);
        edges
            .windows(2)
            .map(|w| (w[1] - w[0], w[0]))
            .fold((0.0, 0.0), |a, b| if b.0 > a.0 { b } else { a })
    }

    /// Spreads a downward point load at lane distance `s` onto the two
    /// stations either side of it (lever rule). Beyond the first or last
    /// station the share that would go to the lane end goes to the ground.
    pub fn point_load(&self, s: f64, load: f64, out: &mut [Vec3]) {
        if s < 0.0 || s > self.length || self.stations.is_empty() {
            return;
        }
        let i = self.stations.partition_point(|st| st.s <= s);
        let (left, right) = (
            i.checked_sub(1).map(|k| &self.stations[k]),
            self.stations.get(i),
        );
        let (s0, s1) = (
            left.map_or(0.0, |l| l.s),
            right.map_or(self.length, |r| r.s),
        );
        let t = if s1 > s0 { (s - s0) / (s1 - s0) } else { 0.0 };
        if let Some(l) = left {
            self.put(l, load * (1.0 - t), out);
        }
        if let Some(r) = right {
            self.put(r, load * t, out);
        }
    }

    /// Spreads a uniform downward load (N/m) along the whole lane.
    pub fn uniform_load(&self, per_metre: f64, out: &mut [Vec3]) {
        let n = self.stations.len();
        for (k, st) in self.stations.iter().enumerate() {
            let prev = if k == 0 { 0.0 } else { self.stations[k - 1].s };
            let next = if k + 1 == n {
                self.length
            } else {
                self.stations[k + 1].s
            };
            self.put(st, per_metre * 0.5 * (next - prev), out);
        }
    }

    fn put(&self, st: &Station, load: f64, out: &mut [Vec3]) {
        let each = load / st.nodes.len() as f64;
        for &n in &st.nodes {
            out[n].y -= each;
        }
    }

    pub fn point(&self, s: f64) -> Vec3 {
        self.origin + self.dir * s
    }
}

/// A vehicle on the deck at one instant, for drawing it.
#[derive(Clone, Debug, PartialEq)]
pub struct VehiclePose {
    pub event: usize,
    pub index: usize,
    /// Lane distance of the front axle, m (may be off the deck).
    pub front: f64,
    /// World position of the front axle on the undeformed lane.
    pub position: Vec3,
    pub dir: Vec3,
    pub length: f64,
}

/// Where the scenario's loads land on one structure.
#[derive(Clone, Debug)]
pub struct Binding {
    pub deck: Option<DeckNodes>,
    /// Carrying node per payload, in scenario order.
    pub payload_nodes: Vec<Option<usize>>,
    /// Translational mass per node including payloads, kg; 0 when fixed.
    pub node_mass: Vec<f64>,
    /// Constant extra loads (sandbox), ramped in over the first second.
    pub extra: Vec<(usize, Vec3)>,
    nodes: usize,
}

impl Binding {
    pub fn new(scenario: &Scenario, s: &Structure) -> Self {
        let deck = scenario.deck.as_ref().map(|d| DeckNodes::find(d, s));
        let payload_nodes = scenario
            .payloads
            .iter()
            .map(|p| {
                let at = v(p.at);
                s.nodes
                    .iter()
                    .enumerate()
                    .map(|(i, n)| (i, (n - at).norm()))
                    .filter(|&(_, d)| d <= p.tolerance)
                    .min_by(|a, b| a.1.total_cmp(&b.1))
                    .map(|(i, _)| i)
            })
            .collect();
        Self {
            deck,
            payload_nodes,
            node_mass: vec![0.0; s.nodes.len()],
            extra: Vec::new(),
            nodes: s.nodes.len(),
        }
    }

    /// External force per node at test time `t` (0 = end of settling), with
    /// every scenario load multiplied by `factor` (1 during the test, more
    /// in the proof test). Self weight and payload weight come from gravity
    /// in the solver, not from here.
    pub fn forces_at(&self, scenario: &Scenario, t: f64) -> Vec<Vec3> {
        let mut out = vec![Vec3::zeros(); self.nodes];
        let over = scenario.overload.filter(|_| t > scenario.duration);
        let (t_load, factor) = match over {
            Some(o) => {
                let f = (1.0 + o.rate * (t - scenario.duration)).min(o.max_factor);
                (scenario.duration, f)
            }
            None => (t, 1.0),
        };
        let deck = self.deck.as_ref();
        for e in &scenario.events {
            match e {
                Event::Convoy {
                    start,
                    speed,
                    count,
                    spacing,
                    axles,
                    ..
                } => {
                    // Traffic has driven off before the proof test.
                    let (Some(deck), None) = (deck, over) else {
                        continue;
                    };
                    for k in 0..*count {
                        let front = (t_load - start) * speed - *spacing * k as f64;
                        for a in axles {
                            deck.point_load(front - a.offset, a.load, &mut out);
                        }
                    }
                }
                Event::DeckLoad {
                    start,
                    ramp,
                    per_metre,
                    ..
                } => {
                    if let Some(deck) = deck {
                        let r = ramp_in(t_load - start, *ramp);
                        if r > 0.0 {
                            deck.uniform_load(per_metre * r * factor, &mut out);
                        }
                    }
                }
                Event::GroundShake {
                    start,
                    duration,
                    peak,
                    frequency,
                    direction,
                    fade,
                    ..
                } => {
                    if over.is_some() {
                        continue;
                    }
                    let a = shake(t - start, *duration, *peak, *frequency, *fade);
                    if a != 0.0 {
                        let d = v(*direction).normalize();
                        for (f, m) in out.iter_mut().zip(&self.node_mass) {
                            *f -= d * (m * a);
                        }
                    }
                }
            }
        }
        if let (Some(o), Some(deck)) = (over, deck) {
            if let Some(w) = o.deck_per_metre {
                let r = ramp_in(t - scenario.duration, 1.0);
                deck.uniform_load(w * r * factor, &mut out);
            }
        }
        let r = ramp_in(t, 1.0) * factor;
        for &(n, f) in &self.extra {
            out[n] += f * r;
        }
        out
    }

    /// Load factor at test time `t`: 1 during the test, rising in the proof
    /// test.
    pub fn factor_at(scenario: &Scenario, t: f64) -> f64 {
        match scenario.overload {
            Some(o) if t > scenario.duration => {
                (1.0 + o.rate * (t - scenario.duration)).min(o.max_factor)
            }
            _ => 1.0,
        }
    }

    /// Vehicles on or near the deck at test time `t`.
    pub fn vehicles_at(&self, scenario: &Scenario, t: f64) -> Vec<VehiclePose> {
        let Some(deck) = &self.deck else {
            return Vec::new();
        };
        if t > scenario.duration && scenario.overload.is_some() {
            return Vec::new();
        }
        let mut out = Vec::new();
        for (ei, e) in scenario.events.iter().enumerate() {
            if let Event::Convoy {
                start,
                speed,
                count,
                spacing,
                axles,
                ..
            } = e
            {
                let length = axles.iter().map(|a| a.offset).fold(0.0, f64::max);
                for k in 0..*count {
                    let front = (t - start) * speed - spacing * k as f64;
                    if front >= -2.0 && front - length <= deck.length + 2.0 {
                        out.push(VehiclePose {
                            event: ei,
                            index: k,
                            front,
                            position: deck.point(front),
                            dir: deck.dir,
                            length,
                        });
                    }
                }
            }
        }
        out
    }
}

/// 0 before `t = 0`, then linear up to 1 over `ramp` seconds.
fn ramp_in(t: f64, ramp: f64) -> f64 {
    if t <= 0.0 {
        0.0
    } else if ramp <= 0.0 {
        1.0
    } else {
        (t / ramp).min(1.0)
    }
}

fn shake(t: f64, duration: f64, peak: f64, frequency: f64, fade: f64) -> f64 {
    if t <= 0.0 || t >= duration {
        return 0.0;
    }
    let env = ramp_in(t, fade).min(ramp_in(duration - t, fade));
    peak * env * (std::f64::consts::TAU * frequency * t).sin()
}

/// Start and end of each event in test time, s. Convoys end when the last
/// axle of the last vehicle leaves a deck of `deck_length` metres.
pub fn event_window(e: &Event, deck_length: f64) -> (f64, f64) {
    match e {
        Event::Convoy {
            start,
            speed,
            count,
            spacing,
            axles,
            ..
        } => {
            let length = axles.iter().map(|a| a.offset).fold(0.0, f64::max);
            let travel = deck_length + length + spacing * count.saturating_sub(1) as f64;
            (*start, start + travel / speed.max(1e-6))
        }
        Event::DeckLoad { start, .. } => (*start, f64::INFINITY),
        Event::GroundShake {
            start, duration, ..
        } => (*start, start + duration),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use breakit_core::{Material, Section};

    fn line(n: usize) -> Structure {
        let mut s = Structure::new();
        s.add_line(
            Vec3::zeros(),
            Vec3::new(n as f64, 0.0, 0.0),
            n,
            &Material::steel(),
            &Section::tube(0.05, 0.005),
        );
        s
    }

    fn deck(len: f64) -> Deck {
        Deck {
            from: [0.0, 0.0, 0.0],
            to: [len, 0.0, 0.0],
            width: 0.3,
            height: 0.25,
            max_gap: 1.5,
        }
    }

    #[test]
    fn point_load_is_conserved_and_lever_ruled() {
        let s = line(4);
        let d = DeckNodes::find(&deck(4.0), &s);
        assert_eq!(d.stations.len(), 5);
        let mut f = vec![Vec3::zeros(); 5];
        d.point_load(1.25, 100.0, &mut f);
        let total: f64 = f.iter().map(|x| -x.y).sum();
        assert!((total - 100.0).abs() < 1e-9);
        assert!((f[1].y + 75.0).abs() < 1e-9 && (f[2].y + 25.0).abs() < 1e-9);
    }

    #[test]
    fn uniform_load_splits_with_the_ground_beyond_the_last_node() {
        let s = line(4);
        let d = DeckNodes::find(&deck(5.0), &s);
        let mut f = vec![Vec3::zeros(); 5];
        d.uniform_load(10.0, &mut f);
        // The last metre of lane rests half on node 4, half on the ground.
        let total: f64 = f.iter().map(|x| -x.y).sum();
        assert!((total - 45.0).abs() < 1e-9, "{total}");
        assert_eq!(d.largest_gap(), (1.0, 0.0));
    }
}
