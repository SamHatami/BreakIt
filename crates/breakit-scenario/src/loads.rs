//! Binding a scenario's loads to one particular structure.
//!
//! The load physics lives in `breakit_core::loads`: vehicles with sprung
//! axles on a lane captured from the player's nodes, and shake-table ground
//! motion. This module only does what is specific to scenarios:
//!
//! - turning the scenario's [`Event`]s into core [`LoadCase`]s
//!   ([`Binding::load_set`]): a `Convoy` becomes a traffic case on the deck
//!   lane, a `GroundShake` a harmonic quake;
//! - the deck lane itself ([`deck_lane`]: a strip `width` sideways and
//!   `height` vertically, load shared across both trusses) and the gap check;
//! - which nodes carry each payload's feet;
//! - the loads the core has no model for: a uniform deck load (crowds,
//!   ballast, the proof test's traffic stand-in) and sandbox player loads;
//! - a static preview of the loads over the test ([`Binding::forces_at`]) for
//!   build mode's static check and the brief.

use breakit_core::loads::seismic::{GroundMotion, Quake};
use breakit_core::loads::traffic::{Axle as CoreAxle, Drive, Lane, LaneLine, Traffic, Vehicle};
use breakit_core::loads::water::{Airy, Solitary, Water, Waves};
use breakit_core::loads::wind::{Gusts, Profile, Wind, AIR_DENSITY};
use breakit_core::loads::{LoadCase, LoadKind, LoadSet, Window, G};
use breakit_core::{NodeLoad, NodeSpring, Structure, Vec3};

use crate::scenario::{Deck, Event, Scenario, Wave};

fn v(p: [f64; 3]) -> Vec3 {
    Vec3::new(p[0], p[1], p[2])
}

/// Horizontal unit vector along `p` (+x if it has no horizontal part).
fn horizontal(p: [f64; 3]) -> Vec3 {
    let h = Vec3::new(p[0], 0.0, p[2]);
    if h.norm() > 1e-12 {
        h.normalize()
    } else {
        Vec3::x()
    }
}

/// The core lane for a scenario deck: a strip `width` either side of the
/// line and `height` up or down, sharing each station's load between its
/// nodes.
pub fn deck_lane(d: &Deck) -> LaneLine {
    LaneLine::strip(v(d.from), v(d.to), d.width, d.height)
}

/// The deck lane resolved against a structure: its stations in driving
/// order.
pub fn resolve_deck(d: &Deck, s: &Structure) -> Lane {
    let mut t = Traffic::new(vec![deck_lane(d)], Vec::new());
    t.prepare(s);
    t.lane(0).cloned().unwrap_or_default()
}

/// Horizontal stiffness given to a floating node in the static check, as a
/// share of its water-plane stiffness. Still water has none; this only
/// keeps a free raft from being a mechanism.
pub const FLOAT_DRIFT_STIFFNESS: f64 = 1e-3;

/// Where the scenario's loads land on one structure.
#[derive(Clone, Debug)]
pub struct Binding {
    /// The deck lane's stations on this structure, if the scenario has a
    /// deck.
    pub deck: Option<Lane>,
    /// Length of the deck line, m.
    pub deck_length: f64,
    /// Carrying node per foot of each payload, in scenario order; `None`
    /// when a foot has no node within tolerance (or two feet share one).
    pub payload_feet: Vec<Option<Vec<usize>>>,
    /// Translational mass per node including payloads, kg; 0 when fixed.
    pub node_mass: Vec<f64>,
    /// Constant extra loads (sandbox), ramped in over the first second.
    pub extra: Vec<(usize, Vec3)>,
    nodes: usize,
}

impl Binding {
    pub fn new(scenario: &Scenario, s: &Structure) -> Self {
        let deck = scenario.deck.as_ref().map(|d| resolve_deck(d, s));
        let deck_length = scenario
            .deck
            .as_ref()
            .map_or(0.0, |d| (v(d.to) - v(d.from)).norm());
        let payload_feet = scenario
            .payloads
            .iter()
            .map(|p| {
                let feet: Option<Vec<usize>> = p
                    .foot_positions()
                    .into_iter()
                    .map(|f| {
                        let at = v(f);
                        s.nodes
                            .iter()
                            .enumerate()
                            .map(|(i, n)| (i, (n - at).norm()))
                            .filter(|&(_, d)| d <= p.tolerance)
                            .min_by(|a, b| a.1.total_cmp(&b.1))
                            .map(|(i, _)| i)
                    })
                    .collect();
                feet.filter(|f| {
                    let mut u = f.clone();
                    u.sort_unstable();
                    u.dedup();
                    u.len() == f.len()
                })
            })
            .collect();
        Self {
            deck,
            deck_length,
            payload_feet,
            node_mass: vec![0.0; s.nodes.len()],
            extra: Vec::new(),
            nodes: s.nodes.len(),
        }
    }

    /// Every node on the deck lane.
    pub fn deck_nodes(&self) -> Vec<usize> {
        self.deck
            .iter()
            .flat_map(|l| l.stations.iter())
            .flat_map(|st| std::iter::once(st.node).chain(st.others.iter().copied()))
            .collect()
    }

    /// The largest unsupported stretch of lane and where it starts (m),
    /// counting the stretches from the lane ends to the first and last deck
    /// stations. `None` without a deck or with no deck nodes.
    pub fn largest_gap(&self) -> Option<(f64, f64)> {
        let lane = self.deck.as_ref()?;
        if lane.stations.is_empty() {
            return None;
        }
        let mut edges = vec![0.0];
        edges.extend(lane.stations.iter().map(|s| s.s));
        edges.push(self.deck_length);
        Some(
            edges
                .windows(2)
                .map(|w| (w[1] - w[0], w[0]))
                .fold((0.0, 0.0), |a, b| if b.0 > a.0 { b } else { a }),
        )
    }

    /// The core load cases for the scenario's events, with windows in
    /// solver time (`settle` seconds after the solver starts is test time
    /// 0). Uniform deck loads are not in it: see [`Self::deck_forces`].
    pub fn load_set(scenario: &Scenario, settle: f64) -> LoadSet {
        let mut cases = Vec::new();
        for e in &scenario.events {
            match e {
                Event::Convoy {
                    name,
                    start,
                    speed,
                    count,
                    spacing,
                    axles,
                } => {
                    let Some(d) = &scenario.deck else { continue };
                    let vehicles = (0..*count)
                        .map(|k| {
                            Vehicle::new(
                                format!("{name} {}", k + 1),
                                0,
                                axles
                                    .iter()
                                    .map(|a| CoreAxle::new(a.offset, a.load / G))
                                    .collect(),
                                Drive::cruise(*speed, -spacing * k as f64),
                            )
                        })
                        .collect();
                    cases.push(LoadCase::new(
                        name.clone(),
                        Window::new(start + settle, None, 0.0),
                        LoadKind::Traffic(Traffic::new(vec![deck_lane(d)], vehicles)),
                    ));
                }
                Event::GroundShake {
                    name,
                    start,
                    duration,
                    peak,
                    frequency,
                    direction,
                    fade,
                } => cases.push(LoadCase::new(
                    name.clone(),
                    Window::new(start + settle, Some(*duration), *fade),
                    LoadKind::Quake(Quake::new(GroundMotion::Harmonic {
                        amplitude: *peak,
                        frequency: *frequency,
                        duration: *duration,
                        direction: v(*direction),
                    })),
                )),
                Event::Wind {
                    name,
                    start,
                    duration,
                    ramp,
                    speed,
                    direction,
                    gusts,
                    jet,
                } => {
                    let mut w = Wind::new(*speed, v(*direction));
                    if *gusts > 0.0 {
                        w.gusts = Some(Gusts {
                            intensity: *gusts,
                            ..Default::default()
                        });
                    }
                    if let Some(j) = jet {
                        w.profile = Profile::Jet {
                            center: v(j.center),
                            half_width: j.half_width,
                            half_height: j.half_height,
                            edge: 1.0,
                        };
                    }
                    cases.push(LoadCase::new(
                        name.clone(),
                        Window::new(start + settle, *duration, *ramp),
                        LoadKind::Wind(w),
                    ));
                }
                Event::Water {
                    name,
                    start,
                    duration,
                    ramp,
                    bed,
                    level,
                    current,
                    wave,
                    paddle,
                } => {
                    let waves = wave.map(|w| match w {
                        Wave::Surge { height, direction } => {
                            // Crest 4/K behind the paddle, so the paddle
                            // makes the whole stroke as the wave leaves it.
                            let d = (level - bed).max(1e-6);
                            let kappa = (3.0 * height.max(0.0) / (4.0 * d.powi(3))).sqrt();
                            let lead = if kappa > 0.0 { 4.0 / kappa } else { 0.0 };
                            Waves::Solitary(Solitary {
                                height,
                                start: paddle - lead,
                                direction: horizontal(direction),
                            })
                        }
                        Wave::Regular {
                            height,
                            period,
                            direction,
                        } => {
                            let dir = horizontal(direction);
                            Waves::Airy(Airy {
                                height,
                                period,
                                direction: dir,
                                origin: dir * *paddle,
                            })
                        }
                    });
                    let w = Water {
                        bed: *bed,
                        level0: *level,
                        level1: *level,
                        current: v(*current),
                        waves,
                        paddle: *paddle,
                        ..Default::default()
                    };
                    cases.push(LoadCase::new(
                        name.clone(),
                        Window::new(start + settle, *duration, *ramp),
                        LoadKind::Water(w),
                    ));
                }
                Event::DeckLoad { .. } => {}
            }
        }
        LoadSet::new(cases)
    }

    /// What the scenario's still water does for the structure at rest, for
    /// the build-mode static check: buoyant node forces at the built
    /// position and the water-plane springs (core
    /// [`Water::static_support`]), at the level the water reaches by the end
    /// of the test. Every sprung node also gets a soft horizontal spring
    /// ([`FLOAT_DRIFT_STIFFNESS`] of its vertical one), so a structure held
    /// only by floats has a static solution. Empty without a water event.
    pub fn static_support(scenario: &Scenario, s: &Structure) -> (Vec<NodeLoad>, Vec<NodeSpring>) {
        let loads = Self::load_set(scenario, 0.0);
        let (mut forces, mut springs) = (Vec::new(), Vec::new());
        for c in &loads.cases {
            if let LoadKind::Water(w) = &c.kind {
                let t = (scenario.duration - c.window.start).max(0.0);
                let (f, k) = w.static_support(s, t);
                forces.extend(f);
                springs.extend(k);
            }
        }
        for k in &mut springs {
            let h = k.stiffness.y * FLOAT_DRIFT_STIFFNESS;
            k.stiffness.x += h;
            k.stiffness.z += h;
        }
        (forces, springs)
    }

    /// Ground (shake table) acceleration of `loads` at solver time `t`,
    /// m/s², as the quake cases apply it.
    pub fn ground_accel(loads: &LoadSet, t: f64) -> Vec3 {
        loads
            .cases
            .iter()
            .filter_map(|c| match &c.kind {
                LoadKind::Quake(q) => Some(
                    q.acceleration(t - c.window.start)
                        * (c.window.factor(t) * c.factor * loads.factor),
                ),
                _ => None,
            })
            .sum()
    }

    /// Wind force on each payload (indexed like `scenario.payloads`) at
    /// solver time `t`, N: `½ ρ C_d A |v| v` with the core wind velocity
    /// (gusts included) at the item's centre `centres[k]`.
    pub fn payload_wind(
        scenario: &Scenario,
        loads: &LoadSet,
        centres: &[Vec3],
        t: f64,
    ) -> Vec<Vec3> {
        let mut out = vec![Vec3::zeros(); scenario.payloads.len()];
        for c in &loads.cases {
            let LoadKind::Wind(w) = &c.kind else { continue };
            let ramp = c.window.factor(t);
            if ramp <= 0.0 {
                continue;
            }
            let dir = [w.direction.x, w.direction.y, w.direction.z];
            for ((p, at), f) in scenario.payloads.iter().zip(centres).zip(&mut out) {
                let a = p.wind_area(dir);
                if a <= 0.0 {
                    continue;
                }
                let u = w.velocity(*at, t - c.window.start, ramp);
                *f += u * (0.5 * w.air_density * p.drag * a * u.norm() * c.factor * loads.factor);
            }
        }
        out
    }

    /// The loads the core does not model, per node at test time `t`:
    /// uniform deck loads, the proof test's deck stand-in and sandbox
    /// player loads, all times the load factor.
    pub fn deck_forces(&self, scenario: &Scenario, t: f64) -> Vec<Vec3> {
        let mut out = vec![Vec3::zeros(); self.nodes];
        let factor = Self::factor_at(scenario, t);
        let proof = factor > 1.0 || (scenario.overload.is_some() && t > scenario.duration);
        let t_load = if proof { scenario.duration } else { t };
        for e in &scenario.events {
            if let Event::DeckLoad {
                start,
                ramp,
                per_metre,
                ..
            } = e
            {
                let r = ramp_in(t_load - start, *ramp);
                if r > 0.0 {
                    self.uniform_load(per_metre * r * factor, &mut out);
                }
            }
        }
        if let (true, Some(o)) = (proof, scenario.overload) {
            if let Some(w) = o.deck_per_metre {
                let r = ramp_in(t - scenario.duration, 1.0);
                self.uniform_load(w * r * factor, &mut out);
            }
        }
        let r = ramp_in(t, 1.0) * factor;
        for &(n, f) in &self.extra {
            out[n] += f * r;
        }
        out
    }

    /// Static preview of the scenario's loads at test time `t`: deck loads
    /// and player loads plus every vehicle's axle loads standing where the
    /// convoy would be, split between the stations either side. No bounce,
    /// no braking, no ground shaking: build mode's static check and the
    /// brief use it; the test itself runs the core models.
    pub fn forces_at(&self, scenario: &Scenario, t: f64) -> Vec<Vec3> {
        let mut out = self.deck_forces(scenario, t);
        // Mean wind on payloads, shared between the feet (no gusts, no
        // overturning couple).
        for e in &scenario.events {
            if let Event::Wind {
                start,
                duration,
                ramp,
                speed,
                direction,
                ..
            } = e
            {
                let r = Window::new(*start, *duration, *ramp).factor(t);
                let d = horizontal(*direction);
                for (p, feet) in scenario.payloads.iter().zip(&self.payload_feet) {
                    let Some(feet) = feet else { continue };
                    let f = d
                        * (0.5
                            * AIR_DENSITY
                            * p.drag
                            * p.wind_area(*direction)
                            * (speed * r).powi(2));
                    for &n in feet {
                        out[n] += f / feet.len() as f64;
                    }
                }
            }
        }
        if t > scenario.duration && scenario.overload.is_some() {
            return out;
        }
        let Some(lane) = &self.deck else { return out };
        if lane.stations.len() < 2 {
            return out;
        }
        for e in &scenario.events {
            if let Event::Convoy {
                start,
                speed,
                count,
                spacing,
                axles,
                ..
            } = e
            {
                if t < *start {
                    continue;
                }
                for k in 0..*count {
                    let front = (t - start) * speed - spacing * k as f64;
                    for a in axles {
                        point_load(lane, front - a.offset, a.load, &mut out);
                    }
                }
            }
        }
        out
    }

    fn uniform_load(&self, per_metre: f64, out: &mut [Vec3]) {
        let Some(lane) = &self.deck else { return };
        let st = &lane.stations;
        for k in 0..st.len() {
            let prev = if k == 0 { 0.0 } else { st[k - 1].s };
            let next = st.get(k + 1).map_or(self.deck_length, |n| n.s);
            lane.load(
                k,
                Vec3::new(0.0, -per_metre * 0.5 * (next - prev), 0.0),
                out,
            );
        }
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
}

/// A static downward point load at lane distance `s`, split between the
/// stations either side; off the deck it is on the ground.
fn point_load(lane: &Lane, s: f64, load: f64, out: &mut [Vec3]) {
    let st = &lane.stations;
    if s <= st[0].s || s >= st[st.len() - 1].s {
        return;
    }
    let k = st
        .partition_point(|x| x.s <= s)
        .saturating_sub(1)
        .min(st.len() - 2);
    let l = (s - st[k].s) / (st[k + 1].s - st[k].s);
    lane.load(k, Vec3::new(0.0, -load * (1.0 - l), 0.0), out);
    lane.load(k + 1, Vec3::new(0.0, -load * l, 0.0), out);
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
        Event::Wind {
            start, duration, ..
        }
        | Event::Water {
            start, duration, ..
        } => (*start, duration.map_or(f64::INFINITY, |d| start + d)),
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
        let lane = resolve_deck(&deck(4.0), &s);
        assert_eq!(lane.stations.len(), 5);
        let mut f = vec![Vec3::zeros(); 5];
        point_load(&lane, 1.25, 100.0, &mut f);
        let total: f64 = f.iter().map(|x| -x.y).sum();
        assert!((total - 100.0).abs() < 1e-9);
        assert!((f[1].y + 75.0).abs() < 1e-9 && (f[2].y + 25.0).abs() < 1e-9);
    }

    #[test]
    fn uniform_load_splits_with_the_ground_beyond_the_last_node() {
        let s = line(4);
        let sc = crate::builtin().remove(0);
        let mut b = Binding::new(&sc, &s);
        b.deck = Some(resolve_deck(&deck(5.0), &s));
        b.deck_length = 5.0;
        let mut f = vec![Vec3::zeros(); 5];
        b.uniform_load(10.0, &mut f);
        // The last metre of lane rests half on node 4, half on the ground.
        let total: f64 = f.iter().map(|x| -x.y).sum();
        assert!((total - 45.0).abs() < 1e-9, "{total}");
        assert_eq!(b.largest_gap(), Some((1.0, 0.0)));
    }
}
