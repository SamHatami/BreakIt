//! The running playground: a [`DynamicSim`] of the player's structure and a
//! [`LoadSet`] that follows the load bench ([`Setup`]) while it runs.
//!
//! Wind, shaking and water each get one case the first time they are
//! switched on (also when on from the start: they fade in then too). Switching them off and on again fades the case's load
//! factor out and in over [`FADE_TIME`] instead of removing it, so no load
//! jumps; changing a setting rebuilds that one case in place (the others
//! keep their state). Trucks and dropped weights are new cases added when
//! sent. Gravity other than 1 g and clicked point loads go into the set's
//! constant loads, recomputed every frame.

use breakit_core::loads::seismic::{GroundMotion, Quake, Synthetic};
use breakit_core::loads::traffic::{Axle, Drive, LaneLine, Traffic, Vehicle};
use breakit_core::loads::water::{Water, Waves};
use breakit_core::loads::wind::{Gusts, Wind};
use breakit_core::loads::{impact::Impactor, Geometry, G};
use breakit_core::{
    DynamicSim, LoadCase, LoadKind, LoadSet, NodeLoad, RigState, SimConfig, Structure, Vec3, Window,
};

use crate::forces;
use crate::setup::{heading_vector, QuakeMotion, Setup, WaveKind};

/// Time a switched load takes to fade in or out, s.
pub const FADE_TIME: f64 = 1.0;
/// Time constant gravity and wind speed follow their settings with, s.
pub const FOLLOW_TIME: f64 = 0.5;
/// How long a steady sine plays: in practice until switched off, s.
const HARMONIC_SECONDS: f64 = 3600.0;
/// Strong-motion duration of a synthetic earthquake, s.
const SYNTHETIC_SECONDS: f64 = 20.0;
/// Length of a sine sweep, s.
const SWEEP_SECONDS: f64 = 30.0;
/// Lowest frequency of a sweep, Hz.
const SWEEP_FROM: f64 = 0.5;
/// Approach ramp before the first deck node, m.
const APPROACH: f64 = 8.0;
/// Axle spacing of the trucks, m.
const AXLE_SPACING: f64 = 1.4;
/// Radius of a dropped weight per cube root of its mass, m / kg^(1/3)
/// (a steel ball: 0.31 m for a tonne).
const DROP_RADIUS: f64 = 0.031;

/// What the bench needs to know about the table it stands on.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Env {
    /// Floor of the water, absolute y, m.
    pub water_bed: f64,
    /// Rest position of the wave paddle along x, m.
    pub paddle: f64,
}

impl Default for Env {
    fn default() -> Self {
        Self {
            water_bed: -4.0,
            paddle: -13.6,
        }
    }
}

/// The three switchable loads.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Slot {
    Wind = 0,
    Quake = 1,
    Water = 2,
}

/// Why something could not be sent.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum SendError {
    /// No level run of nodes to drive over.
    NoDeck,
}

impl std::fmt::Display for SendError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            SendError::NoDeck => write!(f, "no deck to drive on"),
        }
    }
}

impl std::error::Error for SendError {}

/// A live playground run.
pub struct Live {
    structure: Structure,
    sim: DynamicSim,
    loads: LoadSet,
    geometry: Geometry,
    setup: Setup,
    env: Env,
    lane: Option<LaneLine>,
    /// Case index per [`Slot`].
    slots: [Option<usize>; 3],
    /// Current fade factor per slot, 0..1.
    fade: [f64; 3],
    /// Wind speed and gravity as they follow the settings.
    wind_speed: f64,
    gravity: f64,
    sent: usize,
    /// Constant loads of the design itself, N.
    base: Vec<NodeLoad>,
}

impl Live {
    /// Starts a run of `structure` (its own weight under 1 g in `config`)
    /// with the bench as set.
    pub fn new(structure: Structure, config: SimConfig, setup: &Setup, env: Env) -> Self {
        let sim = DynamicSim::new(&structure, config);
        let mut loads = LoadSet::new(Vec::new());
        loads.prepare(&structure);
        let mut live = Self {
            geometry: Geometry::new(&structure),
            lane: deck_lane(&structure),
            sim,
            loads,
            setup: Setup::default(),
            env,
            slots: [None; 3],
            fade: [0.0; 3],
            wind_speed: setup.wind.speed,
            gravity: setup.gravity,
            sent: 0,
            base: Vec::new(),
            structure,
        };
        live.set_setup(setup);
        live
    }

    /// Adds constant loads the design carries (the build tool's point
    /// loads), on top of the bench's.
    pub fn with_loads(mut self, loads: Vec<NodeLoad>) -> Self {
        self.base = loads;
        self
    }

    pub fn sim(&self) -> &DynamicSim {
        &self.sim
    }

    pub fn structure(&self) -> &Structure {
        &self.structure
    }

    pub fn loads(&self) -> &LoadSet {
        &self.loads
    }

    pub fn setup(&self) -> &Setup {
        &self.setup
    }

    /// Simulated time since the start, s.
    pub fn time(&self) -> f64 {
        self.sim.time()
    }

    /// The lane trucks drive along, if the structure has a deck.
    pub fn lane(&self) -> Option<&LaneLine> {
        self.lane.as_ref()
    }

    /// Visible state of the wind, table, water, trucks and weights.
    pub fn rig_state(&self) -> RigState {
        self.loads.rig_state(self.sim.time())
    }

    /// Shake table displacement now, m.
    pub fn table_offset(&self) -> Vec3 {
        self.loads.table_offset(self.sim.time())
    }

    /// Fade factor of wind, shaking and water, 0..1.
    pub fn fades(&self) -> [f64; 3] {
        self.fade
    }

    /// Force on each node in the last frame, N.
    pub fn applied_forces(&self) -> &[Vec3] {
        self.loads.forces()
    }

    /// Reaction at every support, N.
    pub fn reactions(&self) -> Vec<(usize, Vec3)> {
        forces::reactions(
            &self.structure,
            &self.sim,
            &self.geometry,
            self.loads.forces(),
        )
    }

    /// Force member `m` puts on its end nodes, N.
    pub fn end_forces(&self, m: usize) -> (Vec3, Vec3) {
        forces::member_end_forces(&self.structure, &self.sim, &self.geometry, m)
    }

    /// Follows a changed bench: rebuilds the cases whose settings changed.
    pub fn set_setup(&mut self, new: &Setup) {
        let old = std::mem::replace(&mut self.setup, new.clone());
        let now = self.sim.time();
        let first = |slot: Slot, slots: &[Option<usize>; 3]| slots[slot as usize].is_none();

        let w = &new.wind;
        let wind_changed = old.wind.heading != w.heading
            || old.wind.gusts != w.gusts
            || old.wind.turbulence != w.turbulence;
        if w.on && (first(Slot::Wind, &self.slots) || wind_changed) {
            let mut wind = Wind::new(self.wind_speed, heading_vector(w.heading));
            if w.gusts {
                wind.gusts = Some(Gusts {
                    intensity: w.turbulence,
                    ..Default::default()
                });
            }
            self.put(Slot::Wind, "Wind", 0.0, LoadKind::Wind(wind));
        }

        let q = &new.quake;
        let quake_changed = old.quake != *q && (old.quake.on == q.on || q.on);
        if q.on && (first(Slot::Quake, &self.slots) || quake_changed) {
            self.put(Slot::Quake, "Shake table", now, LoadKind::Quake(quake(q)));
        }

        let wa = &new.water;
        let water_changed = old.water != *wa && (old.water.on == wa.on || wa.on);
        if wa.on && (first(Slot::Water, &self.slots) || water_changed) {
            let from = self.still_level().unwrap_or(self.env.water_bed);
            let water = water(wa, from, self.env);
            self.put(Slot::Water, "Water", now, LoadKind::Water(water));
        }
    }

    /// Still water level now (absolute y), if there is water.
    pub fn still_level(&self) -> Option<f64> {
        let i = self.slots[Slot::Water as usize]?;
        let c = &self.loads.cases[i];
        match &c.kind {
            LoadKind::Water(w) => Some(w.still_level(self.sim.time() - c.window.start)),
            _ => None,
        }
    }

    /// Water surface (absolute y) at (x, z) now, if the water is on.
    pub fn water_surface(&self, x: f64, z: f64) -> Option<f64> {
        let i = self.slots[Slot::Water as usize]?;
        if self.fade[Slot::Water as usize] <= 0.0 {
            return None;
        }
        let c = &self.loads.cases[i];
        match &c.kind {
            LoadKind::Water(w) => Some(w.surface_elevation(x, z, self.sim.time() - c.window.start)),
            _ => None,
        }
    }

    /// Replaces (or adds) the case in a slot, starting at `start`.
    fn put(&mut self, slot: Slot, name: &str, start: f64, kind: LoadKind) {
        let mut case = LoadCase::new(name, Window::new(start, None, 0.0), kind);
        case.factor = self.fade[slot as usize];
        match self.slots[slot as usize] {
            Some(i) => {
                match &mut case.kind {
                    LoadKind::Wind(w) => w.prepare(&self.structure),
                    LoadKind::Quake(q) => q.prepare(),
                    LoadKind::Water(w) => w.prepare(),
                    LoadKind::Traffic(t) => t.prepare(&self.structure),
                    LoadKind::Impact(x) => x.prepare(&self.structure),
                }
                self.loads.cases[i] = case;
            }
            None => {
                self.slots[slot as usize] = Some(self.loads.add_case(&self.structure, case));
            }
        }
    }

    fn apply_fades(&mut self) {
        for (k, slot) in self.slots.iter().enumerate() {
            if let Some(i) = slot {
                self.loads.cases[*i].factor = self.fade[k];
            }
        }
    }

    /// Sends a truck across the deck with the bench's traffic settings.
    pub fn send_truck(&mut self) -> Result<(), SendError> {
        let lane = self.lane.clone().ok_or(SendError::NoDeck)?;
        let t = &self.setup.traffic;
        let n = t.axles.clamp(2, 5) as usize;
        let per = t.mass * 1000.0 / n as f64;
        let axles = (0..n)
            .map(|i| Axle::new(i as f64 * AXLE_SPACING, per))
            .collect();
        self.sent += 1;
        let v = Vehicle::new(
            format!("Truck {}", self.sent),
            0,
            axles,
            Drive::cruise(t.speed, 0.0),
        )
        .with_cg_height(1.5);
        let case = LoadCase::new(
            format!("Truck {}", self.sent),
            Window::new(self.sim.time(), None, 0.0),
            LoadKind::Traffic(Traffic::new(vec![lane], vec![v])),
        );
        self.loads.add_case(&self.structure, case);
        Ok(())
    }

    /// Drops a weight from the bench's height above node `target`, or above
    /// the middle of the structure.
    pub fn drop_weight(&mut self, target: Option<usize>) {
        let d = &self.setup.drop;
        let n = self.sim.node_count();
        let (above, top) = match target.filter(|&t| t < n) {
            Some(t) => {
                let p = self.sim.position(t);
                (p, p.y)
            }
            None => {
                let mut c = Vec3::zeros();
                let mut top = f64::NEG_INFINITY;
                for i in 0..n {
                    let p = self.sim.position(i);
                    c += p;
                    top = top.max(p.y);
                }
                (c / n.max(1) as f64, top)
            }
        };
        let radius = DROP_RADIUS * d.mass.max(1.0).cbrt();
        let at = Vec3::new(above.x, top + d.height + radius, above.z);
        self.sent += 1;
        let case = LoadCase::new(
            format!("Weight {}", self.sent),
            Window::new(self.sim.time(), None, 0.0),
            LoadKind::Impact(Impactor::dropped("Weight", d.mass, radius, at)),
        );
        self.loads.add_case(&self.structure, case);
    }

    /// Advances one frame of `dt` seconds.
    pub fn step(&mut self, dt: f64) {
        let follow = 1.0 - (-dt / FOLLOW_TIME).exp();
        self.wind_speed += (self.setup.wind.speed - self.wind_speed) * follow;
        self.gravity += (self.setup.gravity - self.gravity) * follow;
        let on = [self.setup.wind.on, self.setup.quake.on, self.setup.water.on];
        for (f, on) in self.fade.iter_mut().zip(on) {
            let target = if on { 1.0 } else { 0.0 };
            let d = (target - *f).clamp(-dt / FADE_TIME, dt / FADE_TIME);
            *f += d;
        }
        self.apply_fades();
        if let Some(i) = self.slots[Slot::Wind as usize] {
            if let LoadKind::Wind(w) = &mut self.loads.cases[i].kind {
                w.speed = self.wind_speed;
            }
        }

        // Constant loads: extra gravity on every free node, then clicks.
        // The design's own loads come on over the first [`FADE_TIME`], like
        // everything else, instead of all at once.
        let ramp = breakit_core::loads::smooth_ramp(self.sim.time(), FADE_TIME);
        let mut fixed: Vec<NodeLoad> = self
            .base
            .iter()
            .map(|l| NodeLoad::force(l.node, l.force * ramp))
            .collect();
        let extra = (self.gravity - 1.0) * G;
        if extra.abs() > 1e-9 {
            for n in 0..self.sim.node_count() {
                let m = self.sim.node_mass(n);
                if m > 0.0 {
                    fixed.push(NodeLoad::force(n, Vec3::new(0.0, -m * extra, 0.0)));
                }
            }
        }
        for p in &self.setup.point_loads {
            if p.node < self.sim.node_count() {
                fixed.push(NodeLoad::force(p.node, Vec3::from(p.force)));
            }
        }
        self.loads.static_loads = fixed;
        self.loads.step(&self.structure, &mut self.sim, dt);
    }

    /// Gravity the structure feels now, multiple of g (follows the setting).
    pub fn gravity(&self) -> f64 {
        self.gravity
    }

    /// Wind speed now, m/s (follows the setting).
    pub fn wind_speed(&self) -> f64 {
        self.wind_speed
    }
}

/// The shake table motion for the settings.
fn quake(q: &crate::setup::QuakeSetup) -> Quake {
    let direction = heading_vector(q.heading);
    let a = q.amplitude * G;
    Quake::new(match q.motion {
        QuakeMotion::Harmonic => GroundMotion::Harmonic {
            amplitude: a,
            frequency: q.frequency,
            duration: HARMONIC_SECONDS,
            direction,
        },
        QuakeMotion::Synthetic => GroundMotion::Synthetic(Synthetic {
            pga: a,
            duration: SYNTHETIC_SECONDS,
            ground_frequency: q.frequency,
            direction,
            cross_ratio: 0.3,
            vertical_ratio: 0.5,
            seed: q.seed,
            ..Default::default()
        }),
        QuakeMotion::Sweep => GroundMotion::Sweep {
            amplitude: a,
            f0: SWEEP_FROM,
            f1: q.frequency.max(SWEEP_FROM + 0.5),
            duration: SWEEP_SECONDS,
            direction,
        },
    })
}

/// The water for the settings, rising or falling from `from` (absolute y).
fn water(w: &crate::setup::WaterSetup, from: f64, env: Env) -> Water {
    let dir = Vec3::x();
    let level = w.level.max(env.water_bed);
    let mut out = match w.waves {
        WaveKind::Calm => Water::current(level, dir * w.current),
        WaveKind::Regular => {
            let mut x = Water::airy(level, w.height, w.period, dir);
            x.current = dir * w.current;
            x
        }
        WaveKind::Surge => {
            // A crest well upstream of the paddle, so the wave runs in.
            let mut x = Water::solitary(level, w.height, 0.0, dir);
            let lead = x.paddle;
            if let Some(Waves::Solitary(s)) = &mut x.waves {
                s.start = env.paddle - lead;
            }
            x.current = dir * w.current;
            x
        }
    };
    out.bed = env.water_bed;
    out.paddle = env.paddle;
    out.level0 = from.max(env.water_bed);
    out.level1 = level;
    out.rise_time = ((level - out.level0).abs() / 0.5).max(0.5);
    out
}

/// The lane trucks drive along: the level with the most nodes, along x,
/// through their middle, from an approach ramp before the first node to past
/// the last. `None` if fewer than two nodes share a level or the run is under
/// 1 m long.
pub fn deck_lane(s: &Structure) -> Option<LaneLine> {
    let supported = |n: usize| s.supports.iter().any(|sp| sp.node == n);
    let mut levels: Vec<(i64, usize)> = Vec::new();
    for p in &s.nodes {
        let k = (p.y * 100.0).round() as i64;
        match levels.iter_mut().find(|(l, _)| *l == k) {
            Some(e) => e.1 += 1,
            None => levels.push((k, 1)),
        }
    }
    levels.sort_by(|a, b| b.1.cmp(&a.1).then(a.0.cmp(&b.0)));
    let (level, count) = *levels.first()?;
    if count < 2 {
        return None;
    }
    let y = level as f64 / 100.0;
    let on: Vec<(usize, &Vec3)> = s
        .nodes
        .iter()
        .enumerate()
        .filter(|(_, p)| ((p.y * 100.0).round() as i64) == level)
        .collect();
    let (lo, hi) = on
        .iter()
        .fold((f64::INFINITY, f64::NEG_INFINITY), |a, (_, p)| {
            (a.0.min(p.x), a.1.max(p.x))
        });
    if hi - lo < 1.0 || on.iter().all(|(i, _)| supported(*i)) {
        return None;
    }
    let zc = on.iter().map(|(_, p)| p.z).sum::<f64>() / on.len() as f64;
    let spread = on.iter().map(|(_, p)| (p.z - zc).abs()).fold(0.0, f64::max);
    Some(LaneLine::new(
        Vec3::new(lo - APPROACH, y, zc),
        Vec3::new(hi + APPROACH, y, zc),
        (spread + 0.05).max(0.3),
    ))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::setup::Param;
    use breakit_core::{Material, Section, Support};

    /// A 6 m Warren truss, pinned at both ends of the bottom chord.
    fn truss() -> Structure {
        let mut s = Structure::new();
        let (m, sec) = (Material::steel(), Section::tube(0.0571, 0.005));
        let bottom: Vec<usize> = (0..=6)
            .map(|i| s.add_node(Vec3::new(i as f64 - 3.0, 0.0, 0.0)))
            .collect();
        let top: Vec<usize> = (0..6)
            .map(|i| s.add_node(Vec3::new(i as f64 - 2.5, 1.0, 0.0)))
            .collect();
        for w in bottom.windows(2) {
            s.add_member(w[0], w[1], m.clone(), sec.clone());
        }
        for w in top.windows(2) {
            s.add_member(w[0], w[1], m.clone(), sec.clone());
        }
        for (i, &t) in top.iter().enumerate() {
            s.add_member(bottom[i], t, m.clone(), sec.clone());
            s.add_member(t, bottom[i + 1], m.clone(), sec.clone());
        }
        s.add_support(Support::pinned(bottom[0]));
        s.add_support(Support::pinned(bottom[6]));
        s
    }

    fn run(live: &mut Live, seconds: f64) {
        for _ in 0..(seconds * 60.0) as usize {
            live.step(1.0 / 60.0);
        }
    }

    fn total_reaction(live: &Live) -> Vec3 {
        live.reactions().iter().map(|(_, f)| f).sum()
    }

    #[test]
    fn finds_the_bottom_chord_as_the_deck() {
        let lane = deck_lane(&truss()).unwrap();
        assert_eq!(lane.start.y, 0.0);
        assert!(lane.start.x < -3.0 && lane.end.x > 3.0);
    }

    #[test]
    fn wind_switches_on_fades_in_and_pushes_sideways() {
        let s = truss();
        let mut setup = Setup::default();
        let mut live = Live::new(s, SimConfig::default(), &setup, Env::default());
        run(&mut live, 1.0);
        let calm = total_reaction(&live);
        assert!(calm.z.abs() < 100.0, "no wind yet: {calm:?}");
        setup.wind.on = true;
        setup.wind.gusts = false;
        setup.wind.speed = 40.0;
        setup.wind.heading = 90.0;
        live.set_setup(&setup);
        run(&mut live, 0.1);
        assert!(live.fades()[0] < 0.2, "fades in, not at once");
        run(&mut live, 3.0);
        assert_eq!(live.fades()[0], 1.0);
        let windy = total_reaction(&live);
        // Supports push back against the wind (blowing +z).
        assert!(windy.z < -500.0, "reaction {windy:?}");
        // Off again: fades out.
        setup.wind.on = false;
        live.set_setup(&setup);
        run(&mut live, 3.0);
        assert_eq!(live.fades()[0], 0.0);
        assert_eq!(live.loads().cases.len(), 1, "switching keeps one wind case");
    }

    #[test]
    fn gravity_scale_and_point_loads_change_the_reactions() {
        let s = truss();
        let mut setup = Setup::default();
        let mut live = Live::new(s, SimConfig::default(), &setup, Env::default());
        run(&mut live, 3.0);
        let w1 = total_reaction(&live).y;
        assert!(w1 > 0.0);
        Param::Gravity.set(&mut setup, 2.0);
        live.set_setup(&setup);
        run(&mut live, 4.0);
        let w2 = total_reaction(&live).y;
        assert!((w2 / w1 - 2.0).abs() < 0.1, "1 g {w1}, 2 g {w2}");
        setup.point_loads.push(crate::setup::PointLoad {
            node: 3,
            force: [0.0, -10_000.0, 0.0],
        });
        live.set_setup(&setup);
        run(&mut live, 3.0);
        let w3 = total_reaction(&live).y;
        assert!((w3 - w2 - 10_000.0).abs() < 1_000.0, "{w2} + 10 kN = {w3}?");
    }

    #[test]
    fn trucks_cross_and_weights_fall() {
        let s = truss();
        let setup = Setup::default();
        let mut live = Live::new(s, SimConfig::default(), &setup, Env::default());
        run(&mut live, 1.0);
        let before = total_reaction(&live).y;
        live.send_truck().unwrap();
        // 8 m approach at 8 m/s: on the deck after about a second.
        let mut peak: f64 = 0.0;
        for _ in 0..120 {
            live.step(1.0 / 60.0);
            peak = peak.max(total_reaction(&live).y - before);
        }
        let truck = setup.traffic.mass * 1000.0 * G;
        assert!(peak > 0.5 * truck, "peak {peak} of {truck}");
        assert_eq!(live.rig_state().vehicles.len(), 1);

        live.drop_weight(Some(3));
        let r = live.rig_state();
        assert_eq!(r.impactors.len(), 1);
        let y0 = r.impactors[0].position.y;
        run(&mut live, 0.3);
        assert!(live.rig_state().impactors[0].position.y < y0);
    }

    #[test]
    fn shaking_and_water_build_their_cases() {
        let s = truss();
        let mut setup = Setup::default();
        setup.quake.on = true;
        setup.water.on = true;
        setup.water.level = 0.5;
        let env = Env {
            water_bed: -3.0,
            paddle: -13.6,
        };
        let mut live = Live::new(s, SimConfig::default(), &setup, env);
        assert_eq!(live.fades(), [0.0, 0.0, 0.0]);
        run(&mut live, 2.0);
        assert!(live.table_offset().norm() > 0.0);
        let lvl = live.still_level().unwrap();
        assert!(lvl > -3.0 && lvl <= 0.5);
        // A new level rises from where the water is.
        setup.water.level = 1.0;
        live.set_setup(&setup);
        let again = live.still_level().unwrap();
        assert!((again - lvl).abs() < 0.05, "{lvl} then {again}");
        assert!(live.sim().position(3).iter().all(|v| v.is_finite()));
    }
}
