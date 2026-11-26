//! Scenario loads: wind, earthquakes, waves and floods, traffic and impacts.
//!
//! Every load is produced by a lab rig the player can see: a fan wall, a
//! hydraulic shake table, a wave flume with a paddle, test carts on a track,
//! a drop weight on the crane. The rig is only the visible source; the forces
//! come from the engineering models of the real thing:
//!
//! * [`wind`]: quasi-steady drag on every member, `q = ½ ρ C_d d |v_n| v_n`
//!   with the velocity relative to the moving member, drag coefficients by
//!   section shape and Reynolds number, a vertical profile (power law, log law
//!   or a fan jet), Kaimal-spectrum gusts carried downwind as frozen
//!   turbulence, and shielding of members behind other members.
//! * [`seismic`]: uniform base excitation in the table's frame. Supports stay
//!   put in the simulation; every free node feels `−m a_g(t)`, which is exact
//!   for a rigid table. The renderer offsets the table and the structure by
//!   the table displacement. Ground motion is a recorded accelerogram (PEER
//!   AT2 or raw samples), a Kanai-Tajimi / Clough-Penzien synthetic
//!   earthquake, or a harmonic test.
//! * [`water`]: Morison loads on every submerged member part (inertia +
//!   drag on the relative velocity), buoyancy, rising flood levels with a
//!   current, Airy waves with Wheeler stretching, and a solitary wave for a
//!   tsunami-like surge. The flume paddle motion follows piston wavemaker
//!   theory.
//! * [`traffic`]: axle loads moving along a lane of deck nodes.
//! * [`impact`]: rigid impactors (drop weight, cart) with a padded contact.
//!
//! A [`LoadSet`] holds the load cases of a scenario, turns them into one force
//! per node each frame and hands them to [`DynamicSim::set_force`]. It owns all
//! nodal forces, so constant loads of the scenario go into
//! [`LoadSet::static_loads`] too.
//!
//! Distributed loads are lumped to the end nodes of each member (half each,
//! linearly by position along the member). The solver represents a member's
//! bending by its end rotations only, so the extra span moment of a load
//! between the nodes (q L²/8 against q L²/12 at the ends) is not seen: long
//! members under wind or water should be split into several (`add_line`).
//!
//! Forces are evaluated at the start of each frame and held over its
//! substeps; loads varying faster than the frame rate are sampled once per
//! frame. The impactors integrate their own contact within the frame and
//! apply its average force, which conserves the impulse.

pub mod impact;
pub mod rng;
pub mod seismic;
pub mod traffic;
pub mod water;
pub mod wind;

use nalgebra::UnitQuaternion;

use crate::frame::member_axes;
use crate::section::Shape;
use crate::{DynamicSim, NodeLoad, Structure, Vec3};

/// Standard gravity used by the load models (buoyancy, vehicle weight),
/// independent of the gravity configured in the solver.
pub const G: f64 = 9.81;

/// When a load case acts, in scenario time (s).
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Window {
    /// Start time, s.
    pub start: f64,
    /// How long it acts, s. `None`: until the end of the run.
    pub duration: Option<f64>,
    /// Smooth ramp-in and ramp-out time, s. A suddenly applied load
    /// overshoots to about twice its static effect; a ramp of a few natural
    /// periods avoids that when the scenario does not want the shock.
    pub ramp: f64,
}

impl Default for Window {
    fn default() -> Self {
        Self {
            start: 0.0,
            duration: None,
            ramp: 1.0,
        }
    }
}

impl Window {
    pub fn new(start: f64, duration: Option<f64>, ramp: f64) -> Self {
        Self {
            start,
            duration,
            ramp,
        }
    }
    /// Ramp factor 0..1 at scenario time `t` (smoothstep in and out).
    pub fn factor(&self, t: f64) -> f64 {
        let tl = t - self.start;
        if tl < 0.0 {
            return 0.0;
        }
        let up = smooth_ramp(tl, self.ramp);
        match self.duration {
            Some(d) if tl > d => 0.0,
            Some(d) => up * smooth_ramp(d - tl, self.ramp),
            None => up,
        }
    }
    /// End time, if any.
    pub fn end(&self) -> Option<f64> {
        self.duration.map(|d| self.start + d)
    }
    /// True while `t` is inside the window.
    pub fn active(&self, t: f64) -> bool {
        t >= self.start && self.end().is_none_or(|e| t <= e)
    }
}

/// Smoothstep from 0 at `t = 0` to 1 at `t = ramp`.
pub fn smooth_ramp(t: f64, ramp: f64) -> f64 {
    if ramp <= 0.0 {
        return if t >= 0.0 { 1.0 } else { 0.0 };
    }
    let x = (t / ramp).clamp(0.0, 1.0);
    x * x * (3.0 - 2.0 * x)
}

/// One load of a scenario.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct LoadCase {
    /// Display name for the brief, timeline and chart recorder.
    pub name: String,
    pub window: Window,
    /// Load factor of this case (multiplies its forces; see [`LoadSet::factor`]).
    pub factor: f64,
    pub kind: LoadKind,
}

impl LoadCase {
    pub fn new(name: impl Into<String>, window: Window, kind: LoadKind) -> Self {
        Self {
            name: name.into(),
            window,
            factor: 1.0,
            kind,
        }
    }

    /// The headline intensity for the brief, with its unit, e.g.
    /// (0.35, "g") for an earthquake.
    pub fn intensity(&self) -> (f64, &'static str) {
        match &self.kind {
            LoadKind::Wind(w) => (w.speed, "m/s"),
            LoadKind::Quake(q) => (q.peak_acceleration() / G, "g"),
            LoadKind::Water(w) => w.intensity(),
            LoadKind::Traffic(t) => (t.total_load() / 1000.0, "kN"),
            LoadKind::Impact(i) => (i.energy() / 1000.0, "kJ"),
        }
    }
}

/// The load models. Each holds its parameters (scenario data) and, skipped
/// by serde, its runtime state.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum LoadKind {
    Wind(wind::Wind),
    Quake(seismic::Quake),
    Water(water::Water),
    Traffic(traffic::Traffic),
    Impact(impact::Impactor),
}

/// What a load model sees of the structure each frame.
pub struct View<'a> {
    pub structure: &'a Structure,
    pub sim: &'a DynamicSim,
    pub geometry: &'a Geometry,
    /// Scenario time, s.
    pub time: f64,
}

/// Cross-section data the load models need, per member.
#[derive(Clone, Debug)]
pub struct MemberGeometry {
    /// Rest local axes y and z (see [`crate::frame`]).
    pub rest_y: Vec3,
    pub rest_z: Vec3,
    /// Outer extent of the section along local y and along local z, m.
    pub depth_y: f64,
    pub depth_z: f64,
    /// Area enclosed by the outer outline, m² (a sealed tube displaces this).
    pub outer_area: f64,
    /// Material area, m² (what a flooded tube displaces).
    pub solid_area: f64,
    pub shape: Shape,
}

/// Per-member geometry cached when a [`LoadSet`] is prepared.
#[derive(Clone, Debug, Default)]
pub struct Geometry {
    pub members: Vec<MemberGeometry>,
}

impl Geometry {
    pub fn new(s: &Structure) -> Self {
        let members = s
            .members
            .iter()
            .map(|m| {
                let (_x, y, z) = member_axes(&s.nodes[m.a], &s.nodes[m.b]);
                let sec = &m.section;
                let (depth_y, depth_z, outer_area) = match sec.shape {
                    Shape::Rect { b, h } => (h, b, b * h),
                    Shape::Tube { ro, .. } => (2.0 * ro, 2.0 * ro, std::f64::consts::PI * ro * ro),
                };
                MemberGeometry {
                    rest_y: y,
                    rest_z: z,
                    depth_y,
                    depth_z,
                    outer_area,
                    solid_area: sec.area,
                    shape: sec.shape,
                }
            })
            .collect();
        Self { members }
    }
}

/// Current state of one unbroken member.
#[derive(Clone, Copy, Debug)]
pub struct MemberNow {
    pub a: usize,
    pub b: usize,
    pub xa: Vec3,
    pub xb: Vec3,
    pub va: Vec3,
    pub vb: Vec3,
    /// Unit chord a → b and current local axes y, z.
    pub x: Vec3,
    pub y: Vec3,
    pub z: Vec3,
    pub length: f64,
}

impl MemberNow {
    /// Point and velocity at fraction `s` (0 = a, 1 = b) along the chord.
    pub fn at(&self, s: f64) -> (Vec3, Vec3) {
        (
            self.xa + (self.xb - self.xa) * s,
            self.va + (self.vb - self.va) * s,
        )
    }
}

impl View<'_> {
    /// Current state of member `m`, or `None` if it is broken.
    pub fn member(&self, m: usize) -> Option<MemberNow> {
        if self.sim.is_broken(m) {
            return None;
        }
        let mem = &self.structure.members[m];
        let (xa, xb) = (self.sim.position(mem.a), self.sim.position(mem.b));
        let d = xb - xa;
        let length = d.norm();
        if length < 1e-12 {
            return None;
        }
        let x = d / length;
        let g = &self.geometry.members[m];
        let q: UnitQuaternion<f64> = self
            .sim
            .orientation(mem.a)
            .slerp(&self.sim.orientation(mem.b), 0.5);
        let mut y = q * g.rest_y;
        y -= x * x.dot(&y);
        let y = if y.norm() > 1e-9 {
            y.normalize()
        } else {
            member_axes(&xa, &xb).1
        };
        let z = x.cross(&y);
        Some(MemberNow {
            a: mem.a,
            b: mem.b,
            xa,
            xb,
            va: self.sim.velocity(mem.a),
            vb: self.sim.velocity(mem.b),
            x,
            y,
            z,
            length,
        })
    }

    /// Integrate a line load along member `mn` with `samples` midpoint
    /// samples and lump it onto the end nodes. `q(s, p, v)` returns the force
    /// per unit length (N/m) at fraction `s`, point `p`, member velocity `v`.
    /// Returns the total force.
    pub fn distribute(
        mn: &MemberNow,
        samples: usize,
        out: &mut [Vec3],
        mut q: impl FnMut(f64, Vec3, Vec3) -> Vec3,
    ) -> Vec3 {
        let n = samples.max(1);
        let ds = mn.length / n as f64;
        let mut total = Vec3::zeros();
        for i in 0..n {
            let s = (i as f64 + 0.5) / n as f64;
            let (p, v) = mn.at(s);
            let f = q(s, p, v) * ds;
            out[mn.a] += f * (1.0 - s);
            out[mn.b] += f * s;
            total += f;
        }
        total
    }
}

/// State of the visible rigs for the renderer and the sound, at one time.
#[derive(Clone, Debug, Default)]
pub struct RigState {
    pub wind: Vec<wind::WindRig>,
    pub table: Option<seismic::TableRig>,
    pub flume: Vec<water::FlumeRig>,
    pub vehicles: Vec<traffic::VehicleRig>,
    pub impactors: Vec<impact::ImpactorRig>,
}

/// The loads of a scenario, applied to a [`DynamicSim`] every frame.
///
/// ```ignore
/// let mut loads = LoadSet::new(vec![LoadCase::new("Gale", Window::default(),
///     LoadKind::Wind(Wind::new(25.0, Vec3::x())))]);
/// loads.prepare(&structure);
/// // each frame:
/// loads.step(&structure, &mut sim, 1.0 / 60.0);
/// let rigs = loads.rig_state(sim.time());
/// ```
#[derive(Clone, Debug, Default)]
pub struct LoadSet {
    pub cases: Vec<LoadCase>,
    /// Constant nodal loads (forces only; moments are ignored by the
    /// dynamic solver). Not ramped or factored.
    pub static_loads: Vec<NodeLoad>,
    /// Global load factor on every case, for proof tests that scale the
    /// scenario until collapse.
    pub factor: f64,
    geometry: Geometry,
    forces: Vec<Vec3>,
    scratch: Vec<Vec3>,
    totals: Vec<Vec3>,
    prepared: bool,
}

impl LoadSet {
    pub fn new(cases: Vec<LoadCase>) -> Self {
        Self {
            cases,
            factor: 1.0,
            ..Default::default()
        }
    }

    /// Resolve everything that depends on the structure (member geometry,
    /// lanes, impactor targets). Call once at test start, after the build is
    /// final. [`Self::apply`] calls it if it has not been called.
    pub fn prepare(&mut self, s: &Structure) {
        self.geometry = Geometry::new(s);
        for c in &mut self.cases {
            match &mut c.kind {
                LoadKind::Wind(w) => w.prepare(s),
                LoadKind::Quake(q) => q.prepare(),
                LoadKind::Water(w) => w.prepare(),
                LoadKind::Traffic(t) => t.prepare(s),
                LoadKind::Impact(i) => i.prepare(s),
            }
        }
        self.forces = vec![Vec3::zeros(); s.nodes.len()];
        self.scratch = vec![Vec3::zeros(); s.nodes.len()];
        self.totals = vec![Vec3::zeros(); self.cases.len()];
        self.prepared = true;
    }

    /// Compute every node's force for the frame starting at `sim.time()` and
    /// lasting `dt`, and set it on the simulation.
    pub fn apply(&mut self, s: &Structure, sim: &mut DynamicSim, dt: f64) {
        if !self.prepared || self.forces.len() != s.nodes.len() {
            self.prepare(s);
        }
        let t = sim.time();
        self.forces.iter_mut().for_each(|f| *f = Vec3::zeros());
        for l in &self.static_loads {
            self.forces[l.node] += l.force;
        }
        let view = View {
            structure: s,
            sim,
            geometry: &self.geometry,
            time: t,
        };
        for (ci, c) in self.cases.iter_mut().enumerate() {
            let ramp = c.window.factor(t);
            let scale = c.factor * self.factor;
            self.scratch.iter_mut().for_each(|f| *f = Vec3::zeros());
            let tl = t - c.window.start;
            match &mut c.kind {
                LoadKind::Wind(w) => w.apply(&view, tl, ramp * scale, &mut self.scratch),
                LoadKind::Quake(q) => q.apply(&view, tl, ramp * scale, &mut self.scratch),
                LoadKind::Water(w) => w.apply(&view, tl, ramp, scale, dt, &mut self.scratch),
                LoadKind::Traffic(tr) => tr.apply(&view, tl, ramp, scale, &mut self.scratch),
                LoadKind::Impact(i) => i.apply(&view, tl, scale, dt, &mut self.scratch),
            }
            let mut total = Vec3::zeros();
            for (f, d) in self.forces.iter_mut().zip(&self.scratch) {
                *f += d;
                total += d;
            }
            self.totals[ci] = total;
        }
        for (n, f) in self.forces.iter().enumerate() {
            sim.set_force(n, *f);
        }
    }

    /// [`Self::apply`] then [`DynamicSim::step`].
    pub fn step(&mut self, s: &Structure, sim: &mut DynamicSim, dt: f64) {
        self.apply(s, sim, dt);
        sim.step(dt);
    }

    /// Net force of each case in the last frame (N), in case order: base
    /// shear for an earthquake, total drag for wind, etc. For the chart
    /// recorder.
    pub fn totals(&self) -> &[Vec3] {
        &self.totals
    }

    /// Force on each node in the last frame (N).
    pub fn forces(&self) -> &[Vec3] {
        &self.forces
    }

    /// Displacement of the shake table at scenario time `t` (m): add it to
    /// every simulated position (and to the table) when drawing.
    pub fn table_offset(&self, t: f64) -> Vec3 {
        self.cases
            .iter()
            .filter_map(|c| match &c.kind {
                LoadKind::Quake(q) => {
                    Some(q.table(t - c.window.start, c.factor * self.factor).displacement)
                }
                _ => None,
            })
            .sum()
    }

    /// State of every rig at scenario time `t`.
    pub fn rig_state(&self, t: f64) -> RigState {
        let mut r = RigState::default();
        for c in &self.cases {
            let tl = t - c.window.start;
            let ramp = c.window.factor(t);
            let scale = c.factor * self.factor;
            match &c.kind {
                LoadKind::Wind(w) => r.wind.push(w.rig(tl, ramp)),
                LoadKind::Quake(q) => {
                    let tr = q.table(tl, scale);
                    r.table = Some(match r.table {
                        None => tr,
                        Some(a) => a.combine(&tr),
                    });
                }
                LoadKind::Water(w) => r.flume.push(w.rig(tl, ramp)),
                LoadKind::Traffic(tr) => r.vehicles.extend(tr.rig(tl)),
                LoadKind::Impact(i) => r.impactors.push(i.rig()),
            }
        }
        r
    }
}
