//! Water: flood levels, currents, waves and tsunami-like surges, applied as
//! Morison loads and buoyancy on every submerged part of the structure.
//!
//! The lab rig is a wave flume: the table floods to a still water level
//! (optionally rising over time), a pump drives a uniform current, and a
//! piston paddle makes either regular (Airy) waves or a single solitary wave.
//!
//! # Model
//!
//! Water kinematics (velocity `u` and local acceleration `∂u/∂t`) come from:
//!
//! * a uniform current, optionally with a 1/7 power-law profile from the bed;
//! * linear (Airy) waves: dispersion `ω² = g k tanh(k d)` solved by Newton,
//!   depth decay `cosh k(z+d) / sinh kd`, Wheeler stretching of the column so
//!   the kinematics reach the crest and vanish above the surface;
//! * a solitary wave `η = H sech²(K(ξ − c t))`, `K = √(3H/4d³)`,
//!   `c = √(g(d+H))`, with depth-uniform shallow-water velocity
//!   `u = c η/(d+η)` and the vertical velocity from continuity.
//!
//! Every unbroken member is sampled along its length; a sample is wet when it
//! is below the local surface elevation. On wet samples the force per unit
//! length is Morison's equation, normal to the member axis,
//!
//! `f = ρ C_m A a_n + ½ ρ C_d D |u_rel,n| u_rel,n`,
//!
//! with `u_rel` the water velocity relative to the moving member (so the
//! drag also damps the structure), plus buoyancy `ρ g A_disp` upward.
//!
//! Simplifications: members are slender (`D/L ≪ 1`, `D/λ < 0.2`, no
//! diffraction), there is no slamming or impact term when a dry member is hit
//! by the crest, no wave breaking (a solitary wave above `H/d ≈ 0.78` would
//! break in reality), no current–wave interaction (Doppler shift), and the
//! added mass `ρ C_a A ẍ` of the structure's own acceleration is left out (it
//! would need an implicit mass term in the solver). Waves pass through the
//! structure unchanged. Flood rise is quasi-static: the rising level adds no
//! velocity of its own.
//!
//! The case's window ramp scales the depth above the bed, the wave height and
//! the current together from 0 to full.
//!
//! # Simulated water
//!
//! The analytic waves above pass through the structure and never break. A
//! water simulation (the `breakit-water` crate) can replace them: put its
//! current state into [`Water::field`] each frame and the same Morison and
//! buoyancy loads are evaluated on the simulated surface and velocities
//! instead, so what the player sees is what pushes on the structure. The
//! window ramp then scales the forces, since the simulated water is already
//! there.

use std::sync::Arc;

use super::{smooth_ramp, View, G};
use crate::section::Shape;
use crate::Vec3;

/// Water kinematics from a simulation, read by [`Water::apply`] in place of
/// the analytic [`Sea`]. See the module docs.
pub trait WaterField: std::fmt::Debug + Send + Sync {
    /// Lowest bed level under the water, absolute y, m.
    fn bed(&self) -> f64;
    /// Upper bound of the water surface anywhere, absolute y, m.
    fn max_surface(&self) -> f64;
    /// Water surface at horizontal position (x, z), absolute y, m, or
    /// `None` where it is dry.
    fn surface(&self, x: f64, z: f64) -> Option<f64>;
    /// Water velocity (m/s) and local acceleration `∂u/∂t` (m/s²) at `p`,
    /// or `None` if `p` is dry (above the surface or below the bed).
    fn kinematics(&self, p: Vec3) -> Option<(Vec3, Vec3)>;
}

/// Shape of the current velocity over the depth.
#[derive(Clone, Copy, Debug, PartialEq, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum CurrentProfile {
    /// The same velocity from bed to surface.
    #[default]
    Uniform,
    /// `u(z) = (8/7) U (z/d)^(1/7)` with `z` above the bed: the given current
    /// is the depth average, the surface runs 14 % faster.
    PowerLaw,
}

/// Linear (Airy) regular waves.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Airy {
    /// Crest to trough height H, m.
    pub height: f64,
    /// Period T, s.
    pub period: f64,
    /// Horizontal unit direction of travel.
    pub direction: Vec3,
    /// A point where a crest passes at t = 0 (phase `θ = k (p − origin)·dir − ω t`).
    pub origin: Vec3,
}

/// A single solitary wave: the flume's stand-in for a tsunami surge.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Solitary {
    /// Crest height above still water H, m.
    pub height: f64,
    /// Crest position at t = 0, measured along `direction` from the world
    /// origin (`ξ = p·dir`), m. The wave is effectively `±4/K` long, so put
    /// the crest about `4/K` behind the paddle ([`Water::paddle`]); the
    /// kinematics are not clipped at the paddle, the structure belongs
    /// downstream of it.
    pub start: f64,
    /// Horizontal unit direction of travel.
    pub direction: Vec3,
}

/// Waves on top of the still level and current.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Waves {
    Airy(Airy),
    Solitary(Solitary),
}

/// Morison coefficients by section shape.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Coefficients {
    /// Drag coefficient of a circular tube (post-critical rough ≈ 0.7–1.05).
    pub tube_cd: f64,
    /// Added-mass coefficient of a tube (potential flow: 1), `C_m = 1 + C_a`.
    pub tube_ca: f64,
    /// Drag coefficient of a rectangular section (square, sharp corners ≈ 2).
    pub rect_cd: f64,
    /// Added-mass coefficient of a rectangle (square, DNV-RP-C205: 1.51).
    pub rect_ca: f64,
}

impl Default for Coefficients {
    fn default() -> Self {
        Self {
            tube_cd: 1.0,
            tube_ca: 1.0,
            rect_cd: 2.0,
            rect_ca: 1.51,
        }
    }
}

/// Runtime state: the solitary-wave paddle integrated by Goring's method.
#[derive(Clone, Debug, Default)]
pub struct PaddleState {
    initialized: bool,
    time: f64,
    position: f64,
    velocity: f64,
}

/// A water load case.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Water {
    /// Bed level (the flume floor), absolute y, m. The table top is 0.
    pub bed: f64,
    /// Still water level at the start of the case, absolute y, m.
    pub level0: f64,
    /// Still water level after `rise_time`, absolute y, m.
    pub level1: f64,
    /// Time for the level to rise from `level0` to `level1` (smoothstep), s.
    pub rise_time: f64,
    /// Current velocity (horizontal), m/s. Depth average for the power law.
    pub current: Vec3,
    pub profile: CurrentProfile,
    pub waves: Option<Waves>,
    /// Water density, kg/m³ (1000: fresh water in the flume; sea 1025).
    pub density: f64,
    pub coefficients: Coefficients,
    /// Sealed tubes displace their outer area; flooded ones only their
    /// material.
    pub sealed: bool,
    /// Integration samples per member (8 resolves a member crossing the
    /// surface to 1/8 of its length).
    pub samples: usize,
    /// Rest position of the paddle face along the wave direction (`ξ = p·dir`), m.
    pub paddle: f64,
    /// Simulated water to take the kinematics from instead of the analytic
    /// waves (see [`WaterField`]). Runtime state: set each frame by whoever
    /// runs the simulation, `None` for the analytic model.
    #[cfg_attr(feature = "serde", serde(skip))]
    pub field: Option<Arc<dyn WaterField>>,
    #[cfg_attr(feature = "serde", serde(skip))]
    pub state: PaddleState,
}

impl Default for Water {
    fn default() -> Self {
        Self {
            bed: 0.0,
            level0: 0.0,
            level1: 0.0,
            rise_time: 0.0,
            current: Vec3::zeros(),
            profile: CurrentProfile::Uniform,
            waves: None,
            density: 1000.0,
            coefficients: Coefficients::default(),
            sealed: true,
            samples: 8,
            paddle: -5.0,
            field: None,
            state: PaddleState::default(),
        }
    }
}

/// State of the flume for the renderer and the sound.
#[derive(Clone, Debug, Default)]
pub struct FlumeRig {
    /// Bed level, m.
    pub bed: f64,
    /// Current still water level, absolute y, m.
    pub still_level: f64,
    /// Still water depth above the bed, m.
    pub depth: f64,
    /// Rest position of the paddle face along `direction`, m.
    pub paddle_rest: f64,
    /// Paddle displacement from rest along `direction`, m.
    pub paddle_position: f64,
    /// Paddle velocity along `direction`, m/s.
    pub paddle_velocity: f64,
    /// Current target wave height (ramped), m. 0 without waves.
    pub wave_height: f64,
    /// Wave period, s. 0 for a solitary wave or no waves.
    pub wave_period: f64,
    /// Horizontal direction of the waves (or of the current without waves).
    pub direction: Vec3,
    /// Current speed (depth average, ramped), m/s.
    pub current_speed: f64,
}

/// The waves of a [`Sea`] at one instant, with derived constants.
#[derive(Clone, Copy, Debug)]
pub enum SeaWave {
    Calm,
    Airy {
        amplitude: f64,
        k: f64,
        omega: f64,
        direction: Vec3,
        /// `k origin·dir`.
        phase: f64,
    },
    Solitary {
        height: f64,
        kappa: f64,
        celerity: f64,
        direction: Vec3,
        start: f64,
    },
}

/// Snapshot of the water at time `t` with a given ramp: what the loads and
/// the renderer's water surface read.
#[derive(Clone, Copy, Debug)]
pub struct Sea {
    pub bed: f64,
    /// Still water depth, m (0: dry).
    pub depth: f64,
    pub current: Vec3,
    pub profile: CurrentProfile,
    pub wave: SeaWave,
    /// Time since case start, s.
    pub t: f64,
}

/// Wave number k (rad/m) of linear waves of angular frequency `omega` in
/// depth `depth`: solves `ω² = g k tanh(k d)` by Newton from Eckart's guess.
pub fn wave_number(omega: f64, depth: f64) -> f64 {
    if omega <= 0.0 {
        return 0.0;
    }
    let d = depth.max(1e-9);
    let k0 = omega * omega / G;
    let mut k = k0 / (k0 * d).tanh().sqrt();
    for _ in 0..50 {
        let th = (k * d).tanh();
        let f = G * k * th - omega * omega;
        let df = G * th + G * k * d * (1.0 - th * th);
        let dk = f / df;
        k -= dk;
        if dk.abs() < 1e-14 * k {
            break;
        }
    }
    k
}

/// Piston wavemaker transfer function (Biésel): wave height over paddle
/// stroke, `H/S = 2(cosh 2kd − 1)/(sinh 2kd + 2kd)`. Tends to 2 in deep water
/// and to `kd` in shallow water.
pub fn piston_height_to_stroke(kd: f64) -> f64 {
    if kd <= 0.0 {
        return 0.0;
    }
    // Divided through by sinh 2kd: 2 tanh(kd) / (1 + 2kd / sinh 2kd).
    let x = 2.0 * kd;
    let r = if x > 700.0 { 0.0 } else { x / x.sinh() };
    2.0 * kd.tanh() / (1.0 + r)
}

/// `cosh(k s)/sinh(k d)` and `sinh(k s)/sinh(k d)` without overflow in deep water.
fn depth_decay(k: f64, s: f64, d: f64) -> (f64, f64) {
    let den = -(-2.0 * k * d).exp_m1();
    if den <= 0.0 {
        return (0.0, 0.0);
    }
    let e1 = (k * (s - d)).exp();
    let e2 = (-k * (s + d)).exp();
    ((e1 + e2) / den, (e1 - e2) / den)
}

/// Horizontal unit vector of `d`, or +x.
fn horizontal(d: Vec3) -> Vec3 {
    let h = Vec3::new(d.x, 0.0, d.z);
    let n = h.norm();
    if n > 1e-12 {
        h / n
    } else {
        Vec3::x()
    }
}

impl Sea {
    /// Still water level, absolute y.
    pub fn still_level(&self) -> f64 {
        self.bed + self.depth
    }

    /// Surface elevation above the still level at horizontal position (x, z).
    pub fn eta(&self, x: f64, z: f64) -> f64 {
        if self.depth <= 0.0 {
            return 0.0;
        }
        let p = Vec3::new(x, 0.0, z);
        match self.wave {
            SeaWave::Calm => 0.0,
            SeaWave::Airy {
                amplitude,
                k,
                omega,
                direction,
                phase,
            } => amplitude * (k * p.dot(&direction) - phase - omega * self.t).cos(),
            SeaWave::Solitary {
                height,
                kappa,
                celerity,
                direction,
                start,
            } => {
                let th = kappa * (p.dot(&direction) - start - celerity * self.t);
                height * sech2(th)
            }
        }
    }

    /// Water surface, absolute y.
    pub fn surface(&self, x: f64, z: f64) -> f64 {
        self.still_level() + self.eta(x, z)
    }

    /// Upper bound of the surface anywhere, absolute y.
    pub fn max_surface(&self) -> f64 {
        self.still_level()
            + match self.wave {
                SeaWave::Calm => 0.0,
                SeaWave::Airy { amplitude, .. } => amplitude.abs(),
                SeaWave::Solitary { height, .. } => height.max(0.0),
            }
    }

    /// Water velocity and local acceleration at `p`, or `None` if `p` is dry
    /// (above the surface or below the bed).
    pub fn kinematics(&self, p: Vec3) -> Option<(Vec3, Vec3)> {
        let d = self.depth;
        if d <= 0.0 || p.y < self.bed {
            return None;
        }
        let eta = self.eta(p.x, p.z);
        let h = d + eta;
        let z = p.y - self.bed;
        if z > h || h <= 0.0 {
            return None;
        }
        // Wheeler: map the actual column 0..d+η onto the linear-theory 0..d.
        let s = (z * d / h).clamp(0.0, d);
        let cur = match self.profile {
            CurrentProfile::Uniform => self.current,
            CurrentProfile::PowerLaw => self.current * (8.0 / 7.0 * (s / d).powf(1.0 / 7.0)),
        };
        let (u, a) = match self.wave {
            SeaWave::Calm => (Vec3::zeros(), Vec3::zeros()),
            SeaWave::Airy {
                amplitude,
                k,
                omega,
                direction,
                phase,
            } => {
                let th = k * p.dot(&direction) - phase - omega * self.t;
                let (c, sn) = (th.cos(), th.sin());
                let (ch, sh) = depth_decay(k, s, d);
                let aw = amplitude * omega;
                let aw2 = aw * omega;
                (
                    direction * (aw * ch * c) + Vec3::y() * (aw * sh * sn),
                    direction * (aw2 * ch * sn) - Vec3::y() * (aw2 * sh * c),
                )
            }
            SeaWave::Solitary {
                height,
                kappa,
                celerity: c,
                direction,
                start,
            } => {
                let th = kappa * (p.dot(&direction) - start - c * self.t);
                let s2 = sech2(th);
                let tn = th.tanh();
                let eta_x = -2.0 * kappa * eta * tn;
                let eta_xx = 2.0 * kappa * kappa * height * s2 * (2.0 * tn * tn - s2);
                let eta_t = -c * eta_x;
                let eta_xt = -c * eta_xx;
                // u = g(η) = c η / (d + η)
                let g1 = c * d / (h * h);
                let g2 = -2.0 * c * d / (h * h * h);
                let u = c * eta / h;
                let u_x = g1 * eta_x;
                let u_t = g1 * eta_t;
                let u_xt = g2 * eta_t * eta_x + g1 * eta_xt;
                (
                    direction * u - Vec3::y() * (z * u_x),
                    direction * u_t - Vec3::y() * (z * u_xt),
                )
            }
        };
        Some((u + cur, a))
    }

    /// Water velocity at `p` (zero where dry), m/s.
    pub fn velocity(&self, p: Vec3) -> Vec3 {
        self.kinematics(p).map_or(Vec3::zeros(), |k| k.0)
    }

    /// Local water acceleration `∂u/∂t` at `p` (zero where dry), m/s².
    pub fn acceleration(&self, p: Vec3) -> Vec3 {
        self.kinematics(p).map_or(Vec3::zeros(), |k| k.1)
    }
}

fn sech2(x: f64) -> f64 {
    let c = x.abs().min(350.0).cosh();
    1.0 / (c * c)
}

impl Water {
    /// A flood rising from the bed (y = 0) to `level` over `rise_time`.
    pub fn flood(level: f64, rise_time: f64) -> Self {
        Self {
            level1: level,
            rise_time,
            ..Default::default()
        }
    }

    /// Still water at `level` with a uniform `current`.
    pub fn current(level: f64, current: Vec3) -> Self {
        Self {
            level0: level,
            level1: level,
            current,
            ..Default::default()
        }
    }

    /// Regular Airy waves of height `h` and period `period` on still water
    /// at `level`, travelling along `direction`, crest at the origin at t = 0.
    pub fn airy(level: f64, h: f64, period: f64, direction: Vec3) -> Self {
        Self {
            level0: level,
            level1: level,
            waves: Some(Waves::Airy(Airy {
                height: h,
                period,
                direction: horizontal(direction),
                origin: Vec3::zeros(),
            })),
            ..Default::default()
        }
    }

    /// A solitary wave of height `h` on still water at `level`, crest at
    /// `start` (along `direction`) at t = 0. The paddle is placed `4/K`
    /// ahead of the crest so it makes the whole stroke as the wave passes it.
    pub fn solitary(level: f64, h: f64, start: f64, direction: Vec3) -> Self {
        let d = level.max(1e-6);
        let kappa = (3.0 * h.max(0.0) / (4.0 * d.powi(3))).sqrt();
        let lead = if kappa > 0.0 { 4.0 / kappa } else { 0.0 };
        Self {
            level0: level,
            level1: level,
            waves: Some(Waves::Solitary(Solitary {
                height: h,
                start,
                direction: horizontal(direction),
            })),
            paddle: start + lead,
            ..Default::default()
        }
    }

    /// Reset the runtime state (paddle).
    pub fn prepare(&mut self) {
        self.state = PaddleState::default();
    }

    /// Headline intensity: wave height for waves and surges, else the flood
    /// depth, in m.
    pub fn intensity(&self) -> (f64, &'static str) {
        match &self.waves {
            Some(Waves::Airy(a)) => (a.height, "m"),
            Some(Waves::Solitary(s)) => (s.height, "m"),
            None => ((self.level0.max(self.level1) - self.bed).max(0.0), "m"),
        }
    }

    /// Unramped still water depth at time `t` since case start.
    fn full_depth(&self, t: f64) -> f64 {
        let f = smooth_ramp(t, self.rise_time);
        (self.level0 + (self.level1 - self.level0) * f - self.bed).max(0.0)
    }

    /// Still water level (absolute y) at time `t`, at full ramp.
    pub fn still_level(&self, t: f64) -> f64 {
        self.bed + self.full_depth(t)
    }

    /// The water at time `t` since case start with window factor `ramp`:
    /// depth above the bed, wave height and current all scale with `ramp`.
    pub fn sea(&self, t: f64, ramp: f64) -> Sea {
        let ramp = ramp.clamp(0.0, 1.0);
        let depth = self.full_depth(t) * ramp;
        let current = Vec3::new(self.current.x, 0.0, self.current.z) * ramp;
        let wave = match &self.waves {
            _ if depth <= 0.0 => SeaWave::Calm,
            None => SeaWave::Calm,
            Some(Waves::Airy(a)) if a.period > 0.0 => {
                let omega = 2.0 * std::f64::consts::PI / a.period;
                let k = wave_number(omega, depth);
                let direction = horizontal(a.direction);
                SeaWave::Airy {
                    amplitude: 0.5 * a.height * ramp,
                    k,
                    omega,
                    direction,
                    phase: k * a.origin.dot(&direction),
                }
            }
            Some(Waves::Airy(_)) => SeaWave::Calm,
            Some(Waves::Solitary(s)) => {
                let h = s.height * ramp;
                SeaWave::Solitary {
                    height: h,
                    kappa: (3.0 * h.max(0.0) / (4.0 * depth.powi(3))).sqrt(),
                    celerity: (G * (depth + h)).sqrt(),
                    direction: horizontal(s.direction),
                    start: s.start,
                }
            }
        };
        Sea {
            bed: self.bed,
            depth,
            current,
            profile: self.profile,
            wave,
            t,
        }
    }

    /// Water surface (absolute y) at (x, z), time `t`, full ramp.
    pub fn surface_elevation(&self, x: f64, z: f64, t: f64) -> f64 {
        self.sea(t, 1.0).surface(x, z)
    }

    /// Water velocity at `p`, time `t`, full ramp (zero where dry).
    pub fn water_velocity(&self, p: Vec3, t: f64) -> Vec3 {
        self.sea(t, 1.0).velocity(p)
    }

    /// Local water acceleration at `p`, time `t`, full ramp (zero where dry).
    pub fn water_acceleration(&self, p: Vec3, t: f64) -> Vec3 {
        self.sea(t, 1.0).acceleration(p)
    }

    /// Integrate the solitary-wave paddle (Goring: the paddle moves with the
    /// depth-averaged water velocity at its face, `dX/dt = c η/(d+η)`) from
    /// the last call up to `t`.
    fn advance_paddle(&mut self, sea: &Sea) {
        let SeaWave::Solitary { direction, .. } = sea.wave else {
            return;
        };
        let t = sea.t;
        let st = &mut self.state;
        if !st.initialized || t < st.time - 1e-9 {
            *st = PaddleState {
                initialized: true,
                time: t,
                position: 0.0,
                velocity: 0.0,
            };
        }
        let rest = self.paddle;
        let vel = |x: f64, tau: f64| {
            let s = Sea { t: tau, ..*sea };
            let p = direction * (rest + x);
            let eta = s.eta(p.x, p.z);
            let h = s.depth + eta;
            match s.wave {
                SeaWave::Solitary { celerity, .. } if h > 0.0 => celerity * eta / h,
                _ => 0.0,
            }
        };
        let span = t - st.time;
        if span > 0.0 {
            let n = (span / (1.0 / 480.0)).ceil().max(1.0) as usize;
            let h = span / n as f64;
            for _ in 0..n {
                let t0 = st.time;
                let k1 = vel(st.position, t0);
                let k2 = vel(st.position + 0.5 * h * k1, t0 + 0.5 * h);
                st.position += h * k2;
                st.time = t0 + h;
            }
        }
        st.velocity = vel(st.position, t);
    }

    /// Add this frame's water forces to `out`.
    pub fn apply(&mut self, v: &View, t: f64, ramp: f64, scale: f64, _dt: f64, out: &mut [Vec3]) {
        let sea = self.sea(t, ramp);
        self.advance_paddle(&sea);
        // Simulated water is already in the flume: the ramp fades the
        // forces in instead of the depth.
        let field = self.field.clone();
        let (scale, top, bed) = match &field {
            Some(f) => (scale * ramp.clamp(0.0, 1.0), f.max_surface(), f.bed()),
            None => (scale, sea.max_surface(), sea.bed),
        };
        if (field.is_none() && sea.depth <= 0.0) || scale == 0.0 {
            return;
        }
        let kinematics = |p: Vec3| match &field {
            Some(f) => f.kinematics(p),
            None => sea.kinematics(p),
        };
        let rho = self.density;
        let co = &self.coefficients;
        for m in 0..v.structure.members.len() {
            let Some(mn) = v.member(m) else { continue };
            if mn.xa.y.min(mn.xb.y) > top || mn.xa.y.max(mn.xb.y) < bed {
                continue;
            }
            let g = &v.geometry.members[m];
            let (cd, ca) = match g.shape {
                Shape::Tube { .. } => (co.tube_cd, co.tube_ca),
                Shape::Rect { .. } => (co.rect_cd, co.rect_ca),
            };
            let inertia = rho * (1.0 + ca) * g.outer_area;
            let buoy = rho
                * G
                * if self.sealed {
                    g.outer_area
                } else {
                    g.solid_area
                };
            let shape = g.shape;
            let (dy, dz) = (g.depth_y, g.depth_z);
            View::distribute(&mn, self.samples, out, |_, p, vel| {
                let Some((u, a)) = kinematics(p) else {
                    return Vec3::zeros();
                };
                let ur = u - vel;
                let urn = ur - mn.x * mn.x.dot(&ur);
                let an = a - mn.x * mn.x.dot(&a);
                let q = 0.5 * rho * cd * urn.norm();
                let drag = match shape {
                    Shape::Tube { .. } => urn * (q * dy),
                    // Flow along local y meets the face of width dz, and vice versa.
                    Shape::Rect { .. } => {
                        mn.y * (q * dz * urn.dot(&mn.y)) + mn.z * (q * dy * urn.dot(&mn.z))
                    }
                };
                (an * inertia + drag + Vec3::y() * buoy) * scale
            });
        }
    }

    /// The flume rig at time `t` with window factor `ramp`. The solitary
    /// paddle comes from the state integrated by [`Self::apply`],
    /// extrapolated to `t`.
    pub fn rig(&self, t: f64, ramp: f64) -> FlumeRig {
        let sea = self.sea(t, ramp);
        let mut r = FlumeRig {
            bed: self.bed,
            still_level: sea.still_level(),
            depth: sea.depth,
            paddle_rest: self.paddle,
            direction: horizontal(self.current),
            current_speed: sea.current.norm(),
            ..Default::default()
        };
        match sea.wave {
            SeaWave::Calm => {}
            SeaWave::Airy {
                amplitude,
                k,
                omega,
                direction,
                phase,
            } => {
                let stroke = 2.0 * amplitude / piston_height_to_stroke(k * sea.depth);
                let th = k * self.paddle - phase - omega * t;
                r.paddle_position = -0.5 * stroke * th.sin();
                r.paddle_velocity = 0.5 * stroke * omega * th.cos();
                r.wave_height = 2.0 * amplitude;
                r.wave_period = 2.0 * std::f64::consts::PI / omega;
                r.direction = direction;
            }
            SeaWave::Solitary {
                height, direction, ..
            } => {
                let st = &self.state;
                if st.initialized {
                    r.paddle_position = st.position + st.velocity * (t - st.time).max(0.0);
                    r.paddle_velocity = st.velocity;
                }
                r.wave_height = height;
                r.direction = direction;
            }
        }
        r
    }
}
