//! Earthquakes on the shake table: uniform base excitation.
//!
//! The structure is simulated in the frame of the table. Its supports stay
//! fixed in the simulation and every free node feels the inertial force
//! `−m a_g(t)` of the ground acceleration `a_g`. For a rigid table this is
//! exact: the equations of motion of the displacements relative to the
//! ground are those of the fixed-base structure with that effective load.
//! The renderer adds the table displacement `u_g(t)`
//! ([`super::LoadSet::table_offset`]) to the table and to every simulated
//! position. Solver damping acts on the relative velocity, which is what
//! structural damping does.
//!
//! Ground motion comes from
//!
//! * a recorded accelerogram (raw samples, or a PEER NGA `.AT2` file),
//! * a synthetic earthquake: Gaussian white noise through a Kanai-Tajimi
//!   soil filter (dominant ground frequency and damping) and a
//!   Clough-Penzien high-pass, shaped by a Jennings envelope (build-up,
//!   strong motion, exponential decay) and scaled to a target peak ground
//!   acceleration,
//! * a shake-table test signal: a sine or a sine sweep.
//!
//! Every motion is sampled once in [`Quake::prepare`], high-pass filtered at
//! 0.1 Hz with a zero-phase Butterworth filter and integrated to velocity and
//! displacement, with the linear drift removed from the displacement (a
//! constant table velocity does nothing in the table frame), so the drawn
//! table motion stays bounded and matches the applied acceleration.
//!
//! Point masses added to nodes (storey mass blocks) take part through
//! [`crate::DynamicSim::node_mass`].

use super::rng::Rng;
use super::{View, G};
use crate::Vec3;

/// Where the ground acceleration comes from.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum GroundMotion {
    /// Recorded acceleration samples (m/s², world axes) at a fixed step.
    Record {
        dt: f64,
        accel: Vec<Vec3>,
    },
    Synthetic(Synthetic),
    /// Sine test: `a = A sin(2π f t)` along `direction`, with two-cycle
    /// tapers at both ends.
    Harmonic {
        amplitude: f64,
        frequency: f64,
        duration: f64,
        direction: Vec3,
    },
    /// Linear sine sweep from `f0` to `f1` Hz over `duration`, used on real
    /// shake tables to find a structure's natural frequencies.
    Sweep {
        amplitude: f64,
        f0: f64,
        f1: f64,
        duration: f64,
        direction: Vec3,
    },
}

/// Parameters of a synthetic earthquake.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Synthetic {
    /// Peak ground acceleration of the main component, m/s².
    pub pga: f64,
    /// Strong-motion duration, s.
    pub duration: f64,
    /// Dominant frequency of the soil filter, Hz: about 2.5 on firm ground,
    /// 1 on soft soil, 5 on rock.
    pub ground_frequency: f64,
    /// Soil filter damping ratio (0.6 is the classic Kanai-Tajimi value).
    pub ground_damping: f64,
    /// Main horizontal direction.
    pub direction: Vec3,
    /// Peak of the second horizontal component (perpendicular) over the main
    /// one; 0 for a one-axis table.
    pub cross_ratio: f64,
    /// Peak of the vertical component over the main one (about 2/3 in real
    /// records); 0 for none.
    pub vertical_ratio: f64,
    pub seed: u64,
}

impl Default for Synthetic {
    fn default() -> Self {
        Self {
            pga: 0.3 * G,
            duration: 10.0,
            ground_frequency: 2.5,
            ground_damping: 0.6,
            direction: Vec3::x(),
            cross_ratio: 0.0,
            vertical_ratio: 0.0,
            seed: 1,
        }
    }
}

#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Quake {
    pub motion: GroundMotion,
    #[cfg_attr(feature = "serde", serde(skip))]
    state: QuakeState,
}

#[derive(Clone, Debug, Default)]
struct QuakeState {
    dt: f64,
    acc: Vec<Vec3>,
    vel: Vec<Vec3>,
    disp: Vec<Vec3>,
    pga: f64,
}

/// Shake table state for the renderer, the sound and the seismograph.
#[derive(Clone, Copy, Debug, Default)]
pub struct TableRig {
    pub displacement: Vec3,
    pub velocity: Vec3,
    pub acceleration: Vec3,
    /// Actuator demand 0..1: |acceleration| over the peak of the motion.
    pub drive: f64,
}

impl TableRig {
    /// Two motions on one table add up.
    pub fn combine(&self, o: &TableRig) -> TableRig {
        TableRig {
            displacement: self.displacement + o.displacement,
            velocity: self.velocity + o.velocity,
            acceleration: self.acceleration + o.acceleration,
            drive: (self.drive + o.drive).min(1.0),
        }
    }
}

/// Sample step of generated motions, s.
const SAMPLE_DT: f64 = 0.005;
/// High-pass corner applied to every motion, Hz.
const HIGH_PASS: f64 = 0.1;

impl Quake {
    pub fn new(motion: GroundMotion) -> Self {
        Self {
            motion,
            state: QuakeState::default(),
        }
    }

    /// Synthetic earthquake with a peak ground acceleration in g along `direction`.
    pub fn synthetic(pga_g: f64, duration: f64, direction: Vec3, seed: u64) -> Self {
        Self::new(GroundMotion::Synthetic(Synthetic {
            pga: pga_g * G,
            duration,
            direction,
            seed,
            ..Default::default()
        }))
    }

    /// Sample, filter and integrate the motion. Called by
    /// [`super::LoadSet::prepare`]; cheap enough to call again after editing.
    pub fn prepare(&mut self) {
        let (dt, acc) = sample(&self.motion);
        let (mut acc, mut vel, mut disp) =
            process(dt, acc, !matches!(self.motion, GroundMotion::Record { .. }));
        if let GroundMotion::Synthetic(s) = &self.motion {
            // Filtering moved the peak a little: scale back to the target.
            let d = synthetic_direction(s);
            let peak = acc.iter().map(|a| a.dot(&d).abs()).fold(0.0, f64::max);
            if peak > 0.0 {
                let k = s.pga / peak;
                for v in acc.iter_mut().chain(vel.iter_mut()).chain(disp.iter_mut()) {
                    *v *= k;
                }
            }
        }
        let pga = acc.iter().map(|a| a.norm()).fold(0.0, f64::max);
        self.state = QuakeState {
            dt,
            acc,
            vel,
            disp,
            pga,
        };
    }

    fn ensure(&self) -> bool {
        !self.state.acc.is_empty()
    }

    /// Processed acceleration samples (m/s², world axes) and their step,
    /// for the seismograph strip.
    pub fn samples(&self) -> (f64, &[Vec3]) {
        (self.state.dt, &self.state.acc)
    }

    /// Length of the motion, s.
    pub fn duration(&self) -> f64 {
        self.state.dt * self.state.acc.len().saturating_sub(1) as f64
    }

    /// Peak ground acceleration (vector magnitude), m/s². Before
    /// [`Self::prepare`], the target of a generated motion.
    pub fn peak_acceleration(&self) -> f64 {
        if self.ensure() {
            return self.state.pga;
        }
        match &self.motion {
            GroundMotion::Record { accel, .. } => {
                accel.iter().map(|a| a.norm()).fold(0.0, f64::max)
            }
            GroundMotion::Synthetic(s) => s.pga,
            GroundMotion::Harmonic { amplitude, .. } | GroundMotion::Sweep { amplitude, .. } => {
                *amplitude
            }
        }
    }

    fn lerp(&self, v: &[Vec3], t: f64) -> Vec3 {
        if v.is_empty() || t < 0.0 {
            return Vec3::zeros();
        }
        let x = t / self.state.dt;
        let i = x.floor() as usize;
        if i + 1 >= v.len() {
            return *v.last().unwrap();
        }
        let w = x - i as f64;
        v[i] * (1.0 - w) + v[i + 1] * w
    }

    /// Ground acceleration at case time `t`, m/s² (zero after the record).
    pub fn acceleration(&self, t: f64) -> Vec3 {
        if t > self.duration() {
            return Vec3::zeros();
        }
        self.lerp(&self.state.acc, t)
    }

    /// Add `−m a_g` on every free node. `scale` is ramp × load factor.
    pub fn apply(&mut self, v: &View, t: f64, scale: f64, out: &mut [Vec3]) {
        if !self.ensure() {
            self.prepare();
        }
        let a = self.acceleration(t) * scale;
        if a == Vec3::zeros() {
            return;
        }
        for (n, f) in out.iter_mut().enumerate() {
            let m = v.sim.node_mass(n);
            if m > 0.0 {
                *f -= a * m;
            }
        }
    }

    /// Table motion at case time `t`, scaled by the load factor.
    pub fn table(&self, t: f64, scale: f64) -> TableRig {
        if !self.ensure() || t < 0.0 {
            return TableRig::default();
        }
        let a = self.acceleration(t) * scale;
        TableRig {
            displacement: self.lerp(&self.state.disp, t) * scale,
            velocity: self.lerp(&self.state.vel, t) * scale,
            acceleration: a,
            drive: if self.state.pga > 0.0 {
                (a.norm() / (self.state.pga * scale.max(1e-9))).min(1.0)
            } else {
                0.0
            },
        }
    }

    /// Pseudo-spectral acceleration (m/s²) of the motion for each natural
    /// period (s) at damping ratio `zeta`: the peak acceleration a linear
    /// single-storey structure of that period feels. For the scenario brief.
    pub fn response_spectrum(&mut self, periods: &[f64], zeta: f64) -> Vec<f64> {
        if !self.ensure() {
            self.prepare();
        }
        let dt = self.state.dt;
        periods
            .iter()
            .map(|&tn| {
                let w = std::f64::consts::TAU / tn;
                // Newmark average acceleration on each axis, sub-stepped.
                let sub = ((dt / (tn / 40.0)).ceil() as usize).max(1);
                let h = dt / sub as f64;
                let (mut u, mut ud) = (Vec3::zeros(), Vec3::zeros());
                let mut udd = -self.state.acc[0];
                let mut peak: f64 = 0.0;
                let keff = w * w + 2.0 * zeta * w * 2.0 / h + 4.0 / (h * h);
                for k in 0..self.state.acc.len() - 1 {
                    for s in 0..sub {
                        let x = (s + 1) as f64 / sub as f64;
                        let ag = self.state.acc[k] * (1.0 - x) + self.state.acc[k + 1] * x;
                        let p = -ag
                            + (u * (4.0 / (h * h)) + ud * (4.0 / h) + udd)
                            + (u * (2.0 / h) + ud) * (2.0 * zeta * w);
                        let un = p / keff;
                        let udn = (un - u) * (2.0 / h) - ud;
                        udd = (udn - ud) * (2.0 / h) - udd;
                        u = un;
                        ud = udn;
                        peak = peak.max(u.norm());
                    }
                }
                w * w * peak
            })
            .collect()
    }
}

impl GroundMotion {
    /// Parse a PEER NGA strong-motion file (`.AT2`: four header lines, the
    /// fourth with `NPTS=` and `DT=`, then accelerations in g) as a record
    /// along `direction`.
    pub fn from_peer_at2(text: &str, direction: Vec3) -> Result<Self, String> {
        let mut lines = text.lines();
        let header: Vec<&str> = lines.by_ref().take(4).collect();
        let h = header
            .get(3)
            .ok_or("AT2: missing header")?
            .to_ascii_uppercase();
        let field = |key: &str| -> Option<f64> {
            let i = h.find(key)? + key.len();
            let rest = h[i..].trim_start_matches([' ', '=']);
            let end = rest
                .find(|c: char| !(c.is_ascii_digit() || matches!(c, '.' | '-' | '+' | 'E')))
                .unwrap_or(rest.len());
            rest[..end].parse().ok()
        };
        let dt = field("DT").ok_or("AT2: no DT in header")?;
        let npts = field("NPTS").map(|n| n as usize);
        let dir = direction.normalize();
        let mut accel = Vec::new();
        for l in lines {
            for tok in l.split_whitespace() {
                let a: f64 = tok.parse().map_err(|_| format!("AT2: bad value {tok}"))?;
                accel.push(dir * (a * G));
            }
        }
        if let Some(n) = npts {
            accel.truncate(n);
        }
        if accel.is_empty() || dt <= 0.0 {
            return Err("AT2: no samples".into());
        }
        Ok(GroundMotion::Record { dt, accel })
    }
}

fn smoothstep(x: f64) -> f64 {
    let x = x.clamp(0.0, 1.0);
    x * x * (3.0 - 2.0 * x)
}

fn sample(m: &GroundMotion) -> (f64, Vec<Vec3>) {
    match m {
        GroundMotion::Record { dt, accel } => (*dt, accel.clone()),
        GroundMotion::Harmonic {
            amplitude,
            frequency,
            duration,
            direction,
        } => {
            let d = direction.normalize();
            let taper = 2.0 / frequency;
            let n = (duration / SAMPLE_DT).ceil() as usize + 1;
            let acc = (0..n)
                .map(|i| {
                    let t = i as f64 * SAMPLE_DT;
                    let env = smoothstep(t / taper) * smoothstep((duration - t) / taper);
                    d * (amplitude * env * (std::f64::consts::TAU * frequency * t).sin())
                })
                .collect();
            (SAMPLE_DT, acc)
        }
        GroundMotion::Sweep {
            amplitude,
            f0,
            f1,
            duration,
            direction,
        } => {
            let d = direction.normalize();
            let taper = 2.0 / f0;
            let n = (duration / SAMPLE_DT).ceil() as usize + 1;
            let acc = (0..n)
                .map(|i| {
                    let t = i as f64 * SAMPLE_DT;
                    let env = smoothstep(t / taper) * smoothstep((duration - t) / taper);
                    let ph = std::f64::consts::TAU * (f0 * t + 0.5 * (f1 - f0) * t * t / duration);
                    d * (amplitude * env * ph.sin())
                })
                .collect();
            (SAMPLE_DT, acc)
        }
        GroundMotion::Synthetic(s) => (SAMPLE_DT, synthetic(s)),
    }
}

/// One component of filtered, enveloped white noise with unit peak.
fn synthetic_component(s: &Synthetic, rng: &mut Rng) -> Vec<f64> {
    let t1 = (0.2 * s.duration).min(2.0);
    let t2 = t1 + s.duration;
    let decay = 20f64.ln() / (0.5 * s.duration);
    let total = t2 + 0.6 * s.duration;
    let n = (total / SAMPLE_DT).ceil() as usize + 1;
    let wg = std::f64::consts::TAU * s.ground_frequency;
    let zg = s.ground_damping;
    let (wf, zf) = (0.1 * wg, zg);
    // Filters integrated at a finer step with semi-implicit Euler.
    let sub = 4;
    let h = SAMPLE_DT / sub as f64;
    let (mut x, mut xd, mut y, mut yd) = (0.0, 0.0, 0.0, 0.0);
    let mut out = Vec::with_capacity(n);
    for i in 0..n {
        let t = i as f64 * SAMPLE_DT;
        let env = if t < t1 {
            (t / t1).powi(2)
        } else if t < t2 {
            1.0
        } else {
            (-decay * (t - t2)).exp()
        };
        let mut a = 0.0;
        for _ in 0..sub {
            let w = rng.normal() / h.sqrt() * env;
            // Kanai-Tajimi: soil column on bedrock white noise.
            let xdd = -w - 2.0 * zg * wg * xd - wg * wg * x;
            xd += h * xdd;
            x += h * xd;
            let a_kt = -(2.0 * zg * wg * xd + wg * wg * x);
            // Clough-Penzien high-pass.
            let ydd = a_kt - 2.0 * zf * wf * yd - wf * wf * y;
            yd += h * ydd;
            y += h * yd;
            a = ydd;
        }
        out.push(a);
    }
    // Taper the tail end to exactly zero.
    let nt = (0.5 / SAMPLE_DT) as usize;
    for k in 0..nt.min(n) {
        out[n - 1 - k] *= smoothstep(k as f64 / nt as f64);
    }
    let peak = out.iter().fold(0.0f64, |m, a| m.max(a.abs()));
    if peak > 0.0 {
        out.iter_mut().for_each(|a| *a /= peak);
    }
    out
}

fn synthetic_direction(s: &Synthetic) -> Vec3 {
    let d = Vec3::new(s.direction.x, 0.0, s.direction.z);
    if d.norm() > 1e-12 {
        d.normalize()
    } else {
        Vec3::x()
    }
}

fn synthetic(s: &Synthetic) -> Vec<Vec3> {
    let mut rng = Rng::new(s.seed);
    let d = synthetic_direction(s);
    let c = Vec3::y().cross(&d);
    let main = synthetic_component(s, &mut rng);
    let mut acc: Vec<Vec3> = main.iter().map(|a| d * (a * s.pga)).collect();
    if s.cross_ratio > 0.0 {
        let cross = synthetic_component(s, &mut rng);
        for (a, x) in acc.iter_mut().zip(cross) {
            *a += c * (x * s.pga * s.cross_ratio);
        }
    }
    if s.vertical_ratio > 0.0 {
        // Vertical motion is richer in high frequencies.
        let sv = Synthetic {
            ground_frequency: s.ground_frequency * 2.0,
            ..s.clone()
        };
        let vert = synthetic_component(&sv, &mut rng);
        for (a, x) in acc.iter_mut().zip(vert) {
            a.y += x * s.pga * s.vertical_ratio;
        }
    }
    acc
}

/// Second-order Butterworth high-pass, run forward and backward (zero phase).
fn high_pass(x: &mut [f64], dt: f64, fc: f64) {
    if x.len() < 3 {
        return;
    }
    let k = (std::f64::consts::PI * fc * dt).tan();
    let q = std::f64::consts::FRAC_1_SQRT_2;
    let norm = 1.0 / (1.0 + k / q + k * k);
    let (b0, b1, b2) = (norm, -2.0 * norm, norm);
    let (a1, a2) = (2.0 * (k * k - 1.0) * norm, (1.0 - k / q + k * k) * norm);
    let pass = |x: &mut [f64]| {
        let (mut x1, mut x2, mut y1, mut y2) = (0.0, 0.0, 0.0, 0.0);
        for v in x.iter_mut() {
            let y = b0 * *v + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
            (x2, x1, y2, y1) = (x1, *v, y1, y);
            *v = y;
        }
    };
    pass(x);
    x.reverse();
    pass(x);
    x.reverse();
}

#[allow(clippy::type_complexity)]
fn process(dt: f64, acc: Vec<Vec3>, generated: bool) -> (Vec<Vec3>, Vec<Vec3>, Vec<Vec3>) {
    let n = acc.len();
    let mut axes: [Vec<f64>; 3] = [0, 1, 2].map(|k| acc.iter().map(|a| a[k]).collect());
    for ax in axes.iter_mut() {
        if ax.iter().all(|v| *v == 0.0) {
            continue;
        }
        if !generated {
            // Recorded: remove the pre-event mean and taper the ends.
            let mean = ax.iter().take(n.min(100)).sum::<f64>() / n.min(100) as f64;
            ax.iter_mut().for_each(|v| *v -= mean);
            let nt = ((0.5 / dt) as usize).min(n / 4);
            for k in 0..nt {
                let w = smoothstep(k as f64 / nt as f64);
                ax[k] *= w;
                ax[n - 1 - k] *= w;
            }
        }
        high_pass(ax, dt, HIGH_PASS);
    }
    let mut out_a = vec![Vec3::zeros(); n];
    let mut out_v = vec![Vec3::zeros(); n];
    let mut out_d = vec![Vec3::zeros(); n];
    for k in 0..3 {
        let a = &axes[k];
        let mut v = vec![0.0; n];
        let mut d = vec![0.0; n];
        for i in 1..n {
            v[i] = v[i - 1] + 0.5 * dt * (a[i - 1] + a[i]);
            d[i] = d[i - 1] + dt * v[i - 1] + dt * dt * (a[i - 1] / 3.0 + a[i] / 6.0);
        }
        // Remove the least-squares linear drift of d (a constant velocity).
        let tm = 0.5 * dt * (n - 1) as f64;
        let (mut sxy, mut sxx, mut sy) = (0.0, 0.0, 0.0);
        for (i, di) in d.iter().enumerate() {
            let x = i as f64 * dt - tm;
            sxy += x * di;
            sxx += x * x;
            sy += di;
        }
        let slope = if sxx > 0.0 { sxy / sxx } else { 0.0 };
        let mean = sy / n.max(1) as f64;
        for i in 0..n {
            let x = i as f64 * dt - tm;
            out_a[i][k] = a[i];
            out_v[i][k] = v[i] - slope;
            out_d[i][k] = d[i] - mean - slope * x;
        }
    }
    // Start the table at rest at zero offset: shift so d(0) = 0.
    let d0 = out_d.first().copied().unwrap_or_default();
    out_d.iter_mut().for_each(|d| *d -= d0);
    (out_a, out_v, out_d)
}
