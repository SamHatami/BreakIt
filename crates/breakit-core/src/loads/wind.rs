//! Wind: quasi-steady drag on members, gusts, shielding and vortex shedding.
//!
//! Each member carries a line load from the wind velocity relative to the
//! member, resolved normal to its axis (the cross-flow principle):
//!
//! `f = ½ ρ (C_y d_z v_y ŷ + C_z d_y v_z ẑ) |v_n| ψ`
//!
//! where `v_n` is the relative velocity normal to the member, `v_y`, `v_z`
//! its components on the member's local axes, `d_y`, `d_z` the section's
//! outer extents and `ψ` the shielding factor. A tube has one drag
//! coefficient for all directions, so this is the familiar
//! `½ ρ C_d D |v_n| v_n`. Using the relative velocity gives aerodynamic
//! damping for free.
//!
//! * Drag coefficients: circular sections follow the drag crisis with
//!   Reynolds number (1.2 subcritical, about 0.5 just past 4·10⁵, back up to
//!   0.7 supercritical); rectangles use the force coefficient of
//!   EN 1991-1-4 Fig. 7.23 against the depth-to-width ratio in the flow
//!   direction. No end-effect reduction: frame members are joined at both
//!   ends.
//! * Profile: uniform, power law, log law above the table top, or the
//!   top-hat jet of the fan wall rig.
//! * Gusts: along-wind turbulence with the Kaimal spectrum, made of sines
//!   with random phases from a seed and convected downwind as frozen
//!   turbulence (Taylor), so a gust sweeps through the structure. Fully
//!   coherent across the flow (conservative for small models).
//! * Shielding: members are projected on the plane normal to the wind on a
//!   grid of cells about one member depth across. Going downwind, each member
//!   sees the velocity deficit the members upstream of it left in its cells,
//!   and adds its own (its solidity in the cell times an efficiency); the
//!   deficit recovers with distance downstream. This reproduces the drop in
//!   load on the leeward truss of a pair; it is an approximation, not a
//!   wake model.
//! * Vortex shedding: a cross-flow force `½ ρ C_L D v_n² sin φ` at the
//!   Strouhal frequency `St v_n / D` per member, with random phases between
//!   members (the spanwise correlation is short away from lock-in). Faded
//!   out above a few Hz, where frame sampling would alias it.

use std::collections::HashMap;

use super::rng::Rng;
use super::View;
use crate::section::Shape;
use crate::{Structure, Vec3};

/// Kinematic viscosity of air at 15 °C, m²/s.
pub const AIR_VISCOSITY: f64 = 1.5e-5;
/// Density of air at 15 °C, sea level, kg/m³.
pub const AIR_DENSITY: f64 = 1.225;

/// How the mean wind speed varies over the table.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Profile {
    /// Same speed everywhere above the ground.
    Uniform,
    /// `U(h) = U_ref (h / h_ref)^α`, h above the ground. α ≈ 0.12 over open
    /// water, 0.16 open country, 0.3 city centre.
    PowerLaw { ref_height: f64, alpha: f64 },
    /// `U(h) = U_ref ln(h/z0) / ln(h_ref/z0)`, roughness length z0.
    LogLaw { ref_height: f64, roughness: f64 },
    /// The fan wall: a top-hat jet centred on `center` (only its position
    /// across the flow matters), `half_width` across and `half_height` up
    /// from the centre, with a soft edge of width `edge`.
    Jet {
        center: Vec3,
        half_width: f64,
        half_height: f64,
        edge: f64,
    },
}

/// Along-wind turbulence.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Gusts {
    /// Turbulence intensity σ_u / U (0.1 to 0.25 is typical).
    pub intensity: f64,
    /// Integral length scale of the Kaimal spectrum, m.
    pub length_scale: f64,
    pub seed: u64,
}

impl Default for Gusts {
    fn default() -> Self {
        Self {
            intensity: 0.15,
            length_scale: 50.0,
            seed: 1,
        }
    }
}

#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Wind {
    /// Mean speed at the reference height (or in the jet core), m/s.
    pub speed: f64,
    /// Direction the wind blows towards (horizontal, normalised on use).
    pub direction: Vec3,
    pub profile: Profile,
    pub gusts: Option<Gusts>,
    /// Height of the ground the profile starts from (table top), m.
    pub ground: f64,
    pub air_density: f64,
    pub shielding: bool,
    /// Velocity deficit a fully solid cell leaves behind (0..1).
    pub shield_efficiency: f64,
    /// Wake recovery length in member depths.
    pub shield_recovery: f64,
    pub vortex_shedding: bool,
    #[cfg_attr(feature = "serde", serde(skip))]
    state: WindState,
}

#[derive(Clone, Debug, Default)]
struct WindState {
    /// Gust components: frequency (Hz), amplitude (m/s per m/s of mean
    /// speed), phase.
    components: Vec<(f64, f64, f64)>,
    /// Along-wind coordinate range of the structure (with margin).
    xi_range: (f64, f64),
    /// Shielding factor per member (on dynamic pressure).
    shield: Vec<f64>,
    shield_time: f64,
    shield_broken: usize,
    shed_phase: Vec<f64>,
    last_t: Option<f64>,
    /// u'(τ) sampled for this frame over the structure, and the τ range.
    gust_table: Vec<f64>,
    tau_range: (f64, f64),
}

/// State of the fan wall for the renderer and sound.
#[derive(Clone, Debug, Default)]
pub struct WindRig {
    /// Fan power 0..1 (mean plus gusts over the design speed).
    pub fan_drive: f64,
    /// Speed at the reference point including the gust, m/s.
    pub speed: f64,
    pub direction: Vec3,
    /// Current gust at the upstream edge of the structure, m/s.
    pub gust: f64,
}

impl Wind {
    pub fn new(speed: f64, direction: Vec3) -> Self {
        Self {
            speed,
            direction,
            profile: Profile::Uniform,
            gusts: None,
            ground: 0.0,
            air_density: AIR_DENSITY,
            shielding: true,
            shield_efficiency: 0.75,
            shield_recovery: 12.0,
            vortex_shedding: true,
            state: WindState::default(),
        }
    }

    fn dir(&self) -> Vec3 {
        let d = Vec3::new(self.direction.x, 0.0, self.direction.z);
        if d.norm() > 1e-12 {
            d.normalize()
        } else {
            Vec3::x()
        }
    }

    /// Basis of the plane normal to the wind: across (horizontal) and up.
    fn cross_basis(&self) -> (Vec3, Vec3) {
        let d = self.dir();
        (Vec3::y().cross(&d).normalize(), Vec3::y())
    }

    pub fn prepare(&mut self, s: &Structure) {
        let d = self.dir();
        let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
        for p in &s.nodes {
            let xi = p.dot(&d);
            lo = lo.min(xi);
            hi = hi.max(xi);
        }
        if !lo.is_finite() {
            (lo, hi) = (0.0, 0.0);
        }
        let margin = 0.25 * (hi - lo) + 1.0;
        self.state = WindState {
            xi_range: (lo - margin, hi + margin),
            components: self.gust_components(),
            ..Default::default()
        };
        let mut rng = Rng::new(self.gusts.as_ref().map_or(7, |g| g.seed ^ 0x5eed));
        self.state.shed_phase = (0..s.members.len())
            .map(|_| rng.uniform() * std::f64::consts::TAU)
            .collect();
    }

    /// Kaimal spectrum discretised into sines, scaled so their variance is
    /// exactly (intensity)². Amplitudes are per unit mean speed.
    fn gust_components(&self) -> Vec<(f64, f64, f64)> {
        let Some(g) = &self.gusts else {
            return Vec::new();
        };
        let u = self.speed.max(0.1);
        let n = 64;
        // Band: from well below the spectral peak (f ≈ U / (10 L)) to 5 Hz,
        // which a 60 Hz frame still samples well.
        let f_lo = (0.01 * u / g.length_scale).max(1e-3);
        let f_hi = 5.0f64.max(2.0 * f_lo);
        let mut rng = Rng::new(g.seed);
        let r = (f_hi / f_lo).powf(1.0 / n as f64);
        let mut comps = Vec::with_capacity(n);
        let mut var = 0.0;
        for i in 0..n {
            let (fa, fb) = (f_lo * r.powi(i as i32), f_lo * r.powi(i as i32 + 1));
            let f = (fa * fb).sqrt();
            let nn = f * g.length_scale / u;
            let s = 6.8 * nn / (1.0 + 10.2 * nn).powf(5.0 / 3.0) / f; // S(f)/σ²
            let a2 = 2.0 * s * (fb - fa);
            var += 0.5 * a2;
            comps.push((f, a2.sqrt(), rng.uniform() * std::f64::consts::TAU));
        }
        let k = g.intensity / var.sqrt();
        comps.iter_mut().for_each(|c| c.1 *= k);
        comps
    }

    /// Along-wind gust u'(τ) per unit mean speed, τ = t − ξ/U.
    fn gust_raw(&self, tau: f64) -> f64 {
        self.state
            .components
            .iter()
            .map(|&(f, a, ph)| a * (std::f64::consts::TAU * f * tau + ph).cos())
            .sum()
    }

    fn gust_at(&self, tau: f64) -> f64 {
        let tab = &self.state.gust_table;
        let (t0, t1) = self.state.tau_range;
        if tab.len() < 2 || t1 <= t0 || tau < t0 || tau > t1 {
            return self.gust_raw(tau);
        }
        let x = ((tau - t0) / (t1 - t0)).clamp(0.0, 1.0) * (tab.len() - 1) as f64;
        let i = (x as usize).min(tab.len() - 2);
        let w = x - i as f64;
        tab[i] * (1.0 - w) + tab[i + 1] * w
    }

    /// Mean-speed factor of the profile at point `p` (0 below the ground).
    pub fn profile_factor(&self, p: Vec3) -> f64 {
        let h = p.y - self.ground;
        if h <= 0.0 {
            return 0.0;
        }
        match &self.profile {
            Profile::Uniform => 1.0,
            Profile::PowerLaw { ref_height, alpha } => (h / ref_height).powf(*alpha),
            Profile::LogLaw {
                ref_height,
                roughness,
            } => {
                if h <= *roughness {
                    0.0
                } else {
                    (h / roughness).ln() / (ref_height / roughness).ln()
                }
            }
            Profile::Jet {
                center,
                half_width,
                half_height,
                edge,
            } => {
                let (across, up) = self.cross_basis();
                let r = p - center;
                let top_hat = |x: f64, half: f64| {
                    let e = edge.max(1e-9);
                    let u = (x.abs() - (half - 0.5 * e)) / e;
                    if u <= 0.0 {
                        1.0
                    } else if u >= 1.0 {
                        0.0
                    } else {
                        0.5 * (1.0 + (std::f64::consts::PI * u).cos())
                    }
                };
                top_hat(r.dot(&across), *half_width) * top_hat(r.dot(&up), *half_height)
            }
        }
    }

    /// Wind velocity at `p`, `t` seconds after the case started, with the
    /// window ramp `ramp` (0..1). For smoke streamers and ribbons too.
    pub fn velocity(&self, p: Vec3, t: f64, ramp: f64) -> Vec3 {
        let u = self.speed * ramp;
        let pf = self.profile_factor(p);
        if pf == 0.0 || u == 0.0 {
            return Vec3::zeros();
        }
        let d = self.dir();
        let gust = if self.state.components.is_empty() {
            0.0
        } else {
            let tau = t - (p.dot(&d) - self.state.xi_range.0) / u.max(0.5);
            self.gust_at(tau)
        };
        d * (u * (pf + gust * pf.min(1.0))).max(0.0)
    }

    fn update_shielding(&mut self, v: &View) {
        let nm = v.structure.members.len();
        let broken = (0..nm).filter(|&m| v.sim.is_broken(m)).count();
        let fresh = self.state.shield.len() == nm
            && broken == self.state.shield_broken
            && (v.time - self.state.shield_time).abs() < 0.25;
        if fresh {
            return;
        }
        self.state.shield_time = v.time;
        self.state.shield_broken = broken;
        self.state.shield = vec![1.0; nm];
        if !self.shielding {
            return;
        }
        let d = self.dir();
        let (across, up) = self.cross_basis();
        // Member state and its silhouette width facing the wind.
        let mut items: Vec<(usize, super::MemberNow, f64)> = (0..nm)
            .filter_map(|m| {
                let mn = v.member(m)?;
                let g = &v.geometry.members[m];
                let w = g.depth_z * mn.y.dot(&d).abs() + g.depth_y * mn.z.dot(&d).abs();
                Some((m, mn, w))
            })
            .collect();
        if items.is_empty() {
            return;
        }
        let mut widths: Vec<f64> = items.iter().map(|i| i.2).filter(|w| *w > 0.0).collect();
        widths.sort_by(f64::total_cmp);
        let cell = widths
            .get(widths.len() / 2)
            .copied()
            .unwrap_or(0.1)
            .max(1e-3);
        let area = cell * cell;
        items.sort_by(|a, b| {
            let xa = (a.1.xa + a.1.xb).dot(&d);
            let xb = (b.1.xa + b.1.xb).dot(&d);
            xa.total_cmp(&xb)
        });
        // Per cell: velocity deficit, where it was left, its recovery length.
        let mut cells: HashMap<(i64, i64), (f64, f64, f64)> = HashMap::new();
        let mut marks: Vec<((i64, i64), f64, f64)> = Vec::new();
        for (m, mn, w) in &items {
            let along = (mn.xb - mn.xa).dot(&across).hypot((mn.xb - mn.xa).dot(&up));
            let n = ((along / (0.5 * cell)).ceil() as usize).clamp(1, 64);
            let ds = along / n as f64;
            let mut psi = 0.0;
            marks.clear();
            for i in 0..n {
                let (p, _) = mn.at((i as f64 + 0.5) / n as f64);
                let key = (
                    (p.dot(&across) / cell).floor() as i64,
                    (p.dot(&up) / cell).floor() as i64,
                );
                let xi = p.dot(&d);
                // Only members clearly upstream shield; members side by side
                // in the same plane (a mast's own segments, a truss face) do
                // not.
                let deficit = cells
                    .get(&key)
                    .filter(|&&(_, x0, l)| xi - x0 > 0.5 * l / self.shield_recovery)
                    .map(|&(def, x0, l)| def * (-(xi - x0) / l).exp())
                    .unwrap_or(0.0);
                psi += (1.0 - deficit) * (1.0 - deficit);
                let block = (self.shield_efficiency * w * ds / area).min(1.0);
                marks.push((key, xi, block));
            }
            self.state.shield[*m] = (psi / n as f64).max(0.1);
            for &(key, xi, block) in &marks {
                let l = self.shield_recovery * w.max(1e-3);
                let e = cells.entry(key).or_insert((0.0, xi, l));
                let def = e.0 * (-(xi - e.1).max(0.0) / e.2).exp();
                *e = (1.0 - (1.0 - def) * (1.0 - block), xi, l);
            }
        }
    }

    /// Add the wind forces of the frame at case time `t` to `out`. `ramp`
    /// scales the wind speed, `scale` (load factor) the forces.
    pub fn apply(&mut self, v: &View, t: f64, ramp: f64, scale: f64, out: &mut [Vec3]) {
        let dt = self.state.last_t.map_or(0.0, |l| (t - l).max(0.0));
        self.state.last_t = Some(t);
        if self.state.shed_phase.len() != v.structure.members.len() {
            self.prepare(v.structure);
        }
        if ramp <= 0.0 || self.speed <= 0.0 {
            return;
        }
        // Gust table over the τ range the structure spans this frame.
        if !self.state.components.is_empty() {
            let u = (self.speed * ramp).max(0.5);
            let (x0, x1) = self.state.xi_range;
            let range = (t - (x1 - x0) / u, t);
            let n = 65;
            self.state.tau_range = (range.0, range.1);
            self.state.gust_table = (0..n)
                .map(|i| self.gust_raw(range.0 + (range.1 - range.0) * i as f64 / (n - 1) as f64))
                .collect();
        }
        self.update_shielding(v);
        let rho = self.air_density;
        for m in 0..v.structure.members.len() {
            let Some(mn) = v.member(m) else { continue };
            let g = &v.geometry.members[m];
            let psi = self.state.shield[m];
            let mut shed_speed = 0.0;
            let mut samples = 0.0;
            View::distribute(&mn, 4, out, |_, p, vs| {
                let rel = self.velocity(p, t, ramp) - vs;
                let vn = rel - mn.x * mn.x.dot(&rel);
                let s = vn.norm();
                shed_speed += s;
                samples += 1.0;
                if s < 1e-9 {
                    return Vec3::zeros();
                }
                let (vy, vz) = (vn.dot(&mn.y), vn.dot(&mn.z));
                let (cy, cz) = drag_coefficients(&g.shape, g.depth_y, g.depth_z, s);
                (mn.y * (cy * g.depth_z * vy) + mn.z * (cz * g.depth_y * vz))
                    * (0.5 * rho * s * psi * scale)
            });
            if self.vortex_shedding && samples > 0.0 {
                let s = shed_speed / samples;
                let (st, cl, d) = match g.shape {
                    Shape::Tube { ro, .. } => (0.2, 0.25, 2.0 * ro),
                    Shape::Rect { .. } => (0.12, 0.6, 0.5 * (g.depth_y + g.depth_z)),
                };
                let f = st * s / d;
                let phase = (self.state.shed_phase[m] + std::f64::consts::TAU * f * dt)
                    % std::f64::consts::TAU;
                self.state.shed_phase[m] = phase;
                let fade = 1.0 - super::smooth_ramp(f - 4.0, 4.0);
                if fade > 0.0 && s > 1e-6 {
                    let (pm, _) = mn.at(0.5);
                    let rel = self.velocity(pm, t, ramp) - 0.5 * (mn.va + mn.vb);
                    let vn = rel - mn.x * mn.x.dot(&rel);
                    let lift_dir = mn.x.cross(&vn);
                    if lift_dir.norm() > 1e-9 {
                        let fl = lift_dir.normalize()
                            * (0.5
                                * rho
                                * cl
                                * d
                                * s
                                * s
                                * phase.sin()
                                * fade
                                * psi
                                * scale
                                * mn.length);
                        out[mn.a] += 0.5 * fl;
                        out[mn.b] += 0.5 * fl;
                    }
                }
            }
        }
    }

    /// Fan wall state at case time `t` with window ramp `ramp`.
    pub fn rig(&self, t: f64, ramp: f64) -> WindRig {
        let gust = if self.state.components.is_empty() {
            0.0
        } else {
            self.gust_raw(t) * self.speed * ramp
        };
        let speed = (self.speed * ramp + gust).max(0.0);
        WindRig {
            fan_drive: if self.speed > 0.0 {
                (speed / (self.speed * 1.3)).clamp(0.0, 1.0)
            } else {
                0.0
            },
            speed,
            direction: self.dir(),
            gust,
        }
    }

    /// Dynamic pressure of the mean wind, Pa, for the brief.
    pub fn dynamic_pressure(&self) -> f64 {
        0.5 * self.air_density * self.speed * self.speed
    }
}

/// Drag coefficients for flow along local y and along local z, for a
/// section with outer extents `dy`, `dz` and normal flow speed `speed`.
pub fn drag_coefficients(shape: &Shape, dy: f64, dz: f64, speed: f64) -> (f64, f64) {
    match shape {
        Shape::Tube { ro, .. } => {
            let c = cylinder_drag(speed * 2.0 * ro / AIR_VISCOSITY);
            (c, c)
        }
        // Flow along y meets the face of width dz, depth dy along the flow.
        Shape::Rect { .. } => (rect_drag(dy / dz), rect_drag(dz / dy)),
    }
}

/// Drag coefficient of a smooth-ish circular cylinder against Reynolds
/// number, through the drag crisis.
pub fn cylinder_drag(re: f64) -> f64 {
    const T: [(f64, f64); 5] = [(1e4, 1.2), (2e5, 1.2), (4e5, 0.5), (1e6, 0.55), (1e7, 0.7)];
    log_interp(&T, re)
}

/// Force coefficient of a sharp-edged rectangle in flow normal to a face,
/// against depth (along the flow) over width (across it). EN 1991-1-4
/// Fig. 7.23.
pub fn rect_drag(d_over_b: f64) -> f64 {
    const T: [(f64, f64); 8] = [
        (0.1, 2.0),
        (0.2, 2.1),
        (0.6, 2.4),
        (0.7, 2.4),
        (1.0, 2.1),
        (2.0, 1.65),
        (5.0, 1.0),
        (10.0, 0.9),
    ];
    log_interp(&T, d_over_b)
}

/// Piecewise linear in log(x), clamped at the ends.
fn log_interp(t: &[(f64, f64)], x: f64) -> f64 {
    if x.is_nan() || x <= t[0].0 {
        return t[0].1;
    }
    for w in t.windows(2) {
        if x <= w[1].0 {
            let u = (x / w[0].0).ln() / (w[1].0 / w[0].0).ln();
            return w[0].1 + u * (w[1].1 - w[0].1);
        }
    }
    t[t.len() - 1].1
}
