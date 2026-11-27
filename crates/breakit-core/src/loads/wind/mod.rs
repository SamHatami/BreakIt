//! Wind: drag, turbulence, shielding, vortex shedding and galloping.
//!
//! A full flow simulation around a structure is far too slow for a game, so
//! this is the engineering model wind engineers design with: quasi-steady
//! forces from the local relative wind, a turbulent wind field with the
//! right spectrum and spatial coherence, and reduced-order models of the
//! unsteady effects that matter for structures.
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
//! * Turbulence ([`turbulence`]): a 3D divergence-free random field with the
//!   von Kármán spectrum, carried downwind with the mean wind, so gusts have
//!   a size: they hit neighbouring members together and distant ones apart.
//!   σ_u is constant with height (EN 1991-1-4), so the intensity grows
//!   towards the ground.
//! * Shielding: members are projected on the plane normal to the wind on a
//!   grid of cells about one member depth across. Going downwind, each member
//!   sees the velocity deficit the members upstream of it left in its cells,
//!   and adds its own (its solidity in the cell times an efficiency); the
//!   deficit recovers with distance downstream. This reproduces the drop in
//!   load on the leeward truss of a pair; it is an approximation, not a
//!   wake model.
//! * Vortex shedding ([`shedding`]): a wake oscillator per member, coupled
//!   to the member's motion, so shedding locks in to the structure near
//!   resonance.
//! * Galloping: rectangular sections get the quasi-steady cross-wind force
//!   of a bluff section at small angles of attack, whose negative slope
//!   (Den Hartog) feeds energy into cross-wind motion. A cubic in tan α with
//!   Parkinson and Smith's initial slope `A₁ = 2.69` for a square and a zero
//!   crossing at 13°, scaled to other depth ratios by the galloping factor
//!   of EN 1991-1-4 Table E.7 ([`checks::galloping_factor`]); none outside
//!   d/b from 1/3 to 2, where the table stops. It blends back into the drag model by 20°.
//!
//! Per frame the turbulence is evaluated at the nodes, and the members are
//! processed in parallel. [`Wind::member_aero`] exposes
//! each member's flow, shedding frequency and state for visuals and sound.

pub mod checks;
pub mod panels;
pub mod shedding;
pub mod turbulence;

use std::collections::HashMap;

use super::rng::Rng;
use super::View;
use crate::section::Shape;
use crate::{SimdLevel, Structure, Vec3};
use shedding::WakeOscillator;
use turbulence::TurbulenceField;

/// Kinematic viscosity of air at 15 °C, m²/s.
pub const AIR_VISCOSITY: f64 = 1.5e-5;
/// Density of air at 15 °C, sea level, kg/m³.
pub const AIR_DENSITY: f64 = 1.225;
/// Turbulence modes used by default.
pub const DEFAULT_MODES: usize = 96;
/// Parkinson and Smith's initial slope of the square's cross-wind force
/// coefficient against tan α.
pub const GALLOPING_A1: f64 = 2.69;
/// Angle (rad) where the square's quasi-steady cross-wind force changes sign.
pub const GALLOPING_ZERO: f64 = 0.227;
/// Angle (rad) by which the galloping force has blended into plain drag.
pub const GALLOPING_BLEND: f64 = 0.35;

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

/// Atmospheric turbulence, as a 3D field ([`turbulence`]).
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Gusts {
    /// Turbulence intensity σ_u / U at the reference height (0.1 to 0.25 is
    /// typical). σ_u is the same at every height (EN 1991-1-4 eq. 4.7), so
    /// the intensity grows towards the ground where the mean wind is slower.
    pub intensity: f64,
    /// Longitudinal integral length scale of the von Kármán spectrum, m.
    pub length_scale: f64,
    /// Seed of the random field; the same seed replays the same wind.
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

/// Wind on the structure: mean profile, turbulence and the aerodynamic
/// models above.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Wind {
    /// Mean speed at the reference height (or in the jet core), m/s.
    pub speed: f64,
    /// Direction the wind blows towards (horizontal, normalised on use).
    pub direction: Vec3,
    /// How the mean speed varies with height.
    pub profile: Profile,
    /// Turbulence, or `None` for a steady wind.
    pub gusts: Option<Gusts>,
    /// Height of the ground the profile starts from (table top), m.
    pub ground: f64,
    /// Air density, kg/m³.
    pub air_density: f64,
    /// Members behind other members see less wind.
    pub shielding: bool,
    /// Velocity deficit a fully solid cell leaves behind (0..1).
    pub shield_efficiency: f64,
    /// Wake recovery length in member depths.
    pub shield_recovery: f64,
    /// Vortex shedding forces (wake oscillators).
    pub vortex_shedding: bool,
    /// Quasi-steady galloping forces on rectangular members.
    #[cfg_attr(feature = "serde", serde(default = "yes"))]
    pub galloping: bool,
    /// Signs, cladding and plates that catch the wind.
    #[cfg_attr(feature = "serde", serde(default))]
    pub panels: Vec<panels::PanelSpec>,
    /// Number of random Fourier modes in the turbulence field.
    #[cfg_attr(feature = "serde", serde(default = "default_modes"))]
    pub turbulence_modes: usize,
    #[cfg_attr(feature = "serde", serde(skip))]
    state: Box<WindState>,
}

#[cfg(feature = "serde")]
fn yes() -> bool {
    true
}

#[cfg(feature = "serde")]
fn default_modes() -> usize {
    DEFAULT_MODES
}

/// How [`Wind::apply`] runs: instruction set and threads. Kept by
/// [`Wind::prepare`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Execution {
    /// Instruction set of the turbulence kernel. Default
    /// [`SimdLevel::from_env`]; [`SimdLevel::Scalar`] is the reference.
    pub simd: SimdLevel,
    /// Structures with at least this many members evaluate members and
    /// turbulence on the rayon pool. The answer is the same on any thread
    /// count, bit for bit. `usize::MAX` keeps it on the calling thread.
    ///
    /// The default is [`PARALLEL_MIN_MEMBERS`].
    pub parallel_min_members: usize,
}

/// Default of [`Execution::parallel_min_members`]. On the 4-core cloud
/// machine, with the solver stepping between frames, threads broke even at
/// about 500 members and took a third off from 1,300 (see `docs/loads.md`).
pub const PARALLEL_MIN_MEMBERS: usize = 768;

/// Two-point Gauss-Legendre points on [0, 1] (equal weights 1/2).
const GAUSS2: [f64; 2] = [0.211_324_865_405_187_1, 0.788_675_134_594_812_9];

/// Hashes a shielding cell: two multiplies, against SipHash's dozens. The
/// keys are small grid indices from our own geometry, not untrusted input.
#[derive(Default)]
struct CellHasher(u64);

impl std::hash::Hasher for CellHasher {
    fn finish(&self) -> u64 {
        self.0
    }
    fn write(&mut self, bytes: &[u8]) {
        for &b in bytes {
            self.write_u64(b as u64);
        }
    }
    fn write_i64(&mut self, v: i64) {
        self.write_u64(v as u64);
    }
    fn write_u64(&mut self, v: u64) {
        self.0 = (self.0.rotate_left(5) ^ v).wrapping_mul(0x51_7c_c1_b7_27_22_0a_95);
    }
}

type CellMap<V> = HashMap<(i64, i64), V, std::hash::BuildHasherDefault<CellHasher>>;

/// Members per parallel work item: a few microseconds of work each.
const MEMBER_CHUNK: usize = 32;
/// Nodes per parallel turbulence work item.
const NODE_CHUNK: usize = 16;

impl Default for Execution {
    fn default() -> Self {
        Self {
            simd: SimdLevel::from_env(),
            parallel_min_members: PARALLEL_MIN_MEMBERS,
        }
    }
}

/// The wind frame: along the wind, across it (horizontal) and up.
#[derive(Clone, Copy, Debug)]
struct Basis {
    d: Vec3,
    across: Vec3,
    up: Vec3,
}

/// Section coefficients that depend only on the member's shape, cached at
/// prepare.
#[derive(Clone, Copy, Debug, Default)]
struct SectionCoef {
    /// Rectangles: drag coefficients for flow along local y and z.
    rect_cy: f64,
    rect_cz: f64,
    /// Rectangles: drag on the face met by flow along y and along z, and
    /// the galloping factor (0 = none) for each.
    gallop_y: (f64, f64),
    gallop_z: (f64, f64),
}

#[derive(Clone, Debug, Default)]
struct WindState {
    exec: Option<Execution>,
    field: Option<TurbulenceField>,
    /// Point where the rig reports the wind: upstream edge, mid height.
    reference: Vec3,
    /// Shielding factor per member (on dynamic pressure).
    shield: Vec<f64>,
    shield_time: f64,
    shield_broken: usize,
    members: Vec<MemberState>,
    /// Gust at each node this frame (scratch kept between frames).
    gust_nodes: Vec<Vec3>,
    /// Resolved panels, in `panels` order.
    panels: Vec<Result<panels::PanelState, panels::PanelError>>,
    last_t: Option<f64>,
}

#[derive(Clone, Debug, Default)]
struct MemberState {
    wake: WakeOscillator,
    coef: SectionCoef,
    /// Midpoint velocity last frame, m/s.
    prev_v: Option<Vec3>,
    /// Running mean of the squared cross-wind velocity, m²/s².
    ydot2: f64,
    aero: MemberAero,
}

/// Flow and aerodynamic state of one member in the last frame, for visuals
/// (tufts, streamers, pressure colours) and sound (Aeolian tones).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct MemberAero {
    /// Relative wind normal to the member at its midpoint, m/s.
    pub flow: Vec3,
    /// Dynamic pressure of that flow, ½ρv², Pa.
    pub dynamic_pressure: f64,
    /// Shielding factor on the dynamic pressure (1 = in the open).
    pub shielding: f64,
    /// Total wind force on the member this frame, N.
    pub force: Vec3,
    /// Vortex shedding frequency, Hz (reported even when too fast to force).
    pub shedding_frequency: f64,
    /// Current lift coefficient of the shedding.
    pub lift_coefficient: f64,
    /// Direction of the shedding force (normal to member and flow).
    pub lift_direction: Vec3,
    /// Cross-wind motion amplitude over the member depth (estimate).
    pub amplitude_ratio: f64,
    /// The member is moving with the shedding (lock-in) or galloping.
    pub locked_in: bool,
    /// Section depth across the flow, m.
    pub depth: f64,
}

/// A member's shedding tone, for the sound.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SheddingTone {
    /// Member index.
    pub member: usize,
    /// Tone frequency, Hz.
    pub frequency: f64,
    /// Relative loudness: fluctuating lift force amplitude over the member, N.
    pub strength: f64,
    /// Member midpoint, m.
    pub position: Vec3,
}

/// State of the fan wall for the renderer and sound.
#[derive(Clone, Debug, Default)]
pub struct WindRig {
    /// Fan power 0..1 (mean plus gusts over the design speed).
    pub fan_drive: f64,
    /// Speed at the reference point including the gust, m/s.
    pub speed: f64,
    /// Mean wind direction (unit, horizontal).
    pub direction: Vec3,
    /// Current along-wind gust at the reference point, m/s.
    pub gust: f64,
    /// The loudest shedding tones this frame, loudest first (at most 8).
    pub tones: Vec<SheddingTone>,
    /// Signs and plates: pressure and force, for pressure colours.
    pub panels: Vec<panels::PanelRig>,
}

impl Wind {
    /// Steady uniform wind of `speed` (m/s) blowing along `direction`.
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
            galloping: true,
            turbulence_modes: DEFAULT_MODES,
            panels: Vec::new(),
            state: Box::default(),
        }
    }

    fn dir(&self) -> Vec3 {
        self.basis().d
    }

    /// Along the wind, and the plane normal to it: across (horizontal) and
    /// up.
    fn basis(&self) -> Basis {
        let d = Vec3::new(self.direction.x, 0.0, self.direction.z);
        let d = if d.norm() > 1e-12 {
            d.normalize()
        } else {
            Vec3::x()
        };
        Basis {
            d,
            across: Vec3::y().cross(&d).normalize(),
            up: Vec3::y(),
        }
    }

    fn cross_basis(&self) -> (Vec3, Vec3) {
        let b = self.basis();
        (b.across, b.up)
    }

    /// How [`Self::apply`] runs.
    pub fn execution(&self) -> Execution {
        self.state.exec.unwrap_or_default()
    }

    /// Set the instruction set and thread threshold of [`Self::apply`].
    pub fn set_execution(&mut self, exec: Execution) {
        self.state.exec = Some(exec);
    }

    /// Build the turbulence field and per-member state for structure `s`.
    pub fn prepare(&mut self, s: &Structure) {
        let d = self.dir();
        let (mut lo, mut y0, mut y1) = (f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY);
        for p in &s.nodes {
            lo = lo.min(p.dot(&d));
            y0 = y0.min(p.y);
            y1 = y1.max(p.y);
        }
        let reference = if lo.is_finite() {
            d * lo + Vec3::y() * (0.5 * (y0 + y1)).max(self.ground + 0.1)
        } else {
            Vec3::new(0.0, self.ground + 1.0, 0.0)
        };
        let field = self.gusts.as_ref().map(|g| {
            TurbulenceField::new(
                g.length_scale,
                self.speed,
                g.intensity,
                self.turbulence_modes,
                g.seed,
            )
        });
        let mut rng = Rng::new(self.gusts.as_ref().map_or(7, |g| g.seed ^ 0x5eed));
        let geometry = super::Geometry::new(s);
        let members = geometry
            .members
            .iter()
            .map(|g| MemberState {
                // Shedding starts at random phases: members are not
                // correlated until motion couples them.
                wake: WakeOscillator::with_phase(std::f64::consts::TAU * rng.uniform(), 1.0),
                coef: section_coef(g),
                ..Default::default()
            })
            .collect();
        let panels = self.panels.iter().map(|p| p.resolve(s)).collect();
        *self.state = WindState {
            exec: self.state.exec,
            field,
            reference,
            members,
            panels,
            ..Default::default()
        };
    }

    /// Aerodynamic state of every member in the last frame, in member order.
    pub fn member_aero(&self) -> impl ExactSizeIterator<Item = &MemberAero> + '_ {
        self.state.members.iter().map(|m| &m.aero)
    }

    pub fn profile_factor(&self, p: Vec3) -> f64 {
        self.profile_at(&self.basis(), p)
    }

    fn profile_at(&self, basis: &Basis, p: Vec3) -> f64 {
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
                let (across, up) = (basis.across, basis.up);
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

    /// Mean wind velocity at `p` with window ramp `ramp`, m/s.
    pub fn mean_velocity(&self, p: Vec3, ramp: f64) -> Vec3 {
        let b = self.basis();
        self.mean_at(&b, p, ramp)
    }

    fn mean_at(&self, b: &Basis, p: Vec3, ramp: f64) -> Vec3 {
        b.d * (self.speed * ramp * self.profile_at(b, p))
    }

    /// Turbulent fluctuation at `p` (m/s, world axes), scaled with the
    /// profile: zero below the ground and outside the fan jet, full where
    /// the mean wind is at least half its reference.
    fn gust_at(&self, b: &Basis, level: SimdLevel, p: Vec3, t: f64, ramp: f64) -> Vec3 {
        let (Some(f), Some(g)) = (&self.state.field, &self.gusts) else {
            return Vec3::zeros();
        };
        let gs = (self.profile_at(b, p) / 0.5).min(1.0);
        if gs == 0.0 {
            return Vec3::zeros();
        }
        let xi = Vec3::new(
            p.dot(&b.d) - self.speed.max(0.5) * t,
            p.dot(&b.across),
            p.dot(&b.up),
        );
        let u = f.sample_with(level, xi, t);
        let sigma = g.intensity * self.speed * ramp * gs;
        (b.d * u.x + b.across * (0.75 * u.y) + b.up * (0.5 * u.z)) * sigma
    }

    /// Wind velocity at `p`, `t` seconds after the case started, with the
    /// window ramp `ramp` (0..1): mean plus turbulence. For smoke streamers,
    /// tufts and payloads too.
    pub fn velocity(&self, p: Vec3, t: f64, ramp: f64) -> Vec3 {
        let b = self.basis();
        self.mean_at(&b, p, ramp) + self.gust_at(&b, self.execution().simd, p, t, ramp)
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
        let mut cells: CellMap<(f64, f64, f64)> = CellMap::default();
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
        let nm = v.structure.members.len();
        if self.state.members.len() != nm || (self.gusts.is_some() && self.state.field.is_none()) {
            let last = self.state.last_t;
            self.prepare(v.structure);
            self.state.last_t = last;
        }
        if ramp <= 0.0 || self.speed <= 0.0 {
            for m in &mut self.state.members {
                m.aero = MemberAero::default();
                m.prev_v = None;
            }
            return;
        }
        self.update_shielding(v);
        let exec = self.execution();
        let parallel = nm >= exec.parallel_min_members && rayon::current_num_threads() > 1;
        let mut members = std::mem::take(&mut self.state.members);
        let mut gust_nodes = std::mem::take(&mut self.state.gust_nodes);
        let me = &*self;
        let basis = self.basis();
        let threads = if parallel {
            rayon::current_num_threads()
        } else {
            1
        };
        // Turbulence at every node. Members interpolate between their ends:
        // they are short against the eddies that carry the energy (the
        // integral length scale is tens of metres).
        let nn = v.structure.nodes.len();
        gust_nodes.clear();
        if me.state.field.is_some() {
            gust_nodes.resize(nn, Vec3::zeros());
            let gust = |n0: usize, chunk: &mut [Vec3]| {
                for (i, g) in chunk.iter_mut().enumerate() {
                    *g = me.gust_at(&basis, exec.simd, v.sim.position(n0 + i), t, ramp);
                }
            };
            if threads > 1 {
                let mut slots: Vec<_> = gust_nodes.chunks_mut(NODE_CHUNK).collect();
                let helpers = (slots.len() / 4).min(threads - 1);
                crate::dynamics::pool::ordered_chunks(
                    &mut slots,
                    helpers,
                    |c, chunk| gust(c * NODE_CHUNK, chunk),
                    |_, _| {},
                );
            } else {
                gust(0, &mut gust_nodes);
            }
        }
        let frame = Frame {
            basis,
            ramp,
            scale,
            dt,
            gust_nodes: &gust_nodes,
        };
        // Each member's end forces, added in member order on this thread,
        // so the result does not depend on the threads.
        let member = |m0: usize, chunk: &mut [MemberState], forces: &mut Vec<_>| {
            forces.clear();
            for (i, ms) in chunk.iter_mut().enumerate() {
                forces.push(me.member_frame(v, &frame, m0 + i, ms));
            }
        };
        let add = |forces: &[(usize, usize, Vec3, Vec3)], out: &mut [Vec3]| {
            for &(a, b, fa, fb) in forces {
                out[a] += fa;
                out[b] += fb;
            }
        };
        if threads > 1 {
            let mut slots: Vec<_> = members
                .chunks_mut(MEMBER_CHUNK)
                .map(|c| (c, Vec::with_capacity(MEMBER_CHUNK)))
                .collect();
            let helpers = (slots.len() / 4).min(threads - 1);
            crate::dynamics::pool::ordered_chunks(
                &mut slots,
                helpers,
                |c, (chunk, forces)| member(c * MEMBER_CHUNK, chunk, forces),
                |_, (_, forces)| add(forces, out),
            );
        } else {
            let mut forces = Vec::with_capacity(nm);
            member(0, &mut members, &mut forces);
            add(&forces, out);
        }
        self.state.members = members;
        self.state.gust_nodes = gust_nodes;
        let mut states = std::mem::take(&mut self.state.panels);
        for (spec, st) in self.panels.iter().zip(states.iter_mut()) {
            if let Ok(st) = st {
                let wind = |p: Vec3| self.velocity(p, t, ramp);
                panels::apply_panel(spec, st, v, self.air_density, scale, &wind, out);
            }
        }
        self.state.panels = states;
    }

    /// Panels that could not be attached to the structure (missing corner
    /// nodes, degenerate shape), for the build check.
    pub fn panel_errors(&self) -> impl Iterator<Item = &panels::PanelError> + '_ {
        self.state.panels.iter().filter_map(|p| p.as_ref().err())
    }

    /// Wind forces on one member for this frame; advances its wake.
    fn member_frame(
        &self,
        v: &View,
        fr: &Frame,
        m: usize,
        ms: &mut MemberState,
    ) -> (usize, usize, Vec3, Vec3) {
        let Some(mn) = v.member(m) else {
            ms.aero = MemberAero::default();
            ms.prev_v = None;
            let mem = &v.structure.members[m];
            return (mem.a, mem.b, Vec3::zeros(), Vec3::zeros());
        };
        let g = &v.geometry.members[m];
        let psi = self.state.shield.get(m).copied().unwrap_or(1.0);
        let rho = self.air_density;
        let gust = |s: f64| -> Vec3 {
            if fr.gust_nodes.is_empty() {
                return Vec3::zeros();
            }
            fr.gust_nodes[mn.a] * (1.0 - s) + fr.gust_nodes[mn.b] * s
        };
        let wind_at = |s: f64| {
            let (p, _) = mn.at(s);
            self.mean_at(&fr.basis, p, fr.ramp) + gust(s)
        };

        // Drag (and galloping) by two-point Gauss quadrature along the
        // member: exact while the force per length times the lumping weight
        // is a cubic, which it is for a uniform wind on a member moving
        // rigidly until the normal flow changes sign.
        let ds = 0.5 * mn.length;
        let (mut fa, mut fb) = (Vec3::zeros(), Vec3::zeros());
        for s in GAUSS2 {
            let (_, vs) = mn.at(s);
            let rel = wind_at(s) - vs;
            let vn = rel - mn.x * mn.x.dot(&rel);
            let f = self.section_force(&mn, g, &ms.coef, vn) * (rho * psi * fr.scale * ds);
            fa += f * (1.0 - s);
            fb += f * s;
        }

        // Vortex shedding at the midpoint.
        let v_mid = 0.5 * (mn.va + mn.vb);
        let rel = wind_at(0.5) - v_mid;
        let vn = rel - mn.x * mn.x.dot(&rel);
        let un = vn.norm();
        let mut aero = MemberAero {
            flow: vn,
            dynamic_pressure: 0.5 * rho * un * un,
            shielding: psi,
            force: fa + fb,
            ..Default::default()
        };
        if un > 1e-6 {
            let (vy, vz) = (vn.dot(&mn.y).abs(), vn.dot(&mn.z).abs());
            // Depth along the flow and width across it.
            let (along, across) = match g.shape {
                Shape::Tube { ro, .. } => (2.0 * ro, 2.0 * ro),
                Shape::Rect { .. } if vy >= vz => (g.depth_y, g.depth_z),
                Shape::Rect { .. } => (g.depth_z, g.depth_y),
            };
            let re = un * across / AIR_VISCOSITY;
            let st = shedding::strouhal(&g.shape, along, across, re);
            let fs = st * un / across;
            let omega = std::f64::consts::TAU * fs;
            let lift_dir = mn.x.cross(&vn).normalize();
            let ydd = match (ms.prev_v, fr.dt > 0.0) {
                (Some(pv), true) => (v_mid - pv).dot(&lift_dir) / fr.dt,
                _ => 0.0,
            };
            let ydot = v_mid.dot(&lift_dir);
            let k = 1.0 - (-fr.dt / 1.0).exp();
            ms.ydot2 += (ydot * ydot - ms.ydot2) * k;
            let amp = (2.0 * ms.ydot2).sqrt() / omega.max(1e-6) / across;
            let fade = 1.0
                - super::smooth_ramp(
                    fs - 0.8 * shedding::MAX_FORCED_FREQUENCY,
                    0.2 * shedding::MAX_FORCED_FREQUENCY,
                );
            if self.vortex_shedding && fade > 0.0 {
                ms.wake.advance(fr.dt, omega, ydd, across);
            }
            let cl0 = shedding::lift_amplitude(&g.shape, along / across, re);
            let cl = 0.5 * cl0 * ms.wake.q;
            aero.shedding_frequency = fs;
            aero.lift_coefficient = cl;
            aero.lift_direction = lift_dir;
            aero.amplitude_ratio = amp;
            aero.locked_in = amp > 0.05;
            aero.depth = across;
            if self.vortex_shedding && fade > 0.0 {
                let l1 = shedding::correlation_length(amp) * across;
                let span = (l1 / mn.length).min(1.0).sqrt();
                let fl = lift_dir
                    * (0.5
                        * rho
                        * un
                        * un
                        * across
                        * cl
                        * mn.length
                        * span
                        * fade
                        * psi
                        * fr.scale);
                fa += 0.5 * fl;
                fb += 0.5 * fl;
                aero.force += fl;
            }
        }
        ms.prev_v = Some(v_mid);
        ms.aero = aero;
        (mn.a, mn.b, fa, fb)
    }

    /// Quasi-steady force per unit length over ρ for relative normal flow
    /// `vn`: drag by section, plus galloping for rectangles.
    fn section_force(
        &self,
        mn: &super::MemberNow,
        g: &super::MemberGeometry,
        coef: &SectionCoef,
        vn: Vec3,
    ) -> Vec3 {
        let s = vn.norm();
        if s < 1e-9 {
            return Vec3::zeros();
        }
        let (vy, vz) = (vn.dot(&mn.y), vn.dot(&mn.z));
        let (cy, cz) = match g.shape {
            Shape::Tube { ro, .. } => {
                let c = cylinder_drag(s * 2.0 * ro / AIR_VISCOSITY);
                (c, c)
            }
            Shape::Rect { .. } => (coef.rect_cy, coef.rect_cz),
        };
        let drag = (mn.y * (cy * g.depth_z * vy) + mn.z * (cz * g.depth_y * vz)) * (0.5 * s);
        if !self.galloping || !matches!(g.shape, Shape::Rect { .. }) {
            return drag;
        }
        // Main flow axis, the face it meets, and the angle of attack.
        let (main, cross, v_main, v_cross, b, (cd, factor)) = if vy.abs() >= vz.abs() {
            (mn.y, mn.z, vy, vz, g.depth_z, coef.gallop_y)
        } else {
            (mn.z, mn.y, vz, vy, g.depth_y, coef.gallop_z)
        };
        if factor <= 0.0 {
            return drag;
        }
        let tan_a = v_cross.abs() / v_main.abs().max(1e-12);
        if tan_a >= GALLOPING_BLEND.tan() {
            return drag;
        }
        let alpha = tan_a.atan();
        // Quasi-steady: drag along the main axis on the face b, and the
        // cross-wind coefficient (positive = against the cross flow, i.e.
        // with the member's motion that caused it).
        let q = 0.5 * s * s;
        let cfy = factor * galloping_coefficient(tan_a);
        let qs = main * (q * b * cd * v_main.signum()) - cross * (q * b * cfy * v_cross.signum());
        let w = 1.0 - super::smooth_ramp(alpha - GALLOPING_ZERO, GALLOPING_BLEND - GALLOPING_ZERO);
        qs * w + drag * (1.0 - w)
    }

    /// Fan wall state at case time `t` with window ramp `ramp`.
    pub fn rig(&self, t: f64, ramp: f64) -> WindRig {
        let p = self.state.reference;
        let w = self.velocity(p, t, ramp);
        let mean = self.mean_velocity(p, ramp).dot(&self.dir());
        let speed = w.dot(&self.dir()).max(0.0);
        let mut tones: Vec<SheddingTone> = self
            .state
            .members
            .iter()
            .enumerate()
            .filter(|(_, m)| m.aero.shedding_frequency > 0.0)
            .map(|(i, m)| SheddingTone {
                member: i,
                frequency: m.aero.shedding_frequency,
                strength: m.aero.dynamic_pressure
                    * m.aero.depth
                    * m.aero.lift_coefficient.abs().max(0.05),
                position: Vec3::zeros(),
            })
            .collect();
        tones.sort_by(|a, b| b.strength.total_cmp(&a.strength));
        tones.truncate(8);
        WindRig {
            fan_drive: if self.speed > 0.0 {
                (speed / (self.speed * 1.3)).clamp(0.0, 1.0)
            } else {
                0.0
            },
            speed,
            direction: self.dir(),
            gust: speed - mean,
            tones,
            panels: self
                .panels
                .iter()
                .zip(&self.state.panels)
                .filter_map(|(spec, st)| st.as_ref().ok().map(|st| st.rig(&spec.name)))
                .collect(),
        }
    }

    /// Dynamic pressure of the mean wind, Pa, for the brief.
    pub fn dynamic_pressure(&self) -> f64 {
        0.5 * self.air_density * self.speed * self.speed
    }
}

/// Shape-only coefficients of member `g` ([`SectionCoef`]).
fn section_coef(g: &super::MemberGeometry) -> SectionCoef {
    match g.shape {
        Shape::Tube { .. } => SectionCoef::default(),
        Shape::Rect { .. } => {
            let (rect_cy, rect_cz) = drag_coefficients(&g.shape, g.depth_y, g.depth_z, 0.0);
            // Flow along y meets the face of width depth_z, depth_y deep.
            let gallop = |d: f64, b: f64| {
                let factor = checks::galloping_factor(&g.shape, d, b).map_or(0.0, |a| a / 1.2);
                (rect_drag(d / b), factor)
            };
            SectionCoef {
                rect_cy,
                rect_cz,
                gallop_y: gallop(g.depth_y, g.depth_z),
                gallop_z: gallop(g.depth_z, g.depth_y),
            }
        }
    }
}

/// Per-frame inputs shared by every member.
struct Frame<'a> {
    basis: Basis,
    ramp: f64,
    scale: f64,
    dt: f64,
    gust_nodes: &'a [Vec3],
}

/// Quasi-steady cross-wind force coefficient of a square section against
/// tan α: a cubic with Parkinson and Smith's initial slope
/// [`GALLOPING_A1`] and its zero at [`GALLOPING_ZERO`] (peak 0.23 at 7.5°).
/// Positive drives the motion that produced the angle (Den Hartog unstable).
pub fn galloping_coefficient(tan_alpha: f64) -> f64 {
    let x = tan_alpha.abs();
    let z = GALLOPING_ZERO.tan();
    GALLOPING_A1 * x * (1.0 - (x / z).powi(2))
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
