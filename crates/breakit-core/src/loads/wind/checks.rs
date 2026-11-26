//! Engineering checks for wind briefs and for validating the wind model.
//!
//! Pure functions in SI units, each with the formula's source:
//!
//! * [`fundamental_frequency`]: Rayleigh estimate of a structure's first
//!   natural frequency from a static solve.
//! * Vortex shedding (EN 1991-1-4 Annex E.1): [`strouhal_number`],
//!   [`critical_vortex_speed`], [`scruton_number`].
//! * Galloping (EN 1991-1-4 E.2): [`galloping_factor`],
//!   [`galloping_onset_speed`], and Den Hartog's criterion
//!   [`den_hartog_unstable`].
//! * Classical flutter of a bridge deck: [`selberg_flutter_speed`] and
//!   [`rocard_flutter_speed`].
//! * Spanwise correlation of turbulence: [`davenport_coherence`].

use std::f64::consts::{PI, TAU};

use crate::section::Shape;
use crate::statics::{solve_static, SolveError};
use crate::{NodeLoad, Structure, Vec3};

/// Gravity used to build the Rayleigh load case, m/s². It cancels out of
/// the frequency; it only sets the size of the trial deflection.
const G: f64 = 9.81;

/// Why a check could not be evaluated.
#[derive(Debug, PartialEq)]
pub enum CheckError {
    /// The static solve failed (mechanism or no supports).
    Solve(SolveError),
    /// The direction vector has zero length.
    Direction,
    /// An extra point mass names a node that does not exist.
    Node(usize),
    /// No mass can move: every massive node is held by a support.
    NoDeflection,
}

impl std::fmt::Display for CheckError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Solve(e) => write!(f, "static solve failed: {e}"),
            Self::Direction => write!(f, "direction has zero length"),
            Self::Node(n) => write!(f, "point mass on missing node {n}"),
            Self::NoDeflection => write!(f, "no mass deflects under the trial load"),
        }
    }
}

impl std::error::Error for CheckError {}

impl From<SolveError> for CheckError {
    fn from(e: SolveError) -> Self {
        Self::Solve(e)
    }
}

/// First natural frequency (Hz) of the structure vibrating along
/// `direction`, by Rayleigh's method with lumped member masses (half of each
/// member's `ρ A L` to each end). See
/// [`fundamental_frequency_with_masses`].
pub fn fundamental_frequency(s: &Structure, direction: Vec3) -> Result<f64, CheckError> {
    fundamental_frequency_with_masses(s, direction, &[])
}

/// First natural frequency (Hz) along `direction`, with extra point masses
/// `(node, kg)` on top of the members' own lumped masses.
///
/// Rayleigh's quotient `ω² = (δᵀ K δ) / (δᵀ M δ)` with the trial shape `δ`
/// the static deflection under the weights `F_i = m_i g d` (d the unit
/// direction). Since `K δ = F`, the numerator is `Σ m_i g (d·δ_i)`:
///
/// `f = (1/2π) √( g Σ m_i (d·δ_i) / Σ m_i |δ_i|² )`
///
/// (Clough & Penzien, *Dynamics of Structures*, 2nd ed., §8-5; the
/// "gravity-load" shape). Rotary inertia is ignored. The static shape is
/// close to the first mode, so the estimate is an upper bound a fraction of
/// a percent high for beams (0.4 % for a cantilever); the lumped masses err
/// the other way, by O(1/n²) for n members per span.
pub fn fundamental_frequency_with_masses(
    s: &Structure,
    direction: Vec3,
    extra: &[(usize, f64)],
) -> Result<f64, CheckError> {
    let len = direction.norm();
    if len < 1e-12 || !len.is_finite() {
        return Err(CheckError::Direction);
    }
    let d = direction / len;
    let mut mass = vec![0.0; s.nodes.len()];
    for (i, m) in s.members.iter().enumerate() {
        let mm = m.material.density * m.section.area * s.member_length(i);
        mass[m.a] += 0.5 * mm;
        mass[m.b] += 0.5 * mm;
    }
    for &(node, m) in extra {
        *mass.get_mut(node).ok_or(CheckError::Node(node))? += m;
    }
    let loads: Vec<NodeLoad> = mass
        .iter()
        .enumerate()
        .filter(|(_, &m)| m > 0.0)
        .map(|(n, &m)| NodeLoad::force(n, d * (m * G)))
        .collect();
    let r = solve_static(s, &loads)?;
    let (mut work, mut inertia) = (0.0, 0.0);
    for (m, u) in mass.iter().zip(&r.displacements) {
        work += m * G * d.dot(u);
        inertia += m * u.norm_squared();
    }
    if inertia <= 0.0 || work <= 0.0 {
        return Err(CheckError::NoDeflection);
    }
    Ok((work / inertia).sqrt() / TAU)
}

/// Strouhal number for vortex shedding, EN 1991-1-4 Table E.1 and
/// Fig. E.1.
///
/// `depth_along` (d) is the section's extent along the wind, `width_across`
/// (b) across it, m (the reference width for the shedding frequency). A
/// circular section gives 0.18 for all Reynolds numbers (EN; textbooks
/// often quote 0.2 subcritical). Sharp-cornered rectangles follow the
/// curve of Fig. E.1 through d/b = 1: 0.12, 2: 0.06, 3: 0.06, 3.5: 0.15,
/// 5: 0.11, 10: 0.09 (the jump at 3.5 is the shear layers reattaching to
/// the sides), linearly interpolated, and clamped outside 1..10.
pub fn strouhal_number(shape: &Shape, depth_along: f64, width_across: f64) -> f64 {
    const T: [(f64, f64); 6] = [
        (1.0, 0.12),
        (2.0, 0.06),
        (3.0, 0.06),
        (3.5, 0.15),
        (5.0, 0.11),
        (10.0, 0.09),
    ];
    match shape {
        Shape::Tube { .. } => 0.18,
        Shape::Rect { .. } => lin_interp(&T, depth_along / width_across),
    }
}

/// Critical wind speed for vortex resonance, m/s: `v_crit = f_n b / St`,
/// EN 1991-1-4 (E.2). `f_n` natural frequency (Hz), `width_across` the
/// reference width b (diameter for a circle), m.
pub fn critical_vortex_speed(f_n: f64, width_across: f64, strouhal: f64) -> f64 {
    f_n * width_across / strouhal
}

/// Scruton number `Sc = 2 δ_s m_e / (ρ b²)`, EN 1991-1-4 (E.4): the mass-
/// damping parameter that sets the vortex-induced amplitude and the
/// galloping onset. `mass_per_length` m_e (kg/m), `log_decrement` δ_s the
/// structural logarithmic decrement (EN Table F.2: about 0.012 for a welded
/// steel stack), `rho` air density (kg/m³), `width_across` b (m).
pub fn scruton_number(
    mass_per_length: f64,
    log_decrement: f64,
    rho: f64,
    width_across: f64,
) -> f64 {
    2.0 * log_decrement * mass_per_length / (rho * width_across * width_across)
}

/// Galloping instability factor a_G for a sharp-cornered rectangle, from
/// EN 1991-1-4 Table E.7, against d/b (`depth_along` / `width_across`):
/// 1/3: 0.4, 1/2: 0.7, 2/3: 1.0, 1: 1.2, 1.5: 1.7, 2: 2.0, linearly
/// interpolated. `None` outside 1/3..2 (the standard forbids extrapolating)
/// and for circular sections, which do not gallop (a_G = 0 by symmetry).
pub fn galloping_factor(shape: &Shape, depth_along: f64, width_across: f64) -> Option<f64> {
    const T: [(f64, f64); 6] = [
        (1.0 / 3.0, 0.4),
        (0.5, 0.7),
        (2.0 / 3.0, 1.0),
        (1.0, 1.2),
        (1.5, 1.7),
        (2.0, 2.0),
    ];
    match shape {
        Shape::Tube { .. } => None,
        Shape::Rect { .. } => {
            let r = depth_along / width_across;
            (r >= T[0].0 - 1e-12 && r <= T[T.len() - 1].0 + 1e-12).then(|| lin_interp(&T, r))
        }
    }
}

/// Onset wind speed of galloping, m/s: `v_CG = 2 Sc f_n b / a_G`,
/// EN 1991-1-4 (E.18). `f_n` cross-wind natural frequency (Hz),
/// `width_across` b (m), `scruton` from [`scruton_number`], `a_g` from
/// [`galloping_factor`]. Infinite for `a_g ≤ 0` (no galloping).
pub fn galloping_onset_speed(f_n: f64, width_across: f64, scruton: f64, a_g: f64) -> f64 {
    if a_g <= 0.0 {
        return f64::INFINITY;
    }
    2.0 * scruton * f_n * width_across / a_g
}

/// Den Hartog's criterion for galloping: a section is aerodynamically
/// unstable across the wind when `dC_L/dα + C_D < 0` (lift slope per rad,
/// drag coefficient, both on the same reference width). Den Hartog,
/// *Mechanical Vibrations*, 4th ed. (1956), §7.8. A square has
/// dC_L/dα ≈ −2.7 and C_D ≈ 2.0 at α = 0 (Parkinson & Smith 1964): unstable.
pub fn den_hartog_unstable(dcl_dalpha: f64, cd: f64) -> bool {
    dcl_dalpha + cd < 0.0
}

/// Critical flutter speed of a bridge deck by Selberg's formula, m/s:
///
/// `U_f = 0.44 B ω_t √[ (1 − (ω_b/ω_t)²) √ν / μ ]`,
/// `ν = 8 r² / B²`, `μ = π ρ B² / (2 m)`
///
/// (Selberg 1961, in the form of Dyrbye & Hansen, *Wind Loads on
/// Structures*, 1997, §6.4). `width` B is the full deck width (m), `mass`
/// m the mass per length (kg/m), `mass_moment` I the polar mass moment per
/// length (kg·m²/m, r = √(I/m)), `f_bending` and `f_torsion` the first
/// vertical bending and torsion frequencies (Hz), `rho` the air density.
///
/// It is the flutter speed of a thin flat plate (Theodorsen forces) with
/// its elastic axis at mid width; a bluff deck flutters earlier, a
/// streamlined box close to it. `None` when the torsion frequency is not
/// above the bending frequency (the formula does not apply).
pub fn selberg_flutter_speed(
    width: f64,
    mass: f64,
    mass_moment: f64,
    f_bending: f64,
    f_torsion: f64,
    rho: f64,
) -> Option<f64> {
    if f_torsion <= f_bending || width <= 0.0 || mass <= 0.0 || mass_moment <= 0.0 {
        return None;
    }
    let ratio = f_bending / f_torsion;
    let r2 = mass_moment / mass;
    let nu = 8.0 * r2 / (width * width);
    let mu = PI * rho * width * width / (2.0 * mass);
    Some(0.44 * width * TAU * f_torsion * ((1.0 - ratio * ratio) * nu.sqrt() / mu).sqrt())
}

/// Critical flutter speed of a bridge deck by Rocard's formula, m/s, an
/// independent check on [`selberg_flutter_speed`]:
///
/// `U_f² = (2 r² ℓ² / (2 r² + ℓ²)) · (m / (ρ B²)) · (ω_t² − ω_b²)`
///
/// with ℓ = B/2 the half width (Rocard, *Dynamic Instability*, 1957, as
/// quoted by Arioli & Gazzola, arXiv:1612.03684, eq. 34). Same inputs and
/// units as [`selberg_flutter_speed`].
pub fn rocard_flutter_speed(
    width: f64,
    mass: f64,
    mass_moment: f64,
    f_bending: f64,
    f_torsion: f64,
    rho: f64,
) -> Option<f64> {
    if f_torsion <= f_bending || width <= 0.0 || mass <= 0.0 || mass_moment <= 0.0 {
        return None;
    }
    let r2 = mass_moment / mass;
    let l2 = 0.25 * width * width;
    let (wt, wb) = (TAU * f_torsion, TAU * f_bending);
    let u2 = 2.0 * r2 * l2 / (2.0 * r2 + l2) * mass / (rho * width * width) * (wt * wt - wb * wb);
    Some(u2.sqrt())
}

/// Davenport's coherence of turbulence at frequency `f` (Hz) between two
/// points `separation` (m) apart, in a mean wind `u_mean` (m/s):
/// `coh = exp(−C f Δ / U)` with decay constant `decay` C (about 10 for
/// lateral and vertical separations of the along-wind component).
/// Davenport, QJRMS 87 (1961).
pub fn davenport_coherence(f: f64, separation: f64, u_mean: f64, decay: f64) -> f64 {
    if u_mean <= 0.0 {
        return if f * separation == 0.0 { 1.0 } else { 0.0 };
    }
    (-decay * f * separation.abs() / u_mean).exp()
}

/// Piecewise linear in x, clamped at the ends.
fn lin_interp(t: &[(f64, f64)], x: f64) -> f64 {
    if x.is_nan() || x <= t[0].0 {
        return t[0].1;
    }
    for w in t.windows(2) {
        if x <= w[1].0 {
            let u = (x - w[0].0) / (w[1].0 - w[0].0);
            return w[0].1 + u * (w[1].1 - w[0].1);
        }
    }
    t[t.len() - 1].1
}
