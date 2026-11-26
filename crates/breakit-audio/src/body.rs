//! What rings: a member's vibration modes, computed from its geometry and
//! material.
//!
//! A [`Body`] is a prismatic bar: Young's modulus, shear modulus, density,
//! length, section area and second moments, outer dimensions, and how its
//! ends are held. [`Body::modes`] turns that into a list of modes (frequency,
//! decay rate, gain) using beam theory:
//!
//! * **Bending** about both section axes, `f_n = (β_n L)² / (2π L²) ·
//!   √(EI / ρA)`, with the wavenumbers `β_n L` of the end conditions and
//!   Timoshenko's shear and rotary-inertia correction, which matters for
//!   stocky concrete and for timber (E/G is about 16 along the grain).
//! * **Axial** (longitudinal) modes, `f_n = n c / 2L` with `c = √(E/ρ)`: a
//!   harmonic series. This is the ping of a bar struck on its end and the
//!   ring left by a tension member that snaps.
//! * **Wall** modes of hollow tubes: the ovalling modes of the ring,
//!   `f_i = i(i²−1)/√(i²+1) · √(E t² / 12ρ(1−ν²)R⁴) / 2π`, which give pipes
//!   their bright clang.
//!
//! Decay comes from the material's loss factor η: a mode's amplitude falls
//! as `exp(−π f η t)`, so high modes die first and steel (η about 0.001)
//! rings for seconds where timber (0.015) knocks and concrete (0.03) thuds.
//! Damage raises η: a cracked member sounds dead.
//!
//! A mode's gain is how loud it is per unit impulse at the drive point:
//! `ω · φ_n(x) · σ(f)`. The modal velocity after an impulse is the same for
//! every mode, radiated pressure follows surface acceleration (hence ω), φ is
//! the mode shape at the point that was hit or released, and σ is the
//! radiation efficiency: a bar narrower than the wavelength in air is a
//! dipole and a poor radiator (amplitude falls as f^1.5 below `ka = 1`), so
//! slender members sound thin and bright and fat ones sound full.
//!
//! The gains are relative. Absolute loudness is set by the voice from the
//! event's energy (see [`crate::material`]).

use std::f32::consts::{PI, TAU};

use breakit_core::{Material, Section, Shape};

use crate::dsp::Rng;
use crate::MaterialKind;

/// Speed of sound in air, m/s.
const C_AIR: f32 = 343.0;
/// Modes below this are felt, not heard, and cost as much as any other.
const F_MIN: f32 = 35.0;
/// Highest mode kept, Hz.
const F_MAX: f32 = 13_000.0;

/// How a bar's ends are held.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Ends {
    /// Both ends in joints: a member in the structure.
    Held,
    /// One end in a joint, one free: the stub left by a break.
    Cantilever,
    /// Both ends free: a loose piece.
    Free,
}

/// A prismatic bar that can ring. SI units.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Body {
    pub material: MaterialKind,
    /// Young's modulus along the member, Pa.
    pub youngs: f32,
    /// Shear modulus, Pa.
    pub shear: f32,
    /// Density, kg/m³.
    pub density: f32,
    /// Length, m.
    pub length: f32,
    /// Section area, m².
    pub area: f32,
    /// Second moments of area, m⁴.
    pub iy: f32,
    pub iz: f32,
    /// Outer width and depth of the section, m.
    pub width: f32,
    pub depth: f32,
    /// Wall thickness of a hollow section, m; 0 for a solid one.
    pub wall: f32,
    pub ends: Ends,
    /// Damage 0..1: raises the loss factor.
    pub damage: f32,
}

/// One vibration mode.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ModeSpec {
    /// Frequency, Hz.
    pub freq: f32,
    /// Amplitude decay rate, 1/s: amplitude falls as `exp(-decay t)`.
    pub decay: f32,
    /// Relative loudness per unit impulse at the drive point.
    pub gain: f32,
}

/// Where and how a body is driven.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Drive {
    /// Position along the bar, 0..1.
    pub at: f32,
    /// Share of the force across the bar (excites bending and wall modes).
    pub transverse: f32,
    /// Share of the force along the bar (excites axial modes).
    pub axial: f32,
}

impl Drive {
    pub fn transverse(at: f32) -> Self {
        Self {
            at,
            transverse: 1.0,
            axial: 0.0,
        }
    }
}

/// Material loss factor η of a member held in joints. Textbook structural
/// values: the joints, not the metal, set steel's.
fn loss_factor(material: MaterialKind) -> f32 {
    match material {
        MaterialKind::Steel => 0.0022,
        MaterialKind::Timber => 0.016,
        MaterialKind::Concrete => 0.028,
        MaterialKind::Composite => 0.006,
    }
}

impl Body {
    /// The body of a structural member.
    pub fn member(material: &Material, section: &Section, length: f64) -> Self {
        let (width, depth, wall) = match section.shape {
            Shape::Rect { b, h } => (b, h, 0.0),
            Shape::Tube { ro, t } => (2.0 * ro, 2.0 * ro, t),
        };
        Self {
            material: material.kind,
            youngs: material.youngs_modulus as f32,
            shear: material.shear_modulus as f32,
            density: material.density as f32,
            length: (length as f32).max(0.02),
            area: section.area as f32,
            iy: section.iy as f32,
            iz: section.iz as f32,
            width: width as f32,
            depth: depth as f32,
            wall: wall as f32,
            ends: Ends::Held,
            damage: 0.0,
        }
    }

    /// A typical member of a material: the catalogue default section. For
    /// tests, previews and loose pieces whose member is not known.
    pub fn typical(material: MaterialKind, length: f32) -> Self {
        let (m, s) = match material {
            MaterialKind::Steel => (Material::steel(), Section::tube(0.05, 0.005)),
            MaterialKind::Timber => (Material::timber(), Section::rect(0.1, 0.15)),
            MaterialKind::Concrete => (Material::reinforced_concrete(), Section::rect(0.25, 0.3)),
            MaterialKind::Composite => (Material::cfrp(), Section::tube(0.04, 0.004)),
        };
        Self::member(&m, &s, length as f64)
    }

    /// A loose piece of bounding radius `radius` (m) broken off a typical
    /// member. A piece shorter than its section is a chunk: the section is
    /// shrunk to fit.
    pub fn fragment(material: MaterialKind, radius: f32) -> Self {
        let length = (2.0 * radius).max(0.03);
        let mut b = Self::typical(material, length);
        let big = b.width.max(b.depth);
        if big > length {
            let k = length / big;
            b.width *= k;
            b.depth *= k;
            b.wall *= k;
            b.area *= k * k;
            b.iy *= k.powi(4);
            b.iz *= k.powi(4);
        }
        b.ends = Ends::Free;
        b
    }

    /// The same bar cut to `fraction` of its length, with new end conditions.
    pub fn piece(&self, fraction: f32, ends: Ends) -> Self {
        Self {
            length: (self.length * fraction).max(0.02),
            ends,
            ..*self
        }
    }

    pub fn with_damage(mut self, damage: f32) -> Self {
        self.damage = damage.clamp(0.0, 1.0);
        self
    }

    /// Mass, kg.
    pub fn mass(&self) -> f32 {
        self.density * self.area * self.length
    }

    /// Bar wave speed √(E/ρ), m/s.
    pub fn wave_speed(&self) -> f32 {
        (self.youngs / self.density).sqrt()
    }

    /// Loss factor including end conditions and damage.
    pub fn loss(&self) -> f32 {
        let ends = match self.ends {
            // Friction in the joints at both ends.
            Ends::Held => 1.0,
            Ends::Cantilever => 0.8,
            // A loose piece loses the joint friction but lies on the floor.
            Ends::Free => 0.7,
        };
        loss_factor(self.material) * ends * (1.0 + 3.0 * self.damage)
    }

    /// Frequency above which the bar radiates efficiently: where its half
    /// width times the wavenumber in air reaches 1.
    pub fn radiation_corner(&self) -> f32 {
        C_AIR / (PI * 0.5 * (self.width + self.depth).max(0.01))
    }

    /// Radiated pressure per unit surface velocity, 0..1. A bar vibrating
    /// sideways is a dipole: below the corner the air just sloshes round it
    /// and the radiated power falls as (ka)³, the amplitude as f^1.5.
    fn radiation(&self, f: f32) -> f32 {
        let x = f / self.radiation_corner();
        let a = x * x.sqrt();
        a / (1.0 + a)
    }

    /// The same for the end faces and joints an axial mode drives: small
    /// pistons, power as (ka)².
    fn end_radiation(&self, f: f32) -> f32 {
        let x = f / self.radiation_corner();
        x / (1.0 + x)
    }

    /// Bending wavelength at `f` Hz, m: how much of the bar an event of
    /// that frequency involves.
    pub fn bending_wavelength(&self, f: f32) -> f32 {
        let r = (self.iy.min(self.iz) / self.area).sqrt();
        (TAU * f.max(1.0) * self.wave_speed() * r).sqrt() / f.max(1.0)
    }

    fn decay(&self, f: f32) -> f32 {
        // Material loss plus a little air and support loss that does not
        // depend on frequency.
        PI * f * self.loss() + 0.6
    }

    /// Bending wavenumbers β_n L and whether the mode shape is that of a free
    /// end at 0 / at 1.
    fn bending_wavenumber(&self, n: usize) -> f32 {
        let n = n as f32;
        match self.ends {
            // Between pinned (nπ) and clamped ((n+½)π): real joints give
            // some rotational restraint.
            Ends::Held => (n + 0.25) * PI,
            Ends::Cantilever => (n - 0.5).max(0.597) * PI,
            Ends::Free => (n + 0.5) * PI,
        }
    }

    /// Bending mode shape at `x` (0..1), scaled so the interior amplitude is
    /// about √2 for every end condition.
    fn bending_shape(&self, n: usize, x: f32) -> f32 {
        let b = self.bending_wavenumber(n);
        let sqrt2 = std::f32::consts::SQRT_2;
        match self.ends {
            Ends::Held => sqrt2 * (n as f32 * PI * x).sin(),
            // Free end at x = 1, where the shape reaches 2.
            Ends::Cantilever => {
                let u = 1.0 - x;
                (sqrt2 * (b * u + PI / 4.0).cos() + (-b * u).exp()) * (1.0 - (-b * x).exp())
            }
            Ends::Free => {
                let sign = if n.is_multiple_of(2) { -1.0 } else { 1.0 };
                sqrt2 * (b * x + PI / 4.0).cos() + (-b * x).exp() + sign * (-b * (1.0 - x)).exp()
            }
        }
    }

    fn axial_mode(&self, n: usize, x: f32) -> (f32, f32) {
        let c = self.wave_speed();
        let n = n as f32;
        let sqrt2 = std::f32::consts::SQRT_2;
        match self.ends {
            Ends::Held => (n * c / (2.0 * self.length), sqrt2 * (n * PI * x).sin()),
            Ends::Cantilever => (
                (2.0 * n - 1.0) * c / (4.0 * self.length),
                sqrt2 * ((n - 0.5) * PI * x).sin(),
            ),
            Ends::Free => (n * c / (2.0 * self.length), sqrt2 * (n * PI * x).cos()),
        }
    }

    /// Every audible mode of the bar for a drive, unpruned.
    fn all_modes(&self, drive: Drive) -> Vec<ModeSpec> {
        let mut out = Vec::new();
        let x = drive.at.clamp(0.0, 1.0);
        let l = self.length;
        let c = self.wave_speed();
        let rho_a = self.density * self.area;

        // Bending about both axes.
        if drive.transverse > 0.0 {
            for (i, inertia) in [(0, self.iy), (1, self.iz)] {
                let r2 = inertia / self.area; // radius of gyration squared
                let base = (self.youngs * inertia / rho_a).sqrt() / (TAU * l * l);
                // Shear deformation and rotary inertia (Timoshenko), with a
                // shear coefficient of 5/6.
                let shear = 1.0 + self.youngs / (0.83 * self.shear);
                for n in 1..400 {
                    let b = self.bending_wavenumber(n);
                    let k = b / l;
                    let f = base * b * b / (1.0 + k * k * r2 * shear).sqrt();
                    if f > F_MAX {
                        break;
                    }
                    if f < F_MIN {
                        continue;
                    }
                    let shape = self.bending_shape(n, x).abs().max(0.2);
                    // A square or round section has the same modes on both
                    // axes; a real one is never perfect, so the second axis
                    // sits a touch off and the pair beats.
                    let f = if i == 1 { f * 1.013 } else { f };
                    out.push(ModeSpec {
                        freq: f,
                        decay: self.decay(f),
                        gain: drive.transverse * f * shape * self.radiation(f) * 0.71,
                    });
                }
            }
            // Ovalling modes of a hollow section's wall. Each has a family
            // of variants along the length just above it.
            if self.wall > 0.0 {
                let r = 0.25 * (self.width + self.depth) - 0.5 * self.wall;
                let ring = c * self.wall / (r * r * 12f32.sqrt() * 0.954) / TAU;
                for i in 2..7 {
                    let i = i as f32;
                    let f0 = ring * i * (i * i - 1.0) / (i * i + 1.0).sqrt();
                    for m in 0..3 {
                        let f = f0 * (1.0 + 0.021 * (m * m) as f32 + 0.008 * m as f32);
                        if !(F_MIN..=F_MAX).contains(&f) {
                            continue;
                        }
                        out.push(ModeSpec {
                            freq: f,
                            // Walls radiate well and lose more to the air.
                            decay: self.decay(f) * 1.5,
                            gain: drive.transverse * f * 0.8 * self.radiation(f) / (1.0 + m as f32),
                        });
                    }
                }
            }
        }

        // Axial modes. They move the surface only through Poisson
        // contraction, but drive the end faces and the joints directly.
        if drive.axial > 0.0 {
            for n in 1..200 {
                let (f, shape) = self.axial_mode(n, x);
                if f > F_MAX {
                    break;
                }
                if f < F_MIN {
                    continue;
                }
                out.push(ModeSpec {
                    freq: f,
                    decay: self.decay(f),
                    gain: drive.axial * f * shape.abs().max(0.2) * self.end_radiation(f) * 0.5,
                });
            }
        }
        out
    }

    /// At most `budget` modes for a drive. Where the bar has more audible
    /// modes than the budget (long slender members have hundreds), neighbours
    /// are merged: the strongest of each group stays and carries the group's
    /// energy. Each mode is detuned by a fraction of a percent from `seed`,
    /// as real members are never ideal. Gains are scaled so their squares sum
    /// to 1.
    pub fn modes(&self, drive: Drive, budget: usize, seed: u32) -> Vec<ModeSpec> {
        let mut all = self.all_modes(drive);
        all.sort_by(|a, b| a.freq.total_cmp(&b.freq));
        let budget = budget.max(1);
        let mut modes: Vec<ModeSpec> = if all.len() <= budget {
            all
        } else {
            let n = all.len();
            (0..budget)
                .map(|g| {
                    let group = &all[g * n / budget..(g + 1) * n / budget];
                    let energy: f32 = group.iter().map(|m| m.gain * m.gain).sum();
                    let mut best = group[0];
                    for m in group {
                        if m.gain > best.gain {
                            best = *m;
                        }
                    }
                    best.gain = energy.sqrt();
                    best
                })
                .collect()
        };
        let mut rng = Rng::new(seed ^ 0x51ED_270B);
        for m in &mut modes {
            m.freq *= 1.0 + 0.006 * rng.bipolar();
        }
        let norm = modes.iter().map(|m| m.gain * m.gain).sum::<f32>().sqrt();
        if norm > 0.0 {
            for m in &mut modes {
                m.gain /= norm;
            }
        }
        modes
    }

    /// Lowest bending frequency, Hz (ignoring the audibility floor).
    pub fn fundamental(&self) -> f32 {
        let b = self.bending_wavenumber(1);
        let i = self.iy.min(self.iz);
        (self.youngs * i / (self.density * self.area)).sqrt() * b * b
            / (TAU * self.length * self.length)
    }
}

/// What a loose piece lands on: the play table and the slab under it, as a
/// handful of heavily damped modes. It gives impacts their thump.
pub fn floor_modes(seed: u32) -> Vec<ModeSpec> {
    let mut rng = Rng::new(seed ^ 0x0F10_0812);
    // A stiff plate's modes are roughly evenly spaced in frequency.
    let mut modes: Vec<ModeSpec> = (0..10)
        .map(|i| {
            let f = 85.0 + 75.0 * i as f32 * rng.range(0.85, 1.15);
            ModeSpec {
                freq: f,
                decay: PI * f * 0.07 + 8.0,
                // A plate is a poor radiator at the bottom of its range too.
                gain: (f / 300.0).min(1.0) / (1.0 + f / 600.0),
            }
        })
        .collect();
    let norm = modes.iter().map(|m| m.gain * m.gain).sum::<f32>().sqrt();
    for m in &mut modes {
        m.gain /= norm;
    }
    modes
}

#[cfg(test)]
mod tests {
    use super::*;

    fn steel_bar(length: f32, ends: Ends) -> Body {
        let mut b = Body::member(
            &Material::steel(),
            &Section::rect(0.02, 0.02),
            length as f64,
        );
        b.ends = ends;
        b
    }

    /// A free steel bar 20 mm square and 1 m long: textbook free-free
    /// bending modes at 22.4, 61.7, 121 × √(EI/ρA) / 2πL².
    #[test]
    fn free_bar_matches_beam_theory() {
        let b = steel_bar(1.0, Ends::Free);
        let k = (210e9f32 * b.iy / (7850.0 * b.area)).sqrt() / TAU;
        let modes = b.all_modes(Drive::transverse(0.0));
        let mut f: Vec<f32> = modes.iter().map(|m| m.freq).collect();
        f.sort_by(f32::total_cmp);
        // Two axes: modes come in pairs 1.3% apart.
        for (found, exact) in [(f[0], 22.37), (f[2], 61.67), (f[4], 120.9)] {
            let exact = exact * k;
            assert!(
                (found / exact - 1.0).abs() < 0.03,
                "{found} Hz vs {exact} Hz"
            );
        }
        // About 105 Hz for the first mode.
        assert!((f[0] - 105.0).abs() < 6.0, "{}", f[0]);
    }

    #[test]
    fn axial_modes_are_harmonic_at_c_over_2l() {
        let b = steel_bar(2.0, Ends::Free);
        let modes = b.all_modes(Drive {
            at: 0.0,
            transverse: 0.0,
            axial: 1.0,
        });
        let c = (210e9f32 / 7850.0).sqrt();
        assert!((modes[0].freq - c / 4.0).abs() < 1.0, "{}", modes[0].freq);
        assert!((modes[1].freq / modes[0].freq - 2.0).abs() < 1e-3);
    }

    #[test]
    fn longer_is_lower_and_stiffer_is_higher() {
        let short = steel_bar(1.0, Ends::Held).fundamental();
        let long = steel_bar(2.0, Ends::Held).fundamental();
        assert!((short / long - 4.0).abs() < 0.01);
        let timber = Body::typical(MaterialKind::Timber, 3.0);
        let mut soft = timber;
        soft.youngs *= 0.25;
        assert!((timber.fundamental() / soft.fundamental() - 2.0).abs() < 0.01);
    }

    #[test]
    fn steel_rings_longer_than_timber_and_concrete() {
        let t60 = |m| 6.91 / Body::typical(m, 3.0).decay(1000.0);
        let (s, t, c) = (
            t60(MaterialKind::Steel),
            t60(MaterialKind::Timber),
            t60(MaterialKind::Concrete),
        );
        assert!(s > 0.7 && s < 2.0, "steel {s}");
        assert!(t > 0.08 && t < 0.25, "timber {t}");
        assert!(c < t, "concrete {c}");
        let cracked = Body::typical(MaterialKind::Concrete, 3.0).with_damage(1.0);
        assert!(6.91 / cracked.decay(1000.0) < 0.5 * c);
    }

    #[test]
    fn budget_is_kept_and_gains_are_normalised() {
        for m in [
            MaterialKind::Steel,
            MaterialKind::Timber,
            MaterialKind::Concrete,
            MaterialKind::Composite,
        ] {
            for len in [0.2, 1.0, 6.0] {
                let b = Body::typical(m, len);
                let modes = b.modes(
                    Drive {
                        at: 0.3,
                        transverse: 1.0,
                        axial: 0.5,
                    },
                    32,
                    7,
                );
                assert!(!modes.is_empty() && modes.len() <= 32, "{m:?} {len}");
                let e: f32 = modes.iter().map(|m| m.gain * m.gain).sum();
                assert!((e - 1.0).abs() < 1e-3);
                assert!(modes
                    .iter()
                    .all(|m| m.freq.is_finite() && m.freq >= F_MIN * 0.9 && m.decay > 0.0));
            }
        }
    }

    #[test]
    fn a_node_is_quiet_and_an_antinode_loud() {
        let b = steel_bar(1.0, Ends::Held);
        // Mode 2 has a node at mid-span.
        assert!(b.bending_shape(2, 0.5).abs() < 1e-3);
        assert!(b.bending_shape(1, 0.5) > 1.4);
        // A free end moves twice as far as the interior.
        let f = steel_bar(1.0, Ends::Free);
        assert!((f.bending_shape(3, 0.0).abs() - 2.0).abs() < 0.05);
        let c = steel_bar(1.0, Ends::Cantilever);
        assert!(c.bending_shape(3, 0.0).abs() < 1e-3);
        assert!((c.bending_shape(3, 1.0).abs() - 2.0).abs() < 0.05);
    }

    #[test]
    fn fragments_fit_their_radius() {
        let b = Body::fragment(MaterialKind::Concrete, 0.05);
        assert!(b.width <= b.length + 1e-6 && b.depth <= b.length + 1e-6);
        assert!(b.mass() < 3.0, "{}", b.mass());
    }
}
