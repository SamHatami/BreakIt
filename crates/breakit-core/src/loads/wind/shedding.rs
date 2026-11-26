//! Vortex shedding: a wake oscillator per member.
//!
//! Wind past a bluff member sheds vortices from alternate sides at the
//! Strouhal frequency `f_s = St U / D`, which pushes the member back and
//! forth across the wind. When `f_s` comes near a natural frequency of the
//! structure, the motion takes over the shedding (lock-in): the vortices
//! follow the structure's frequency over a band of wind speeds and the
//! motion can grow to a large fraction of the member depth. That is how
//! chimneys, lamp posts and bridge stays sing, fatigue and sometimes fail.
//!
//! Each member carries a van der Pol wake oscillator (Facchinetti, de Langre
//! and Biolley 2004, J. Fluids Struct. 19):
//!
//! `q̈ + ε Ω (q² − 1) q̇ + Ω² q = (A / D) ÿ`,   `C_L = (C_L0 / 2) q`
//!
//! with `Ω = 2π f_s`, `ε = 0.3`, `A = 12` (their acceleration coupling) and
//! `ÿ` the member's cross-wind acceleration. Left alone, `q` settles into a
//! limit cycle of amplitude 2 at the Strouhal frequency, so the lift
//! coefficient swings by `C_L0`. Coupled to a moving member it synchronises
//! with the motion near resonance, which reproduces the lock-in band and
//! the self-limiting amplitude.
//!
//! The lift acting on a long member is reduced by the spanwise correlation
//! of the shedding: away from lock-in the vortices are coherent over only
//! about six depths; with motion the correlation length grows (EN 1991-1-4
//! E.1.5.2, `L₁/b` from 6 to 12 with the amplitude). Members get the
//! factor `√(L₁ / L)` for an uncorrelated force over the rest of their length.
//!
//! The solver takes forces once per frame, so shedding above
//! [`MAX_FORCED_FREQUENCY`] does not push the structure (it would alias);
//! its frequency is still reported for sound (the Aeolian tone of a thin
//! stay is in the hundreds of hertz).

use crate::section::Shape;

/// van der Pol damping of the wake oscillator.
pub const WAKE_EPSILON: f64 = 0.3;
/// Acceleration coupling of the wake oscillator.
pub const WAKE_COUPLING: f64 = 12.0;
/// Shedding above this frequency (Hz) is faded out of the forces.
pub const MAX_FORCED_FREQUENCY: f64 = 10.0;

/// Shedding state of one member.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WakeOscillator {
    /// Wake variable q (lift coefficient over C_L0 / 2).
    pub q: f64,
    /// dq/dt, 1/s.
    pub qd: f64,
}

impl Default for WakeOscillator {
    fn default() -> Self {
        Self { q: 0.1, qd: 0.0 }
    }
}

impl WakeOscillator {
    /// Start with a given phase (rad) on the free limit cycle at angular
    /// frequency `omega` (rad/s).
    pub fn with_phase(phase: f64, omega: f64) -> Self {
        Self {
            q: 2.0 * phase.cos(),
            qd: -2.0 * omega * phase.sin(),
        }
    }

    /// Advance by `dt` (s) at shedding angular frequency `omega` (rad/s)
    /// with cross-wind acceleration `ydd` (m/s²) of a member of depth `d`
    /// (m), held over the step. Substeps keep `Ω h` small.
    pub fn advance(&mut self, dt: f64, omega: f64, ydd: f64, d: f64) {
        if dt <= 0.0 || omega <= 0.0 {
            return;
        }
        let n = ((omega * dt / 0.1).ceil() as usize).clamp(1, 400);
        let h = dt / n as f64;
        let forcing = WAKE_COUPLING / d.max(1e-6) * ydd;
        for _ in 0..n {
            // Semi-implicit Euler with the damping term implicit in q̇.
            let c = WAKE_EPSILON * omega * (self.q * self.q - 1.0);
            self.qd = (self.qd + h * (forcing - omega * omega * self.q)) / (1.0 + h * c);
            self.q += h * self.qd;
        }
        if !self.q.is_finite() || !self.qd.is_finite() {
            *self = Self::default();
        }
    }
}

/// Strouhal number of a section, depth `depth_along` along the flow and
/// `width_across` across it: EN 1991-1-4 Table E.1 through
/// [`super::checks::strouhal_number`] (circular 0.18), and 0.25 for a
/// circular section past the drag crisis.
pub fn strouhal(shape: &Shape, depth_along: f64, width_across: f64, reynolds: f64) -> f64 {
    match shape {
        Shape::Tube { .. } if reynolds > 5e5 => 0.25,
        _ => super::checks::strouhal_number(shape, depth_along, width_across),
    }
}

/// Amplitude of the fluctuating lift coefficient C_L0 for a section.
/// Circular: 0.3 subcritical, 0.15 past the drag crisis (Blevins, Flow-Induced
/// Vibration, Table 3-2); rectangles about 1.0 for a square (Vickery 1966),
/// less for elongated sections.
pub fn lift_amplitude(shape: &Shape, d_over_b: f64, reynolds: f64) -> f64 {
    match shape {
        Shape::Tube { .. } => {
            if reynolds > 3e5 {
                0.15
            } else {
                0.3
            }
        }
        Shape::Rect { .. } => (1.0 / d_over_b.max(1.0)).sqrt().max(0.4),
    }
}

/// Spanwise correlation length over depth, `L₁ / b`, for a cross-wind
/// amplitude over depth `y_over_b` (EN 1991-1-4 Table E.4).
pub fn correlation_length(y_over_b: f64) -> f64 {
    let y = y_over_b.abs();
    if y < 0.1 {
        6.0
    } else if y < 0.6 {
        4.8 + 12.0 * y
    } else {
        12.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn free_wake_oscillates_at_the_strouhal_frequency_with_amplitude_two() {
        let omega = std::f64::consts::TAU * 2.0;
        let mut w = WakeOscillator::default();
        let dt = 1.0 / 60.0;
        for _ in 0..600 {
            w.advance(dt, omega, 0.0, 0.1);
        }
        let mut peak: f64 = 0.0;
        let mut crossings = 0;
        let mut last = w.q;
        for _ in 0..600 {
            w.advance(dt, omega, 0.0, 0.1);
            peak = peak.max(w.q.abs());
            if last.signum() != w.q.signum() {
                crossings += 1;
            }
            last = w.q;
        }
        assert!((peak - 2.0).abs() < 0.05, "limit cycle {peak}");
        // 10 s at 2 Hz: 40 zero crossings.
        assert!((38..=42).contains(&crossings), "{crossings}");
    }
}
