//! Wavemaker theory for the solver's piston paddle, and paddle programs.
//!
//! The solver's waves obey the dispersion relation of its own model,
//! `ω² = g k² d / (1 + (kd)²/4)`, close to but not the same as the exact
//! `ω² = g k tanh(kd)`. The helpers here use the model's relation, so a
//! paddle program asking for a wave height gets that height in the
//! simulated water.

use std::f64::consts::TAU;

/// Wave number (rad/m) of waves of angular frequency `omega` on depth
/// `depth` in the solver's model. `None` above the model's cut-off
/// `ω = 2 √(g/d)`, where it carries no waves (period below about
/// `π √(d/g)`: 1.6 s on 2.6 m).
pub fn model_wave_number(omega: f64, depth: f64, gravity: f64) -> Option<f64> {
    let den = gravity * depth - 0.25 * omega * omega * depth * depth;
    (den > 0.0 && omega > 0.0).then(|| omega / den.sqrt())
}

/// Phase speed (m/s) of waves of wave number `k` on depth `depth` in the
/// solver's model.
pub fn model_phase_speed(k: f64, depth: f64, gravity: f64) -> f64 {
    (gravity * depth / (1.0 + 0.25 * (k * depth).powi(2))).sqrt()
}

/// Wave height over piston stroke in the solver's model: `H/S = k d`. A
/// depth-averaged model has no evanescent modes at the paddle, so this is
/// the long-wave limit of Biésel's transfer function; the two agree to 3 %
/// up to `kd = 1.2`.
pub fn model_height_to_stroke(k: f64, depth: f64) -> f64 {
    k * depth
}

/// What the paddle does over time.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Program {
    /// Parked.
    Still,
    /// Regular waves of `height` (m, crest to trough) and `period` (s),
    /// faded in over `ramp` seconds.
    Regular {
        /// Target wave height, m.
        height: f64,
        /// Period, s.
        period: f64,
        /// Fade-in time, s.
        ramp: f64,
    },
    /// One solitary wave of `height` (m above still water), the paddle
    /// stroke centred on time `at` (s). Goring's method: the paddle moves
    /// with the water velocity under the wave it is making.
    Solitary {
        /// Crest height above still water, m.
        height: f64,
        /// Time of the middle of the stroke, s.
        at: f64,
    },
}

impl Program {
    /// Paddle displacement from rest (m) and velocity (m/s) at time `t` on
    /// still depth `depth`.
    pub fn motion(&self, t: f64, depth: f64, gravity: f64) -> (f64, f64) {
        match *self {
            Program::Still => (0.0, 0.0),
            Program::Regular {
                height,
                period,
                ramp,
            } => {
                let omega = TAU / period;
                let Some(k) = model_wave_number(omega, depth, gravity) else {
                    return (0.0, 0.0);
                };
                let half = 0.5 * height / model_height_to_stroke(k, depth);
                let (f, df) = if ramp > 0.0 && t < ramp {
                    let x = (t / ramp).max(0.0);
                    (x * x * (3.0 - 2.0 * x), 6.0 * x * (1.0 - x) / ramp)
                } else {
                    (1.0, 0.0)
                };
                let (s, c) = (omega * t).sin_cos();
                // x = −f S/2 cos ωt + S/2 f: starts at rest, at rest position.
                (
                    half * f * (1.0 - c),
                    half * (df * (1.0 - c) + f * omega * s),
                )
            }
            Program::Solitary { height, at } => {
                if height <= 0.0 || depth <= 0.0 {
                    return (0.0, 0.0);
                }
                let kappa = (3.0 * height / (4.0 * depth.powi(3))).sqrt();
                let c = (gravity * (depth + height)).sqrt();
                let amp = height / (kappa * depth);
                // ξ = amp tanh(κ(c τ − ξ)), by Newton.
                let tau = t - at;
                let mut xi = 0.0;
                for _ in 0..30 {
                    let th = (kappa * (c * tau - xi)).tanh();
                    let f = xi - amp * th;
                    let df = 1.0 + amp * kappa * (1.0 - th * th);
                    let step = f / df;
                    xi -= step;
                    if step.abs() < 1e-13 {
                        break;
                    }
                }
                let sech2 = 1.0 - (kappa * (c * tau - xi)).tanh().powi(2);
                let eta = height * sech2;
                (xi + amp, c * eta / (depth + eta))
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn model_wave_number_tends_to_shallow_water() {
        let k = model_wave_number(0.2, 1.0, 9.81).expect("far below cut-off");
        assert!((k - 0.2 / 9.81_f64.sqrt()).abs() < 1e-3);
        assert!(model_wave_number(10.0, 2.0, 9.81).is_none());
    }

    #[test]
    fn programs_start_at_rest_and_match_their_velocity() {
        let programs = [
            Program::Regular {
                height: 0.3,
                period: 3.0,
                ramp: 4.0,
            },
            Program::Solitary {
                height: 0.5,
                at: 6.0,
            },
        ];
        for p in programs {
            let (x0, v0) = p.motion(0.0, 2.0, 9.81);
            assert!(x0.abs() < 1e-4 && v0.abs() < 1e-4, "{p:?}: {x0} {v0}");
            for t in [1.0, 3.3, 5.9, 6.2, 9.0] {
                let e = 1e-5;
                let (a, _) = p.motion(t - e, 2.0, 9.81);
                let (b, _) = p.motion(t + e, 2.0, 9.81);
                let (_, v) = p.motion(t, 2.0, 9.81);
                assert!(((b - a) / (2.0 * e) - v).abs() < 1e-5, "{p:?} at {t}");
            }
        }
    }

    #[test]
    fn solitary_stroke_is_gorings() {
        let (h, d) = (0.6, 2.0);
        let (x, _) = Program::Solitary { height: h, at: 0.0 }.motion(60.0, d, 9.81);
        assert!((x - (16.0 * h * d / 3.0_f64).sqrt()).abs() < 1e-6);
    }
}
