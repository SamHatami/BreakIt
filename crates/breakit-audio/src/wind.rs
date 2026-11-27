//! Members singing in the wind (Aeolian tones).
//!
//! Wind past a member sheds vortices from alternate sides at the Strouhal
//! frequency `f = St·U / d`. Each vortex pushes the member sideways, so the
//! member feels a lift force that swings at `f`, and that force is also what
//! makes the sound, in two ways:
//!
//! * **The air itself.** A fluctuating force on a body much smaller than the
//!   wavelength radiates as a dipole (Curle 1955): at distance `r` the
//!   pressure amplitude is `ω F / 4π r c`. This is the hum of a wire in the
//!   wind, and needs no motion of the member at all.
//! * **The member.** The same force drives the member's bending modes. A mode
//!   moving at velocity `v` pushes the air it displaces and the added mass
//!   around it, `2 ρ₀ S L` for a slender body of cross-section outline `S`,
//!   which radiates as a dipole too. Per unit of lift the mode with modal
//!   mass `ρ A L / 2` therefore radiates
//!
//!   ```text
//!   p_member / p_air = 4 (ρ₀/ρ) (S/A) · β² / √((1 − β²)² + η²),   β = f / f_n
//!   ```
//!
//!   Away from a mode this is nothing (the density ratio is 10⁻³ to 10⁻⁴);
//!   at a mode it grows by `1/η`, so a lightly damped steel tube sings above
//!   the wind's own tone and a timber one barely answers. The modes are the
//!   member's real ones ([`Body::modes`]), so the pitch that rings out is the
//!   member's, not the wind's.
//!
//! Shedding is not a pure tone. Along a still member it is in step only over
//! a few depths, and its rate and strength wander by a few percent. When the
//! member moves with it (lock-in) the wake falls into step along the span
//! and the tone becomes steady. The caller folds the span into the force
//! ([`coherent_force`]); the voice makes the wander.

use std::f32::consts::{PI, TAU};

use crate::body::Drive;
use crate::dsp::Rng;
use crate::mixer::BLOCK;
use crate::modal::Bank;
use crate::Body;

/// Density of air, kg/m³, and speed of sound in it, m/s.
const AIR_DENSITY: f32 = 1.2;
const SOUND_SPEED: f32 = 343.0;

/// Distance the level is worked out at, m.
const REFERENCE_DISTANCE: f32 = 10.0;

/// Sound pressure at the reference distance that plays at full level, Pa
/// (94 dB SPL). Levels follow the same compressive law as the energy of a
/// break, `level ∝ (p²)^0.3`.
const FULL_PRESSURE: f32 = 1.0;
const OUTPUT: f32 = 0.8;

/// Shedding tones are heard between these, Hz.
pub const AUDIBLE: (f32, f32) = (25.0, 4000.0);

/// Most member modes a voice keeps (the lowest ones).
const MODES: usize = 16;

/// How far the shedding rate and strength wander (standard deviation, as a
/// fraction) on a still member, and when locked in.
const RATE_WANDER: (f32, f32) = (0.03, 0.002);
const STRENGTH_WANDER: (f32, f32) = (0.35, 0.05);

/// Lift amplitude that radiates like the whole span, N, for a lift per
/// length `per_length` (N/m) that is in step over `correlation` metres of a
/// member `length` long: the cells along the span add as random phases,
/// `F = f' √(L · min(L, l_c))`.
pub fn coherent_force(per_length: f32, length: f32, correlation: f32) -> f32 {
    let length = length.max(0.0);
    per_length.abs() * (length * correlation.clamp(0.0, length)).sqrt()
}

/// Sound pressure amplitude of a lift force `force` (N) swinging at
/// `frequency` (Hz), at the reference distance, Pa: `ω F / 4π r c`.
pub fn dipole_pressure(frequency: f32, force: f32) -> f32 {
    TAU * frequency * force / (4.0 * PI * REFERENCE_DISTANCE * SOUND_SPEED)
}

/// Level of the wind's own tone, 0..1.
pub fn tone_level(frequency: f32, force: f32) -> f32 {
    if !(frequency.is_finite() && force.is_finite())
        || frequency < AUDIBLE.0
        || frequency > AUDIBLE.1
        || force <= 0.0
    {
        return 0.0;
    }
    (dipole_pressure(frequency, force) / FULL_PRESSURE)
        .powf(0.6)
        .min(1.0)
}

/// How much more than the air a member's mode radiates per unit of lift when
/// it is driven exactly at its frequency: `4 (ρ₀/ρ) (S/A) / η`.
pub fn resonant_gain(body: &Body) -> f32 {
    let outline = body.width * body.depth;
    4.0 * (AIR_DENSITY / body.density) * (outline / body.area) / body.loss()
}

/// One member in the wind.
pub struct WindVoice {
    bank: Bank,
    rng: Rng,
    sr: f32,
    phase: f32,
    /// Where the game says the shedding is, and where the voice has got to.
    target_freq: f32,
    target_level: f32,
    target_lock: f32,
    freq: f32,
    level: f32,
    lock: f32,
    /// Wander of the shedding rate and strength (unit variance).
    rate: f32,
    strength: f32,
    force: [f32; BLOCK],
}

impl WindVoice {
    pub fn new(body: &Body, sr: f32, seed: u32) -> Option<Self> {
        let ok = [body.length, body.area, body.density, body.width, body.depth]
            .iter()
            .all(|x| x.is_finite() && *x > 0.0);
        if !ok || !(sr.is_finite() && sr > 0.0) {
            return None;
        }
        // The member's own modes, each as loud per unit of lift as the air
        // it moves. A two-pole mode driven at its frequency answers with
        // gain g/2(1 − r), 1 − r = πfη/sr, which must be 4(ρ₀/ρ)(S/A)/η.
        let outline = body.width * body.depth;
        let per_hz = 8.0 * PI * (AIR_DENSITY / body.density) * (outline / body.area) / sr;
        let mut modes = body.modes(Drive::transverse(0.37), 64, seed);
        modes.retain(|m| m.freq < AUDIBLE.1);
        modes.truncate(MODES);
        for m in &mut modes {
            m.gain = per_hz * m.freq;
        }
        Some(Self {
            bank: Bank::new(&modes, sr),
            rng: Rng::new(seed),
            sr,
            phase: 0.0,
            target_freq: 0.0,
            target_level: 0.0,
            target_lock: 0.0,
            freq: 0.0,
            level: 0.0,
            lock: 0.0,
            rate: 0.0,
            strength: 0.0,
            force: [0.0; BLOCK],
        })
    }

    /// The shedding now: its rate (Hz), the lift amplitude that radiates
    /// (N, see [`coherent_force`]) and whether the member moves with it.
    pub fn set(&mut self, frequency: f32, force: f32, locked: bool) {
        let level = tone_level(frequency, force);
        if level > 0.0 {
            self.target_freq = frequency;
            if self.freq <= 0.0 {
                self.freq = frequency;
            }
        }
        self.target_level = level * OUTPUT;
        self.target_lock = if locked { 1.0 } else { 0.0 };
    }

    /// The wind has dropped (or the game stopped reporting this member).
    pub fn release(&mut self) {
        self.target_level = 0.0;
    }

    pub fn quiet(&self) -> bool {
        self.target_level <= 0.0 && self.level < 1e-4 && self.bank.level() < 1e-4
    }

    /// Adds the voice to `out` (at most [`BLOCK`] samples per call).
    pub fn render(&mut self, out: &mut [f32]) {
        let n = out.len().min(BLOCK);
        let out = &mut out[..n];
        let dt = 1.0 / self.sr;
        // Gusts move the rate over tenths of a second; the wake's own
        // wander is quicker.
        let follow = 1.0 - (-dt / 0.08).exp();
        let k_rate = dt / 0.05;
        let k_strength = dt / 0.15;
        for (o, f) in out.iter_mut().zip(self.force.iter_mut()) {
            self.freq += (self.target_freq - self.freq) * follow;
            self.level += (self.target_level - self.level) * follow;
            self.lock += (self.target_lock - self.lock) * follow;
            // Ornstein-Uhlenbeck wander, unit variance (uniform noise has
            // variance 1/3).
            self.rate += -self.rate * k_rate + (6.0 * k_rate).sqrt() * self.rng.bipolar();
            self.strength +=
                -self.strength * k_strength + (6.0 * k_strength).sqrt() * self.rng.bipolar();
            let wander = RATE_WANDER.0 + (RATE_WANDER.1 - RATE_WANDER.0) * self.lock;
            let swell = STRENGTH_WANDER.0 + (STRENGTH_WANDER.1 - STRENGTH_WANDER.0) * self.lock;
            self.phase += TAU * self.freq * (1.0 + wander * self.rate) * dt;
            if self.phase > TAU {
                self.phase -= TAU;
            }
            let lift = self.level * (1.0 + swell * self.strength).max(0.0) * self.phase.sin();
            *f = lift;
            *o += lift;
        }
        self.bank.process(&self.force[..n], out);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use breakit_core::MaterialKind;

    const SR: f32 = 48_000.0;

    fn render(v: &mut WindVoice, seconds: f32) -> Vec<f32> {
        let mut out = vec![0.0; (seconds * SR) as usize];
        for chunk in out.chunks_mut(BLOCK) {
            v.render(chunk);
        }
        out
    }

    fn rms(x: &[f32]) -> f32 {
        (x.iter().map(|s| s * s).sum::<f32>() / x.len().max(1) as f32).sqrt()
    }

    /// Amplitude of `x` at `f` (Goertzel over the whole slice).
    fn at(x: &[f32], f: f32) -> f32 {
        let w = TAU * f / SR;
        let (mut re, mut im) = (0.0f64, 0.0f64);
        for (i, s) in x.iter().enumerate() {
            let p = (w * i as f32) as f64;
            re += *s as f64 * p.cos();
            im += *s as f64 * p.sin();
        }
        (2.0 * (re * re + im * im).sqrt() / x.len() as f64) as f32
    }

    #[test]
    fn dipole_pressure_matches_curle() {
        // 10 N swinging at 100 Hz, heard at 10 m: ωF/4πrc.
        let p = dipole_pressure(100.0, 10.0);
        assert!((p - 0.1458).abs() < 1e-3, "{p}");
        // Doubling the wind doubles the rate and quadruples the force.
        assert!((dipole_pressure(200.0, 40.0) / p - 8.0).abs() < 1e-3);
    }

    #[test]
    fn span_adds_as_random_cells() {
        // In step along the whole span: the full force.
        assert!((coherent_force(2.0, 4.0, 10.0) - 8.0).abs() < 1e-5);
        // In step over a sixteenth of it: a quarter.
        assert!((coherent_force(2.0, 4.0, 0.25) - 2.0).abs() < 1e-5);
    }

    #[test]
    fn tone_follows_the_shedding_rate() {
        let body = Body::typical(MaterialKind::Timber, 3.0);
        let mut v = WindVoice::new(&body, SR, 1).unwrap();
        v.set(180.0, 6.0, true);
        let x = render(&mut v, 1.5);
        let tail = &x[(0.5 * SR) as usize..];
        let on = at(tail, 180.0);
        assert!(on > 5.0 * at(tail, 120.0), "{on}");
        assert!(on > 5.0 * at(tail, 260.0), "{on}");
    }

    #[test]
    fn member_sings_when_the_wind_finds_its_mode() {
        let body = Body::typical(MaterialKind::Steel, 3.0);
        let modes = body.modes(Drive::transverse(0.37), 64, 7);
        let mode = modes.iter().find(|m| m.freq > 60.0).unwrap().freq;
        let level = |f: f32| {
            let mut v = WindVoice::new(&body, SR, 7).unwrap();
            // The same radiated tone both times, so only the member differs.
            let force = 4.0 * 100.0 / f;
            v.set(f, force, true);
            let x = render(&mut v, 4.0);
            rms(&x[(3.0 * SR) as usize..])
        };
        let on = level(mode);
        let off = level(mode * 1.2);
        assert!(resonant_gain(&body) > 0.5, "{}", resonant_gain(&body));
        assert!(on > 1.3 * off, "on {on} off {off}");
    }

    #[test]
    fn timber_answers_less_than_steel() {
        let steel = resonant_gain(&Body::typical(MaterialKind::Steel, 3.0));
        let timber = resonant_gain(&Body::typical(MaterialKind::Timber, 3.0));
        assert!(steel > timber, "steel {steel} timber {timber}");
    }

    #[test]
    fn locked_in_is_steadier() {
        let body = Body::typical(MaterialKind::Timber, 3.0);
        let spread = |locked: bool| {
            let mut v = WindVoice::new(&body, SR, 3).unwrap();
            v.set(200.0, 6.0, locked);
            let x = render(&mut v, 6.0);
            let levels: Vec<f32> = x[(1.0 * SR) as usize..]
                .chunks((0.05 * SR) as usize)
                .map(rms)
                .collect();
            let mean = levels.iter().sum::<f32>() / levels.len() as f32;
            let var = levels.iter().map(|l| (l - mean).powi(2)).sum::<f32>() / levels.len() as f32;
            var.sqrt() / mean
        };
        assert!(spread(true) < 0.5 * spread(false));
    }

    #[test]
    fn still_air_is_silent_and_bad_numbers_are_ignored() {
        let body = Body::typical(MaterialKind::Steel, 3.0);
        let mut v = WindVoice::new(&body, SR, 1).unwrap();
        assert!(v.quiet());
        assert!(rms(&render(&mut v, 0.2)) < 1e-9);
        for (f, force) in [
            (f32::NAN, 1.0),
            (100.0, f32::INFINITY),
            (5.0, 1e6),
            (100.0, -1.0),
        ] {
            v.set(f, force, false);
            assert!(render(&mut v, 0.1).iter().all(|s| s.is_finite()));
        }
        v.set(150.0, 5.0, false);
        render(&mut v, 0.5);
        v.release();
        let x = render(&mut v, 8.0);
        assert!(x.iter().all(|s| s.abs() <= 2.0));
        assert!(rms(&x[x.len() - 4800..]) < 1e-3);
    }
}
