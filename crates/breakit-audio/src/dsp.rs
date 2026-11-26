//! Small DSP building blocks every patch is made from.
//!
//! All of them are per-sample, allocation free after construction and
//! `Send`, so voices can live on the audio thread. Frequencies are in Hz and
//! times in seconds; each block is told the sample rate when it is built.

use std::f32::consts::{PI, TAU};

/// Fast deterministic noise source (xorshift32). Seeded, so a sound is the
/// same every time it is rendered with the same seed.
#[derive(Clone, Debug)]
pub struct Rng(u32);

impl Rng {
    pub fn new(seed: u32) -> Self {
        // Scramble so that nearby seeds give unrelated streams, never zero.
        let mut s = seed.wrapping_mul(0x9E37_79B9) ^ 0x85EB_CA6B;
        s ^= s >> 16;
        Self(s.max(1))
    }

    #[inline]
    pub fn next_u32(&mut self) -> u32 {
        let mut x = self.0;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        self.0 = x;
        x
    }

    /// Uniform in [0, 1).
    #[inline]
    pub fn unit(&mut self) -> f32 {
        (self.next_u32() >> 8) as f32 * (1.0 / 16_777_216.0)
    }

    /// Uniform in [-1, 1): white noise.
    #[inline]
    pub fn bipolar(&mut self) -> f32 {
        self.unit() * 2.0 - 1.0
    }

    /// Uniform in [lo, hi).
    #[inline]
    pub fn range(&mut self, lo: f32, hi: f32) -> f32 {
        lo + (hi - lo) * self.unit()
    }

    /// True with probability `p`.
    #[inline]
    pub fn chance(&mut self, p: f32) -> bool {
        self.unit() < p
    }

    /// Exponentially distributed waiting time for a Poisson process of
    /// `rate` events per second.
    pub fn exp_interval(&mut self, rate: f32) -> f32 {
        if rate <= 1e-6 {
            return f32::INFINITY;
        }
        -(1.0 - self.unit()).max(1e-7).ln() / rate
    }

    /// Multiplies `x` by a random factor in [1 - spread, 1 + spread].
    pub fn vary(&mut self, x: f32, spread: f32) -> f32 {
        x * (1.0 + spread * self.bipolar())
    }
}

/// Pink (-3 dB/octave) noise, Paul Kellet's economy filter.
#[derive(Clone, Debug)]
pub struct Pink {
    rng: Rng,
    b: [f32; 3],
}

impl Pink {
    pub fn new(seed: u32) -> Self {
        Self {
            rng: Rng::new(seed),
            b: [0.0; 3],
        }
    }

    #[inline]
    pub fn next(&mut self) -> f32 {
        let w = self.rng.bipolar();
        self.b[0] = 0.99765 * self.b[0] + w * 0.0990460;
        self.b[1] = 0.96300 * self.b[1] + w * 0.2965164;
        self.b[2] = 0.57000 * self.b[2] + w * 1.0526913;
        (self.b[0] + self.b[1] + self.b[2] + w * 0.1848) * 0.25
    }
}

/// Brown (-6 dB/octave) noise: leaky integrated white noise.
#[derive(Clone, Debug)]
pub struct Brown {
    rng: Rng,
    y: f32,
}

impl Brown {
    pub fn new(seed: u32) -> Self {
        Self {
            rng: Rng::new(seed),
            y: 0.0,
        }
    }

    #[inline]
    pub fn next(&mut self) -> f32 {
        self.y = (self.y * 0.996 + self.rng.bipolar() * 0.06).clamp(-1.0, 1.0);
        self.y * 3.0
    }
}

/// One-pole low-pass / high-pass.
#[derive(Clone, Debug)]
pub struct OnePole {
    a: f32,
    y: f32,
}

impl OnePole {
    pub fn new(cutoff: f32, sr: f32) -> Self {
        let mut f = Self { a: 0.0, y: 0.0 };
        f.set_cutoff(cutoff, sr);
        f
    }

    pub fn set_cutoff(&mut self, cutoff: f32, sr: f32) {
        self.a = 1.0 - (-TAU * cutoff.clamp(1.0, sr * 0.49) / sr).exp();
    }

    #[inline]
    pub fn lp(&mut self, x: f32) -> f32 {
        self.y += self.a * (x - self.y);
        self.y
    }

    #[inline]
    pub fn hp(&mut self, x: f32) -> f32 {
        x - self.lp(x)
    }
}

/// Topology-preserving state variable filter (Zavalishin). Stable under
/// fast cutoff modulation, which the patches use a lot.
#[derive(Clone, Debug)]
pub struct Svf {
    g: f32,
    k: f32,
    ic1: f32,
    ic2: f32,
}

/// The three simultaneous outputs of [`Svf::process`].
#[derive(Clone, Copy, Debug)]
pub struct SvfOut {
    pub lp: f32,
    pub bp: f32,
    pub hp: f32,
}

impl Svf {
    /// `q` around 0.7 is flat; higher rings.
    pub fn new(cutoff: f32, q: f32, sr: f32) -> Self {
        let mut f = Self {
            g: 0.0,
            k: 1.0,
            ic1: 0.0,
            ic2: 0.0,
        };
        f.set(cutoff, q, sr);
        f
    }

    pub fn set(&mut self, cutoff: f32, q: f32, sr: f32) {
        self.g = (PI * cutoff.clamp(5.0, sr * 0.45) / sr).tan();
        self.k = 1.0 / q.max(0.05);
    }

    #[inline]
    pub fn process(&mut self, x: f32) -> SvfOut {
        let g = self.g;
        let k = self.k;
        let a1 = 1.0 / (1.0 + g * (g + k));
        let a2 = g * a1;
        let a3 = g * a2;
        let v3 = x - self.ic2;
        let v1 = a1 * self.ic1 + a2 * v3;
        let v2 = self.ic2 + a2 * self.ic1 + a3 * v3;
        self.ic1 = 2.0 * v1 - self.ic1;
        self.ic2 = 2.0 * v2 - self.ic2;
        SvfOut {
            lp: v2,
            bp: v1,
            hp: x - k * v1 - v2,
        }
    }

    #[inline]
    pub fn lp(&mut self, x: f32) -> f32 {
        self.process(x).lp
    }

    #[inline]
    pub fn bp(&mut self, x: f32) -> f32 {
        self.process(x).bp
    }

    #[inline]
    pub fn hp(&mut self, x: f32) -> f32 {
        self.process(x).hp
    }
}

/// A single struck mode: a two-pole resonator that rings at `freq` and
/// decays by 60 dB in `t60` seconds. Feed it an impulse or a noise burst.
/// Banks of these are the core of every solid-body sound (modal synthesis).
#[derive(Clone, Debug)]
pub struct Resonator {
    b1: f32,
    b2: f32,
    y1: f32,
    y2: f32,
    gain: f32,
}

impl Resonator {
    pub fn new(freq: f32, t60: f32, sr: f32) -> Self {
        let mut r = Self {
            b1: 0.0,
            b2: 0.0,
            y1: 0.0,
            y2: 0.0,
            gain: 1.0,
        };
        r.set(freq, t60, sr);
        r
    }

    pub fn set(&mut self, freq: f32, t60: f32, sr: f32) {
        let f = freq.clamp(10.0, sr * 0.45);
        // Pole radius from the decay time: amplitude falls 60 dB in t60.
        let radius = (-6.907_755 / (t60.max(1e-3) * sr)).exp();
        let w = TAU * f / sr;
        self.b1 = 2.0 * radius * w.cos();
        self.b2 = -radius * radius;
        // Normalise so an impulse of 1 gives a peak of about 1: the impulse
        // response is r^n sin((n+1)w) / sin(w).
        self.gain = w.sin().max(1e-4);
    }

    #[inline]
    pub fn process(&mut self, x: f32) -> f32 {
        let y = x * self.gain + self.b1 * self.y1 + self.b2 * self.y2;
        self.y2 = self.y1;
        self.y1 = y;
        y
    }

    pub fn energy(&self) -> f32 {
        self.y1.abs() + self.y2.abs()
    }
}

/// A bank of modes with individual amplitudes.
#[derive(Clone, Debug, Default)]
pub struct ModalBank {
    modes: Vec<(Resonator, f32)>,
}

impl ModalBank {
    /// `modes` is (frequency, t60, amplitude).
    pub fn new(modes: &[(f32, f32, f32)], sr: f32) -> Self {
        Self {
            modes: modes
                .iter()
                .map(|&(f, t, a)| (Resonator::new(f, t, sr), a))
                .collect(),
        }
    }

    pub fn push(&mut self, freq: f32, t60: f32, amp: f32, sr: f32) {
        self.modes.push((Resonator::new(freq, t60, sr), amp));
    }

    #[inline]
    pub fn process(&mut self, x: f32) -> f32 {
        let mut y = 0.0;
        for (r, a) in &mut self.modes {
            y += r.process(x) * *a;
        }
        y
    }

    /// Sum of mode magnitudes, to tell when the bank has rung out.
    pub fn energy(&self) -> f32 {
        self.modes.iter().map(|(r, a)| r.energy() * a.abs()).sum()
    }

    pub fn len(&self) -> usize {
        self.modes.len()
    }

    pub fn is_empty(&self) -> bool {
        self.modes.is_empty()
    }
}

/// Exponential decay envelope: 1 at start, -60 dB after `t60`.
#[derive(Clone, Debug)]
pub struct Decay {
    level: f32,
    k: f32,
}

impl Decay {
    pub fn new(t60: f32, sr: f32) -> Self {
        Self {
            level: 1.0,
            k: (-6.907_755 / (t60.max(1e-4) * sr)).exp(),
        }
    }

    #[inline]
    pub fn next(&mut self) -> f32 {
        let l = self.level;
        self.level *= self.k;
        l
    }

    pub fn retrigger(&mut self, level: f32) {
        self.level = level;
    }

    pub fn level(&self) -> f32 {
        self.level
    }
}

/// Attack-decay envelope with a linear attack.
#[derive(Clone, Debug)]
pub struct AttackDecay {
    attack: f32,
    t: f32,
    dt: f32,
    decay: Decay,
}

impl AttackDecay {
    pub fn new(attack: f32, t60: f32, sr: f32) -> Self {
        Self {
            attack: attack.max(1e-4),
            t: 0.0,
            dt: 1.0 / sr,
            decay: Decay::new(t60, sr),
        }
    }

    #[inline]
    pub fn next(&mut self) -> f32 {
        if self.t < self.attack {
            self.t += self.dt;
            self.t / self.attack
        } else {
            self.decay.next()
        }
    }

    pub fn done(&self) -> bool {
        self.t >= self.attack && self.decay.level() < 1e-4
    }
}

/// Sine oscillator with a phase accumulator.
#[derive(Clone, Debug, Default)]
pub struct Osc {
    phase: f32,
}

impl Osc {
    pub fn new(phase: f32) -> Self {
        Self { phase }
    }

    #[inline]
    pub fn sine(&mut self, freq: f32, sr: f32) -> f32 {
        let y = (self.phase * TAU).sin();
        self.advance(freq, sr);
        y
    }

    /// Naive sawtooth in [-1, 1]; fine for low, filtered tones.
    #[inline]
    pub fn saw(&mut self, freq: f32, sr: f32) -> f32 {
        let y = self.phase * 2.0 - 1.0;
        self.advance(freq, sr);
        y
    }

    /// Phase in [0, 1) before advancing.
    #[inline]
    pub fn phase(&self) -> f32 {
        self.phase
    }

    #[inline]
    pub fn advance(&mut self, freq: f32, sr: f32) -> bool {
        self.phase += freq / sr;
        if self.phase >= 1.0 {
            self.phase -= self.phase.floor();
            true
        } else {
            false
        }
    }
}

/// Smooths a control value towards its target (one-pole, `time` to ~63%).
#[derive(Clone, Debug)]
pub struct Smooth {
    pub value: f32,
    pub target: f32,
    k: f32,
}

impl Smooth {
    pub fn new(value: f32, time: f32, sr: f32) -> Self {
        Self {
            value,
            target: value,
            k: 1.0 - (-1.0 / (time.max(1e-4) * sr)).exp(),
        }
    }

    #[inline]
    pub fn next(&mut self) -> f32 {
        self.value += self.k * (self.target - self.value);
        self.value
    }
}

/// Fixed-length delay line with linear-interpolated reads.
#[derive(Clone, Debug)]
pub struct Delay {
    buf: Vec<f32>,
    pos: usize,
}

impl Delay {
    pub fn new(max_samples: usize) -> Self {
        Self {
            buf: vec![0.0; max_samples.max(2)],
            pos: 0,
        }
    }

    #[inline]
    pub fn write(&mut self, x: f32) {
        self.buf[self.pos] = x;
        self.pos = (self.pos + 1) % self.buf.len();
    }

    /// Sample written `delay` samples ago (fractional allowed, >= 1).
    #[inline]
    pub fn read(&self, delay: f32) -> f32 {
        let n = self.buf.len();
        let d = delay.clamp(1.0, (n - 1) as f32);
        let i = d.floor() as usize;
        let frac = d - i as f32;
        let a = self.buf[(self.pos + n - i) % n];
        let b = self.buf[(self.pos + n - i - 1) % n];
        a + (b - a) * frac
    }
}

/// Soft saturation, unity gain for small signals.
#[inline]
pub fn soft_clip(x: f32) -> f32 {
    x / (1.0 + x.abs())
}

/// Decibels to linear gain.
#[inline]
pub fn db(x: f32) -> f32 {
    10f32.powf(x / 20.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn resonator_rings_at_its_frequency_and_decays() {
        let sr = 48_000.0;
        let mut r = Resonator::new(1000.0, 0.5, sr);
        let mut out = Vec::new();
        out.push(r.process(1.0));
        for _ in 0..48_000 {
            out.push(r.process(0.0));
        }
        let peak = out.iter().fold(0.0f32, |m, x| m.max(x.abs()));
        assert!(peak > 0.3 && peak < 1.5, "peak {peak}");
        // Zero crossings in the first 0.1 s: ~2 per cycle.
        let crossings = out[..4800].windows(2).filter(|w| w[0] * w[1] < 0.0).count();
        assert!((190..=210).contains(&crossings), "{crossings}");
        // 60 dB down after t60.
        let late = out[24_000..24_200]
            .iter()
            .fold(0.0f32, |m, x| m.max(x.abs()));
        assert!(late < peak * 2e-3, "late {late}");
    }

    #[test]
    fn noise_is_bounded_and_seeded() {
        let mut a = Pink::new(3);
        let mut b = Pink::new(3);
        for _ in 0..10_000 {
            let x = a.next();
            assert_eq!(x, b.next());
            assert!(x.abs() < 2.0);
        }
    }
}
