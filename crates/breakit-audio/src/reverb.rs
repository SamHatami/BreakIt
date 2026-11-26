//! The test hall's reverb: an 8-line feedback delay network.
//!
//! Sized for the lab: 60 x 30 m, 11-13.5 m high, profiled steel cladding
//! and a concrete floor. Hard, mostly parallel surfaces give a long, slightly
//! metallic tail (about 3 s in the mids, shorter in the highs) with early
//! reflections that arrive late because the walls are far away.

use crate::dsp::{Delay, OnePole};

const LINES: usize = 8;

/// Delay lengths in milliseconds: spread around the hall's mean free path
/// (4V/S, about 15 m or 44 ms) and mutually prime once in samples.
const DELAYS_MS: [f32; LINES] = [31.7, 37.1, 43.9, 51.3, 59.9, 67.3, 79.1, 89.7];

pub struct Hall {
    sr: f32,
    pre: Delay,
    lines: Vec<Delay>,
    lengths: [f32; LINES],
    gains: [f32; LINES],
    damp: Vec<OnePole>,
    input_hp: OnePole,
    /// Early reflection taps (ms, gain, side).
    early: [(f32, f32, f32); 6],
}

impl Hall {
    pub fn new(sr: f32) -> Self {
        Self::with_rt60(sr, 3.0)
    }

    pub fn with_rt60(sr: f32, rt60: f32) -> Self {
        let mut lengths = [0.0; LINES];
        let mut gains = [0.0; LINES];
        for i in 0..LINES {
            lengths[i] = (DELAYS_MS[i] * 0.001 * sr).round();
            // Per-pass gain so the loop loses 60 dB in rt60.
            gains[i] = 10f32.powf(-3.0 * lengths[i] / (rt60 * sr));
        }
        Self {
            sr,
            pre: Delay::new((0.12 * sr) as usize),
            lines: lengths.iter().map(|&l| Delay::new(l as usize + 4)).collect(),
            lengths,
            gains,
            // Highs die faster: cladding and air absorb them.
            damp: (0..LINES).map(|_| OnePole::new(5_500.0, sr)).collect(),
            input_hp: OnePole::new(90.0, sr),
            early: [
                (19.0, 0.55, -0.7),
                (27.0, 0.45, 0.8),
                (41.0, 0.40, -0.3),
                (53.0, 0.32, 0.5),
                (71.0, 0.28, -0.9),
                (97.0, 0.22, 0.2),
            ],
        }
    }

    /// Mono send in, stereo reverb out (overwrites `out`).
    pub fn process(&mut self, input: &[f32], out: &mut [[f32; 2]]) {
        let ms = self.sr * 0.001;
        for (x, o) in input.iter().zip(out.iter_mut()) {
            // Low rumble would pile up in the loop as boom.
            let x = self.input_hp.hp(*x);
            self.pre.write(x);
            let src = self.pre.read(14.0 * ms);
            let mut el = 0.0;
            let mut er = 0.0;
            for &(t, g, side) in &self.early {
                let s = self.pre.read(t * ms) * g;
                el += s * (1.0 - side) * 0.5;
                er += s * (1.0 + side) * 0.5;
            }

            let mut v = [0.0f32; LINES];
            for i in 0..LINES {
                v[i] = self.damp[i].lp(self.lines[i].read(self.lengths[i])) * self.gains[i];
            }
            let (l, r) = (
                v[0] - v[2] + v[4] - v[6] + 0.5 * (v[1] - v[5]),
                v[1] - v[3] + v[5] - v[7] + 0.5 * (v[2] - v[6]),
            );
            hadamard(&mut v);
            for i in 0..LINES {
                // Inject the input with alternating signs for diffusion.
                let sign = if i % 2 == 0 { 1.0 } else { -1.0 };
                self.lines[i].write(v[i] + src * sign * 0.35);
            }
            *o = [l * 0.35 + el * 0.4, r * 0.35 + er * 0.4];
        }
    }
}

/// In-place normalised 8-point fast Walsh-Hadamard transform: a lossless,
/// maximally mixing feedback matrix.
fn hadamard(v: &mut [f32; LINES]) {
    let mut h = 1;
    while h < LINES {
        for i in (0..LINES).step_by(h * 2) {
            for j in i..i + h {
                let (a, b) = (v[j], v[j + h]);
                v[j] = a + b;
                v[j + h] = a - b;
            }
        }
        h *= 2;
    }
    let s = 1.0 / (LINES as f32).sqrt();
    for x in v.iter_mut() {
        *x *= s;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn impulse_decays_in_about_rt60() {
        let sr = 48_000.0;
        let mut hall = Hall::with_rt60(sr, 2.0);
        let n = (sr * 3.0) as usize;
        let mut input = vec![0.0; n];
        input[0] = 1.0;
        let mut out = vec![[0.0; 2]; n];
        hall.process(&input, &mut out);
        let rms = |a: usize, b: usize| {
            (out[a..b].iter().map(|f| f[0] * f[0] + f[1] * f[1]).sum::<f32>() / (b - a) as f32).sqrt()
        };
        let early = rms((0.1 * sr) as usize, (0.3 * sr) as usize);
        let late = rms((2.1 * sr) as usize, (2.3 * sr) as usize);
        assert!(early > 0.0 && late.is_finite());
        let drop_db = 20.0 * (early / late.max(1e-12)).log10();
        // About 60 dB over 2 s (the high-frequency damping makes it a bit
        // faster).
        assert!((45.0..90.0).contains(&drop_db), "dropped {drop_db} dB");
    }
}
