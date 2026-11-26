//! Modal synthesis engine: a bank of damped modes driven by a force signal,
//! and a voice that plays a score of force events into a few banks.
//!
//! A [`Bank`] is a linear system from force to radiated sound. Each mode is
//! a two-pole resonator with zeros at DC and Nyquist,
//! `H(z) = g (1 − z⁻²) / (1 − 2r cos θ z⁻¹ + r² z⁻²)`, so a unit impulse
//! rings every mode at its gain and a force *step* rings mode n at
//! `gain / (ω_n T)`. That 1/ω is physics, not a patch: releasing a static
//! load leaves each mode with its static deflection, which falls with ω².
//! One bank therefore serves impacts (short force pulses) and fractures
//! (the sudden loss of the force a member was carrying).
//!
//! Modes are stored eight to a lane so the inner loop vectorises.

use crate::body::ModeSpec;
use crate::dsp::OnePole;
use crate::Voice;
use std::f32::consts::TAU;

const LANE: usize = 8;

#[derive(Clone, Debug, Default)]
struct Lane {
    a1: [f32; LANE],
    a2: [f32; LANE],
    g: [f32; LANE],
    y1: [f32; LANE],
    y2: [f32; LANE],
}

/// A bank of modes.
#[derive(Clone, Debug)]
pub struct Bank {
    lanes: Vec<Lane>,
    modes: usize,
    x1: f32,
    x2: f32,
    /// Scale that makes a unit step ring the bank at unit energy.
    step_scale: f32,
}

impl Bank {
    pub fn new(modes: &[ModeSpec], sr: f32) -> Self {
        let mut lanes = vec![Lane::default(); modes.len().div_ceil(LANE)];
        let mut step = 0.0f32;
        for (i, m) in modes.iter().enumerate() {
            let lane = &mut lanes[i / LANE];
            let k = i % LANE;
            let theta = TAU * m.freq.clamp(10.0, sr * 0.45) / sr;
            let r = (-m.decay.max(0.1) / sr).exp();
            lane.a1[k] = 2.0 * r * theta.cos();
            lane.a2[k] = -r * r;
            lane.g[k] = 0.5 * m.gain;
            step += (lane.g[k] / (0.5 * theta).sin()).powi(2);
        }
        Self {
            lanes,
            modes: modes.len(),
            x1: 0.0,
            x2: 0.0,
            step_scale: if step > 0.0 { step.sqrt().recip() } else { 0.0 },
        }
    }

    pub fn len(&self) -> usize {
        self.modes
    }

    pub fn is_empty(&self) -> bool {
        self.modes == 0
    }

    /// Multiply a step's height by this so it rings the bank as loudly as a
    /// unit impulse does.
    pub fn step_scale(&self) -> f32 {
        self.step_scale
    }

    /// Adds the bank's response to `input` into `out`.
    pub fn process(&mut self, input: &[f32], out: &mut [f32]) {
        let (mut x1, mut x2) = (self.x1, self.x2);
        for (&x, o) in input.iter().zip(out.iter_mut()) {
            // The tiny offset keeps ringing-out states out of subnormals.
            let d = x - x2 + 1e-18;
            x2 = x1;
            x1 = x;
            let mut acc = [0.0f32; LANE];
            for lane in &mut self.lanes {
                for k in 0..LANE {
                    let y = lane.a1[k] * lane.y1[k] + lane.a2[k] * lane.y2[k] + lane.g[k] * d;
                    lane.y2[k] = lane.y1[k];
                    lane.y1[k] = y;
                    acc[k] += y;
                }
            }
            *o += acc.iter().sum::<f32>();
        }
        self.x1 = x1;
        self.x2 = x2;
    }

    /// Largest mode amplitude right now.
    pub fn level(&self) -> f32 {
        let mut m = 0.0f32;
        for lane in &self.lanes {
            for k in 0..LANE {
                m = m.max(lane.y1[k].abs()).max(lane.y2[k].abs());
            }
        }
        m
    }

    /// Stops all ringing.
    pub fn silence(&mut self) {
        for lane in &mut self.lanes {
            lane.y1 = [0.0; LANE];
            lane.y2 = [0.0; LANE];
        }
        self.x1 = 0.0;
        self.x2 = 0.0;
    }
}

/// One force event in a score.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Hit {
    /// Start, samples from the start of the voice.
    pub at: u32,
    /// Duration in samples: the contact time of an impact, or the time a
    /// crack takes to cross the section.
    pub len: u16,
    /// Which bank it drives.
    pub to: u8,
    /// A force step (a load released) rather than a pulse (an impact).
    pub step: bool,
    /// Impulse, in units where 1 rings the bank at unit energy.
    pub amp: f32,
    /// Level of the event's own sound: the pressure pulse of the crack or
    /// the contact itself, before any body has rung.
    pub direct: f32,
}

#[derive(Clone, Copy, Debug)]
struct Live {
    hit: Hit,
    /// Samples to wait before it starts.
    delay: u32,
    pos: u32,
}

/// Raised-cosine force pulses and their direct sound, shared by one-shots
/// and the continuous strain voices.
#[derive(Clone, Debug, Default)]
pub struct Pulses {
    live: Vec<Live>,
}

impl Pulses {
    /// Starts an event `delay` samples into the next rendered block.
    pub fn start(&mut self, hit: Hit, delay: u32) {
        self.live.push(Live { hit, delay, pos: 0 });
    }

    pub fn is_empty(&self) -> bool {
        self.live.is_empty()
    }

    pub fn len(&self) -> usize {
        self.live.len()
    }

    /// Writes the next `n` samples of every live event: its force into
    /// `imp[to]` (pulses) or `step[to]` (steps, as the pulse the caller
    /// integrates), and its own sound into `direct`.
    pub fn render(
        &mut self,
        n: usize,
        imp: &mut [Vec<f32>],
        step: &mut [Vec<f32>],
        direct: &mut [f32],
    ) {
        for l in &mut self.live {
            let begin = (l.delay as usize).min(n);
            l.delay -= begin as u32;
            let len = l.hit.len.max(1) as u32;
            let dlen = len.max(2);
            let to = l.hit.to as usize;
            let buf = if l.hit.step {
                &mut step[to]
            } else {
                &mut imp[to]
            };
            let w = TAU / len as f32;
            let wd = TAU / dlen as f32;
            let a = l.hit.amp / len as f32;
            for j in begin..n {
                if l.pos >= dlen {
                    break;
                }
                let p = l.pos as f32 + 0.5;
                if l.pos < len {
                    buf[j] += a * (1.0 - (w * p).cos());
                }
                direct[j] += l.hit.direct * (wd * p).sin();
                l.pos += 1;
            }
        }
        self.live.retain(|l| l.pos < (l.hit.len.max(2)) as u32);
    }
}

/// A one-shot: a score of force events played into up to four banks.
pub struct ModalVoice {
    banks: Vec<Bank>,
    /// Per bank: pulse force, step pulses, and the leaky integrator that
    /// turns step pulses into the step itself.
    imp: Vec<Vec<f32>>,
    step: Vec<Vec<f32>>,
    held: Vec<f32>,
    hits: Vec<Hit>,
    next: usize,
    pulses: Pulses,
    clock: u32,
    gain: f32,
    direct: Vec<f32>,
    direct_lp: OnePole,
    direct_gain: f32,
    leak: f32,
    max_samples: u32,
}

/// Integrator leak for held steps: forgets a released load in about 13 ms,
/// far slower than any audible mode's period.
pub fn step_leak(sr: f32) -> f32 {
    1.0 - TAU * 12.0 / sr
}

impl ModalVoice {
    /// `hits` need not be sorted. `direct_cutoff` (Hz) colours the events'
    /// own sound: dull for concrete, bright for steel and carbon.
    pub fn new(
        banks: Vec<Bank>,
        mut hits: Vec<Hit>,
        gain: f32,
        direct_gain: f32,
        direct_cutoff: f32,
        sr: f32,
    ) -> Self {
        hits.sort_by_key(|h| h.at);
        let n = banks.len();
        Self {
            banks,
            imp: vec![Vec::new(); n],
            step: vec![Vec::new(); n],
            held: vec![0.0; n],
            hits,
            next: 0,
            pulses: Pulses::default(),
            clock: 0,
            gain,
            direct: Vec::new(),
            direct_lp: OnePole::new(direct_cutoff, sr),
            direct_gain,
            leak: step_leak(sr),
            max_samples: (10.0 * sr) as u32,
        }
    }

    pub fn mode_count(&self) -> usize {
        self.banks.iter().map(Bank::len).sum()
    }
}

impl Voice for ModalVoice {
    fn render(&mut self, out: &mut [f32]) -> bool {
        let n = out.len();
        for b in self.imp.iter_mut().chain(self.step.iter_mut()) {
            b.clear();
            b.resize(n, 0.0);
        }
        self.direct.clear();
        self.direct.resize(n, 0.0);

        // Start the events that fall in this block.
        let end = self.clock + n as u32;
        while self.next < self.hits.len() && self.hits[self.next].at < end {
            let h = self.hits[self.next];
            self.next += 1;
            if (h.to as usize) < self.banks.len() {
                self.pulses.start(h, h.at.saturating_sub(self.clock));
            }
        }
        self.pulses
            .render(n, &mut self.imp, &mut self.step, &mut self.direct);

        // The events' own sound first, then the banks ring on top of it.
        let dg = self.direct_gain;
        for d in &mut self.direct {
            *d = dg * self.direct_lp.lp(*d + 1e-18);
        }
        let mut ringing = false;
        for (i, bank) in self.banks.iter_mut().enumerate() {
            let scale = bank.step_scale();
            let mut held = self.held[i];
            let mut active = held.abs() > 1e-6;
            for (x, &st) in self.imp[i].iter_mut().zip(&self.step[i]) {
                held = held * self.leak + st * scale;
                *x += held;
                active |= *x != 0.0;
            }
            self.held[i] = held;
            if !active && bank.level() < 2e-5 {
                continue;
            }
            bank.process(&self.imp[i], &mut self.direct);
            ringing |= bank.level() > 2e-5;
        }
        for (o, &d) in out.iter_mut().zip(&self.direct) {
            *o += d * self.gain;
        }
        self.clock = end;
        (self.next < self.hits.len() || !self.pulses.is_empty() || ringing)
            && self.clock < self.max_samples
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SR: f32 = 48_000.0;

    fn one(freq: f32, decay: f32) -> Bank {
        Bank::new(
            &[ModeSpec {
                freq,
                decay,
                gain: 1.0,
            }],
            SR,
        )
    }

    #[test]
    fn impulse_rings_a_mode_at_its_gain_and_frequency() {
        let mut b = one(1000.0, 1.0);
        let mut x = vec![0.0; 4800];
        x[0] = 1.0;
        let mut y = vec![0.0; 4800];
        b.process(&x, &mut y);
        let peak = y.iter().fold(0.0f32, |m, v| m.max(v.abs()));
        assert!((peak - 1.0).abs() < 0.03, "{peak}");
        // 100 cycles in 0.1 s: 200 zero crossings.
        let crossings = y.windows(2).filter(|w| w[0] * w[1] < 0.0).count();
        assert!((crossings as i32 - 200).abs() <= 2, "{crossings}");
    }

    #[test]
    fn decay_rate_is_met() {
        let mut b = one(500.0, 20.0);
        let mut x = vec![0.0; 9600];
        x[0] = 1.0;
        let mut y = vec![0.0; 9600];
        b.process(&x, &mut y);
        let early = y[..960].iter().fold(0.0f32, |m, v| m.max(v.abs()));
        let late = y[4800..5760].iter().fold(0.0f32, |m, v| m.max(v.abs()));
        // exp(-20 * 0.1) = 0.135
        assert!((late / early - 0.135).abs() < 0.02, "{}", late / early);
    }

    #[test]
    fn a_step_favours_low_modes_by_one_over_omega() {
        let amp = |f: f32| {
            let mut b = one(f, 0.5);
            let x = vec![1.0; 9600];
            let mut y = vec![0.0; 9600];
            b.process(&x, &mut y);
            y[2400..].iter().fold(0.0f32, |m, v| m.max(v.abs()))
        };
        let ratio = amp(200.0) / amp(800.0);
        assert!((ratio - 4.0).abs() < 0.3, "{ratio}");
    }

    #[test]
    fn voice_plays_its_score_and_ends() {
        let bank = Bank::new(
            &[
                ModeSpec {
                    freq: 400.0,
                    decay: 30.0,
                    gain: 0.7,
                },
                ModeSpec {
                    freq: 1900.0,
                    decay: 60.0,
                    gain: 0.7,
                },
            ],
            SR,
        );
        let hits = vec![
            Hit {
                at: 4800,
                len: 24,
                to: 0,
                step: true,
                amp: 1.0,
                direct: 0.5,
            },
            Hit {
                at: 0,
                len: 8,
                to: 0,
                step: false,
                amp: 1.0,
                direct: 0.5,
            },
        ];
        let mut v = ModalVoice::new(vec![bank], hits, 1.0, 1.0, 8000.0, SR);
        let mut total = Vec::new();
        let mut alive = true;
        let mut blocks = 0;
        while alive && blocks < 4000 {
            let mut buf = [0.0f32; 128];
            alive = v.render(&mut buf);
            total.extend_from_slice(&buf);
            blocks += 1;
        }
        assert!(!alive, "voice never ended");
        assert!(total.iter().all(|x| x.is_finite()));
        let peak = total.iter().fold(0.0f32, |m, v| m.max(v.abs()));
        assert!(peak > 0.3 && peak < 4.0, "{peak}");
        // Silent between the first ring-out and the step at 0.1 s.
        let gap = total[4000..4700].iter().fold(0.0f32, |m, v| m.max(v.abs()));
        assert!(gap < 0.05 * peak, "{gap}");
        let second = total[4800..6000].iter().fold(0.0f32, |m, v| m.max(v.abs()));
        assert!(second > 0.2, "{second}");
    }
}
