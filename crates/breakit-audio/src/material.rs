//! Structure sounds: stress beds, damage onsets, breaks and debris impacts.
//!
//! Everything here is modal synthesis driven by stochastic excitation:
//!
//! * a **body** is what rings: a bank of resonant modes (a steel tube's
//!   bending modes, a timber member's broad wooden resonances, a concrete
//!   member's heavily damped modes, a carbon laminate's stiff high-Q ones),
//!   or a plain band filter for grit and fibres;
//! * an **exciter** is what hits it: a single click (a crack front), a noise
//!   burst (crushing, grinding), a stick-slip pulse train (creaks and groans:
//!   friction that sticks, builds up and slips, again and again) or a
//!   crackle (a Poisson shower of clicks whose rate and size decay, like
//!   fibres or grains giving way one after another).
//!
//! A sound is a few [`Layer`]s (exciter into body, with a start delay and a
//! gain). Members under load get a [`StressVoice`] per material that spawns
//! short layers at a rate that climbs steeply with load, so a structure is
//! silent when relaxed, ticks and creaks as it is loaded and groans near
//! capacity. Every random choice comes from the voice's seed, so repeated
//! events vary but a given seed always sounds the same.

use std::cell::RefCell;

use crate::dsp::{AttackDecay, Brown, ModalBank, OnePole, Rng, Svf};
use crate::{BoxVoice, BreakCause, DamageState, MaterialKind, Voice};

/// Reference member masses (kg) the patches are tuned at.
const REF_STEEL: f32 = 40.0;
const REF_TIMBER: f32 = 30.0;
const REF_CONCRETE: f32 = 300.0;
const REF_COMPOSITE: f32 = 10.0;

fn reference(m: MaterialKind) -> f32 {
    match m {
        MaterialKind::Steel => REF_STEEL,
        MaterialKind::Timber => REF_TIMBER,
        MaterialKind::Concrete => REF_CONCRETE,
        MaterialKind::Composite => REF_COMPOSITE,
    }
}

/// How a member's mass (relative to its material's reference) scales a
/// sound: heavier is lower, longer and louder, all on a gentle log scale.
#[derive(Clone, Copy, Debug)]
struct Scale {
    /// Frequency multiplier.
    pitch: f32,
    /// Duration multiplier.
    len: f32,
    /// Gain multiplier.
    loud: f32,
}

impl Scale {
    fn of(material: MaterialKind, size: f32) -> Self {
        let r = (size.max(0.01) / reference(material)).clamp(0.01, 100.0);
        Self {
            pitch: r.powf(-0.2).clamp(0.5, 2.0),
            len: r.powf(0.2).clamp(0.6, 2.0),
            loud: r.powf(0.12).clamp(0.6, 1.4),
        }
    }
}

// ---------------------------------------------------------------------------
// Exciters and bodies

enum Excite {
    /// A single crack: a spike and a short noise tail.
    Click { spike: f32, level: f32, k: f32 },
    /// Enveloped noise, optionally low-passed.
    Burst {
        env: AttackDecay,
        amp: f32,
        lp: Option<OnePole>,
    },
    /// Stick-slip: slips at a gliding rate `f0 -> f1` over `dur` seconds.
    Pulses {
        phase: f32,
        f0: f32,
        f1: f32,
        dur: f32,
        t: f32,
        amp: f32,
        jitter: f32,
        tail: f32,
        k: f32,
    },
    /// Poisson shower of clicks; rate and size decay with time constants.
    Crackle {
        rate: f32,
        rate_tau: f32,
        dur: f32,
        t: f32,
        amp: f32,
        amp_tau: f32,
        /// Exponent on the random click size: higher = more small clicks
        /// between rare big ones.
        var: f32,
        tail: f32,
        k: f32,
    },
}

impl Excite {
    #[inline]
    fn next(&mut self, rng: &mut Rng, dt: f32) -> f32 {
        match self {
            Excite::Click { spike, level, k } => {
                let x = *spike + *level * 0.15 * rng.bipolar();
                *spike = 0.0;
                *level *= *k;
                x
            }
            Excite::Burst { env, amp, lp } => {
                let n = rng.bipolar();
                let n = match lp {
                    Some(f) => f.lp(n) * 2.0,
                    None => n,
                };
                n * env.next() * *amp
            }
            Excite::Pulses {
                phase,
                f0,
                f1,
                dur,
                t,
                amp,
                jitter,
                tail,
                k,
            } => {
                let mut x = *tail * 0.15 * rng.bipolar();
                // Flush to zero between events (long gaps would otherwise
                // decay into slow subnormal floats).
                *tail = if *tail > 1e-6 { *tail * *k } else { 0.0 };
                if *t < *dur {
                    let u = *t / *dur;
                    let freq = *f0 + (*f1 - *f0) * u;
                    // Fast swell, slower fade.
                    let env = (u / 0.12).min(1.0) * ((1.0 - u) / 0.35).min(1.0);
                    *phase += freq * dt * (1.0 + *jitter * rng.bipolar());
                    if *phase >= 1.0 {
                        *phase -= 1.0;
                        let a = *amp * env * rng.range(0.55, 1.0);
                        x += a;
                        *tail = a;
                    }
                    *t += dt;
                }
                x
            }
            Excite::Crackle {
                rate,
                rate_tau,
                dur,
                t,
                amp,
                amp_tau,
                var,
                tail,
                k,
            } => {
                let mut x = *tail * 0.15 * rng.bipolar();
                // Flush to zero between events (long gaps would otherwise
                // decay into slow subnormal floats).
                *tail = if *tail > 1e-6 { *tail * *k } else { 0.0 };
                if *t < *dur {
                    let r = *rate * (-*t / *rate_tau).exp();
                    if rng.chance(r * dt) {
                        let a = *amp * (-*t / *amp_tau).exp() * rng.unit().powf(*var);
                        let a = if rng.chance(0.5) { a } else { -a };
                        x += a;
                        *tail = a.abs();
                    }
                    *t += dt;
                }
                x
            }
        }
    }

    fn done(&self) -> bool {
        match self {
            Excite::Click { level, .. } => *level < 1e-4,
            Excite::Burst { env, .. } => env.done(),
            Excite::Pulses { t, dur, tail, .. } => *t >= *dur && *tail < 1e-4,
            Excite::Crackle { t, dur, tail, .. } => *t >= *dur && *tail < 1e-4,
        }
    }
}

enum Body {
    Modes(ModalBank),
    Band(Svf),
    Low(Svf),
    High(Svf),
}

impl Body {
    #[inline]
    fn process(&mut self, x: f32) -> f32 {
        match self {
            Body::Modes(b) => b.process(x),
            Body::Band(f) => f.bp(x),
            Body::Low(f) => f.lp(x),
            Body::High(f) => f.hp(x),
        }
    }
}

/// One exciter into one body.
pub struct Layer {
    delay: u32,
    ex: Excite,
    body: Body,
    gain: f32,
    tail: u32,
    max_tail: u32,
}

impl Layer {
    /// Adds this layer into `out`; false once it has rung out.
    fn render(&mut self, rng: &mut Rng, dt: f32, out: &mut [f32]) -> bool {
        let mut start = 0;
        if self.delay > 0 {
            let skip = (self.delay as usize).min(out.len());
            self.delay -= skip as u32;
            start = skip;
            if self.delay > 0 {
                return true;
            }
        }
        // Once the exciter is spent only the body rings on. Not calling the
        // exciter also keeps its decaying state out of subnormal floats,
        // which are very slow to multiply.
        let excited = !self.ex.done();
        for o in &mut out[start..] {
            // The tiny offset keeps quickly damped modes out of subnormals
            // while slower ones in the same bank still ring.
            let x = if excited { self.ex.next(rng, dt) } else { 0.0 } + 1e-15;
            *o += self.body.process(x) * self.gain;
        }
        if excited {
            return true;
        }
        match &self.body {
            Body::Modes(b) => b.energy() > 1e-5,
            _ => {
                self.tail += (out.len() - start) as u32;
                self.tail < self.max_tail
            }
        }
    }
}

/// Builds layers for one sound.
struct Kit {
    sr: f32,
    rng: RefCell<Rng>,
    layers: Vec<Layer>,
}

impl Kit {
    fn new(sr: f32, seed: u32) -> Self {
        Self {
            sr,
            rng: RefCell::new(Rng::new(seed)),
            layers: Vec::with_capacity(8),
        }
    }

    fn range(&self, lo: f32, hi: f32) -> f32 {
        self.rng.borrow_mut().range(lo, hi)
    }

    fn unit(&self) -> f32 {
        self.rng.borrow_mut().unit()
    }

    fn vary(&self, x: f32, spread: f32) -> f32 {
        self.rng.borrow_mut().vary(x, spread)
    }

    fn chance(&self, p: f32) -> bool {
        self.rng.borrow_mut().chance(p)
    }

    fn add(&mut self, delay: f32, ex: Excite, body: Body, gain: f32) {
        self.layers.push(Layer {
            delay: (delay.max(0.0) * self.sr) as u32,
            ex,
            body,
            gain,
            tail: 0,
            max_tail: (0.06 * self.sr) as u32,
        });
    }

    // Exciters --------------------------------------------------------------

    /// `decay` seconds for the noise tail to fall 60 dB.
    fn click(&self, level: f32, decay: f32) -> Excite {
        Excite::Click {
            spike: level,
            level,
            k: (-6.9 / (decay.max(1e-4) * self.sr)).exp(),
        }
    }

    fn burst(&self, amp: f32, attack: f32, t60: f32, lp: Option<f32>) -> Excite {
        Excite::Burst {
            env: AttackDecay::new(attack, t60, self.sr),
            amp,
            lp: lp.map(|c| OnePole::new(c, self.sr)),
        }
    }

    fn pulses(&self, f0: f32, f1: f32, dur: f32, amp: f32, jitter: f32) -> Excite {
        Excite::Pulses {
            phase: self.unit(),
            f0,
            f1,
            dur,
            t: 0.0,
            amp,
            jitter,
            tail: 0.0,
            k: (-6.9 / (0.0015 * self.sr)).exp(),
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn crackle(
        &self,
        rate: f32,
        rate_tau: f32,
        dur: f32,
        amp: f32,
        amp_tau: f32,
        var: f32,
    ) -> Excite {
        Excite::Crackle {
            rate,
            rate_tau,
            dur,
            t: 0.0,
            amp,
            amp_tau,
            var,
            tail: 0.0,
            k: (-6.9 / (0.0012 * self.sr)).exp(),
        }
    }

    // Bodies ----------------------------------------------------------------

    fn band(&self, f: f32, q: f32) -> Body {
        Body::Band(Svf::new(f, q, self.sr))
    }

    fn low(&self, f: f32) -> Body {
        Body::Low(Svf::new(f, 0.7, self.sr))
    }

    fn high(&self, f: f32) -> Body {
        Body::High(Svf::new(f, 0.7, self.sr))
    }

    fn modes(&self, modes: Vec<(f32, f32, f32)>) -> Body {
        // Normalise so the amplitudes sum to 1: a unit click peaks near 1.
        let sum: f32 = modes.iter().map(|m| m.2.abs()).sum::<f32>().max(1e-6);
        let modes: Vec<_> = modes.into_iter().map(|(f, t, a)| (f, t, a / sum)).collect();
        Body::Modes(ModalBank::new(&modes, self.sr))
    }

    /// A steel member: free-free Euler-Bernoulli bending modes above `base`
    /// plus a few inharmonic shell (ovalling) modes. `tilt` darkens.
    fn steel(
        &self,
        base: f32,
        t60: f32,
        bend: usize,
        shell: usize,
        shell_band: (f32, f32),
        tilt: f32,
    ) -> Body {
        const R: [f32; 7] = [1.0, 2.756, 5.404, 8.933, 13.344, 18.64, 24.81];
        let mut m = Vec::with_capacity(bend + shell);
        for (i, r) in R.iter().take(bend).enumerate() {
            let f = base * r * self.vary(1.0, 0.03);
            let a = self.range(0.6, 1.0) / (1.0 + i as f32).powf(tilt);
            m.push((f, t60 * (base / f).powf(0.35), a));
        }
        for _ in 0..shell {
            let f = self.range(shell_band.0, shell_band.1);
            m.push((f, t60 * self.range(0.3, 0.7), self.range(0.15, 0.45)));
        }
        self.modes(m)
    }

    /// Wooden body: broad, quickly damped resonances.
    fn wood(&self, pitch: f32, t60: f32) -> Body {
        const F: [(f32, f32); 5] = [
            (190.0, 1.0),
            (410.0, 0.85),
            (760.0, 0.6),
            (1150.0, 0.45),
            (2300.0, 0.22),
        ];
        let m = F
            .iter()
            .map(|&(f, a)| {
                let f = f * pitch * self.vary(1.0, 0.12);
                (
                    f,
                    t60 * self.range(0.6, 1.2) * (400.0 / f).powf(0.3),
                    a * self.range(0.6, 1.0),
                )
            })
            .collect();
        self.modes(m)
    }

    /// Concrete: low, very heavily damped modes.
    fn concrete(&self, pitch: f32, t60: f32) -> Body {
        const F: [(f32, f32); 5] = [
            (85.0, 1.0),
            (230.0, 0.8),
            (510.0, 0.6),
            (1100.0, 0.45),
            (2400.0, 0.3),
        ];
        let m = F
            .iter()
            .map(|&(f, a)| {
                let f = f * pitch * self.vary(1.0, 0.15);
                (
                    f,
                    t60 * self.range(0.6, 1.2) * (200.0 / f).powf(0.4),
                    a * self.range(0.6, 1.0),
                )
            })
            .collect();
        self.modes(m)
    }

    /// Carbon laminate: stiff, light, ringing high modes.
    fn laminate(&self, lo: f32, hi: f32, t60: f32, count: usize) -> Body {
        let m = (0..count)
            .map(|_| {
                let f = lo * (hi / lo).powf(self.unit());
                (f, t60 * self.range(0.5, 1.2), self.range(0.4, 1.0))
            })
            .collect();
        self.modes(m)
    }

    fn finish(self, max_len: f32) -> BoxVoice {
        Box::new(Shot {
            layers: self.layers,
            rng: self.rng.into_inner(),
            dt: 1.0 / self.sr,
            left: (max_len * self.sr) as u32,
        })
    }
}

/// A one-shot made of layers.
struct Shot {
    layers: Vec<Layer>,
    rng: Rng,
    dt: f32,
    left: u32,
}

impl Voice for Shot {
    fn render(&mut self, out: &mut [f32]) -> bool {
        let (rng, dt) = (&mut self.rng, self.dt);
        self.layers.retain_mut(|l| l.render(rng, dt, out));
        if self.left <= out.len() as u32 {
            // Hard cap: fade the last block instead of cutting.
            let n = out.len() as f32;
            for (i, o) in out.iter_mut().enumerate() {
                *o *= 1.0 - i as f32 / n;
            }
            return false;
        }
        self.left -= out.len() as u32;
        !self.layers.is_empty()
    }
}

// ---------------------------------------------------------------------------
// Stress beds

/// Most grains a bed plays at once.
const MAX_GRAINS: usize = 8;

/// Continuous creaking / groaning of one material's loaded members.
pub struct StressVoice {
    material: MaterialKind,
    sr: f32,
    rng: Rng,
    seed: u32,
    load: f32,
    target: f32,
    rate: f32,
    members: f32,
    grains: Vec<Layer>,
    /// Concrete's low strain rumble.
    rumble: Option<(Brown, Svf, f32)>,
}

impl StressVoice {
    pub fn new(material: MaterialKind, sr: f32, seed: u32) -> Self {
        let rumble = (material == MaterialKind::Concrete)
            .then(|| (Brown::new(seed ^ 0xC0FE), Svf::new(65.0, 0.9, sr), 0.0));
        Self {
            material,
            sr,
            rng: Rng::new(seed),
            seed,
            load: 0.0,
            target: 0.0,
            rate: 0.0,
            members: 1.0,
            grains: Vec::with_capacity(MAX_GRAINS),
            rumble,
        }
    }

    /// `load` is utilisation over the onset (0 quiet, 1 at capacity), `rate`
    /// its rise per second, `members` how many members contribute.
    pub fn set(&mut self, load: f32, rate: f32, members: f32) {
        self.target = if load.is_finite() {
            load.clamp(0.0, 3.0)
        } else {
            0.0
        };
        self.rate = if rate.is_finite() {
            rate.clamp(0.0, 4.0)
        } else {
            0.0
        };
        self.members = if members.is_finite() {
            members.clamp(0.0, 400.0)
        } else {
            0.0
        };
    }

    pub fn reset(&mut self) {
        self.target = 0.0;
        self.load = 0.0;
        self.rate = 0.0;
        self.grains.clear();
        if let Some(r) = &mut self.rumble {
            r.2 = 0.0;
        }
    }

    /// Load past the onset, 0 at 0.3, 1 at capacity.
    fn intensity(&self) -> f32 {
        ((self.load - 0.3) / 0.7).max(0.0)
    }

    fn spawn(&mut self, delay: f32) {
        let f = self.intensity();
        self.seed = self.seed.wrapping_add(0x6D2B_79F5);
        let mut kit = Kit::new(self.sr, self.seed);
        let amp = (0.3 + 0.7 * f.min(1.2)) * kit.range(0.45, 1.0);
        match self.material {
            MaterialKind::Steel => {
                if f > 0.7 && kit.chance(0.25 * f.min(1.5)) {
                    // A tick or ping: a bolt or weld seat shifting.
                    let ex = kit.click(1.0, 0.002);
                    let body =
                        kit.steel(kit.range(500.0, 1100.0), 0.5, 4, 2, (2500.0, 5500.0), 0.6);
                    kit.add(delay, ex, body, 0.18 * amp);
                } else {
                    // Groan: slow stick-slip through the member's low modes.
                    let f0 = kit.range(8.0, 22.0);
                    let f1 = f0 * kit.range(0.7, 1.9);
                    let dur = kit.range(0.4, 1.4);
                    let ex = kit.pulses(f0, f1, dur, 1.0, 0.25);
                    let body = kit.steel(
                        kit.range(55.0, 110.0),
                        kit.range(0.6, 1.4),
                        5,
                        2,
                        (400.0, 900.0),
                        0.35,
                    );
                    kit.add(delay, ex, body, 0.35 * amp);
                }
            }
            MaterialKind::Timber => {
                if f > 0.8 && kit.chance(0.3) {
                    // Dry fibre crackle.
                    let ex = kit.crackle(90.0, 0.4, kit.range(0.1, 0.3), 1.0, 1.0, 2.5);
                    let body = kit.band(kit.range(1800.0, 3200.0), 1.2);
                    kit.add(delay, ex, body, 0.5 * amp);
                } else {
                    // Creak: a stick-slip that glides in pitch.
                    let (a, b) = (kit.range(15.0, 40.0), kit.range(40.0, 90.0));
                    let (f0, f1) = if kit.chance(0.6) { (a, b) } else { (b, a) };
                    let dur = kit.range(0.2, 0.7);
                    let ex = kit.pulses(f0, f1, dur, 1.0, 0.08);
                    let pitch = kit.range(0.8, 1.3);
                    let body = kit.wood(pitch, 0.07);
                    kit.add(delay, ex, body, 0.45 * amp);
                }
            }
            MaterialKind::Concrete => {
                if f > 0.8 && kit.chance(0.2) {
                    // Fine crumble.
                    let ex = kit.crackle(250.0, 0.15, kit.range(0.1, 0.3), 1.0, 0.2, 2.0);
                    let body = kit.band(kit.range(1200.0, 2500.0), 0.9);
                    kit.add(delay, ex, body, 0.4 * amp);
                } else {
                    // Micro-crack: a bright, very short tick.
                    let ex = kit.click(1.0, 0.0008);
                    let body = kit.band(kit.range(2800.0, 6500.0), 2.0);
                    kit.add(delay, ex, body, 0.9 * amp);
                }
            }
            MaterialKind::Composite => {
                if f > 0.85 && kit.chance(0.2) {
                    // A cluster of fibre breaks.
                    let ex = kit.crackle(200.0, 0.08, 0.15, 1.0, 0.1, 1.5);
                    let body = kit.laminate(2500.0, 8000.0, 0.15, 5);
                    kit.add(delay, ex, body, 0.25 * amp);
                } else {
                    let ex = kit.click(1.0, 0.0006);
                    let body = kit.laminate(2000.0, 8000.0, kit.range(0.15, 0.4), 5);
                    kit.add(delay, ex, body, 0.4 * amp);
                }
            }
        }
        self.grains.extend(kit.layers);
    }

    fn base_rate(&self) -> f32 {
        match self.material {
            MaterialKind::Steel => 0.9,
            MaterialKind::Timber => 2.0,
            MaterialKind::Concrete => 4.0,
            MaterialKind::Composite => 4.5,
        }
    }
}

impl Voice for StressVoice {
    fn render(&mut self, out: &mut [f32]) -> bool {
        let n = out.len();
        let dt = n as f32 / self.sr;
        self.load += (self.target - self.load) * (1.0 - (-dt / 0.15).exp());
        let f = self.intensity();
        let rumble_level = self.rumble.as_ref().map_or(0.0, |r| r.2);
        if f <= 0.0 && self.grains.is_empty() && rumble_level < 1e-5 {
            return true;
        }

        if f > 0.0 {
            let i = f.min(1.5);
            let rate =
                self.base_rate() * i * i * i * (1.0 + self.rate) * self.members.max(1.0).sqrt();
            let mut t = self.rng.exp_interval(rate);
            // Steel grains ring for a long time; fewer of them overlap.
            let cap = if self.material == MaterialKind::Steel {
                6
            } else {
                MAX_GRAINS
            };
            while t < dt && self.grains.len() < cap {
                self.spawn(t);
                t += self.rng.exp_interval(rate);
            }
        }

        let sdt = 1.0 / self.sr;
        let rng = &mut self.rng;
        self.grains.retain_mut(|g| g.render(rng, sdt, out));

        if let Some((noise, filter, level)) = &mut self.rumble {
            let target = 0.08
                * f.min(1.3)
                * (1.0 + 0.5 * self.rate)
                * (self.members.max(1.0).sqrt() / 2.45).min(2.0);
            let start = *level;
            let end = start + (target - start) * (1.0 - (-dt / 0.3).exp());
            *level = end;
            if start.max(end) > 1e-5 {
                for (k, o) in out.iter_mut().enumerate() {
                    let g = start + (end - start) * (k as f32 / n as f32);
                    *o += filter.lp(noise.next()) * g;
                }
            }
        }
        true
    }
}

// ---------------------------------------------------------------------------
// Damage onsets

/// The moment a member reaches a worse damage state. `None` for states that
/// have no onset sound (intact; broken has its own [`break_voice`]).
pub fn damage_voice(
    material: MaterialKind,
    state: DamageState,
    size: f32,
    sr: f32,
    seed: u32,
) -> Option<BoxVoice> {
    let s = Scale::of(material, size);
    let mut k = Kit::new(sr, seed);
    match state {
        DamageState::Intact | DamageState::Broken => return None,
        DamageState::Yielded => {
            // Deep groan, then the member settles with a low bong.
            let darker = if material == MaterialKind::Concrete {
                0.6
            } else {
                1.0
            };
            let ex = k.pulses(
                k.range(9.0, 14.0),
                k.range(18.0, 30.0),
                1.1 * s.len,
                1.0,
                0.2,
            );
            let body = k.steel(
                k.range(45.0, 70.0) * s.pitch * darker,
                1.5,
                5,
                3,
                (300.0, 800.0),
                0.4,
            );
            k.add(0.0, ex, body, 0.35 * s.loud);
            let ex = k.click(1.0, 0.004);
            let body = k.steel(
                k.range(70.0, 100.0) * s.pitch * darker,
                2.5 * s.len,
                5,
                2,
                (400.0, 1200.0),
                0.5,
            );
            k.add(1.0 * s.len, ex, body, 0.4 * s.loud);
            if material == MaterialKind::Concrete {
                crack(&mut k, material, s, 0.4, 0.0);
            }
        }
        DamageState::Cracked => {
            crack(&mut k, material, s, 0.45, 0.0);
            let (ex, body) = match material {
                MaterialKind::Timber => (
                    k.crackle(120.0, 0.12, 0.25, 1.0, 0.15, 2.0),
                    k.band(2200.0 * s.pitch, 1.0),
                ),
                MaterialKind::Composite => (
                    k.crackle(180.0, 0.1, 0.2, 1.0, 0.1, 2.0),
                    k.band(4500.0, 1.2),
                ),
                _ => (
                    k.crackle(200.0, 0.15, 0.3, 1.0, 0.15, 2.0),
                    k.band(1800.0 * s.pitch, 0.9),
                ),
            };
            k.add(0.01, ex, body, 0.35 * s.loud);
        }
        DamageState::Crushed => crunch(&mut k, material, s, 0.45),
        DamageState::Delaminated => rip(&mut k, s, 0.8, 0.35),
    }
    Some(k.finish(5.0))
}

// ---------------------------------------------------------------------------
// Shared gestures

/// A crack front: a spike into the material's body plus a bright snap.
fn crack(k: &mut Kit, material: MaterialKind, s: Scale, gain: f32, delay: f32) {
    let body = match material {
        MaterialKind::Steel => k.steel(300.0 * s.pitch, 1.2, 5, 3, (1500.0, 4000.0), 0.3),
        MaterialKind::Timber => {
            let p = k.range(0.7, 1.0) * s.pitch;
            k.wood(p, 0.12)
        }
        MaterialKind::Concrete => {
            let p = k.range(0.8, 1.2) * s.pitch;
            k.concrete(p, 0.06)
        }
        MaterialKind::Composite => k.laminate(900.0, 5000.0, 0.35, 6),
    };
    let ex = k.click(1.0, 0.002);
    k.add(delay, ex, body, gain);
    let ex = k.burst(1.0, 0.0002, 0.012, None);
    let body = k.high(2500.0);
    k.add(delay, ex, body, gain * 0.6);
}

/// Crushing: dense crunch with a low thump.
fn crunch(k: &mut Kit, material: MaterialKind, s: Scale, gain: f32) {
    let dur = 0.45 * s.len;
    let (rate, centre, q) = match material {
        MaterialKind::Timber => (450.0, 1100.0, 0.7),
        MaterialKind::Composite => (700.0, 2500.0, 0.8),
        _ => (650.0, 850.0, 0.6),
    };
    let ex = k.crackle(rate, dur * 0.8, dur, 1.0, dur * 0.7, 1.6);
    let body = k.band(centre * s.pitch, q);
    k.add(0.0, ex, body, gain * 0.9);
    let ex = k.burst(1.0, 0.01, dur, Some(500.0 * s.pitch));
    let body = k.low(350.0 * s.pitch);
    k.add(0.0, ex, body, gain * 0.25);
    // Thump.
    let ex = k.click(1.0, 0.003);
    let f = if material == MaterialKind::Concrete {
        55.0
    } else {
        80.0
    } * s.pitch;
    let body = k.modes(vec![(f, 0.18, 1.0), (f * 2.3, 0.08, 0.4)]);
    k.add(0.0, ex, body, gain * 1.2);
    // Grit.
    let ex = k.crackle(150.0, 0.3, 0.5 * s.len, 1.0, 0.25, 2.5);
    let body = k.band(3800.0, 1.0);
    k.add(0.05, ex, body, gain * 0.35);
}

/// Tearing along a seam: a dense crackle plus a fibrous rasp.
fn rip(k: &mut Kit, s: Scale, gain: f32, dur: f32) {
    let dur = dur * s.len;
    let ex = k.crackle(900.0, dur * 2.0, dur, 1.0, dur * 1.5, 1.8);
    let body = k.band(k.range(2600.0, 3600.0), 0.8);
    k.add(0.0, ex, body, gain * 0.7);
    let ex = k.burst(1.0, dur * 0.3, dur * 1.2, None);
    let body = k.high(3000.0);
    k.add(0.0, ex, body, gain * 0.12);
    let ex = k.click(1.0, 0.001);
    let body = k.laminate(1500.0, 6000.0, 0.25, 5);
    k.add(dur * 0.9, ex, body, gain * 0.5);
}

/// Steel snapping: transient, bright long ring and a twang.
fn snap(k: &mut Kit, s: Scale, gain: f32, delay: f32) {
    let ex = k.burst(1.0, 0.0001, 0.008, None);
    let body = k.high(1800.0);
    k.add(delay, ex, body, gain * 0.9);
    let ex = k.click(1.0, 0.002);
    let base = k.range(380.0, 520.0) * s.pitch;
    let body = k.steel(base, 2.2 * s.len, 6, 4, (2500.0, 6500.0), 0.15);
    k.add(delay, ex, body, gain * 0.9);
    // Twang: the freed end vibrating, nearly harmonic.
    let f = k.range(160.0, 240.0) * s.pitch;
    let body = k.modes(vec![
        (f, 1.1 * s.len, 1.0),
        (f * 2.01, 0.8 * s.len, 0.5),
        (f * 3.03, 0.5 * s.len, 0.3),
        (f * 4.07, 0.3 * s.len, 0.2),
    ]);
    let ex = k.click(1.0, 0.003);
    k.add(delay + 0.004, ex, body, gain * 0.6);
}

// ---------------------------------------------------------------------------
// Breaks

/// A member breaking. Picks the gesture from the cause and colours it by the
/// material; `size` is the member's mass in kg.
pub fn break_voice(
    material: MaterialKind,
    cause: BreakCause,
    size: f32,
    sr: f32,
    seed: u32,
) -> BoxVoice {
    let s = Scale::of(material, size);
    let mut k = Kit::new(sr, seed);
    let g = s.loud;
    use BreakCause as C;
    use MaterialKind as M;
    match (material, cause) {
        (M::Steel, C::Rupture) | (M::Steel, C::Brittle) => {
            snap(&mut k, s, 0.75 * g, 0.0);
            let ex = k.click(1.0, 0.003);
            let body = k.modes(vec![(60.0 * s.pitch, 0.15, 1.0)]);
            k.add(0.0, ex, body, 0.5 * g);
        }
        (M::Steel, _) => {
            // Hinge: the section grinds as it folds, then gives with a clank.
            let bend = 0.55 * s.len;
            let ex = k.pulses(k.range(14.0, 20.0), k.range(35.0, 55.0), bend, 1.0, 0.3);
            let body = k.steel(
                k.range(50.0, 80.0) * s.pitch,
                1.0,
                5,
                3,
                (300.0, 900.0),
                0.3,
            );
            k.add(0.0, ex, body, 0.4 * g);
            let ex = k.burst(1.0, bend * 0.6, bend, None);
            let body = k.band(k.range(700.0, 1100.0) * s.pitch, 1.5);
            k.add(0.0, ex, body, 0.06 * g);
            let ex = k.click(1.0, 0.004);
            let body = k.steel(
                k.range(150.0, 220.0) * s.pitch,
                1.6 * s.len,
                6,
                4,
                (900.0, 2500.0),
                0.3,
            );
            k.add(bend, ex, body, 0.75 * g);
            let ex = k.click(1.0, 0.003);
            let body = k.modes(vec![
                (55.0 * s.pitch, 0.2, 1.0),
                (130.0 * s.pitch, 0.1, 0.4),
            ]);
            k.add(bend, ex, body, 0.6 * g);
        }
        (M::Timber, C::TimberSplit) => {
            // Long tear along the grain, ending in a crack.
            let dur = k.range(0.8, 1.2) * s.len;
            let ex = k.crackle(320.0, dur * 3.0, dur, 1.0, dur * 3.0, 2.0);
            let body = k.band(1500.0 * s.pitch, 0.8);
            k.add(0.0, ex, body, 0.55 * g);
            let ex = k.pulses(30.0, 60.0, dur, 1.0, 0.3);
            let p = 0.8 * s.pitch;
            let body = k.wood(p, 0.06);
            k.add(0.0, ex, body, 0.4 * g);
            crack(&mut k, material, s, 0.7 * g, dur);
        }
        (M::Timber, C::Crushing) => crunch(&mut k, material, s, 0.65 * g),
        (M::Timber, _) => {
            // Splinter: a big crack and a cascade of fibres letting go.
            crack(&mut k, material, s, 0.65 * g, 0.0);
            let dur = k.range(0.3, 0.8) * s.len;
            let ex = k.crackle(180.0, dur * 0.6, dur, 1.0, dur * 0.8, 1.8);
            let body = k.band(k.range(1800.0, 2800.0), 0.9);
            k.add(0.02, ex, body, 0.6 * g);
            let ex = k.crackle(60.0, dur, dur, 1.0, dur, 1.5);
            let p = s.pitch * 1.2;
            let body = k.wood(p, 0.08);
            k.add(0.03, ex, body, 0.5 * g);
            let ex = k.click(1.0, 0.003);
            let body = k.modes(vec![(90.0 * s.pitch, 0.12, 1.0)]);
            k.add(0.0, ex, body, 0.5 * g);
        }
        (M::Concrete, C::Crushing) => {
            crunch(&mut k, material, s, 0.7 * g);
            grit(&mut k, s, 0.3 * g, 0.3);
        }
        (M::Concrete, C::RebarRupture) | (M::Concrete, C::Rupture) => {
            crack(&mut k, material, s, 0.65 * g, 0.0);
            crumble(&mut k, s, 0.4 * g);
            let bar = Scale {
                pitch: 1.4,
                len: 0.8,
                loud: 1.0,
            };
            snap(&mut k, bar, 0.5 * g, 0.06);
        }
        (M::Concrete, _) => {
            // Brittle or shear: loud crack, crumble and falling grit.
            crack(&mut k, material, s, 0.65 * g, 0.0);
            crumble(&mut k, s, 0.5 * g);
            grit(&mut k, s, 0.3 * g, 0.25);
            if cause == C::ConcreteShear {
                let ex = k.burst(1.0, 0.05, 0.5 * s.len, Some(1200.0));
                let body = k.band(600.0 * s.pitch, 0.8);
                k.add(0.0, ex, body, 0.3 * g);
            }
            let ex = k.click(1.0, 0.004);
            let body = k.modes(vec![
                (50.0 * s.pitch, 0.25, 1.0),
                (115.0 * s.pitch, 0.1, 0.5),
            ]);
            k.add(0.0, ex, body, 0.7 * g);
        }
        (M::Composite, C::Delamination) => {
            rip(&mut k, s, 0.7 * g, 0.5);
            crack(&mut k, material, s, 0.5 * g, 0.0);
        }
        (M::Composite, _) => {
            // Fibre rupture: an explosive crack and a fizz of fibres.
            crack(&mut k, material, s, 0.55 * g, 0.0);
            let ex = k.crackle(2500.0, 0.15, 0.45 * s.len, 1.0, 0.15, 1.3);
            let body = k.high(4500.0);
            k.add(0.005, ex, body, 0.45 * g);
            let ex = k.click(1.0, 0.002);
            let body = k.laminate(600.0, 2500.0, 0.5, 5);
            k.add(0.0, ex, body, 0.4 * g);
        }
    }
    k.finish(6.0)
}

fn crumble(k: &mut Kit, s: Scale, gain: f32) {
    let dur = 0.6 * s.len;
    let ex = k.crackle(400.0, dur * 0.5, dur, 1.0, dur * 0.6, 1.8);
    let body = k.band(1500.0 * s.pitch, 0.8);
    k.add(0.02, ex, body, gain);
}

fn grit(k: &mut Kit, s: Scale, gain: f32, delay: f32) {
    let dur = 1.0 * s.len;
    let ex = k.crackle(90.0, dur * 0.8, dur, 1.0, dur, 2.2);
    let body = k.band(k.range(3500.0, 5000.0), 1.2);
    k.add(delay, ex, body, gain);
}

// ---------------------------------------------------------------------------
// Impacts

/// A loose piece hitting something. `None` when too gentle to hear.
pub fn impact_voice(
    material: MaterialKind,
    speed: f32,
    size: f32,
    sr: f32,
    seed: u32,
) -> Option<BoxVoice> {
    if !(speed.is_finite() && size.is_finite()) || speed < 0.3 {
        return None;
    }
    // Debris is smaller than members: pitch from the piece's own size.
    let pitch = (reference(material) * 0.2 / size.max(0.01))
        .powf(0.25)
        .clamp(0.5, 3.0);
    let gain = ((speed / 6.0).powf(1.2) * (size / (reference(material) * 0.2)).powf(0.15))
        .clamp(0.0, 1.0)
        * 0.6;
    let mut k = Kit::new(sr, seed);
    match material {
        MaterialKind::Steel => {
            let ex = k.click(1.0, 0.0015);
            let base = k.range(260.0, 420.0) * pitch;
            let t60 = k.range(0.4, 0.9);
            let body = k.steel(base, t60, 5, 3, (1500.0, 5000.0), 0.25);
            k.add(0.0, ex, body, 1.4 * gain);
        }
        MaterialKind::Timber => {
            let ex = k.click(1.0, 0.002);
            let p = 1.3 * pitch * k.range(0.85, 1.15);
            let body = k.wood(p, 0.05);
            k.add(0.0, ex, body, 0.9 * gain);
        }
        MaterialKind::Concrete => {
            let ex = k.click(1.0, 0.004);
            let f = k.range(70.0, 110.0) * pitch;
            let body = k.modes(vec![(f, 0.1, 1.0), (f * 2.6, 0.05, 0.5)]);
            k.add(0.0, ex, body, 0.9 * gain);
            let ex = k.crackle(300.0, 0.08, 0.15, 1.0, 0.08, 2.0);
            let body = k.band(2200.0 * pitch.min(1.5), 0.9);
            k.add(0.0, ex, body, 0.5 * gain);
        }
        MaterialKind::Composite => {
            let ex = k.click(1.0, 0.001);
            let lo = 800.0 * pitch;
            let body = k.laminate(lo, lo * 4.0, 0.08, 5);
            k.add(0.0, ex, body, 0.8 * gain);
        }
    }
    Some(k.finish(2.0))
}

#[cfg(test)]
mod tests {
    use super::*;

    const SR: f32 = 48_000.0;

    /// Renders until the voice ends; (seconds, peak, rms).
    fn run(mut v: BoxVoice, max: f32) -> (f32, f32, f32) {
        let mut buf = [0.0f32; 128];
        let mut t = 0.0;
        let (mut peak, mut sum, mut n) = (0.0f32, 0.0f32, 0usize);
        loop {
            buf.iter_mut().for_each(|x| *x = 0.0);
            let alive = v.render(&mut buf);
            for &x in &buf {
                assert!(x.is_finite());
                peak = peak.max(x.abs());
                sum += x * x;
                n += 1;
            }
            t += 128.0 / SR;
            if !alive || t > max {
                break;
            }
        }
        (t, peak, (sum / n as f32).sqrt())
    }

    const CAUSES: [BreakCause; 10] = [
        BreakCause::Rupture,
        BreakCause::Hinge,
        BreakCause::Brittle,
        BreakCause::Splinter,
        BreakCause::TimberSplit,
        BreakCause::Crushing,
        BreakCause::ConcreteShear,
        BreakCause::RebarRupture,
        BreakCause::FibreRupture,
        BreakCause::Delamination,
    ];
    const MATS: [MaterialKind; 4] = [
        MaterialKind::Steel,
        MaterialKind::Timber,
        MaterialKind::Concrete,
        MaterialKind::Composite,
    ];

    #[test]
    #[ignore]
    fn print_levels() {
        for m in MATS {
            for c in CAUSES {
                let (t, p, r) = run(break_voice(m, c, reference(m), SR, 1), 10.0);
                println!("break {m:?} {c:?}: {t:.2}s peak {p:.2} rms {r:.3}");
            }
            for s in [
                DamageState::Cracked,
                DamageState::Yielded,
                DamageState::Crushed,
                DamageState::Delaminated,
            ] {
                let (t, p, r) = run(damage_voice(m, s, reference(m), SR, 1).unwrap(), 10.0);
                println!("damage {m:?} {s:?}: {t:.2}s peak {p:.2} rms {r:.3}");
            }
            for v in [1.0, 3.0, 8.0] {
                let (t, p, r) = run(impact_voice(m, v, reference(m) * 0.2, SR, 1).unwrap(), 10.0);
                println!("impact {m:?} {v}: {t:.2}s peak {p:.2} rms {r:.3}");
            }
        }
    }

    #[test]
    fn breaks_terminate_and_peak_in_range() {
        for m in MATS {
            for c in CAUSES {
                for seed in 0..3 {
                    let (t, peak, _) = run(break_voice(m, c, reference(m), SR, seed), 10.0);
                    assert!(t < 6.1, "{m:?} {c:?} ran {t} s");
                    assert!((0.2..1.2).contains(&peak), "{m:?} {c:?} peak {peak}");
                }
            }
        }
    }

    #[test]
    fn damage_and_impacts_terminate_and_stay_bounded() {
        let states = [
            DamageState::Cracked,
            DamageState::Yielded,
            DamageState::Crushed,
            DamageState::Delaminated,
        ];
        for m in MATS {
            for s in states {
                let (t, peak, _) = run(damage_voice(m, s, reference(m), SR, 5).unwrap(), 10.0);
                assert!(
                    t < 5.1 && peak < 1.0 && peak > 0.1,
                    "{m:?} {s:?}: {t} s, peak {peak}"
                );
            }
            assert!(damage_voice(m, DamageState::Intact, 10.0, SR, 1).is_none());
            assert!(impact_voice(m, 0.1, 10.0, SR, 1).is_none());
            let (t, peak, _) = run(
                impact_voice(m, 8.0, reference(m) * 0.2, SR, 2).unwrap(),
                5.0,
            );
            assert!(
                t < 2.1 && peak <= 0.7 && peak > 0.05,
                "{m:?} impact: {t} s, peak {peak}"
            );
        }
    }

    #[test]
    fn stress_is_silent_unloaded_and_audible_at_capacity() {
        let mut bad = Vec::new();
        for m in MATS {
            let mut v = StressVoice::new(m, SR, 9);
            let mut buf = vec![0.0f32; 128];
            for _ in 0..400 {
                v.render(&mut buf);
            }
            assert!(buf.iter().all(|x| *x == 0.0));
            v.set(1.0, 0.2, 6.0);
            let (mut sum, mut n) = (0.0f32, 0);
            for _ in 0..(10.0 * SR / 128.0) as usize {
                buf.iter_mut().for_each(|x| *x = 0.0);
                v.render(&mut buf);
                sum += buf.iter().map(|x| x * x).sum::<f32>();
                n += buf.len();
            }
            let rms = (sum / n as f32).sqrt();
            println!("{m:?} stress rms at capacity: {rms:.3}");
            if !(0.03..0.15).contains(&rms) {
                bad.push(format!("{m:?} rms {rms}"));
            }
            v.reset();
            buf.iter_mut().for_each(|x| *x = 0.0);
            v.render(&mut buf);
            assert!(buf.iter().all(|x| *x == 0.0));
        }
        assert!(bad.is_empty(), "{bad:?}");
    }
}

#[cfg(test)]
mod bench {
    use super::*;

    #[test]
    #[ignore]
    fn speed() {
        let sr = 48_000.0;
        let mut buf = vec![0.0f32; 128];
        let blocks = (10.0 * sr / 128.0) as usize;
        for m in [
            MaterialKind::Steel,
            MaterialKind::Timber,
            MaterialKind::Concrete,
            MaterialKind::Composite,
        ] {
            let mut v = StressVoice::new(m, sr, 1);
            v.set(1.2, 0.5, 8.0);
            let t = std::time::Instant::now();
            for _ in 0..blocks {
                v.render(&mut buf);
            }
            println!(
                "{m:?} stress at 1.2: {:.0}x real time",
                10.0 / t.elapsed().as_secs_f32()
            );
            let t = std::time::Instant::now();
            let mut b = break_voice(m, BreakCause::Hinge, 40.0, sr, 3);
            let mut n = 0;
            while b.render(&mut buf) {
                n += 1;
            }
            println!(
                "{m:?} break: {:.0}x real time",
                (n as f32 * 128.0 / sr) / t.elapsed().as_secs_f32()
            );
        }
        let mut bank = ModalBank::new(&[(100.0, 1.0, 0.1); 8], sr);
        let t = std::time::Instant::now();
        let mut acc = 0.0;
        for i in 0..480_000 {
            acc += bank.process(if i % 1000 == 0 { 1.0 } else { 0.0 });
        }
        println!(
            "8-mode bank: {:.0}x real time ({acc})",
            10.0 / t.elapsed().as_secs_f32()
        );
        let t = std::time::Instant::now();
        let mut mix = crate::Mixer::new(sr);
        let out = mix.render_seconds(10.0);
        println!(
            "empty mixer: {:.0}x ({})",
            10.0 / t.elapsed().as_secs_f32(),
            out.len()
        );
        let mut mix = crate::Mixer::new(sr);
        mix.handle(crate::Command::Ui(crate::UiSound::Select));
        mix.handle(crate::Command::Impact {
            material: MaterialKind::Steel,
            speed: 5.0,
            size: 5.0,
            place: crate::Place::at([0.0, 0.0, 0.0]),
        });
        mix.render_seconds(1.0);
        let t = std::time::Instant::now();
        mix.render_seconds(10.0);
        println!(
            "mixer after one impact, decaying: {:.0}x",
            10.0 / t.elapsed().as_secs_f32()
        );
    }
}
