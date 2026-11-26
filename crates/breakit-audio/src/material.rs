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

/// Output roll-off per material, Hz: metal and carbon stay bright,
/// concrete is dull.
fn air(m: MaterialKind) -> f32 {
    match m {
        MaterialKind::Steel => 12_000.0,
        MaterialKind::Timber => 9_000.0,
        MaterialKind::Concrete => 6_500.0,
        MaterialKind::Composite => 12_000.0,
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
        /// How much each slip is a noise grain rather than a click.
        noise: f32,
    },
    /// Poisson shower of short noise grains: rate rises over `rise`, then
    /// decays with `rate_tau`; each grain is a burst of white noise that
    /// decays in `k` per sample. Dense showers of these are what real
    /// fracture recordings are made of: thousands of tiny broadband events.
    Grains {
        rate: f32,
        rise: f32,
        rate_tau: f32,
        dur: f32,
        t: f32,
        amp: f32,
        amp_tau: f32,
        var: f32,
        level: f32,
        k: f32,
    },
    /// Poisson shower of clicks; rate and size decay with time constants.
    #[allow(dead_code)]
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
                noise,
            } => {
                let mut x = *tail * *noise * rng.bipolar();
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
                        let a = *amp * env * rng.range(0.4, 1.0);
                        x += a * (1.0 - *noise).max(0.2);
                        *tail = a;
                    }
                    *t += dt;
                }
                x
            }
            Excite::Grains {
                rate,
                rise,
                rate_tau,
                dur,
                t,
                amp,
                amp_tau,
                var,
                level,
                k,
            } => {
                let x = *level * rng.bipolar();
                *level = if *level > 1e-6 { *level * *k } else { 0.0 };
                if *t < *dur {
                    let shape = (*t / *rise).min(1.0) * (-*t / *rate_tau).exp();
                    if rng.chance(*rate * shape * dt) {
                        let a = *amp * (-*t / *amp_tau).exp() * rng.unit().powf(*var);
                        *level = level.max(a);
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
            Excite::Grains { t, dur, level, .. } => *t >= *dur && *level < 1e-4,
        }
    }
}

enum Body {
    /// No filtering: broadband.
    Pass,
    Modes(ModalBank),
    Band(Svf),
    Low(Svf),
    High(Svf),
}

impl Body {
    #[inline]
    fn process(&mut self, x: f32) -> f32 {
        match self {
            Body::Pass => x,
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
    /// Output low-pass cutoff, Hz.
    air: f32,
    rng: RefCell<Rng>,
    layers: Vec<Layer>,
}

impl Kit {
    fn new(sr: f32, seed: u32) -> Self {
        Self {
            sr,
            air: 9000.0,
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

    #[allow(dead_code)]
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
            noise: 0.15,
        }
    }

    /// Stick-slip where every slip is a short noise grain (`grain` s to
    /// -60 dB): grinding and groaning rather than a clean buzz.
    fn grind(&self, f0: f32, f1: f32, dur: f32, amp: f32, jitter: f32, grain: f32) -> Excite {
        Excite::Pulses {
            phase: self.unit(),
            f0,
            f1,
            dur,
            t: 0.0,
            amp,
            jitter,
            tail: 0.0,
            k: (-6.9 / (grain.max(1e-4) * self.sr)).exp(),
            noise: 0.8,
        }
    }

    /// A shower of noise grains. `rate` per second at its peak, reached
    /// after `rise` s, then falling with `tau`; grains last `grain` s.
    #[allow(clippy::too_many_arguments)]
    fn grains(
        &self,
        rate: f32,
        rise: f32,
        tau: f32,
        dur: f32,
        amp: f32,
        amp_tau: f32,
        var: f32,
        grain: f32,
    ) -> Excite {
        Excite::Grains {
            rate,
            rise: rise.max(1e-4),
            rate_tau: tau.max(1e-4),
            dur,
            t: 0.0,
            amp,
            amp_tau: amp_tau.max(1e-4),
            var,
            level: 0.0,
            k: (-6.9 / (grain.max(1e-4) * self.sr)).exp(),
        }
    }

    #[allow(clippy::too_many_arguments)]
    #[allow(dead_code)]
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

    fn pass(&self) -> Body {
        Body::Pass
    }

    /// A dense, slightly detuned metal body: `n` modes spread log-randomly
    /// over `lo..hi`, about a third of them in close pairs that beat. This
    /// is what makes a ring sound like a big piece of steel instead of a
    /// bell.
    fn steel_dense(&self, lo: f32, hi: f32, n: usize, t60: f32) -> Body {
        let mut m = Vec::with_capacity(n);
        while m.len() < n {
            let f = lo * (hi / lo).powf(self.unit());
            let t = t60 * (lo / f).powf(0.45) * self.range(0.6, 1.3);
            let a = self.range(0.3, 1.0) / (f / lo).powf(0.25);
            m.push((f, t, a));
            if m.len() < n && self.chance(0.35) {
                m.push((f + self.range(0.4, 3.0), t, a * self.range(0.6, 1.0)));
            }
        }
        self.modes(m)
    }

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
    #[allow(dead_code)]
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

    /// Scales every layer added so far: per-patch loudness trim.
    fn scale(&mut self, g: f32) {
        for l in &mut self.layers {
            l.gain *= g;
        }
    }

    fn finish(self, max_len: f32) -> BoxVoice {
        Box::new(Shot {
            air: OnePole::new(self.air, self.sr),
            layers: self.layers,
            scratch: Vec::new(),
            rng: self.rng.into_inner(),
            dt: 1.0 / self.sr,
            left: (max_len * self.sr) as u32,
        })
    }
}

/// A one-shot made of layers.
struct Shot {
    layers: Vec<Layer>,
    /// Gentle top-end roll-off: real recordings of these events have little
    /// above ~8-12 kHz, and raw noise there reads as hiss.
    air: OnePole,
    /// The layers are summed here, then soft-limited: coinciding transients
    /// of a big event would otherwise stack past full scale.
    scratch: Vec<f32>,
    rng: Rng,
    dt: f32,
    left: u32,
}

impl Voice for Shot {
    fn render(&mut self, out: &mut [f32]) -> bool {
        let (rng, dt) = (&mut self.rng, self.dt);
        self.scratch.clear();
        self.scratch.resize(out.len(), 0.0);
        let scratch = &mut self.scratch;
        self.layers.retain_mut(|l| l.render(rng, dt, scratch));
        let last = self.left <= out.len() as u32;
        let n = out.len() as f32;
        for (i, (o, x)) in out.iter_mut().zip(self.scratch.iter()).enumerate() {
            let x = &self.air.lp(*x);
            let a = x.abs();
            let y = if a <= 0.6 {
                *x
            } else {
                x.signum() * (0.6 + 0.35 * ((a - 0.6) / 0.35).tanh())
            };
            // Hard cap: fade the last block instead of cutting.
            *o += if last { y * (1.0 - i as f32 / n) } else { y };
        }
        if last {
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

    /// Load past the onset, 0 at 0.35, 1 at capacity.
    fn intensity(&self) -> f32 {
        ((self.load - 0.35) / 0.65).max(0.0)
    }

    fn spawn(&mut self, delay: f32) {
        let f = self.intensity();
        self.seed = self.seed.wrapping_add(0x6D2B_79F5);
        let mut kit = Kit::new(self.sr, self.seed);
        let amp = (0.7 + 0.3 * f.min(1.2)) * kit.range(0.5, 1.0);
        match self.material {
            MaterialKind::Steel => {
                if f > 0.6 && kit.chance(0.3 * f.min(1.5)) {
                    // A bang or tick: a bolt seat or a plate edge slipping.
                    let ex = kit.burst(1.0, 0.0001, 0.003, None);
                    let lo = kit.range(250.0, 600.0);
                    let body = kit.steel_dense(lo, 5000.0, 12, 0.5);
                    kit.add(delay, ex, body, 0.3 * amp);
                } else {
                    // Groan: slow grinding stick-slip through the member's
                    // dense low modes.
                    let f0 = kit.range(10.0, 25.0);
                    let f1 = f0 * kit.range(0.8, 2.2);
                    let dur = kit.range(0.4, 1.3);
                    let ex = kit.grind(f0, f1, dur, 1.0, 0.4, 0.006);
                    let lo = kit.range(40.0, 70.0);
                    let body = kit.steel_dense(lo, 420.0, 14, kit.range(0.3, 0.7));
                    kit.add(delay, ex, body, 0.42 * amp);
                    let ex = kit.burst(1.0, dur * 0.5, dur * 0.7, None);
                    let body = kit.band(kit.range(600.0, 1200.0), 0.9);
                    kit.add(delay, ex, body, 0.04 * amp);
                }
            }
            MaterialKind::Timber => {
                if f > 0.7 && kit.chance(0.35) {
                    // Dry fibre crackle.
                    let dur = kit.range(0.1, 0.35);
                    let ex = kit.grains(900.0, 0.02, dur, dur, 1.0, dur, 2.2, 0.0015);
                    let body = kit.band(kit.range(1800.0, 3200.0), 0.5);
                    kit.add(delay, ex, body, 0.6 * amp);
                } else {
                    // Creak: a rusty-hinge stick-slip that glides in pitch.
                    let (a, b) = (kit.range(18.0, 45.0), kit.range(45.0, 110.0));
                    let (f0, f1) = if kit.chance(0.6) { (a, b) } else { (b, a) };
                    let dur = kit.range(0.25, 0.8);
                    let ex = kit.grind(f0, f1, dur, 1.0, 0.12, 0.0025);
                    let pitch = kit.range(0.8, 1.3);
                    let body = kit.wood(pitch, 0.08);
                    kit.add(delay, ex, body, 0.5 * amp);
                }
            }
            MaterialKind::Concrete => {
                if f > 0.6 && kit.chance(0.3) {
                    // Fine crumble.
                    let dur = kit.range(0.15, 0.4);
                    let ex = kit.grains(1500.0, 0.01, dur * 0.4, dur, 1.0, dur, 2.2, 0.002);
                    let body = kit.band(kit.range(1200.0, 2500.0), 0.5);
                    kit.add(delay, ex, body, 0.5 * amp);
                } else {
                    // Micro-crack: a short broadband tick with a little body.
                    let ex = kit.burst(1.0, 0.0001, 0.003, None);
                    let body = kit.band(kit.range(1500.0, 5000.0), 0.6);
                    kit.add(delay, ex, body, 0.9 * amp);
                }
            }
            MaterialKind::Composite => {
                if f > 0.7 && kit.chance(0.25) {
                    // A cluster of fibre breaks.
                    let ex = kit.grains(3000.0, 0.005, 0.06, 0.15, 1.0, 0.1, 1.5, 0.0006);
                    let body = kit.high(3000.0);
                    kit.add(delay, ex, body, 0.45 * amp);
                } else {
                    let ex = kit.burst(1.0, 0.0001, 0.002, None);
                    let body = kit.laminate(2000.0, 8000.0, kit.range(0.05, 0.15), 6);
                    kit.add(delay, ex, body, 0.6 * amp);
                }
            }
        }
        self.grains.extend(kit.layers);
    }

    fn base_rate(&self) -> f32 {
        match self.material {
            MaterialKind::Steel => 1.5,
            MaterialKind::Timber => 3.0,
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
            // Audible from about 0.45 utilisation, dense near capacity.
            let i = f.min(1.5);
            let rate = self.base_rate()
                * (0.15 + 1.4 * i * i)
                * (1.0 + self.rate)
                * self.members.max(1.0).sqrt();
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
    k.air = air(material);
    match state {
        DamageState::Intact | DamageState::Broken => return None,
        DamageState::Yielded => {
            if material == MaterialKind::Concrete {
                // Reinforced concrete: the bars yield inside a cracking
                // section. A crack, a low grind and a crumble.
                crack(&mut k, material, s, 0.45, 0.0);
                groan(&mut k, s, 0.3, 0.6 * s.len, 0.7);
                crumble(&mut k, material, s, 0.3, 0.05);
            } else {
                // The section gives: a long grinding groan with a few jerks.
                groan(&mut k, s, 0.5, 1.1 * s.len, 1.0);
                for _ in 0..3 {
                    let t = k.range(0.15, 0.9) * s.len;
                    let f = k.range(50.0, 80.0) * s.pitch;
                    thump(&mut k, f, 0.25, 0.18, t);
                }
            }
        }
        DamageState::Cracked => {
            crack(&mut k, material, s, 0.5, 0.0);
            // A short flurry of the material giving way around the crack.
            let (centre, rate) = match material {
                MaterialKind::Timber => (2400.0, 900.0),
                MaterialKind::Composite => (5000.0, 1500.0),
                _ => (1800.0, 1200.0),
            };
            let ex = k.grains(rate * 0.4, 0.01, 0.12, 0.3, 1.0, 0.15, 2.4, 0.0015);
            let body = k.band(centre * s.pitch, 0.5);
            k.add(0.005, ex, body, 0.45 * s.loud);
        }
        DamageState::Crushed => crunch(&mut k, material, s, 0.55, 0.5),
        DamageState::Delaminated => rip(&mut k, s, 0.7, 0.35),
    }
    // Loudness trims, measured at the reference sizes.
    let trim = match (material, state) {
        (MaterialKind::Concrete, DamageState::Yielded) => 0.65,
        (_, DamageState::Yielded) => 0.4,
        (MaterialKind::Steel, DamageState::Cracked) => 0.8,
        (MaterialKind::Composite, DamageState::Cracked) => 0.38,
        (_, DamageState::Cracked) => 0.65,
        (_, DamageState::Crushed) => 2.0,
        _ => 0.6,
    };
    k.scale(trim);
    Some(k.finish(5.0))
}

// ---------------------------------------------------------------------------
// Shared gestures

/// A heavy low thump: a damped low resonance hit by a soft click and a
/// low-passed noise puff. Gives every big event its weight.
fn thump(k: &mut Kit, f: f32, gain: f32, t60: f32, delay: f32) {
    let ex = k.click(1.0, 0.004);
    let body = k.modes(vec![
        (f, t60, 1.0),
        (f * k.range(1.9, 2.4), t60 * 0.5, 0.35),
    ]);
    k.add(delay, ex, body, gain);
    let ex = k.burst(1.0, 0.002, t60 * 0.5, Some(220.0));
    let body = k.low(180.0);
    k.add(delay, ex, body, gain * 0.9);
}

/// A crack front: a very short, very loud broadband snap with the
/// material's body under it, a low thump and a few milliseconds of
/// micro-fracture.
fn crack(k: &mut Kit, material: MaterialKind, s: Scale, gain: f32, delay: f32) {
    // The snap itself: raw noise, a few milliseconds long.
    let snap_len = match material {
        MaterialKind::Steel => 0.012,
        MaterialKind::Timber => 0.03,
        MaterialKind::Concrete => 0.04,
        MaterialKind::Composite => 0.02,
    };
    let ex = k.burst(1.0, 0.0001, snap_len, None);
    let body = k.high(600.0);
    k.add(delay, ex, body, gain * 0.9);
    // The material ringing from it.
    let body = match material {
        MaterialKind::Steel => k.steel_dense(250.0 * s.pitch, 7000.0, 18, 0.7),
        MaterialKind::Timber => {
            let p = k.range(0.7, 1.0) * s.pitch;
            k.wood(p, 0.1)
        }
        MaterialKind::Concrete => {
            let p = k.range(0.8, 1.2) * s.pitch;
            k.concrete(p, 0.05)
        }
        MaterialKind::Composite => k.laminate(900.0, 6000.0, 0.15, 8),
    };
    let ex = k.burst(1.0, 0.0001, 0.006, None);
    k.add(delay, ex, body, gain * 0.7);
    // Micro-fracture right behind the front.
    let ex = k.grains(1500.0, 0.002, 0.03, 0.08, 0.8, 0.04, 2.0, 0.001);
    let body = k.high(1500.0);
    k.add(delay + 0.002, ex, body, gain * 0.6);
    let f = match material {
        MaterialKind::Concrete => 50.0,
        MaterialKind::Steel => 70.0,
        MaterialKind::Timber => 85.0,
        MaterialKind::Composite => 120.0,
    } * s.pitch;
    thump(k, f, gain * 0.7, 0.25 * s.len, delay);
}

/// A steel section folding: stick-slip grinding through a dense low metal
/// body, rising in pitch, with a metallic scrape on top.
fn groan(k: &mut Kit, s: Scale, gain: f32, dur: f32, rise: f32) {
    let f0 = k.range(14.0, 22.0);
    let ex = k.grind(f0, f0 * (1.0 + 1.6 * rise), dur, 1.0, 0.4, 0.006);
    let lo = k.range(40.0, 60.0) * s.pitch;
    let body = k.steel_dense(lo, 450.0 * s.pitch, 22, 0.5);
    k.add(0.0, ex, body, gain);
    // Scrape: band noise following the grind.
    let ex = k.burst(1.0, dur * 0.7, dur * 0.8, None);
    let body = k.band(k.range(700.0, 1400.0) * s.pitch, 0.5);
    k.add(0.0, ex, body, gain * 0.12);
    // The grind also excites higher, brighter modes a little.
    let ex = k.grind(f0 * 2.0, f0 * 3.5, dur, 1.0, 0.5, 0.003);
    let body = k.steel_dense(600.0 * s.pitch, 3000.0, 10, 0.25);
    k.add(0.0, ex, body, gain * 0.12);
}

/// Crushing: grinding crunch (dense grains, low-mid heavy), a low rumble
/// and a thump.
fn crunch(k: &mut Kit, material: MaterialKind, s: Scale, gain: f32, dur: f32) {
    let dur = dur * s.len;
    let (rate, centre) = match material {
        MaterialKind::Timber => (600.0, 1300.0),
        MaterialKind::Composite => (900.0, 2800.0),
        _ => (800.0, 800.0),
    };
    let ex = k.grains(rate, dur * 0.15, dur * 0.7, dur, 1.0, dur, 2.2, 0.003);
    let body = k.band(centre * s.pitch, 0.45);
    k.add(0.0, ex, body, gain * 1.1);
    let ex = k.grains(rate * 0.35, dur * 0.1, dur, dur, 1.0, dur, 2.4, 0.0015);
    let body = k.high(2500.0);
    k.add(0.0, ex, body, gain * 0.35);
    let ex = k.burst(1.0, dur * 0.2, dur * 1.2, Some(300.0 * s.pitch));
    let body = k.low(250.0 * s.pitch);
    k.add(0.0, ex, body, gain * 0.5);
    let f = if material == MaterialKind::Concrete {
        45.0
    } else {
        70.0
    };
    thump(k, f * s.pitch, gain * 0.8, 0.3, 0.0);
}

/// Grit and fragments trickling down after a fracture.
fn crumble(k: &mut Kit, material: MaterialKind, s: Scale, gain: f32, delay: f32) {
    let dur = k.range(0.7, 1.2) * s.len;
    let centre = match material {
        MaterialKind::Concrete => 2200.0,
        MaterialKind::Timber => 2800.0,
        _ => 3500.0,
    };
    let ex = k.grains(220.0, 0.05, dur * 0.4, dur, 1.0, dur * 0.6, 2.6, 0.002);
    let body = k.band(centre, 0.5);
    k.add(delay, ex, body, gain);
    let ex = k.grains(80.0, 0.1, dur * 0.6, dur, 1.0, dur * 0.8, 2.2, 0.005);
    let body = k.band(700.0 * s.pitch, 0.6);
    k.add(delay, ex, body, gain * 0.6);
}

/// Fragments landing: a handful of small, quick hits spread over `span`,
/// most early.
fn debris(k: &mut Kit, material: MaterialKind, count: usize, start: f32, span: f32, gain: f32) {
    for _ in 0..count {
        let t = start + span * k.unit().powf(1.8);
        let g = gain * k.range(0.3, 1.0) * (1.0 - (t - start) / (span + 1e-3) * 0.6);
        let ex = k.burst(1.0, 0.0001, 0.007, None);
        let body = match material {
            MaterialKind::Steel => {
                let lo = k.range(350.0, 900.0);
                k.steel_dense(lo, lo * 8.0, 8, 0.12)
            }
            MaterialKind::Timber => {
                let p = k.range(1.4, 2.5);
                k.wood(p, 0.04)
            }
            MaterialKind::Concrete => k.band(k.range(1200.0, 3500.0), 0.7),
            MaterialKind::Composite => k.laminate(1500.0, 6000.0, 0.05, 4),
        };
        k.add(t, ex, body, g);
    }
}

/// Tearing along a seam: a dense rasp of grains plus a fibrous hiss.
fn rip(k: &mut Kit, s: Scale, gain: f32, dur: f32) {
    let dur = dur * s.len;
    let ex = k.grains(
        1200.0,
        dur * 0.3,
        dur * 2.0,
        dur,
        1.0,
        dur * 1.5,
        2.0,
        0.0012,
    );
    let body = k.band(k.range(2600.0, 3600.0), 0.5);
    k.add(0.0, ex, body, gain * 0.8);
    let ex = k.burst(1.0, dur * 0.3, dur * 1.2, None);
    let body = k.high(3000.0);
    k.add(0.0, ex, body, gain * 0.15);
    let ex = k.burst(1.0, 0.0001, 0.01, None);
    let body = k.laminate(1500.0, 6000.0, 0.12, 6);
    k.add(dur * 0.9, ex, body, gain * 0.5);
}

/// Steel snapping in tension: a hard broadband crack, a noisy dense ring,
/// a short twang of the freed end and a thump.
fn snap(k: &mut Kit, s: Scale, gain: f32, delay: f32) {
    let ex = k.burst(1.0, 0.0001, 0.006, None);
    let body = k.pass();
    k.add(delay, ex, body, gain * 0.8);
    let ex = k.burst(1.0, 0.0001, 0.004, None);
    let base = k.range(220.0, 320.0) * s.pitch;
    let body = k.steel_dense(base, 8000.0, 26, 1.1 * s.len);
    k.add(delay, ex, body, gain * 0.9);
    // Twang: the freed end, nearly harmonic but detuned and short.
    let f = k.range(140.0, 220.0) * s.pitch;
    let body = k.modes(vec![
        (f, 0.5 * s.len, 1.0),
        (f * 2.02 + k.range(-2.0, 2.0), 0.35 * s.len, 0.6),
        (f * 3.05, 0.25 * s.len, 0.4),
        (f * 4.1, 0.15 * s.len, 0.3),
        (f * 5.3, 0.1 * s.len, 0.2),
    ]);
    let ex = k.burst(1.0, 0.0001, 0.003, None);
    k.add(delay + 0.003, ex, body, gain * 0.35);
    let f = k.range(60.0, 80.0) * s.pitch;
    thump(k, f, gain * 0.6, 0.2, delay);
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
    k.air = air(material);
    let g = s.loud;
    use BreakCause as C;
    use MaterialKind as M;
    match (material, cause) {
        (M::Steel, C::Rupture) | (M::Steel, C::Brittle) => {
            snap(&mut k, s, 0.8 * g, 0.0);
            debris(&mut k, material, 5, 0.25, 0.9, 0.25 * g);
        }
        (M::Steel, _) => {
            // Hinge: the section grinds as it folds, then gives with a bang
            // and rings.
            let bend = k.range(0.6, 1.0) * s.len;
            groan(&mut k, s, 0.55 * g, bend, 1.0);
            let ex = k.burst(1.0, 0.0001, 0.01, None);
            let body = k.pass();
            k.add(bend, ex, body, 0.6 * g);
            let ex = k.burst(1.0, 0.0001, 0.005, None);
            let lo = k.range(110.0, 160.0) * s.pitch;
            let body = k.steel_dense(lo, 6000.0, 26, 1.3 * s.len);
            k.add(bend, ex, body, 0.8 * g);
            let f = k.range(50.0, 65.0) * s.pitch;
            thump(&mut k, f, 0.8 * g, 0.3, bend);
            debris(&mut k, material, 4, bend + 0.2, 0.8, 0.2 * g);
        }
        (M::Timber, C::TimberSplit) => {
            // Long tear along the grain: irregular creaking stick-slip and
            // a rising rasp of fibres, ending in a crack.
            let dur = k.range(0.8, 1.2) * s.len;
            let ex = k.grind(25.0, 70.0, dur, 1.0, 0.45, 0.004);
            let p = 0.8 * s.pitch;
            let body = k.wood(p, 0.06);
            k.add(0.0, ex, body, 0.55 * g);
            let ex = k.grains(
                450.0,
                dur * 0.8,
                dur * 3.0,
                dur,
                1.0,
                dur * 3.0,
                2.2,
                0.0012,
            );
            let body = k.band(2200.0 * s.pitch, 0.45);
            k.add(0.0, ex, body, 0.5 * g);
            crack(&mut k, material, s, 0.8 * g, dur);
            splinters(&mut k, s, 0.5 * g, dur + 0.01, 0.5);
        }
        (M::Timber, C::Crushing) => crunch(&mut k, material, s, 0.7 * g, 0.6),
        (M::Timber, _) => {
            // Splinter: a big dry crack, then a cascade of fibres letting go.
            crack(&mut k, material, s, 0.85 * g, 0.0);
            let dur = k.range(0.7, 1.2);
            splinters(&mut k, s, 0.65 * g, 0.01, dur);
            debris(&mut k, material, 4, 0.3, 0.8, 0.3 * g);
        }
        (M::Concrete, C::Crushing) => {
            crunch(&mut k, material, s, 0.75 * g, 0.8);
            crumble(&mut k, material, s, 0.45 * g, 0.2);
            debris(&mut k, material, 8, 0.2, 1.2, 0.3 * g);
        }
        (M::Concrete, C::RebarRupture) | (M::Concrete, C::Rupture) => {
            crack(&mut k, material, s, 0.75 * g, 0.0);
            crunch(&mut k, material, s, 0.4 * g, 0.3);
            crumble(&mut k, material, s, 0.45 * g, 0.05);
            let bar = Scale {
                pitch: 1.5,
                len: 0.7,
                loud: 1.0,
            };
            snap(&mut k, bar, 0.5 * g, 0.07);
            debris(&mut k, material, 8, 0.15, 1.2, 0.3 * g);
        }
        (M::Concrete, _) => {
            // Brittle or shear: a loud crack, a crunch, crumbling and
            // fragments landing.
            crack(&mut k, material, s, 0.85 * g, 0.0);
            let len = if cause == C::ConcreteShear { 0.6 } else { 0.35 };
            crunch(&mut k, material, s, 0.5 * g, len);
            crumble(&mut k, material, s, 0.5 * g, 0.05);
            debris(&mut k, material, 10, 0.15, 1.3, 0.3 * g);
        }
        (M::Composite, C::Delamination) => {
            rip(&mut k, s, 0.75 * g, 0.5);
            crack(&mut k, material, s, 0.6 * g, 0.0);
        }
        (M::Composite, _) => {
            // Fibre rupture: an explosive crack and a fizz of fibres.
            crack(&mut k, material, s, 0.8 * g, 0.0);
            let ex = k.grains(3500.0, 0.005, 0.18, 0.5 * s.len, 1.0, 0.2, 1.8, 0.0005);
            let body = k.high(3500.0);
            k.add(0.003, ex, body, 0.5 * g);
            let ex = k.grains(2000.0, 0.005, 0.1, 0.3, 1.0, 0.1, 1.6, 0.002);
            let body = k.band(1800.0, 0.6);
            k.add(0.003, ex, body, 0.35 * g);
            debris(&mut k, material, 3, 0.2, 0.6, 0.2 * g);
        }
    }
    // Loudness trims, measured at the reference sizes.
    let trim = match (material, cause) {
        (M::Steel, C::Rupture) | (M::Steel, C::Brittle) => 0.8,
        (M::Steel, _) => 0.5,
        (M::Timber, C::TimberSplit) => 0.58,
        (M::Timber, C::Crushing) => 2.3,
        (M::Timber, _) => 0.55,
        (M::Concrete, C::Crushing) => 2.5,
        (M::Concrete, C::RebarRupture) | (M::Concrete, C::Rupture) => 0.75,
        (M::Concrete, _) => 0.55,
        (M::Composite, C::Delamination) => 0.85,
        (M::Composite, _) => 0.55,
    };
    k.scale(trim);
    k.finish(6.0)
}

/// Timber fibres giving way after the main crack: an accelerating, then
/// thinning cascade of dry crackles, with wood under it and a few fibres
/// pulling out with a short creak.
fn splinters(k: &mut Kit, s: Scale, gain: f32, delay: f32, dur: f32) {
    let dur = dur * s.len;
    let ex = k.grains(420.0, 0.02, dur * 0.6, dur, 1.0, dur * 0.7, 2.4, 0.0012);
    let body = k.band(k.range(2200.0, 3200.0), 0.45);
    k.add(delay, ex, body, gain);
    let ex = k.grains(160.0, 0.02, dur * 0.6, dur, 1.0, dur * 0.7, 2.0, 0.003);
    let body = k.band(k.range(450.0, 800.0) * s.pitch, 0.5);
    k.add(delay, ex, body, gain * 0.7);
    let ex = k.grains(60.0, dur * 0.1, dur * 0.5, dur, 1.0, dur * 0.7, 1.6, 0.002);
    let p = s.pitch * k.range(1.0, 1.4);
    let body = k.wood(p, 0.04);
    k.add(delay, ex, body, gain * 0.5);
    for _ in 0..3 {
        let t = delay + k.range(0.03, dur * 0.7);
        let f0 = k.range(60.0, 140.0);
        let ex = k.grind(
            f0,
            f0 * k.range(1.3, 2.2),
            k.range(0.05, 0.15),
            1.0,
            0.3,
            0.002,
        );
        let p = s.pitch * k.range(1.2, 2.0);
        let body = k.wood(p, 0.05);
        k.add(t, ex, body, gain * 0.35);
    }
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
    let rel = size.max(0.01) / (reference(material) * 0.2);
    let pitch = rel.powf(-0.25).clamp(0.5, 3.0);
    // Heavier pieces carry more low end.
    let weight = rel.powf(0.3).clamp(0.3, 2.0);
    let gain = ((speed / 6.0).powf(1.1) * rel.powf(0.15)).clamp(0.0, 1.0) * 0.55;
    let mut k = Kit::new(sr, seed);
    k.air = air(material);
    let bounce = speed > 2.0 && k.chance(0.6);
    match material {
        MaterialKind::Steel => {
            let ex = k.burst(1.0, 0.0001, 0.004, None);
            let body = k.pass();
            k.add(0.0, ex, body, 0.45 * gain);
            // Clatter: the contact chattering, a noisy metallic hiss.
            let ex = k.burst(1.0, 0.0001, k.range(0.12, 0.25), None);
            let body = k.band(k.range(2500.0, 4500.0), 0.4);
            k.add(0.0, ex, body, 0.35 * gain);
            let ex = k.burst(1.0, 0.0001, 0.015, None);
            let lo = k.range(180.0, 300.0) * pitch;
            let t60 = k.range(0.5, 0.9);
            let body = k.steel_dense(lo, 11000.0, 28, t60);
            k.add(0.0, ex, body, 1.0 * gain);
            thump(&mut k, 70.0 * pitch.sqrt(), 0.5 * gain * weight, 0.15, 0.0);
            if bounce {
                let t = k.range(0.12, 0.3);
                let ex = k.burst(1.0, 0.0001, 0.003, None);
                let body = k.steel_dense(lo * 1.1, 6000.0, 10, t60 * 0.6);
                k.add(t, ex, body, 0.45 * gain);
            }
        }
        MaterialKind::Timber => {
            // Hollow knock with a little clatter.
            let ex = k.burst(1.0, 0.0001, 0.006, None);
            let p = 1.2 * pitch * k.range(0.85, 1.15);
            let body = k.wood(p, 0.06);
            k.add(0.0, ex, body, 1.3 * gain);
            let ex = k.burst(1.0, 0.0001, 0.008, None);
            let body = k.band(1600.0, 0.6);
            k.add(0.0, ex, body, 0.4 * gain);
            thump(&mut k, 95.0 * pitch.sqrt(), 0.4 * gain * weight, 0.1, 0.0);
            if bounce {
                for i in 0..2 {
                    let t = k.range(0.08, 0.18) * (i + 1) as f32;
                    let ex = k.burst(1.0, 0.0001, 0.004, None);
                    let p = p * k.range(0.9, 1.2);
                    let body = k.wood(p, 0.04);
                    k.add(t, ex, body, 0.5 * gain / (i + 1) as f32);
                }
            }
        }
        MaterialKind::Concrete => {
            // Dull thud and a scatter of gravel.
            let f = k.range(55.0, 80.0) * pitch.sqrt();
            thump(&mut k, f, 0.9 * gain * weight, 0.12, 0.0);
            let ex = k.burst(1.0, 0.0001, 0.03, Some(900.0));
            let body = k.band(500.0 * pitch, 0.6);
            k.add(0.0, ex, body, 0.6 * gain);
            let ex = k.grains(500.0, 0.005, 0.08, 0.3, 1.0, 0.1, 2.2, 0.002);
            let body = k.band(2600.0, 0.5);
            k.add(0.002, ex, body, 0.5 * gain);
        }
        MaterialKind::Composite => {
            let ex = k.burst(1.0, 0.0001, 0.003, None);
            let lo = 800.0 * pitch;
            let body = k.laminate(lo, lo * 5.0, 0.06, 6);
            k.add(0.0, ex, body, 1.0 * gain);
            let ex = k.burst(1.0, 0.0001, 0.004, None);
            let body = k.high(2000.0);
            k.add(0.0, ex, body, 0.35 * gain);
            thump(
                &mut k,
                120.0 * pitch.sqrt(),
                0.25 * gain * weight,
                0.08,
                0.0,
            );
        }
    }
    k.scale(match material {
        MaterialKind::Steel => 0.38,
        MaterialKind::Timber => 0.45,
        MaterialKind::Concrete => 1.1,
        MaterialKind::Composite => 0.95,
    });
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

    /// Writes every break, damage and impact sound dry to
    /// `$BREAKIT_DRY_DIR` for listening and spectrograms.
    #[test]
    #[ignore]
    fn dump_dry() {
        let Ok(dir) = std::env::var("BREAKIT_DRY_DIR") else {
            return;
        };
        let dir = std::path::PathBuf::from(dir);
        std::fs::create_dir_all(&dir).unwrap();
        let render = |mut v: BoxVoice| {
            let mut out = Vec::new();
            let mut buf = [0.0f32; 128];
            loop {
                buf.iter_mut().for_each(|x| *x = 0.0);
                let alive = v.render(&mut buf);
                out.extend_from_slice(&buf);
                if !alive {
                    break;
                }
            }
            out.extend(std::iter::repeat(0.0).take(SR as usize / 4));
            out
        };
        for m in MATS {
            let mut all = Vec::new();
            for c in CAUSES {
                all.extend(render(break_voice(m, c, reference(m), SR, 7)));
            }
            crate::wav::write_wav(&dir.join(format!("break_{m:?}.wav")), &all, 1, SR as u32)
                .unwrap();
            let mut all = Vec::new();
            for v in [8.0, 4.0, 2.0] {
                for seed in 0..2 {
                    all.extend(render(
                        impact_voice(m, v, reference(m) * 0.2, SR, seed).unwrap(),
                    ));
                }
            }
            crate::wav::write_wav(&dir.join(format!("impact_{m:?}.wav")), &all, 1, SR as u32)
                .unwrap();
        }
    }

    #[test]
    #[ignore]
    fn print_stress_curve() {
        for m in MATS {
            let mut line = format!("{m:?}:");
            for load in [0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0] {
                let mut v = StressVoice::new(m, SR, 3);
                v.set(load, 0.1, 4.0);
                let mut buf = vec![0.0f32; 128];
                let (mut sum, mut n) = (0.0f32, 0);
                for _ in 0..(10.0 * SR / 128.0) as usize {
                    buf.iter_mut().for_each(|x| *x = 0.0);
                    v.render(&mut buf);
                    sum += buf.iter().map(|x| x * x).sum::<f32>();
                    n += buf.len();
                }
                line += &format!(" {load}:{:.3}", (sum / n as f32).sqrt());
            }
            println!("{line}");
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
