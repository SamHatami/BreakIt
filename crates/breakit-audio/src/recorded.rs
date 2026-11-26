//! Structure one-shots built from CC0 recordings, varied and layered with
//! synthesis.
//!
//! Pure synthesis struggles with the dense, irregular texture of real
//! fracture (thousands of micro-cracks, grit, fibres tearing). So the
//! transient and texture of a break or impact come from a small bank of
//! recordings (`samples/`, made by `tools/cc0_samples.py`, all CC0), and code
//! does what recordings cannot:
//!
//! * **Variation.** Every event picks a clip at random, shifts its pitch
//!   (and so its length) by a few percent, filters it, and spreads layers in
//!   time, so a collapse of forty members never plays the same file twice in
//!   the same way.
//! * **Physics.** Member mass sets pitch and weight (heavier is lower and
//!   longer), impact speed sets loudness and brightness, the failure cause
//!   picks the layers.
//! * **What no recording has.** The synthesised parts stay: steel's ring
//!   and groan, the low thump of a heavy member, grit scatter. They are
//!   mixed under the recordings.

use std::sync::{Arc, OnceLock};

use crate::dsp::{OnePole, Rng, Svf};
use crate::sample_index::{files, Group, GROUPS};
use crate::{material, BoxVoice, BreakCause, DamageState, MaterialKind, Voice};

/// Sample rate of the files in `samples/`.
const SOURCE_RATE: f32 = 32_000.0;

type Clip = Arc<[f32]>;

fn bank() -> &'static Vec<(Group, Vec<Clip>)> {
    static BANK: OnceLock<Vec<(Group, Vec<Clip>)>> = OnceLock::new();
    BANK.get_or_init(|| {
        GROUPS
            .iter()
            .map(|&g| (g, files(g).iter().filter_map(|b| decode_wav(b)).collect()))
            .collect()
    })
}

/// Decodes the bank now (about 2 MB of 16-bit PCM), so the first break in a
/// game does not pay for it on the audio thread.
pub fn preload() {
    let _ = bank();
}

fn clips(g: Group) -> &'static [Clip] {
    bank()
        .iter()
        .find(|(k, _)| *k == g)
        .map(|(_, v)| v.as_slice())
        .unwrap_or(&[])
}

/// 16-bit mono PCM WAV to f32. Only what `cc0_samples.py` writes.
fn decode_wav(bytes: &[u8]) -> Option<Clip> {
    if bytes.len() < 12 || &bytes[0..4] != b"RIFF" || &bytes[8..12] != b"WAVE" {
        return None;
    }
    let mut i = 12;
    while i + 8 <= bytes.len() {
        let id = &bytes[i..i + 4];
        let len = u32::from_le_bytes(bytes[i + 4..i + 8].try_into().ok()?) as usize;
        let body = bytes.get(i + 8..(i + 8 + len).min(bytes.len()))?;
        if id == b"data" {
            return Some(
                body.chunks_exact(2)
                    .map(|c| i16::from_le_bytes([c[0], c[1]]) as f32 / 32768.0)
                    .collect(),
            );
        }
        i += 8 + len + (len & 1);
    }
    None
}

/// Plays one clip: resampled (pitch and speed together), gained and
/// optionally low- or high-passed.
pub struct SampleVoice {
    clip: Clip,
    pos: f64,
    step: f64,
    gain: f32,
    lp: Option<Svf>,
    hp: Option<OnePole>,
    delay: u32,
}

impl SampleVoice {
    fn new(clip: Clip, rate: f32, gain: f32, sr: f32) -> Self {
        Self {
            clip,
            pos: 0.0,
            step: (SOURCE_RATE / sr * rate) as f64,
            gain,
            lp: None,
            hp: None,
            delay: 0,
        }
    }

    fn lowpass(mut self, cutoff: f32, sr: f32) -> Self {
        if cutoff < sr * 0.45 {
            self.lp = Some(Svf::new(cutoff, 0.7, sr));
        }
        self
    }

    fn highpass(mut self, cutoff: f32, sr: f32) -> Self {
        self.hp = Some(OnePole::new(cutoff, sr));
        self
    }

    fn after(mut self, seconds: f32, sr: f32) -> Self {
        self.delay = (seconds.max(0.0) * sr) as u32;
        self
    }
}

impl Voice for SampleVoice {
    fn render(&mut self, out: &mut [f32]) -> bool {
        let n = self.clip.len();
        for o in out.iter_mut() {
            if self.delay > 0 {
                self.delay -= 1;
                continue;
            }
            let i = self.pos as usize;
            if i + 1 >= n {
                return false;
            }
            let f = (self.pos - i as f64) as f32;
            let mut x = self.clip[i] + (self.clip[i + 1] - self.clip[i]) * f;
            if let Some(lp) = &mut self.lp {
                x = lp.lp(x);
            }
            if let Some(hp) = &mut self.hp {
                x = hp.hp(x);
            }
            *o += x * self.gain;
            self.pos += self.step;
        }
        true
    }
}

/// Several voices started together (each may carry its own delay), with a
/// gain per voice. Ends when all have.
pub struct Stack {
    parts: Vec<(BoxVoice, f32, u32)>,
    scratch: Vec<f32>,
}

impl Stack {
    fn new() -> Self {
        Self {
            parts: Vec::new(),
            scratch: Vec::new(),
        }
    }

    fn push(&mut self, v: impl Voice + 'static, gain: f32) {
        self.parts.push((Box::new(v), gain, 0));
    }

    fn push_boxed(&mut self, v: BoxVoice, gain: f32, delay: f32, sr: f32) {
        self.parts.push((v, gain, (delay.max(0.0) * sr) as u32));
    }
}

impl Voice for Stack {
    fn render(&mut self, out: &mut [f32]) -> bool {
        self.scratch.resize(out.len(), 0.0);
        let scratch = &mut self.scratch;
        self.parts.retain_mut(|(v, g, delay)| {
            let skip = (*delay as usize).min(out.len());
            *delay -= skip as u32;
            if skip == out.len() {
                return true;
            }
            let s = &mut scratch[skip..];
            s.iter_mut().for_each(|x| *x = 0.0);
            let alive = v.render(s);
            for (o, x) in out[skip..].iter_mut().zip(s.iter()) {
                *o += *x * *g;
            }
            alive
        });
        !self.parts.is_empty()
    }
}

/// Reference member masses (kg) at which clips play at their own pitch.
fn reference(m: MaterialKind) -> f32 {
    match m {
        MaterialKind::Steel => 40.0,
        MaterialKind::Timber => 30.0,
        MaterialKind::Concrete => 300.0,
        MaterialKind::Composite => 10.0,
    }
}

/// Playback rate from mass: twice the mass plays about 6% lower.
fn mass_rate(m: MaterialKind, size: f32) -> f32 {
    (reference(m) / size.max(0.01)).powf(0.09).clamp(0.7, 1.35)
}

/// Loudness from mass, relative to the reference.
fn mass_gain(m: MaterialKind, size: f32) -> f32 {
    (1.0 + 0.25 * (size.max(0.01) / reference(m)).log2()).clamp(0.45, 1.4)
}

/// One step of building a sound: picks clips and varies them.
struct Builder {
    rng: Rng,
    sr: f32,
    stack: Stack,
}

impl Builder {
    fn new(sr: f32, seed: u32) -> Self {
        Self {
            rng: Rng::new(seed ^ 0x5EC0_4D3D),
            sr,
            stack: Stack::new(),
        }
    }

    /// Adds a random clip from `group`. `rate` is varied by ±6%, `gain` by
    /// about ±2 dB, and the brightness by a random low-pass.
    fn clip(&mut self, group: Group, rate: f32, gain: f32, delay: f32) -> &mut Self {
        let all = clips(group);
        if all.is_empty() {
            return self;
        }
        let clip = all[(self.rng.next_u32() as usize) % all.len()].clone();
        let rate = self.rng.vary(rate, 0.06);
        let gain = self.rng.vary(gain, 0.22);
        let cutoff = self.rng.range(7_000.0, 16_000.0);
        let v = SampleVoice::new(clip, rate, gain, self.sr)
            .lowpass(cutoff, self.sr)
            .after(delay, self.sr);
        self.stack.push(v, 1.0);
        self
    }

    /// Like [`Builder::clip`] but thinned to the highs (fibres, fizz).
    fn bright(&mut self, group: Group, rate: f32, gain: f32, delay: f32) -> &mut Self {
        let all = clips(group);
        if all.is_empty() {
            return self;
        }
        let clip = all[(self.rng.next_u32() as usize) % all.len()].clone();
        let rate = self.rng.vary(rate, 0.06);
        let v = SampleVoice::new(clip, rate, gain, self.sr)
            .highpass(1_800.0, self.sr)
            .after(delay, self.sr);
        self.stack.push(v, 1.0);
        self
    }

    /// A shower of `count` small pieces from `group` over `spread` seconds,
    /// thinning out (debris settling after a break).
    fn scatter(
        &mut self,
        group: Group,
        count: usize,
        start: f32,
        spread: f32,
        rate: f32,
        gain: f32,
    ) -> &mut Self {
        for i in 0..count {
            let t = start + spread * (self.rng.unit().powf(1.6));
            let g = gain * (1.0 - 0.6 * i as f32 / count.max(1) as f32);
            let r = rate * self.rng.range(1.0, 1.4);
            self.clip(group, r, g, t);
        }
        self
    }

    /// Mixes in a synthesised layer.
    fn synth(&mut self, v: Option<BoxVoice>, gain: f32, delay: f32) -> &mut Self {
        if let Some(v) = v {
            self.stack.push_boxed(v, gain, delay, self.sr);
        }
        self
    }

    fn done(self) -> BoxVoice {
        Box::new(self.stack)
    }
}

/// Damage onsets: smaller, quieter cousins of the breaks.
pub fn damage_voice(
    m: MaterialKind,
    state: DamageState,
    size: f32,
    sr: f32,
    seed: u32,
) -> Option<BoxVoice> {
    let rate = mass_rate(m, size);
    let g = mass_gain(m, size);
    let mut b = Builder::new(sr, seed);
    let synth = material::damage_voice(m, state, size, sr, seed);
    match (m, state) {
        (_, DamageState::Intact | DamageState::Broken) => return None,
        // Steel yielding is a groan: synthesis does that better than any
        // clip we have; a muffled heavy knock gives it weight.
        (MaterialKind::Steel, _) => {
            b.synth(synth, 1.0, 0.0);
            b.clip(Group::MetalHeavy, rate * 0.8, 0.15 * g, 0.05);
        }
        (MaterialKind::Timber, DamageState::Crushed) => {
            b.clip(Group::WoodBreak, rate * 0.8, 0.35 * g, 0.0);
            b.synth(synth, 0.5, 0.0);
        }
        (MaterialKind::Timber, _) => {
            b.clip(Group::WoodBreak, rate, 0.3 * g, 0.0);
            b.synth(synth, 0.6, 0.0);
        }
        (MaterialKind::Concrete, DamageState::Crushed) => {
            b.clip(Group::Crunch, rate * 0.9, 0.4 * g, 0.0);
            b.scatter(Group::RockFall, 2, 0.1, 0.4, rate * 1.2, 0.12 * g);
        }
        (MaterialKind::Concrete, _) => {
            b.clip(Group::RockBreak, rate * 1.15, 0.22 * g, 0.0);
            b.synth(synth, 0.5, 0.0);
        }
        (MaterialKind::Composite, _) => {
            b.bright(Group::WoodCrack, rate * 1.35, 0.25 * g, 0.0);
            b.synth(synth, 0.6, 0.0);
        }
    }
    Some(b.done())
}

/// A member breaking.
pub fn break_voice(m: MaterialKind, cause: BreakCause, size: f32, sr: f32, seed: u32) -> BoxVoice {
    let rate = mass_rate(m, size);
    let g = mass_gain(m, size);
    let mut b = Builder::new(sr, seed);
    let synth = || material::break_voice(m, cause, size, sr, seed);
    match (m, cause) {
        // Tension rupture: a hard metallic crack, the synthesised twang and
        // ring under it, then the ends dropping.
        (MaterialKind::Steel, BreakCause::Rupture) => {
            b.clip(Group::MetalHit, rate * 1.05, 0.75 * g, 0.0);
            b.clip(Group::MetalSmall, rate * 0.9, 0.35 * g, 0.004);
            b.synth(Some(synth()), 0.55, 0.0);
            b.clip(Group::MetalFall, rate, 0.35 * g, 0.25);
        }
        // Plastic hinge: the groan of the bend (synthesis) running into a
        // heavy clank and the pieces sagging down.
        (MaterialKind::Steel, _) => {
            b.synth(Some(synth()), 0.6, 0.0);
            b.clip(Group::MetalHeavy, rate * 0.85, 0.6 * g, 0.18);
            b.clip(Group::MetalHit, rate * 0.8, 0.55 * g, 0.2);
            b.clip(Group::MetalFall, rate * 0.9, 0.3 * g, 0.45);
        }
        (MaterialKind::Timber, BreakCause::TimberSplit) => {
            b.clip(Group::WoodCrack, rate * 0.85, 0.8 * g, 0.0);
            b.clip(Group::WoodBreak, rate * 0.9, 0.35 * g, 0.25);
            b.synth(Some(synth()), 0.35, 0.0);
        }
        (MaterialKind::Timber, BreakCause::Crushing) => {
            b.clip(Group::WoodBreak, rate * 0.75, 0.7 * g, 0.0);
            b.clip(Group::Crunch, rate * 1.2, 0.3 * g, 0.02);
            b.synth(Some(synth()), 0.35, 0.0);
        }
        // Splinter (and anything else timber): the big crack, a second
        // break as fibres let go, splinters landing.
        (MaterialKind::Timber, _) => {
            b.clip(Group::WoodCrack, rate, 0.9 * g, 0.0);
            b.clip(Group::WoodBreak, rate, 0.45 * g, 0.06);
            b.synth(Some(synth()), 0.3, 0.0);
            b.scatter(Group::WoodHit, 3, 0.35, 0.6, rate * 1.3, 0.18 * g);
        }
        (MaterialKind::Concrete, BreakCause::Crushing) => {
            b.clip(Group::Crunch, rate * 0.85, 0.85 * g, 0.0);
            b.clip(Group::Crunch, rate * 0.95, 0.5 * g, 0.12);
            b.scatter(Group::RockFall, 4, 0.2, 0.9, rate * 1.1, 0.3 * g);
            b.synth(Some(synth()), 0.3, 0.0);
        }
        (MaterialKind::Concrete, BreakCause::RebarRupture) => {
            b.clip(Group::RockBreak, rate, 0.75 * g, 0.0);
            b.clip(Group::MetalSmall, rate * 0.8, 0.45 * g, 0.03);
            b.clip(Group::MetalHit, rate * 1.1, 0.35 * g, 0.05);
            b.scatter(Group::RockFall, 4, 0.2, 0.9, rate * 1.1, 0.3 * g);
        }
        // Brittle and shear: crack, crunch, chunks and grit raining down.
        (MaterialKind::Concrete, _) => {
            b.clip(Group::RockBreak, rate, 0.85 * g, 0.0);
            b.clip(Group::Crunch, rate * 0.9, 0.45 * g, 0.05);
            b.scatter(Group::RockFall, 5, 0.25, 1.1, rate * 1.1, 0.35 * g);
            b.synth(Some(synth()), 0.25, 0.0);
        }
        // Composites: a sharp, dry crack with fizzing fibres. Pitched-up
        // wood cracks thinned to the highs, over the synthesised fizz.
        (MaterialKind::Composite, _) => {
            b.clip(Group::WoodCrack, rate * 1.3, 0.6 * g, 0.0);
            b.bright(Group::WoodBreak, rate * 1.5, 0.5 * g, 0.01);
            b.synth(Some(synth()), 0.55, 0.0);
        }
    }
    b.done()
}

/// A loose piece hitting the floor.
pub fn impact_voice(
    m: MaterialKind,
    speed: f32,
    size: f32,
    sr: f32,
    seed: u32,
) -> Option<BoxVoice> {
    if speed < 0.3 {
        return None;
    }
    // Loudness and brightness with speed; pitch with mass (pieces are
    // smaller than members, so the reference is a tenth).
    let v = (speed / 6.0).clamp(0.0, 1.5);
    let gain = 0.75 * v.powf(1.2) * (0.6 + 0.25 * (size.max(0.05)).log10().clamp(-1.0, 2.0));
    let rate = (reference(m) * 0.1 / size.max(0.01))
        .powf(0.08)
        .clamp(0.75, 1.4);
    let cutoff = 2_500.0 + 12_000.0 * v.min(1.0);
    let mut b = Builder::new(sr, seed);
    let group = match m {
        MaterialKind::Steel if size > 15.0 => Group::MetalHeavy,
        MaterialKind::Steel if size < 2.0 => Group::MetalSmall,
        MaterialKind::Steel => Group::MetalHit,
        MaterialKind::Timber | MaterialKind::Composite => Group::WoodHit,
        MaterialKind::Concrete => Group::RockFall,
    };
    let rate = if m == MaterialKind::Composite {
        rate * 1.3
    } else {
        rate
    };
    b.clip(group, rate, gain, 0.0);
    if let Some(last) = b.stack.parts.last_mut() {
        // Slow hits are duller.
        let inner = std::mem::replace(&mut last.0, Box::new(Silent));
        last.0 = Box::new(LowPassed {
            inner,
            lp: Svf::new(cutoff, 0.7, sr),
            scratch: Vec::new(),
        });
    }
    // Big concrete hits shed grit.
    if m == MaterialKind::Concrete && size > 5.0 && v > 0.4 {
        b.scatter(Group::RockFall, 2, 0.05, 0.3, rate * 1.3, gain * 0.3);
    }
    Some(b.done())
}

struct Silent;
impl Voice for Silent {
    fn render(&mut self, _out: &mut [f32]) -> bool {
        false
    }
}

struct LowPassed {
    inner: BoxVoice,
    lp: Svf,
    scratch: Vec<f32>,
}

impl Voice for LowPassed {
    fn render(&mut self, out: &mut [f32]) -> bool {
        self.scratch.clear();
        self.scratch.resize(out.len(), 0.0);
        let alive = self.inner.render(&mut self.scratch);
        for (o, x) in out.iter_mut().zip(&self.scratch) {
            *o += self.lp.lp(*x);
        }
        alive
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn render(mut v: BoxVoice, max_s: f32) -> (Vec<f32>, bool) {
        let sr = 48_000.0;
        let mut out = Vec::new();
        let mut buf = vec![0.0; 128];
        let mut ended = false;
        for _ in 0..(max_s * sr / 128.0) as usize {
            buf.iter_mut().for_each(|x| *x = 0.0);
            let alive = v.render(&mut buf);
            out.extend_from_slice(&buf);
            if !alive {
                ended = true;
                break;
            }
        }
        (out, ended)
    }

    #[test]
    fn every_group_has_decodable_clips() {
        for g in GROUPS {
            let c = clips(g);
            assert!(!c.is_empty(), "{g:?} empty");
            for clip in c {
                assert!(clip.len() > 1000);
                let peak = clip.iter().fold(0.0f32, |m, x| m.max(x.abs()));
                assert!(peak > 0.5 && peak <= 1.0, "{g:?} peak {peak}");
            }
        }
    }

    #[test]
    fn breaks_and_impacts_are_bounded_and_end() {
        let causes = [
            (MaterialKind::Steel, BreakCause::Rupture),
            (MaterialKind::Steel, BreakCause::Hinge),
            (MaterialKind::Timber, BreakCause::Splinter),
            (MaterialKind::Timber, BreakCause::TimberSplit),
            (MaterialKind::Timber, BreakCause::Crushing),
            (MaterialKind::Concrete, BreakCause::Brittle),
            (MaterialKind::Concrete, BreakCause::Crushing),
            (MaterialKind::Concrete, BreakCause::RebarRupture),
            (MaterialKind::Composite, BreakCause::FibreRupture),
        ];
        for (seed, (m, c)) in causes.into_iter().enumerate() {
            let (out, ended) = render(break_voice(m, c, reference(m), 48_000.0, seed as u32), 8.0);
            let peak = out.iter().fold(0.0f32, |a, x| a.max(x.abs()));
            assert!(ended, "{m:?} {c:?} never ended");
            assert!(peak > 0.2 && peak < 2.0, "{m:?} {c:?} peak {peak}");
        }
        for m in [
            MaterialKind::Steel,
            MaterialKind::Timber,
            MaterialKind::Concrete,
            MaterialKind::Composite,
        ] {
            assert!(impact_voice(m, 0.1, 5.0, 48_000.0, 1).is_none());
            let (out, ended) = render(impact_voice(m, 5.0, 5.0, 48_000.0, 2).unwrap(), 4.0);
            assert!(ended && out.iter().all(|x| x.is_finite()));
        }
    }
}
