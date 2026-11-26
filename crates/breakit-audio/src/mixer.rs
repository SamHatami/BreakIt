//! Voices, buses, positioning, the hall reverb and the output limiter.
//!
//! The mixer owns everything that sounds: a voice per straining member, the
//! rig and ambience voices keyed by the game's ids, and a capped pool of
//! one-shots (damage, breaks, impacts, UI). [`Mixer::render`] runs on the
//! audio thread; [`Mixer::handle`] applies a [`Command`] between blocks.

use crate::ambience::AmbientVoice;
use crate::dsp::{soft_clip, OnePole};
use crate::material;
use crate::material::StrainVoice;
use crate::reverb::Hall;
use crate::rigs::RigVoice;
use crate::ui::ui_voice;
use crate::{AmbientKind, BoxVoice, Bus, Command, Place, Pos, RigKind, Voice};

/// Samples processed per internal block. Positions and gains are updated
/// once per block and ramped across it.
pub const BLOCK: usize = 128;

/// Most one-shots alive at once. New ones replace the quietest.
pub const MAX_ONE_SHOTS: usize = 48;

/// Impacts allowed per 50 ms window, so a collapse does not turn into a wall
/// of identical knocks.
const IMPACTS_PER_WINDOW: u32 = 6;

/// A rig not updated for this long (seconds) fades out.
const RIG_TIMEOUT: f32 = 0.5;

/// Most members heard straining at once.
pub const MAX_STRAIN_VOICES: usize = 8;

/// A straining member not updated for this long (seconds) holds its load
/// (and so goes quiet); after `STRAIN_DROP` it is dropped.
const STRAIN_STALE: f32 = 0.25;
const STRAIN_DROP: f32 = 1.5;

#[derive(Clone, Copy, Debug)]
struct Listener {
    position: Pos,
    right: Pos,
}

impl Default for Listener {
    fn default() -> Self {
        Self {
            position: [0.0, 2.0, 10.0],
            right: [1.0, 0.0, 0.0],
        }
    }
}

/// Gains a positioned voice gets this block.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct Gains {
    l: f32,
    r: f32,
    send: f32,
    /// Air absorption low-pass cutoff, Hz.
    air: f32,
}

/// Distance attenuation, panning and reverb send for one source.
fn spatial(listener: &Listener, place: Option<&Place>, ref_distance: f32) -> Gains {
    let Some(p) = place else {
        let g = std::f32::consts::FRAC_1_SQRT_2;
        return Gains {
            l: g,
            r: g,
            send: 0.25,
            air: 20_000.0,
        };
    };
    let d = [
        p.position[0] - listener.position[0],
        p.position[1] - listener.position[1],
        p.position[2] - listener.position[2],
    ];
    let units = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
    let metres = units * p.metres_per_unit;
    // Inverse distance past the reference, never louder than at it.
    let att = ref_distance / ref_distance.max(metres);
    let side = if units > 1e-4 {
        (d[0] * listener.right[0] + d[1] * listener.right[1] + d[2] * listener.right[2]) / units
    } else {
        0.0
    };
    // Close sources spread over both ears instead of snapping hard to one.
    let pan = side.clamp(-1.0, 1.0) * (metres / (metres + 0.5));
    let angle = (pan + 1.0) * std::f32::consts::FRAC_PI_4;
    // In a hall the reverb level hardly falls with distance while the direct
    // sound does, so far sources get relatively more of it.
    let send = (0.12 + 0.55 * (1.0 - att)) * att.sqrt();
    Gains {
        l: angle.cos() * att,
        r: angle.sin() * att,
        send,
        air: (20_000.0 / (1.0 + metres / 25.0)).max(1_500.0),
    }
}

/// A mono voice and everything needed to place it.
struct Channel {
    gains: Gains,
    air: OnePole,
    prev: Gains,
}

impl Channel {
    fn new(sr: f32) -> Self {
        Self {
            gains: Gains::default(),
            air: OnePole::new(20_000.0, sr),
            prev: Gains::default(),
        }
    }

    /// Mixes `mono` into the dry and reverb buffers with gains ramped from
    /// last block's.
    fn mix(&mut self, mono: &[f32], gain: f32, sr: f32, dry: &mut [[f32; 2]], wet: &mut [f32]) {
        self.air.set_cutoff(self.gains.air, sr);
        let n = mono.len() as f32;
        let (p, g) = (self.prev, self.gains);
        for (i, &x) in mono.iter().enumerate() {
            let t = (i as f32 + 1.0) / n;
            // Offset: see the reverb; keeps the filter out of subnormals.
            let x = self.air.lp(x + 1e-12) * gain;
            dry[i][0] += x * (p.l + (g.l - p.l) * t);
            dry[i][1] += x * (p.r + (g.r - p.r) * t);
            wet[i] += x * (p.send + (g.send - p.send) * t);
        }
        self.prev = g;
    }
}

struct OneShot {
    voice: BoxVoice,
    bus: Bus,
    place: Option<Place>,
    gain: f32,
    ref_distance: f32,
    channel: Channel,
    /// Running peak of the output, used to pick a voice to steal.
    loudness: f32,
}

struct Strain {
    id: u32,
    voice: StrainVoice,
    place: Place,
    channel: Channel,
    /// Seconds since the game last updated it.
    idle: f32,
}

struct Rig {
    id: u32,
    kind: RigKind,
    voice: RigVoice,
    place: Place,
    channel: Channel,
    idle: f32,
    fade: f32,
}

struct Ambient {
    id: u32,
    kind: AmbientKind,
    /// Two decorrelated voices for a wide bed, one for a positioned source.
    voices: Vec<AmbientVoice>,
    place: Option<Place>,
    channel: Channel,
}

/// The audio engine's state. Build it with the device's sample rate, feed it
/// commands, pull samples.
pub struct Mixer {
    sr: f32,
    listener: Listener,
    strains: Vec<Strain>,
    rigs: Vec<Rig>,
    ambient: Vec<Ambient>,
    shots: Vec<OneShot>,
    reverb: Hall,
    bus_gain: [f32; 4],
    master: f32,
    seed: u32,
    impact_window: (u32, u32),
    /// Sample counter.
    clock: u64,
    limiter: f32,
    // Scratch, reused every block.
    mono: Vec<f32>,
    mono2: Vec<f32>,
    dry: Vec<[f32; 2]>,
    wet: Vec<f32>,
    wet_out: Vec<[f32; 2]>,
    pending: Vec<[f32; 2]>,
    pending_pos: usize,
}

impl Mixer {
    pub fn new(sample_rate: f32) -> Self {
        let sr = sample_rate;
        Self {
            sr,
            listener: Listener::default(),
            strains: Vec::with_capacity(MAX_STRAIN_VOICES),
            rigs: Vec::new(),
            ambient: Vec::new(),
            shots: Vec::with_capacity(MAX_ONE_SHOTS),
            reverb: Hall::new(sr),
            bus_gain: [1.0, 0.8, 0.6, 0.7],
            master: 0.8,
            seed: 1,
            impact_window: (0, 0),
            clock: 0,
            limiter: 1.0,
            mono: vec![0.0; BLOCK],
            mono2: vec![0.0; BLOCK],
            dry: vec![[0.0; 2]; BLOCK],
            wet: vec![0.0; BLOCK],
            wet_out: vec![[0.0; 2]; BLOCK],
            pending: vec![[0.0; 2]; BLOCK],
            pending_pos: BLOCK,
        }
    }

    pub fn sample_rate(&self) -> f32 {
        self.sr
    }

    /// Number of one-shots currently sounding.
    pub fn active_one_shots(&self) -> usize {
        self.shots.len()
    }

    fn next_seed(&mut self) -> u32 {
        self.seed = self.seed.wrapping_add(0x9E37_79B9);
        self.seed
    }

    /// Applies one command. Call between `render` calls (the output stream
    /// does this for you).
    pub fn handle(&mut self, cmd: Command) {
        let sr = self.sr;
        match cmd {
            Command::Listener {
                position, right, ..
            } => {
                self.listener = Listener { position, right };
            }
            Command::Strain {
                id,
                body,
                utilization,
                damage,
                place,
            } => {
                if let Some(s) = self.strains.iter_mut().find(|s| s.id == id) {
                    s.voice
                        .set(&body, utilization, damage, s.idle > STRAIN_STALE);
                    s.place = place;
                    s.idle = 0.0;
                    return;
                }
                if self.strains.len() >= MAX_STRAIN_VOICES {
                    // Make room by dropping the one longest without news,
                    // if it has finished speaking.
                    let stale = self
                        .strains
                        .iter()
                        .enumerate()
                        .filter(|(_, s)| s.idle > 0.05 && s.voice.quiet())
                        .max_by(|a, b| a.1.idle.total_cmp(&b.1.idle))
                        .map(|(i, _)| i);
                    match stale {
                        Some(i) => {
                            self.strains.swap_remove(i);
                        }
                        None => return,
                    }
                }
                let seed = self.next_seed();
                if let Some(mut voice) = StrainVoice::new(&body, sr, seed) {
                    voice.set(&body, utilization, damage, true);
                    self.strains.push(Strain {
                        id,
                        voice,
                        place,
                        channel: Channel::new(sr),
                        idle: 0.0,
                    });
                }
            }
            Command::Damage {
                body,
                state,
                energy,
                place,
            } => {
                let seed = self.next_seed();
                if let Some(v) = material::damage_voice(&body, state, energy, sr, seed) {
                    self.play(v, Bus::Structure, Some(place), 1.0, 3.0);
                }
            }
            Command::Break { info, place } => {
                let seed = self.next_seed();
                if let Some(v) = material::break_voice(&info, sr, seed) {
                    self.play(v, Bus::Structure, Some(place), 1.0, 4.0);
                }
            }
            Command::Impact {
                body,
                speed,
                mass,
                place,
            } => {
                let window = (self.clock / (sr as u64 / 20).max(1)) as u32;
                if self.impact_window.0 != window {
                    self.impact_window = (window, 0);
                }
                if self.impact_window.1 >= IMPACTS_PER_WINDOW {
                    return;
                }
                let seed = self.next_seed();
                if let Some(v) = material::impact_voice(&body, speed, mass, sr, seed) {
                    self.impact_window.1 += 1;
                    self.play(v, Bus::Structure, Some(place), 1.0, 3.0);
                }
            }
            Command::Rig {
                id,
                kind,
                drive,
                speed,
                place,
            } => {
                if let Some(r) = self.rigs.iter_mut().find(|r| r.id == id && r.kind == kind) {
                    r.voice.set(drive, speed);
                    r.place = place;
                    r.idle = 0.0;
                } else {
                    let seed = self.next_seed();
                    let mut voice = RigVoice::new(kind, sr, seed);
                    voice.set(drive, speed);
                    self.rigs.push(Rig {
                        id,
                        kind,
                        voice,
                        place,
                        channel: Channel::new(sr),
                        idle: 0.0,
                        fade: 0.0,
                    });
                }
            }
            Command::Ambient {
                id,
                kind,
                level,
                place,
            } => {
                if let Some(a) = self
                    .ambient
                    .iter_mut()
                    .find(|a| a.id == id && a.kind == kind)
                {
                    for v in &mut a.voices {
                        v.set_level(level);
                    }
                    a.place = place;
                } else {
                    let count = if place.is_some() { 1 } else { 2 };
                    let voices = (0..count)
                        .map(|_| {
                            let seed = self.next_seed();
                            let mut v = AmbientVoice::new(kind, sr, seed);
                            v.set_level(level);
                            v
                        })
                        .collect();
                    self.ambient.push(Ambient {
                        id,
                        kind,
                        voices,
                        place,
                        channel: Channel::new(sr),
                    });
                }
            }
            Command::Ui(sound) => {
                let seed = self.next_seed();
                let v = ui_voice(sound, sr, seed);
                self.play(v, Bus::Ui, None, 1.0, 1.0);
            }
            Command::Volume { bus, gain } => self.bus_gain[bus.index()] = gain.clamp(0.0, 2.0),
            Command::Master(g) => self.master = g.clamp(0.0, 2.0),
            Command::ResetStructure => {
                self.strains.clear();
                self.shots.retain(|s| s.bus != Bus::Structure);
            }
        }
    }

    /// Starts a one-shot. `ref_distance` (m) is how close you must be to hear
    /// it at full level.
    pub fn play(
        &mut self,
        voice: BoxVoice,
        bus: Bus,
        place: Option<Place>,
        gain: f32,
        ref_distance: f32,
    ) {
        if self.shots.len() >= MAX_ONE_SHOTS {
            // Steal the quietest structure or UI voice.
            if let Some((i, _)) = self
                .shots
                .iter()
                .enumerate()
                .min_by(|a, b| a.1.loudness.total_cmp(&b.1.loudness))
            {
                self.shots.swap_remove(i);
            }
        }
        let mut channel = Channel::new(self.sr);
        channel.gains = spatial(&self.listener, place.as_ref(), ref_distance);
        channel.prev = channel.gains;
        self.shots.push(OneShot {
            voice,
            bus,
            place,
            gain,
            ref_distance,
            channel,
            loudness: f32::INFINITY,
        });
    }

    /// Fills an interleaved buffer with `channels` channels (1 or more; only
    /// the first two carry sound, extra channels are silent).
    pub fn render(&mut self, out: &mut [f32], channels: usize) {
        let channels = channels.max(1);
        for frame in out.chunks_mut(channels) {
            if self.pending_pos >= BLOCK {
                self.render_block();
                self.pending_pos = 0;
            }
            let [l, r] = self.pending[self.pending_pos];
            self.pending_pos += 1;
            if channels == 1 {
                frame[0] = 0.5 * (l + r);
            } else {
                frame[0] = l;
                frame[1] = r;
                for c in frame.iter_mut().skip(2) {
                    *c = 0.0;
                }
            }
        }
    }

    /// Renders `seconds` of stereo audio into a new interleaved buffer. Handy
    /// for tests and offline renders.
    pub fn render_seconds(&mut self, seconds: f32) -> Vec<f32> {
        let mut out = vec![0.0; (seconds * self.sr) as usize * 2];
        self.render(&mut out, 2);
        out
    }

    fn render_block(&mut self) {
        let sr = self.sr;
        let n = BLOCK;
        let dt = n as f32 / sr;
        for s in self.dry.iter_mut() {
            *s = [0.0; 2];
        }
        self.wet.iter_mut().for_each(|x| *x = 0.0);
        let listener = self.listener;
        let bus = self.bus_gain;

        // Straining members.
        self.strains.retain_mut(|s| {
            s.idle += dt;
            if s.idle > STRAIN_DROP && s.voice.quiet() {
                return false;
            }
            self.mono.iter_mut().for_each(|x| *x = 0.0);
            s.voice.render(&mut self.mono);
            s.channel.gains = spatial(&listener, Some(&s.place), 4.0);
            s.channel.mix(
                &self.mono,
                bus[Bus::Structure.index()],
                sr,
                &mut self.dry,
                &mut self.wet,
            );
            true
        });

        // Rigs, fading out the ones the game stopped updating.
        self.rigs.retain_mut(|r| {
            r.idle += dt;
            let target = if r.idle > RIG_TIMEOUT { 0.0 } else { 1.0 };
            let start = r.fade;
            r.fade = if target > r.fade {
                (r.fade + dt / 0.05).min(1.0)
            } else {
                (r.fade - dt / 0.4).max(0.0)
            };
            if r.idle > RIG_TIMEOUT && r.fade <= 0.0 {
                return false;
            }
            self.mono.iter_mut().for_each(|x| *x = 0.0);
            r.voice.render(&mut self.mono);
            for (i, x) in self.mono.iter_mut().enumerate() {
                *x *= start + (r.fade - start) * (i as f32 + 1.0) / n as f32;
            }
            r.channel.gains = spatial(&listener, Some(&r.place), 4.0);
            r.channel.mix(
                &self.mono,
                bus[Bus::Rigs.index()],
                sr,
                &mut self.dry,
                &mut self.wet,
            );
            true
        });

        // Ambience.
        for a in &mut self.ambient {
            let g = bus[Bus::Ambience.index()];
            if a.place.is_some() || a.voices.len() < 2 {
                self.mono.iter_mut().for_each(|x| *x = 0.0);
                for v in &mut a.voices {
                    v.render(&mut self.mono);
                }
                a.channel.gains = spatial(&listener, a.place.as_ref(), 4.0);
                a.channel
                    .mix(&self.mono, g, sr, &mut self.dry, &mut self.wet);
            } else {
                // Wide bed: one voice per ear, a little of it into the hall.
                self.mono.iter_mut().for_each(|x| *x = 0.0);
                self.mono2.iter_mut().for_each(|x| *x = 0.0);
                a.voices[0].render(&mut self.mono);
                a.voices[1].render(&mut self.mono2);
                for i in 0..n {
                    let (l, r) = (self.mono[i] * g, self.mono2[i] * g);
                    self.dry[i][0] += l * 0.8 + r * 0.2;
                    self.dry[i][1] += r * 0.8 + l * 0.2;
                    self.wet[i] += (l + r) * 0.1;
                }
            }
        }

        // One-shots.
        let (mono, dry, wet) = (&mut self.mono, &mut self.dry, &mut self.wet);
        self.shots.retain_mut(|s| {
            mono.iter_mut().for_each(|x| *x = 0.0);
            let alive = s.voice.render(mono);
            let peak = mono.iter().fold(0.0f32, |m, x| m.max(x.abs()));
            s.loudness = if s.loudness.is_finite() {
                s.loudness.max(peak) * 0.9 + peak * 0.1
            } else {
                peak
            };
            s.channel.gains = spatial(&listener, s.place.as_ref(), s.ref_distance);
            if s.bus == Bus::Ui {
                // Interface sounds sit in the head, almost dry.
                s.channel.gains.send = 0.06;
                s.channel.gains.air = 20_000.0;
            }
            s.channel
                .mix(mono, s.gain * bus[s.bus.index()], sr, dry, wet);
            alive
        });

        // Hall reverb.
        self.reverb.process(&self.wet, &mut self.wet_out);

        // Master and limiter: fast attack, 150 ms release, then a soft clip
        // for anything the block-rate envelope misses.
        let release = (-dt / 0.15).exp();
        let mut peak = 0.0f32;
        for i in 0..n {
            let l = (self.dry[i][0] + self.wet_out[i][0]) * self.master;
            let r = (self.dry[i][1] + self.wet_out[i][1]) * self.master;
            self.pending[i] = [l, r];
            peak = peak.max(l.abs()).max(r.abs());
        }
        let target = if peak > 0.8 { 0.8 / peak } else { 1.0 };
        let start = self.limiter;
        self.limiter = if target < self.limiter {
            target
        } else {
            1.0 - (1.0 - self.limiter) * release
        };
        for (i, f) in self.pending.iter_mut().enumerate() {
            let t = (i as f32 + 1.0) / n as f32;
            // Gain falling: drop at once. Rising: ramp across the block.
            let g = if self.limiter < start {
                self.limiter
            } else {
                start + (self.limiter - start) * t
            };
            f[0] = limit(f[0] * g);
            f[1] = limit(f[1] * g);
            if !f[0].is_finite() || !f[1].is_finite() {
                *f = [0.0; 2];
            }
        }
        self.clock += n as u64;
    }
}

/// Linear below 0.7, then a soft knee that never exceeds 1.
#[inline]
fn limit(x: f32) -> f32 {
    let a = x.abs();
    if a <= 0.7 {
        x
    } else {
        x.signum() * (0.7 + 0.3 * soft_clip((a - 0.7) / 0.3))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Body, BreakCause, BreakInfo, MaterialKind, UiSound};

    fn peak(buf: &[f32]) -> f32 {
        buf.iter().fold(0.0f32, |m, x| m.max(x.abs()))
    }

    #[test]
    fn silent_until_told_otherwise() {
        let mut m = Mixer::new(48_000.0);
        let out = m.render_seconds(0.5);
        assert!(peak(&out) < 1e-6);
    }

    #[test]
    fn every_command_renders_finite_bounded_audio() {
        let mut m = Mixer::new(48_000.0);
        let place = Place::at([1.0, 1.0, 0.0]);
        let materials = [
            MaterialKind::Steel,
            MaterialKind::Timber,
            MaterialKind::Concrete,
            MaterialKind::Composite,
        ];
        for (i, mat) in materials.iter().enumerate() {
            let body = Body::typical(*mat, 3.0);
            for (u, d) in [(0.2, 0.2), (1.2, 0.9)] {
                m.handle(Command::Strain {
                    id: i as u32,
                    body,
                    utilization: u,
                    damage: d,
                    place,
                });
            }
            m.handle(Command::Damage {
                body,
                state: crate::DamageState::Yielded,
                energy: 200.0,
                place,
            });
            m.handle(Command::Break {
                info: BreakInfo {
                    body,
                    cause: BreakCause::Brittle,
                    at: 0.5,
                    energy: 800.0,
                    axial: 0.3,
                },
                place,
            });
            m.handle(Command::Impact {
                body: Body::fragment(*mat, 0.3),
                speed: 4.0,
                mass: 10.0,
                place,
            });
            m.handle(Command::Rig {
                id: i as u32,
                kind: [
                    RigKind::WindFan,
                    RigKind::ShakeTable,
                    RigKind::WavePaddle,
                    RigKind::Cart,
                ][i],
                drive: 0.8,
                speed: 1.0,
                place,
            });
        }
        m.handle(Command::Ambient {
            id: 0,
            kind: AmbientKind::Hvac,
            level: 1.0,
            place: None,
        });
        m.handle(Command::Ui(UiSound::PlaceBeam));
        let out = m.render_seconds(2.0);
        assert!(out.iter().all(|x| x.is_finite()));
        let p = peak(&out);
        assert!(p > 1e-3, "nothing audible: {p}");
        assert!(p <= 1.2, "limiter let {p} through");
    }

    #[test]
    fn rigs_fade_out_when_no_longer_updated() {
        let mut m = Mixer::new(48_000.0);
        let place = Place::at([0.0, 0.0, 0.0]);
        m.handle(Command::Rig {
            id: 7,
            kind: RigKind::WindFan,
            drive: 1.0,
            speed: 20.0,
            place,
        });
        m.render_seconds(0.2);
        assert_eq!(m.rigs.len(), 1);
        m.render_seconds(1.5);
        assert!(m.rigs.is_empty());
    }

    #[test]
    fn far_sources_are_quieter_and_wetter() {
        let l = Listener::default();
        let near = spatial(&l, Some(&Place::at([0.0, 2.0, 7.0])), 3.0);
        let far = spatial(&l, Some(&Place::at([0.0, 2.0, -40.0])), 3.0);
        assert!(far.l < near.l * 0.2);
        assert!(far.send / far.l > near.send / near.l);
        let right = spatial(&l, Some(&Place::at([10.0, 2.0, 10.0])), 3.0);
        assert!(right.r > right.l * 3.0);
    }

    #[test]
    fn strain_voices_are_capped_and_dropped_when_stale() {
        let mut m = Mixer::new(48_000.0);
        let body = Body::typical(MaterialKind::Timber, 3.0);
        let place = Place::at([0.0, 1.0, 0.0]);
        for id in 0..20 {
            m.handle(Command::Strain {
                id,
                body,
                utilization: 0.5,
                damage: 0.5,
                place,
            });
        }
        assert_eq!(m.strains.len(), MAX_STRAIN_VOICES);
        m.render_seconds(2.5);
        assert!(m.strains.is_empty());
        m.handle(Command::Strain {
            id: 3,
            body,
            utilization: 0.5,
            damage: 0.5,
            place,
        });
        m.handle(Command::ResetStructure);
        assert!(m.strains.is_empty());
    }

    /// Worst case the mixer allows: every one-shot slot ringing a break and
    /// every strain voice working. Run with
    /// `cargo test --release -p breakit-audio -- --ignored --nocapture cost`.
    #[test]
    #[ignore = "timing; run by hand in release"]
    fn cost_of_a_full_mixer() {
        let sr = 48_000.0;
        let mut m = Mixer::new(sr);
        let place = Place::at([0.0, 1.0, 0.0]);
        let steel = Body::typical(MaterialKind::Steel, 6.0);
        let seconds = 4.0;
        let steps = (seconds / 0.05) as usize;
        let t = std::time::Instant::now();
        for step in 0..steps {
            // Keep all 48 one-shots busy with long-ringing steel breaks.
            for _ in 0..MAX_ONE_SHOTS / 8 {
                m.handle(Command::Break {
                    info: BreakInfo {
                        body: steel,
                        cause: BreakCause::Rupture,
                        at: 0.5,
                        energy: 1500.0,
                        axial: 0.5,
                    },
                    place,
                });
            }
            for id in 0..MAX_STRAIN_VOICES as u32 {
                let u = step as f32 / steps as f32;
                m.handle(Command::Strain {
                    id,
                    body: Body::typical(MaterialKind::Timber, 3.0),
                    utilization: u,
                    damage: u,
                    place,
                });
            }
            let out = m.render_seconds(0.05);
            assert!(out.iter().all(|x| x.is_finite()));
        }
        let cost = t.elapsed().as_secs_f32() / seconds;
        println!(
            "full mixer: {} one-shots, {:.1}% of one core",
            m.active_one_shots(),
            cost * 100.0
        );
        assert_eq!(m.active_one_shots(), MAX_ONE_SHOTS);
    }
}
