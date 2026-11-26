//! The test hall's ambience: room tone, air handling, mains hum and distant
//! machinery.
//!
//! Every source is quiet (rms about 0.01-0.03 at level 1) and built from
//! slow random processes and incommensurate modulations, so loops never
//! repeat audibly. Voices are dry and mono; the mixer adds the hall reverb
//! and, for wide beds, plays two voices with different seeds.

use std::f32::consts::TAU;

use crate::dsp::{Brown, ModalBank, OnePole, Osc, Pink, Rng, Smooth, Svf};
use crate::{AmbientKind, Voice};

/// Tiny offset fed into filters and resonators that can fall silent, so
/// their state never decays into (very slow) subnormal floats.
pub(crate) const ANTI_DENORMAL: f32 = 1e-15;

/// Steps a smoother and snaps it onto its target once there, so a value
/// gliding to zero does not end up computing with subnormals.
#[inline]
pub(crate) fn snap(s: &mut Smooth) -> f32 {
    let v = s.next();
    if (v - s.target).abs() < 1e-7 {
        s.value = s.target;
    }
    s.value
}

/// A slow random wander in [-1, 1]: a new random target every few seconds,
/// approached smoothly. Used for drift that must not sound periodic.
#[derive(Clone, Debug)]
struct Drift {
    value: Smooth,
    left: f32,
    min_hold: f32,
    max_hold: f32,
}

impl Drift {
    fn new(min_hold: f32, max_hold: f32, rng: &mut Rng, sr: f32) -> Self {
        let mut d = Self {
            value: Smooth::new(rng.bipolar(), (min_hold + max_hold) * 0.3, sr),
            left: 0.0,
            min_hold,
            max_hold,
        };
        d.value.target = rng.bipolar();
        d
    }

    #[inline]
    fn next(&mut self, rng: &mut Rng, dt: f32) -> f32 {
        self.left -= dt;
        if self.left <= 0.0 {
            self.left = rng.range(self.min_hold, self.max_hold);
            self.value.target = rng.bipolar();
        }
        snap(&mut self.value)
    }
}

struct Room {
    air: Brown,
    air_lp: OnePole,
    air_drift: Drift,
    traffic: Brown,
    traffic_lp: OnePole,
    traffic_swell: Drift,
    tick: ModalBank,
    tick_wait: f32,
}

struct Hvac {
    noise: Pink,
    band: Svf,
    hp: OnePole,
    lp: OnePole,
    drift: Drift,
    flutter: Drift,
    fan: Osc,
    fan2: Osc,
    fan_freq: f32,
    wobble: Drift,
    lfo: [f32; 2],
}

struct Hum {
    partials: [Osc; 5],
    beat: Osc,
    flicker: Drift,
    buzz_phase: f32,
    buzz_band: Svf,
    buzz_drift: Drift,
}

struct Machinery {
    // Compressor: cycles on and off.
    on: bool,
    state_left: f32,
    run: Smooth,
    motor: Osc,
    motor_freq: f32,
    rumble: Brown,
    motor_lp: OnePole,
    // Far clanks.
    clank: ModalBank,
    clank_wait: f32,
    clank_lp: OnePole,
    // Forklift reverse beeps.
    beep: Osc,
    beeps_left: u32,
    beep_t: f32,
    beep_wait: f32,
    beep_lp: OnePole,
}

enum Patch {
    Room(Room),
    Hvac(Hvac),
    Hum(Hum),
    Machinery(Machinery),
}

/// One ambient source, mono.
pub struct AmbientVoice {
    #[allow(dead_code)]
    kind: AmbientKind,
    sr: f32,
    dt: f32,
    rng: Rng,
    level: Smooth,
    patch: Patch,
}

impl AmbientVoice {
    pub fn new(kind: AmbientKind, sr: f32, seed: u32) -> Self {
        let mut rng = Rng::new(seed ^ 0xA3B1_0001);
        let patch = match kind {
            AmbientKind::Room => Patch::Room(Room {
                air: Brown::new(rng.next_u32()),
                air_lp: OnePole::new(300.0, sr),
                air_drift: Drift::new(3.0, 9.0, &mut rng, sr),
                traffic: Brown::new(rng.next_u32()),
                traffic_lp: OnePole::new(70.0, sr),
                traffic_swell: Drift::new(8.0, 30.0, &mut rng, sr),
                tick: ModalBank::default(),
                tick_wait: rng.range(3.0, 15.0),
            }),
            AmbientKind::Hvac => {
                let fan_freq = rng.range(105.0, 155.0);
                Patch::Hvac(Hvac {
                    noise: Pink::new(rng.next_u32()),
                    band: Svf::new(420.0, 0.5, sr),
                    hp: OnePole::new(80.0, sr),
                    lp: OnePole::new(2_000.0, sr),
                    drift: Drift::new(4.0, 14.0, &mut rng, sr),
                    flutter: Drift::new(0.3, 1.2, &mut rng, sr),
                    fan: Osc::new(rng.unit()),
                    fan2: Osc::new(rng.unit()),
                    fan_freq,
                    wobble: Drift::new(1.5, 5.0, &mut rng, sr),
                    lfo: [rng.unit(), rng.unit()],
                })
            }
            AmbientKind::ElectricalHum => Patch::Hum(Hum {
                partials: [
                    Osc::new(rng.unit()),
                    Osc::new(rng.unit()),
                    Osc::new(rng.unit()),
                    Osc::new(rng.unit()),
                    Osc::new(rng.unit()),
                ],
                beat: Osc::new(rng.unit()),
                flicker: Drift::new(0.5, 4.0, &mut rng, sr),
                buzz_phase: rng.unit(),
                buzz_band: Svf::new(rng.range(2_400.0, 3_600.0), 2.0, sr),
                buzz_drift: Drift::new(2.0, 10.0, &mut rng, sr),
            }),
            AmbientKind::Machinery => Patch::Machinery(Machinery {
                on: false,
                state_left: rng.range(4.0, 25.0),
                run: Smooth::new(0.0, 0.6, sr),
                motor: Osc::new(rng.unit()),
                motor_freq: rng.range(27.0, 33.0),
                rumble: Brown::new(rng.next_u32()),
                motor_lp: OnePole::new(500.0, sr),
                clank: ModalBank::default(),
                clank_wait: rng.range(6.0, 30.0),
                clank_lp: OnePole::new(1_400.0, sr),
                beep: Osc::new(0.0),
                beeps_left: 0,
                beep_t: 0.0,
                beep_wait: rng.range(30.0, 90.0),
                beep_lp: OnePole::new(1_600.0, sr),
            }),
        };
        Self {
            kind,
            sr,
            dt: 1.0 / sr,
            rng,
            level: Smooth::new(0.0, 0.5, sr),
            patch,
        }
    }

    /// Level 0..1, smoothed over about half a second.
    pub fn set_level(&mut self, level: f32) {
        self.level.target = level.clamp(0.0, 2.0);
    }
}

impl Voice for AmbientVoice {
    fn render(&mut self, out: &mut [f32]) -> bool {
        if self.level.target < 1e-5 && self.level.value < 1e-5 {
            self.level.value = 0.0;
            return true;
        }
        let (sr, dt) = (self.sr, self.dt);
        let rng = &mut self.rng;
        match &mut self.patch {
            Patch::Room(p) => {
                for o in out.iter_mut() {
                    let lvl = snap(&mut self.level);
                    let drift = 0.85 + 0.15 * p.air_drift.next(rng, dt);
                    let air = p.air_lp.lp(p.air.next()) * 0.014 * drift;
                    let swell = (0.5 + 0.5 * p.traffic_swell.next(rng, dt)).powi(2);
                    let traffic = p.traffic_lp.lp(p.traffic.next()) * 0.025 * swell;
                    // Cladding ticks: thermal pings from the steel skin.
                    p.tick_wait -= dt;
                    let mut x = 0.0;
                    if p.tick_wait <= 0.0 {
                        p.tick_wait = rng.range(4.0, 22.0);
                        let f = rng.range(900.0, 2_600.0);
                        p.tick = ModalBank::new(
                            &[(f, 0.25, 1.0), (f * 2.31, 0.15, 0.5), (f * 3.7, 0.08, 0.25)],
                            sr,
                        );
                        x = rng.range(0.02, 0.06);
                    }
                    let tick = if p.tick.is_empty() {
                        0.0
                    } else {
                        p.tick.process(x + ANTI_DENORMAL)
                    };
                    *o += (air + traffic + tick) * lvl;
                }
            }
            Patch::Hvac(p) => {
                let w1 = 1.0 / 13.7;
                let w2 = 1.0 / 29.3;
                for (i, o) in out.iter_mut().enumerate() {
                    let lvl = snap(&mut self.level);
                    p.lfo[0] = (p.lfo[0] + w1 * dt).fract();
                    p.lfo[1] = (p.lfo[1] + w2 * dt).fract();
                    let slow = 0.6 * (p.lfo[0] * TAU).sin() + 0.4 * (p.lfo[1] * TAU).sin();
                    let drift = p.drift.next(rng, dt);
                    let flutter = p.flutter.next(rng, dt);
                    if i % 32 == 0 {
                        p.band
                            .set(420.0 * (1.0 + 0.12 * slow + 0.08 * drift), 0.5, sr);
                    }
                    let n = p.noise.next();
                    let air = p.lp.lp(p.hp.hp(p.band.bp(n) * 1.6 + n * 0.25));
                    let gain = 0.9 + 0.12 * slow + 0.06 * drift + 0.04 * flutter;
                    let wob = 1.0 + 0.004 * p.wobble.next(rng, dt);
                    let fan = p.fan.sine(p.fan_freq * wob, sr) * 0.006
                        + p.fan2.sine(p.fan_freq * 2.0 * wob, sr) * 0.0025;
                    *o += (air * 0.05 * gain + fan * (0.8 + 0.2 * slow)) * lvl;
                }
            }
            Patch::Hum(p) => {
                // 50 Hz mains and harmonics; one 100 Hz partial slightly off
                // (a second transformer) beats against the other.
                const F: [(f32, f32); 5] = [
                    (50.0, 0.004),
                    (100.0, 0.009),
                    (150.0, 0.003),
                    (200.0, 0.0025),
                    (250.0, 0.001),
                ];
                for o in out.iter_mut() {
                    let lvl = snap(&mut self.level);
                    let flick = 1.0 + 0.08 * p.flicker.next(rng, dt);
                    let mut x = 0.0;
                    for (osc, &(f, a)) in p.partials.iter_mut().zip(F.iter()) {
                        x += osc.sine(f, sr) * a;
                    }
                    x += p.beat.sine(100.13, sr) * 0.006;
                    // Ballast buzz: a 100 Hz pulse train, band-passed.
                    p.buzz_phase += 100.0 * dt;
                    let pulse = if p.buzz_phase >= 1.0 {
                        p.buzz_phase -= 1.0;
                        1.0
                    } else {
                        0.0
                    };
                    let bz = p.buzz_band.bp(pulse + rng.bipolar() * 0.02);
                    let bz_gain = 0.006 * (0.6 + 0.4 * p.buzz_drift.next(rng, dt));
                    *o += (x * flick + bz * bz_gain) * lvl;
                }
            }
            Patch::Machinery(p) => {
                for o in out.iter_mut() {
                    let lvl = snap(&mut self.level);
                    p.state_left -= dt;
                    if p.state_left <= 0.0 {
                        p.on = !p.on;
                        p.state_left = if p.on {
                            rng.range(10.0, 30.0)
                        } else {
                            rng.range(18.0, 60.0)
                        };
                        p.run.target = if p.on { 1.0 } else { 0.0 };
                        p.motor_freq = rng.range(27.0, 33.0);
                    }
                    let run = snap(&mut p.run);
                    let mut x = 0.0;
                    if run > 1e-4 {
                        // Spins up / down: pitch follows the run envelope.
                        let f = p.motor_freq * (0.6 + 0.4 * run);
                        let saw = p.motor.saw(f, sr);
                        let body = p.motor_lp.lp(saw * 0.7 + p.rumble.next() * 0.3);
                        x += body * 0.035 * run;
                    }
                    // Far clank.
                    p.clank_wait -= dt;
                    let mut hit = 0.0;
                    if p.clank_wait <= 0.0 {
                        p.clank_wait = rng.range(15.0, 45.0);
                        let f = rng.range(150.0, 260.0);
                        p.clank = ModalBank::new(
                            &[
                                (f, 0.7, 1.0),
                                (f * 2.17, 0.5, 0.6),
                                (f * 3.93, 0.4, 0.4),
                                (f * 5.6, 0.3, 0.25),
                            ],
                            sr,
                        );
                        hit = rng.range(0.03, 0.07);
                    }
                    if !p.clank.is_empty() {
                        x += p.clank_lp.lp(p.clank.process(hit + ANTI_DENORMAL));
                    }
                    // Forklift reversing, far away.
                    p.beep_wait -= dt;
                    if p.beep_wait <= 0.0 && p.beeps_left == 0 {
                        p.beeps_left = 3 + (rng.next_u32() % 4);
                        p.beep_t = 0.0;
                        p.beep_wait = rng.range(45.0, 120.0);
                    }
                    if p.beeps_left > 0 {
                        p.beep_t += dt;
                        let on = p.beep_t < 0.4;
                        let s = p.beep.sine(1_040.0, sr);
                        let env = if on {
                            (p.beep_t / 0.01).min(1.0) * ((0.4 - p.beep_t) / 0.01).min(1.0)
                        } else {
                            0.0
                        };
                        x += p.beep_lp.lp(s * env * 0.004 + ANTI_DENORMAL);
                        if p.beep_t >= 0.8 {
                            p.beep_t = 0.0;
                            p.beeps_left -= 1;
                        }
                    }
                    *o += x * lvl;
                }
            }
        }
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rms(v: &[f32]) -> f32 {
        (v.iter().map(|x| x * x).sum::<f32>() / v.len() as f32).sqrt()
    }

    #[test]
    fn ambience_is_quiet_bounded_and_seeded() {
        let sr = 48_000.0;
        for kind in [
            AmbientKind::Room,
            AmbientKind::Hvac,
            AmbientKind::ElectricalHum,
            AmbientKind::Machinery,
        ] {
            let mut v = AmbientVoice::new(kind, sr, 5);
            v.set_level(1.0);
            let mut out = vec![0.0; (sr * 30.0) as usize];
            for chunk in out.chunks_mut(128) {
                v.render(chunk);
            }
            assert!(out.iter().all(|x| x.is_finite()));
            let peak = out.iter().fold(0.0f32, |m, x| m.max(x.abs()));
            let r = rms(&out[(sr * 2.0) as usize..]);
            println!("{kind:?}: rms {r:.4} peak {peak:.3}");
            assert!(peak < 0.3, "{kind:?} peak {peak}");
            assert!(r < 0.04, "{kind:?} rms {r}");
            assert!(r > 0.001, "{kind:?} silent: {r}");
        }
    }

    #[test]
    fn silent_at_zero_level() {
        let mut v = AmbientVoice::new(AmbientKind::Hvac, 48_000.0, 1);
        let mut out = vec![0.0; 4800];
        v.render(&mut out);
        assert!(out.iter().all(|&x| x == 0.0));
    }
}
