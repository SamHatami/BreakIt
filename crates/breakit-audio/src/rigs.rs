//! Test rig sounds: fan wall, shake table, wave paddle, carts, winch and the
//! hydraulic power unit.
//!
//! Each rig is driven by two smoothed controls (see [`crate::Command::Rig`]):
//! `drive` 0..1, how hard it works, and `speed`, its physical rate. A rig at
//! rest costs almost nothing: with both controls at zero it renders nothing.

use crate::ambience::{snap, ANTI_DENORMAL};
use crate::dsp::{Brown, Decay, ModalBank, OnePole, Osc, Pink, Resonator, Rng, Smooth, Svf};
use crate::{RigKind, Voice};

/// A low thump: a damped low resonator plus a short noise burst.
struct Thump {
    body: Resonator,
    noise: Decay,
    lp: OnePole,
    amp: f32,
}

impl Thump {
    fn new(freq: f32, t60: f32, lp: f32, sr: f32) -> Self {
        let mut noise = Decay::new(0.06, sr);
        noise.retrigger(0.0);
        Self {
            body: Resonator::new(freq, t60, sr),
            noise,
            lp: OnePole::new(lp, sr),
            amp: 0.0,
        }
    }

    fn hit(&mut self, amp: f32) -> f32 {
        self.amp = amp;
        self.noise.retrigger(1.0);
        amp
    }

    #[inline]
    fn next(&mut self, impulse: f32, rng: &mut Rng) -> f32 {
        let env = if self.noise.level() > 1e-6 {
            self.noise.next()
        } else {
            0.0
        };
        let n = self.lp.lp(rng.bipolar() * env + ANTI_DENORMAL) * self.amp * 0.6;
        self.body.process(impulse + n * 0.3 + ANTI_DENORMAL) + n
    }
}

struct Fan {
    blades: [Osc; 4],
    fan2: Osc,
    motor: Osc,
    noise: Pink,
    band: Svf,
    hp: OnePole,
    gust: OnePole,
}

struct Shake {
    pump: [Osc; 3],
    hiss: Svf,
    rumble: Brown,
    rumble_lp: OnePole,
    thump: Thump,
    sign: f32,
    peak: f32,
}

struct Paddle {
    hum: [Osc; 2],
    water: Pink,
    slosh: Svf,
    slosh_drift: OnePole,
    splash: Decay,
    splash_amp: f32,
    splash_band: Svf,
    bubbles: [(Osc, f32, f32, f32); 4], // osc, freq, env, rise
    sign: f32,
    peak: f32,
}

struct Cart {
    roll: Brown,
    roll_lp: OnePole,
    hiss: Svf,
    joint: ModalBank,
    joint_thud: Resonator,
    travel: f32,
    second_axle: f32,
    rattle: ModalBank,
}

struct Winch {
    motor: Osc,
    motor2: Osc,
    mesh: Osc,
    lp: OnePole,
    ratchet: ModalBank,
    ratchet_phase: f32,
    clunk: Thump,
    clunk_modes: ModalBank,
    moving: bool,
}

struct Power {
    motor: [Osc; 3],
    ripple: [Osc; 2],
    lp: Svf,
    hiss: Svf,
}

enum Patch {
    Fan(Fan),
    Shake(Shake),
    Paddle(Paddle),
    Cart(Cart),
    Winch(Winch),
    Power(Power),
}

pub struct RigVoice {
    #[allow(dead_code)]
    kind: RigKind,
    sr: f32,
    dt: f32,
    rng: Rng,
    drive: Smooth,
    speed: Smooth,
    patch: Patch,
}

impl RigVoice {
    pub fn new(kind: RigKind, sr: f32, seed: u32) -> Self {
        let mut rng = Rng::new(seed ^ 0x51C0_0002);
        let mut osc = || Osc::new(rng.unit());
        let patch = match kind {
            RigKind::WindFan => Patch::Fan(Fan {
                blades: [osc(), osc(), osc(), osc()],
                fan2: osc(),
                motor: osc(),
                noise: Pink::new(seed),
                band: Svf::new(500.0, 0.6, sr),
                hp: OnePole::new(60.0, sr),
                gust: OnePole::new(1.5, sr),
            }),
            RigKind::ShakeTable => Patch::Shake(Shake {
                pump: [osc(), osc(), osc()],
                hiss: Svf::new(3_200.0, 0.8, sr),
                rumble: Brown::new(seed),
                rumble_lp: OnePole::new(70.0, sr),
                thump: Thump::new(42.0, 0.3, 250.0, sr),
                sign: 0.0,
                peak: 0.0,
            }),
            RigKind::WavePaddle => Patch::Paddle(Paddle {
                hum: [osc(), osc()],
                water: Pink::new(seed),
                slosh: Svf::new(500.0, 1.2, sr),
                slosh_drift: OnePole::new(0.7, sr),
                splash: {
                    let mut d = Decay::new(0.9, sr);
                    d.retrigger(0.0);
                    d
                },
                splash_amp: 0.0,
                splash_band: Svf::new(1_800.0, 0.7, sr),
                bubbles: [
                    (osc(), 0.0, 0.0, 1.0),
                    (osc(), 0.0, 0.0, 1.0),
                    (osc(), 0.0, 0.0, 1.0),
                    (osc(), 0.0, 0.0, 1.0),
                ],
                sign: 0.0,
                peak: 0.0,
            }),
            RigKind::Cart => Patch::Cart(Cart {
                roll: Brown::new(seed),
                roll_lp: OnePole::new(100.0, sr),
                hiss: Svf::new(1_500.0, 0.8, sr),
                joint: ModalBank::new(
                    &[
                        (930.0, 0.07, 1.0),
                        (1_710.0, 0.05, 0.6),
                        (2_630.0, 0.04, 0.4),
                    ],
                    sr,
                ),
                joint_thud: Resonator::new(120.0, 0.1, sr),
                travel: 0.0,
                second_axle: -1.0,
                rattle: ModalBank::new(&[(3_100.0, 0.02, 1.0), (4_700.0, 0.015, 0.5)], sr),
            }),
            RigKind::Winch => Patch::Winch(Winch {
                motor: osc(),
                motor2: osc(),
                mesh: osc(),
                lp: OnePole::new(1_200.0, sr),
                ratchet: ModalBank::new(&[(2_300.0, 0.03, 1.0), (3_900.0, 0.02, 0.5)], sr),
                ratchet_phase: 0.0,
                clunk: Thump::new(80.0, 0.2, 600.0, sr),
                clunk_modes: ModalBank::new(
                    &[(310.0, 0.25, 1.0), (720.0, 0.2, 0.6), (1_350.0, 0.12, 0.3)],
                    sr,
                ),
                moving: false,
            }),
            RigKind::PowerUnit => Patch::Power(Power {
                motor: [osc(), osc(), osc()],
                ripple: [osc(), osc()],
                lp: Svf::new(500.0, 0.7, sr),
                hiss: Svf::new(4_200.0, 0.9, sr),
            }),
        };
        Self {
            kind,
            sr,
            dt: 1.0 / sr,
            rng,
            drive: Smooth::new(0.0, 0.15, sr),
            speed: Smooth::new(0.0, 0.04, sr),
            patch,
        }
    }

    /// `drive` 0..1, `speed` physical rate (see [`crate::Command::Rig`]).
    pub fn set(&mut self, drive: f32, speed: f32) {
        self.drive.target = drive.clamp(0.0, 1.5);
        self.speed.target = if speed.is_finite() { speed } else { 0.0 };
    }

    fn at_rest(&self) -> bool {
        self.drive.target < 1e-4
            && self.drive.value < 1e-4
            && self.speed.target.abs() < 1e-4
            && self.speed.value.abs() < 1e-4
    }
}

impl Voice for RigVoice {
    fn render(&mut self, out: &mut [f32]) -> bool {
        if self.at_rest() {
            return true;
        }
        let (sr, dt) = (self.sr, self.dt);
        let rng = &mut self.rng;
        match &mut self.patch {
            Patch::Fan(p) => {
                for (i, o) in out.iter_mut().enumerate() {
                    let d = snap(&mut self.drive);
                    let w = snap(&mut self.speed).abs();
                    let u = (w / 25.0).min(1.6);
                    // 7 blades; blade-pass about 4.2 Hz per m/s of wind.
                    let bpf = 4.2 * w + 1.0;
                    let mut tone = 0.0;
                    for (h, b) in p.blades.iter_mut().enumerate() {
                        let k = (h + 1) as f32;
                        tone += b.sine(bpf * k, sr) / (k * k);
                    }
                    // A second fan in the wall, slightly off speed: beating.
                    tone += p.fan2.sine(bpf * 1.013, sr) * 0.7;
                    let gust = 1.0 + 0.6 * p.gust.lp(rng.bipolar()) * 4.0;
                    if i % 32 == 0 {
                        p.band.set(250.0 + 22.0 * w, 0.6, sr);
                    }
                    let n = p.noise.next();
                    let air = p.hp.hp(p.band.bp(n) + n * 0.35) * u * u * 0.22 * gust;
                    let motor = p.motor.sine(560.0 + 34.0 * w, sr) * 0.006 * d;
                    *o += tone * 0.07 * u.powf(1.5) + air + motor;
                }
            }
            Patch::Shake(p) => {
                for o in out.iter_mut() {
                    let d = snap(&mut self.drive);
                    let v = snap(&mut self.speed);
                    let a = v.abs();
                    let u = (a / 0.4).min(1.5);
                    // Axial piston pump, pitch sagging a little under flow.
                    let f = 228.0 * (1.0 - 0.03 * u);
                    let pump = p.pump[0].sine(f, sr)
                        + 0.5 * p.pump[1].sine(f * 2.0, sr)
                        + 0.25 * p.pump[2].sine(f * 3.0, sr);
                    let pump = pump * (0.012 + 0.025 * d);
                    let hiss = p.hiss.bp(rng.bipolar()) * 0.06 * u;
                    let rumble = p.rumble_lp.lp(p.rumble.next()) * 0.1 * u;
                    // Direction reversal: the platform's mass is stopped and
                    // thrown back.
                    let mut imp = 0.0;
                    p.peak = p.peak.max(a);
                    if a > 0.01 && v.signum() != p.sign {
                        if p.sign != 0.0 && p.peak > 0.03 {
                            imp = p.thump.hit((p.peak / 0.4).min(1.0) * 0.2);
                        }
                        p.sign = v.signum();
                        p.peak = a;
                    }
                    let thump = p.thump.next(imp, rng);
                    *o += pump + hiss + rumble + thump;
                }
            }
            Patch::Paddle(p) => {
                for (i, o) in out.iter_mut().enumerate() {
                    let d = snap(&mut self.drive);
                    let v = snap(&mut self.speed);
                    let a = v.abs();
                    let u = (a / 0.6).min(1.5);
                    let hum =
                        (p.hum[0].sine(162.0, sr) + 0.4 * p.hum[1].sine(324.0, sr)) * 0.012 * d;
                    let drift = p.slosh_drift.lp(rng.bipolar()) * 6.0;
                    if i % 32 == 0 {
                        p.slosh.set(
                            (380.0 + 650.0 * u) * (1.0 + 0.3 * drift.clamp(-1.0, 1.0)),
                            1.2,
                            sr,
                        );
                    }
                    let n = p.water.next();
                    let slosh = p.slosh.bp(n) * (0.05 * d.min(1.0) + 0.3 * u);
                    // Stroke reversal: the wave crest folds over.
                    p.peak = p.peak.max(a);
                    if a > 0.02 && v.signum() != p.sign {
                        if p.sign != 0.0 && p.peak > 0.05 {
                            p.splash.retrigger(1.0);
                            p.splash_amp = (p.peak / 0.6).min(1.0);
                        }
                        p.sign = v.signum();
                        p.peak = a;
                    }
                    let s = if p.splash.level() > 1e-6 {
                        p.splash.next() * p.splash_amp
                    } else {
                        0.0
                    };
                    let splash = p.splash_band.bp(rng.bipolar()) * s * 0.25;
                    // Droplets / bubbles: short upward chirps during splashes.
                    if s > 0.02 && rng.chance(s * 40.0 * dt) {
                        if let Some(b) = p.bubbles.iter_mut().find(|b| b.2 < 1e-3) {
                            b.1 = rng.range(600.0, 2_200.0);
                            b.2 = rng.range(0.3, 1.0) * s;
                            b.3 = 1.0 + rng.range(2.0, 6.0) * dt;
                        }
                    }
                    let mut bub = 0.0;
                    for b in p.bubbles.iter_mut() {
                        if b.2 > 1e-3 {
                            bub += b.0.sine(b.1, sr) * b.2;
                            b.1 *= b.3;
                            b.2 *= 1.0 - 60.0 * dt;
                        }
                    }
                    *o += hum + slosh + splash + bub * 0.03;
                }
            }
            Patch::Cart(p) => {
                const JOINT: f32 = 0.6; // m between rail joints
                const WHEELBASE: f32 = 0.3;
                for o in out.iter_mut() {
                    let d = snap(&mut self.drive);
                    let v = snap(&mut self.speed);
                    let a = v.abs();
                    let u = (a / 2.0).min(1.5);
                    let load = 0.6 + 0.6 * d;
                    p.roll_lp.set_cutoff(60.0 + 60.0 * a, sr);
                    let roll = p.roll_lp.lp(p.roll.next()) * 0.05 * u * load;
                    let hiss = p.hiss.bp(rng.bipolar()) * 0.012 * u;
                    let before = p.travel;
                    p.travel += a * dt;
                    let mut imp = 0.0;
                    if (before / JOINT).floor() != (p.travel / JOINT).floor() {
                        imp += 0.16 * u.sqrt() * load;
                        p.second_axle = p.travel + WHEELBASE;
                    }
                    if p.second_axle > 0.0 && p.travel >= p.second_axle {
                        imp += 0.13 * u.sqrt() * load;
                        p.second_axle = -1.0;
                    }
                    let imp = imp + ANTI_DENORMAL;
                    let clack = p.joint.process(imp) * 0.6 + p.joint_thud.process(imp) * 0.8;
                    let tick = if rng.chance(15.0 * a * dt) {
                        rng.range(0.01, 0.04)
                    } else {
                        0.0
                    };
                    let rattle = p.rattle.process(tick + ANTI_DENORMAL);
                    *o += roll + hiss + clack + rattle;
                }
            }
            Patch::Winch(p) => {
                for o in out.iter_mut() {
                    let d = snap(&mut self.drive);
                    let v = snap(&mut self.speed);
                    let a = v.abs();
                    let u = (a / 0.5).min(1.5);
                    let mut imp = 0.0;
                    let moving = a > 0.02;
                    if moving != p.moving {
                        // Brake releasing / engaging.
                        imp = p.clunk.hit(if moving { 0.14 } else { 0.2 });
                        p.moving = moving;
                    }
                    let f = 180.0 + 1_400.0 * a;
                    // Geared motor: fundamental and a second harmonic, filtered.
                    let m = p.motor.sine(f, sr)
                        + 0.35 * p.motor2.sine(f * 2.0, sr)
                        + 0.1 * rng.bipolar();
                    let motor = p.lp.lp(m) * 0.06 * u.min(1.0);
                    let mesh = p.mesh.sine(f * 1.5, sr) * 0.015 * u.min(1.0);
                    p.ratchet_phase += a * 40.0 * dt;
                    let tick = if p.ratchet_phase >= 1.0 {
                        p.ratchet_phase -= 1.0;
                        0.08 * (0.5 + 0.5 * d)
                    } else {
                        0.0
                    };
                    let ratchet = p.ratchet.process(tick + ANTI_DENORMAL);
                    let clunk =
                        p.clunk.next(imp, rng) + p.clunk_modes.process(imp * 0.5 + ANTI_DENORMAL);
                    *o += motor + mesh + ratchet + clunk;
                }
            }
            Patch::Power(p) => {
                for (i, o) in out.iter_mut().enumerate() {
                    let d = snap(&mut self.drive).min(1.2);
                    let _ = snap(&mut self.speed);
                    // 4-pole motor at ~1480 rpm: 24.7 Hz shaft, 100 Hz mains
                    // magnetostriction; 9-piston pump ripple at 222 Hz.
                    let m = p.motor[0].sine(24.7, sr)
                        + 0.6 * p.motor[1].sine(49.4, sr)
                        + 0.8 * p.motor[2].sine(100.0, sr);
                    let r = p.ripple[0].sine(222.0, sr) + 0.5 * p.ripple[1].sine(444.0, sr);
                    if i % 32 == 0 {
                        p.lp.set(400.0 + 1_600.0 * d, 0.7, sr);
                    }
                    let body =
                        p.lp.lp(m * 0.5 + r * (0.3 + 0.7 * d) + rng.bipolar() * 0.15);
                    let relief = (d - 0.6).max(0.0) * 2.5;
                    let hiss = p.hiss.bp(rng.bipolar()) * 0.03 * relief;
                    *o += body * (0.05 + 0.06 * d) + hiss;
                }
            }
        }
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const KINDS: [RigKind; 6] = [
        RigKind::WindFan,
        RigKind::ShakeTable,
        RigKind::WavePaddle,
        RigKind::Cart,
        RigKind::Winch,
        RigKind::PowerUnit,
    ];

    #[test]
    fn rigs_are_silent_at_rest() {
        for k in KINDS {
            let mut v = RigVoice::new(k, 48_000.0, 3);
            v.set(0.0, 0.0);
            let mut out = vec![0.0; 4800];
            v.render(&mut out);
            assert!(out.iter().all(|&x| x == 0.0), "{k:?}");
        }
    }

    #[test]
    fn rigs_at_full_drive_are_audible_and_bounded() {
        let sr = 48_000.0;
        for k in KINDS {
            let mut v = RigVoice::new(k, sr, 9);
            let mut out = vec![0.0; (sr * 4.0) as usize];
            for (c, chunk) in out.chunks_mut(128).enumerate() {
                let t = c as f32 * 128.0 / sr;
                let speed = match k {
                    RigKind::WindFan => 25.0,
                    RigKind::ShakeTable => 0.4 * (t * 12.0).sin(),
                    RigKind::WavePaddle => 0.6 * (t * 4.4).cos(),
                    RigKind::Cart => 2.0,
                    RigKind::Winch => 0.5,
                    RigKind::PowerUnit => 0.0,
                };
                v.set(1.0, speed);
                v.render(chunk);
            }
            assert!(out.iter().all(|x| x.is_finite()));
            let peak = out.iter().fold(0.0f32, |m, x| m.max(x.abs()));
            let rms = (out.iter().map(|x| x * x).sum::<f32>() / out.len() as f32).sqrt();
            println!("{k:?}: peak {peak:.3} rms {rms:.3}");
            assert!(peak <= 0.6, "{k:?} peak {peak}");
            assert!(rms > 0.01, "{k:?} rms {rms}");
        }
    }
}
