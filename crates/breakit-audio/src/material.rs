//! Structure sounds from the structure's physics: members straining,
//! damage, breaks and loose pieces landing.
//!
//! Nothing here is a patch tuned to sound like something. Every sound is a
//! score of **force events** played into the **modes of the bodies
//! involved** (see [`crate::body`] and [`crate::modal`]). What differs
//! between a steel rupture and a timber splinter is the physics of how the
//! force is let go:
//!
//! * **Release.** A member carrying force that suddenly cannot: the force
//!   steps to zero in the time the crack takes to cross the section. Both
//!   new pieces spring back and ring from their static deflection. A
//!   tension member dumps its energy into axial modes (a hard, pitched
//!   bang); a bending failure into bending modes (lower, clangier).
//! * **Micro-fracture** (acoustic emission). Cracks advance in jumps:
//!   fibres, grains and laminate plies give way in avalanches whose
//!   energies follow a power law, `P(E) ~ E^-β` with β about 1.4 for wood
//!   under compression (Mäkinen et al. 2015), close to other brittle solids,
//!   and whose aftershocks thin out by Omori's law, rate `~ t^-0.75`. Bigger
//!   events last longer (duration scales with crack size, size with `E^⅓`),
//!   so big cracks are low and small ones tick.
//! * **Stick-slip.** Joints and crack faces under changing load stick,
//!   build up force and slip. Each slip is a small impulse. At a steady
//!   sliding speed slips come at a steady rate, which is the pitch of a
//!   creak: it rises when the load changes faster.
//! * **Contact.** A piece landing pushes on the floor for a Hertz contact
//!   time, `τ = 2.87 (m² / (R E*² v))^⅕`: milliseconds for timber (dull
//!   knock), a fraction of one for steel on concrete (bright clang).
//!
//! Loudness follows energy: a voice's level is `(E / 1 kJ)^0.3`, between
//! the physical square root and nothing at all, because a game cannot play
//! the 60 dB between a creak and a girder snapping.

use crate::body::{floor_modes, Body, Drive, Ends, ModeSpec};
use crate::dsp::{OnePole, Rng};
use crate::modal::{step_leak, Bank, GrainHit, Grains, Hit, ModalVoice, Pulses};
use crate::timbre::{self, Timbre};
use crate::{BoxVoice, BreakCause, DamageState, MaterialKind, Voice};

/// Modes per body in a break, an impact and a straining member, at full
/// detail.
const BREAK_MODES: usize = 28;
const IMPACT_MODES: usize = 22;
const STRAIN_MODES: usize = 18;

/// A mode budget scaled by `detail` (0..1). The mixer lowers detail when
/// many voices already sound: in a collapse nobody can tell 28 modes from
/// 12, and the cost is all in the modes.
fn budget(modes: usize, detail: f32) -> usize {
    ((modes as f32 * detail.clamp(0.3, 1.0)) as usize).max(6)
}

/// Overall scale from unit-energy bank output to full scale.
const OUTPUT: f32 = 0.8;

/// Level of a voice for an event that releases `energy` joules.
pub fn level(energy: f32) -> f32 {
    (energy.max(0.0) / 1000.0).powf(0.3).clamp(0.0, 2.2)
}

/// Elastic energy per volume at the material's strength, J/m³: σ²/2E.
fn energy_density(m: MaterialKind) -> f32 {
    match m {
        MaterialKind::Steel => 3.0e5,
        MaterialKind::Timber => 2.6e4,
        MaterialKind::Concrete => 1.4e4,
        MaterialKind::Composite => 2.0e6,
    }
}

/// Elastic energy a member holds at capacity, J.
pub fn capacity_energy(body: &Body) -> f32 {
    energy_density(body.material) * body.area * body.length
}

/// How one material cracks at the small scale.
#[derive(Clone, Copy, Debug)]
struct Micro {
    /// Power-law exponent of event energies.
    beta: f32,
    /// Duration of the largest events, s (crack size over crack speed).
    tau: f32,
    /// Roll-off of the events' own sound, Hz.
    cutoff: f32,
    /// Level of the events' own sound against the ringing bodies.
    direct: f32,
}

fn micro(m: MaterialKind) -> Micro {
    match m {
        // Ductile tearing is nearly silent; what cracks is scale, welds
        // and bolts.
        MaterialKind::Steel => Micro {
            beta: 1.8,
            tau: 0.15e-3,
            cutoff: 11_000.0,
            direct: 0.5,
        },
        // Fibre bundles a millimetre or more across.
        MaterialKind::Timber => Micro {
            beta: 1.4,
            tau: 0.35e-3,
            cutoff: 9_000.0,
            direct: 1.1,
        },
        // Aggregate-sized cracks and stones knocking together: hard, light
        // and short.
        MaterialKind::Concrete => Micro {
            beta: 1.5,
            tau: 0.25e-3,
            cutoff: 9_000.0,
            direct: 1.5,
        },
        // Fibres a few microns thick: all ticks.
        MaterialKind::Composite => Micro {
            beta: 1.6,
            tau: 0.09e-3,
            cutoff: 12_000.0,
            direct: 0.9,
        },
    }
}

/// Loss factor a member picks up from the structure it is built into:
/// friction in its joints and energy leaking into its neighbours. It is
/// several times the material's own and it is why a member in a frame thuds
/// where the same bar hung from a string would ring like a glass.
fn joint_loss(m: MaterialKind) -> f32 {
    match m {
        MaterialKind::Steel => 0.012,
        MaterialKind::Timber => 0.03,
        MaterialKind::Concrete => 0.04,
        MaterialKind::Composite => 0.015,
    }
}

/// A body's modes as they ring in place: with the joint loss added.
fn seated(body: &Body, drive: Drive, budget: usize, seed: u32) -> Vec<ModeSpec> {
    let eta = joint_loss(body.material);
    let mut modes = body.modes(drive, budget, seed);
    for m in &mut modes {
        m.decay += std::f32::consts::PI * m.freq * eta;
        // A joint passes less of a mode the shorter its wavelength gets
        // against the joint itself; above a kilohertz or two a member in a
        // frame hardly rings at all. (An estimate, not a calculation.)
        m.gain *= (RING_CORNER / m.freq.max(RING_CORNER)).powf(1.5);
    }
    modes
}

/// Above this (Hz) a seated member's modes fall away.
const RING_CORNER: f32 = 1200.0;

/// How hard a recorded event rings the member it happens in, against how
/// loud the recording itself is heard.
const BODY: f32 = 0.35;

/// Longest a single recorded micro-event plays, s.
const MICRO_LIMIT: f32 = 0.09;

/// Largest avalanche over the smallest, in energy.
const E_RANGE: f32 = 1000.0;

/// An event energy from the power law, as a fraction of the largest
/// possible (`cap` 0..1 lowers the cutoff).
fn avalanche(rng: &mut Rng, beta: f32, cap: f32) -> f32 {
    let u = rng.unit().max(1e-6);
    let e = u.powf(-1.0 / (beta - 1.0));
    (e / E_RANGE).min(cap.clamp(1.0 / E_RANGE, 1.0))
}

/// A score under construction.
struct Score {
    hits: Vec<Hit>,
    recorded: Vec<GrainHit>,
    rng: Rng,
    sr: f32,
    micro: Micro,
    material: MaterialKind,
    /// Section size of the member, m: sets the pitch of its cracks.
    size: f32,
}

impl Score {
    fn new(material: MaterialKind, sr: f32, seed: u32) -> Self {
        Self {
            hits: Vec::new(),
            recorded: Vec::new(),
            rng: Rng::new(seed),
            sr,
            micro: micro(material),
            material,
            size: 0.1,
        }
    }

    fn sized(mut self, body: &Body) -> Self {
        self.size = body.width.max(body.depth);
        self
    }

    /// Plays `slice` at `t`: heard at `gain`, and ringing bank `to`.
    fn play(&mut self, t: f32, slice: timbre::Slice, rate: f32, gain: f32, to: u8, limit: f32) {
        self.recorded.push(GrainHit {
            at: (t.max(0.0) * self.sr) as u32,
            to,
            slice,
            rate,
            gain,
            body: gain * BODY,
            limit,
        });
    }

    /// A whole recording of `what` at `t`, pitched for a thing `size` m
    /// across. False if there is no such recording.
    fn event(&mut self, t: f32, what: Timbre, size: f32, gain: f32, to: u8) -> bool {
        let Some(slice) = timbre::whole(what, &mut self.rng) else {
            return false;
        };
        let rate = timbre::size_rate(what, size) * self.rng.range(0.94, 1.06);
        self.play(t, slice, rate, gain, to, 6.0);
        true
    }

    /// One single event out of a recording of `what`.
    fn single(&mut self, t: f32, what: Timbre, size: f32, gain: f32, to: u8) -> bool {
        let Some(slice) = timbre::event(what, &mut self.rng) else {
            return false;
        };
        let rate = timbre::size_rate(what, size) * self.rng.range(0.9, 1.1);
        self.play(t, slice, rate, gain, to, 0.6);
        true
    }

    /// `seconds` of a sustained recording of `what`.
    fn sustained(&mut self, t: f32, what: Timbre, seconds: f32, gain: f32, to: u8) -> bool {
        let Some(slice) = timbre::stretch(what, seconds, &mut self.rng) else {
            return false;
        };
        let rate = timbre::size_rate(what, self.size) * self.rng.range(0.94, 1.06);
        self.play(t, slice, rate, gain, to, seconds);
        true
    }

    /// The finished one-shot. The recordings are the events' own sound, so
    /// the computed clicks that stood in for it are dropped where a
    /// recording plays.
    fn voice(mut self, banks: Vec<Bank>, gain: f32, direct_gain: f32) -> BoxVoice {
        if !self.recorded.is_empty() {
            for h in &mut self.hits {
                h.direct = 0.0;
            }
        }
        Box::new(
            ModalVoice::new(
                banks,
                self.hits,
                gain,
                direct_gain,
                self.micro.cutoff,
                self.sr,
            )
            .with_recorded(self.recorded),
        )
    }

    fn samples(&self, seconds: f32) -> u16 {
        (seconds * self.sr).round().clamp(1.0, 4000.0) as u16
    }

    fn push(&mut self, t: f32, tau: f32, to: u8, step: bool, amp: f32, direct: f32) {
        self.hits.push(Hit {
            at: (t.max(0.0) * self.sr) as u32,
            len: self.samples(tau),
            to,
            step,
            amp,
            direct,
        });
    }

    /// A force pulse: an impact or a slip.
    fn pulse(&mut self, t: f32, tau: f32, to: u8, amp: f32, direct: f32) {
        self.push(t, tau, to, false, amp, direct);
    }

    /// A force step: a load let go over `tau`.
    fn release(&mut self, t: f32, tau: f32, to: u8, amp: f32, direct: f32) {
        self.push(t, tau, to, true, amp, direct);
    }

    /// One micro-fracture of relative energy `e` (0..1) at `t`.
    fn crack(&mut self, t: f32, e: f32, to: u8, amp: f32) {
        let a = amp * e.sqrt();
        let what = timbre::crack_of(self.material);
        if let Some(slice) = timbre::event(what, &mut self.rng) {
            // Bigger cracks last longer, so they sound lower: duration goes
            // as energy to the third; a quarter of that reaches the pitch.
            let rate = timbre::size_rate(what, self.size)
                * e.max(1e-3).powf(-0.08)
                * self.rng.range(0.9, 1.1);
            self.play(t, slice, rate, a * self.micro.direct, to, MICRO_LIMIT);
            return;
        }
        let tau = self.micro.tau * e.cbrt();
        self.pulse(t, tau, to, a, a * self.micro.direct);
    }

    /// `n` micro-fractures spread over the banks `to`, at times drawn by
    /// `when`. Sizes follow the material's power law; `fade` (s) is how fast
    /// the largest possible event shrinks.
    fn shower(
        &mut self,
        n: usize,
        to: &[u8],
        amp: f32,
        fade: f32,
        mut when: impl FnMut(&mut Rng) -> f32,
    ) {
        // A recorded crack is already a small avalanche of its own, so
        // fewer of them make the same shower.
        let recorded = !timbre::clips(timbre::crack_of(self.material)).is_empty();
        let n = if recorded { n.div_ceil(4).max(2) } else { n };
        let amp = if recorded { amp * 1.6 } else { amp };
        for _ in 0..n {
            let t = when(&mut self.rng);
            let cap = if fade > 0.0 {
                1.0 / (1.0 + t / fade)
            } else {
                1.0
            };
            let e = avalanche(&mut self.rng, self.micro.beta, cap);
            let bank = to[(self.rng.next_u32() as usize) % to.len()];
            self.crack(t, e, bank, amp);
        }
    }
}

/// A time from Omori's law, rate `~ (t + c)^-p` on `[0, span]`.
fn omori(rng: &mut Rng, c: f32, span: f32, p: f32) -> f32 {
    let q = 1.0 - p;
    let (lo, hi) = (c.powf(q), (span + c).powf(q));
    (lo + rng.unit() * (hi - lo)).powf(1.0 / q) - c
}

/// A time from an exponentially decaying rate with time constant `tau`.
fn decaying(rng: &mut Rng, tau: f32, span: f32) -> f32 {
    (-tau * rng.unit().max(1e-6).ln()).min(span * rng.range(0.6, 1.0))
}

/// What the game knows about a break.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BreakInfo {
    pub body: Body,
    pub cause: BreakCause,
    /// Where along the member, 0..1.
    pub at: f32,
    /// Elastic energy the member held when it let go, J.
    pub energy: f32,
    /// Share of that energy that was axial (the rest was bending), 0..1.
    pub axial: f32,
}

fn sanitize(body: &Body) -> Option<Body> {
    let ok = [
        body.youngs,
        body.shear,
        body.density,
        body.length,
        body.area,
        body.iy,
        body.iz,
        body.width,
        body.depth,
    ]
    .iter()
    .all(|x| x.is_finite() && *x > 0.0);
    (ok && body.wall.is_finite() && body.damage.is_finite()).then_some(*body)
}

fn finite(x: f32, or: f32) -> f32 {
    if x.is_finite() {
        x
    } else {
        or
    }
}

const A: u8 = 0;
const B: u8 = 1;
const FLOOR: u8 = 2;
const EXTRA: u8 = 3;

/// A member breaks.
pub fn break_voice(info: &BreakInfo, detail: f32, sr: f32, seed: u32) -> Option<BoxVoice> {
    let body = sanitize(&info.body)?;
    let energy = finite(info.energy, 0.0).max(0.02 * capacity_energy(&body));
    let gain = level(energy) * OUTPUT;
    if gain < 1e-3 {
        return None;
    }
    let material = body.material;
    let cause = info.cause;
    let axial = finite(info.axial, 0.5).clamp(0.0, 1.0);
    let x = finite(info.at, 0.5).clamp(0.0, 1.0);
    let mut s = Score::new(material, sr, seed).sized(&body);
    let m = s.micro;

    // The two pieces: each still held at its joint, free at the break. A
    // connection failure leaves the member whole, hanging from its other
    // end, and jolts whatever it was bolted to.
    let connection = matches!(cause, BreakCause::Connection);
    let split = x.clamp(0.15, 0.85);
    let (pa, pb) = if connection {
        (
            body.piece(1.0, Ends::Cantilever),
            body.piece(1.0, Ends::Held),
        )
    } else {
        (
            body.piece(split, Ends::Cantilever),
            body.piece(1.0 - split, Ends::Cantilever),
        )
    };
    // Bending and axial shares of the force that was released.
    let drive = Drive {
        at: 1.0,
        transverse: (1.0 - axial).sqrt().max(0.25),
        axial: axial.sqrt().max(0.15),
    };
    let modes = budget(BREAK_MODES, detail);
    // Steel pulled apart is computed from end to end: the load let go rings
    // the two halves with steel's own low loss, and they whip sideways as
    // well as recoiling, since no break is square.
    let computed = material == MaterialKind::Steel && matches!(cause, BreakCause::Rupture);
    let held = |piece: &Body, drive: Drive, seed: u32| {
        if computed {
            let drive = Drive {
                transverse: drive.transverse.max(0.6),
                ..drive
            };
            piece.modes(drive, modes, seed)
        } else {
            seated(piece, drive, modes, seed)
        }
    };
    let mut banks = vec![
        Bank::new(&held(&pa, drive, seed), sr),
        Bank::new(
            &held(
                &pb,
                if connection {
                    Drive { at: 0.04, ..drive }
                } else {
                    drive
                },
                seed ^ 0xB,
            ),
            sr,
        ),
        Bank::new(&floor_modes(seed), sr),
    ];
    let both = [A, B];
    // Time for a stress wave to cross a piece and come back.
    let echo = |b: &Body| 2.0 * b.length / b.wave_speed();

    match cause {
        // Steel in tension: necking is silent, then the last ligament
        // shears through in a fraction of a millisecond and the halves
        // recoil. The wave that bounces along each half is its axial modes,
        // already in the banks.
        BreakCause::Rupture if computed => {
            s.release(0.0, 0.25e-3, A, 0.8, 1.0);
            s.release(0.0, 0.25e-3, B, 0.8, 0.0);
            s.release(0.0, 1.5e-3, FLOOR, 0.15, 0.0);
            // The jolt runs on into the frame: the members next to this one
            // take it a wave transit later, spread out by the joints, and
            // ring lower and longer. (Their sizes are a guess: twice this
            // member, held at both ends.)
            let frame = Body {
                length: 2.0 * body.length,
                ends: Ends::Held,
                ..body
            };
            banks.push(Bank::new(
                &frame.modes(
                    Drive {
                        at: 0.31,
                        transverse: 1.0,
                        axial: 0.3,
                    },
                    modes,
                    seed ^ 0xF2A,
                ),
                sr,
            ));
            let transit = body.length / body.wave_speed();
            s.release(transit, 1.2e-3, EXTRA, 0.6, 0.0);
            s.release(3.0 * transit, 2.0e-3, EXTRA, 0.3, 0.0);
        }
        BreakCause::Rupture => {
            s.release(0.0, 0.25e-3, A, 0.8, 1.0);
            s.release(0.0, 0.25e-3, B, 0.8, 0.0);
            for (bank, piece) in [(A, &pa), (B, &pb)] {
                let period = echo(piece);
                for k in 0..4 {
                    let t = (0.5 + k as f32) * period * s.rng.range(0.95, 1.05);
                    let a = 0.4 * 0.55f32.powi(k);
                    s.pulse(t, 0.12e-3, bank, a, 0.5 * a);
                }
            }
            s.release(0.0, 1.5e-3, FLOOR, 0.15, 0.0);
        }
        // A plastic hinge used up: the tension side tears open in steps,
        // each a little larger and sooner than the last.
        BreakCause::Hinge => {
            let span = s.rng.range(0.035, 0.1);
            let n = 5 + (s.rng.next_u32() % 4) as usize;
            for i in 0..n {
                let f = (i + 1) as f32 / n as f32;
                let t = span * (1.0 - (1.0 - f).powf(1.7));
                let a = 0.15 + 0.6 * f * f;
                s.release(t, 0.6e-3, both[i % 2], a, 0.5 * a);
                s.release(t, 0.6e-3, both[(i + 1) % 2], 0.6 * a, 0.0);
            }
            s.release(span, 3e-3, FLOOR, 0.12, 0.0);
        }
        // Bolts or a weld: the fasteners shear one after another as each
        // hands its load to the next, then the member is free.
        BreakCause::Connection => {
            let n = 2 + (s.rng.next_u32() % 3) as usize;
            let mut t = 0.0;
            for i in 0..n {
                let a = 0.45 + 0.1 * i as f32;
                s.pulse(t, 0.08e-3, A, a, 1.2 * a);
                s.pulse(t, 0.08e-3, B, 0.5 * a, 0.0);
                t += s.rng.range(0.6e-3, 3.5e-3);
            }
            s.release(t, 0.3e-3, A, 0.8, 0.6);
            s.release(t, 0.3e-3, B, 0.45, 0.0);
            s.release(t, 1.5e-3, FLOOR, 0.15, 0.0);
        }
        // Timber in tension or bending: fibre bundles let go in a ragged
        // run across the section, then splinters keep tearing out.
        BreakCause::Splinter => {
            let span = s.rng.range(0.010, 0.028);
            let n = 6 + (s.rng.next_u32() % 5) as usize;
            let mut sizes: Vec<f32> = (0..n)
                .map(|_| avalanche(&mut s.rng, m.beta, 1.0).sqrt() + 0.15)
                .collect();
            let norm = sizes.iter().map(|a| a * a).sum::<f32>().sqrt();
            sizes.iter_mut().for_each(|a| *a /= norm);
            for (i, a) in sizes.iter().enumerate() {
                let t = span * (i as f32 + s.rng.unit()) / n as f32;
                let tau = s.rng.range(0.3e-3, 0.9e-3);
                s.release(t, tau, both[i % 2], 1.1 * a, 1.3 * a);
                s.release(t, tau, both[(i + 1) % 2], 0.7 * a, 0.0);
            }
            let after = 70 + (s.rng.next_u32() % 70) as usize;
            s.shower(after, &both, 0.6, 0.12, |r| {
                span + omori(r, 0.005, 0.5, 0.75)
            });
            s.release(span, 3e-3, FLOOR, 0.12, 0.0);
        }
        // Timber shear: one crack runs the length of the member along the
        // grain at a few hundred metres a second.
        BreakCause::TimberSplit => {
            let run = (body.length / 300.0).clamp(0.006, 0.03);
            s.shower(36, &both, 0.5, 0.0, |r| run * r.unit());
            s.release(run, 0.6e-3, A, 0.75, 0.9);
            s.release(run, 0.6e-3, B, 0.75, 0.0);
            s.shower(40, &both, 0.4, 0.08, |r| run + omori(r, 0.004, 0.3, 0.75));
            s.release(run, 3e-3, FLOOR, 0.10, 0.0);
        }
        // Crushing: no clean release, the section grinds down grain by
        // grain (concrete) or fibres kink and fold (timber).
        BreakCause::Crushing => {
            let (n, span, tau) = if material == MaterialKind::Timber {
                (130, 0.45, 0.14)
            } else {
                (320, 0.7, 0.2)
            };
            s.release(0.0, 2.5e-3, A, 0.4, 0.5);
            s.release(0.0, 2.5e-3, B, 0.4, 0.0);
            s.shower(n, &[A, B, FLOOR], 0.55, 0.25, |r| decaying(r, tau, span));
            s.release(0.0, 5e-3, FLOOR, 0.17, 0.0);
        }
        // Reinforced concrete: the concrete cracks, then the bars snap one
        // by one and ring inside it.
        BreakCause::RebarRupture => {
            s.release(0.0, 0.6e-3, A, 0.55, 0.8);
            s.release(0.0, 0.6e-3, B, 0.55, 0.0);
            s.shower(70, &[A, B, FLOOR], 0.4, 0.15, |r| decaying(r, 0.08, 0.35));
            let bar = Body {
                ends: Ends::Held,
                // Cast in: the concrete damps it hard.
                damage: 0.9,
                ..Body::member(
                    &breakit_core::Material::steel(),
                    &breakit_core::Section::rect(0.016, 0.016),
                    body.length as f64,
                )
            };
            banks.push(Bank::new(
                &bar.modes(
                    Drive {
                        at: split,
                        transverse: 0.5,
                        axial: 0.9,
                    },
                    16,
                    seed ^ 0xBA2,
                ),
                sr,
            ));
            let bars = 2 + (s.rng.next_u32() % 3) as usize;
            let mut t = s.rng.range(0.004, 0.02);
            for _ in 0..bars {
                s.release(t, 0.1e-3, EXTRA, 0.55, 0.5);
                t += s.rng.range(0.008, 0.05);
            }
            s.release(0.0, 3e-3, FLOOR, 0.17, 0.0);
        }
        // Carbon fibre: stores far more energy per kilo than anything
        // else here and lets go of all of it at once, then loose fibres
        // keep snapping.
        BreakCause::FibreRupture => {
            s.release(0.0, 0.08e-3, A, 0.8, 1.1);
            s.release(0.0, 0.08e-3, B, 0.8, 0.0);
            s.shower(260, &both, 0.45, 0.02, |r| decaying(r, 0.012, 0.07));
            s.release(0.0, 1e-3, FLOOR, 0.10, 0.0);
        }
        // Plies peel apart: a crack runs the length at over a kilometre a
        // second.
        BreakCause::Delamination => {
            let run = (body.length / 1500.0).clamp(0.002, 0.008);
            s.shower(50, &both, 0.5, 0.0, |r| run * r.unit());
            s.release(run, 0.15e-3, A, 0.7, 0.8);
            s.release(run, 0.15e-3, B, 0.7, 0.0);
            s.shower(90, &both, 0.35, 0.02, |r| run + decaying(r, 0.012, 0.05));
        }
        // Brittle cracking and diagonal shear in concrete: one sharp
        // release, then the faces grind and rubble falls.
        _ => {
            let shear = matches!(cause, BreakCause::ConcreteShear);
            s.release(0.0, 0.5e-3, A, 0.8, 1.0);
            s.release(0.0, 0.5e-3, B, 0.8, 0.0);
            let n = if shear { 150 } else { 110 };
            s.shower(n, &[A, B, FLOOR], 0.6, 0.12, |r| decaying(r, 0.09, 0.4));
            s.release(0.0, 3e-3, FLOOR, 0.20, 0.0);
        }
    }
    // What the failure itself sounds like, from a recording of one.
    let size = s.size;
    let _ = m;
    match cause {
        BreakCause::Rupture if computed => {}
        BreakCause::Rupture => {
            s.event(0.0, Timbre::SteelSnap, size, 1.0, A);
        }
        BreakCause::Hinge => {
            s.sustained(0.0, Timbre::SteelGroan, 0.7, 0.7, A);
            s.event(0.05, Timbre::SteelTear, size, 0.9, B);
        }
        BreakCause::Connection => {
            s.event(0.0, Timbre::SteelTear, size, 1.0, A);
        }
        BreakCause::Splinter | BreakCause::TimberSplit => {
            s.event(0.0, Timbre::TimberBreak, size, 1.0, A);
        }
        BreakCause::Crushing => {
            let what = if material == MaterialKind::Timber {
                Timbre::TimberCrush
            } else {
                Timbre::ConcreteCrush
            };
            s.event(0.0, what, size, 0.9, A);
        }
        BreakCause::RebarRupture => {
            s.event(0.0, Timbre::ConcreteBreak, size, 1.0, A);
            let bars = 2 + (s.rng.next_u32() % 3) as usize;
            let mut t = s.rng.range(0.004, 0.02);
            for _ in 0..bars {
                s.single(t, Timbre::RebarSnap, 0.016, 0.6, B);
                t += s.rng.range(0.008, 0.05);
            }
        }
        BreakCause::FibreRupture => {
            s.event(0.0, Timbre::CarbonSnap, size, 1.0, A);
        }
        BreakCause::Delamination => {
            s.event(0.0, Timbre::CarbonCrackle, size, 0.9, A);
        }
        _ => {
            s.event(0.0, Timbre::ConcreteBreak, size, 1.0, A);
        }
    }
    Some(s.voice(banks, gain, 0.5))
}

/// A member reaches a worse damage state without breaking. `energy` is the
/// elastic energy it holds, J; a few percent of it is released.
pub fn damage_voice(
    body: &Body,
    state: DamageState,
    energy: f32,
    sr: f32,
    seed: u32,
) -> Option<BoxVoice> {
    let body = sanitize(body)?;
    let material = body.material;
    let held = finite(energy, 0.0).max(0.05 * capacity_energy(&body));
    let mut s = Score::new(material, sr, seed).sized(&body);
    let at = s.rng.range(0.1, 0.9);
    let drive = Drive {
        at,
        transverse: 0.9,
        axial: 0.45,
    };
    let share = match state {
        DamageState::Intact | DamageState::Broken => return None,
        // First yield: the metal itself flows silently. What is heard is
        // the member lengthening in its connections: a bolt slips into
        // bearing, mill scale flakes off.
        DamageState::Yielded => {
            s.pulse(0.0, 0.2e-3, A, 0.9, 0.35);
            let size = s.size;
            s.single(0.0, Timbre::SteelKnock, size, 0.8, A);
            s.sustained(0.02, Timbre::SteelGroan, 0.6, 0.6, A);
            0.02
        }
        // A crack opens: one jump and its aftershocks.
        DamageState::Cracked => {
            s.crack(0.0, 1.0, A, 0.9);
            let n = if material == MaterialKind::Timber {
                22
            } else {
                12
            };
            s.shower(n, &[A], 0.6, 0.03, |r| omori(r, 0.003, 0.09, 0.75));
            0.03
        }
        // Plies let go along part of the length.
        DamageState::Delaminated => {
            let run = (body.length / 1500.0).clamp(0.002, 0.006);
            s.shower(30, &[A], 0.6, 0.0, |r| run * r.unit());
            s.release(run, 0.15e-3, A, 0.6, 0.6);
            0.06
        }
        // Crushing starts: grains or fibres giving way under compression.
        DamageState::Crushed => {
            s.shower(110, &[A], 0.6, 0.15, |r| decaying(r, 0.1, 0.35));
            let what = if material == MaterialKind::Timber {
                Timbre::TimberCrush
            } else {
                Timbre::ConcreteCrush
            };
            s.sustained(0.0, what, 0.5, 0.7, A);
            0.06
        }
    };
    let gain = level(held * share) * OUTPUT;
    if gain < 1e-3 {
        return None;
    }
    let bank = Bank::new(&seated(&body, drive, STRAIN_MODES + 4, seed), sr);
    Some(s.voice(vec![bank], gain, 0.5))
}

/// Contact stiffness E* against the floor, Pa, and the coefficient of
/// restitution. Timber and concrete crush locally, so their contact is far
/// softer than their Young's modulus along the member suggests.
fn contact(m: MaterialKind) -> (f32, f32) {
    match m {
        MaterialKind::Steel => (2.6e10, 0.45),
        MaterialKind::Timber => (1.5e9, 0.4),
        MaterialKind::Concrete => (6.0e9, 0.2),
        MaterialKind::Composite => (5.0e9, 0.5),
    }
}

/// Hertz contact time of a mass `m` (kg) landing at `v` (m/s) on a corner
/// of radius `r` (m) with contact modulus `e` (Pa).
pub fn contact_time(m: f32, v: f32, r: f32, e: f32) -> f32 {
    2.87 * (m * m / (r * e * e * v.max(0.05))).powf(0.2)
}

/// A loose piece lands. `speed` m/s into the floor, `mass` kg.
pub fn impact_voice(
    body: &Body,
    speed: f32,
    mass: f32,
    detail: f32,
    sr: f32,
    seed: u32,
) -> Option<BoxVoice> {
    let body = sanitize(body)?;
    let speed = finite(speed, 0.0).clamp(0.0, 60.0);
    let mass = finite(mass, 0.0).clamp(0.0, 20_000.0);
    let energy = 0.5 * mass * speed * speed;
    let gain = level(energy) * OUTPUT;
    if gain < 0.015 {
        return None;
    }
    let material = body.material;
    let mut s = Score::new(material, sr, seed).sized(&body);
    let (stiffness, restitution) = contact(material);
    // It lands on a corner or an edge a fraction of its section across.
    let corner = (0.08 * body.width.min(body.depth)).clamp(0.002, 0.03);
    // A long piece is not rigid: during the contact the floor only feels
    // the part of it a bending wave has had time to reach, about a
    // wavelength at the contact's own frequency. Solve for that.
    let mut felt = mass.max(1e-4);
    let mut tau = contact_time(felt, speed, corner, stiffness);
    for _ in 0..3 {
        let reach = body.bending_wavelength(0.5 / tau) / std::f32::consts::PI;
        felt = mass.min(body.density * body.area * reach).max(1e-4);
        tau = contact_time(felt, speed, corner, stiffness);
    }
    let tau = tau.clamp(0.04e-3, 8e-3);
    let piece = Body {
        ends: Ends::Free,
        ..body
    };
    // Long pieces land on an end, chunks anywhere.
    let slender = piece.length / piece.width.max(piece.depth);
    let at = if slender > 3.0 {
        s.rng.range(0.0, 0.12)
    } else {
        s.rng.unit()
    };
    let drive = Drive {
        at,
        transverse: 1.0,
        axial: if slender > 3.0 { 0.25 } else { 0.5 },
    };
    // Lying on the floor damps it a little more than hanging free.
    let resting = piece.with_damage(piece.damage.max(0.15));
    // Quiet pieces get fewer modes: nobody hears the difference.
    let budget = budget(
        if gain > 0.05 {
            IMPACT_MODES
        } else {
            IMPACT_MODES / 2
        },
        detail,
    );
    // A loose steel or timber piece is a free bar and rings as one: its own modes and
    // the contact pulse are the whole sound, nothing recorded. The other
    // materials land with a recorded knock.
    let rings = matches!(material, MaterialKind::Steel | MaterialKind::Timber);
    let ringing = |drive: Drive, budget: usize, seed: u32| {
        if rings {
            piece.modes(drive, budget, seed)
        } else {
            seated(&resting, drive, budget, seed)
        }
    };
    let modes = ringing(drive, budget, seed);
    // The mode gains assume an ideal impulse. A real contact lasts `tau`
    // and leaves the modes above about 1/tau alone, so a soft landing rings
    // less of the piece. Most of that loss is real (a soft landing *is*
    // quieter); the level law already compressed loudness once, so give
    // some of it back.
    let reached: f32 = modes
        .iter()
        .map(|m| {
            let x = m.freq * tau;
            let p = if (x - 1.0).abs() < 1e-3 {
                0.5
            } else {
                ((std::f32::consts::PI * x).sin() / (std::f32::consts::PI * x * (1.0 - x * x)))
                    .abs()
            };
            (m.gain * p.min(1.0)).powi(2)
        })
        .sum::<f32>()
        .sqrt();
    let gain = gain * (1.0 / reached.max(0.05)).sqrt().min(4.0);
    let banks = vec![
        Bank::new(&modes, sr),
        // The same bar, driven at the far end, for when it slaps down.
        Bank::new(
            &ringing(
                Drive {
                    at: 1.0 - at,
                    ..drive
                },
                budget / 2,
                seed ^ 0x5,
            ),
            sr,
        ),
        Bank::new(&floor_modes(seed), sr),
    ];
    // The contact's own sound: louder the shorter it is against the
    // piece's ringing.
    let click = (0.4e-3 / tau).sqrt().clamp(0.3, 1.2);
    // Neither surface is smooth: the force arrives through a few asperities
    // that each make contact for a fraction of the whole.
    // What the landing sounds like: the whole recording for the first
    // contact, single knocks out of one for the bounces. The length of the
    // piece sets its pitch, the hardness of the contact its level.
    let what = timbre::drop_of(material, mass);
    let length = piece.length;
    let land = |s: &mut Score, t: f32, bank: u8, a: f32| {
        if rings {
        } else if t <= 0.0 {
            s.event(t, what, length, a * click, bank);
        } else {
            s.single(t, what, length, a * click, bank);
        }
        s.pulse(t, tau, bank, a, a * click);
        for _ in 0..if rings { 0 } else { 3 } {
            let dt = tau * s.rng.range(0.0, 0.8);
            let fine = tau * s.rng.range(0.1, 0.3);
            s.pulse(t + dt, fine, bank, 0.22 * a, 0.3 * a * click);
        }
        s.pulse(t, tau * 1.3, FLOOR, 0.3 * a, 0.0);
    };
    land(&mut s, 0.0, A, 1.0);
    // A long piece that landed on one end falls over onto the other.
    if slender > 3.0 {
        let fall = (0.5 * piece.length / speed.max(1.0)).clamp(0.02, 0.16) * s.rng.range(0.7, 1.3);
        let a = s.rng.range(0.45, 0.8);
        land(&mut s, fall, B, a);
    }
    // It chatters: each bounce is shorter and softer by the restitution.
    let mut t = 0.0;
    let mut gap = (2.0 * restitution * speed / 9.81).clamp(0.01, 0.25) * 0.35;
    let mut a = 1.0;
    for i in 0..3 {
        gap *= restitution;
        a *= restitution;
        t += gap;
        if a < 0.05 || gap < 2e-3 {
            break;
        }
        land(&mut s, t, both_ends(i), a);
    }
    // Concrete sheds grit when it lands, timber a few splinters.
    let grit = match material {
        MaterialKind::Concrete => 24,
        MaterialKind::Timber => 5,
        _ => 0,
    };
    if grit > 0 && gain > 0.03 {
        s.shower(grit, &[A, FLOOR], 0.3, 0.05, |r| decaying(r, 0.05, 0.2));
    }
    Some(s.voice(banks, gain, 0.6))
}

fn both_ends(i: usize) -> u8 {
    if i.is_multiple_of(2) {
        B
    } else {
        A
    }
}

// ---------------------------------------------------------------------------
// Members under load

/// How a material's joints and crack faces slip, and how much it cracks on
/// the way to failure.
#[derive(Clone, Copy, Debug)]
struct Friction {
    /// Change in utilisation that breaks a joint loose (mean).
    hold: f32,
    /// Change in utilisation it then slides for (mean).
    slide: f32,
    /// Change in utilisation per slip while sliding.
    slip: f32,
    /// Irregularity of the slips, 0 = a steady creak, 1 = grinding.
    rough: f32,
    /// Slip impulse.
    amp: f32,
    /// Slip duration, s.
    tau: f32,
    /// Micro-fractures over the whole way from intact to failure.
    cracks: f32,
    /// Level of the largest micro-fracture.
    crack_amp: f32,
}

fn friction(m: MaterialKind) -> Friction {
    match m {
        // Bolted steel: nothing, then a bolt slips into bearing with a
        // bang. One slip per event.
        MaterialKind::Steel => Friction {
            hold: 0.07,
            slide: 0.004,
            slip: 0.004,
            rough: 0.3,
            amp: 1.0,
            tau: 0.15e-3,
            cracks: 50.0,
            crack_amp: 0.4,
        },
        // Timber joints work all the time: dowels and bearing faces creak
        // in long, fairly regular slip trains.
        MaterialKind::Timber => Friction {
            hold: 0.022,
            slide: 0.02,
            slip: 0.0011,
            rough: 0.12,
            amp: 0.28,
            tau: 0.3e-3,
            cracks: 520.0,
            crack_amp: 1.3,
        },
        // Concrete only grinds once it has cracked (see `StrainVoice`).
        MaterialKind::Concrete => Friction {
            hold: 0.03,
            slide: 0.012,
            slip: 0.0016,
            rough: 0.7,
            amp: 0.35,
            tau: 0.6e-3,
            cracks: 420.0,
            crack_amp: 0.9,
        },
        // Bonded laminate: no joints to creak, only matrix cracks.
        MaterialKind::Composite => Friction {
            hold: 0.09,
            slide: 0.003,
            slip: 0.003,
            rough: 0.4,
            amp: 0.45,
            tau: 0.07e-3,
            cracks: 300.0,
            crack_amp: 0.8,
        },
    }
}

/// Most force events a straining member starts per block.
const MAX_EVENTS_PER_BLOCK: usize = 10;

/// One member under changing load, heard through its own modes.
///
/// Silent while the load is steady: a structure at rest makes no sound.
/// It is driven by two things the solver reports:
///
/// * **utilisation changing**: joints stick and slip (see `Friction`);
/// * **damage growing**: micro-fractures. Damage never decreases, so this
///   reproduces the Kaiser effect for free: reloading a member to a load it
///   has already seen is quiet, new ground is not. Events get more frequent
///   and the largest ones larger as failure nears.
pub struct StrainVoice {
    bank: Bank,
    body: Body,
    seed: u32,
    sr: f32,
    rng: Rng,
    law: Friction,
    micro: Micro,
    gain: f32,
    /// Targets from the game and the smoothed values the voice has reached.
    target_u: f32,
    target_d: f32,
    u: f32,
    d: f32,
    fresh: bool,
    /// Stick-slip state, in utilisation travelled.
    stuck: f32,
    hold: f32,
    sliding: f32,
    /// A recorded creak is playing for the current slide.
    creaking: bool,
    crept: f32,
    next_slip: f32,
    /// Expected micro-fractures not yet played.
    owed: f32,
    pulses: Pulses,
    /// Recorded events and what is heard of them directly.
    grains: Grains,
    heard: Vec<f32>,
    /// Section size, m: the pitch of the member's cracks.
    size: f32,
    imp: Vec<Vec<f32>>,
    step: Vec<Vec<f32>>,
    held: f32,
    leak: f32,
    direct: Vec<f32>,
    direct_lp: OnePole,
}

impl StrainVoice {
    pub fn new(body: &Body, sr: f32, seed: u32) -> Option<Self> {
        let body = sanitize(body)?;
        let law = friction(body.material);
        let micro = micro(body.material);
        let mut rng = Rng::new(seed);
        let drive = Drive {
            at: rng.range(0.03, 0.2),
            transverse: 0.8,
            axial: 0.6,
        };
        Some(Self {
            bank: Bank::new(&seated(&body, drive, STRAIN_MODES, seed), sr),
            body,
            seed,
            sr,
            hold: law.hold * rng.range(0.3, 1.0),
            next_slip: law.slip,
            rng,
            law,
            micro,
            gain: level(capacity_energy(&body)) * OUTPUT * 0.5,
            target_u: 0.0,
            target_d: 0.0,
            u: 0.0,
            d: 0.0,
            fresh: true,
            stuck: 0.0,
            sliding: 0.0,
            creaking: false,
            crept: 0.0,
            owed: 0.0,
            pulses: Pulses::default(),
            grains: Grains::default(),
            heard: Vec::new(),
            size: body.width.max(body.depth),
            imp: vec![Vec::new()],
            step: vec![Vec::new()],
            held: 0.0,
            leak: step_leak(sr),
            direct: Vec::new(),
            direct_lp: OnePole::new(micro.cutoff, sr),
        })
    }

    /// The solver's latest word on this member: `utilization` (1 = at
    /// capacity) and `damage` (0..1, never decreasing). `jump` is true when
    /// the voice has not been updated for a while: it then adopts the values
    /// without sounding the difference.
    pub fn set(&mut self, body: &Body, utilization: f32, damage: f32, jump: bool) {
        let u = finite(utilization, 0.0).clamp(0.0, 3.0);
        let d = finite(damage, 0.0).clamp(0.0, 1.0);
        self.target_u = u;
        self.target_d = d.max(self.target_d);
        if jump || self.fresh {
            self.u = u;
            self.d = self.target_d;
            self.fresh = false;
        }
        // A member that has cracked or yielded since rings duller.
        if (body.damage - self.body.damage).abs() > 0.2 {
            if let Some(b) = sanitize(body) {
                self.body = b;
                let drive = Drive {
                    at: 0.1,
                    transverse: 0.8,
                    axial: 0.6,
                };
                self.bank = Bank::new(&seated(&b, drive, STRAIN_MODES, self.seed), self.sr);
            }
        }
    }

    /// True when it has nothing left to play.
    pub fn quiet(&self) -> bool {
        self.pulses.is_empty()
            && self.grains.is_empty()
            && self.bank.level() < 2e-5
            && self.held.abs() < 1e-6
    }

    fn start(&mut self, offset: usize, tau: f32, amp: f32, direct: f32) {
        let len = (tau * self.sr).round().clamp(1.0, 4000.0) as u16;
        self.pulses.start(
            Hit {
                at: 0,
                len,
                to: 0,
                step: false,
                amp,
                direct,
            },
            offset as u32,
        );
    }

    /// Starts a recorded event `offset` samples into the block.
    fn record(&mut self, offset: usize, slice: timbre::Slice, rate: f32, gain: f32, limit: f32) {
        // Voices are cheap, but a member cannot say more than a few things
        // at once.
        if self.grains.len() >= 24 {
            return;
        }
        self.grains.start(
            GrainHit {
                at: 0,
                to: 0,
                slice,
                rate,
                gain,
                body: gain * BODY,
                limit,
            },
            offset as u32,
            self.sr,
        );
    }

    /// Turns this block's change in load and damage into force events.
    fn drive(&mut self, n: usize) {
        let dt = n as f32 / self.sr;
        // The solver steps at frame rate; follow it smoothly.
        let k = 1.0 - (-dt / 0.03).exp();
        let du = (self.target_u - self.u) * k;
        let dd = (self.target_d - self.d) * k;
        self.u += du;
        self.d += dd;
        let mut budget = MAX_EVENTS_PER_BLOCK;
        let law = self.law;

        // Stick-slip. Concrete has nothing to slip on until it has cracked.
        let travel = du.abs();
        let can_slip = self.body.material != MaterialKind::Concrete || self.d > 0.25;
        if travel > 0.0 && can_slip {
            // Friction force follows the normal force, which follows load.
            let force = 0.35 + self.u.min(1.5);
            if self.sliding <= 0.0 {
                self.stuck += travel;
                if self.stuck >= self.hold {
                    self.stuck = 0.0;
                    self.hold = law.hold * (0.25 - self.rng.unit().max(1e-3).ln());
                    self.sliding = law.slide * self.rng.range(0.5, 1.5);
                    self.crept = self.next_slip;
                    // The joint lets go. If there is a recording of this
                    // material straining, the slide plays a stretch of it:
                    // as long as the slide will take at this loading rate,
                    // and higher the faster the load moves.
                    let speed = travel / dt;
                    let seconds = (self.sliding / speed.max(1e-4)).clamp(0.15, 1.5);
                    let what = timbre::strain_of(self.body.material);
                    self.creaking = false;
                    // Bonded laminate has no joints to creak.
                    let jointed = self.body.material != MaterialKind::Composite;
                    if jointed && timbre::recorded(what) {
                        if let Some(slice) = timbre::stretch(what, seconds, &mut self.rng) {
                            let rate = timbre::size_rate(what, self.size)
                                * (speed / 0.2).clamp(0.2, 5.0).powf(0.2);
                            self.record(0, slice, rate, 0.6 * law.amp.max(0.5) * force, seconds);
                            self.creaking = true;
                        }
                    }
                }
            }
            if self.sliding > 0.0 {
                self.sliding -= travel;
                let before = self.crept;
                self.crept += travel;
                while self.crept >= self.next_slip && budget > 0 {
                    // Where in the block the threshold was crossed: steady
                    // sliding gives evenly spaced slips, which is the pitch
                    // of a creak.
                    let frac = ((self.next_slip - before) / travel).clamp(0.0, 0.999);
                    let size = self.next_slip / law.slip;
                    let amp = law.amp * size * force;
                    let offset = (frac * n as f32) as usize;
                    // A bolt slipping into bearing is a knock: play a
                    // recorded one. A train of slips under a recorded creak
                    // only shakes the member; without a recording the
                    // computed slips are all there is.
                    let knock = self.body.material == MaterialKind::Steel;
                    let what = timbre::crack_of(self.body.material);
                    let slice = if knock {
                        timbre::event(what, &mut self.rng)
                    } else {
                        None
                    };
                    if let Some(slice) = slice {
                        let rate = timbre::size_rate(what, self.size) * self.rng.range(0.9, 1.1);
                        self.record(offset, slice, rate, 0.6 * amp, 0.5);
                        self.start(offset, law.tau, 0.3 * amp, 0.0);
                    } else if self.creaking {
                        self.start(offset, law.tau, 0.3 * amp, 0.0);
                    } else {
                        self.start(offset, law.tau, amp, amp * 0.5 * self.micro.direct);
                    }
                    self.crept -= self.next_slip;
                    self.next_slip = law.slip * (1.0 + law.rough * self.rng.bipolar()).max(0.2);
                    budget -= 1;
                }
                if budget == 0 {
                    self.crept = self.crept.min(self.next_slip);
                }
            }
        }

        // Micro-fractures from new damage: sparse and small early, dense
        // with rare large ones close to failure.
        if dd > 0.0 {
            let d = self.d;
            self.owed += law.cracks * dd * (0.25 + 1.5 * d * d);
            let cap = 0.01 + 0.99 * d * d * d;
            while self.owed >= 1.0 && budget > 0 {
                self.owed -= 1.0;
                let e = avalanche(&mut self.rng, self.micro.beta, cap);
                let amp = law.crack_amp * e.sqrt();
                let offset = ((self.rng.unit() * n as f32) as usize).min(n - 1);
                let what = timbre::crack_of(self.body.material);
                if let Some(slice) = timbre::event(what, &mut self.rng) {
                    let rate = timbre::size_rate(what, self.size)
                        * e.max(1e-3).powf(-0.08)
                        * self.rng.range(0.9, 1.1);
                    self.record(offset, slice, rate, amp * self.micro.direct, MICRO_LIMIT);
                    // A recorded crack is a small avalanche already.
                    self.owed -= 3.0;
                } else {
                    self.start(
                        offset,
                        self.micro.tau * e.cbrt(),
                        amp,
                        amp * self.micro.direct,
                    );
                }
                budget -= 1;
            }
            self.owed = self.owed.clamp(-3.0, 4.0);
        }
    }
}

impl Voice for StrainVoice {
    fn render(&mut self, out: &mut [f32]) -> bool {
        let n = out.len();
        if n == 0 {
            return true;
        }
        self.drive(n);
        if self.quiet() {
            return true;
        }
        for b in [
            &mut self.imp[0],
            &mut self.step[0],
            &mut self.direct,
            &mut self.heard,
        ] {
            b.clear();
            b.resize(n, 0.0);
        }
        self.pulses
            .render(n, &mut self.imp, &mut self.step, &mut self.direct);
        self.grains.render(n, &mut self.imp, &mut self.heard);
        for (d, h) in self.direct.iter_mut().zip(&self.heard) {
            *d = 0.5 * self.direct_lp.lp(*d + 1e-18) + h;
        }
        let scale = self.bank.step_scale();
        for (x, &st) in self.imp[0].iter_mut().zip(&self.step[0]) {
            self.held = self.held * self.leak + st * scale;
            *x += self.held;
        }
        self.bank.process(&self.imp[0], &mut self.direct);
        for (o, &d) in out.iter_mut().zip(&self.direct) {
            *o += d * self.gain;
        }
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SR: f32 = 48_000.0;
    const ALL: [MaterialKind; 4] = [
        MaterialKind::Steel,
        MaterialKind::Timber,
        MaterialKind::Concrete,
        MaterialKind::Composite,
    ];

    fn render(mut v: BoxVoice, seconds: f32) -> (Vec<f32>, bool) {
        let mut out = Vec::new();
        let mut alive = true;
        while alive && out.len() < (seconds * SR) as usize {
            let mut buf = [0.0f32; 128];
            alive = v.render(&mut buf);
            out.extend_from_slice(&buf);
        }
        (out, alive)
    }

    fn peak(x: &[f32]) -> f32 {
        x.iter().fold(0.0f32, |m, v| m.max(v.abs()))
    }

    fn rms(x: &[f32]) -> f32 {
        (x.iter().map(|v| v * v).sum::<f32>() / x.len().max(1) as f32).sqrt()
    }

    /// Share of the energy above `hz`, by a one-pole split.
    fn brightness(x: &[f32], hz: f32) -> f32 {
        let mut lp = OnePole::new(hz, SR);
        let (mut lo, mut hi) = (0.0, 0.0);
        for &v in x {
            let l = lp.lp(v);
            lo += l * l;
            hi += (v - l) * (v - l);
        }
        hi / (lo + hi + 1e-20)
    }

    fn info(m: MaterialKind, cause: BreakCause) -> BreakInfo {
        let body = Body::typical(m, 3.0);
        BreakInfo {
            body,
            cause,
            at: 0.4,
            energy: capacity_energy(&body),
            axial: 0.5,
        }
    }

    const CAUSES: [(MaterialKind, BreakCause); 11] = [
        (MaterialKind::Steel, BreakCause::Rupture),
        (MaterialKind::Steel, BreakCause::Hinge),
        (MaterialKind::Steel, BreakCause::Connection),
        (MaterialKind::Timber, BreakCause::Splinter),
        (MaterialKind::Timber, BreakCause::TimberSplit),
        (MaterialKind::Timber, BreakCause::Crushing),
        (MaterialKind::Concrete, BreakCause::Brittle),
        (MaterialKind::Concrete, BreakCause::ConcreteShear),
        (MaterialKind::Concrete, BreakCause::RebarRupture),
        (MaterialKind::Composite, BreakCause::FibreRupture),
        (MaterialKind::Composite, BreakCause::Delamination),
    ];

    #[test]
    fn every_break_sounds_ends_and_stays_finite() {
        for (m, cause) in CAUSES {
            let v = break_voice(&info(m, cause), 1.0, SR, 3).unwrap();
            let (x, alive) = render(v, 10.5);
            assert!(!alive, "{m:?} {cause:?} never ended");
            assert!(x.iter().all(|v| v.is_finite()), "{m:?} {cause:?}");
            let p = peak(&x);
            assert!(p > 0.03 && p < 6.0, "{m:?} {cause:?} peak {p}");
        }
    }

    #[test]
    fn a_member_in_a_frame_does_not_ring_on() {
        // A bare steel bar rings for seconds, which is what made the first
        // version sound like glass. Built into a frame it is over quickly.
        let (x, _) = render(
            break_voice(&info(MaterialKind::Steel, BreakCause::Rupture), 1.0, SR, 5).unwrap(),
            2.5,
        );
        let at = (1.5 * SR) as usize;
        let tail = if x.len() > at { rms(&x[at..]) } else { 0.0 };
        let onset = rms(&x[..(0.1 * SR) as usize]);
        assert!(onset > 1e-3, "{onset}");
        assert!(tail < 0.05 * onset, "tail {tail} vs onset {onset}");
    }

    #[test]
    fn more_energy_is_louder_but_compressed() {
        let loud = |e: f32| {
            let mut i = info(MaterialKind::Steel, BreakCause::Rupture);
            i.energy = e;
            rms(&render(break_voice(&i, 1.0, SR, 9).unwrap(), 0.3).0)
        };
        let ratio = loud(4000.0) / loud(250.0);
        // 16x the energy: 4x by physics, 2.3x after compression.
        assert!(ratio > 1.8 && ratio < 3.0, "{ratio}");
    }

    #[test]
    fn an_axial_break_is_pitched_at_the_bar_wave_frequency() {
        // A steel tie that snaps rings at multiples of c / 4L of its
        // pieces. Count the strongest low partial by correlation.
        let mut i = info(MaterialKind::Steel, BreakCause::Rupture);
        i.axial = 1.0;
        i.at = 0.5;
        let (x, _) = render(break_voice(&i, 1.0, SR, 2).unwrap(), 0.5);
        let x = &x[(0.05 * SR) as usize..];
        let power = |f: f32| {
            let (mut s, mut c) = (0.0f32, 0.0f32);
            for (n, v) in x.iter().enumerate() {
                let p = std::f32::consts::TAU * f * n as f32 / SR;
                s += v * p.sin();
                c += v * p.cos();
            }
            s * s + c * c
        };
        let piece = i.body.piece(0.5, Ends::Cantilever);
        let f1 = piece.wave_speed() / (4.0 * piece.length);
        let on = (-6..=6)
            .map(|k| power(f1 * (1.0 + 0.002 * k as f32)))
            .fold(0.0, f32::max);
        let off = power(f1 * 1.37).max(power(f1 * 0.71));
        // The recorded snap is heard over it, so the margin is modest.
        assert!(on > 4.0 * off, "axial partial {on} vs {off}");
    }

    #[test]
    fn impacts_follow_contact_physics() {
        // Steel on the floor: sub-millisecond. Timber: milliseconds.
        let steel = contact_time(5.0, 3.0, 0.004, contact(MaterialKind::Steel).0);
        let timber = contact_time(5.0, 3.0, 0.004, contact(MaterialKind::Timber).0);
        assert!(steel > 0.2e-3 && steel < 1.5e-3, "{steel}");
        assert!(timber > 3.0 * steel, "{timber}");
        // Faster is shorter.
        assert!(contact_time(5.0, 9.0, 0.004, 1e10) < contact_time(5.0, 1.0, 0.004, 1e10));

        // A steel piece rings as a free bar under the contact pulse alone,
        // so a harder landing, with its shorter contact, is brighter.
        let hit = |speed: f32| {
            let body = Body::typical(MaterialKind::Steel, 1.5);
            let mass = body.mass();
            render(impact_voice(&body, speed, mass, 1.0, SR, 4).unwrap(), 3.0).0
        };
        let (soft, hard) = (hit(0.7), hit(12.0));
        assert!(
            brightness(&hard, 1500.0) > 1.2 * brightness(&soft, 1500.0),
            "hard {} soft {}",
            brightness(&hard, 1500.0),
            brightness(&soft, 1500.0)
        );
        // The other materials land with a recorded knock.
        for (m, mass) in [(MaterialKind::Timber, 6.0), (MaterialKind::Concrete, 40.0)] {
            assert!(impact_voice(&Body::fragment(m, 0.4), 4.0, mass, 1.0, SR, 4).is_some());
        }
    }

    fn strain(m: MaterialKind, path: impl Fn(f32) -> (f32, f32), seconds: f32) -> Vec<f32> {
        let body = Body::typical(m, 3.0);
        let mut v = StrainVoice::new(&body, SR, 11).unwrap();
        let mut out = Vec::new();
        let blocks = (seconds * SR / 128.0) as usize;
        for b in 0..blocks {
            let t = b as f32 * 128.0 / SR;
            // The game updates at 60 Hz.
            if b % 6 == 0 {
                let (u, d) = path(t);
                v.set(&body, u, d, false);
            }
            let mut buf = [0.0f32; 128];
            v.render(&mut buf);
            out.extend_from_slice(&buf);
        }
        out
    }

    #[test]
    fn a_steady_load_is_silent() {
        for m in ALL {
            let x = strain(m, |_| (0.9, 0.9), 2.0);
            // Adopted on the first update: nothing to say.
            assert!(peak(&x) < 1e-4, "{m:?} {}", peak(&x));
        }
    }

    #[test]
    fn loading_is_heard_and_reloading_is_quieter() {
        for m in ALL {
            // Up to 0.9, back to 0.2, up to 0.9 again; damage follows the
            // peak (Kaiser effect).
            let path = |t: f32| {
                let u = if t < 3.0 {
                    0.3 * t
                } else if t < 5.0 {
                    0.9 - 0.35 * (t - 3.0)
                } else {
                    (0.2 + 0.35 * (t - 5.0)).min(0.9)
                };
                (u, (0.3 * t).min(0.9))
            };
            let x = strain(m, path, 7.0);
            assert!(x.iter().all(|v| v.is_finite()));
            let first = rms(&x[..(3.0 * SR) as usize]);
            let again = rms(&x[(5.0 * SR) as usize..]);
            assert!(first > 1e-4, "{m:?} first loading silent: {first}");
            assert!(peak(&x) < 3.0, "{m:?} {}", peak(&x));
            // Steel is the exception: its sound is bolts slipping, and they
            // slip again on the way back up.
            if m != MaterialKind::Steel {
                assert!(again < 0.9 * first, "{m:?} reload {again} vs {first}");
            }
        }
    }

    #[test]
    fn nearer_failure_is_busier() {
        for m in [
            MaterialKind::Timber,
            MaterialKind::Concrete,
            MaterialKind::Composite,
        ] {
            let x = strain(m, |t| (0.25 * t, (0.25 * t).min(1.0)), 4.0);
            let early = rms(&x[..(1.5 * SR) as usize]);
            let late = rms(&x[(2.5 * SR) as usize..]);
            assert!(late > 1.4 * early, "{m:?} {early} -> {late}");
        }
    }

    #[test]
    fn a_creak_rises_in_pitch_with_loading_rate() {
        // Slips per second while sliding = rate / slip distance.
        let count = |rate: f32| {
            let x = strain(MaterialKind::Timber, |t| (0.1 + rate * t, 0.0), 2.0);
            // Onsets: samples where the envelope jumps.
            let mut env = 0.0f32;
            let mut onsets = 0;
            let mut armed = true;
            for v in &x {
                let a = v.abs();
                if a > 4.0 * env + 1e-4 && armed {
                    onsets += 1;
                    armed = false;
                }
                env = 0.995 * env + 0.005 * a;
                if a < 1.5 * env {
                    armed = true;
                }
            }
            onsets
        };
        let (slow, fast) = (count(0.1), count(0.4));
        assert!(slow > 5, "{slow}");
        assert!(fast as f32 > 1.3 * slow as f32, "{slow} -> {fast}");
    }

    #[test]
    fn bad_numbers_make_no_voice() {
        let mut body = Body::typical(MaterialKind::Steel, 3.0);
        body.area = f32::NAN;
        assert!(impact_voice(&body, 3.0, 5.0, 1.0, SR, 1).is_none());
        assert!(StrainVoice::new(&body, SR, 1).is_none());
        body.area = 0.0;
        assert!(damage_voice(&body, DamageState::Cracked, 10.0, SR, 1).is_none());
        let good = Body::typical(MaterialKind::Steel, 3.0);
        let v = break_voice(
            &BreakInfo {
                body: good,
                cause: BreakCause::Rupture,
                at: f32::NAN,
                energy: f32::INFINITY,
                axial: f32::NAN,
            },
            1.0,
            SR,
            1,
        );
        let (x, _) = render(v.unwrap(), 1.0);
        assert!(x.iter().all(|v| v.is_finite()));
    }
}
