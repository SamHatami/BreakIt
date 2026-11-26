//! Interface sounds.
//!
//! The UI is the lab's paperwork: drafting paper, instrument switches,
//! relays and rubber stamps. So the sounds are tactile and mechanical
//! (struck modes, short noise bursts, a few damped tones), not synth bleeps.
//! Each sound is a short list of timed parts and ends within 1.5 s.

use crate::dsp::{Decay, ModalBank, OnePole, Osc, Rng, Svf};
use crate::{BoxVoice, UiSound, Voice};

enum Part {
    /// Struck modes plus a short noise burst: clicks, clacks, thuds.
    Strike {
        start: usize,
        end: usize,
        modes: ModalBank,
        noise: Decay,
        noise_amp: f32,
        noise_filter: Svf,
        amp: f32,
    },
    /// Band-passed noise with an attack and decay, roughened: scrapes.
    Scrape {
        start: usize,
        len: usize,
        band: Svf,
        rough: OnePole,
        amp: f32,
    },
    /// A held tone: sine with a few harmonics or a filtered saw, with a
    /// linear attack / release and an optional pitch glide.
    Tone {
        start: usize,
        len: usize,
        freq: f32,
        glide: f32,
        harmonics: &'static [f32],
        saw: bool,
        lp: OnePole,
        lp2: OnePole,
        osc: Vec<Osc>,
        amp: f32,
        attack: usize,
        release: usize,
    },
}

struct UiVoice {
    sr: f32,
    t: usize,
    end: usize,
    rng: Rng,
    parts: Vec<Part>,
}

struct Builder {
    sr: f32,
    rng: Rng,
    parts: Vec<Part>,
    end: usize,
}

impl Builder {
    fn s(&self, secs: f32) -> usize {
        (secs * self.sr) as usize
    }

    /// `modes` (freq, t60, gain); `noise` (amp, t60, filter centre).
    fn strike(
        &mut self,
        at: f32,
        amp: f32,
        modes: &[(f32, f32, f32)],
        noise: (f32, f32, f32),
    ) -> &mut Self {
        let sr = self.sr;
        let modes: Vec<_> = modes
            .iter()
            .map(|&(f, t, g)| (self.rng.vary(f, 0.02), t, g))
            .collect();
        let tail = modes.iter().map(|m| m.1).fold(noise.1, f32::max);
        let start = self.s(at);
        self.end = self.end.max(start + self.s(tail * 1.1));
        self.parts.push(Part::Strike {
            start,
            end: start + self.s(tail * 1.1),
            modes: ModalBank::new(&modes, sr),
            noise: Decay::new(noise.1, sr),
            noise_amp: noise.0,
            noise_filter: Svf::new(noise.2, 0.8, sr),
            amp,
        });
        self
    }

    fn scrape(&mut self, at: f32, len: f32, centre: f32, amp: f32) -> &mut Self {
        let start = self.s(at);
        let len = self.s(len);
        self.end = self.end.max(start + len);
        self.parts.push(Part::Scrape {
            start,
            len,
            band: Svf::new(centre, 1.0, self.sr),
            rough: OnePole::new(60.0, self.sr),
            amp,
        });
        self
    }

    #[allow(clippy::too_many_arguments)]
    fn tone(
        &mut self,
        at: f32,
        len: f32,
        freq: f32,
        glide: f32,
        harmonics: &'static [f32],
        saw: bool,
        lp: f32,
        amp: f32,
    ) -> &mut Self {
        let start = self.s(at);
        let n = self.s(len);
        self.end = self.end.max(start + n);
        let osc = (0..harmonics.len().max(1))
            .map(|_| Osc::new(self.rng.unit()))
            .collect();
        self.parts.push(Part::Tone {
            start,
            len: n,
            freq,
            glide,
            harmonics,
            saw,
            lp: OnePole::new(lp, self.sr),
            lp2: OnePole::new(lp * 1.5, self.sr),
            osc,
            amp,
            attack: self.s(0.006).max(1),
            release: self.s((len * 0.35).min(0.12)).max(1),
        });
        self
    }

    /// Rubber stamp: a felt-damped thud on the desk and the paper's slap.
    fn stamp(&mut self, at: f32, amp: f32) -> &mut Self {
        self.strike(
            at,
            amp,
            &[(105.0, 0.12, 1.0), (390.0, 0.07, 0.35), (870.0, 0.04, 0.15)],
            (0.5, 0.03, 900.0),
        )
        .strike(
            at + 0.004,
            amp * 0.4,
            &[(2_600.0, 0.015, 0.4)],
            (0.8, 0.012, 3_200.0),
        )
    }
}

/// Builds a one-shot for an interface sound.
pub fn ui_voice(sound: UiSound, sr: f32, seed: u32) -> BoxVoice {
    let mut b = Builder {
        sr,
        rng: Rng::new(seed ^ 0x0071_0003),
        parts: Vec::new(),
        end: 0,
    };
    const SINE: &[f32] = &[1.0];
    const SOFT: &[f32] = &[1.0, 0.25, 0.08];
    match sound {
        UiSound::Hover => {
            b.strike(
                0.0,
                0.03,
                &[(4_300.0, 0.012, 1.0), (6_200.0, 0.008, 0.5)],
                (0.3, 0.003, 5_000.0),
            );
        }
        UiSound::PlaceNode => {
            // Push-button switch: press, then the lighter release.
            b.strike(
                0.0,
                0.17,
                &[
                    (2_400.0, 0.03, 1.0),
                    (3_900.0, 0.025, 0.6),
                    (5_600.0, 0.02, 0.35),
                ],
                (0.6, 0.004, 4_000.0),
            )
            .strike(
                0.014,
                0.1,
                &[(2_900.0, 0.02, 1.0), (4_700.0, 0.015, 0.5)],
                (0.4, 0.003, 5_000.0),
            );
        }
        UiSound::PlaceBeam => {
            // A steel ruler laid down on the board.
            b.strike(
                0.0,
                0.16,
                &[
                    (850.0, 0.07, 1.0),
                    (1_720.0, 0.05, 0.6),
                    (2_930.0, 0.04, 0.4),
                    (4_400.0, 0.03, 0.2),
                ],
                (0.5, 0.008, 2_500.0),
            )
            .strike(0.0, 0.12, &[(175.0, 0.06, 1.0)], (0.3, 0.01, 600.0))
            .strike(
                0.022,
                0.09,
                &[(1_100.0, 0.04, 1.0), (2_300.0, 0.03, 0.5)],
                (0.3, 0.004, 3_000.0),
            );
        }
        UiSound::Delete => {
            // Eraser, two strokes.
            b.scrape(0.0, 0.09, 1_300.0, 1.0)
                .scrape(0.1, 0.07, 1_000.0, 0.7);
        }
        UiSound::Select => {
            // Rotary switch: spring wipe into the detent.
            b.strike(0.0, 0.08, &[(3_400.0, 0.012, 1.0)], (0.5, 0.004, 4_500.0))
                .strike(
                    0.018,
                    0.22,
                    &[
                        (1_850.0, 0.025, 1.0),
                        (3_300.0, 0.018, 0.5),
                        (650.0, 0.03, 0.3),
                    ],
                    (0.5, 0.004, 3_000.0),
                );
        }
        UiSound::Support => {
            b.stamp(0.0, 0.35);
        }
        UiSound::Invalid => {
            // Dull double buzz-thud.
            const BUZZ: &[f32] = &[1.0];
            b.tone(0.0, 0.08, 110.0, 1.0, BUZZ, true, 500.0, 0.22)
                .strike(0.0, 0.18, &[(95.0, 0.08, 1.0)], (0.2, 0.01, 400.0))
                .tone(0.12, 0.08, 104.0, 1.0, BUZZ, true, 450.0, 0.2)
                .strike(0.12, 0.16, &[(90.0, 0.08, 1.0)], (0.2, 0.01, 400.0));
        }
        UiSound::Undo => {
            // Tick-tick, high then low, the first one swelling in.
            b.scrape(0.0, 0.03, 3_600.0, 0.3)
                .strike(
                    0.03,
                    0.15,
                    &[(3_800.0, 0.02, 1.0), (5_900.0, 0.012, 0.4)],
                    (0.4, 0.003, 5_000.0),
                )
                .strike(
                    0.09,
                    0.15,
                    &[(2_600.0, 0.025, 1.0), (4_100.0, 0.015, 0.4)],
                    (0.4, 0.003, 4_000.0),
                );
        }
        UiSound::TestStart => {
            // Contactor pulling in, then a two-tone lab warning.
            b.strike(
                0.0,
                0.2,
                &[
                    (300.0, 0.12, 1.0),
                    (760.0, 0.08, 0.6),
                    (1_620.0, 0.05, 0.35),
                    (3_100.0, 0.03, 0.2),
                ],
                (0.6, 0.01, 1_800.0),
            )
            .strike(0.0, 0.25, &[(90.0, 0.1, 1.0)], (0.2, 0.01, 300.0))
            .tone(0.18, 0.22, 784.0, 1.0, SOFT, false, 3_000.0, 0.11)
            .tone(0.42, 0.26, 622.0, 1.0, SOFT, false, 3_000.0, 0.11);
        }
        UiSound::TestStop => {
            // Contactor dropping out: lighter, with a little spring buzz.
            b.strike(
                0.0,
                0.22,
                &[
                    (520.0, 0.08, 1.0),
                    (1_230.0, 0.06, 0.6),
                    (2_450.0, 0.04, 0.3),
                ],
                (0.5, 0.008, 2_200.0),
            )
            .strike(0.03, 0.05, &[(3_700.0, 0.06, 1.0)], (0.0, 0.001, 3_000.0));
        }
        UiSound::Pass => {
            // Stamp, then a bright two-note bell.
            b.stamp(0.0, 0.3);
            for (at, f, a) in [(0.13, 880.0, 0.16), (0.24, 1_318.5, 0.14)] {
                b.strike(
                    at,
                    a,
                    &[
                        (f, 0.9, 1.0),
                        (f * 2.0, 0.55, 0.3),
                        (f * 2.76, 0.35, 0.15),
                        (f * 5.4, 0.15, 0.05),
                    ],
                    (0.0, 0.001, 1_000.0),
                );
            }
        }
        UiSound::Fail => {
            // Stamp, then a short, muted low horn sagging in pitch.
            b.stamp(0.0, 0.32)
                .tone(0.13, 0.45, 131.0, 0.94, SINE, true, 650.0, 0.2);
        }
    }
    let end = b.end + b.s(0.02);
    Box::new(UiVoice {
        sr,
        t: 0,
        end,
        rng: b.rng,
        parts: b.parts,
    })
}

impl Voice for UiVoice {
    fn render(&mut self, out: &mut [f32]) -> bool {
        let sr = self.sr;
        for o in out.iter_mut() {
            let t = self.t;
            let mut y = 0.0;
            for p in &mut self.parts {
                match p {
                    Part::Strike {
                        start,
                        end,
                        modes,
                        noise,
                        noise_amp,
                        noise_filter,
                        amp,
                    } => {
                        if t < *start || t >= *end {
                            continue;
                        }
                        let imp = if t == *start { 1.0 } else { 1e-15 };
                        let env = if noise.level() > 1e-6 {
                            noise.next()
                        } else {
                            0.0
                        };
                        let n = noise_filter.bp(self.rng.bipolar()) * env * *noise_amp;
                        y += (modes.process(imp + n * 0.05) + n) * *amp;
                    }
                    Part::Scrape {
                        start,
                        len,
                        band,
                        rough,
                        amp,
                    } => {
                        if t < *start || t >= *start + *len {
                            continue;
                        }
                        let x = (t - *start) as f32 / *len as f32;
                        let env = (x / 0.3).min(1.0) * (1.0 - x).powi(2);
                        let grain = 0.5 + 4.0 * rough.lp(self.rng.bipolar()).abs();
                        y += band.bp(self.rng.bipolar()) * env * grain * *amp;
                    }
                    Part::Tone {
                        start,
                        len,
                        freq,
                        glide,
                        harmonics,
                        saw,
                        lp,
                        lp2,
                        osc,
                        amp,
                        attack,
                        release,
                    } => {
                        if t < *start || t >= *start + *len {
                            continue;
                        }
                        let i = t - *start;
                        let env = (i as f32 / *attack as f32).min(1.0)
                            * ((*len - i) as f32 / *release as f32).min(1.0);
                        let f = *freq * (1.0 + (*glide - 1.0) * i as f32 / *len as f32);
                        let mut s = 0.0;
                        if *saw {
                            s = osc[0].saw(f, sr);
                        } else {
                            for (k, (o, g)) in osc.iter_mut().zip(harmonics.iter()).enumerate() {
                                s += o.sine(f * (k + 1) as f32, sr) * g;
                            }
                        }
                        y += lp2.lp(lp.lp(s)) * env * *amp;
                    }
                }
            }
            *o += y;
            self.t += 1;
        }
        self.t < self.end
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const ALL: [UiSound; 12] = [
        UiSound::Hover,
        UiSound::PlaceNode,
        UiSound::PlaceBeam,
        UiSound::Delete,
        UiSound::Select,
        UiSound::Support,
        UiSound::Invalid,
        UiSound::Undo,
        UiSound::TestStart,
        UiSound::TestStop,
        UiSound::Pass,
        UiSound::Fail,
    ];

    #[test]
    fn ui_sounds_terminate_and_peak_in_range() {
        let sr = 48_000.0;
        for s in ALL {
            let mut v = ui_voice(s, sr, 11);
            let mut out = Vec::new();
            let mut alive = true;
            while alive && out.len() < (sr * 3.0) as usize {
                let mut buf = vec![0.0; 128];
                alive = v.render(&mut buf);
                out.extend(buf);
            }
            assert!(!alive, "{s:?} did not finish");
            let secs = out.len() as f32 / sr;
            let peak = out.iter().fold(0.0f32, |m, x| m.max(x.abs()));
            println!("{s:?}: {secs:.2} s, peak {peak:.3}");
            assert!(secs <= 1.5, "{s:?} lasts {secs} s");
            assert!(out.iter().all(|x| x.is_finite()));
            if s == UiSound::Hover {
                assert!(peak <= 0.05 && peak > 0.003, "hover peak {peak}");
            } else {
                assert!((0.08..=0.45).contains(&peak), "{s:?} peak {peak}");
            }
        }
    }
}
