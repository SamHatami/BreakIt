//! Recorded timbre: what a crack, a groan or a clang actually sounds like.
//!
//! The solver steps tens of times a second. It knows that a member broke,
//! how, and with how much energy, but nothing about the milliseconds in
//! which fibres tear or a weld rips, and that is where the ear tells wood
//! from steel. So that part is not computed: it is taken from recordings of
//! the real thing (or clips made by an audio model), one folder per kind of
//! event under `samples/`.
//!
//! The recordings are never played as they are. The simulation decides
//! which event happens, when, how many times and how hard; it picks a slice,
//! sets its level from the energy released and its pitch from the size of
//! the member, and feeds the same slice into the member's own modes as the
//! force that rings them (see [`crate::material`]).
//!
//! Files are 16-bit WAV, any rate, mono (the first channel is used).
//! `build.rs` indexes whatever is in the folders, so adding a file needs no
//! code. A timbre whose folder is empty borrows the closest free recording
//! from `samples/cc0/`.

use std::sync::{Arc, OnceLock};

use crate::dsp::Rng;
use crate::MaterialKind;

include!(concat!(env!("OUT_DIR"), "/sample_index.rs"));

/// One kind of recorded event.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Timbre {
    /// A thick steel bar snapping in tension.
    SteelSnap,
    /// A steel beam bending and buckling slowly.
    SteelGroan,
    /// A bolt slipping into bearing.
    SteelKnock,
    /// A weld or bolt group tearing.
    SteelTear,
    /// A heavy steel member landing.
    SteelDropHeavy,
    /// A short steel piece landing.
    SteelDropLight,
    /// A timber beam creaking under load.
    TimberCreak,
    /// One dry crack of a beam starting to split.
    TimberCrack,
    /// A beam snapping and splintering.
    TimberBreak,
    /// Wood fibres crushing.
    TimberCrush,
    /// A plank landing.
    TimberDrop,
    /// One crack in concrete.
    ConcreteCrack,
    /// A concrete beam breaking and crumbling.
    ConcreteBreak,
    /// Concrete grinding and crushing.
    ConcreteCrush,
    /// A reinforcement bar snapping inside concrete.
    RebarSnap,
    /// Rubble landing.
    Rubble,
    /// A carbon fibre tube snapping.
    CarbonSnap,
    /// Laminate delaminating: fast dry crackle.
    CarbonCrackle,
    /// A carbon tube landing.
    CarbonDrop,
}

pub const TIMBRES: [Timbre; 19] = [
    Timbre::SteelSnap,
    Timbre::SteelGroan,
    Timbre::SteelKnock,
    Timbre::SteelTear,
    Timbre::SteelDropHeavy,
    Timbre::SteelDropLight,
    Timbre::TimberCreak,
    Timbre::TimberCrack,
    Timbre::TimberBreak,
    Timbre::TimberCrush,
    Timbre::TimberDrop,
    Timbre::ConcreteCrack,
    Timbre::ConcreteBreak,
    Timbre::ConcreteCrush,
    Timbre::RebarSnap,
    Timbre::Rubble,
    Timbre::CarbonSnap,
    Timbre::CarbonCrackle,
    Timbre::CarbonDrop,
];

impl Timbre {
    /// Folder under `samples/`.
    pub fn folder(self) -> &'static str {
        match self {
            Timbre::SteelSnap => "steel_snap",
            Timbre::SteelGroan => "steel_groan",
            Timbre::SteelKnock => "steel_knock",
            Timbre::SteelTear => "steel_tear",
            Timbre::SteelDropHeavy => "steel_drop_heavy",
            Timbre::SteelDropLight => "steel_drop_light",
            Timbre::TimberCreak => "timber_creak",
            Timbre::TimberCrack => "timber_crack",
            Timbre::TimberBreak => "timber_break",
            Timbre::TimberCrush => "timber_crush",
            Timbre::TimberDrop => "timber_drop",
            Timbre::ConcreteCrack => "concrete_crack",
            Timbre::ConcreteBreak => "concrete_break",
            Timbre::ConcreteCrush => "concrete_crush",
            Timbre::RebarSnap => "rebar_snap",
            Timbre::Rubble => "rubble",
            Timbre::CarbonSnap => "carbon_snap",
            Timbre::CarbonCrackle => "carbon_crackle",
            Timbre::CarbonDrop => "carbon_drop",
        }
    }

    /// Free recordings that stand in while the folder is empty, and the
    /// playback rate that brings them closest (a stick is not a beam).
    fn stand_in(self) -> (&'static [&'static str], f32) {
        match self {
            Timbre::SteelSnap => (&["cc0/metal_heavy", "cc0/metal_hit"], 0.8),
            Timbre::SteelGroan => (&[], 1.0),
            Timbre::SteelKnock => (&["cc0/metal_heavy"], 0.9),
            Timbre::SteelTear => (&["cc0/metal_hit"], 0.8),
            Timbre::SteelDropHeavy => (&["cc0/metal_fall", "cc0/metal_heavy"], 0.8),
            Timbre::SteelDropLight => (&["cc0/metal_hit", "cc0/metal_fall"], 1.0),
            Timbre::TimberCreak => (&[], 1.0),
            Timbre::TimberCrack => (&["cc0/wood_crack"], 0.85),
            Timbre::TimberBreak => (&["cc0/wood_break", "cc0/wood_crack"], 0.75),
            Timbre::TimberCrush => (&["cc0/crunch"], 0.9),
            Timbre::TimberDrop => (&["cc0/wood_hit"], 0.85),
            Timbre::ConcreteCrack => (&["cc0/rock_break"], 1.0),
            Timbre::ConcreteBreak => (&["cc0/rock_break"], 0.8),
            Timbre::ConcreteCrush => (&["cc0/crunch", "cc0/rock_fall"], 0.8),
            Timbre::RebarSnap => (&["cc0/metal_hit"], 1.1),
            Timbre::Rubble => (&["cc0/rock_fall"], 0.9),
            Timbre::CarbonSnap => (&["cc0/wood_break"], 1.3),
            Timbre::CarbonCrackle => (&["cc0/wood_crack"], 1.4),
            Timbre::CarbonDrop => (&["cc0/wood_hit"], 1.3),
        }
    }

    /// Size of the thing in the recordings, m: the section depth for
    /// cracks and breaks, the length for a piece landing. A member of
    /// another size plays the slice faster or slower.
    pub fn reference_size(self) -> f32 {
        match self {
            Timbre::SteelSnap | Timbre::SteelTear | Timbre::SteelKnock => 0.06,
            Timbre::SteelGroan => 0.2,
            Timbre::SteelDropHeavy => 3.0,
            Timbre::SteelDropLight => 0.6,
            Timbre::TimberCreak | Timbre::TimberCrack | Timbre::TimberBreak => 0.15,
            Timbre::TimberCrush => 0.15,
            Timbre::TimberDrop => 1.5,
            Timbre::ConcreteCrack | Timbre::ConcreteBreak | Timbre::ConcreteCrush => 0.3,
            Timbre::RebarSnap => 0.016,
            Timbre::Rubble => 0.2,
            Timbre::CarbonSnap | Timbre::CarbonCrackle => 0.05,
            Timbre::CarbonDrop => 1.0,
        }
    }
}

/// The small-scale crack of a material: what a micro-fracture plays.
pub fn crack_of(material: MaterialKind) -> Timbre {
    match material {
        MaterialKind::Steel => Timbre::SteelKnock,
        MaterialKind::Timber => Timbre::TimberCrack,
        MaterialKind::Concrete => Timbre::ConcreteCrack,
        MaterialKind::Composite => Timbre::CarbonCrackle,
    }
}

/// A material straining slowly, if it makes a sustained sound.
pub fn strain_of(material: MaterialKind) -> Timbre {
    match material {
        MaterialKind::Steel => Timbre::SteelGroan,
        MaterialKind::Timber => Timbre::TimberCreak,
        MaterialKind::Concrete => Timbre::ConcreteCrush,
        MaterialKind::Composite => Timbre::CarbonCrackle,
    }
}

/// A piece of `mass` kg landing.
pub fn drop_of(material: MaterialKind, mass: f32) -> Timbre {
    match material {
        MaterialKind::Steel if mass > 8.0 => Timbre::SteelDropHeavy,
        MaterialKind::Steel => Timbre::SteelDropLight,
        MaterialKind::Timber => Timbre::TimberDrop,
        MaterialKind::Concrete => Timbre::Rubble,
        MaterialKind::Composite => Timbre::CarbonDrop,
    }
}

/// One recording, with the single events found in it.
#[derive(Clone, Debug)]
pub struct Clip {
    pub data: Arc<[f32]>,
    /// Sample rate of `data`, Hz.
    pub rate: f32,
    /// Extra playback rate for a stand-in.
    pub tune: f32,
    /// Single events: start and length in samples.
    pub events: Vec<(u32, u32)>,
}

/// A slice of a recording to play.
#[derive(Clone, Debug)]
pub struct Slice {
    pub data: Arc<[f32]>,
    pub start: u32,
    pub len: u32,
    /// Source samples per second at natural pitch (rate times tuning).
    pub rate: f32,
    /// One over the slice's peak, and one over the root of its summed
    /// squares (which makes it ring a mode as hard as a unit impulse would).
    pub peak: f32,
    pub norm: f32,
}

impl Slice {
    fn new(clip: &Clip, start: u32, len: u32) -> Self {
        let part = &clip.data[start as usize..(start + len) as usize];
        let peak = part.iter().fold(0.0f32, |m, x| m.max(x.abs()));
        let energy: f32 = part.iter().map(|x| x * x).sum();
        Self {
            data: clip.data.clone(),
            start,
            len,
            rate: clip.rate * clip.tune,
            peak: if peak > 1e-6 { 1.0 / peak } else { 0.0 },
            norm: if energy > 1e-9 {
                energy.sqrt().recip()
            } else {
                0.0
            },
        }
    }
}

/// 16-bit PCM WAV to mono f32 (first channel) and its sample rate.
fn decode_wav(bytes: &[u8]) -> Option<(Vec<f32>, f32)> {
    if bytes.len() < 12 || &bytes[0..4] != b"RIFF" || &bytes[8..12] != b"WAVE" {
        return None;
    }
    let (mut channels, mut rate, mut bits) = (1usize, 32_000.0f32, 16u16);
    let mut i = 12;
    while i + 8 <= bytes.len() {
        let id = &bytes[i..i + 4];
        let len = u32::from_le_bytes(bytes[i + 4..i + 8].try_into().ok()?) as usize;
        let body = bytes.get(i + 8..(i + 8 + len).min(bytes.len()))?;
        if id == b"fmt " && body.len() >= 16 {
            channels = u16::from_le_bytes([body[2], body[3]]).max(1) as usize;
            rate = u32::from_le_bytes([body[4], body[5], body[6], body[7]]) as f32;
            bits = u16::from_le_bytes([body[14], body[15]]);
        } else if id == b"data" {
            if bits != 16 || rate < 8000.0 {
                return None;
            }
            let data = body
                .chunks_exact(2 * channels)
                .map(|c| i16::from_le_bytes([c[0], c[1]]) as f32 / 32768.0)
                .collect();
            return Some((data, rate));
        }
        i += 8 + len + (len & 1);
    }
    None
}

/// Finds the single events in a recording: places where the level jumps
/// up, each running until the next one or until it has died away.
fn find_events(data: &[f32], rate: f32) -> Vec<(u32, u32)> {
    let peak = data.iter().fold(0.0f32, |m, x| m.max(x.abs()));
    if peak <= 0.0 {
        return Vec::new();
    }
    let fast = 1.0 - (-1.0 / (0.0005 * rate)).exp();
    let slow = 1.0 - (-1.0 / (0.02 * rate)).exp();
    let (mut quick, mut lazy) = (0.0f32, 0.0f32);
    let gap = (0.02 * rate) as usize;
    let lead = (0.001 * rate) as usize;
    let mut starts: Vec<usize> = Vec::new();
    for (i, x) in data.iter().enumerate() {
        let a = x.abs();
        quick += (a - quick) * fast;
        lazy += (quick - lazy) * slow;
        let jumped = quick > 0.12 * peak && quick > 2.5 * lazy;
        if jumped && starts.last().is_none_or(|s| i > s + gap) {
            starts.push(i.saturating_sub(lead));
        }
    }
    let longest = (0.25 * rate) as usize;
    starts
        .iter()
        .enumerate()
        .map(|(k, &s)| {
            let end = starts.get(k + 1).copied().unwrap_or(data.len());
            (s as u32, (end - s).min(longest) as u32)
        })
        .filter(|&(_, len)| len as f32 > 0.004 * rate)
        .collect()
}

fn load(folder: &str, tune: f32) -> Vec<Clip> {
    FILES
        .iter()
        .filter(|(f, _)| *f == folder)
        .filter_map(|(_, bytes)| decode_wav(bytes))
        .map(|(data, rate)| Clip {
            events: find_events(&data, rate),
            data: data.into(),
            rate,
            tune,
        })
        .collect()
}

fn bank() -> &'static Vec<Vec<Clip>> {
    static BANK: OnceLock<Vec<Vec<Clip>>> = OnceLock::new();
    BANK.get_or_init(|| {
        TIMBRES
            .iter()
            .map(|t| {
                let own = load(t.folder(), 1.0);
                if !own.is_empty() {
                    return own;
                }
                let (folders, tune) = t.stand_in();
                folders.iter().flat_map(|f| load(f, tune)).collect()
            })
            .collect()
    })
}

/// Decodes every recording now, so the first break in a game does not pay
/// for it on the audio thread.
pub fn preload() {
    let _ = bank();
}

/// The recordings of a timbre (possibly none).
pub fn clips(timbre: Timbre) -> &'static [Clip] {
    let i = TIMBRES.iter().position(|t| *t == timbre).unwrap_or(0);
    &bank()[i]
}

/// True when the timbre's own folder has recordings (not a stand-in).
pub fn recorded(timbre: Timbre) -> bool {
    FILES.iter().any(|(f, _)| *f == timbre.folder())
}

/// A whole recording of the timbre.
pub fn whole(timbre: Timbre, rng: &mut Rng) -> Option<Slice> {
    let all = clips(timbre);
    if all.is_empty() {
        return None;
    }
    let c = &all[rng.next_u32() as usize % all.len()];
    Some(Slice::new(c, 0, c.data.len() as u32))
}

/// One single event out of a recording of the timbre: a crack, a tick, a
/// knock.
pub fn event(timbre: Timbre, rng: &mut Rng) -> Option<Slice> {
    let all = clips(timbre);
    if all.is_empty() {
        return None;
    }
    let c = &all[rng.next_u32() as usize % all.len()];
    let (start, len) = if c.events.is_empty() {
        (0, c.data.len() as u32)
    } else {
        c.events[rng.next_u32() as usize % c.events.len()]
    };
    Some(Slice::new(c, start, len))
}

/// A stretch of `seconds` from somewhere in a sustained recording.
pub fn stretch(timbre: Timbre, seconds: f32, rng: &mut Rng) -> Option<Slice> {
    let all = clips(timbre);
    if all.is_empty() {
        return None;
    }
    let c = &all[rng.next_u32() as usize % all.len()];
    let total = c.data.len() as u32;
    let len = ((seconds * c.rate) as u32).clamp(1, total);
    let start = (rng.unit() * (total - len) as f32) as u32;
    Some(Slice::new(c, start, len))
}

/// Playback rate for a member of size `size` (m) against the recording's:
/// a bigger thing sounds lower. Frequencies of similar bodies go as one
/// over their size; half of that is applied, and at most an octave either
/// way, because a recording slowed too far stops sounding like anything.
pub fn size_rate(timbre: Timbre, size: f32) -> f32 {
    if !(size.is_finite() && size > 0.0) {
        return 1.0;
    }
    (timbre.reference_size() / size).sqrt().clamp(0.5, 2.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_timbre_but_the_sustained_ones_has_recordings() {
        for t in TIMBRES {
            let n = clips(t).len();
            if matches!(t, Timbre::SteelGroan | Timbre::TimberCreak) && !recorded(t) {
                continue;
            }
            assert!(n > 0, "{t:?} has no recordings");
            for c in clips(t) {
                assert!(c.rate >= 8000.0 && c.data.len() > 100, "{t:?}");
                assert!(c.data.iter().all(|x| x.is_finite() && x.abs() <= 1.0));
                for &(s, l) in &c.events {
                    assert!((s + l) as usize <= c.data.len(), "{t:?}");
                }
            }
        }
    }

    #[test]
    fn single_events_are_found_in_a_run_of_cracks() {
        // Three clicks 100 ms apart, each dying away in a few ms.
        let rate = 32_000.0;
        let mut data = vec![0.0f32; 16_000];
        for k in 0..3 {
            for i in 0..600 {
                data[2000 + k * 3200 + i] =
                    (-(i as f32) / 80.0).exp() * if i % 2 == 0 { 1.0 } else { -1.0 };
            }
        }
        let events = find_events(&data, rate);
        assert_eq!(events.len(), 3, "{events:?}");
        for (k, &(s, _)) in events.iter().enumerate() {
            assert!(
                (s as i64 - (2000 + k * 3200) as i64).abs() < 100,
                "{events:?}"
            );
        }
    }

    #[test]
    fn bigger_members_play_lower_within_an_octave() {
        let t = Timbre::TimberBreak;
        assert!((size_rate(t, t.reference_size()) - 1.0).abs() < 1e-6);
        assert!(size_rate(t, 0.6) < size_rate(t, 0.15));
        assert_eq!(size_rate(t, 100.0), 0.5);
        assert_eq!(size_rate(t, 0.0001), 2.0);
        assert_eq!(size_rate(t, f32::NAN), 1.0);
    }

    #[test]
    fn slices_stay_inside_their_recordings() {
        let mut rng = Rng::new(9);
        for t in TIMBRES {
            for _ in 0..20 {
                for s in [
                    whole(t, &mut rng),
                    event(t, &mut rng),
                    stretch(t, 0.4, &mut rng),
                ]
                .into_iter()
                .flatten()
                {
                    assert!((s.start + s.len) as usize <= s.data.len(), "{t:?}");
                    assert!(s.len > 0);
                }
            }
        }
    }
}
