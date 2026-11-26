//! Structure sounds: stress beds, damage onsets, breaks and debris impacts.
//! (Stub: patches pending.)

use crate::{BoxVoice, BreakCause, DamageState, MaterialKind, Voice};

/// Continuous creaking / groaning of one material's loaded members.
pub struct StressVoice {
    #[allow(dead_code)]
    material: MaterialKind,
}

impl StressVoice {
    pub fn new(material: MaterialKind, _sr: f32, _seed: u32) -> Self {
        Self { material }
    }
    pub fn set(&mut self, _load: f32, _rate: f32, _members: f32) {}
    pub fn reset(&mut self) {}
}

impl Voice for StressVoice {
    fn render(&mut self, _out: &mut [f32]) -> bool {
        true
    }
}

pub fn damage_voice(_m: MaterialKind, _s: DamageState, _size: f32, _sr: f32, _seed: u32) -> Option<BoxVoice> {
    None
}

pub fn break_voice(_m: MaterialKind, _c: BreakCause, _size: f32, _sr: f32, _seed: u32) -> BoxVoice {
    Box::new(Silence)
}

pub fn impact_voice(_m: MaterialKind, _speed: f32, _size: f32, _sr: f32, _seed: u32) -> Option<BoxVoice> {
    None
}

struct Silence;
impl Voice for Silence {
    fn render(&mut self, _out: &mut [f32]) -> bool {
        false
    }
}
