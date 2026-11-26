//! Hall ambience. (Stub: patches pending.)

use crate::{AmbientKind, Voice};

pub struct AmbientVoice {
    #[allow(dead_code)]
    kind: AmbientKind,
}

impl AmbientVoice {
    pub fn new(kind: AmbientKind, _sr: f32, _seed: u32) -> Self {
        Self { kind }
    }
    pub fn set_level(&mut self, _level: f32) {}
}

impl Voice for AmbientVoice {
    fn render(&mut self, _out: &mut [f32]) -> bool {
        true
    }
}
