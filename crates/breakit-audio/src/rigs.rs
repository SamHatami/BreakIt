//! Test rig sounds. (Stub: patches pending.)

use crate::{RigKind, Voice};

pub struct RigVoice {
    #[allow(dead_code)]
    kind: RigKind,
}

impl RigVoice {
    pub fn new(kind: RigKind, _sr: f32, _seed: u32) -> Self {
        Self { kind }
    }
    /// `drive` 0..1, `speed` physical rate (see [`crate::Command::Rig`]).
    pub fn set(&mut self, _drive: f32, _speed: f32) {}
}

impl Voice for RigVoice {
    fn render(&mut self, _out: &mut [f32]) -> bool {
        true
    }
}
