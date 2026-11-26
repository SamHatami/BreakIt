//! Interface sounds. (Stub: patches pending.)

use crate::{BoxVoice, UiSound, Voice};

pub fn ui_voice(_s: UiSound, _sr: f32, _seed: u32) -> BoxVoice {
    Box::new(Silence)
}

struct Silence;
impl Voice for Silence {
    fn render(&mut self, _out: &mut [f32]) -> bool {
        false
    }
}
