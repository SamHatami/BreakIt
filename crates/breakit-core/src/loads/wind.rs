//! Wind (stub, being written).
use super::View;
use crate::{Structure, Vec3};
#[derive(Clone, Debug, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Wind { pub speed: f64 }
#[derive(Clone, Debug, Default)]
pub struct WindRig {}
impl Wind {
    pub fn prepare(&mut self, _s: &Structure) {}
    pub fn apply(&mut self, _v: &View, _t: f64, _scale: f64, _out: &mut [Vec3]) {}
    pub fn rig(&self, _t: f64, _ramp: f64) -> WindRig { WindRig {} }
}
