//! Water: flood levels, currents, waves and surges (stub, being written).
use super::View;
use crate::Vec3;

#[derive(Clone, Debug, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Water {}

#[derive(Clone, Debug, Default)]
pub struct FlumeRig {}

impl Water {
    pub fn prepare(&mut self) {}
    pub fn intensity(&self) -> (f64, &'static str) {
        (0.0, "m")
    }
    pub fn apply(&mut self, _v: &View, _t: f64, _ramp: f64, _scale: f64, _dt: f64, _out: &mut [Vec3]) {}
    pub fn rig(&self, _t: f64, _ramp: f64) -> FlumeRig {
        FlumeRig {}
    }
}
