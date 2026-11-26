//! Moving vehicle loads along a lane (stub, being written).
use super::View;
use crate::{Structure, Vec3};

#[derive(Clone, Debug, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Traffic {}

#[derive(Clone, Debug, Default)]
pub struct VehicleRig {}

impl Traffic {
    pub fn prepare(&mut self, _s: &Structure) {}
    pub fn total_load(&self) -> f64 {
        0.0
    }
    pub fn apply(&mut self, _v: &View, _t: f64, _ramp: f64, _scale: f64, _out: &mut [Vec3]) {}
    pub fn rig(&self, _t: f64) -> Vec<VehicleRig> {
        Vec::new()
    }
}
