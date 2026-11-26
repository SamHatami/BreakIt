//! Rigid impactors: drop weight and carts (stub, being written).
use super::View;
use crate::{Structure, Vec3};

#[derive(Clone, Debug, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Impactor {}

#[derive(Clone, Debug, Default)]
pub struct ImpactorRig {}

impl Impactor {
    pub fn prepare(&mut self, _s: &Structure) {}
    pub fn energy(&self) -> f64 {
        0.0
    }
    pub fn apply(&mut self, _v: &View, _t: f64, _scale: f64, _dt: f64, _out: &mut [Vec3]) {}
    pub fn rig(&self) -> ImpactorRig {
        ImpactorRig {}
    }
}
