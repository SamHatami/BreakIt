//! Seismic (stub, being written).
use super::View;
use crate::Vec3;
#[derive(Clone, Debug, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Quake {}
#[derive(Clone, Copy, Debug, Default)]
pub struct TableRig { pub displacement: Vec3 }
impl TableRig { pub fn combine(&self, o: &TableRig) -> TableRig { TableRig { displacement: self.displacement + o.displacement } } }
impl Quake {
    pub fn prepare(&mut self) {}
    pub fn peak_acceleration(&self) -> f64 { 0.0 }
    pub fn apply(&mut self, _v: &View, _t: f64, _scale: f64, _out: &mut [Vec3]) {}
    pub fn table(&self, _t: f64, _scale: f64) -> TableRig { TableRig::default() }
}
