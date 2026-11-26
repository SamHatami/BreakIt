//! A set of mesh buffers, one per material.

use std::collections::BTreeMap;

use bevy::math::Vec3;

use crate::geom::MeshBuf;
use crate::palette::Mat;

/// Named spots in the lab that other systems (sound, scenario rigs) may
/// want to find, since the scenery itself is merged into a few meshes.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Anchor {
    HighBayLamp,
    PowerUnit,
    Pump,
    /// Hook block of the overhead crane.
    CraneHook,
    FlumePaddle,
    ControlRoom,
    ElectricalCabinets,
    TestRig,
}

#[derive(Default, Debug, Clone)]
pub struct Scenery {
    pub bufs: BTreeMap<Mat, MeshBuf>,
    /// Anchors, in the same units as the meshes.
    pub anchors: Vec<(Anchor, Vec3)>,
}

impl Scenery {
    pub fn m(&mut self, mat: Mat) -> &mut MeshBuf {
        self.bufs.entry(mat).or_default()
    }

    pub fn anchor(&mut self, kind: Anchor, at: Vec3) {
        self.anchors.push((kind, at));
    }

    pub fn triangles(&self) -> usize {
        self.bufs.values().map(MeshBuf::triangles).sum()
    }

    pub fn merge(&mut self, other: Scenery) {
        for (mat, buf) in other.bufs {
            self.m(mat).append(&buf, Vec3::ZERO);
        }
        self.anchors.extend(other.anchors);
    }
}

/// A tiny deterministic generator (SplitMix64) so the dressing is the same
/// every run without pulling in a crate.
#[derive(Clone, Debug)]
pub struct Rng(u64);

impl Rng {
    pub fn new(seed: u64) -> Self {
        Self(seed)
    }

    pub fn next_u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    /// Uniform in [0, 1).
    pub fn f(&mut self) -> f32 {
        (self.next_u64() >> 40) as f32 / (1u64 << 24) as f32
    }

    pub fn range(&mut self, lo: f32, hi: f32) -> f32 {
        lo + (hi - lo) * self.f()
    }

    pub fn chance(&mut self, p: f32) -> bool {
        self.f() < p
    }

    pub fn pick<T: Copy>(&mut self, items: &[T]) -> T {
        items[(self.next_u64() % items.len() as u64) as usize]
    }
}
