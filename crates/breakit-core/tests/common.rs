#![allow(dead_code)]
use breakit_core::{Material, Section, Structure, Support, Vec3};

pub fn tube() -> Section {
    Section::tube(0.05, 0.005)
}

/// Horizontal cantilever along +X, clamped at x = 0. Returns (structure, node ids).
pub fn cantilever(length: f64, segments: usize, mat: Material) -> (Structure, Vec<usize>) {
    let mut s = Structure::new();
    let ids = s.add_line(
        Vec3::zeros(),
        Vec3::new(length, 0.0, 0.0),
        segments,
        &mat,
        &tube(),
    );
    s.add_support(Support::fixed(ids[0]));
    (s, ids)
}

pub fn rel_err(a: f64, b: f64) -> f64 {
    (a - b).abs() / b.abs()
}
