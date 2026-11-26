//! The play table: a model basin the player builds on, in world (play)
//! units, its top at y = 0 where the solver's ground is. The frame, legs
//! and rim are the same for every scenario; what fills the basin comes
//! from a [`TableInsert`], so scenarios plug in their own terrain.

use std::sync::Arc;

use bevy::math::{Vec2, Vec3};
use bevy::prelude::Resource;

use crate::palette::Mat;
use crate::scenery::{Rng, Scenery};

/// Minor grid pitch on the table, world units. Matches the build snap.
pub const GRID_MINOR: f32 = 0.5;
/// Every this many minor lines is a major line.
pub const GRID_MAJOR_EVERY: i32 = 4;

/// Depth of the basin below the playing surface, world units.
pub const BASIN_DEPTH: f32 = 4.0;

/// What a scenario puts in the table.
pub trait TableInsert: Send + Sync + 'static {
    /// Shown in debug read-outs.
    fn name(&self) -> &str;
    /// Fill the basin: everything from y = -[`BASIN_DEPTH`] up to the
    /// playing surface, inside `top.half` in x and z.
    fn build(&self, top: &mut TableTop);
    /// Solid ground for the solver's contact, as axis-aligned boxes
    /// (min, max) in world units, covering what `build` makes solid. The
    /// default is a flat slab over the whole opening.
    fn contact_boxes(&self, half: Vec2) -> Vec<(Vec3, Vec3)> {
        vec![slab(-half, half, 0.0)]
    }
}

/// A block of basin fill from the basin floor up to `top`.
pub fn slab(min: Vec2, max: Vec2, top: f32) -> (Vec3, Vec3) {
    (
        Vec3::new(min.x, -BASIN_DEPTH, min.y),
        Vec3::new(max.x, top, max.y),
    )
}

/// The table the current scenario plays on. Change it and the lab rebuilds
/// the table next frame.
#[derive(Resource, Clone)]
pub struct PlayTable {
    /// Playing surface size, world units (x by z).
    pub size: Vec2,
    pub insert: Arc<dyn TableInsert>,
}

impl Default for PlayTable {
    fn default() -> Self {
        Self::new(Ravine::default())
    }
}

impl PlayTable {
    pub fn new(insert: impl TableInsert) -> Self {
        Self {
            size: Vec2::new(28.0, 16.0),
            insert: Arc::new(insert),
        }
    }

    /// The table's solid ground for contact, world units.
    pub fn contact_boxes(&self) -> Vec<(Vec3, Vec3)> {
        self.insert.contact_boxes(self.size * 0.5)
    }

    /// The built-in inserts in a fixed order, for cycling through them.
    pub fn builtin(index: usize) -> Self {
        match index % 4 {
            0 => Self::new(Ravine::default()),
            1 => Self::new(Flat),
            2 => Self::new(Flume::default()),
            _ => Self::new(ShakeTable::default()),
        }
    }
}

/// Builder handed to a [`TableInsert`].
pub struct TableTop<'a> {
    /// Half size of the basin opening, x and z.
    pub half: Vec2,
    pub out: &'a mut Scenery,
    /// What rides the shake platform: drawn apart from the rest so the
    /// shake rig can move it (see `rigs::shake`). Empty for most inserts.
    pub platform: Scenery,
}

impl TableTop<'_> {
    /// A solid block of the basin fill, top at `top_y`, with the grid on
    /// its top face.
    pub fn surface(&mut self, min: Vec2, max: Vec2, top_y: f32) {
        self.out.m(Mat::TableTop).aabb(
            Vec3::new(min.x, -BASIN_DEPTH, min.y),
            Vec3::new(max.x, top_y, max.y),
        );
        self.grid(min, max, top_y);
    }

    /// Grid lines on a horizontal rectangle at height `y`, aligned to the
    /// world origin so they match where nodes snap.
    pub fn grid(&mut self, min: Vec2, max: Vec2, y: f32) {
        let lift = 0.004;
        let line = |s: &mut Scenery, k: i32, along_x: bool, lo: f32, hi: f32, at: f32| {
            let major = k % GRID_MAJOR_EVERY == 0;
            let (mat, w) = if major {
                (Mat::GridMajor, 0.016)
            } else {
                (Mat::GridMinor, 0.008)
            };
            let (a, b) = if along_x {
                (Vec3::new(lo, y, at - w), Vec3::new(hi, y + lift, at + w))
            } else {
                (Vec3::new(at - w, y, lo), Vec3::new(at + w, y + lift, hi))
            };
            s.m(mat).aabb(a, b);
        };
        let (k0, k1) = (
            (min.x / GRID_MINOR).ceil() as i32,
            (max.x / GRID_MINOR).floor() as i32,
        );
        for k in k0..=k1 {
            line(self.out, k, false, min.y, max.y, k as f32 * GRID_MINOR);
        }
        let (k0, k1) = (
            (min.y / GRID_MINOR).ceil() as i32,
            (max.y / GRID_MINOR).floor() as i32,
        );
        for k in k0..=k1 {
            line(self.out, k, true, min.x, max.x, k as f32 * GRID_MINOR);
        }
    }

    /// A terrain bank in the style of a contour model: stacked foam
    /// layers whose edges wander and step out towards `out_dir` (+1 or -1
    /// in x) as they go down, from the edge at `x` between `z0` and `z1`.
    pub fn contour_bank(&mut self, x: f32, z0: f32, z1: f32, bottom: f32, out_dir: f32, seed: u64) {
        let mut rng = Rng::new(seed);
        let phase: Vec<f32> = (0..6)
            .map(|_| rng.range(0.0, std::f32::consts::TAU))
            .collect();
        let wander = |z: f32, layer: f32| {
            0.12 * (z * 0.9 + phase[0] + layer * 0.3).sin()
                + 0.07 * (z * 2.3 + phase[1] - layer * 0.5).sin()
                + 0.04 * (z * 5.1 + phase[2]).sin()
        };
        let layer_h = 0.4;
        let layers = ((-bottom) / layer_h).ceil() as i32;
        let seg = 0.5;
        let n = ((z1 - z0) / seg).ceil() as i32;
        for i in 0..layers {
            let top = -(i as f32) * layer_h;
            let lo = (top - layer_h).max(bottom);
            // Lower layers reach further into the gap: a sloping bank.
            let reach = 0.13 * (i as f32 + 1.0);
            for k in 0..n {
                let za = z0 + k as f32 * seg;
                let zb = (za + seg).min(z1);
                let off = (reach + wander((za + zb) * 0.5, i as f32)).max(0.03);
                let (xa, xb) = (x - out_dir * 0.2, x + out_dir * off);
                self.out.m(Mat::Foam).aabb(
                    Vec3::new(xa.min(xb), lo, za),
                    Vec3::new(xa.max(xb), top - 0.01, zb),
                );
            }
        }
    }
}

/// Builds the whole table: frame, legs down to `floor_y`, and the insert.
pub fn build(table: &PlayTable, floor_y: f32) -> Scenery {
    let (mut out, platform) = build_parts(table, floor_y);
    out.merge(platform);
    out
}

/// The table as what stands still and what rides the shake platform.
pub fn build_parts(table: &PlayTable, floor_y: f32) -> (Scenery, Scenery) {
    let mut out = Scenery::default();
    let half = table.size * 0.5;
    frame(&mut out, half, floor_y);
    let mut top = TableTop {
        half,
        out: &mut out,
        platform: Scenery::default(),
    };
    table.insert.build(&mut top);
    let platform = top.platform;
    (out, platform)
}

fn frame(s: &mut Scenery, half: Vec2, floor_y: f32) {
    let rim = 0.35;
    let h = BASIN_DEPTH + 0.6;
    let top = 0.12;
    // Side walls of the basin.
    let walls = [
        (
            Vec3::new(-half.x - rim, -h, -half.y - rim),
            Vec3::new(half.x + rim, top, -half.y),
        ),
        (
            Vec3::new(-half.x - rim, -h, half.y),
            Vec3::new(half.x + rim, top, half.y + rim),
        ),
        (
            Vec3::new(-half.x - rim, -h, -half.y),
            Vec3::new(-half.x, top, half.y),
        ),
        (
            Vec3::new(half.x, -h, -half.y),
            Vec3::new(half.x + rim, top, half.y),
        ),
    ];
    for (a, b) in walls {
        s.m(Mat::TableFrame).aabb(a, b);
    }
    // A dark rubber edge on the rim and a bottom tray.
    for (a, b) in walls {
        s.m(Mat::TableDark)
            .aabb(Vec3::new(a.x, top, a.z), Vec3::new(b.x, top + 0.05, b.z));
    }
    s.m(Mat::TableFrame).aabb(
        Vec3::new(-half.x, -h, -half.y),
        Vec3::new(half.x, -BASIN_DEPTH, half.y),
    );
    // Scale ticks along the front and back rims: every metre of play space.
    let mut k = (-half.x).ceil() as i32;
    while (k as f32) <= half.x {
        let long = k % 5 == 0;
        let len = if long { 0.3 } else { 0.15 };
        for z in [-half.y - 0.02, half.y + 0.02] {
            let dir = if z < 0.0 { -1.0 } else { 1.0 };
            s.m(Mat::GridMajor).aabb(
                Vec3::new(k as f32 - 0.02, top + 0.05, z.min(z + dir * len)),
                Vec3::new(k as f32 + 0.02, top + 0.06, z.max(z + dir * len)),
            );
        }
        k += 1;
    }
    // Legs: square tubes with levelling feet, stretchers between them.
    let leg = 0.9;
    let inset = Vec2::new(half.x - 1.5, half.y - 1.2);
    let xs = [-inset.x, 0.0, inset.x];
    for &x in &xs {
        for z in [-inset.y, inset.y] {
            s.m(Mat::TableFrame).aabb(
                Vec3::new(x - leg * 0.5, floor_y + 0.5, z - leg * 0.5),
                Vec3::new(x + leg * 0.5, -h, z + leg * 0.5),
            );
            s.m(Mat::TableDark).cylinder(
                Vec3::new(x, floor_y, z),
                Vec3::new(x, floor_y + 0.5, z),
                leg * 0.7,
                16,
                true,
            );
        }
    }
    let ys = floor_y + (-h - floor_y) * 0.25;
    for z in [-inset.y, inset.y] {
        s.m(Mat::TableFrame).bar(
            Vec3::new(-inset.x, ys, z),
            Vec3::new(inset.x, ys, z),
            0.5,
            0.7,
            bevy::math::Vec3::Y,
        );
    }
    for &x in &xs {
        s.m(Mat::TableFrame).bar(
            Vec3::new(x, ys, -inset.y),
            Vec3::new(x, ys, inset.y),
            0.5,
            0.7,
            bevy::math::Vec3::Y,
        );
    }
    // A shelf under the table.
    s.m(Mat::Timber).aabb(
        Vec3::new(-inset.x, ys + 0.35, -inset.y),
        Vec3::new(inset.x, ys + 0.6, inset.y),
    );
}

/// A plain grid: open ground.
pub struct Flat;

impl TableInsert for Flat {
    fn name(&self) -> &str {
        "flat grid"
    }

    fn build(&self, top: &mut TableTop) {
        let h = top.half;
        top.surface(-h, h, 0.0);
    }
}

/// Two banks and a gap to bridge, along z, centred on x = 0.
pub struct Ravine {
    /// Clear gap between the banks, world units.
    pub gap: f32,
    /// Depth of the ravine floor below the banks.
    pub depth: f32,
}

impl Default for Ravine {
    fn default() -> Self {
        // The starter bridge spans 6 m with supports at x = +-3.
        Self {
            gap: 5.0,
            depth: 3.2,
        }
    }
}

impl TableInsert for Ravine {
    fn name(&self) -> &str {
        "ravine"
    }

    fn contact_boxes(&self, h: Vec2) -> Vec<(Vec3, Vec3)> {
        // The contour layers step out below the top; the sheer bank edge
        // is close enough for contact.
        let g = self.gap * 0.5;
        vec![
            slab(Vec2::new(-h.x, -h.y), Vec2::new(-g, h.y), 0.0),
            slab(Vec2::new(g, -h.y), Vec2::new(h.x, h.y), 0.0),
            slab(Vec2::new(-g, -h.y), Vec2::new(g, h.y), -self.depth),
        ]
    }

    fn build(&self, top: &mut TableTop) {
        let h = top.half;
        let g = self.gap * 0.5;
        top.surface(Vec2::new(-h.x, -h.y), Vec2::new(-g, h.y), 0.0);
        top.surface(Vec2::new(g, -h.y), Vec2::new(h.x, h.y), 0.0);
        let floor = -self.depth;
        top.out
            .m(Mat::Foam)
            .aabb(Vec3::new(-g, -BASIN_DEPTH, -h.y), Vec3::new(g, floor, h.y));
        // Contour-model banks down both sides and a stream at the bottom.
        top.contour_bank(-g, -h.y, h.y, floor, 1.0, 11);
        top.contour_bank(g, -h.y, h.y, floor, -1.0, 12);
        let w = g * 0.45;
        top.out.m(Mat::Water).quad([
            Vec3::new(-w, floor + 0.06, -h.y),
            Vec3::new(-w, floor + 0.06, h.y),
            Vec3::new(w, floor + 0.06, h.y),
            Vec3::new(w, floor + 0.06, -h.y),
        ]);
    }
}

/// A water channel along x: the structure stands in or over the water.
pub struct Flume {
    /// Channel width (z), world units.
    pub width: f32,
    /// Channel depth below the playing surface.
    pub depth: f32,
    /// Water surface below the playing surface.
    pub freeboard: f32,
}

impl Default for Flume {
    fn default() -> Self {
        Self {
            width: 6.0,
            depth: 3.2,
            freeboard: 1.0,
        }
    }
}

impl TableInsert for Flume {
    fn name(&self) -> &str {
        "wave flume"
    }

    fn contact_boxes(&self, h: Vec2) -> Vec<(Vec3, Vec3)> {
        let w = self.width * 0.5;
        vec![
            slab(Vec2::new(-h.x, -h.y), Vec2::new(h.x, -w), 0.0),
            slab(Vec2::new(-h.x, w), Vec2::new(h.x, h.y), 0.0),
            slab(Vec2::new(-h.x, -w), Vec2::new(h.x, w), -self.depth),
        ]
    }

    fn build(&self, top: &mut TableTop) {
        let h = top.half;
        let w = self.width * 0.5;
        top.surface(Vec2::new(-h.x, -h.y), Vec2::new(h.x, -w), 0.0);
        top.surface(Vec2::new(-h.x, w), Vec2::new(h.x, h.y), 0.0);
        let floor = -self.depth;
        top.out
            .m(Mat::TableFrame)
            .aabb(Vec3::new(-h.x, -BASIN_DEPTH, -w), Vec3::new(h.x, floor, w));
        // Grid on the channel bed, seen through the water.
        top.grid(Vec2::new(-h.x, -w), Vec2::new(h.x, w), floor);
        // Glass-like lining on the channel walls.
        for z in [-w + 0.01, w - 0.01] {
            let (a, b) = (Vec3::new(-h.x, floor, z), Vec3::new(h.x, floor, z));
            let (c, d) = (Vec3::new(h.x, 0.0, z), Vec3::new(-h.x, 0.0, z));
            top.out.m(Mat::Glass).quad([a, b, c, d]);
        }
        let wl = -self.freeboard;
        let n = (h.x * 2.0 / 0.5) as i32;
        for k in 0..n {
            let x0 = -h.x + k as f32 * 0.5;
            let x1 = x0 + 0.5;
            let y = |x: f32| wl + 0.08 * (x * 0.8).sin();
            top.out.m(Mat::Water).quad([
                Vec3::new(x0, y(x0), -w),
                Vec3::new(x0, y(x0), w),
                Vec3::new(x1, y(x1), w),
                Vec3::new(x1, y(x1), -w),
            ]);
        }
        // A paddle housing at the -x end.
        top.out
            .m(Mat::MachineBlue)
            .aabb(Vec3::new(-h.x, wl - 0.5, -w), Vec3::new(-h.x + 0.4, 0.6, w));
    }
}

/// A square shake table set flush into the surface.
pub struct ShakeTable {
    /// Platform edge length, world units.
    pub size: f32,
}

impl Default for ShakeTable {
    fn default() -> Self {
        Self { size: 8.0 }
    }
}

impl TableInsert for ShakeTable {
    fn name(&self) -> &str {
        "shake table"
    }

    fn build(&self, top: &mut TableTop) {
        let h = top.half;
        let p = self.size * 0.5;
        let gap = 0.15;
        let o = p + gap;
        // Surround, as four slabs.
        top.surface(Vec2::new(-h.x, -h.y), Vec2::new(h.x, -o), 0.0);
        top.surface(Vec2::new(-h.x, o), Vec2::new(h.x, h.y), 0.0);
        top.surface(Vec2::new(-h.x, -o), Vec2::new(-o, o), 0.0);
        top.surface(Vec2::new(o, -o), Vec2::new(h.x, o), 0.0);
        // Pit, and the platform that rides the table motion.
        top.out
            .m(Mat::TableDark)
            .aabb(Vec3::new(-o, -BASIN_DEPTH, -o), Vec3::new(o, -1.0, o));
        let mut plat = TableTop {
            half: top.half,
            out: &mut top.platform,
            platform: Scenery::default(),
        };
        plat.out
            .m(Mat::MachineBlue)
            .aabb(Vec3::new(-p, -0.6, -p), Vec3::new(p, 0.0, p));
        plat.grid(Vec2::splat(-p), Vec2::splat(p), 0.0);
        // Bolt holes on a 1 m pitch.
        let mut x = -p + 0.5;
        while x < p {
            let mut z = -p + 0.5;
            while z < p {
                plat.out.m(Mat::TableDark).disc(
                    Vec3::new(x + 0.25, 0.006, z + 0.25),
                    Vec3::Y,
                    0.07,
                    8,
                );
                z += 1.0;
            }
            x += 1.0;
        }
        // Bearing pads under the edges.
        for (dx, dz) in [(1.0f32, 0.0f32), (-1.0, 0.0), (0.0, 1.0), (0.0, -1.0)] {
            let c = Vec3::new(dx * (p - 0.4), -0.6, dz * (p - 0.4));
            top.platform
                .m(Mat::Stainless)
                .cylinder(c, c - Vec3::Y * 0.4, 0.12, 10, false);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The highest point of the playing surface, inside the opening.
    fn surface_top(s: &Scenery, half: Vec2) -> f32 {
        s.bufs[&Mat::TableTop]
            .positions
            .iter()
            .filter(|p| p[0].abs() <= half.x + 1e-3 && p[2].abs() <= half.y + 1e-3)
            .map(|p| p[1])
            .fold(f32::MIN, f32::max)
    }

    #[test]
    fn every_builtin_plays_at_y_zero() {
        for i in 0..4 {
            let t = PlayTable::builtin(i);
            let s = build(&t, -17.0);
            let top = surface_top(&s, t.size * 0.5);
            assert!(top.abs() < 1e-5, "{} top at {top}", t.insert.name());
            assert!(s.bufs.contains_key(&Mat::GridMajor));
        }
    }

    #[test]
    fn contact_boxes_top_out_at_the_playing_surface() {
        for i in 0..4 {
            let t = PlayTable::builtin(i);
            let boxes = t.contact_boxes();
            assert!(!boxes.is_empty());
            let top = boxes.iter().map(|b| b.1.y).fold(f32::MIN, f32::max);
            assert_eq!(top, 0.0, "{}", t.insert.name());
            for (min, max) in boxes {
                assert!(min.cmple(max).all());
                assert!(max.x <= t.size.x * 0.5 + 1e-4 && max.z <= t.size.y * 0.5 + 1e-4);
            }
        }
    }

    #[test]
    fn ravine_leaves_the_starter_supports_on_the_banks() {
        let r = Ravine::default();
        // Starter bridge supports sit at x = +-3.
        assert!(r.gap * 0.5 < 3.0 - 0.2);
    }

    #[test]
    fn legs_reach_the_floor() {
        let s = build(&PlayTable::default(), -17.0);
        let low = s
            .bufs
            .values()
            .flat_map(|b| b.positions.iter())
            .map(|p| p[1])
            .fold(f32::MAX, f32::min);
        assert!((low + 17.0).abs() < 1e-4);
    }
}
