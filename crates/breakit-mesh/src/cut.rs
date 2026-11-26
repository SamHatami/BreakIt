//! The shape of a fracture face.
//!
//! When a member breaks it is not re-meshed. Its two halves keep drawing the
//! same template, each clamped to its side of a *cut surface*:
//! `s = s_break + cut_offset(y, z) / length`. The vertex shader evaluates the
//! same function (`member.wesl`, `cut_offset`), so both halves share one
//! surface and fit together like the pieces they are. Each material family
//! has its own surface:
//!
//! * steel: a tilted, slightly ragged tear (the shader also necks the section)
//! * timber: fibre bundles that each let go at a different length (splinters)
//! * concrete: a rough, chunky face
//! * composite: plies that each break at a different length (steps)

use glam::Vec2;

use crate::state::MaterialKind;

fn fract(x: f32) -> f32 {
    x - x.floor()
}

/// Cheap 2D hash to 0..1, identical in the shader.
pub fn hash2(p: Vec2) -> f32 {
    let (mut x, mut y, mut z) = (
        fract(p.x * 0.1031),
        fract(p.y * 0.1031),
        fract(p.x * 0.1031),
    );
    let d = x * (y + 33.33) + y * (z + 33.33) + z * (x + 33.33);
    x += d;
    y += d;
    z += d;
    fract((x + y) * z)
}

/// Smooth value noise, 0..1.
pub fn vnoise(p: Vec2) -> f32 {
    let i = p.floor();
    let f = p - i;
    let u = f * f * (Vec2::splat(3.0) - 2.0 * f);
    let a = hash2(i);
    let b = hash2(i + Vec2::new(1.0, 0.0));
    let c = hash2(i + Vec2::new(0.0, 1.0));
    let d = hash2(i + Vec2::new(1.0, 1.0));
    a + (b - a) * u.x + (c - a) * u.y + (a - b - c + d) * u.x * u.y
}

/// Offset of the fracture surface along the member, in metres, at section
/// point `yz`. `depth` is the section depth and `seed` the member's seed.
pub fn cut_offset(kind: MaterialKind, yz: Vec2, depth: f32, seed: f32) -> f32 {
    let d = depth.max(1e-4);
    let q = yz / d + Vec2::splat(seed * 37.0);
    match kind {
        MaterialKind::Steel => {
            let tilt = Vec2::new(
                hash2(Vec2::splat(seed * 91.0)),
                hash2(Vec2::splat(seed * 53.0)),
            ) - Vec2::splat(0.5);
            d * (0.5 * tilt.dot(yz / d) + 0.12 * (vnoise(q * 9.0) - 0.5))
        }
        MaterialKind::Timber => {
            // Fibre bundles: thin along z (the growth rings), each with its
            // own length.
            let cell = (q * Vec2::new(5.0, 9.0)).floor();
            let h = hash2(cell);
            d * (1.7 * (h - 0.5) * (0.4 + 0.6 * h) + 0.25 * (vnoise(q * 3.0) - 0.5))
        }
        MaterialKind::Concrete | MaterialKind::ReinforcedConcrete => {
            d * (0.45 * (vnoise(q * 3.5) - 0.5) + 0.18 * (vnoise(q * 11.0) - 0.5))
        }
        MaterialKind::Composite => {
            let ply = (q.x * 7.0).floor();
            d * (1.3 * (hash2(Vec2::new(ply, 7.0)) - 0.5) + 0.2 * (vnoise(q * 14.0) - 0.5))
        }
    }
}

/// How far the cut surface can reach either side of the break, metres.
pub fn cut_reach(kind: MaterialKind, depth: f32) -> f32 {
    depth
        * match kind {
            MaterialKind::Steel => 0.35,
            MaterialKind::Timber => 1.0,
            MaterialKind::Concrete | MaterialKind::ReinforcedConcrete => 0.35,
            MaterialKind::Composite => 0.8,
        }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cut_stays_within_its_reach_and_varies() {
        for kind in MaterialKind::ALL {
            let depth = 0.2;
            let reach = cut_reach(kind, depth);
            let (mut lo, mut hi) = (f32::INFINITY, f32::NEG_INFINITY);
            for i in 0..40 {
                for j in 0..40 {
                    let yz = Vec2::new(i as f32, j as f32) / 40.0 * depth - Vec2::splat(0.1);
                    let c = cut_offset(kind, yz, depth, 0.37);
                    assert!(c.abs() <= reach, "{kind:?}: {c} beyond {reach}");
                    lo = lo.min(c);
                    hi = hi.max(c);
                }
            }
            assert!(hi - lo > 0.1 * depth, "{kind:?} cut is nearly flat");
        }
    }

    #[test]
    fn hash_is_in_unit_range() {
        for i in 0..200 {
            let h = hash2(Vec2::new(i as f32 * 1.7 - 90.0, i as f32 * -0.3));
            assert!((0.0..1.0).contains(&h));
        }
    }
}
