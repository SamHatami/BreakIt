//! Parametric cross-section profiles.
//!
//! A [`Profile`] is the single description of a member's cross-section. The
//! visual mesh is generated from it, and so are the section properties the
//! solver needs ([`Profile::section`]), so what you see is what is computed.
//!
//! Profile coordinates are the member's local (y, z) plane from
//! [`breakit_core::frame`]: y is "up" (depth `h`), z is sideways (width `b`).

use std::f32::consts::TAU;

use breakit_core::Section;
use glam::Vec2;

/// A cross-section shape. All dimensions in metres.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Profile {
    /// Solid rectangle: `b` wide (local z), `h` deep (local y). Timber, concrete.
    Rect { b: f32, h: f32 },
    /// Solid circle of radius `r`. Bars, rods, cables.
    Round { r: f32 },
    /// Circular hollow section, outer radius `r`, wall `t`.
    Tube { r: f32, t: f32 },
    /// Rectangular hollow section, `b` x `h` outside, wall `t`.
    BoxSection { b: f32, h: f32, t: f32 },
    /// Doubly symmetric I/H section: depth `h`, flange width `b`, web
    /// thickness `tw`, flange thickness `tf`.
    IBeam { h: f32, b: f32, tw: f32, tf: f32 },
}

/// One vertex of a profile outline, with the outward surface normal.
#[derive(Clone, Copy, Debug)]
pub struct OutlineVertex {
    pub p: Vec2,
    pub n: Vec2,
    /// Perimeter coordinate along this loop, 0..1.
    pub u: f32,
}

/// The cross-section's cap, triangulated in terms of 2D points.
#[derive(Clone, Debug)]
pub struct Cap {
    pub points: Vec<Vec2>,
    /// Counter-clockwise triangles (seen looking down -x, i.e. from end a).
    pub triangles: Vec<[u32; 3]>,
}

impl Profile {
    /// Half the depth (local y extent).
    pub fn half_depth(&self) -> f32 {
        match *self {
            Profile::Rect { h, .. } | Profile::BoxSection { h, .. } | Profile::IBeam { h, .. } => {
                0.5 * h
            }
            Profile::Round { r } | Profile::Tube { r, .. } => r,
        }
    }

    /// Half the width (local z extent).
    pub fn half_width(&self) -> f32 {
        match *self {
            Profile::Rect { b, .. } | Profile::BoxSection { b, .. } | Profile::IBeam { b, .. } => {
                0.5 * b
            }
            Profile::Round { r } | Profile::Tube { r, .. } => r,
        }
    }

    /// Radius of the smallest circle around the section.
    pub fn bounding_radius(&self) -> f32 {
        Vec2::new(self.half_depth(), self.half_width()).length()
    }

    /// Default number of segments around round profiles.
    fn round_segments(&self) -> usize {
        16
    }

    /// Outline loops. The first loop is the outer boundary; hollow sections add
    /// an inner loop whose normals point into the hole. Hard corners are
    /// duplicated so each face gets its own normal.
    pub fn outline(&self) -> Vec<Vec<OutlineVertex>> {
        match *self {
            Profile::Rect { b, h } => vec![polygon_loop(&rect_pts(b, h), false)],
            Profile::Round { r } => vec![circle_loop(r, self.round_segments(), false)],
            Profile::Tube { r, t } => vec![
                circle_loop(r, self.round_segments(), false),
                circle_loop((r - t).max(r * 0.05), self.round_segments(), true),
            ],
            Profile::BoxSection { b, h, t } => vec![
                polygon_loop(&rect_pts(b, h), false),
                polygon_loop(&rect_pts(b - 2.0 * t, h - 2.0 * t), true),
            ],
            Profile::IBeam { h, b, tw, tf } => vec![polygon_loop(&ibeam_pts(h, b, tw, tf), false)],
        }
    }

    /// Triangulated end cap.
    pub fn cap(&self) -> Cap {
        match *self {
            Profile::Rect { b, h } => ear_clip(rect_pts(b, h)),
            Profile::Round { r } => ear_clip(circle_pts(r, self.round_segments())),
            Profile::IBeam { h, b, tw, tf } => {
                // Three rectangles. Ear clipping the 12-gon trips over its
                // collinear corners.
                let (y, yw) = (0.5 * h, 0.5 * h - tf);
                let mut cap = Cap {
                    points: Vec::new(),
                    triangles: Vec::new(),
                };
                for (y0, y1, half) in [(-y, -yw, 0.5 * b), (-yw, yw, 0.5 * tw), (yw, y, 0.5 * b)] {
                    let base = cap.points.len() as u32;
                    cap.points.extend([
                        Vec2::new(y0, -half),
                        Vec2::new(y0, half),
                        Vec2::new(y1, half),
                        Vec2::new(y1, -half),
                    ]);
                    cap.triangles.push([base, base + 1, base + 2]);
                    cap.triangles.push([base, base + 2, base + 3]);
                }
                cap
            }
            Profile::Tube { r, t } => annulus(
                circle_pts(r, self.round_segments()),
                circle_pts((r - t).max(r * 0.05), self.round_segments()),
            ),
            Profile::BoxSection { b, h, t } => {
                annulus(rect_pts(b, h), rect_pts(b - 2.0 * t, h - 2.0 * t))
            }
        }
    }

    /// Section properties for the solver, computed from the same dimensions.
    pub fn section(&self) -> Section {
        let f = |x: f32| x as f64;
        match *self {
            Profile::Rect { b, h } => Section::rect(f(b), f(h)),
            Profile::Tube { r, t } => Section::tube(f(r), f(t)),
            Profile::Round { r } => {
                let r = f(r);
                let i = std::f64::consts::PI * r.powi(4) / 4.0;
                Section {
                    area: std::f64::consts::PI * r * r,
                    iy: i,
                    iz: i,
                    j: 2.0 * i,
                    zy: 4.0 / 3.0 * r.powi(3),
                    zz: 4.0 / 3.0 * r.powi(3),
                }
            }
            Profile::BoxSection { b, h, t } => {
                let (b, h, t) = (f(b), f(h), f(t));
                let (bi, hi) = (b - 2.0 * t, h - 2.0 * t);
                // Bredt-Batho thin-walled torsion constant.
                let (bm, hm) = (b - t, h - t);
                Section {
                    area: b * h - bi * hi,
                    iz: (b * h.powi(3) - bi * hi.powi(3)) / 12.0,
                    iy: (h * b.powi(3) - hi * bi.powi(3)) / 12.0,
                    j: 2.0 * t * bm * bm * hm * hm / (bm + hm),
                    zz: (b * h * h - bi * hi * hi) / 4.0,
                    zy: (h * b * b - hi * bi * bi) / 4.0,
                }
            }
            Profile::IBeam { h, b, tw, tf } => {
                let (h, b, tw, tf) = (f(h), f(b), f(tw), f(tf));
                let hw = h - 2.0 * tf;
                Section {
                    area: 2.0 * b * tf + hw * tw,
                    iz: (b * h.powi(3) - (b - tw) * hw.powi(3)) / 12.0,
                    iy: 2.0 * tf * b.powi(3) / 12.0 + hw * tw.powi(3) / 12.0,
                    j: (2.0 * b * tf.powi(3) + (h - tf) * tw.powi(3)) / 3.0,
                    zz: b * tf * (h - tf) + tw * hw * hw / 4.0,
                    zy: tf * b * b / 2.0 + hw * tw * tw / 4.0,
                }
            }
        }
    }
}

fn rect_pts(b: f32, h: f32) -> Vec<Vec2> {
    // (y, z) counter-clockwise when looking from end a (down -x):
    // z to the right, y up.
    let (y, z) = (0.5 * h, 0.5 * b);
    vec![
        Vec2::new(-y, -z),
        Vec2::new(-y, z),
        Vec2::new(y, z),
        Vec2::new(y, -z),
    ]
}

fn circle_pts(r: f32, n: usize) -> Vec<Vec2> {
    (0..n)
        .map(|i| {
            let a = TAU * i as f32 / n as f32;
            Vec2::new(-r * a.cos(), r * a.sin())
        })
        .collect()
}

fn ibeam_pts(h: f32, b: f32, tw: f32, tf: f32) -> Vec<Vec2> {
    let (y, z, yw, zw) = (0.5 * h, 0.5 * b, 0.5 * h - tf, 0.5 * tw);
    // Counter-clockwise, starting bottom-left of the bottom flange.
    [
        (-y, -z),
        (-y, z),
        (-yw, z),
        (-yw, zw),
        (yw, zw),
        (yw, z),
        (y, z),
        (y, -z),
        (yw, -z),
        (yw, -zw),
        (-yw, -zw),
        (-yw, -z),
    ]
    .iter()
    .map(|&(a, b)| Vec2::new(a, b))
    .collect()
}

/// Signed area in the (z, y) plane, positive when counter-clockwise with z
/// right and y up.
fn signed_area(pts: &[Vec2]) -> f32 {
    let mut a = 0.0;
    for i in 0..pts.len() {
        let (p, q) = (pts[i], pts[(i + 1) % pts.len()]);
        a += p.y * q.x - q.y * p.x;
    }
    0.5 * a
}

/// Hard-edged loop: every edge gets its own pair of vertices.
fn polygon_loop(pts: &[Vec2], hole: bool) -> Vec<OutlineVertex> {
    let ccw = signed_area(pts) > 0.0;
    let perim: f32 = (0..pts.len())
        .map(|i| (pts[(i + 1) % pts.len()] - pts[i]).length())
        .sum();
    let mut out = Vec::with_capacity(pts.len() * 2);
    let mut acc = 0.0;
    for i in 0..pts.len() {
        let (p, q) = (pts[i], pts[(i + 1) % pts.len()]);
        let d = q - p;
        // In (y, z) coordinates with z right, y up, the right-hand normal of a
        // counter-clockwise edge points outward.
        let mut n = Vec2::new(-d.y, d.x).normalize();
        if !ccw {
            n = -n;
        }
        if hole {
            n = -n;
        }
        out.push(OutlineVertex {
            p,
            n,
            u: acc / perim,
        });
        acc += d.length();
        out.push(OutlineVertex {
            p: q,
            n,
            u: acc / perim,
        });
    }
    out
}

/// Smooth loop: one vertex per point with a radial normal, closed by
/// repeating the first point (so the perimeter coordinate can reach 1).
fn circle_loop(r: f32, n: usize, hole: bool) -> Vec<OutlineVertex> {
    let mut out: Vec<OutlineVertex> = circle_pts(r, n)
        .into_iter()
        .enumerate()
        .map(|(i, p)| OutlineVertex {
            p,
            n: if hole { -p / r } else { p / r },
            u: i as f32 / n as f32,
        })
        .collect();
    let mut first = out[0];
    first.u = 1.0;
    out.push(first);
    out
}

/// Ear clipping for a simple polygon (any winding).
fn ear_clip(mut pts: Vec<Vec2>) -> Cap {
    if signed_area(&pts) < 0.0 {
        pts.reverse();
    }
    let n = pts.len();
    let mut idx: Vec<u32> = (0..n as u32).collect();
    let mut tris = Vec::with_capacity(n - 2);
    let cross = |a: Vec2, b: Vec2, c: Vec2| {
        // z-component in (z, y) = (x', y') coordinates.
        let (ab, ac) = (b - a, c - a);
        ab.y * ac.x - ab.x * ac.y
    };
    let mut guard = 0;
    while idx.len() > 3 && guard < 10_000 {
        guard += 1;
        let m = idx.len();
        let mut clipped = false;
        for i in 0..m {
            let (ia, ib, ic) = (idx[(i + m - 1) % m], idx[i], idx[(i + 1) % m]);
            let (a, b, c) = (pts[ia as usize], pts[ib as usize], pts[ic as usize]);
            if cross(a, b, c) <= 1e-12 {
                continue; // reflex or degenerate
            }
            let inside = idx.iter().any(|&j| {
                if j == ia || j == ib || j == ic {
                    return false;
                }
                let p = pts[j as usize];
                cross(a, b, p) > 0.0 && cross(b, c, p) > 0.0 && cross(c, a, p) > 0.0
            });
            if inside {
                continue;
            }
            tris.push([ia, ib, ic]);
            idx.remove(i);
            clipped = true;
            break;
        }
        if !clipped {
            break;
        }
    }
    if idx.len() == 3 {
        tris.push([idx[0], idx[1], idx[2]]);
    }
    Cap {
        points: pts,
        triangles: tris,
    }
}

/// Cap between an outer and inner loop with the same number of points.
fn annulus(mut outer: Vec<Vec2>, mut inner: Vec<Vec2>) -> Cap {
    if signed_area(&outer) < 0.0 {
        outer.reverse();
    }
    if signed_area(&inner) < 0.0 {
        inner.reverse();
    }
    let n = outer.len() as u32;
    let mut points = outer;
    points.extend(inner);
    let mut triangles = Vec::new();
    for i in 0..n {
        let j = (i + 1) % n;
        triangles.push([i, j, n + j]);
        triangles.push([i, n + j, n + i]);
    }
    Cap { points, triangles }
}

/// Area of a cap, for tests.
pub fn cap_area(cap: &Cap) -> f32 {
    cap.triangles
        .iter()
        .map(|t| {
            let (a, b, c) = (
                cap.points[t[0] as usize],
                cap.points[t[1] as usize],
                cap.points[t[2] as usize],
            );
            let (ab, ac) = (b - a, c - a);
            0.5 * (ab.y * ac.x - ab.x * ac.y)
        })
        .sum()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn all() -> Vec<Profile> {
        vec![
            Profile::Rect { b: 0.1, h: 0.2 },
            Profile::Round { r: 0.02 },
            Profile::Tube { r: 0.05, t: 0.005 },
            Profile::BoxSection {
                b: 0.1,
                h: 0.15,
                t: 0.008,
            },
            Profile::IBeam {
                h: 0.3,
                b: 0.15,
                tw: 0.0071,
                tf: 0.0107,
            },
        ]
    }

    #[test]
    fn cap_area_matches_section_area() {
        for p in all() {
            let a_cap = cap_area(&p.cap()) as f64;
            let a = p.section().area;
            // Round profiles are polygons, so allow the 16-gon error (~2.6%).
            let tol = match p {
                Profile::Round { .. } | Profile::Tube { .. } => 0.03,
                _ => 1e-4,
            };
            assert!(
                ((a_cap - a) / a).abs() < tol,
                "{p:?}: cap {a_cap} vs section {a}"
            );
            assert!(a_cap > 0.0, "{p:?}: caps must be counter-clockwise");
        }
    }

    #[test]
    fn ibeam_matches_ipe300_tables() {
        // IPE 300: A = 53.8 cm², Iy(strong) = 8356 cm⁴, Wpl = 628 cm³.
        let s = Profile::IBeam {
            h: 0.3,
            b: 0.15,
            tw: 0.0071,
            tf: 0.0107,
        }
        .section();
        // Tables include root radii; the sharp-cornered shape is ~4% smaller.
        assert!((s.area / 53.8e-4 - 1.0).abs() < 0.05, "A {}", s.area);
        assert!((s.iz / 8356e-8 - 1.0).abs() < 0.05, "I {}", s.iz);
        assert!((s.zz / 628e-6 - 1.0).abs() < 0.05, "Z {}", s.zz);
    }

    #[test]
    fn outline_normals_point_outward() {
        // The I-beam is not convex, so the origin test only applies to the rest.
        for p in all()
            .into_iter()
            .filter(|p| !matches!(p, Profile::IBeam { .. }))
        {
            let loops = p.outline();
            for (li, lp) in loops.iter().enumerate() {
                for v in lp {
                    // Outward = away from the centroid for the outer loop,
                    // towards it for holes. Use the edge midpoint direction
                    // test against the origin (all profiles are centred).
                    let s = v.n.dot(v.p);
                    if li == 0 {
                        assert!(s > -1e-6, "{p:?} outer normal points in at {:?}", v.p);
                    } else {
                        assert!(s < 1e-6, "{p:?} hole normal points out at {:?}", v.p);
                    }
                }
            }
        }
    }
}
