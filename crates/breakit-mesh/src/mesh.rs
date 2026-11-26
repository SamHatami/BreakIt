//! Plain triangle-mesh buffers and the member template mesh.
//!
//! A member template is generated once per [`Profile`] and shared by every
//! member with that profile. Its vertices are stored in *member space*:
//! `position = (s, y, z)` where `s` in 0..1 runs from end a to end b and
//! `(y, z)` is the point in the cross-section. The renderer bends it along the
//! member's current curve on the GPU ([`crate::pose::MemberPose::deform`] is
//! the CPU reference of that shader), so the vertex data never changes after
//! upload, whatever the members do.

use glam::{Vec2, Vec3};

use crate::profile::Profile;

/// Indexed triangle mesh with normals and one UV set.
#[derive(Clone, Debug, Default)]
pub struct MeshData {
    pub positions: Vec<[f32; 3]>,
    pub normals: Vec<[f32; 3]>,
    pub uvs: Vec<[f32; 2]>,
    pub indices: Vec<u32>,
}

impl MeshData {
    pub fn vertex_count(&self) -> usize {
        self.positions.len()
    }
    pub fn triangle_count(&self) -> usize {
        self.indices.len() / 3
    }

    pub fn push_vertex(&mut self, p: Vec3, n: Vec3, uv: Vec2) -> u32 {
        self.positions.push(p.to_array());
        self.normals.push(n.to_array());
        self.uvs.push(uv.to_array());
        (self.positions.len() - 1) as u32
    }

    /// Append a triangle, flipping it if needed so its winding agrees with the
    /// vertex normals (counter-clockwise seen from the outside).
    pub fn push_triangle_facing(&mut self, a: u32, b: u32, c: u32) {
        let p = |i: u32| Vec3::from_array(self.positions[i as usize]);
        let n = |i: u32| Vec3::from_array(self.normals[i as usize]);
        let face = (p(b) - p(a)).cross(p(c) - p(a));
        if face.dot(n(a) + n(b) + n(c)) >= 0.0 {
            self.indices.extend([a, b, c]);
        } else {
            self.indices.extend([a, c, b]);
        }
    }

    /// Append another mesh.
    pub fn append(&mut self, other: &MeshData) {
        let base = self.positions.len() as u32;
        self.positions.extend_from_slice(&other.positions);
        self.normals.extend_from_slice(&other.normals);
        self.uvs.extend_from_slice(&other.uvs);
        self.indices.extend(other.indices.iter().map(|i| i + base));
    }

    /// Axis-aligned bounds.
    pub fn bounds(&self) -> (Vec3, Vec3) {
        let mut lo = Vec3::splat(f32::INFINITY);
        let mut hi = Vec3::splat(f32::NEG_INFINITY);
        for p in &self.positions {
            let p = Vec3::from_array(*p);
            lo = lo.min(p);
            hi = hi.max(p);
        }
        (lo, hi)
    }

    /// Volume by the divergence theorem (closed meshes only).
    pub fn volume(&self) -> f32 {
        self.indices
            .chunks_exact(3)
            .map(|t| {
                let p = |i: u32| Vec3::from_array(self.positions[i as usize]);
                p(t[0]).dot(p(t[1]).cross(p(t[2]))) / 6.0
            })
            .sum()
    }
}

/// Template-mesh resolution.
#[derive(Clone, Copy, Debug)]
pub struct TemplateOptions {
    /// Rings along the member. Spacing is cosine-clustered towards the ends,
    /// where plastic hinges put their kinks.
    pub rings: usize,
    /// Add end caps (members whose ends are hidden inside joints can skip them).
    pub caps: bool,
    /// Split each flat face into this many strips, and give the caps interior
    /// vertices to match. 1 = corners only.
    pub face_segments: usize,
    /// Interior rings in each cap when `face_segments > 1`.
    pub cap_rings: usize,
}

impl Default for TemplateOptions {
    fn default() -> Self {
        Self {
            rings: 17,
            caps: true,
            face_segments: 1,
            cap_rings: 0,
        }
    }
}

impl TemplateOptions {
    /// The template broken members switch to: enough vertices across the
    /// section and along the member for jagged, necked or splayed fracture
    /// ends. Intact members keep the lean default.
    pub fn detailed() -> Self {
        Self {
            rings: 25,
            caps: true,
            face_segments: 6,
            cap_rings: 3,
        }
    }
}

/// Ring parameters `s` in 0..1, clustered towards both ends.
pub fn ring_params(rings: usize) -> Vec<f32> {
    assert!(rings >= 2);
    (0..rings)
        .map(|i| {
            let t = i as f32 / (rings - 1) as f32;
            // Blend of uniform and cosine spacing: dense at the ends without
            // starving the middle of the span.
            let c = 0.5 - 0.5 * (std::f32::consts::PI * t).cos();
            0.35 * t + 0.65 * c
        })
        .collect()
}

/// The shared member template for a profile, in member space (see module docs).
pub fn member_template(profile: &Profile, opts: TemplateOptions) -> MeshData {
    let mut m = MeshData::default();
    let rings = ring_params(opts.rings);
    for lp in profile.outline_dense(opts.face_segments) {
        let n = lp.len();
        let base = m.positions.len() as u32;
        for &s in &rings {
            for v in &lp {
                m.push_vertex(
                    Vec3::new(s, v.p.x, v.p.y),
                    Vec3::new(0.0, v.n.x, v.n.y),
                    Vec2::new(s, v.u),
                );
            }
        }
        for r in 0..rings.len() - 1 {
            for k in 0..n - 1 {
                // Skip the zero-width quads between duplicated corner vertices.
                if (lp[k].p - lp[k + 1].p).length_squared() < 1e-14 {
                    continue;
                }
                let i0 = base + (r * n + k) as u32;
                let (i1, i2, i3) = (i0 + 1, i0 + n as u32, i0 + n as u32 + 1);
                m.push_triangle_facing(i0, i2, i1);
                m.push_triangle_facing(i1, i2, i3);
            }
            // Close polygon loops (circle loops repeat their first vertex).
            if (lp[n - 1].p - lp[0].p).length_squared() > 1e-14 {
                let i0 = base + (r * n + n - 1) as u32;
                let i1 = base + (r * n) as u32;
                let (i2, i3) = (i0 + n as u32, i1 + n as u32);
                m.push_triangle_facing(i0, i2, i1);
                m.push_triangle_facing(i1, i2, i3);
            }
        }
    }
    if opts.caps {
        let cap = profile.cap_dense(opts.face_segments, opts.cap_rings);
        for (s, nx) in [(0.0f32, -1.0f32), (1.0, 1.0)] {
            let base = m.positions.len() as u32;
            for p in &cap.points {
                m.push_vertex(
                    Vec3::new(s, p.x, p.y),
                    Vec3::new(nx, 0.0, 0.0),
                    Vec2::new(s, 0.0),
                );
            }
            for t in &cap.triangles {
                m.push_triangle_facing(base + t[0], base + t[1], base + t[2]);
            }
        }
    }
    m
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn template_is_closed_with_section_volume() {
        for p in [
            Profile::Rect { b: 0.1, h: 0.2 },
            Profile::IBeam {
                h: 0.3,
                b: 0.15,
                tw: 0.0071,
                tf: 0.0107,
            },
            Profile::BoxSection {
                b: 0.1,
                h: 0.15,
                t: 0.008,
            },
        ] {
            let m = member_template(&p, TemplateOptions::default());
            // Member space has unit length along s, so volume = area.
            let v = m.volume() as f64;
            let a = p.section().area;
            assert!(
                ((v - a) / a).abs() < 1e-3,
                "{p:?}: volume {v} vs area {a} (bad winding or holes)"
            );
        }
    }

    #[test]
    fn detailed_template_is_closed_too() {
        for p in [
            Profile::Rect { b: 0.1, h: 0.2 },
            Profile::Round { r: 0.05 },
            Profile::Tube { r: 0.05, t: 0.005 },
            Profile::BoxSection {
                b: 0.1,
                h: 0.15,
                t: 0.008,
            },
        ] {
            let lean = member_template(&p, TemplateOptions::default());
            let m = member_template(&p, TemplateOptions::detailed());
            assert!(m.vertex_count() > lean.vertex_count());
            let (v, a) = (m.volume(), lean.volume());
            assert!(((v - a) / a).abs() < 1e-3, "{p:?}: volume {v} vs {a}");
        }
    }

    #[test]
    fn ring_params_cover_unit_interval() {
        let r = ring_params(17);
        assert_eq!(r[0], 0.0);
        assert!((r[16] - 1.0).abs() < 1e-6);
        assert!(r.windows(2).all(|w| w[1] > w[0]));
        // Denser at the ends than in the middle.
        assert!(r[1] - r[0] < 0.5 * (r[9] - r[8]));
    }
}
