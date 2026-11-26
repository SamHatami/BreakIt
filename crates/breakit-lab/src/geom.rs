//! A small mesh builder for static, procedural scenery.
//!
//! Everything in the lab is made of boxes, cylinders, spheres and quads
//! appended into one [`MeshBuf`] per material, so the whole hall draws in a
//! couple of dozen draw calls and costs nothing per frame. No assets.

use std::f32::consts::TAU;

use bevy::asset::RenderAssetUsages;
use bevy::math::{Quat, Vec2, Vec3};
use bevy::mesh::{Indices, Mesh, PrimitiveTopology};

#[derive(Clone, Debug, Default)]
pub struct MeshBuf {
    pub positions: Vec<[f32; 3]>,
    pub normals: Vec<[f32; 3]>,
    pub uvs: Vec<[f32; 2]>,
    pub indices: Vec<u32>,
}

/// An orthonormal frame along a member from `a` to `b`: `x` along the
/// member, `y` as close to `up` as the member allows, `z` to the side.
pub fn frame_along(a: Vec3, b: Vec3, up: Vec3) -> (Vec3, Vec3, Vec3) {
    let x = (b - a).normalize_or(Vec3::X);
    let up = if x.cross(up).length_squared() < 1e-6 {
        if x.y.abs() < 0.9 {
            Vec3::Y
        } else {
            Vec3::X
        }
    } else {
        up
    };
    let z = x.cross(up).normalize();
    let y = z.cross(x);
    (x, y, z)
}

impl MeshBuf {
    pub fn is_empty(&self) -> bool {
        self.indices.is_empty()
    }

    pub fn triangles(&self) -> usize {
        self.indices.len() / 3
    }

    fn base(&self) -> u32 {
        self.positions.len() as u32
    }

    fn vert(&mut self, p: Vec3, n: Vec3, uv: Vec2) {
        self.positions.push(p.to_array());
        self.normals.push(n.to_array());
        self.uvs.push(uv.to_array());
    }

    /// A flat quad with corners in counter-clockwise order seen from the
    /// front (the side its normal points to).
    pub fn quad(&mut self, c: [Vec3; 4]) {
        let n = (c[1] - c[0]).cross(c[2] - c[0]).normalize_or(Vec3::Y);
        let b = self.base();
        let uv = [Vec2::ZERO, Vec2::X, Vec2::ONE, Vec2::Y];
        for k in 0..4 {
            self.vert(c[k], n, uv[k]);
        }
        self.indices
            .extend_from_slice(&[b, b + 1, b + 2, b, b + 2, b + 3]);
    }

    /// A flat triangle, counter-clockwise seen from the front.
    pub fn tri(&mut self, a: Vec3, b: Vec3, c: Vec3) {
        let n = (b - a).cross(c - a).normalize_or(Vec3::Y);
        let i = self.base();
        self.vert(a, n, Vec2::ZERO);
        self.vert(b, n, Vec2::X);
        self.vert(c, n, Vec2::Y);
        self.indices.extend_from_slice(&[i, i + 1, i + 2]);
    }

    /// A box with centre `c`, half extents `h` along the rotated axes.
    pub fn cuboid(&mut self, c: Vec3, h: Vec3, rot: Quat) {
        let ax = [rot * Vec3::X, rot * Vec3::Y, rot * Vec3::Z];
        self.oriented_box(c, ax, h);
    }

    /// An axis-aligned box from `min` to `max`.
    pub fn aabb(&mut self, min: Vec3, max: Vec3) {
        self.cuboid((min + max) * 0.5, (max - min).abs() * 0.5, Quat::IDENTITY);
    }

    fn oriented_box(&mut self, c: Vec3, ax: [Vec3; 3], h: Vec3) {
        let (x, y, z) = (ax[0] * h.x, ax[1] * h.y, ax[2] * h.z);
        let faces = [
            (x, y, z),
            (-x, z, y),
            (y, z, x),
            (-y, x, z),
            (z, x, y),
            (-z, y, x),
        ];
        for (n, u, v) in faces {
            // Corners n - u - v, n + u - v, n + u + v, n - u + v, wound so
            // the normal points out.
            let (u, v) = if u.cross(v).dot(n) > 0.0 {
                (u, v)
            } else {
                (v, u)
            };
            self.quad([c + n - u - v, c + n + u - v, c + n + u + v, c + n - u + v]);
        }
    }

    /// A box from `a` to `b` with cross-section `w` (sideways) by `h`
    /// (towards `up`).
    pub fn bar(&mut self, a: Vec3, b: Vec3, w: f32, h: f32, up: Vec3) {
        let (x, y, z) = frame_along(a, b, up);
        let len = a.distance(b);
        self.oriented_box(
            (a + b) * 0.5,
            [x, y, z],
            Vec3::new(len * 0.5, h * 0.5, w * 0.5),
        );
    }

    /// An I-section from `a` to `b`: overall `depth` towards `up`, flange
    /// `width`, web and flange thickness `tw`, `tf`.
    #[allow(clippy::too_many_arguments)]
    pub fn i_beam(&mut self, a: Vec3, b: Vec3, depth: f32, width: f32, tw: f32, tf: f32, up: Vec3) {
        let (_, y, _) = frame_along(a, b, up);
        let off = y * (depth - tf) * 0.5;
        self.bar(a + off, b + off, width, tf, up);
        self.bar(a - off, b - off, width, tf, up);
        self.bar(a, b, tw, depth - 2.0 * tf, up);
    }

    /// A smooth cylinder from `a` to `b`, with flat caps if `caps`.
    pub fn cylinder(&mut self, a: Vec3, b: Vec3, r: f32, segs: u32, caps: bool) {
        self.frustum(a, b, r, r, segs, caps);
    }

    /// A cone frustum from radius `ra` at `a` to `rb` at `b`.
    pub fn frustum(&mut self, a: Vec3, b: Vec3, ra: f32, rb: f32, segs: u32, caps: bool) {
        let (x, y, z) = frame_along(a, b, Vec3::Y);
        let len = a.distance(b).max(1e-6);
        let slope = (ra - rb) / len;
        let base = self.base();
        for i in 0..=segs {
            let t = i as f32 / segs as f32;
            let (s, c) = (t * TAU).sin_cos();
            let radial = y * c + z * s;
            let n = (radial + x * slope).normalize();
            self.vert(a + radial * ra, n, Vec2::new(t, 0.0));
            self.vert(b + radial * rb, n, Vec2::new(t, 1.0));
        }
        for i in 0..segs {
            let k = base + i * 2;
            self.indices
                .extend_from_slice(&[k, k + 2, k + 1, k + 1, k + 2, k + 3]);
        }
        if caps {
            self.disc(a, -x, ra, segs);
            self.disc(b, x, rb, segs);
        }
    }

    /// A flat disc facing `n`.
    pub fn disc(&mut self, c: Vec3, n: Vec3, r: f32, segs: u32) {
        if r <= 0.0 {
            return;
        }
        let (_, y, z) = frame_along(c, c + n, Vec3::Y);
        let base = self.base();
        self.vert(c, n, Vec2::splat(0.5));
        for i in 0..segs {
            let (s, co) = (i as f32 / segs as f32 * TAU).sin_cos();
            self.vert(c + (y * co + z * s) * r, n, Vec2::new(co, s) * 0.5 + 0.5);
        }
        for i in 0..segs {
            let a = base + 1 + i;
            let b = base + 1 + (i + 1) % segs;
            self.indices.extend_from_slice(&[base, a, b]);
        }
    }

    /// A ring (flange, hoop) of square-ish section around the axis `a`->`b`
    /// at `a`: outer radius `r`, radial thickness `t`, axial width `w`.
    pub fn ring(&mut self, a: Vec3, axis: Vec3, r: f32, t: f32, w: f32, segs: u32) {
        let axis = axis.normalize_or(Vec3::Y);
        let p0 = a - axis * w * 0.5;
        let p1 = a + axis * w * 0.5;
        self.cylinder(p0, p1, r, segs, false);
        self.annulus(p1, axis, r, r - t, segs);
        self.annulus(p0, -axis, r, r - t, segs);
    }

    /// A flat ring facing `n` between radii `r_in` and `r_out`.
    pub fn annulus(&mut self, c: Vec3, n: Vec3, r_out: f32, r_in: f32, segs: u32) {
        let (_, y, z) = frame_along(c, c + n, Vec3::Y);
        let base = self.base();
        for i in 0..=segs {
            let (s, co) = (i as f32 / segs as f32 * TAU).sin_cos();
            let d = y * co + z * s;
            self.vert(c + d * r_out, n, Vec2::new(i as f32 / segs as f32, 0.0));
            self.vert(c + d * r_in, n, Vec2::new(i as f32 / segs as f32, 1.0));
        }
        for i in 0..segs {
            let k = base + i * 2;
            self.indices
                .extend_from_slice(&[k, k + 2, k + 1, k + 1, k + 2, k + 3]);
        }
    }

    /// A UV sphere, or the top half of one if `dome`.
    pub fn sphere(&mut self, c: Vec3, r: f32, segs: u32, dome: bool) {
        let rings = (segs / 2).max(2);
        let base = self.base();
        let end = if dome { rings / 2 } else { rings };
        for j in 0..=end {
            let v = j as f32 / rings as f32;
            let phi = v * std::f32::consts::PI;
            let (sp, cp) = phi.sin_cos();
            for i in 0..=segs {
                let u = i as f32 / segs as f32;
                let (st, ct) = (u * TAU).sin_cos();
                let n = Vec3::new(sp * ct, -cp, sp * st);
                // Flip so `dome` keeps the upper half.
                let n = Vec3::new(n.x, -n.y, n.z);
                self.vert(c + n * r, n, Vec2::new(u, v));
            }
        }
        let cols = segs + 1;
        for j in 0..end {
            for i in 0..segs {
                let a = base + j * cols + i;
                let b = a + cols;
                self.indices
                    .extend_from_slice(&[a, a + 1, b, a + 1, b + 1, b]);
            }
        }
    }

    /// Appends `other`, moved by `offset`.
    pub fn append(&mut self, other: &MeshBuf, offset: Vec3) {
        let base = self.base();
        self.positions.extend(
            other
                .positions
                .iter()
                .map(|p| (Vec3::from_array(*p) + offset).to_array()),
        );
        self.normals.extend_from_slice(&other.normals);
        self.uvs.extend_from_slice(&other.uvs);
        self.indices.extend(other.indices.iter().map(|i| i + base));
    }

    pub fn into_mesh(self) -> Mesh {
        Mesh::new(
            PrimitiveTopology::TriangleList,
            RenderAssetUsages::default(),
        )
        .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, self.positions)
        .with_inserted_attribute(Mesh::ATTRIBUTE_NORMAL, self.normals)
        .with_inserted_attribute(Mesh::ATTRIBUTE_UV_0, self.uvs)
        .with_inserted_indices(Indices::U32(self.indices))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Every triangle's winding agrees with its vertex normals, so faces
    /// are not culled from the outside.
    fn assert_outward(m: &MeshBuf, centre: Vec3) {
        for t in m.indices.chunks(3) {
            let p: Vec<Vec3> = t
                .iter()
                .map(|&i| Vec3::from_array(m.positions[i as usize]))
                .collect();
            let face_n = (p[1] - p[0]).cross(p[2] - p[0]);
            if face_n.length_squared() < 1e-12 {
                continue;
            }
            let mid = (p[0] + p[1] + p[2]) / 3.0;
            assert!(face_n.dot(mid - centre) > 0.0, "inward face at {mid}");
            let n = Vec3::from_array(m.normals[t[0] as usize]);
            assert!(n.dot(face_n) > 0.0);
        }
    }

    #[test]
    fn box_faces_point_out() {
        let mut m = MeshBuf::default();
        m.cuboid(
            Vec3::new(1.0, 2.0, 3.0),
            Vec3::new(0.5, 1.0, 2.0),
            Quat::from_rotation_y(0.3),
        );
        assert_eq!(m.triangles(), 12);
        assert_outward(&m, Vec3::new(1.0, 2.0, 3.0));
    }

    #[test]
    fn bar_spans_its_ends() {
        let mut m = MeshBuf::default();
        let (a, b) = (Vec3::ZERO, Vec3::new(3.0, 4.0, 0.0));
        m.bar(a, b, 0.2, 0.1, Vec3::Y);
        assert_outward(&m, (a + b) * 0.5);
        let max_d = m
            .positions
            .iter()
            .map(|p| Vec3::from_array(*p).distance((a + b) * 0.5))
            .fold(0.0, f32::max);
        assert!((max_d - (2.5f32.powi(2) + 0.1f32.powi(2) + 0.05f32.powi(2)).sqrt()).abs() < 1e-4);
    }

    #[test]
    fn cylinder_and_sphere_point_out() {
        let mut m = MeshBuf::default();
        m.cylinder(Vec3::ZERO, Vec3::new(0.0, 0.0, 2.0), 0.5, 12, true);
        assert_outward(&m, Vec3::new(0.0, 0.0, 1.0));
        let mut s = MeshBuf::default();
        s.sphere(Vec3::ONE, 1.0, 12, false);
        assert_outward(&s, Vec3::ONE);
        let mut d = MeshBuf::default();
        d.sphere(Vec3::ZERO, 1.0, 12, true);
        assert!(d.positions.iter().all(|p| p[1] >= -1e-5));
    }
}
