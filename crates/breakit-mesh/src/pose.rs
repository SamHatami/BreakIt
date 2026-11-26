//! Member pose: how a member's centreline and cross-sections sit in the world
//! this frame, and the per-member record the GPU bends the template with.
//!
//! The centreline is the cubic Hermite curve through the two end nodes with
//! the beam's end tangents. For an Euler-Bernoulli beam without span loads
//! the exact deflected shape *is* that cubic, so elastic bending is drawn
//! exactly from nothing but the node positions and rotations the solver
//! already has.
//!
//! A plastic hinge at an end is a concentrated rotation between the node and
//! the beam. It is drawn as a short straight stub that follows the node, then
//! a crease, then the elastic curve leaving at the beam's own end tangent.
//!
//! [`MemberPose::deform`] is the reference implementation of
//! `breakit-render`'s vertex shader (`member.wesl`); keep the two in step.

use glam::{Mat3, Quat, Vec3, Vec4};

/// Where a member is this frame.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MemberPose {
    /// End positions.
    pub pa: Vec3,
    pub pb: Vec3,
    /// Member frames at each end, carried by the end node: local x along the
    /// member, y and z the section axes.
    pub qa: Quat,
    pub qb: Quat,
    /// Unit tangents of the elastic part of the beam at each end. Equal to
    /// the frames' x axes unless a plastic hinge has formed there.
    pub ta: Vec3,
    pub tb: Vec3,
    /// Length of the straight hinge stub at each end, m (0 without a hinge).
    pub stub_a: f32,
    pub stub_b: f32,
    /// Undeformed length, m. Maps the template's `s` to arc length.
    pub rest_length: f32,
}

/// Member frame (x along a→b, y up-ish, z sideways) as a rotation, matching
/// `breakit_core::frame::member_axes`.
pub fn rest_frame(a: Vec3, b: Vec3) -> Quat {
    let x = (b - a).normalize();
    let up = if x.y.abs() > 0.999 { Vec3::X } else { Vec3::Y };
    let z = x.cross(up).normalize();
    let y = z.cross(x);
    Quat::from_mat3(&Mat3::from_cols(x, y, z)).normalize()
}

impl MemberPose {
    /// A straight, unloaded member.
    pub fn straight(a: Vec3, b: Vec3) -> Self {
        let q = rest_frame(a, b);
        let t = (b - a).normalize();
        Self {
            pa: a,
            pb: b,
            qa: q,
            qb: q,
            ta: t,
            tb: t,
            stub_a: 0.0,
            stub_b: 0.0,
            rest_length: (b - a).length(),
        }
    }

    /// Pose from solver output.
    ///
    /// * `rest`: the member's rest frame ([`rest_frame`] of the undeformed ends)
    /// * `node_qa`, `node_qb`: the end nodes' rotations since rest
    /// * `hinge`: plastic hinge rotations, `[axis y, axis z][end a, end b]`, rad,
    ///   with the sign convention of `breakit_core::dynamics`
    /// * `section_depth`: used for the hinge stub length
    #[allow(clippy::too_many_arguments)]
    pub fn from_nodes(
        pa: Vec3,
        pb: Vec3,
        rest: Quat,
        node_qa: Quat,
        node_qb: Quat,
        hinge: [[f32; 2]; 2],
        rest_length: f32,
        section_depth: f32,
    ) -> Self {
        let qa = (node_qa * rest).normalize();
        let qb = (node_qb * rest).normalize();
        let (ta_node, tb_node) = (qa * Vec3::X, qb * Vec3::X);
        // The solver measures end rotations about the member's current mean
        // section axes; each end's own axes are close enough here.
        let beam_tangent = |q: Quat, t: Vec3, end: usize| {
            let (u_y, u_z) = (q * Vec3::Y, q * Vec3::Z);
            let r = Quat::from_axis_angle(u_y, -hinge[0][end])
                * Quat::from_axis_angle(u_z, -hinge[1][end]);
            (r * t).normalize()
        };
        let stub = |end: usize| {
            let mag = (hinge[0][end].powi(2) + hinge[1][end].powi(2)).sqrt();
            let grow = smoothstep(0.002, 0.02, mag);
            (section_depth.min(0.25 * rest_length)) * grow
        };
        Self {
            pa,
            pb,
            qa,
            qb,
            ta: beam_tangent(qa, ta_node, 0),
            tb: beam_tangent(qb, tb_node, 1),
            stub_a: stub(0),
            stub_b: stub(1),
            rest_length,
        }
    }

    /// Centreline point and unit tangent at template parameter `s` (0..1).
    pub fn centreline(&self, s: f32) -> (Vec3, Vec3) {
        let l0 = self.rest_length.max(1e-6);
        let (ta_node, tb_node) = (self.qa * Vec3::X, self.qb * Vec3::X);
        let (sa, sb) = (self.stub_a / l0, self.stub_b / l0);
        if s < sa {
            return (self.pa + ta_node * (s * l0), ta_node);
        }
        if s > 1.0 - sb {
            return (self.pb - tb_node * ((1.0 - s) * l0), tb_node);
        }
        let u = ((s - sa) / (1.0 - sa - sb).max(1e-6)).clamp(0.0, 1.0);
        let p0 = self.pa + ta_node * self.stub_a;
        let p1 = self.pb - tb_node * self.stub_b;
        let lc = (p1 - p0).length();
        let (m0, m1) = (self.ta * lc, self.tb * lc);
        let (u2, u3) = (u * u, u * u * u);
        let p = (2.0 * u3 - 3.0 * u2 + 1.0) * p0
            + (u3 - 2.0 * u2 + u) * m0
            + (-2.0 * u3 + 3.0 * u2) * p1
            + (u3 - u2) * m1;
        let d = (6.0 * u2 - 6.0 * u) * p0
            + (3.0 * u2 - 4.0 * u + 1.0) * m0
            + (-6.0 * u2 + 6.0 * u) * p1
            + (3.0 * u2 - 2.0 * u) * m1;
        let t = d.try_normalize().unwrap_or(ta_node);
        (p, t)
    }

    /// Section frame at `s`: the end frames blended, then turned so local x
    /// follows the curve tangent.
    pub fn frame(&self, s: f32, tangent: Vec3) -> Quat {
        let qb = if self.qa.dot(self.qb) < 0.0 {
            -self.qb
        } else {
            self.qb
        };
        // Normalised lerp, as in the shader.
        let q = (self.qa + (qb - self.qa) * s).normalize();
        let x = q * Vec3::X;
        (Quat::from_rotation_arc(x, tangent) * q).normalize()
    }

    /// Bend one template vertex: member-space position `(s, y, z)` and
    /// normal to world space. The CPU twin of the vertex shader.
    pub fn deform(&self, local: Vec3, normal: Vec3) -> (Vec3, Vec3) {
        let s = local.x;
        let (c, t) = self.centreline(s);
        let q = self.frame(s, t);
        (c + q * Vec3::new(0.0, local.y, local.z), q * normal)
    }

    /// The same pose moved rigidly: rotated by `q` about the origin, then
    /// translated by `t`.
    pub fn transformed(&self, q: Quat, t: Vec3) -> Self {
        Self {
            pa: q * self.pa + t,
            pb: q * self.pb + t,
            qa: (q * self.qa).normalize(),
            qb: (q * self.qb).normalize(),
            ta: q * self.ta,
            tb: q * self.tb,
            ..*self
        }
    }

    /// Blend towards `other` (positions and tangents linearly, frames by
    /// slerp). `t` may overshoot 0..1, which the spring-back of freshly
    /// broken members uses.
    pub fn blend(&self, other: &Self, t: f32) -> Self {
        let mix = |a: Vec3, b: Vec3| a + (b - a) * t;
        let dir = |a: Vec3, b: Vec3| mix(a, b).try_normalize().unwrap_or(a);
        let rot = |a: Quat, b: Quat| {
            let b = if a.dot(b) < 0.0 { -b } else { b };
            (a + (b - a) * t).normalize()
        };
        Self {
            pa: mix(self.pa, other.pa),
            pb: mix(self.pb, other.pb),
            qa: rot(self.qa, other.qa),
            qb: rot(self.qb, other.qb),
            ta: dir(self.ta, other.ta),
            tb: dir(self.tb, other.tb),
            stub_a: self.stub_a + (other.stub_a - self.stub_a) * t,
            stub_b: self.stub_b + (other.stub_b - self.stub_b) * t,
            rest_length: self.rest_length,
        }
    }

    /// World-space bounds of the deformed member, padded by `radius` (the
    /// section's bounding radius). Uses the Bézier hull of the curve, which
    /// always contains it.
    pub fn bounds(&self, radius: f32) -> (Vec3, Vec3) {
        let (ta_node, tb_node) = (self.qa * Vec3::X, self.qb * Vec3::X);
        let p0 = self.pa + ta_node * self.stub_a;
        let p1 = self.pb - tb_node * self.stub_b;
        let lc = (p1 - p0).length();
        let pts = [
            self.pa,
            p0,
            p0 + self.ta * (lc / 3.0),
            p1 - self.tb * (lc / 3.0),
            p1,
            self.pb,
        ];
        let mut lo = pts[0];
        let mut hi = pts[0];
        for p in pts {
            lo = lo.min(p);
            hi = hi.max(p);
        }
        (lo - Vec3::splat(radius), hi + Vec3::splat(radius))
    }
}

pub fn smoothstep(e0: f32, e1: f32, x: f32) -> f32 {
    let t = ((x - e0) / (e1 - e0)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

/// Per-member shader input: 11 × vec4, 176 bytes. Layout mirrors the
/// `Member` struct in `member.wesl`. This is all the CPU writes per member
/// per frame, however detailed the member's mesh is.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq, bytemuck::Pod, bytemuck::Zeroable)]
pub struct GpuMember {
    /// xyz end a, w hinge stub length at a.
    pub pa: Vec4,
    /// xyz end b, w hinge stub length at b.
    pub pb: Vec4,
    /// Member frame at a and b (x, y, z, w quaternion).
    pub qa: Vec4,
    pub qb: Vec4,
    /// xyz elastic tangent at a, w hinge damage at a (0..1 of capacity).
    pub ta: Vec4,
    /// xyz elastic tangent at b, w hinge damage at b.
    pub tb: Vec4,
    /// utilisation, crack at a, crack at b, crack along the span.
    pub damage0: Vec4,
    /// crush at a, crush at b, delamination, yielded (0/1).
    pub damage1: Vec4,
    /// rest length, random seed, material kind, flags.
    pub info: Vec4,
    /// Drawn window along the member (`s` from x to y) and whether each end
    /// of the window is a fracture face (z for the low end, w for the high
    /// end, 0 or 1). An intact member is `(0, 1, 0, 0)`.
    pub clip: Vec4,
    /// section depth, section width, seconds since the member broke, unused.
    pub size: Vec4,
}

/// Flags in `GpuMember::info.w`.
pub mod flags {
    /// The member is gone (broken); the shader collapses it.
    pub const HIDDEN: u32 = 1;
    /// Show the stress heatmap instead of the material.
    pub const STRESS_VIEW: u32 = 2;
    /// Highlight (selection / hover).
    pub const HIGHLIGHT: u32 = 4;
    /// A reinforcing bar exposed by a concrete break (ribbed steel look).
    pub const REBAR: u32 = 8;
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: Vec3, b: Vec3, tol: f32) -> bool {
        (a - b).length() < tol
    }

    #[test]
    fn rest_frame_matches_core_member_axes() {
        use breakit_core::frame::member_axes;
        use breakit_core::Vec3 as V;
        for (a, b) in [
            (V::new(0.0, 0.0, 0.0), V::new(3.0, 1.0, 0.5)),
            (V::new(1.0, 0.0, 0.0), V::new(1.0, 2.0, 0.0)), // vertical
            (V::new(0.0, 1.0, 0.0), V::new(0.0, 1.0, -2.0)),
        ] {
            let (x, y, z) = member_axes(&a, &b);
            let q = rest_frame(
                Vec3::new(a.x as f32, a.y as f32, a.z as f32),
                Vec3::new(b.x as f32, b.y as f32, b.z as f32),
            );
            let g = |v: V| Vec3::new(v.x as f32, v.y as f32, v.z as f32);
            assert!(close(q * Vec3::X, g(x), 1e-5));
            assert!(close(q * Vec3::Y, g(y), 1e-5));
            assert!(close(q * Vec3::Z, g(z), 1e-5));
        }
    }

    #[test]
    fn straight_member_is_an_extrusion() {
        let (a, b) = (Vec3::new(1.0, 2.0, 3.0), Vec3::new(4.0, 2.0, 3.0));
        let pose = MemberPose::straight(a, b);
        for s in [0.0, 0.3, 1.0] {
            let (p, n) = pose.deform(Vec3::new(s, 0.1, -0.05), Vec3::Y);
            assert!(close(p, a.lerp(b, s) + Vec3::new(0.0, 0.1, -0.05), 1e-5));
            assert!(close(n, Vec3::Y, 1e-5));
        }
    }

    #[test]
    fn rigid_motion_moves_member_rigidly() {
        let (a, b) = (Vec3::ZERO, Vec3::new(2.0, 0.0, 0.0));
        let rest = rest_frame(a, b);
        let r = Quat::from_euler(glam::EulerRot::XYZ, 0.3, -0.7, 1.1);
        let t = Vec3::new(5.0, -1.0, 2.0);
        let pose =
            MemberPose::from_nodes(r * a + t, r * b + t, rest, r, r, [[0.0; 2]; 2], 2.0, 0.2);
        let straight = MemberPose::straight(a, b);
        for s in [0.0, 0.25, 0.5, 0.9, 1.0] {
            let local = Vec3::new(s, 0.07, 0.03);
            let (p, n) = pose.deform(local, Vec3::Z);
            let (p0, n0) = straight.deform(local, Vec3::Z);
            assert!(close(p, r * p0 + t, 1e-4), "s={s}");
            assert!(close(n, r * n0, 1e-4));
        }
    }

    #[test]
    fn hinge_puts_a_kink_at_the_stub() {
        // Horizontal member, clamped end a level, hinge rotating the beam down.
        let (a, b) = (Vec3::ZERO, Vec3::new(2.0, -0.2, 0.0));
        let rest = rest_frame(Vec3::ZERO, Vec3::new(2.0, 0.0, 0.0));
        let pose = MemberPose::from_nodes(
            a,
            b,
            rest,
            Quat::IDENTITY,
            Quat::from_rotation_z(-0.1),
            [[0.0, 0.0], [0.1, 0.0]],
            2.0,
            0.2,
        );
        assert!(pose.stub_a > 0.19);
        let (_, t_stub) = pose.centreline(0.05);
        let (_, t_after) = pose.centreline(0.11);
        assert!(close(t_stub, Vec3::X, 1e-5), "stub follows the node");
        // The beam leaves the crease pointing down.
        assert!(t_after.y < -0.05, "tangent after hinge {t_after}");
    }
}
