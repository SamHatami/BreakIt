//! Connection hardware at member ends: what a joint looks like and where it
//! sits each frame. No engine dependency.
//!
//! A member is drawn from its node to its node, but a real one stops short:
//! there is a node (a cast steel ball, a block of concrete) and something
//! that fixes the member to it. With joints set on a [`VisualStructure`],
//! each member is trimmed back by its node's radius and the gap is filled
//! with that hardware:
//!
//! | joint | what is drawn |
//! |---|---|
//! | pinned | a clevis on the node, a pin through the member end |
//! | bolted | an end plate on the member, a matching plate on the node, bolts |
//! | welded | the member butted on a node plate, a fillet weld round it |
//! | monolithic | a concrete haunch that flares into the node |
//! | dowelled | a steel plate slotted into the timber, rows of dowels |
//! | sleeve | a steel shoe round the member end (rigid timber, composite) |
//!
//! Every joint is two rigid parts, one carried by the node and one by the
//! member end, so a pin visibly turns in its clevis and a bolted end plate
//! opens away from its partner as the joint yields. Parts are plain meshes in
//! *joint space* (origin on the member axis where the member is cut, +x into
//! the member, y and z the section axes), one mesh per (joint, profile), so
//! all members of a kind share them and the renderer can instance them. Per
//! frame a joint costs two transforms.

use std::f32::consts::{PI, TAU};

use glam::{Quat, Vec2, Vec3};

use breakit_core::{Joint, JointKind};

use crate::mesh::MeshData;
use crate::pose::MemberPose;
use crate::profile::Profile;
use crate::state::MaterialKind;

/// What is drawn at a member end.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub enum JointLook {
    /// Nothing: the member runs to the node's centre.
    #[default]
    None,
    Pinned,
    Bolted,
    Welded,
    Monolithic,
    Dowelled,
    /// A steel shoe: how a rigid joint is made in timber or composite.
    Sleeve,
}

impl JointLook {
    /// The hardware for a solver joint on a member of `material`. A rigid
    /// joint is drawn as whatever makes one in that material.
    pub fn of(kind: JointKind, material: MaterialKind) -> Self {
        use MaterialKind as M;
        match kind {
            JointKind::Pinned => JointLook::Pinned,
            JointKind::Bolted => JointLook::Bolted,
            JointKind::Welded => JointLook::Welded,
            JointKind::Monolithic => JointLook::Monolithic,
            JointKind::Dowelled => JointLook::Dowelled,
            JointKind::Rigid => match material {
                M::Steel => JointLook::Welded,
                M::Concrete | M::ReinforcedConcrete => JointLook::Monolithic,
                M::Timber | M::Composite => JointLook::Sleeve,
            },
        }
    }

    /// The hardware for a solver joint known only by its numbers: a pin
    /// where it passes no moment, a weld or a cast node where it is rigid,
    /// and for a semi-rigid joint the usual one in that material.
    pub fn infer(joint: &Joint, material: MaterialKind) -> Self {
        use MaterialKind as M;
        if joint.is_pinned() {
            JointLook::Pinned
        } else if joint.stiffness.iter().all(|k| k.is_infinite()) {
            JointLook::of(JointKind::Rigid, material)
        } else {
            match material {
                M::Steel | M::Composite => JointLook::Bolted,
                M::Timber => JointLook::Dowelled,
                M::Concrete | M::ReinforcedConcrete => JointLook::Monolithic,
            }
        }
    }
}

/// Which body a part is fixed to.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Side {
    Node,
    Member,
}

/// Surface of a part.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum PartLook {
    /// Painted plate and castings, the colour of the steelwork.
    Plate,
    /// Bolts, pins, dowels: bright zinc.
    Fastener,
    /// Weld bead: dull, dark, rippled.
    Weld,
    Concrete,
}

#[derive(Clone, Debug)]
pub struct JointPart {
    pub side: Side,
    pub look: PartLook,
    /// True for the things that go when the connection fails: the pin, the
    /// bolts, the dowels, the weld.
    pub fuse: bool,
    pub mesh: MeshData,
}

// ---------------------------------------------------------------- meshes

fn quad(m: &mut MeshData, c: [Vec3; 4]) {
    let n = (c[1] - c[0]).cross(c[2] - c[0]).normalize_or_zero();
    let uv = [Vec2::ZERO, Vec2::X, Vec2::ONE, Vec2::Y];
    let i: Vec<u32> = (0..4).map(|k| m.push_vertex(c[k], n, uv[k])).collect();
    m.indices.extend([i[0], i[1], i[2], i[0], i[2], i[3]]);
}

/// Axis-aligned box.
fn cuboid(m: &mut MeshData, lo: Vec3, hi: Vec3) {
    let p = |x: bool, y: bool, z: bool| {
        Vec3::new(
            if x { hi.x } else { lo.x },
            if y { hi.y } else { lo.y },
            if z { hi.z } else { lo.z },
        )
    };
    let (f, t) = (false, true);
    quad(m, [p(t, f, f), p(t, t, f), p(t, t, t), p(t, f, t)]);
    quad(m, [p(f, f, f), p(f, f, t), p(f, t, t), p(f, t, f)]);
    quad(m, [p(f, t, f), p(f, t, t), p(t, t, t), p(t, t, f)]);
    quad(m, [p(f, f, f), p(t, f, f), p(t, f, t), p(f, f, t)]);
    quad(m, [p(f, f, t), p(t, f, t), p(t, t, t), p(f, t, t)]);
    quad(m, [p(f, f, f), p(f, t, f), p(t, t, f), p(t, f, f)]);
}

/// A prism of `sides` round the segment `a` to `b`, radii `ra` and `rb`,
/// capped. `smooth` shares normals round the barrel (a cylinder); otherwise
/// each face is flat (a hexagon head).
fn prism(m: &mut MeshData, a: Vec3, b: Vec3, ra: f32, rb: f32, sides: usize, smooth: bool) {
    let x = (b - a).normalize_or(Vec3::X);
    let up = if x.y.abs() > 0.9 { Vec3::X } else { Vec3::Y };
    let u = x.cross(up).normalize();
    let v = x.cross(u);
    let dir = |k: usize| {
        let t = k as f32 / sides as f32 * TAU;
        u * t.cos() + v * t.sin()
    };
    for k in 0..sides {
        let (d0, d1) = (dir(k), dir(k + 1));
        if smooth {
            let i0 = m.push_vertex(a + d0 * ra, d0, Vec2::ZERO);
            let i1 = m.push_vertex(a + d1 * ra, d1, Vec2::X);
            let i2 = m.push_vertex(b + d1 * rb, d1, Vec2::ONE);
            let i3 = m.push_vertex(b + d0 * rb, d0, Vec2::Y);
            m.push_triangle_facing(i0, i1, i2);
            m.push_triangle_facing(i0, i2, i3);
        } else {
            let c = [a + d0 * ra, a + d1 * ra, b + d1 * rb, b + d0 * rb];
            let n = (c[1] - c[0]).cross(c[3] - c[0]);
            if n.dot(d0 + d1) >= 0.0 {
                quad(m, c);
            } else {
                quad(m, [c[1], c[0], c[3], c[2]]);
            }
        }
        for (centre, r, n) in [(a, ra, -x), (b, rb, x)] {
            let i0 = m.push_vertex(centre, n, Vec2::ZERO);
            let i1 = m.push_vertex(centre + d0 * r, n, Vec2::X);
            let i2 = m.push_vertex(centre + d1 * r, n, Vec2::Y);
            m.push_triangle_facing(i0, i1, i2);
        }
    }
}

/// A bolt along `axis` through `at`: hex head on the +axis side of a
/// package `grip` thick, nut and a thread's worth of shank on the other.
fn bolt(m: &mut MeshData, at: Vec3, axis: Vec3, grip: f32, d: f32) {
    let head = at + axis * (0.5 * grip);
    let nut = at - axis * (0.5 * grip);
    prism(
        m,
        head,
        head + axis * (0.65 * d),
        0.9 * d,
        0.9 * d,
        6,
        false,
    );
    prism(m, nut - axis * (0.8 * d), nut, 0.9 * d, 0.9 * d, 6, false);
    prism(m, nut - axis * (1.3 * d), nut, 0.5 * d, 0.5 * d, 8, true);
}

/// Sizes every joint is built from.
#[derive(Clone, Copy, Debug)]
struct Dims {
    /// Section half depth (y) and half width (z).
    hy: f32,
    hz: f32,
    /// Section depth, the unit most details scale with.
    d: f32,
    /// Distance from the cut back to the node's centre.
    back: f32,
    /// Plate thickness.
    tp: f32,
}

fn dims(profile: &Profile, back: f32) -> Dims {
    let (hy, hz) = (profile.half_depth(), profile.half_width());
    let d = 2.0 * hy.max(hz);
    Dims {
        hy,
        hz,
        d,
        back,
        tp: (0.07 * d).clamp(0.004, 0.03),
    }
}

/// A stub from the node (at x = -back) to x = `to`, a little slimmer than
/// the section, so plates and clevises have something to stand on.
fn neck(m: &mut MeshData, k: &Dims, to: f32) {
    let r = 0.62 * k.hy.min(k.hz).max(0.3 * k.hy.max(k.hz));
    prism(
        m,
        Vec3::new(-k.back, 0.0, 0.0),
        Vec3::new(to, 0.0, 0.0),
        1.25 * r,
        r,
        10,
        true,
    );
}

/// A fillet weld round the profile where it meets a plate at x = 0: a
/// rippled bead along every outline of the section.
fn weld_bead(m: &mut MeshData, profile: &Profile, leg: f32) {
    for outline in profile.outline_dense(24) {
        let n = outline.len();
        if n < 2 {
            continue;
        }
        // Three rows across the bead: toe on the plate, crown, toe on the
        // member.
        let mut rows: Vec<[u32; 3]> = Vec::with_capacity(n);
        let mut along = 0.0;
        for (i, v) in outline.iter().enumerate() {
            if i > 0 {
                along += (v.p - outline[i - 1].p).length();
            }
            // Ripples: the weld pool freezes as it is laid.
            let ripple = 1.0 + 0.16 * (along / (0.7 * leg) * TAU).sin();
            let l = leg * ripple;
            let at = |out: f32, x: f32| {
                let q = v.p + v.n * out;
                Vec3::new(x, q.x, q.y)
            };
            let nrm = |out: f32, x: f32| Vec3::new(x, v.n.x * out, v.n.y * out).normalize();
            rows.push([
                m.push_vertex(at(l, 0.0), nrm(1.0, 0.25), Vec2::new(v.u, 0.0)),
                m.push_vertex(at(0.62 * l, 0.62 * l), nrm(0.7, 0.7), Vec2::new(v.u, 0.5)),
                m.push_vertex(at(0.0, l), nrm(0.25, 1.0), Vec2::new(v.u, 1.0)),
            ]);
        }
        for i in 0..n {
            let (a, b) = (rows[i], rows[(i + 1) % n]);
            for r in 0..2 {
                m.push_triangle_facing(a[r], b[r], b[r + 1]);
                m.push_triangle_facing(a[r], b[r + 1], a[r + 1]);
            }
        }
    }
}

/// The parts of a joint on a member of `profile`, cut `back` metres from
/// the node's centre.
pub fn joint_parts(look: JointLook, profile: &Profile, back: f32) -> Vec<JointPart> {
    let k = dims(profile, back);
    let mut parts: Vec<JointPart> = Vec::new();
    let mut part = |side: Side, look: PartLook, fuse: bool, build: &dyn Fn(&mut MeshData)| {
        let mut mesh = MeshData::default();
        build(&mut mesh);
        parts.push(JointPart {
            side,
            look,
            fuse,
            mesh,
        });
    };
    // Plates stand proud of the section all round.
    let (py, pz) = (k.hy + 0.22 * k.d, k.hz + 0.22 * k.d);
    let plate = |m: &mut MeshData, x0: f32, x1: f32| {
        cuboid(m, Vec3::new(x0, -py, -pz), Vec3::new(x1, py, pz));
    };
    match look {
        JointLook::None => {}
        JointLook::Pinned => {
            // The pin sits a little inside the member end, across z.
            let pin_x = 0.55 * k.d;
            let cheek = k.tp * 1.3;
            let gap = 0.012 * k.d;
            part(Side::Node, PartLook::Plate, false, &|m| {
                neck(m, &k, 0.0);
                // Base, then two cheeks either side of the member.
                cuboid(
                    m,
                    Vec3::new(-k.tp, -0.8 * k.hy, -(k.hz + gap + cheek)),
                    Vec3::new(0.0, 0.8 * k.hy, k.hz + gap + cheek),
                );
                for s in [-1.0f32, 1.0] {
                    let z0 = s * (k.hz + gap);
                    let z1 = s * (k.hz + gap + cheek);
                    cuboid(
                        m,
                        Vec3::new(0.0, -0.62 * k.hy, z0.min(z1)),
                        Vec3::new(pin_x, 0.62 * k.hy, z0.max(z1)),
                    );
                    // Rounded nose of the cheek.
                    prism(
                        m,
                        Vec3::new(pin_x, 0.0, z0.min(z1)),
                        Vec3::new(pin_x, 0.0, z0.max(z1)),
                        0.62 * k.hy,
                        0.62 * k.hy,
                        14,
                        true,
                    );
                }
            });
            part(Side::Node, PartLook::Fastener, true, &|m| {
                let r = 0.2 * k.hy.min(k.hz).max(0.12 * k.d);
                let half = k.hz + gap + cheek;
                prism(
                    m,
                    Vec3::new(pin_x, 0.0, -half - 0.5 * r),
                    Vec3::new(pin_x, 0.0, half + 0.5 * r),
                    r,
                    r,
                    12,
                    true,
                );
                for s in [-1.0f32, 1.0] {
                    prism(
                        m,
                        Vec3::new(pin_x, 0.0, s * (half + 0.5 * r)),
                        Vec3::new(pin_x, 0.0, s * (half + 0.95 * r)),
                        1.5 * r,
                        1.5 * r,
                        12,
                        true,
                    );
                }
            });
        }
        JointLook::Bolted => {
            part(Side::Node, PartLook::Plate, false, &|m| {
                neck(m, &k, -k.tp);
                plate(m, -k.tp, 0.0);
            });
            part(Side::Member, PartLook::Plate, false, &|m| {
                plate(m, 0.0, k.tp)
            });
            part(Side::Member, PartLook::Fastener, true, &|m| {
                let bd = (0.09 * k.d).clamp(0.004, 0.024);
                // A bolt in each corner, and one between on a deep plate.
                let (by, bz) = (py - 1.6 * bd, pz - 1.6 * bd);
                let rows: &[f32] = if py > 1.6 * pz {
                    &[-1.0, 0.0, 1.0]
                } else {
                    &[-1.0, 1.0]
                };
                let cols: &[f32] = if pz > 1.6 * py {
                    &[-1.0, 0.0, 1.0]
                } else {
                    &[-1.0, 1.0]
                };
                for &ry in rows {
                    for &cz in cols {
                        if ry == 0.0 && cz == 0.0 {
                            continue;
                        }
                        bolt(m, Vec3::new(0.0, ry * by, cz * bz), Vec3::X, 2.0 * k.tp, bd);
                    }
                }
            });
        }
        JointLook::Welded => {
            part(Side::Node, PartLook::Plate, false, &|m| {
                neck(m, &k, -k.tp);
                plate(m, -k.tp, 0.0);
            });
            part(Side::Member, PartLook::Weld, true, &|m| {
                weld_bead(m, profile, (0.09 * k.d).clamp(0.004, 0.02));
            });
        }
        JointLook::Monolithic => {
            // A haunch: the section at the cut, flaring towards the node.
            part(Side::Node, PartLook::Concrete, false, &|m| {
                let f = 1.22;
                let ring = |x: f32, s: f32| {
                    [
                        Vec3::new(x, -k.hy * s, -k.hz * s),
                        Vec3::new(x, k.hy * s, -k.hz * s),
                        Vec3::new(x, k.hy * s, k.hz * s),
                        Vec3::new(x, -k.hy * s, k.hz * s),
                    ]
                };
                let (a, b) = (ring(-k.back, f), ring(0.0, 1.0));
                for i in 0..4 {
                    let j = (i + 1) % 4;
                    quad(m, [a[i], a[j], b[j], b[i]]);
                }
                quad(m, [a[3], a[2], a[1], a[0]]);
            });
        }
        JointLook::Dowelled => {
            // A plate in a slot sawn down the middle of the timber, held by
            // dowels through both.
            let reach = 1.5 * k.d;
            part(Side::Node, PartLook::Plate, false, &|m| {
                neck(m, &k, 0.0);
                cuboid(
                    m,
                    Vec3::new(-0.02 * k.d, -0.86 * k.hy, -0.5 * k.tp),
                    Vec3::new(reach, 0.86 * k.hy, 0.5 * k.tp),
                );
                cuboid(
                    m,
                    Vec3::new(-k.tp, -k.hy, -k.hz),
                    Vec3::new(0.0, k.hy, k.hz),
                );
            });
            part(Side::Member, PartLook::Fastener, true, &|m| {
                let r = (0.045 * k.d).clamp(0.003, 0.012);
                let proud = 0.25 * r;
                for cx in [0.45f32, 0.85, 1.25] {
                    for ry in [-0.5f32, 0.0, 0.5] {
                        // Staggered, as dowel groups are.
                        let x = (cx + 0.1 * ry) * k.d;
                        prism(
                            m,
                            Vec3::new(x, ry * k.hy * 1.1, -k.hz - proud),
                            Vec3::new(x, ry * k.hy * 1.1, k.hz + proud),
                            r,
                            r,
                            8,
                            true,
                        );
                    }
                }
            });
        }
        JointLook::Sleeve => {
            let wall = k.tp * 0.8;
            let len = 0.9 * k.d;
            let round = matches!(profile, Profile::Round { .. } | Profile::Tube { .. });
            part(Side::Node, PartLook::Plate, false, &|m| {
                neck(m, &k, -k.tp);
                if round {
                    // A collar with a flange at its root.
                    let x = |v: f32| Vec3::new(v, 0.0, 0.0);
                    prism(
                        m,
                        x(-k.tp),
                        x(0.0),
                        k.hy + 3.0 * wall,
                        k.hy + 3.0 * wall,
                        20,
                        true,
                    );
                    prism(m, x(0.0), x(len), k.hy + wall, k.hy + wall, 20, true);
                    return;
                }
                cuboid(
                    m,
                    Vec3::new(-k.tp, -k.hy - wall, -k.hz - wall),
                    Vec3::new(0.0, k.hy + wall, k.hz + wall),
                );
                // Four walls of the shoe round the member end.
                for (lo, hi) in [
                    (
                        Vec3::new(0.0, k.hy, -k.hz - wall),
                        Vec3::new(len, k.hy + wall, k.hz + wall),
                    ),
                    (
                        Vec3::new(0.0, -k.hy - wall, -k.hz - wall),
                        Vec3::new(len, -k.hy, k.hz + wall),
                    ),
                    (
                        Vec3::new(0.0, -k.hy, k.hz),
                        Vec3::new(len, k.hy, k.hz + wall),
                    ),
                    (
                        Vec3::new(0.0, -k.hy, -k.hz - wall),
                        Vec3::new(len, k.hy, -k.hz),
                    ),
                ] {
                    cuboid(m, lo, hi);
                }
            });
            part(Side::Node, PartLook::Fastener, true, &|m| {
                let bd = (0.07 * k.d).clamp(0.003, 0.016);
                for cx in [0.3f32, 0.65] {
                    bolt(
                        m,
                        Vec3::new(cx * k.d, 0.0, 0.0),
                        Vec3::Z,
                        2.0 * (k.hz + wall),
                        bd,
                    );
                }
            });
        }
    }
    parts
}

/// The node itself: a faceted cast-steel ball, or a concrete block where
/// concrete members meet. Unit size; scale by the node's radius.
pub fn node_hub(concrete: bool) -> MeshData {
    let mut m = MeshData::default();
    if concrete {
        cuboid(&mut m, Vec3::splat(-0.85), Vec3::splat(0.85));
        return m;
    }
    let (rings, sectors) = (8, 14);
    let mut grid = Vec::new();
    for i in 0..=rings {
        let phi = i as f32 / rings as f32 * PI;
        for j in 0..=sectors {
            let th = j as f32 / sectors as f32 * TAU;
            let n = Vec3::new(phi.sin() * th.cos(), phi.cos(), phi.sin() * th.sin());
            grid.push(m.push_vertex(
                n * 0.86,
                n,
                Vec2::new(j as f32 / sectors as f32, i as f32 / rings as f32),
            ));
        }
    }
    let at = |i: usize, j: usize| grid[i * (sectors + 1) + j];
    for i in 0..rings {
        for j in 0..sectors {
            if i > 0 {
                m.push_triangle_facing(at(i, j), at(i, j + 1), at(i + 1, j + 1));
            }
            if i + 1 < rings {
                m.push_triangle_facing(at(i, j), at(i + 1, j + 1), at(i + 1, j));
            }
        }
    }
    m
}

// ------------------------------------------------------------- placement

/// Where a joint's two parts are this frame.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct JointPose {
    /// The part carried by the node.
    pub node: (Vec3, Quat),
    /// The part carried by the member end.
    pub member: (Vec3, Quat),
}

/// Turns a member frame at end b round so +x points back into the member.
fn into_member(q: Quat, end: usize) -> Quat {
    if end == 0 {
        q
    } else {
        q * Quat::from_rotation_y(PI)
    }
}

/// Pose of the node-side part of a joint: `p` the node, `q` the member's
/// end frame as the node carries it (node rotation times rest frame).
pub fn node_part_pose(p: Vec3, q: Quat, end: usize, back: f32) -> (Vec3, Quat) {
    let qn = into_member(q, end);
    (p + qn * Vec3::X * back, qn)
}

/// Pose of the member-side part: on the member's own curve where it is cut,
/// so the part sits exactly on the drawn member end.
pub fn member_part_pose(pose: &MemberPose, end: usize, back: f32) -> (Vec3, Quat) {
    let s = back / pose.rest_length.max(1e-6);
    let s = if end == 0 { s } else { 1.0 - s };
    let (c, t) = pose.centreline(s);
    (c, into_member(pose.frame(s, t), end))
}

/// Pose of the joint at `end` (0 = a, 1 = b) of a member in `pose`, cut
/// `back` metres from the node. The node part follows the node's frame; the
/// member part follows the member's own curve, so any rotation the joint has
/// taken shows between the two.
pub fn joint_pose(pose: &MemberPose, end: usize, back: f32) -> JointPose {
    let (p, q) = if end == 0 {
        (pose.pa, pose.qa)
    } else {
        (pose.pb, pose.qb)
    };
    JointPose {
        node: node_part_pose(p, q, end, back),
        member: member_part_pose(pose, end, back),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const LOOKS: [JointLook; 6] = [
        JointLook::Pinned,
        JointLook::Bolted,
        JointLook::Welded,
        JointLook::Monolithic,
        JointLook::Dowelled,
        JointLook::Sleeve,
    ];

    fn profiles() -> Vec<Profile> {
        vec![
            Profile::Rect { b: 0.1, h: 0.2 },
            Profile::Round { r: 0.02 },
            Profile::Tube { r: 0.05, t: 0.005 },
            Profile::BoxSection {
                b: 0.1,
                h: 0.1,
                t: 0.006,
            },
            Profile::IBeam {
                h: 0.2,
                b: 0.1,
                tw: 0.0056,
                tf: 0.0085,
            },
        ]
    }

    #[test]
    fn every_joint_builds_on_every_profile() {
        for look in LOOKS {
            for p in profiles() {
                let back = 1.2 * p.bounding_radius();
                let parts = joint_parts(look, &p, back);
                assert!(!parts.is_empty(), "{look:?} on {p:?}");
                for part in &parts {
                    let m = &part.mesh;
                    assert!(m.triangle_count() > 0);
                    assert_eq!(m.positions.len(), m.normals.len());
                    assert!(m.indices.iter().all(|&i| (i as usize) < m.positions.len()));
                    for n in &m.normals {
                        let l = Vec3::from_array(*n).length();
                        assert!((l - 1.0).abs() < 1e-3, "{look:?} {p:?}: normal {l}");
                    }
                    // Nothing reaches behind the node's centre or far up
                    // the member.
                    let (lo, hi) = m.bounds();
                    assert!(lo.x >= -back - 1e-4, "{look:?} {p:?}: {lo}");
                    assert!(hi.x <= 4.0 * p.bounding_radius(), "{look:?} {p:?}: {hi}");
                }
            }
        }
        assert!(joint_parts(JointLook::None, &profiles()[0], 0.1).is_empty());
    }

    #[test]
    fn every_real_joint_has_something_to_lose() {
        for look in LOOKS {
            let parts = joint_parts(look, &profiles()[0], 0.15);
            let fuses = parts.iter().filter(|p| p.fuse).count();
            if look == JointLook::Monolithic {
                assert_eq!(fuses, 0);
            } else {
                assert_eq!(fuses, 1, "{look:?}");
            }
        }
    }

    #[test]
    fn weld_follows_the_section() {
        // The bead of an I-beam is as long as its outline, so it has far
        // more triangles than a tube's and stays within a leg of the steel.
        let i = profiles()[4];
        let parts = joint_parts(JointLook::Welded, &i, 0.15);
        let weld = parts.iter().find(|p| p.look == PartLook::Weld).unwrap();
        let (lo, hi) = weld.mesh.bounds();
        let leg = 0.09 * 0.2 * 1.2;
        assert!(hi.y <= 0.1 + leg && lo.y >= -0.1 - leg);
        assert!(hi.z <= 0.05 + leg && lo.z >= -0.05 - leg);
        assert!(lo.x >= -1e-6 && hi.x <= leg);
    }

    #[test]
    fn rigid_joints_look_like_their_material() {
        use MaterialKind as M;
        assert_eq!(JointLook::of(JointKind::Rigid, M::Steel), JointLook::Welded);
        assert_eq!(
            JointLook::of(JointKind::Rigid, M::ReinforcedConcrete),
            JointLook::Monolithic
        );
        assert_eq!(
            JointLook::of(JointKind::Rigid, M::Timber),
            JointLook::Sleeve
        );
        assert_eq!(
            JointLook::of(JointKind::Pinned, M::Timber),
            JointLook::Pinned
        );
    }

    #[test]
    fn both_ends_point_into_the_member() {
        let pose = MemberPose::straight(Vec3::new(1.0, 2.0, 0.0), Vec3::new(4.0, 2.0, 0.0));
        let a = joint_pose(&pose, 0, 0.2);
        let b = joint_pose(&pose, 1, 0.2);
        assert!((a.node.0 - Vec3::new(1.2, 2.0, 0.0)).length() < 1e-5);
        assert!((b.node.0 - Vec3::new(3.8, 2.0, 0.0)).length() < 1e-5);
        assert!((a.node.1 * Vec3::X - Vec3::X).length() < 1e-5);
        assert!((b.node.1 * Vec3::X + Vec3::X).length() < 1e-5);
        // The section's up stays up at both ends.
        assert!((a.node.1 * Vec3::Y - Vec3::Y).length() < 1e-5);
        assert!((b.node.1 * Vec3::Y - Vec3::Y).length() < 1e-5);
        // Straight member: the two parts agree.
        assert!(a.node.1.angle_between(a.member.1) < 1e-4);
    }

    #[test]
    fn a_turned_member_end_opens_the_joint() {
        let mut pose = MemberPose::straight(Vec3::ZERO, Vec3::new(3.0, 0.0, 0.0));
        // The member leaves end a 0.1 rad below the node's frame.
        pose.ta = Vec3::new(0.1f32.cos(), -0.1f32.sin(), 0.0);
        let j = joint_pose(&pose, 0, 0.2);
        // Close to the full 0.1 rad: the cut is a little way along the curve.
        let open = j.node.1.angle_between(j.member.1);
        assert!(open > 0.07 && open < 0.1 + 1e-4, "{open}");
        // The member part sits on the member's curve.
        let (c, _) = pose.centreline(0.2 / 3.0);
        assert!((j.member.0 - c).length() < 1e-5);
    }
}
