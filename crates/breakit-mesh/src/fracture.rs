//! Turning a broken member into visual-only debris.
//!
//! Pieces are generated procedurally at the moment of failure from the
//! member's *current* deformed pose, so a beam that bent before it broke
//! stays bent as debris. Each material family has its own fracture look:
//!
//! * steel: ductile tear, necked and slightly jagged ends, sparks
//! * timber: long splinters along the grain, splinter particles
//! * concrete: rough faces, loose chunks, dust; reinforced concrete also gets
//!   exposed, bent rebar stubs
//! * composite: delaminated ends whose plies splay apart (brooming), fibre dust
//!
//! Generating the pieces of one break costs tens of microseconds (see
//! `examples/bench.rs`), so nothing is pre-cut.

use glam::{Quat, Vec2, Vec3};

use crate::mesh::MeshData;
use crate::pose::MemberPose;
use crate::profile::{OutlineVertex, Profile};
use crate::state::{hash01, MaterialKind};

/// A rigid debris piece. `mesh` is in piece space (origin at the centroid);
/// the piece starts at `position` with identity rotation.
#[derive(Clone, Debug)]
pub struct DebrisPiece {
    pub mesh: MeshData,
    pub position: Vec3,
    pub rotation: Quat,
    pub velocity: Vec3,
    pub angular_velocity: Vec3,
    pub kind: MaterialKind,
    /// Which part this is, for picking a material (rebar is drawn as steel).
    pub part: PiecePart,
    /// Local points used for ground contact (bounding-box corners).
    pub contact_points: Vec<Vec3>,
    pub radius: f32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PiecePart {
    Body,
    Chunk,
    Rebar,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BurstKind {
    Sparks,
    Dust,
    Splinters,
    Fibres,
}

/// A particle emission at the fracture.
#[derive(Clone, Copy, Debug)]
pub struct ParticleBurst {
    pub kind: BurstKind,
    pub origin: Vec3,
    /// Member tangent at the break: particles spray around it.
    pub axis: Vec3,
    pub count: u32,
    /// Size of the section, scales the spread.
    pub scale: f32,
}

#[derive(Clone, Debug, Default)]
pub struct Fracture {
    pub pieces: Vec<DebrisPiece>,
    pub bursts: Vec<ParticleBurst>,
}

/// Inputs for one break.
#[derive(Clone, Copy, Debug)]
pub struct BreakInput<'a> {
    pub profile: &'a Profile,
    pub kind: MaterialKind,
    pub pose: &'a MemberPose,
    /// Failure position along the member, 0..1.
    pub position: f32,
    /// Velocities of the end nodes at failure.
    pub velocity_a: Vec3,
    pub velocity_b: Vec3,
    pub seed: u32,
}

/// Small deterministic RNG.
#[derive(Clone, Copy, Debug)]
pub struct Rng(u32);
impl Rng {
    pub fn new(seed: u32) -> Self {
        Self(seed.wrapping_mul(0x9e37_79b9) ^ 0x85eb_ca6b)
    }
    pub fn next(&mut self) -> f32 {
        self.0 = self.0.wrapping_add(0x6d2b_79f5);
        hash01(self.0)
    }
    pub fn range(&mut self, lo: f32, hi: f32) -> f32 {
        lo + (hi - lo) * self.next()
    }
    pub fn signed(&mut self) -> f32 {
        self.range(-1.0, 1.0)
    }
    pub fn unit_vec(&mut self) -> Vec3 {
        loop {
            let v = Vec3::new(self.signed(), self.signed(), self.signed());
            let l = v.length_squared();
            if l > 1e-4 && l <= 1.0 {
                return v / l.sqrt();
            }
        }
    }
}

/// Smooth value noise on the section plane, -1..1.
fn section_noise(p: Vec2, freq: f32, seed: f32) -> f32 {
    let q = p * freq + Vec2::splat(seed * 17.0);
    let (i, f) = (q.floor(), q.fract());
    let h = |x: f32, y: f32| {
        let k =
            (x as i32 as u32).wrapping_mul(73_856_093) ^ (y as i32 as u32).wrapping_mul(19_349_663);
        2.0 * hash01(k) - 1.0
    };
    let u = f * f * (Vec2::splat(3.0) - 2.0 * f);
    let a = h(i.x, i.y);
    let b = h(i.x + 1.0, i.y);
    let c = h(i.x, i.y + 1.0);
    let d = h(i.x + 1.0, i.y + 1.0);
    a + (b - a) * u.x + (c - a) * u.y + (a - b - c + d) * u.x * u.y
}

/// Extrude a cross-section between two cut surfaces, bend it with `pose`.
///
/// `cut0`/`cut1` give the member parameter `s` of each end face at a section
/// point; `warp` moves member-space points before bending (necking, splay).
fn extrude(
    loops: &[Vec<OutlineVertex>],
    cap: &crate::profile::Cap,
    rings: usize,
    cut0: &dyn Fn(Vec2) -> f32,
    cut1: &dyn Fn(Vec2) -> f32,
    warp: &dyn Fn(Vec3) -> Vec3,
    pose: &MemberPose,
    caps: [bool; 2],
) -> MeshData {
    let mut m = MeshData::default();
    for lp in loops {
        let n = lp.len();
        let base = m.positions.len() as u32;
        for r in 0..rings {
            let t = r as f32 / (rings - 1) as f32;
            for v in lp {
                let s = cut0(v.p) + (cut1(v.p) - cut0(v.p)) * t;
                let local = warp(Vec3::new(s, v.p.x, v.p.y));
                let (p, nn) = pose.deform(local, Vec3::new(0.0, v.n.x, v.n.y));
                m.push_vertex(p, nn, Vec2::new(s, v.u));
            }
        }
        for r in 0..rings - 1 {
            let quad = |m: &mut MeshData, k0: usize, k1: usize| {
                let i0 = base + (r * n + k0) as u32;
                let i1 = base + (r * n + k1) as u32;
                let (i2, i3) = (i0 + n as u32, i1 + n as u32);
                m.push_triangle_facing(i0, i2, i1);
                m.push_triangle_facing(i1, i2, i3);
            };
            for k in 0..n - 1 {
                if (lp[k].p - lp[k + 1].p).length_squared() > 1e-14 {
                    quad(&mut m, k, k + 1);
                }
            }
            if (lp[n - 1].p - lp[0].p).length_squared() > 1e-14 {
                quad(&mut m, n - 1, 0);
            }
        }
    }
    for (end, cut) in [(0usize, cut0), (1usize, cut1)] {
        if !caps[end] {
            continue;
        }
        let base = m.positions.len() as u32;
        // Cap normal: along the curve, outward. Rough faces read fine with it.
        let s_mid = cut(Vec2::ZERO);
        let (_, t) = pose.centreline(s_mid);
        let nrm = if end == 0 { -t } else { t };
        for p in &cap.points {
            let local = warp(Vec3::new(cut(*p), p.x, p.y));
            let (wp, _) = pose.deform(local, Vec3::X);
            m.push_vertex(wp, nrm, Vec2::new(cut(*p), 0.0));
        }
        for tri in &cap.triangles {
            m.push_triangle_facing(base + tri[0], base + tri[1], base + tri[2]);
        }
    }
    m
}

/// Lumpy convex-ish rock, flat shaded, radius ~`r`.
fn chunk_mesh(r: Vec3, rng: &mut Rng) -> MeshData {
    let (nu, nv) = (6usize, 4usize);
    let mut pts = Vec::new();
    for j in 0..=nv {
        for i in 0..nu {
            let th = std::f32::consts::PI * j as f32 / nv as f32;
            let ph = std::f32::consts::TAU * (i as f32 + 0.5 * (j % 2) as f32) / nu as f32;
            let k = if j == 0 || j == nv {
                1.0
            } else {
                rng.range(0.7, 1.15)
            };
            pts.push(Vec3::new(th.sin() * ph.cos(), th.cos(), th.sin() * ph.sin()) * r * k);
        }
    }
    let mut m = MeshData::default();
    let idx = |i: usize, j: usize| j * nu + (i % nu);
    for j in 0..nv {
        for i in 0..nu {
            let (a, b, c, d) = (idx(i, j), idx(i + 1, j), idx(i, j + 1), idx(i + 1, j + 1));
            for tri in [[a, c, b], [b, c, d]] {
                let (p0, p1, p2) = (pts[tri[0]], pts[tri[1]], pts[tri[2]]);
                let n = (p1 - p0).cross(p2 - p0);
                if n.length_squared() < 1e-16 {
                    continue; // collapsed pole triangle
                }
                // Flat normal pointing away from the centre.
                let c0 = (p0 + p1 + p2) / 3.0;
                let n = if n.dot(c0) < 0.0 { -n } else { n }.normalize();
                let base = m.positions.len() as u32;
                for p in [p0, p1, p2] {
                    m.push_vertex(p, n, Vec2::ZERO);
                }
                m.push_triangle_facing(base, base + 1, base + 2);
            }
        }
    }
    m
}

fn finish_piece(
    mut mesh: MeshData,
    kind: MaterialKind,
    part: PiecePart,
    velocity: Vec3,
    angular_velocity: Vec3,
) -> DebrisPiece {
    let n = mesh.positions.len().max(1) as f32;
    let c = mesh
        .positions
        .iter()
        .fold(Vec3::ZERO, |acc, p| acc + Vec3::from_array(*p))
        / n;
    for p in &mut mesh.positions {
        *p = (Vec3::from_array(*p) - c).to_array();
    }
    let (lo, hi) = mesh.bounds();
    let contact_points = (0..8)
        .map(|i| {
            Vec3::new(
                if i & 1 == 0 { lo.x } else { hi.x },
                if i & 2 == 0 { lo.y } else { hi.y },
                if i & 4 == 0 { lo.z } else { hi.z },
            )
        })
        .collect();
    let radius = lo.abs().max(hi.abs()).length();
    DebrisPiece {
        mesh,
        position: c,
        rotation: Quat::IDENTITY,
        velocity,
        angular_velocity,
        kind,
        part,
        contact_points,
        radius,
    }
}

/// Break a member into debris and particle bursts.
pub fn fracture(input: &BreakInput) -> Fracture {
    let BreakInput {
        profile,
        kind,
        pose,
        position,
        velocity_a,
        velocity_b,
        seed,
    } = *input;
    let mut rng = Rng::new(seed);
    let l = pose.rest_length.max(1e-3);
    let depth = 2.0 * profile.half_depth();
    let width = 2.0 * profile.half_width();
    let size = depth.max(width);
    let sb = position.clamp(0.02, 0.98);
    let fseed = rng.next();
    let loops = profile.outline();
    let cap = profile.cap();
    let none = |p: Vec3| p;
    let (break_point, break_tangent) = pose.centreline(sb);
    let vel_at = |s: f32| velocity_a.lerp(velocity_b, s);
    let rings_for = |len_s: f32| ((len_s * l / (0.5 * size)).ceil() as usize).clamp(2, 24);

    let mut out = Fracture::default();
    // Separation kick: pieces spring apart a little along the member.
    let kick = |rng: &mut Rng, dir: f32, mag: f32| {
        break_tangent * dir * mag + rng.unit_vec() * (0.3 * mag)
    };
    let spin = |rng: &mut Rng, mag: f32| rng.unit_vec() * mag;

    match kind {
        MaterialKind::Steel => {
            // Ductile tear: slightly tilted, slightly jagged faces, necked ends.
            let tilt = Vec2::new(rng.signed(), rng.signed()) * (0.25 * size / l);
            let amp = 0.08 * size / l;
            let cut =
                move |p: Vec2| sb + tilt.dot(p / size) + amp * section_noise(p, 9.0 / size, fseed);
            let neck_len = 0.6 * size / l;
            let neck = move |p: Vec3| {
                let d = (p.x - sb).abs();
                let k = 1.0 - 0.3 * (1.0 - (d / neck_len).min(1.0)).powi(2);
                Vec3::new(p.x, p.y * k, p.z * k)
            };
            let start = |_: Vec2| 0.0;
            let end = |_: Vec2| 1.0;
            let a = extrude(
                &loops,
                &cap,
                rings_for(sb) + 3,
                &start,
                &cut,
                &neck,
                pose,
                [true, true],
            );
            let b = extrude(
                &loops,
                &cap,
                rings_for(1.0 - sb) + 3,
                &cut,
                &end,
                &neck,
                pose,
                [true, true],
            );
            out.pieces.push(finish_piece(
                a,
                kind,
                PiecePart::Body,
                vel_at(sb * 0.5) + kick(&mut rng, -1.0, 0.4),
                spin(&mut rng, 0.5),
            ));
            out.pieces.push(finish_piece(
                b,
                kind,
                PiecePart::Body,
                vel_at(0.5 + sb * 0.5) + kick(&mut rng, 1.0, 0.4),
                spin(&mut rng, 0.5),
            ));
            out.bursts.push(ParticleBurst {
                kind: BurstKind::Sparks,
                origin: break_point,
                axis: break_tangent,
                count: 48,
                scale: size,
            });
        }
        MaterialKind::Timber => {
            // Brittle bending break: a long, jagged splintered face.
            let amp = 1.2 * size / l;
            let cut = move |p: Vec2| sb + amp * section_noise(p, 14.0 / size, fseed) * 0.5;
            let start = |_: Vec2| 0.0;
            let end = |_: Vec2| 1.0;
            let mut a = extrude(
                &loops,
                &cap,
                rings_for(sb) + 1,
                &start,
                &cut,
                &none,
                pose,
                [true, true],
            );
            let mut b = extrude(
                &loops,
                &cap,
                rings_for(1.0 - sb) + 1,
                &cut,
                &end,
                &none,
                pose,
                [true, true],
            );
            // Splinters: thin slivers sticking out of both faces.
            for (piece, dir) in [(&mut a, 1.0f32), (&mut b, -1.0f32)] {
                let n_spl = 7 + (rng.next() * 5.0) as usize;
                for _ in 0..n_spl {
                    let p = Vec2::new(
                        rng.signed() * profile.half_depth(),
                        rng.signed() * profile.half_width(),
                    ) * 0.85;
                    let len = rng.range(0.4, 1.6) * size / l;
                    let th = rng.range(0.08, 0.2) * size;
                    let s0 = cut(p) - dir * 0.2 * len;
                    let s1 = cut(p) + dir * len;
                    let sliver = Profile::Rect {
                        b: th * rng.range(0.5, 1.2),
                        h: th,
                    };
                    let flare = rng.signed() * 0.15 * size;
                    let warp = move |q: Vec3| {
                        // Taper to a point and bend outward towards the tip.
                        let t = ((q.x - s0) / (s1 - s0)).clamp(0.0, 1.0);
                        let k = 1.0 - 0.85 * t;
                        Vec3::new(q.x, p.x + q.y * k + flare * t * t, p.y + q.z * k)
                    };
                    let (c0, c1) = if dir > 0.0 { (s0, s1) } else { (s1, s0) };
                    let m = extrude(
                        &sliver.outline(),
                        &sliver.cap(),
                        4,
                        &move |_| c0,
                        &move |_| c1,
                        &warp,
                        pose,
                        [false, true],
                    );
                    piece.append(&m);
                }
            }
            out.pieces.push(finish_piece(
                a,
                kind,
                PiecePart::Body,
                vel_at(sb * 0.5) + kick(&mut rng, -1.0, 0.8),
                spin(&mut rng, 1.0),
            ));
            out.pieces.push(finish_piece(
                b,
                kind,
                PiecePart::Body,
                vel_at(0.5 + sb * 0.5) + kick(&mut rng, 1.0, 0.8),
                spin(&mut rng, 1.0),
            ));
            out.bursts.push(ParticleBurst {
                kind: BurstKind::Splinters,
                origin: break_point,
                axis: break_tangent,
                count: 40,
                scale: size,
            });
            out.bursts.push(ParticleBurst {
                kind: BurstKind::Dust,
                origin: break_point,
                axis: break_tangent,
                count: 12,
                scale: size,
            });
        }
        MaterialKind::Concrete | MaterialKind::ReinforcedConcrete => {
            // A crushed zone of loose chunks between two rough faces.
            let half_zone = (0.45 * size / l).min(0.2);
            let (za, zb) = ((sb - half_zone).max(0.01), (sb + half_zone).min(0.99));
            let amp = 0.35 * size / l;
            let cut_a = move |p: Vec2| za + amp * section_noise(p, 6.0 / size, fseed);
            let cut_b = move |p: Vec2| zb + amp * section_noise(p, 6.0 / size, fseed + 0.37);
            let start = |_: Vec2| 0.0;
            let end = |_: Vec2| 1.0;
            let mut a = extrude(
                &loops,
                &cap,
                rings_for(za) + 1,
                &start,
                &cut_a,
                &none,
                pose,
                [true, true],
            );
            let mut b = extrude(
                &loops,
                &cap,
                rings_for(1.0 - zb) + 1,
                &cut_b,
                &end,
                &none,
                pose,
                [true, true],
            );
            if kind == MaterialKind::ReinforcedConcrete {
                // Four corner bars, 12% cover, torn in the zone and bent out.
                let (hy, hz) = (profile.half_depth() * 0.76, profile.half_width() * 0.76);
                let bar = Profile::Round { r: 0.045 * size };
                for (cy, cz) in [(-hy, -hz), (-hy, hz), (hy, hz), (hy, -hz)] {
                    let c = Vec2::new(cy, cz);
                    let tear = za + (zb - za) * rng.range(0.3, 0.7);
                    for (from, to, piece_a) in [
                        (za - 0.3 * half_zone, tear, true),
                        (zb + 0.3 * half_zone, tear, false),
                    ] {
                        let bend =
                            Vec2::new(cy.signum(), cz.signum()) * rng.range(0.1, 0.35) * size;
                        let warp = move |q: Vec3| {
                            let t = ((q.x - from) / (to - from)).clamp(0.0, 1.0);
                            Vec3::new(q.x, c.x + q.y + bend.x * t * t, c.y + q.z + bend.y * t * t)
                        };
                        let (c0, c1) = if from < to { (from, to) } else { (to, from) };
                        let m = extrude(
                            &bar.outline(),
                            &bar.cap(),
                            6,
                            &move |_| c0,
                            &move |_| c1,
                            &warp,
                            pose,
                            [true, true],
                        );
                        if piece_a {
                            a.append(&m)
                        } else {
                            b.append(&m)
                        }
                    }
                }
            }
            out.pieces.push(finish_piece(
                a,
                kind,
                PiecePart::Body,
                vel_at(za * 0.5) + kick(&mut rng, -1.0, 0.3),
                spin(&mut rng, 0.4),
            ));
            out.pieces.push(finish_piece(
                b,
                kind,
                PiecePart::Body,
                vel_at(0.5 + zb * 0.5) + kick(&mut rng, 1.0, 0.3),
                spin(&mut rng, 0.4),
            ));
            let n_chunks = 4 + (rng.next() * 4.0) as usize;
            for _ in 0..n_chunks {
                let s = rng.range(za, zb);
                let off = Vec2::new(
                    rng.signed() * profile.half_depth(),
                    rng.signed() * profile.half_width(),
                ) * 0.6;
                let (p, _) = pose.deform(Vec3::new(s, off.x, off.y), Vec3::X);
                let r = Vec3::new(
                    rng.range(0.15, 0.32),
                    rng.range(0.12, 0.28),
                    rng.range(0.15, 0.3),
                ) * size;
                let mut m = chunk_mesh(r, &mut rng);
                for q in &mut m.positions {
                    *q = (Vec3::from_array(*q) + p).to_array();
                }
                let v = vel_at(s) + rng.unit_vec() * rng.range(0.5, 2.0) + Vec3::Y * 0.5;
                out.pieces.push(finish_piece(
                    m,
                    kind,
                    PiecePart::Chunk,
                    v,
                    spin(&mut rng, 4.0),
                ));
            }
            out.bursts.push(ParticleBurst {
                kind: BurstKind::Dust,
                origin: break_point,
                axis: break_tangent,
                count: 60,
                scale: size,
            });
        }
        MaterialKind::Composite => {
            // Clean-ish fibre fracture with delaminated, splayed plies.
            let plies = 4usize;
            let zone = (1.4 * size / l).min(0.3);
            let (za, zb) = ((sb - zone).max(0.01), (sb + zone).min(0.99));
            let start = |_: Vec2| 0.0;
            let end = |_: Vec2| 1.0;
            let mut a = extrude(
                &loops,
                &cap,
                rings_for(za) + 1,
                &start,
                &move |_| za,
                &none,
                pose,
                [true, false],
            );
            let mut b = extrude(
                &loops,
                &cap,
                rings_for(1.0 - zb) + 1,
                &move |_| zb,
                &end,
                &none,
                pose,
                [false, true],
            );
            let (hd, hw) = (profile.half_depth(), profile.half_width());
            let ply_h = 2.0 * hd / plies as f32;
            for j in 0..plies {
                let yc = -hd + ply_h * (j as f32 + 0.5);
                let ply = Profile::Rect {
                    b: 2.0 * hw,
                    h: ply_h * 0.8,
                };
                let side = if yc >= 0.0 { 1.0 } else { -1.0 };
                let spread = (0.15 + 0.25 * (yc.abs() / hd)) * size * rng.range(0.7, 1.3);
                for (from, to, piece_a) in [
                    (za, sb + rng.signed() * 0.3 * zone, true),
                    (zb, sb + rng.signed() * 0.3 * zone, false),
                ] {
                    let warp = move |q: Vec3| {
                        let t = ((q.x - from) / (to - from)).clamp(0.0, 1.0);
                        Vec3::new(q.x, yc + q.y + side * spread * t * t, q.z)
                    };
                    let (c0, c1) = if from < to { (from, to) } else { (to, from) };
                    let m = extrude(
                        &ply.outline(),
                        &ply.cap(),
                        6,
                        &move |_| c0,
                        &move |_| c1,
                        &warp,
                        pose,
                        [true, true],
                    );
                    if piece_a {
                        a.append(&m)
                    } else {
                        b.append(&m)
                    }
                }
            }
            out.pieces.push(finish_piece(
                a,
                kind,
                PiecePart::Body,
                vel_at(za * 0.5) + kick(&mut rng, -1.0, 1.0),
                spin(&mut rng, 1.0),
            ));
            out.pieces.push(finish_piece(
                b,
                kind,
                PiecePart::Body,
                vel_at(0.5 + zb * 0.5) + kick(&mut rng, 1.0, 1.0),
                spin(&mut rng, 1.0),
            ));
            out.bursts.push(ParticleBurst {
                kind: BurstKind::Fibres,
                origin: break_point,
                axis: break_tangent,
                count: 50,
                scale: size,
            });
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::pose::MemberPose;

    fn input<'a>(profile: &'a Profile, pose: &'a MemberPose, kind: MaterialKind) -> BreakInput<'a> {
        BreakInput {
            profile,
            kind,
            pose,
            position: 0.5,
            velocity_a: Vec3::ZERO,
            velocity_b: Vec3::ZERO,
            seed: 7,
        }
    }

    #[test]
    fn every_material_breaks_into_pieces_near_the_member() {
        let pose = MemberPose::straight(Vec3::new(0.0, 3.0, 0.0), Vec3::new(4.0, 3.0, 0.0));
        let profile = Profile::Rect { b: 0.2, h: 0.3 };
        for kind in MaterialKind::ALL {
            let f = fracture(&input(&profile, &pose, kind));
            assert!(f.pieces.len() >= 2, "{kind:?}");
            assert!(!f.bursts.is_empty(), "{kind:?}");
            for p in &f.pieces {
                assert!(p.mesh.triangle_count() > 0);
                assert!(p.position.is_finite());
                // Pieces start where the member was.
                assert!(
                    p.position.y > 2.5 && p.position.y < 3.5,
                    "{kind:?} {:?}",
                    p.position
                );
                assert!(p.position.x > -0.5 && p.position.x < 4.5);
                assert!(p
                    .mesh
                    .positions
                    .iter()
                    .all(|q| q.iter().all(|c| c.is_finite())));
            }
        }
    }

    #[test]
    fn steel_halves_conserve_volume_roughly() {
        let pose = MemberPose::straight(Vec3::ZERO, Vec3::new(3.0, 0.0, 0.0));
        let profile = Profile::Rect { b: 0.1, h: 0.2 };
        let f = fracture(&input(&profile, &pose, MaterialKind::Steel));
        let v: f32 = f.pieces.iter().map(|p| p.mesh.volume()).sum();
        let v0 = 3.0 * 0.1 * 0.2;
        // Necking removes a little volume; caps and sides must be consistent.
        assert!(v > 0.9 * v0 && v < 1.01 * v0, "volume {v} vs {v0}");
    }

    #[test]
    fn rebar_only_in_reinforced_concrete() {
        let pose = MemberPose::straight(Vec3::ZERO, Vec3::new(3.0, 0.0, 0.0));
        let profile = Profile::Rect { b: 0.3, h: 0.3 };
        let plain = fracture(&input(&profile, &pose, MaterialKind::Concrete));
        let rc = fracture(&input(&profile, &pose, MaterialKind::ReinforcedConcrete));
        let tris = |f: &Fracture| {
            f.pieces
                .iter()
                .filter(|p| p.part == PiecePart::Body)
                .map(|p| p.mesh.triangle_count())
                .sum::<usize>()
        };
        assert!(tris(&rc) > tris(&plain) + 100);
    }
}
