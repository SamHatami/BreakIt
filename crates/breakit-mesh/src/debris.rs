//! Visual-only debris: cheap rigid bodies that fall, tumble, hit the ground
//! and come to rest. They never feed back into the structural solver.
//!
//! Each piece is a unit-mass box: the bounding box of its contact points
//! (the 8 local bounding-box corners from fracture) gives both the contact
//! shape and a diagonal inertia tensor in the piece's frame. Ground contact
//! is a plane tested at those corners. Per step:
//!
//! 1. integrate gravity, damping and the pose (semi-implicit Euler);
//! 2. collect every corner that ended up under the ground;
//! 3. solve all of them together with sequential impulses: several
//!    iterations over accumulated, clamped normal impulses (non-negative)
//!    and 2D friction impulses clamped to the Coulomb disc `mu * pn`;
//!    restitution only for impacts faster than `bounce_threshold`; resting
//!    corners are warm-started from the previous step's impulses;
//! 4. push the piece out by the deepest penetration (position only, so the
//!    velocity solve has no Baumgarte bias that would make it hop);
//! 5. sleep once the piece is touching, slow, and has not drifted from where
//!    it started resting for `sleep_after` seconds.
//!
//! No extra damping is applied on the ground, so landing, sliding and tipping
//! run at their real speed. Costs well under a microsecond per awake piece.
//! Swap in Avian/Rapier later if debris should pile up on itself or on the
//! structure.

use glam::{Mat3, Quat, Vec2, Vec3};

use crate::fracture::{DebrisPiece, PiecePart};
use crate::state::MaterialKind;

/// Corners beyond this many are ignored by the contact solver (fracture
/// always gives 8).
const MAX_CONTACTS: usize = 8;

#[derive(Clone, Copy, Debug)]
pub struct DebrisConfig {
    pub gravity: Vec3,
    pub ground_y: f32,
    pub restitution: f32,
    pub friction: f32,
    /// Linear and angular damping, 1/s (air drag; not applied extra on the
    /// ground).
    pub damping: f32,
    /// Pieces at rest this long go to sleep.
    pub sleep_after: f32,
    /// Oldest pieces are removed past this count.
    pub max_pieces: usize,
    /// Impacts slower than this along the ground normal (m/s) do not bounce.
    pub bounce_threshold: f32,
    /// Contact solver iterations per step.
    pub iterations: u32,
    /// A piece counts as resting only below this speed (m/s), for both its
    /// centre and the angular speed times its radius.
    pub sleep_speed: f32,
    /// Longest internal step (s). Longer frames are split, so a frame hitch
    /// does not shove pieces deep into the ground.
    pub max_substep: f32,
}

impl Default for DebrisConfig {
    fn default() -> Self {
        Self {
            gravity: Vec3::new(0.0, -9.81, 0.0),
            ground_y: 0.0,
            restitution: 0.25,
            friction: 0.6,
            damping: 0.05,
            sleep_after: 0.5,
            max_pieces: 2000,
            bounce_threshold: 1.0,
            iterations: 8,
            sleep_speed: 0.1,
            max_substep: 1.0 / 50.0,
        }
    }
}

/// One simulated piece. Rendering keeps its own handle per `id`.
#[derive(Clone, Debug)]
pub struct DebrisBody {
    pub id: u64,
    pub position: Vec3,
    pub rotation: Quat,
    pub velocity: Vec3,
    pub angular_velocity: Vec3,
    pub contact_points: Vec<Vec3>,
    pub radius: f32,
    pub age: f32,
    /// Inverse of the diagonal inertia tensor in the piece's frame (unit
    /// mass, box of the contact points' bounds).
    inv_inertia: Vec3,
    rest_time: f32,
    /// Where the piece was when it last started resting. Sleep is decided by
    /// how far it has drifted from here over `sleep_after`, plus a speed test.
    rest_position: Vec3,
    rest_rotation: Quat,
    last_dt: f32,
    /// Accumulated normal and friction impulses per corner from the last
    /// step, to warm-start the contact solve. Resting contact then converges
    /// over a few steps instead of leaving a residual wobble.
    warm: [(f32, Vec2); MAX_CONTACTS],
    pub asleep: bool,
    /// What it is made of, for impact sounds.
    pub kind: MaterialKind,
    pub part: PiecePart,
}

/// A piece hitting the ground hard enough to be heard.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DebrisImpact {
    pub position: Vec3,
    /// Closing speed along the contact normal, m/s.
    pub speed: f32,
    /// Bounding radius of the piece, m.
    pub radius: f32,
    pub kind: MaterialKind,
    pub part: PiecePart,
}

/// Impacts slower than this (m/s) are resting contact, not hits.
pub const IMPACT_MIN_SPEED: f32 = 0.5;

/// At most this many impacts are kept for a consumer that has not drained
/// them (the oldest are dropped).
const MAX_PENDING_IMPACTS: usize = 64;

/// Inverse inertia (unit mass, local diagonal) of the box bounding `points`.
fn box_inv_inertia(points: &[Vec3], radius: f32) -> Vec3 {
    let (lo, hi) = points.iter().fold(
        (Vec3::splat(f32::INFINITY), Vec3::splat(f32::NEG_INFINITY)),
        |(lo, hi), p| (lo.min(*p), hi.max(*p)),
    );
    let h = if lo.x.is_finite() {
        (hi - lo) * 0.5
    } else {
        Vec3::splat(radius.max(1e-3))
    };
    // Keep very thin pieces (rebar, slivers) from having a near-singular
    // inertia about their long axis.
    let h = h.max(Vec3::splat((0.1 * h.max_element()).max(2e-3)));
    let h2 = h * h;
    // Solid box, m = 1: I_x = (hy² + hz²) / 3.
    Vec3::new(
        3.0 / (h2.y + h2.z),
        3.0 / (h2.x + h2.z),
        3.0 / (h2.x + h2.y),
    )
}

#[derive(Clone, Copy, Default)]
struct Contact {
    arm: Vec3,
    /// I⁻¹ (arm × axis) for the normal and the two tangents (X, Z).
    ia_n: Vec3,
    ia_x: Vec3,
    ia_z: Vec3,
    mass_n: f32,
    mass_x: f32,
    mass_z: f32,
    /// Index into `DebrisBody::warm`.
    corner: usize,
    /// Normal velocity the solve aims for (restitution).
    target: f32,
    pn: f32,
    pt: Vec2,
}

#[derive(Clone, Debug, Default)]
pub struct DebrisWorld {
    pub config: DebrisConfig,
    pub bodies: Vec<DebrisBody>,
    next_id: u64,
    /// Ids removed by the piece cap during the last `add`/`step`.
    pub removed: Vec<u64>,
    /// Ground hits since the last [`DebrisWorld::drain_impacts`].
    impacts: Vec<DebrisImpact>,
}

impl DebrisWorld {
    pub fn new(config: DebrisConfig) -> Self {
        Self {
            config,
            ..Default::default()
        }
    }

    /// Add a piece; returns its id.
    pub fn add(&mut self, piece: &DebrisPiece) -> u64 {
        let id = self.next_id;
        self.next_id += 1;
        self.bodies.push(DebrisBody {
            id,
            position: piece.position,
            rotation: piece.rotation,
            velocity: piece.velocity,
            angular_velocity: piece.angular_velocity,
            contact_points: piece.contact_points.clone(),
            radius: piece.radius,
            age: 0.0,
            inv_inertia: box_inv_inertia(&piece.contact_points, piece.radius),
            rest_time: 0.0,
            rest_position: piece.position,
            rest_rotation: piece.rotation,
            last_dt: 1.0 / 60.0,
            warm: [(0.0, Vec2::ZERO); MAX_CONTACTS],
            asleep: false,
            kind: piece.kind,
            part: piece.part,
        });
        let max = self.config.max_pieces;
        if self.bodies.len() > max {
            let extra = self.bodies.len() - max;
            self.removed
                .extend(self.bodies.drain(..extra).map(|b| b.id));
        }
        id
    }

    /// Takes the ground hits recorded since the last call (for sound).
    pub fn drain_impacts(&mut self) -> impl Iterator<Item = DebrisImpact> + '_ {
        self.impacts.drain(..)
    }

    pub fn awake_count(&self) -> usize {
        self.bodies.iter().filter(|b| !b.asleep).count()
    }

    pub fn step(&mut self, dt: f32) {
        if dt.is_nan() || dt <= 0.0 {
            return;
        }
        let c = self.config;
        let max_sub = if c.max_substep > 0.0 {
            c.max_substep
        } else {
            dt
        };
        let n = ((dt / max_sub).ceil() as u32).clamp(1, 8);
        let h = dt / n as f32;
        for b in &mut self.bodies {
            b.age += dt;
            let mut hit = 0.0f32;
            for _ in 0..n {
                if b.asleep {
                    break;
                }
                hit = hit.max(step_body(b, &c, h));
            }
            if hit >= IMPACT_MIN_SPEED {
                if self.impacts.len() >= MAX_PENDING_IMPACTS {
                    self.impacts.remove(0);
                }
                self.impacts.push(DebrisImpact {
                    position: b.position,
                    speed: hit,
                    radius: b.radius,
                    kind: b.kind,
                    part: b.part,
                });
            }
        }
    }
}

/// Advances one piece by `dt`. Returns the fastest closing speed of a corner
/// that hit the ground this step (0 if none), for impact sounds.
fn step_body(b: &mut DebrisBody, c: &DebrisConfig, dt: f32) -> f32 {
    let mut hit = 0.0f32;
    let damp = (-c.damping * dt).exp();
    b.velocity = (b.velocity + c.gravity * dt) * damp;
    b.angular_velocity *= damp;
    b.position += b.velocity * dt;
    let w = b.angular_velocity;
    if w.length_squared() > 1e-12 {
        b.rotation = (Quat::from_scaled_axis(w * dt) * b.rotation).normalize();
    }

    // Collect penetrating corners.
    let mut contacts = [Contact::default(); MAX_CONTACTS];
    let mut count = 0;
    let mut deepest = 0.0f32;
    let rot = Mat3::from_quat(b.rotation);
    let inv_i = rot * Mat3::from_diagonal(b.inv_inertia) * rot.transpose();
    for (corner, p) in b.contact_points.iter().take(MAX_CONTACTS).enumerate() {
        let arm = rot * *p;
        let depth = c.ground_y - (b.position.y + arm.y);
        if depth <= 0.0 {
            b.warm[corner] = (0.0, Vec2::ZERO);
            continue;
        }
        deepest = deepest.max(depth);
        let vn = b.velocity.y + b.angular_velocity.cross(arm).y;
        hit = hit.max(-vn);
        let rn = arm.cross(Vec3::Y);
        let rx = arm.cross(Vec3::X);
        let rz = arm.cross(Vec3::Z);
        let ia_n = inv_i * rn;
        let ia_x = inv_i * rx;
        let ia_z = inv_i * rz;
        contacts[count] = Contact {
            corner,
            arm,
            ia_n,
            ia_x,
            ia_z,
            mass_n: 1.0 / (1.0 + rn.dot(ia_n)),
            mass_x: 1.0 / (1.0 + rx.dot(ia_x)),
            mass_z: 1.0 / (1.0 + rz.dot(ia_z)),
            target: if vn < -c.bounce_threshold {
                -c.restitution * vn
            } else {
                0.0
            },
            pn: 0.0,
            pt: Vec2::ZERO,
        };
        count += 1;
    }
    let contacts = &mut contacts[..count];

    // Sequential impulses over all touching corners, accumulated and clamped,
    // warm-started from the previous step's impulses (scaled to this dt).
    let mut v = b.velocity;
    let mut w = b.angular_velocity;
    let warm_scale = dt / b.last_dt.max(1e-6);
    b.last_dt = dt;
    for k in contacts.iter_mut() {
        let (pn, pt) = b.warm[k.corner];
        if k.target > 0.0 || pn <= 0.0 {
            continue; // fresh impact: start cold
        }
        k.pn = pn * warm_scale;
        k.pt = pt * warm_scale;
        v += Vec3::new(k.pt.x, k.pn, k.pt.y);
        w += k.ia_n * k.pn + k.ia_x * k.pt.x + k.ia_z * k.pt.y;
    }
    for _ in 0..c.iterations.max(1) {
        for k in contacts.iter_mut() {
            // Normal.
            let vn = v.y + w.cross(k.arm).y;
            let pn = (k.pn + k.mass_n * (k.target - vn)).max(0.0);
            let d = pn - k.pn;
            k.pn = pn;
            v.y += d;
            w += k.ia_n * d;
            // Friction, clamped to the Coulomb disc of the normal impulse.
            let vc = v + w.cross(k.arm);
            let mut pt = k.pt + Vec2::new(-vc.x * k.mass_x, -vc.z * k.mass_z);
            let max_t = c.friction * k.pn;
            let l2 = pt.length_squared();
            if l2 > max_t * max_t {
                pt *= max_t / l2.sqrt();
            }
            let d = pt - k.pt;
            k.pt = pt;
            v.x += d.x;
            v.z += d.y;
            w += k.ia_x * d.x + k.ia_z * d.y;
        }
    }
    b.velocity = v;
    b.angular_velocity = w;
    for k in contacts.iter() {
        b.warm[k.corner] = (k.pn, k.pt);
    }

    // Position projection, separate from the velocity solve.
    let touching = count > 0;
    if touching {
        b.position.y += deepest;
    }

    let s2 = c.sleep_speed * c.sleep_speed;
    let slow_now = b.velocity.length_squared() < s2
        && b.angular_velocity.length_squared() * b.radius * b.radius < s2;
    let drift = (b.position - b.rest_position).length();
    let turn = b.rotation.angle_between(b.rest_rotation);
    let still = drift < 0.01 + 0.05 * b.radius && turn < 0.12;
    if touching && slow_now && still {
        b.rest_time += dt;
        if b.rest_time > c.sleep_after {
            b.asleep = true;
            b.velocity = Vec3::ZERO;
            b.angular_velocity = Vec3::ZERO;
        }
    } else {
        b.rest_time = 0.0;
        b.rest_position = b.position;
        b.rest_rotation = b.rotation;
    }
    hit
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fracture::{fracture, BreakInput, PiecePart};
    use crate::mesh::MeshData;
    use crate::pose::MemberPose;
    use crate::profile::Profile;
    use crate::state::MaterialKind;

    const DT: f32 = 1.0 / 60.0;

    fn lowest(b: &DebrisBody) -> f32 {
        b.contact_points
            .iter()
            .map(|p| b.position.y + (b.rotation * *p).y)
            .fold(f32::INFINITY, f32::min)
    }

    fn box_piece(half: Vec3, position: Vec3, rotation: Quat, velocity: Vec3) -> DebrisPiece {
        DebrisPiece {
            mesh: MeshData::default(),
            position,
            rotation,
            velocity,
            angular_velocity: Vec3::ZERO,
            kind: MaterialKind::Concrete,
            part: PiecePart::Chunk,
            contact_points: (0..8)
                .map(|i| {
                    Vec3::new(
                        if i & 1 == 0 { -half.x } else { half.x },
                        if i & 2 == 0 { -half.y } else { half.y },
                        if i & 4 == 0 { -half.z } else { half.z },
                    )
                })
                .collect(),
            radius: half.length(),
        }
    }

    /// Break a member of each `kind` with each seed, drop the pieces and
    /// check that every one lands, does not sink, and sleeps within 3 s of
    /// landing on its lowest corner.
    fn drop_fractures(kinds: &[MaterialKind], seeds: &[u32], profile: Profile) {
        let pose = MemberPose::straight(Vec3::new(0.0, 5.0, 0.0), Vec3::new(4.0, 5.0, 0.0));
        let mut w = DebrisWorld::new(DebrisConfig::default());
        for &kind in kinds {
            for &seed in seeds {
                let f = fracture(&BreakInput {
                    profile: &profile,
                    kind,
                    pose: &pose,
                    position: 0.5,
                    velocity_a: Vec3::ZERO,
                    velocity_b: Vec3::ZERO,
                    seed,
                });
                for p in &f.pieces {
                    w.add(p);
                }
            }
        }
        let n = w.bodies.len();
        assert!(n > 0);
        let mut landed = vec![f32::INFINITY; n];
        let mut slept = vec![f32::INFINITY; n];
        let mut t = 0.0;
        for _ in 0..60 * 10 {
            w.step(DT);
            t += DT;
            for (i, b) in w.bodies.iter().enumerate() {
                assert!(b.position.is_finite() && b.rotation.is_finite());
                let low = lowest(b);
                assert!(low > -1e-3, "piece {i} under the ground: {low}");
                if low < 1e-3 && landed[i].is_infinite() {
                    landed[i] = t;
                }
                if b.asleep && slept[i].is_infinite() {
                    slept[i] = t;
                }
            }
        }
        let mut late = Vec::new();
        for (i, b) in w.bodies.iter().enumerate() {
            let low = lowest(b);
            assert!(low.abs() < 0.01, "piece {i} lowest corner at {low}");
            let on_time = slept[i] - landed[i] <= 3.0; // false for NaN too
            if !on_time {
                late.push((i, landed[i], slept[i], b.velocity, b.angular_velocity));
            }
            if b.asleep {
                assert_eq!(b.velocity, Vec3::ZERO);
            }
        }
        assert!(
            late.is_empty(),
            "{} of {n} pieces did not sleep within 3 s of landing: {:?}",
            late.len(),
            &late[..late.len().min(5)]
        );
    }

    #[test]
    fn debris_falls_and_comes_to_rest_on_the_ground() {
        let profile = Profile::Rect { b: 0.2, h: 0.3 };
        for kind in MaterialKind::ALL {
            drop_fractures(&[kind], &[kind as u32], profile);
        }
    }

    #[test]
    fn falling_pieces_report_impacts_and_resting_ones_do_not() {
        let pose = MemberPose::straight(Vec3::new(0.0, 5.0, 0.0), Vec3::new(4.0, 5.0, 0.0));
        let profile = Profile::Rect { b: 0.2, h: 0.3 };
        let mut w = DebrisWorld::new(DebrisConfig::default());
        for kind in MaterialKind::ALL {
            let f = fracture(&BreakInput {
                profile: &profile,
                kind,
                pose: &pose,
                position: 0.5,
                velocity_a: Vec3::ZERO,
                velocity_b: Vec3::ZERO,
                seed: kind as u32,
            });
            for p in &f.pieces {
                w.add(p);
            }
        }
        let mut hits = Vec::new();
        for _ in 0..60 * 10 {
            w.step(DT);
            hits.extend(w.drain_impacts());
        }
        assert!(!hits.is_empty(), "falling pieces should report ground hits");
        assert!(hits
            .iter()
            .all(|h| h.speed >= IMPACT_MIN_SPEED && h.position.is_finite()));
        for _ in 0..60 {
            w.step(DT);
        }
        assert_eq!(
            w.drain_impacts().count(),
            0,
            "resting contact reported as impacts"
        );
    }

    /// The old jitter case: small concrete chunks that never slept.
    #[test]
    fn concrete_chunks_sleep_for_many_seeds() {
        let profile = Profile::Rect { b: 0.2, h: 0.3 };
        drop_fractures(
            &[MaterialKind::Concrete, MaterialKind::ReinforcedConcrete],
            &[0, 1, 2, 3, 7, 11, 42, 1234],
            profile,
        );
        drop_fractures(
            &MaterialKind::ALL,
            &[5, 6, 99],
            Profile::Rect { b: 0.1, h: 0.15 },
        );
    }

    #[test]
    fn box_dropped_flat_stays_put_and_sleeps() {
        let mut w = DebrisWorld::new(DebrisConfig::default());
        let half = Vec3::new(0.08, 0.04, 0.06);
        w.add(&box_piece(
            half,
            Vec3::new(1.0, 0.5, -2.0),
            Quat::IDENTITY,
            Vec3::ZERO,
        ));
        // Largest angular speed on the ground, over all and once settled.
        let (mut max_w, mut settled_w) = (0.0f32, 0.0f32);
        let mut t = 0.0;
        let mut landed = None;
        let mut slept = None;
        while t < 4.0 {
            w.step(DT);
            t += DT;
            let b = &w.bodies[0];
            if lowest(b) < 1e-3 {
                let t0 = *landed.get_or_insert(t);
                let wl = b.angular_velocity.length();
                max_w = max_w.max(wl);
                if t > t0 + 0.2 {
                    settled_w = settled_w.max(wl);
                }
            }
            if b.asleep {
                slept.get_or_insert(t);
            }
        }
        let b = &w.bodies[0];
        let landed = landed.expect("never landed");
        let slept = slept.expect("never slept");
        assert!(
            slept - landed < 1.5,
            "slept {} s after landing",
            slept - landed
        );
        assert!(max_w < 0.01, "angular velocity on the ground {max_w}");
        assert!(
            settled_w < 1e-4,
            "angular velocity once settled {settled_w}"
        );
        let drift = Vec2::new(b.position.x - 1.0, b.position.z + 2.0).length();
        assert!(drift < 1e-4, "drifted {drift}");
        assert!(
            (b.position.y - half.y).abs() < 1e-3,
            "height {}",
            b.position.y
        );
        assert!(b.rotation.angle_between(Quat::IDENTITY) < 1e-3);
    }

    #[test]
    fn sliding_box_stops_after_coulomb_distance() {
        let config = DebrisConfig::default();
        let mu = config.friction;
        let g = -config.gravity.y;
        let mut w = DebrisWorld::new(config);
        let half = Vec3::new(0.1, 0.05, 0.1);
        let v0 = 3.0;
        w.add(&box_piece(
            half,
            Vec3::new(0.0, half.y, 0.0),
            Quat::IDENTITY,
            Vec3::new(v0, 0.0, 0.0),
        ));
        let mut stopped_at = None;
        for i in 0..600 {
            w.step(DT);
            let b = &w.bodies[0];
            if stopped_at.is_none() && b.velocity.x.abs() < 1e-3 {
                stopped_at = Some((i as f32 * DT, b.position.x));
            }
        }
        let expected = v0 * v0 / (2.0 * mu * g);
        let (t, x) = stopped_at.expect("never stopped");
        let b = &w.bodies[0];
        assert!(
            (x - expected).abs() < 0.25 * expected,
            "slid {x} m, expected about {expected} m"
        );
        // Stop time v0/(mu g) too: friction is not over-damped.
        let t_expected = v0 / (mu * g);
        assert!(
            (t - t_expected).abs() < 0.25 * t_expected,
            "stopped after {t} s"
        );
        assert!(b.asleep);
        assert!((b.position.x - x).abs() < 0.01, "crept after stopping");
        assert!(lowest(b).abs() < 1e-3);
    }

    #[test]
    fn tall_piece_on_its_corner_tips_onto_a_face_and_sleeps() {
        let mut w = DebrisWorld::new(DebrisConfig::default());
        let half = Vec3::new(0.04, 0.25, 0.06);
        // Long axis tilted off vertical, so it lands on one corner.
        let rot = Quat::from_rotation_z(0.3) * Quat::from_rotation_x(0.2);
        w.add(&box_piece(half, Vec3::new(0.0, 1.0, 0.0), rot, Vec3::ZERO));
        let mut t = 0.0;
        while t < 6.0 && !w.bodies[0].asleep {
            w.step(DT);
            t += DT;
        }
        let b = &w.bodies[0];
        assert!(b.asleep, "still awake after {t} s");
        // Lying on a face: the long axis is horizontal and the piece's lowest
        // face is flat on the ground (four corners touching).
        let up = (b.rotation * Vec3::Y).y.abs();
        assert!(up < 0.05, "long axis still up: {up}");
        let touching = b
            .contact_points
            .iter()
            .filter(|p| (b.position.y + (b.rotation * **p).y).abs() < 5e-3)
            .count();
        assert_eq!(touching, 4, "not lying on a face");
        assert!(lowest(b).abs() < 1e-3);
    }

    #[test]
    #[ignore = "timing; run with --ignored --nocapture"]
    fn cost_per_awake_piece() {
        let pose = MemberPose::straight(Vec3::new(0.0, 5.0, 0.0), Vec3::new(4.0, 5.0, 0.0));
        let profile = Profile::Rect { b: 0.2, h: 0.3 };
        let mut w = DebrisWorld::new(DebrisConfig::default());
        for seed in 0..200 {
            let f = fracture(&BreakInput {
                profile: &profile,
                kind: MaterialKind::ALL[seed as usize % 5],
                pose: &pose,
                position: 0.5,
                velocity_a: Vec3::ZERO,
                velocity_b: Vec3::ZERO,
                seed,
            });
            for p in &f.pieces {
                w.add(p);
            }
        }
        let mut awake_steps = 0usize;
        let start = std::time::Instant::now();
        for _ in 0..600 {
            awake_steps += w.awake_count();
            w.step(DT);
        }
        let ns = start.elapsed().as_nanos() as f64 / awake_steps.max(1) as f64;
        println!(
            "{} pieces, {awake_steps} awake piece-steps, {ns:.0} ns each (incl. awake_count)",
            w.bodies.len()
        );
    }
}
