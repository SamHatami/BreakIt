//! Visual-only debris: cheap rigid bodies that fall, tumble, hit the ground
//! and come to rest. They never feed back into the structural solver.
//!
//! Contact is a ground plane tested at each piece's bounding-box corners, with
//! restitution and friction impulses. That is crude but stable, costs well
//! under a microsecond per piece, and debris that has settled sleeps. Swap in
//! Avian/Rapier later if debris should pile up on itself or on the structure.

use glam::{Quat, Vec3};

use crate::fracture::DebrisPiece;

#[derive(Clone, Copy, Debug)]
pub struct DebrisConfig {
    pub gravity: Vec3,
    pub ground_y: f32,
    pub restitution: f32,
    pub friction: f32,
    /// Linear and angular damping, 1/s.
    pub damping: f32,
    /// Pieces at rest this long go to sleep.
    pub sleep_after: f32,
    /// Oldest pieces are removed past this count.
    pub max_pieces: usize,
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
    rest_time: f32,
    pub asleep: bool,
}

#[derive(Clone, Debug, Default)]
pub struct DebrisWorld {
    pub config: DebrisConfig,
    pub bodies: Vec<DebrisBody>,
    next_id: u64,
    /// Ids removed by the piece cap during the last `add`/`step`.
    pub removed: Vec<u64>,
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
            rest_time: 0.0,
            asleep: false,
        });
        let max = self.config.max_pieces;
        if self.bodies.len() > max {
            let extra = self.bodies.len() - max;
            self.removed
                .extend(self.bodies.drain(..extra).map(|b| b.id));
        }
        id
    }

    pub fn awake_count(&self) -> usize {
        self.bodies.iter().filter(|b| !b.asleep).count()
    }

    pub fn step(&mut self, dt: f32) {
        let c = self.config;
        let damp = (-c.damping * dt).exp();
        for b in &mut self.bodies {
            b.age += dt;
            if b.asleep {
                continue;
            }
            b.velocity = (b.velocity + c.gravity * dt) * damp;
            b.angular_velocity *= damp;
            b.position += b.velocity * dt;
            let w = b.angular_velocity;
            if w.length_squared() > 1e-12 {
                b.rotation = (Quat::from_scaled_axis(w * dt) * b.rotation).normalize();
            }
            // Ground contact at every penetrating corner, sequential impulses
            // (two passes). Inertia is a solid sphere of the piece's radius:
            // crude, stable, cheap.
            let inv_i = 2.5 / (b.radius * b.radius).max(1e-6); // 1/(2/5 m r²), m = 1
            let n = Vec3::Y;
            let mut deepest = 0.0f32;
            for pass in 0..2 {
                for p in &b.contact_points {
                    let arm = b.rotation * *p;
                    let depth = c.ground_y - (b.position.y + arm.y);
                    if depth <= 0.0 {
                        continue;
                    }
                    deepest = deepest.max(depth);
                    let v_contact = b.velocity + b.angular_velocity.cross(arm);
                    let vn = v_contact.dot(n);
                    if vn >= 0.0 {
                        continue;
                    }
                    // Bounce only real impacts; resting contact is inelastic.
                    let e = if pass == 0 && vn < -1.0 {
                        c.restitution
                    } else {
                        0.0
                    };
                    let rn = arm.cross(n);
                    let k = 1.0 + inv_i * rn.length_squared();
                    let jn = -(1.0 + e) * vn / k;
                    let vt = v_contact - n * vn;
                    let mut impulse = n * jn;
                    if vt.length_squared() > 1e-10 {
                        let t = vt.normalize();
                        let rt = arm.cross(t);
                        let kt = 1.0 + inv_i * rt.length_squared();
                        let jt = (vt.length() / kt).min(c.friction * jn);
                        impulse -= t * jt;
                    }
                    b.velocity += impulse;
                    b.angular_velocity += arm.cross(impulse) * inv_i;
                }
            }
            let touching = deepest > 0.0;
            if touching {
                b.position.y += deepest;
                // Rolling resistance: lumpy pieces should not roll forever.
                b.angular_velocity *= (-4.0 * dt).exp();
                // Resting contact: corner-by-corner impulses jitter, so slow
                // pieces on the ground are damped hard until they sleep.
                if b.velocity.length_squared() < 0.25 {
                    let k = (-10.0 * dt).exp();
                    b.velocity *= k;
                    b.angular_velocity *= k;
                }
            }
            let slow = touching
                && b.velocity.length_squared() < 0.02
                && b.angular_velocity.length_squared() < 0.36;
            if slow {
                b.rest_time += dt;
                if b.rest_time > c.sleep_after {
                    b.asleep = true;
                    b.velocity = Vec3::ZERO;
                    b.angular_velocity = Vec3::ZERO;
                }
            } else {
                b.rest_time = 0.0;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fracture::{fracture, BreakInput};
    use crate::pose::MemberPose;
    use crate::profile::Profile;
    use crate::state::MaterialKind;

    #[test]
    fn debris_falls_and_comes_to_rest_on_the_ground() {
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
        for _ in 0..60 * 12 {
            w.step(1.0 / 60.0);
        }
        for b in &w.bodies {
            assert!(b.position.is_finite());
            // Resting on (not under, not far above) the ground.
            let lowest = b
                .contact_points
                .iter()
                .map(|p| b.position.y + (b.rotation * *p).y)
                .fold(f32::INFINITY, f32::min);
            assert!(lowest > -0.02 && lowest < 0.3, "lowest point {lowest}");
        }
        assert!(
            w.awake_count() * 10 <= w.bodies.len(),
            "{} of {} still awake",
            w.awake_count(),
            w.bodies.len()
        );
    }
}
