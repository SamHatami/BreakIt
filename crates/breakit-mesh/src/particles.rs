//! Fracture particles: sparks, dust, splinters and fibres.
//!
//! A fixed-capacity pool simulated on the CPU (20k particles step in well
//! under a millisecond) and drawn as one instanced batch.

use glam::{Vec3, Vec4};

use crate::fracture::{BurstKind, ParticleBurst, Rng};

#[derive(Clone, Copy, Debug, Default)]
pub struct Particle {
    pub position: Vec3,
    pub velocity: Vec3,
    pub age: f32,
    pub life: f32,
    pub size: f32,
    /// Linear colour; alpha is unused (particles are opaque, they shrink out).
    pub color: Vec4,
    /// Emissive strength (sparks glow).
    pub glow: f32,
    pub drag: f32,
    pub gravity: f32,
}

impl Particle {
    pub fn alive(&self) -> bool {
        self.age < self.life
    }
    /// Size this frame: grows in briefly, shrinks out at the end.
    pub fn current_size(&self) -> f32 {
        if !self.alive() {
            return 0.0;
        }
        let t = self.age / self.life;
        let grow = (t / 0.05).min(1.0);
        let fade = 1.0 - ((t - 0.6) / 0.4).clamp(0.0, 1.0);
        self.size * grow * fade
    }
}

#[derive(Clone, Debug)]
pub struct ParticlePool {
    pub particles: Vec<Particle>,
    next: usize,
    rng: Rng,
    pub ground_y: f32,
}

impl ParticlePool {
    pub fn new(capacity: usize) -> Self {
        Self {
            particles: vec![Particle::default(); capacity],
            next: 0,
            rng: Rng::new(12345),
            ground_y: 0.0,
        }
    }

    pub fn capacity(&self) -> usize {
        self.particles.len()
    }

    pub fn alive_count(&self) -> usize {
        self.particles.iter().filter(|p| p.alive()).count()
    }

    fn spawn(&mut self, p: Particle) {
        // Ring buffer: overwrite the oldest slot when full.
        let n = self.particles.len();
        if n == 0 {
            return;
        }
        self.particles[self.next] = p;
        self.next = (self.next + 1) % n;
    }

    pub fn emit(&mut self, b: &ParticleBurst) {
        let axis = b.axis.try_normalize().unwrap_or(Vec3::X);
        for _ in 0..b.count {
            let r = &mut self.rng;
            let dir = r.unit_vec();
            let radial = (dir - axis * dir.dot(axis))
                .try_normalize()
                .unwrap_or(Vec3::Y);
            let offset = r.unit_vec() * (0.4 * b.scale);
            let p = match b.kind {
                BurstKind::Sparks => Particle {
                    position: b.origin + offset * 0.3,
                    velocity: (radial * 0.7 + dir * 0.3) * r.range(2.0, 7.0) + Vec3::Y * 1.0,
                    life: r.range(0.3, 0.9),
                    size: r.range(0.008, 0.02),
                    color: Vec4::new(1.0, 0.55, 0.15, 1.0),
                    glow: 12.0,
                    drag: 0.8,
                    gravity: 1.0,
                    age: 0.0,
                },
                BurstKind::Dust => Particle {
                    position: b.origin + offset,
                    velocity: radial * r.range(0.2, 1.2) + Vec3::Y * r.range(0.0, 0.6),
                    life: r.range(1.5, 3.5),
                    size: (r.range(0.25, 0.6) * b.scale).max(0.06),
                    color: Vec4::new(0.55, 0.53, 0.5, 1.0) * r.range(0.85, 1.1),
                    glow: 0.0,
                    drag: 2.5,
                    gravity: 0.05,
                    age: 0.0,
                },
                BurstKind::Splinters => Particle {
                    position: b.origin + offset * 0.5,
                    velocity: (radial + axis * r.signed() * 0.5) * r.range(1.0, 4.0) + Vec3::Y,
                    life: r.range(1.0, 2.5),
                    size: r.range(0.01, 0.03),
                    color: Vec4::new(0.62, 0.45, 0.26, 1.0),
                    glow: 0.0,
                    drag: 0.5,
                    gravity: 1.0,
                    age: 0.0,
                },
                BurstKind::Fibres => Particle {
                    position: b.origin + offset * 0.5,
                    velocity: radial * r.range(0.5, 2.5) + Vec3::Y * 0.3,
                    life: r.range(1.0, 2.5),
                    size: r.range(0.006, 0.015),
                    color: Vec4::new(0.08, 0.08, 0.09, 1.0),
                    glow: 0.0,
                    drag: 1.5,
                    gravity: 0.4,
                    age: 0.0,
                },
            };
            self.spawn(p);
        }
    }

    pub fn step(&mut self, dt: f32) {
        let g = Vec3::new(0.0, -9.81, 0.0);
        for p in &mut self.particles {
            if !p.alive() {
                continue;
            }
            p.age += dt;
            p.velocity = p.velocity * (-p.drag * dt).exp() + g * (p.gravity * dt);
            p.position += p.velocity * dt;
            if p.position.y < self.ground_y {
                p.position.y = self.ground_y;
                p.velocity.y *= -0.3;
                p.velocity.x *= 0.5;
                p.velocity.z *= 0.5;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn particles_live_and_die() {
        let mut pool = ParticlePool::new(256);
        pool.emit(&ParticleBurst {
            kind: BurstKind::Sparks,
            origin: Vec3::new(0.0, 2.0, 0.0),
            axis: Vec3::X,
            count: 100,
            scale: 0.2,
        });
        assert_eq!(pool.alive_count(), 100);
        for _ in 0..120 {
            pool.step(1.0 / 60.0);
        }
        assert_eq!(pool.alive_count(), 0, "sparks die within two seconds");
        assert!(pool.particles.iter().all(|p| p.position.y >= 0.0));
    }

    #[test]
    fn pool_overwrites_oldest_when_full() {
        let mut pool = ParticlePool::new(10);
        let b = ParticleBurst {
            kind: BurstKind::Dust,
            origin: Vec3::ZERO,
            axis: Vec3::X,
            count: 25,
            scale: 0.2,
        };
        pool.emit(&b);
        assert_eq!(pool.alive_count(), 10);
    }
}
