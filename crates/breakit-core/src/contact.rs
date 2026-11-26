//! Frictional contact between structure nodes and the ground (or a table top).
//!
//! The ground is a horizontal plane at `height` (world is Y-up). Every free
//! node is a sphere of radius `r`, about half its members' section size, so a
//! fallen beam rests on its surface instead of its centreline. Members are
//! straight between nodes for contact purposes; with members subdivided the
//! way the game builds them that is enough for a plane.
//!
//! In the direct solver contact is part of the substep objective, so the
//! Gauss-Newton solve, the step check and the linear solver all see it:
//!
//! * Normal: a one-sided penalty on the penetration d = (height + r) − y. Its
//!   stiffness κ is a multiple of the node's m/h², so a resting node sinks by
//!   about (load / own weight) · g h² / `stiffness` (micrometres), and it ramps
//!   in linearly over the first `ramp` of penetration. The ramp keeps the
//!   energy twice differentiable, so the Gauss-Newton matrix changes smoothly
//!   as nodes land and an old factorisation stays a good preconditioner.
//! * Friction: Coulomb, as an elastic-plastic tangential spring between the
//!   node and an anchor point on the ground. The spring holds up to the
//!   friction limit μN after `stick_slip` of tangential slip; beyond that the
//!   anchor is dragged along and the force stays at μN. N is the normal force
//!   at the end of the previous substep (lagged), and the anchor is fixed
//!   during a substep, so the friction energy is exact for the solve. Unlike a
//!   velocity-smoothed friction law a node held by static friction does not
//!   creep.
//!
//! A node landing between substeps would otherwise start the Gauss-Newton
//! iteration far inside the stiff penalty region, where one linearisation
//! overshoots. The substep start position is therefore never put deeper than
//! `ramp`: the node lands on the surface in one substep and its downward
//! velocity is gone (perfectly inelastic, which is what beams and concrete do).
//!
//! Only translations touch the ground. Rolling is resisted by the members, and
//! a loose node's spin by the solver's damping.

use crate::Vec3;

/// A static axis-aligned box the structure can rest on: a table block, a
/// river bank, an abutment, a pier.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Block {
    pub min: Vec3,
    pub max: Vec3,
}

impl Block {
    pub fn new(min: Vec3, max: Vec3) -> Self {
        Self { min, max }
    }

    /// Signed distance from `p` and the outward normal there.
    fn distance(&self, p: Vec3) -> (f64, Vec3) {
        let c = (self.min + self.max) * 0.5;
        let half = (self.max - self.min) * 0.5;
        let d = p - c;
        let q = d.abs() - half;
        let out = q.map(|v| v.max(0.0));
        let len = out.norm();
        if len > 0.0 {
            let n = Vec3::new(
                out.x * d.x.signum(),
                out.y * d.y.signum(),
                out.z * d.z.signum(),
            ) / len;
            (len, n)
        } else {
            // Inside: leave through the nearest face.
            let k = if q.x >= q.y && q.x >= q.z {
                0
            } else if q.y >= q.z {
                1
            } else {
                2
            };
            let mut n = Vec3::zeros();
            n[k] = if d[k] >= 0.0 { 1.0 } else { -1.0 };
            (q[k], n)
        }
    }
}

/// The ground and its contact parameters: a floor plane at `height` plus any
/// number of static boxes.
#[derive(Clone, Debug, PartialEq)]
pub struct Ground {
    /// Height of the floor plane, m. `f64::NEG_INFINITY` for no floor (only
    /// blocks).
    pub height: f64,
    /// Static boxes on or above the floor.
    pub blocks: Vec<Block>,
    /// Coulomb friction coefficient (static = kinetic).
    pub friction: f64,
    /// Penalty stiffness as a multiple of each node's m/h².
    pub stiffness: f64,
    /// Penetration over which the penalty stiffness ramps in, m.
    pub ramp: f64,
    /// Tangential slip at which static friction reaches μN, m.
    pub stick_slip: f64,
    /// Scale on each node's contact radius (half its members' section size).
    /// 0 puts contact at the node itself.
    pub radius_scale: f64,
}

impl Default for Ground {
    fn default() -> Self {
        Self {
            height: 0.0,
            blocks: Vec::new(),
            friction: 0.6,
            stiffness: 20.0,
            ramp: 2e-3,
            stick_slip: 1e-3,
            radius_scale: 1.0,
        }
    }
}

/// A node touching the ground after being off it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ContactEvent {
    pub node: usize,
    pub time: f64,
    /// Speed into the surface just before touching, m/s.
    pub speed: f64,
    /// The node's mass, kg (for scaling sound or dust by impulse).
    pub mass: f64,
}

/// Per-node contact state.
#[derive(Clone, Debug, Default)]
pub(crate) struct NodeContact {
    /// Half the largest member section at this node, m (before
    /// `radius_scale` and before the clamp to the built position).
    pub radius: f64,
    /// Friction anchor while in contact, and the surface (0 = floor, 1 + i =
    /// block i) it belongs to.
    pub anchor: Option<(Vec3, usize)>,
    /// Normal force at the end of the last substep, N.
    pub normal_force: f64,
}

/// Energy, gradient and diagonal Gauss-Newton stiffness of one node's
/// contact. The stiffness is exact on faces (axis-aligned normals) for the
/// normal term and stick friction; on box edges and corners it is the
/// diagonal of the exact one, and while sliding it is the secant μN/s.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Terms {
    pub energy: f64,
    pub grad: Vec3,
    pub hess: Vec3,
}

/// One surface a node is near: (surface id, penetration, outward normal).
type Touch = (usize, f64, Vec3);

impl Ground {
    /// Signed distance of a point to surface `id` and the normal there.
    fn distance(&self, id: usize, p: Vec3) -> (f64, Vec3) {
        if id == 0 {
            (p.y - self.height, Vec3::y())
        } else {
            self.blocks[id - 1].distance(p)
        }
    }

    fn surfaces(&self) -> std::ops::Range<usize> {
        0..1 + self.blocks.len()
    }

    /// Contact radius of a node built at `x0`. Nodes built closer to a
    /// surface than their radius touch at their built position, so a beam
    /// drawn on the table does not jump up at the start.
    fn radius(&self, c: &NodeContact, x0: Vec3) -> f64 {
        let clear = self
            .surfaces()
            .map(|id| self.distance(id, x0).0)
            .fold(f64::INFINITY, f64::min);
        (c.radius * self.radius_scale).min(clear.max(0.0))
    }

    /// Every surface the node at `x` penetrates or touches (penetration ≥ 0).
    fn touches(&self, c: &NodeContact, x0: Vec3, x: Vec3) -> impl Iterator<Item = Touch> + '_ {
        let r = self.radius(c, x0);
        self.surfaces().filter_map(move |id| {
            let (phi, n) = self.distance(id, x);
            let d = r - phi;
            (d > 0.0).then_some((id, d, n))
        })
    }

    /// Distance of the node's surface from the nearest ground surface, m
    /// (negative when penetrating).
    pub(crate) fn clearance(&self, c: &NodeContact, x0: Vec3, x: Vec3) -> f64 {
        let r = self.radius(c, x0);
        self.surfaces()
            .map(|id| self.distance(id, x).0 - r)
            .fold(f64::INFINITY, f64::min)
    }

    fn kappa(&self, mass: f64, h: f64) -> f64 {
        self.stiffness * mass / (h * h)
    }

    /// Normal penalty: (energy, force, stiffness) at penetration `d`.
    fn normal(&self, d: f64, kappa: f64) -> (f64, f64, f64) {
        if d <= 0.0 {
            return (0.0, 0.0, 0.0);
        }
        let r = self.ramp.max(1e-9);
        if d < r {
            (
                kappa * d * d * d / (6.0 * r),
                kappa * d * d / (2.0 * r),
                kappa * d / r,
            )
        } else {
            (
                kappa * (0.5 * d * d - 0.5 * r * d + r * r / 6.0),
                kappa * (d - 0.5 * r),
                kappa,
            )
        }
    }

    /// Contact terms for a node built at `x0`, now at `x`, or `None` when it
    /// is neither touching nor held by friction.
    pub(crate) fn terms(
        &self,
        c: &NodeContact,
        x0: Vec3,
        x: Vec3,
        mass: f64,
        h: f64,
    ) -> Option<Terms> {
        let kappa = self.kappa(mass, h);
        let mut t = Terms {
            energy: 0.0,
            grad: Vec3::zeros(),
            hess: Vec3::zeros(),
        };
        let mut any = false;
        for (_, d, n) in self.touches(c, x0, x) {
            let (e, f, k) = self.normal(d, kappa);
            t.energy += e;
            t.grad -= n * f;
            t.hess += n.component_mul(&n) * k;
            any = true;
        }
        let limit = self.friction * c.normal_force;
        if let (Some((a, id)), true) = (c.anchor, limit > 0.0) {
            // Friction in the tangent plane of the surface it started on.
            let n = self.distance(id, x).1;
            let s_y = self.stick_slip.max(1e-9);
            let w = x - a;
            let u = w - n * n.dot(&w);
            let s = u.norm();
            let (e, gk) = if s <= s_y {
                (0.5 * limit / s_y * s * s, limit / s_y)
            } else {
                (limit * (s - 0.5 * s_y), limit / s)
            };
            t.energy += e;
            t.grad += u * gk;
            t.hess += (Vec3::repeat(1.0) - n.component_mul(&n)) * gk;
            any = true;
        }
        any.then_some(t)
    }

    /// Start position for the substep solve: never deeper than the ramp.
    pub(crate) fn clamp_start(&self, c: &NodeContact, x0: Vec3, x: &mut Vec3) {
        let r = self.radius(c, x0);
        for id in self.surfaces() {
            let (phi, n) = self.distance(id, *x);
            let excess = r - phi - self.ramp;
            if excess > 0.0 {
                *x += n * excess;
            }
        }
    }

    /// The deepest surface the node penetrates at `x`: (penetration, normal).
    pub(crate) fn deepest(&self, c: &NodeContact, x0: Vec3, x: Vec3) -> Option<(f64, Vec3)> {
        self.touches(c, x0, x)
            .max_by(|a, b| a.1.total_cmp(&b.1))
            .map(|(_, d, n)| (d, n))
    }

    /// After a substep: record the normal force for the next substep's
    /// friction and drag the friction anchor. Returns true if the node just
    /// touched down.
    pub(crate) fn commit(&self, c: &mut NodeContact, x0: Vec3, x: Vec3, mass: f64, h: f64) -> bool {
        let kappa = self.kappa(mass, h);
        let was = c.normal_force > 0.0;
        // Friction belongs to the surface pressing hardest.
        let best = self
            .touches(c, x0, x)
            .map(|(id, d, n)| (id, self.normal(d, kappa).1, n))
            .filter(|t| t.1 > 0.0)
            .max_by(|a, b| a.1.total_cmp(&b.1));
        let Some((id, force, n)) = best else {
            c.normal_force = 0.0;
            c.anchor = None;
            return false;
        };
        c.normal_force = force;
        match &mut c.anchor {
            Some((a, aid)) if *aid == id => {
                let s_y = self.stick_slip.max(1e-9);
                let w = x - *a;
                let u = w - n * n.dot(&w);
                let s = u.norm();
                if s > s_y {
                    *a += u * (1.0 - s_y / s);
                }
            }
            _ => c.anchor = Some((x, id)),
        }
        !was
    }

    /// Position-based contact for the Gauss-Seidel solver: push the node out
    /// and cancel tangential motion up to μ times the push.
    pub(crate) fn project(&self, c: &NodeContact, x0: Vec3, x: &mut Vec3, x_prev: Vec3) {
        let r = self.radius(c, x0);
        for id in self.surfaces() {
            let (phi, n) = self.distance(id, *x);
            let depth = r - phi;
            if depth <= 0.0 {
                continue;
            }
            *x += n * depth;
            let w = *x - x_prev;
            let u = w - n * n.dot(&w);
            let s = u.norm();
            let limit = self.friction * depth;
            *x -= u * if s <= limit { 1.0 } else { limit / s };
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn normal_penalty_is_smooth_and_consistent() {
        let g = Ground::default();
        let kappa = 1e6;
        let r = g.ramp;
        for d in [-1e-3, 0.0, 0.3 * r, r, 1.7 * r, 10.0 * r] {
            let (e, f, k) = g.normal(d, kappa);
            let eps = 1e-9;
            let (ep, fp, _) = g.normal(d + eps, kappa);
            let (em, fm, _) = g.normal(d - eps, kappa);
            assert!(
                ((ep - em) / (2.0 * eps) - f).abs() <= 1e-4 * (1.0 + f.abs()),
                "force at {d}"
            );
            assert!(
                ((fp - fm) / (2.0 * eps) - k).abs() <= 1e-6 * kappa,
                "stiffness at {d}"
            );
            assert!(e >= 0.0 && f >= 0.0 && k >= 0.0);
        }
    }

    #[test]
    fn friction_gradient_matches_energy() {
        let g = Ground::default();
        let c = NodeContact {
            radius: 0.0,
            anchor: Some((Vec3::zeros(), 0)),
            normal_force: 100.0,
        };
        for u in [0.0003, 0.0009, 0.004, 0.2] {
            let x0 = Vec3::zeros();
            let x = Vec3::new(u * 0.6, -0.0005, u * 0.8);
            let t = g.terms(&c, x0, x, 10.0, 1.0 / 240.0).unwrap();
            for axis in 0..3 {
                let eps = 1e-8;
                let mut xp = x;
                xp[axis] += eps;
                let mut xm = x;
                xm[axis] -= eps;
                let ep = g.terms(&c, x0, xp, 10.0, 1.0 / 240.0).unwrap().energy;
                let em = g.terms(&c, x0, xm, 10.0, 1.0 / 240.0).unwrap().energy;
                let fd = (ep - em) / (2.0 * eps);
                assert!(
                    (fd - t.grad[axis]).abs() <= 1e-4 * (1.0 + fd.abs()),
                    "slip {u}"
                );
            }
            // Never more than μN.
            assert!((t.grad.x.hypot(t.grad.z)) <= g.friction * c.normal_force * (1.0 + 1e-9));
        }
    }
}

#[cfg(test)]
mod block_tests {
    use super::*;

    #[test]
    fn block_distance_and_normals() {
        let b = Block::new(Vec3::new(-1.0, -1.0, -1.0), Vec3::new(1.0, 0.0, 1.0));
        let (d, n) = b.distance(Vec3::new(0.2, 0.5, 0.0));
        assert!((d - 0.5).abs() < 1e-12 && n == Vec3::y());
        let (d, n) = b.distance(Vec3::new(0.9, -0.05, 0.0));
        assert!((d + 0.05).abs() < 1e-12 && n == Vec3::y());
        let (d, n) = b.distance(Vec3::new(1.3, -0.5, 0.0));
        assert!((d - 0.3).abs() < 1e-12 && n == Vec3::x());
        let (d, n) = b.distance(Vec3::new(1.3, 0.4, 0.0));
        assert!((d - 0.5).abs() < 1e-12 && (n - Vec3::new(0.6, 0.8, 0.0)).norm() < 1e-12);
    }

    #[test]
    fn gradient_matches_energy_on_a_block_edge() {
        let g = Ground {
            height: f64::NEG_INFINITY,
            blocks: vec![Block::new(
                Vec3::new(-1.0, -1.0, -1.0),
                Vec3::new(1.0, 0.0, 1.0),
            )],
            ..Default::default()
        };
        let c = NodeContact {
            radius: 0.05,
            anchor: None,
            normal_force: 0.0,
        };
        let x0 = Vec3::new(0.0, 1.0, 0.0);
        let x = Vec3::new(1.02, 0.03, 0.1);
        let t = g.terms(&c, x0, x, 10.0, 1.0 / 240.0).unwrap();
        for axis in 0..3 {
            let eps = 1e-8;
            let mut xp = x;
            xp[axis] += eps;
            let mut xm = x;
            xm[axis] -= eps;
            let ep = g.terms(&c, x0, xp, 10.0, 1.0 / 240.0).unwrap().energy;
            let em = g.terms(&c, x0, xm, 10.0, 1.0 / 240.0).unwrap().energy;
            let fd = (ep - em) / (2.0 * eps);
            assert!(
                (fd - t.grad[axis]).abs() <= 1e-4 * (1.0 + fd.abs()),
                "axis {axis}: {fd} vs {}",
                t.grad[axis]
            );
        }
    }
}
