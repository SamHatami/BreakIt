//! Glue between the solver and [`crate::contact`]: per-node ground penalty
//! terms and committing touch-downs as contact events.

use super::*;

/// Touch-downs slower than this (m/s) are not reported as contact events.
pub(super) const MIN_IMPACT_SPEED: f64 = 0.05;

/// Mass that sets a node's ground penalty stiffness: its own mass plus the
/// mass equivalent of the load on it, so a heavily loaded light node does not
/// sink into the floor. Weight carried through the members is not included:
/// a lagged stiffness that follows it costs 2 to 4 times the CG iterations
/// for a resting tower, and the column bases of a 3x3x8 steel tower standing
/// on the floor only sink 4 mm without it.
pub(super) fn contact_mass(b: &Body) -> f64 {
    1.0 / b.inv_mass + b.force.norm() / 9.81
}
impl DynamicSim {
    /// Contact terms of a free node, if it touches the ground or is held by
    /// friction.
    pub(super) fn contact_terms(
        &self,
        gr: &Ground,
        bi: usize,
        h: f64,
    ) -> Option<crate::contact::Terms> {
        let b = &self.bodies[bi];
        if b.inv_mass <= 0.0 {
            return None;
        }
        let c = &self.contacts[bi];
        gr.terms(c, b.x0, b.x, contact_mass(b), h)
    }

    pub(super) fn commit_contacts(&mut self, gr: &Ground, h: f64) {
        for (bi, b) in self.bodies.iter_mut().enumerate() {
            if b.inv_mass <= 0.0 {
                continue;
            }
            let c = &mut self.contacts[bi];
            // Pushing a node out of the ground must not launch it: the
            // outward velocity that only undoes last substep's penetration is
            // removed, so landings do not bounce off the penalty.
            if let Some((depth, n)) = gr.deepest(c, b.x0, b.x_prev) {
                let vn = b.v.dot(&n);
                if vn > 0.0 {
                    b.v -= n * vn.min(depth / h);
                }
            }
            // Approach speed: the velocity this substep started with, into
            // the surface it is now touching.
            let speed = gr
                .deepest(c, b.x0, b.x)
                .map_or(0.0, |(_, n)| -(b.x_tilde - b.x_prev).dot(&n) / h);
            let m = contact_mass(b);
            if gr.commit(c, b.x0, b.x, m, h) && speed > MIN_IMPACT_SPEED {
                self.contact_events.push(ContactEvent {
                    node: bi,
                    time: self.time,
                    speed,
                    mass: 1.0 / b.inv_mass,
                });
            }
        }
    }
}
