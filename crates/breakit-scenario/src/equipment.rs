//! Equipment on the structure during a test run: its overturning couple,
//! whether it stays on, and how far it tilts, sags and drifts.
//!
//! The model is a rigid body standing on its feet. Its mass is lumped on
//! the foot nodes (in the solver, as point masses), so weight and sideways
//! inertia already reach the structure. What lumping misses is the centre
//! of mass `height` above the feet: sideways inertia H acting there is a
//! moment h ŷ × H about the footprint, which goes on the feet as the
//! smallest set of vertical forces with that moment. The foot reactions
//! An outside force on the body (wind on a sign) acts at the same height and
//! is added to the feet with its couple. The foot reactions
//! then say whether a resting item lifts a foot (tips) or out-pulls its
//! friction (slides), and whether bolts are pulled out or sheared.
//!
//! An item that stays unstable for [`LOSS_TIME`] seconds in total (stable
//! time pays it back at half rate) comes off: its mass leaves the solver
//! and the run reports where and how fast it went. Falling and hitting
//! things is the loads module's job (`breakit_core::loads::impact`).

use breakit_core::{DynamicSim, Vec3};

use crate::scenario::{Mount, Payload};

/// Seconds of accumulated instability before a resting item is lost.
pub const LOSS_TIME: f64 = 0.4;
/// Friction once bolts have failed.
pub const BROKEN_BOLT_FRICTION: f64 = 0.3;

/// One piece of equipment during a run.
#[derive(Clone, Debug)]
pub struct Equipment {
    pub feet: Vec<usize>,
    /// Horizontal foot offsets from the footprint centre, as built.
    offsets: Vec<(f64, f64)>,
    pub mass: f64,
    pub height: f64,
    pub mount: Mount,
    pub bolts_failed: bool,
    pub lost: bool,
    unstable: f64,
    prev_velocity: Option<Vec3>,
    /// Vertical force per foot from the overturning couple, N (applied to
    /// the structure, + up).
    couple: Vec<f64>,
    /// Outside force on the body this frame (wind), N, acting `height`
    /// above the feet. Set before each step; it goes to the structure
    /// through the feet with its overturning couple.
    pub wind: Vec3,
    pub peak_tilt: f64,
    pub peak_sag: f64,
    pub peak_drift: f64,
    /// Smallest foot reaction seen as a fraction of the static share
    /// (below 0: a foot lifted).
    pub min_reaction: f64,
}

/// What an update found.
#[derive(Clone, Debug, PartialEq)]
pub enum EquipmentEvent {
    BoltsFailed,
    Lost { position: Vec3, velocity: Vec3 },
}

impl Equipment {
    pub fn new(p: &Payload, feet: Vec<usize>, sim: &DynamicSim) -> Self {
        let pos: Vec<Vec3> = feet.iter().map(|&n| sim.position(n)).collect();
        let c = pos.iter().sum::<Vec3>() / pos.len() as f64;
        let offsets = pos.iter().map(|p| (p.x - c.x, p.z - c.z)).collect();
        let n = feet.len();
        Self {
            feet,
            offsets,
            mass: p.mass,
            height: p.height,
            mount: p.mount,
            bolts_failed: false,
            lost: false,
            unstable: 0.0,
            prev_velocity: None,
            couple: vec![0.0; n],
            wind: Vec3::zeros(),
            peak_tilt: 0.0,
            peak_sag: 0.0,
            peak_drift: 0.0,
            min_reaction: 1.0,
        }
    }

    pub fn share(&self) -> f64 {
        self.mass / self.feet.len() as f64
    }

    /// Footprint centre now.
    pub fn centre(&self, sim: &DynamicSim) -> Vec3 {
        self.feet.iter().map(|&n| sim.position(n)).sum::<Vec3>() / self.feet.len() as f64
    }

    fn velocity(&self, sim: &DynamicSim) -> Vec3 {
        self.feet.iter().map(|&n| sim.velocity(n)).sum::<Vec3>() / self.feet.len() as f64
    }

    /// Vertical foot forces whose moment about the footprint centre is
    /// `m` (least squares; zero for one foot or feet in a line along m).
    fn couple_for(&self, m: Vec3) -> Vec<f64> {
        // Moment of a vertical force s at (x, z): (-z s, 0, x s).
        let (mut sxx, mut szz, mut sxz) = (0.0, 0.0, 0.0);
        for &(x, z) in &self.offsets {
            sxx += x * x;
            szz += z * z;
            sxz += x * z;
        }
        // Solve [[szz, -sxz], [-sxz, sxx]] [a, b] = [mx, mz] with
        // s_i = -z_i a + x_i b.
        let det = szz * sxx - sxz * sxz;
        if det.abs() < 1e-9 {
            // Feet in a line: carry the component along it only.
            let r2 = sxx + szz;
            if r2 < 1e-12 {
                return vec![0.0; self.offsets.len()];
            }
            return self
                .offsets
                .iter()
                .map(|&(x, z)| (-z * m.x + x * m.z) / r2)
                .collect();
        }
        let a = (sxx * m.x + sxz * m.z) / det;
        let b = (sxz * m.x + szz * m.z) / det;
        self.offsets.iter().map(|&(x, z)| -z * a + x * b).collect()
    }

    /// Updates from the last frame (`ground` is the table acceleration,
    /// `g` gravity) and returns what changed. Call after each solver step.
    pub fn update(
        &mut self,
        sim: &mut DynamicSim,
        built: &[Vec3],
        ground: Vec3,
        g: Vec3,
        dt: f64,
    ) -> Vec<EquipmentEvent> {
        let mut events = Vec::new();
        if self.lost {
            return events;
        }
        let v = self.velocity(sim);
        let a_rel = self.prev_velocity.map_or(Vec3::zeros(), |p| (v - p) / dt);
        self.prev_velocity = Some(v);

        // Inertial force on the body in the table frame, horizontal part.
        let mut h = -(ground + a_rel) * self.mass + self.wind;
        h.y = 0.0;
        let moment = (Vec3::y() * self.height).cross(&h);
        self.couple = self.couple_for(moment);

        // Foot reactions on the body: weight share minus the couple.
        let weight = self.mass * (-g.y).max(0.0);
        let share = weight / self.feet.len() as f64;
        let reactions: Vec<f64> = self.couple.iter().map(|s| share - s).collect();
        if share > 0.0 {
            let lo = reactions.iter().copied().fold(f64::INFINITY, f64::min) / share;
            self.min_reaction = self.min_reaction.min(lo);
        }
        let shear = h.norm();
        let unstable = match self.mount {
            Mount::Fixed => false,
            Mount::Bolted { capacity } if !self.bolts_failed => {
                let pull = reactions.iter().map(|r| -r).fold(0.0, f64::max);
                if pull > capacity || shear / self.feet.len() as f64 > capacity {
                    self.bolts_failed = true;
                    events.push(EquipmentEvent::BoltsFailed);
                }
                false
            }
            Mount::Bolted { .. } => resting_unstable(&reactions, shear, BROKEN_BOLT_FRICTION),
            Mount::Resting { friction } => resting_unstable(&reactions, shear, friction),
        };
        // Only judge once weight is fully on (gravity ramps in first).
        if unstable && share > 0.0 {
            self.unstable += dt;
        } else {
            self.unstable = (self.unstable - 0.5 * dt).max(0.0);
        }

        // Read-outs.
        let du: Vec<Vec3> = self
            .feet
            .iter()
            .zip(built)
            .map(|(&n, b)| sim.position(n) - b)
            .collect();
        let sag = du.iter().map(|d| -d.y).fold(0.0, f64::max);
        let drift = (du.iter().sum::<Vec3>() / du.len() as f64).norm();
        self.peak_sag = self.peak_sag.max(sag);
        self.peak_drift = self.peak_drift.max(drift);
        self.peak_tilt = self.peak_tilt.max(self.tilt(sim, &du));

        if self.unstable >= LOSS_TIME {
            self.lost = true;
            for &n in &self.feet {
                sim.add_point_mass(n, -self.share());
            }
            self.couple.iter_mut().for_each(|c| *c = 0.0);
            events.push(EquipmentEvent::Lost {
                position: self.centre(sim) + Vec3::y() * self.height,
                velocity: v,
            });
        }
        events
    }

    /// Tilt of the footprint from level, rad: a least-squares plane through
    /// the feet's vertical displacements, the slope along a line of feet,
    /// or for one foot the rotation of its node.
    fn tilt(&self, sim: &DynamicSim, du: &[Vec3]) -> f64 {
        if self.feet.len() == 1 {
            let up = sim.orientation(self.feet[0]) * Vec3::y();
            return up.dot(&Vec3::y()).clamp(-1.0, 1.0).acos();
        }
        let (mut sxx, mut szz, mut sxz, mut sxy, mut szy) = (0.0, 0.0, 0.0, 0.0, 0.0);
        for (&(x, z), d) in self.offsets.iter().zip(du) {
            sxx += x * x;
            szz += z * z;
            sxz += x * z;
            sxy += x * d.y;
            szy += z * d.y;
        }
        let det = sxx * szz - sxz * sxz;
        let slope = if det.abs() < 1e-9 {
            let r2 = sxx + szz;
            if r2 < 1e-12 {
                0.0
            } else {
                // dy = k (x, z)·u along the line.
                (sxy * sxy + szy * szy).sqrt() / r2
            }
        } else {
            let a = (szz * sxy - sxz * szy) / det;
            let b = (sxx * szy - sxz * sxy) / det;
            (a * a + b * b).sqrt()
        };
        slope.atan()
    }

    /// Adds this item's overturning couple to the nodal forces.
    pub fn add_forces(&self, forces: &mut [Vec3]) {
        if self.lost {
            return;
        }
        let share = Vec3::new(self.wind.x, 0.0, self.wind.z) / self.feet.len() as f64;
        for (&n, s) in self.feet.iter().zip(&self.couple) {
            forces[n].y += s;
            forces[n] += share;
        }
    }
}

fn resting_unstable(reactions: &[f64], shear: f64, friction: f64) -> bool {
    let lifted = reactions.iter().any(|&r| r < 0.0);
    let normal: f64 = reactions.iter().map(|r| r.max(0.0)).sum();
    lifted || shear > friction * normal
}

#[cfg(test)]
mod tests {
    use super::*;

    fn item(offsets: Vec<(f64, f64)>) -> Equipment {
        let n = offsets.len();
        Equipment {
            feet: (0..n).collect(),
            offsets,
            mass: 1000.0,
            height: 1.0,
            mount: Mount::Fixed,
            bolts_failed: false,
            lost: false,
            unstable: 0.0,
            prev_velocity: None,
            couple: vec![0.0; n],
            wind: Vec3::zeros(),
            peak_tilt: 0.0,
            peak_sag: 0.0,
            peak_drift: 0.0,
            min_reaction: 1.0,
        }
    }

    fn moment_of(e: &Equipment, s: &[f64]) -> Vec3 {
        e.offsets
            .iter()
            .zip(s)
            .map(|(&(x, z), &f)| Vec3::new(x, 0.0, z).cross(&Vec3::new(0.0, f, 0.0)))
            .sum()
    }

    #[test]
    fn couple_has_the_asked_moment_and_no_net_force() {
        // An uneven footprint, centred.
        let e = item(vec![(-1.0, -0.5), (1.0, -0.5), (1.0, 0.5), (-1.0, 0.5)]);
        let m = Vec3::new(300.0, 0.0, -1200.0);
        let s = e.couple_for(m);
        assert!(s.iter().sum::<f64>().abs() < 1e-9);
        assert!((moment_of(&e, &s) - m).norm() < 1e-9);
        // A sideways push at height 1 m along +x loads the +x feet down
        // (negative force on the structure means pushing it down).
        let h = Vec3::new(1000.0, 0.0, 0.0);
        let s = e.couple_for(Vec3::y().cross(&h));
        assert!(s[1] < 0.0 && s[2] < 0.0 && s[0] > 0.0 && s[3] > 0.0);
    }

    #[test]
    fn resting_item_tips_when_a_foot_lifts_and_slides_past_friction() {
        assert!(!resting_unstable(&[100.0, 50.0], 10.0, 0.3));
        assert!(resting_unstable(&[200.0, -1.0], 0.0, 0.3));
        assert!(resting_unstable(&[100.0, 100.0], 61.0, 0.3));
    }
}
