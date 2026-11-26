//! Listening to a simulation: turns a [`DynamicSim`]'s state into the
//! structure [`Command`]s, with no engine in between.
//!
//! Call [`StructureListener::update`] once per frame after the solver has
//! stepped. It follows every member's utilisation and damage, works out the
//! elastic energy each one holds from its axial force and end moments, and
//! reports:
//!
//! * [`Command::Strain`] for the members whose load or damage is changing
//!   most (at most [`MAX_STRAIN_VOICES`]);
//! * [`Command::Damage`] when a member reaches a worse damage state;
//! * [`Command::Break`] for each new break event, with the energy the member
//!   held just before it let go and how much of it was axial.

use breakit_core::{DamageState, DynamicSim, Material, MemberDamage, Section, Structure};

use crate::mixer::MAX_STRAIN_VOICES;
use crate::{Body, BreakInfo, Command, Place, Pos};

/// What the listener remembers about one member between frames.
#[derive(Clone, Copy, Debug, Default)]
struct MemberTrack {
    /// Utilisation with the solver's frame-to-frame jitter smoothed out.
    load: f32,
    damage: f32,
    /// How much its load and damage have been changing lately, per second.
    activity: f32,
    /// Elastic energy it holds, J, and the axial share of it. The peak is
    /// held for a moment so a break still knows what the member carried.
    energy: f32,
    axial: f32,
    state: DamageState,
}

/// Follows one simulation run.
#[derive(Clone, Debug, Default)]
pub struct StructureListener {
    seen_breaks: usize,
    members: Vec<MemberTrack>,
    /// Scratch: (activity, member) of the members worth hearing this frame.
    active: Vec<(f32, usize)>,
}

/// Elastic strain energy in a member, J, from its axial force and end
/// moments (taken as linear between the ends), and the axial share of it:
/// `N²L / 2EA + Σ L (Ma² + Ma·Mb + Mb²) / 6EI`.
pub fn strain_energy(
    material: &Material,
    section: &Section,
    length: f64,
    d: &MemberDamage,
) -> (f32, f32) {
    let e = material.youngs_modulus;
    let axial = d.axial * d.axial * length / (2.0 * e * section.area);
    let mut bending = 0.0;
    for (axis, inertia) in [(0, section.iy), (1, section.iz)] {
        // Solver sign convention: uniform bending has opposite end signs.
        let (a, b) = (d.moment[axis][0], -d.moment[axis][1]);
        bending += length * (a * a + a * b + b * b) / (6.0 * e * inertia);
    }
    let total = axial + bending;
    if !total.is_finite() || total <= 0.0 {
        return (0.0, 0.5);
    }
    (total as f32, (axial / total) as f32)
}

impl StructureListener {
    pub fn new() -> Self {
        Self::default()
    }

    /// Forget the run (a new test starts).
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    /// One frame. `structure` is what `sim` was built from, `dt` the frame
    /// time in seconds, `offset` is added to every position (the world
    /// position of the structure's origin).
    pub fn update(
        &mut self,
        sim: &DynamicSim,
        structure: &Structure,
        dt: f32,
        offset: Pos,
        mut send: impl FnMut(Command),
    ) {
        // The run may add members of its own after the design's; only the
        // ones both sides know are followed.
        let n = structure.members.len().min(sim.damage_all().len());
        let first = self.members.len() != n;
        if first {
            self.members = vec![MemberTrack::default(); n];
            self.seen_breaks = 0;
        }
        let dt = dt.clamp(1e-3, 0.1);
        let point = |i: usize, t: f64| -> Pos {
            let m = &structure.members[i];
            let p = sim.position(m.a).lerp(&sim.position(m.b), t);
            [
                p.x as f32 + offset[0],
                p.y as f32 + offset[1],
                p.z as f32 + offset[2],
            ]
        };
        let rest_length = |i: usize| -> f64 {
            let m = &structure.members[i];
            (structure.nodes[m.a] - structure.nodes[m.b])
                .norm()
                .max(0.05)
        };
        // A member that has lost stiffness (cracked, yielded) rings duller.
        let body = |i: usize, d: &MemberDamage| -> Body {
            let m = &structure.members[i];
            Body::member(&m.material, &m.section, rest_length(i))
                .with_damage(1.0 - d.stiffness as f32)
        };

        // Follow every member; remember the ones that are changing.
        self.active.clear();
        for i in 0..n {
            if sim.is_broken(i) {
                continue;
            }
            let d = sim.damage(i);
            let u = sim.utilization(i) as f32;
            if !u.is_finite() {
                continue;
            }
            let member = &structure.members[i];
            let (energy, axial) =
                strain_energy(&member.material, &member.section, rest_length(i), d);
            let t = &mut self.members[i];
            if energy >= t.energy {
                t.energy = energy;
                t.axial = axial;
            } else {
                t.energy += (energy - t.energy) * (dt / 0.1).min(1.0);
            }
            let damage = d.damage as f32;
            if first {
                t.load = u;
                t.damage = damage;
                t.state = d.state;
                continue;
            }
            let load = t.load + (u - t.load) * (dt / 0.05).min(1.0);
            let change = (load - t.load).abs() + 10.0 * (damage - t.damage).max(0.0);
            t.activity += (change / dt - t.activity) * (dt / 0.2).min(1.0);
            t.load = load;
            t.damage = damage.max(t.damage);
            if t.activity > 0.01 {
                self.active.push((t.activity, i));
            }
        }

        // Strain: the members changing most are heard, each through its own
        // modes. The audio engine listens to how load and damage move.
        self.active.sort_by(|a, b| b.0.total_cmp(&a.0));
        for &(_, i) in self.active.iter().take(MAX_STRAIN_VOICES) {
            let t = &self.members[i];
            send(Command::Strain {
                id: i as u32,
                body: body(i, sim.damage(i)),
                utilization: t.load,
                damage: t.damage,
                place: Place::at(point(i, 0.5)),
            });
        }

        // Damage onsets (breaks are reported separately below).
        for (i, t) in self.members.iter_mut().enumerate().take(n) {
            let d = sim.damage(i);
            if d.state > t.state {
                t.state = d.state;
                if d.state != DamageState::Broken {
                    send(Command::Damage {
                        body: body(i, d),
                        state: d.state,
                        energy: t.energy,
                        place: Place::at(point(i, 0.5)),
                    });
                }
            }
        }

        // Breaks: what the member was, where it failed, and the elastic
        // energy it was holding when it let go.
        let events = sim.break_events();
        for e in events.iter().skip(self.seen_breaks) {
            if e.member >= n {
                continue;
            }
            let t = &self.members[e.member];
            send(Command::Break {
                info: BreakInfo {
                    body: body(e.member, sim.damage(e.member)),
                    cause: e.cause,
                    at: e.position as f32,
                    energy: t.energy,
                    axial: t.axial,
                },
                place: Place::at(point(e.member, e.position)),
            });
        }
        self.seen_breaks = events.len();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use breakit_core::MaterialKind;

    #[test]
    fn strain_energy_matches_hand_calculation() {
        // A 2 m steel tube under 100 kN tension: N²L / 2EA.
        let (material, section) = (Material::steel(), Section::tube(0.05, 0.005));
        let mut d = MemberDamage::new(MaterialKind::Steel);
        d.axial = 100e3;
        let (u, share) = strain_energy(&material, &section, 2.0, &d);
        let exact = 100e3f64.powi(2) * 2.0 / (2.0 * 210e9 * section.area);
        assert!((u as f64 / exact - 1.0).abs() < 1e-3, "{u} vs {exact}");
        assert!((share - 1.0).abs() < 1e-6);
        // Uniform bending M (end moments M and -M): M²L / 2EI.
        d.axial = 0.0;
        d.moment[1] = [5e3, -5e3];
        let (u, share) = strain_energy(&material, &section, 2.0, &d);
        let exact = 5e3f64.powi(2) * 2.0 / (2.0 * 210e9 * section.iz);
        assert!((u as f64 / exact - 1.0).abs() < 1e-3, "{u} vs {exact}");
        assert!(share < 1e-6);
    }
}
