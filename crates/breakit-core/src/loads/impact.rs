//! Rigid impactors: a runaway cart into a column, a weight dropped from the
//! crane.
//!
//! The impactor is a rigid sphere (point mass with a radius) released at the
//! window start with a position and velocity. It falls under gravity unless it
//! is `guided`: a cart on a track moves only along its guide line, and the
//! guide takes every force across it. Contact is a padded penalty contact
//! against the members: each member is a capsule around its chord with half
//! its section depth as radius, the overlap is `δ = r + d/2 − |p − x|` with
//! `x` the closest point on the chord, and the force `F = k δ + c δ̇` acts
//! along the separation normal, compressive only. It is split between the
//! member's end nodes by where `x` lies; a contact at a joint counts once.
//!
//! Each frame, the impactor is integrated over `[t, t + dt]` with `substeps`
//! semi-implicit Euler steps, the nodes moving linearly with their velocity
//! at the frame start. The impulse each node received is applied as the
//! frame-average force `J / dt`, so the momentum the impactor loses is exactly
//! the momentum the structure gains (along the guide for a guided cart).
//!
//! Pad defaults ([`Impactor::pad`]): against a rigid target the contact is a
//! half period of the mass on the pad spring, `T = π √(m/k)`, with peak force
//! `v √(k m)`. The structure solver sees the contact only as frame-average
//! forces, so the contact must last several frames: `k` is chosen for
//! `T = 50 ms` (3 frames at 60 Hz), `k = m (π / T)²`. The damping is 10% of
//! critical, `c = 2 ζ √(k m)` (restitution about 0.7 off a rigid target),
//! which also takes the ring out of the pad. A real steel-on-steel impact is
//! far shorter and harsher; the pad stands for the crumple zone of the cart
//! or the timber packing under a drop weight. A softer structure lengthens
//! the contact further.
//!
//! The load factor `scale` multiplies the impactor mass (same pad).

use super::{View, G};
use crate::{Structure, Vec3};

/// A member the impactor may touch this frame, with the impulse it took.
struct Cand {
    member: usize,
    a: usize,
    b: usize,
    xa: Vec3,
    xb: Vec3,
    ua: Vec3,
    ub: Vec3,
    half: f64,
    imp_a: Vec3,
    imp_b: Vec3,
}

/// Closest point of segment a-b to `p`: (fraction along, distance).
fn closest_on_segment(p: Vec3, a: Vec3, b: Vec3) -> (f64, f64) {
    let ab = b - a;
    let l2 = ab.norm_squared();
    let s = if l2 > 0.0 {
        ((p - a).dot(&ab) / l2).clamp(0.0, 1.0)
    } else {
        0.0
    };
    (s, (p - (a + ab * s)).norm())
}

/// Default contact duration against a rigid target, s.
pub const DEFAULT_CONTACT_TIME: f64 = 0.05;
/// Default pad damping, fraction of critical.
pub const DEFAULT_PAD_DAMPING_RATIO: f64 = 0.1;
/// Contact events kept for the rig.
const MAX_EVENTS: usize = 64;

#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Impactor {
    pub name: String,
    /// kg.
    pub mass: f64,
    /// Contact radius, m.
    pub radius: f64,
    /// Centre at release (window start), m.
    pub position: Vec3,
    /// Velocity at release, m/s.
    pub velocity: Vec3,
    /// Falls under gravity ([`G`], independent of the solver's gravity).
    pub gravity: bool,
    /// Motion constrained to the line through `position` along this direction.
    pub guided: Option<Vec3>,
    /// Pad stiffness, N/m.
    pub pad_stiffness: f64,
    /// Pad damping, N·s/m.
    pub pad_damping: f64,
    /// Integration substeps per frame.
    pub substeps: usize,
    #[cfg_attr(feature = "serde", serde(skip))]
    state: ImpactState,
}

impl Default for Impactor {
    fn default() -> Self {
        Self::new("Impactor", 100.0, 0.1, Vec3::zeros(), Vec3::zeros())
    }
}

/// One contact between the impactor and a node.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct ContactEvent {
    /// Time since the case start when the contact began, s.
    pub time: f64,
    pub node: usize,
    /// Approach speed along the contact normal at first touch, m/s.
    pub impact_speed: f64,
    /// N.
    pub peak_force: f64,
    /// N·s.
    pub impulse: f64,
    /// s (so far, while active).
    pub duration: f64,
    pub active: bool,
}

/// State of the impactor for the renderer and sound.
#[derive(Clone, Debug, Default)]
pub struct ImpactorRig {
    pub name: String,
    pub position: Vec3,
    pub velocity: Vec3,
    pub radius: f64,
    pub released: bool,
    pub in_contact: bool,
    /// Frame-average contact force on the impactor, N.
    pub contact_force: Vec3,
    /// Largest contact force so far, N.
    pub peak_force: f64,
    /// Contact events, oldest first (the last [`MAX_EVENTS`]).
    pub events: Vec<ContactEvent>,
}

#[derive(Clone, Debug, Default)]
struct ImpactState {
    released: bool,
    p: Vec3,
    v: Vec3,
    last_t: f64,
    /// Open contacts: (node, index into `events`).
    open: Vec<(usize, usize)>,
    events: Vec<ContactEvent>,
    peak_force: f64,
    contact_force: Vec3,
    drop_height: Option<f64>,
}

impl Impactor {
    /// Free (ungravitating) impactor with the default pad.
    pub fn new(
        name: impl Into<String>,
        mass: f64,
        radius: f64,
        position: Vec3,
        velocity: Vec3,
    ) -> Self {
        let (k, c) = Self::pad(mass, DEFAULT_CONTACT_TIME, DEFAULT_PAD_DAMPING_RATIO);
        Self {
            name: name.into(),
            mass,
            radius,
            position,
            velocity,
            gravity: false,
            guided: None,
            pad_stiffness: k,
            pad_damping: c,
            substeps: 32,
            state: ImpactState::default(),
        }
    }
    /// A weight released at rest from `position`, falling under gravity.
    pub fn dropped(name: impl Into<String>, mass: f64, radius: f64, position: Vec3) -> Self {
        Self {
            gravity: true,
            ..Self::new(name, mass, radius, position, Vec3::zeros())
        }
    }
    /// Something that came loose and falls from where it was, keeping its
    /// velocity: lost equipment, a dropped load. Add it with
    /// [`super::LoadSet::add_case`] and a window starting now.
    pub fn released(
        name: impl Into<String>,
        mass: f64,
        radius: f64,
        position: Vec3,
        velocity: Vec3,
    ) -> Self {
        Self {
            gravity: true,
            ..Self::new(name, mass, radius, position, velocity)
        }
    }
    /// A cart on a track along `velocity`.
    pub fn cart(
        name: impl Into<String>,
        mass: f64,
        radius: f64,
        position: Vec3,
        velocity: Vec3,
    ) -> Self {
        Self {
            guided: Some(if velocity.norm() > 0.0 {
                velocity.normalize()
            } else {
                Vec3::x()
            }),
            ..Self::new(name, mass, radius, position, velocity)
        }
    }
    /// Pad stiffness and damping (k, c) for a contact of `duration` seconds
    /// against a rigid target with `damping_ratio` of critical damping.
    pub fn pad(mass: f64, duration: f64, damping_ratio: f64) -> (f64, f64) {
        let w = std::f64::consts::PI / duration;
        let k = mass * w * w;
        (k, 2.0 * damping_ratio * (k * mass).sqrt())
    }
    /// Without pad damping.
    pub fn undamped(mut self) -> Self {
        self.pad_damping = 0.0;
        self
    }

    /// Find the drop height onto the structure for [`Self::energy`].
    pub fn prepare(&mut self, s: &Structure) {
        let r = self.radius;
        let p = self.position;
        let top = s
            .nodes
            .iter()
            .filter_map(|x| {
                let d = ((x.x - p.x).powi(2) + (x.z - p.z).powi(2)).sqrt();
                (d < r && x.y < p.y).then(|| x.y + (r * r - d * d).sqrt())
            })
            .fold(f64::NEG_INFINITY, f64::max);
        self.state = ImpactState {
            drop_height: Some(if top.is_finite() { p.y - top } else { p.y - r }.max(0.0)),
            ..Default::default()
        };
    }

    /// Kinetic energy at impact, J (unit load factor): ½ m v² at release,
    /// plus m g h for a free falling impactor, with h the fall straight down
    /// to the highest node under it (or the table top). The horizontal drift
    /// of a thrown weight is ignored. A guided cart gains the work of gravity
    /// along its guide over the same height only if the guide slopes, which
    /// is not counted.
    pub fn energy(&self) -> f64 {
        let v = match self.guided {
            Some(d) => self.velocity.dot(&d.normalize()),
            None => self.velocity.norm(),
        };
        let mut e = 0.5 * self.mass * v * v;
        if self.gravity && self.guided.is_none() {
            let h = self
                .state
                .drop_height
                .unwrap_or((self.position.y - self.radius).max(0.0));
            e += self.mass * G * h;
        }
        e
    }

    fn guide(&self) -> Option<Vec3> {
        self.guided
            .filter(|d| d.norm() > 1e-12)
            .map(|d| d.normalize())
    }

    fn constrain(&self, f: Vec3) -> Vec3 {
        match self.guide() {
            Some(d) => d * f.dot(&d),
            None => f,
        }
    }

    pub fn apply(&mut self, v: &View, t: f64, scale: f64, dt: f64, out: &mut [Vec3]) {
        if t < 0.0 || dt <= 0.0 {
            return;
        }
        if !self.state.released || t < self.state.last_t - 1e-9 {
            let drop_height = self.state.drop_height;
            self.state = ImpactState {
                released: true,
                p: self.position,
                v: self.constrain(self.velocity),
                drop_height,
                ..Default::default()
            };
        }
        self.state.last_t = t;
        let m = self.mass * scale;
        if m <= 0.0 {
            return;
        }
        let (k, c, r) = (self.pad_stiffness, self.pad_damping, self.radius);
        let gravity = if self.gravity {
            self.constrain(Vec3::new(0.0, -G, 0.0))
        } else {
            Vec3::zeros()
        };
        let n = self.substeps.max(1);
        let h = dt / n as f64;

        // Members the impactor can reach this frame: contact is against the
        // member's surface (chord plus half its section depth), split to its
        // end nodes by where along the member it touches.
        let st = &self.state;
        let reach = r + dt * (st.v.norm() + gravity.norm() * dt) + 1e-6;
        let mut cand: Vec<Cand> = (0..v.structure.members.len())
            .filter_map(|mi| {
                if v.sim.is_broken(mi) {
                    return None;
                }
                let mem = &v.structure.members[mi];
                let (xa, xb) = (v.sim.position(mem.a), v.sim.position(mem.b));
                let (ua, ub) = (v.sim.velocity(mem.a), v.sim.velocity(mem.b));
                let g = &v.geometry.members[mi];
                let half = 0.5 * g.depth_y.max(g.depth_z);
                let (_, d) = closest_on_segment(st.p, xa, xb);
                let slack = reach + half + dt * ua.norm().max(ub.norm());
                (d < slack).then_some(Cand {
                    member: mi,
                    a: mem.a,
                    b: mem.b,
                    xa,
                    xb,
                    ua,
                    ub,
                    half,
                    imp_a: Vec3::zeros(),
                    imp_b: Vec3::zeros(),
                })
            })
            .collect();

        let mut p = st.p;
        let mut vel = st.v;
        let mut open = std::mem::take(&mut self.state.open);
        let mut events = std::mem::take(&mut self.state.events);
        let mut peak = self.state.peak_force;
        let mut total = Vec3::zeros();
        let mut end_contacts: Vec<usize> = Vec::new();
        for i in 0..n {
            let tau = i as f64 * h;
            let mut f_imp = Vec3::zeros();
            end_contacts.clear();
            for cd in &mut cand {
                let (xa, xb) = (cd.xa + cd.ua * tau, cd.xb + cd.ub * tau);
                let (sp, dist) = closest_on_segment(p, xa, xb);
                let delta = r + cd.half - dist;
                let idx = open.iter().position(|(mi, _)| *mi == cd.member);
                // A contact at a member end is a contact with the node: count
                // it once however many members meet there.
                let end_node = if sp <= 0.0 {
                    Some(cd.a)
                } else if sp >= 1.0 {
                    Some(cd.b)
                } else {
                    None
                };
                let dup = end_node.is_some_and(|nd| end_contacts.contains(&nd));
                if delta <= 0.0 || dist < 1e-12 || dup {
                    if let Some(o) = idx {
                        events[open[o].1].active = false;
                        open.swap_remove(o);
                    }
                    continue;
                }
                if let Some(nd) = end_node {
                    end_contacts.push(nd);
                }
                let x = xa + (xb - xa) * sp;
                let u = cd.ua + (cd.ub - cd.ua) * sp;
                let nrm = (p - x) / dist;
                let delta_dot = -(vel - u).dot(&nrm);
                let f = (k * delta + c * delta_dot).max(0.0);
                let fv = nrm * f;
                f_imp += fv;
                cd.imp_a -= fv * (h * (1.0 - sp));
                cd.imp_b -= fv * (h * sp);
                let e = match idx {
                    Some(o) => open[o].1,
                    None => {
                        events.push(ContactEvent {
                            time: t + tau,
                            node: if sp < 0.5 { cd.a } else { cd.b },
                            impact_speed: delta_dot.max(0.0),
                            active: true,
                            ..Default::default()
                        });
                        open.push((cd.member, events.len() - 1));
                        events.len() - 1
                    }
                };
                let ev = &mut events[e];
                ev.peak_force = ev.peak_force.max(f);
                ev.impulse += f * h;
                ev.duration = t + tau + h - ev.time;
                peak = peak.max(f);
            }
            total += f_imp * (h / dt);
            vel += h * (self.constrain(f_imp) / m + gravity);
            p += h * vel;
            if self.gravity && self.guided.is_none() && p.y < r {
                // On the table: stop the fall, let it slide to rest.
                p.y = r;
                vel.y = vel.y.max(0.0);
                let fr = (-5.0 * h).exp();
                vel.x *= fr;
                vel.z *= fr;
            }
        }
        for cd in &cand {
            out[cd.a] += cd.imp_a / dt;
            out[cd.b] += cd.imp_b / dt;
        }
        // Drop old closed events, keeping indices of open ones valid.
        if events.len() > MAX_EVENTS {
            let cut = events.len() - MAX_EVENTS;
            let min_open = open.iter().map(|o| o.1).min().unwrap_or(usize::MAX);
            let cut = cut.min(min_open);
            events.drain(..cut);
            for o in &mut open {
                o.1 -= cut;
            }
        }
        let st = &mut self.state;
        st.p = p;
        st.v = vel;
        st.open = open;
        st.events = events;
        st.peak_force = peak;
        st.contact_force = total;
    }

    /// Current velocity, m/s (the release velocity before release).
    pub fn velocity_now(&self) -> Vec3 {
        if self.state.released {
            self.state.v
        } else {
            self.velocity
        }
    }

    pub fn rig(&self) -> ImpactorRig {
        let st = &self.state;
        ImpactorRig {
            name: self.name.clone(),
            position: if st.released { st.p } else { self.position },
            velocity: self.velocity_now(),
            radius: self.radius,
            released: st.released,
            in_contact: !st.open.is_empty(),
            contact_force: st.contact_force,
            peak_force: st.peak_force,
            events: st.events.clone(),
        }
    }
}
