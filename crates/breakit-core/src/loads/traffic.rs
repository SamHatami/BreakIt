//! Moving vehicle loads: heavy model vehicles driving along a lane.
//!
//! A lane is a line in space ([`LaneLine`]). When the case is prepared, the
//! structure nodes within the capture radius of that line (perpendicular
//! distance, projection inside the segment) become the lane, ordered by arc
//! length along the line; nodes at the same arc position are merged into one
//! station. Off either end of the lane the vehicle is on the approach or exit
//! ramp: a level continuation at the rest height of the end station, which
//! carries its wheels without loading the structure.
//!
//! Each axle at arc position `s` between stations `k` and `k + 1` loads those
//! two stations' nodes, split linearly (the moving-force model). The pair must
//! still be joined by an unbroken member; over a gap the vehicle loses support
//! and is flagged `fallen`, after which it no longer loads the structure.
//!
//! Every axle carries its share of the vehicle as a sprung mass on a
//! spring-damper riding on the deck (a quarter-car model per axle). The wheel
//! contact follows the deflected deck under the axle, so a vehicle bounces on
//! a lively deck, dives when it brakes and can lift a wheel off. The force on
//! the deck is the tyre force `k e + c ė` (never negative), averaged over the
//! frame from several internal substeps. An axle without suspension
//! ([`Axle::rigid`]) is the plain moving static load `m g`.
//!
//! Braking at deceleration `a` pushes the deck forward (along the direction of
//! travel) with `M a`, split over the axles on the structure in proportion to
//! their static loads. With a centre-of-gravity height the braking moment
//! `M a h` also moves load to the front axles (dive).
//!
//! Load factor (`scale`): the vehicle dynamics are integrated at unit scale and
//! every force is multiplied by `scale`, as if the ballast were scaled with the
//! springs (same bounce frequency). The window `ramp` is the fraction of the
//! vehicle's weight resting on its wheels: a parked vehicle at the start of a
//! case is lowered onto the deck by the crane instead of being dropped, and is
//! lifted off at the end of a finite window. Vehicles driving on from the
//! approach ramp after the ramp-in never see it.
//!
//! The frame length is not passed in: it is taken from the time between
//! successive calls (1/60 s before the second call). The suspension is
//! integrated forward over that estimate from the current deck state, with
//! the deck contact extrapolated linearly by its velocity.

use super::{View, G};
use crate::{Structure, Vec3};

/// Default bounce frequency of [`Axle::new`], Hz (heavy trucks: 2–3 Hz).
pub const DEFAULT_BOUNCE_HZ: f64 = 2.5;
/// Default suspension damping of [`Axle::new`], fraction of critical.
pub const DEFAULT_DAMPING_RATIO: f64 = 0.1;
/// Stations closer than this along the lane are merged, m.
pub const MERGE_DISTANCE: f64 = 1e-3;

/// Where a lane runs: the vehicles drive from `start` to `end`.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct LaneLine {
    pub start: Vec3,
    pub end: Vec3,
    /// Nodes within this distance of the line carry the lane, m.
    pub capture_radius: f64,
}

impl LaneLine {
    pub fn new(start: Vec3, end: Vec3, capture_radius: f64) -> Self {
        Self {
            start,
            end,
            capture_radius,
        }
    }
    /// Unit direction of travel (x if the line is degenerate).
    pub fn direction(&self) -> Vec3 {
        let d = self.end - self.start;
        let l = d.norm();
        if l > 1e-12 {
            d / l
        } else {
            Vec3::x()
        }
    }
    pub fn length(&self) -> f64 {
        (self.end - self.start).norm()
    }
}

/// Spring-damper suspension of one axle.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Spring {
    /// N/m.
    pub stiffness: f64,
    /// N·s/m.
    pub damping: f64,
}

impl Spring {
    /// Spring giving sprung mass `mass` (kg) a bounce at `frequency` (Hz)
    /// with `damping_ratio` of critical damping.
    pub fn tuned(mass: f64, frequency: f64, damping_ratio: f64) -> Self {
        let w = 2.0 * std::f64::consts::PI * frequency;
        let stiffness = mass * w * w;
        Self {
            stiffness,
            damping: 2.0 * damping_ratio * (stiffness * mass).sqrt(),
        }
    }
}

/// One axle (or axle group) of a vehicle.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Axle {
    /// Distance behind the front axle, m (0 for the front axle).
    pub offset: f64,
    /// Static axle load as a mass, kg: the axle's share of the vehicle,
    /// which is also its sprung mass.
    pub mass: f64,
    /// `None`: rigid, the axle is a moving static force.
    pub suspension: Option<Spring>,
}

impl Axle {
    /// Axle with the default suspension (2.5 Hz bounce, 10% damping).
    pub fn new(offset: f64, mass: f64) -> Self {
        Self {
            offset,
            mass,
            suspension: Some(Spring::tuned(
                mass,
                DEFAULT_BOUNCE_HZ,
                DEFAULT_DAMPING_RATIO,
            )),
        }
    }
    /// Axle without suspension.
    pub fn rigid(offset: f64, mass: f64) -> Self {
        Self {
            offset,
            mass,
            suspension: None,
        }
    }
}

/// Braking to a stop.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Brake {
    /// Arc position of the front axle along the lane line where braking starts, m.
    pub at: f64,
    /// Deceleration, m/s² (positive).
    pub deceleration: f64,
}

/// Speed profile along the lane.
#[derive(Clone, Debug, PartialEq, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Drive {
    /// Cruise speed, m/s (0: parked).
    pub speed: f64,
    /// Arc position of the front axle along the lane line at the case start,
    /// m. Negative or before the first lane node: on the approach ramp.
    pub start: f64,
    pub brake: Option<Brake>,
}

/// Where a vehicle is along its lane at one time.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Kinematics {
    /// Arc position of the front axle, m.
    pub s: f64,
    /// m/s.
    pub speed: f64,
    /// Deceleration, m/s² (positive while braking).
    pub deceleration: f64,
    pub braking: bool,
}

impl Drive {
    pub fn parked(at: f64) -> Self {
        Self {
            speed: 0.0,
            start: at,
            brake: None,
        }
    }
    pub fn cruise(speed: f64, start: f64) -> Self {
        Self {
            speed,
            start,
            brake: None,
        }
    }
    pub fn braking(mut self, at: f64, deceleration: f64) -> Self {
        self.brake = Some(Brake { at, deceleration });
        self
    }

    /// Position and speed at time `t` since the case start (closed form).
    pub fn at(&self, t: f64) -> Kinematics {
        let t = t.max(0.0);
        let (v, s0) = (self.speed.max(0.0), self.start);
        let cruise = Kinematics {
            s: s0 + v * t,
            speed: v,
            deceleration: 0.0,
            braking: false,
        };
        let Some(b) = &self.brake else {
            return cruise;
        };
        let a = b.deceleration;
        if v <= 0.0 || a <= 0.0 {
            return cruise;
        }
        let tb = ((b.at - s0) / v).max(0.0);
        if t < tb {
            return cruise;
        }
        let sb = s0 + v * tb;
        let tau = t - tb;
        let stop = v / a;
        if tau < stop {
            Kinematics {
                s: sb + v * tau - 0.5 * a * tau * tau,
                speed: v - a * tau,
                deceleration: a,
                braking: true,
            }
        } else {
            Kinematics {
                s: sb + v * v / (2.0 * a),
                speed: 0.0,
                deceleration: 0.0,
                braking: false,
            }
        }
    }
}

/// A vehicle: a dump truck, coil hauler or mobile crane model.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Vehicle {
    pub name: String,
    /// Index into [`Traffic::lanes`].
    pub lane: usize,
    /// Front axle first (offset 0).
    pub axles: Vec<Axle>,
    /// Height of the centre of gravity above the wheel contacts, m, for the
    /// load transfer to the front axles while braking (0: none).
    pub cg_height: f64,
    pub drive: Drive,
    #[cfg_attr(feature = "serde", serde(skip))]
    state: VehicleState,
}

impl Vehicle {
    pub fn new(name: impl Into<String>, lane: usize, axles: Vec<Axle>, drive: Drive) -> Self {
        Self {
            name: name.into(),
            lane,
            axles,
            cg_height: 0.0,
            drive,
            state: VehicleState::default(),
        }
    }
    pub fn with_cg_height(mut self, h: f64) -> Self {
        self.cg_height = h;
        self
    }
    /// Total mass, kg.
    pub fn mass(&self) -> f64 {
        self.axles.iter().map(|a| a.mass).sum()
    }
    /// Weight, N.
    pub fn weight(&self) -> f64 {
        self.mass() * G
    }
    /// Extra axle loads (N, + = more load) from braking at deceleration `a`
    /// for a vehicle of unit load factor: the moment `M a h` taken by the
    /// axles about the centre of the axle loads.
    fn transfer(&self, a: f64) -> Vec<f64> {
        let m = self.mass();
        let n = self.axles.len();
        if n < 2 || m <= 0.0 || self.cg_height == 0.0 || a == 0.0 {
            return vec![0.0; n];
        }
        // Forward coordinate of each axle and of the load centre.
        let xbar = self.axles.iter().map(|x| -x.offset * x.mass).sum::<f64>() / m;
        let d: Vec<f64> = self.axles.iter().map(|x| -x.offset - xbar).collect();
        let sum2: f64 = d.iter().map(|d| d * d).sum();
        if sum2 <= 1e-12 {
            return vec![0.0; n];
        }
        let moment = m * a * self.cg_height;
        d.iter().map(|d| moment * d / sum2).collect()
    }
}

#[derive(Clone, Debug, Default)]
struct AxleState {
    /// Sprung mass height and vertical velocity.
    y: f64,
    vy: f64,
}

#[derive(Clone, Debug, Default)]
struct VehicleState {
    init: bool,
    axles: Vec<AxleState>,
    fallen: Option<(f64, Vec<Vec3>)>,
    rig: Option<VehicleRig>,
}

/// One merged lane station: the node that takes the load, others at the same
/// arc position (for connectivity), and its arc position.
#[derive(Clone, Debug)]
pub struct Station {
    pub s: f64,
    pub node: usize,
    pub others: Vec<usize>,
    pub rest: Vec3,
}

/// A lane resolved against the structure.
#[derive(Clone, Debug, Default)]
pub struct Lane {
    pub dir: Vec3,
    pub start: Vec3,
    pub stations: Vec<Station>,
    /// Members joining station `k` to `k + 1`.
    pub segments: Vec<Vec<usize>>,
}

/// Where a wheel is.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Place {
    /// On a ramp (no structure) at this rest point.
    Ground(Vec3),
    /// Between stations `k` and `k + 1`, fraction `l` of the way.
    Deck(usize, f64),
    /// Over a gap between stations `k` and `k + 1`.
    Gap,
}

impl Lane {
    fn resolve(line: &LaneLine, s: &Structure) -> Self {
        let dir = line.direction();
        let len = line.length();
        let r = line.capture_radius.max(0.0);
        let mut found: Vec<(f64, f64, usize)> = s
            .nodes
            .iter()
            .enumerate()
            .filter_map(|(i, p)| {
                let rel = p - line.start;
                let t = rel.dot(&dir);
                let perp = (rel - dir * t).norm();
                (t >= -1e-9 && t <= len + 1e-9 && perp <= r).then_some((t, perp, i))
            })
            .collect();
        found.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.2.cmp(&b.2)));
        let mut groups: Vec<Vec<(f64, f64, usize)>> = Vec::new();
        for f in found {
            match groups.last_mut() {
                Some(g) if f.0 - g[0].0 < MERGE_DISTANCE => g.push(f),
                _ => groups.push(vec![f]),
            }
        }
        let stations: Vec<Station> = groups
            .into_iter()
            .map(|mut g| {
                g.sort_by(|a, b| a.1.total_cmp(&b.1).then(a.2.cmp(&b.2)));
                let node = g[0].2;
                Station {
                    s: g[0].0,
                    node,
                    others: g[1..].iter().map(|x| x.2).collect(),
                    rest: s.nodes[node],
                }
            })
            .collect();
        let segments = stations
            .windows(2)
            .map(|w| {
                let ga: Vec<usize> = std::iter::once(w[0].node)
                    .chain(w[0].others.iter().copied())
                    .collect();
                let gb: Vec<usize> = std::iter::once(w[1].node)
                    .chain(w[1].others.iter().copied())
                    .collect();
                s.members
                    .iter()
                    .enumerate()
                    .filter(|(_, m)| {
                        (ga.contains(&m.a) && gb.contains(&m.b))
                            || (ga.contains(&m.b) && gb.contains(&m.a))
                    })
                    .map(|(i, _)| i)
                    .collect()
            })
            .collect();
        Self {
            dir,
            start: line.start,
            stations,
            segments,
        }
    }

    fn locate(&self, s: f64, broken: &dyn Fn(usize) -> bool) -> Place {
        let st = &self.stations;
        if st.len() < 2 {
            return Place::Ground(self.start + self.dir * s);
        }
        let (first, last) = (&st[0], &st[st.len() - 1]);
        if s <= first.s {
            return Place::Ground(first.rest + self.dir * (s - first.s));
        }
        if s >= last.s {
            return Place::Ground(last.rest + self.dir * (s - last.s));
        }
        let k = st
            .partition_point(|x| x.s <= s)
            .saturating_sub(1)
            .min(st.len() - 2);
        if self.segments[k].iter().all(|&m| broken(m)) {
            return Place::Gap;
        }
        let l = (s - st[k].s) / (st[k + 1].s - st[k].s);
        Place::Deck(k, l.clamp(0.0, 1.0))
    }
}

/// Everything about one wheel (axle) for drawing and sound.
#[derive(Clone, Debug, Default)]
pub struct WheelRig {
    /// Contact point, world, m.
    pub contact: Vec3,
    /// Frame-average tyre force, N (already scaled).
    pub force: f64,
    /// Body displacement above the axle's static ride height, m.
    pub bounce: f64,
    pub on_structure: bool,
    /// The tyre left the deck during the frame.
    pub lifted: bool,
}

/// State of one vehicle for the renderer and sound.
#[derive(Clone, Debug, Default)]
pub struct VehicleRig {
    pub name: String,
    /// Mean of the wheel contacts, world, m.
    pub position: Vec3,
    /// Unit vector from the rear to the front wheel contact (pitch included).
    pub forward: Vec3,
    /// Yaw about +Y, rad: 0 along +X, π/2 along −Z.
    pub heading: f64,
    /// m/s.
    pub speed: f64,
    /// Front axle first.
    pub wheels: Vec<WheelRig>,
    /// Mean body bounce over the axles, m.
    pub bounce: f64,
    pub braking: bool,
    pub fallen: bool,
}

/// Vehicles driving along lanes.
#[derive(Clone, Debug, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Traffic {
    pub lanes: Vec<LaneLine>,
    pub vehicles: Vec<Vehicle>,
    #[cfg_attr(feature = "serde", serde(skip))]
    runtime: TrafficState,
}

#[derive(Clone, Debug, Default)]
struct TrafficState {
    lanes: Vec<Lane>,
    last_t: Option<f64>,
    dt: f64,
}

impl Traffic {
    pub fn new(lanes: Vec<LaneLine>, vehicles: Vec<Vehicle>) -> Self {
        Self {
            lanes,
            vehicles,
            runtime: TrafficState::default(),
        }
    }

    /// Resolve the lanes against the structure and reset the vehicles.
    pub fn prepare(&mut self, s: &Structure) {
        self.runtime = TrafficState {
            lanes: self.lanes.iter().map(|l| Lane::resolve(l, s)).collect(),
            ..Default::default()
        };
        for v in &mut self.vehicles {
            v.state = VehicleState::default();
        }
    }

    /// The resolved lane `i` (after [`Self::prepare`]).
    pub fn lane(&self, i: usize) -> Option<&Lane> {
        self.runtime.lanes.get(i)
    }

    /// Nodes carrying lane `i`, in driving order (one per station).
    pub fn lane_nodes(&self, i: usize) -> Vec<usize> {
        self.lane(i)
            .map(|l| l.stations.iter().map(|s| s.node).collect())
            .unwrap_or_default()
    }

    /// Sum of the vehicle weights, N.
    pub fn total_load(&self) -> f64 {
        self.vehicles.iter().map(|v| v.weight()).sum()
    }

    pub fn apply(&mut self, v: &View, t: f64, ramp: f64, scale: f64, out: &mut [Vec3]) {
        if t < 0.0 {
            return;
        }
        if self.runtime.lanes.len() != self.lanes.len() {
            self.prepare(v.structure);
        }
        let rt = &mut self.runtime;
        match rt.last_t {
            Some(lt) if t < lt - 1e-9 => {
                // Restarted: start the vehicles over.
                for veh in &mut self.vehicles {
                    veh.state = VehicleState::default();
                }
            }
            Some(lt) if t > lt + 1e-9 => rt.dt = t - lt,
            _ => {}
        }
        rt.last_t = Some(t);
        let dt = if rt.dt > 0.0 { rt.dt } else { 1.0 / 60.0 };
        let fallback = Lane::default();
        for veh in &mut self.vehicles {
            let lane = rt.lanes.get(veh.lane).unwrap_or(&fallback);
            step_vehicle(veh, lane, v, t, dt, ramp, scale, out);
        }
    }

    pub fn rig(&self, t: f64) -> Vec<VehicleRig> {
        let fallback = Lane::default();
        self.vehicles
            .iter()
            .map(|veh| {
                let mut r = match &veh.state.rig {
                    Some(r) => r.clone(),
                    None => {
                        let lane = self.runtime.lanes.get(veh.lane).unwrap_or(&fallback);
                        rest_rig(veh, lane, t)
                    }
                };
                if let Some((tf, contacts)) = &veh.state.fallen {
                    let drop = 0.5 * G * (t - tf).max(0.0).powi(2);
                    for (w, c) in r.wheels.iter_mut().zip(contacts) {
                        w.contact = *c;
                        w.contact.y = (c.y - drop).max(0.0).min(c.y);
                    }
                    finish_rig(&mut r, lane_dir_of(self, veh));
                }
                r
            })
            .collect()
    }
}

fn lane_dir_of(t: &Traffic, v: &Vehicle) -> Vec3 {
    t.lanes
        .get(v.lane)
        .map(|l| l.direction())
        .unwrap_or(Vec3::x())
}

/// Fill the derived fields (position, forward, heading, bounce) from wheels.
fn finish_rig(r: &mut VehicleRig, dir: Vec3) {
    let n = r.wheels.len();
    if n == 0 {
        r.forward = dir;
    } else {
        r.position = r.wheels.iter().map(|w| w.contact).sum::<Vec3>() / n as f64;
        r.bounce = r.wheels.iter().map(|w| w.bounce).sum::<f64>() / n as f64;
        let f = r.wheels[0].contact - r.wheels[n - 1].contact;
        r.forward = if n > 1 && f.norm() > 1e-9 {
            f.normalize()
        } else {
            dir
        };
    }
    r.heading = (-r.forward.z).atan2(r.forward.x);
}

/// Rig before the first frame: wheels on the rest geometry.
fn rest_rig(veh: &Vehicle, lane: &Lane, t: f64) -> VehicleRig {
    let k = veh.drive.at(t);
    let wheels = veh
        .axles
        .iter()
        .map(|a| {
            let s = k.s - a.offset;
            let (contact, on) = match lane.locate(s, &|_| false) {
                Place::Ground(p) => (p, false),
                Place::Deck(i, l) => (
                    lane.stations[i].rest.lerp(&lane.stations[i + 1].rest, l),
                    true,
                ),
                Place::Gap => (lane.start + lane.dir * s, false),
            };
            WheelRig {
                contact,
                on_structure: on,
                ..Default::default()
            }
        })
        .collect();
    let mut r = VehicleRig {
        name: veh.name.clone(),
        speed: k.speed,
        braking: k.braking,
        wheels,
        ..Default::default()
    };
    finish_rig(
        &mut r,
        if lane.dir.norm() > 0.0 {
            lane.dir
        } else {
            Vec3::x()
        },
    );
    r
}

/// Contact point and its vertical velocity for a wheel at `place`, with the
/// node positions extrapolated by `tau` seconds.
fn contact(lane: &Lane, v: &View, place: Place, tau: f64) -> (Vec3, f64) {
    match place {
        Place::Ground(p) => (p, 0.0),
        Place::Deck(k, l) => {
            let (a, b) = (lane.stations[k].node, lane.stations[k + 1].node);
            let (va, vb) = (v.sim.velocity(a), v.sim.velocity(b));
            let xa = v.sim.position(a) + va * tau;
            let xb = v.sim.position(b) + vb * tau;
            (xa.lerp(&xb, l), va.y + (vb.y - va.y) * l)
        }
        Place::Gap => (Vec3::zeros(), 0.0),
    }
}

#[allow(clippy::too_many_arguments)]
fn step_vehicle(
    veh: &mut Vehicle,
    lane: &Lane,
    v: &View,
    t: f64,
    dt: f64,
    ramp: f64,
    scale: f64,
    out: &mut [Vec3],
) {
    let broken = |m: usize| v.sim.is_broken(m);
    let n_ax = veh.axles.len();
    let dir = lane.dir;
    if veh.state.fallen.is_some() {
        return;
    }
    // Initialise each sprung mass at its static ride height.
    if !veh.state.init {
        let k = veh.drive.at(t);
        veh.state.axles = veh
            .axles
            .iter()
            .map(|a| {
                let place = lane.locate(k.s - a.offset, &broken);
                let (p, vy) = contact(lane, v, place, 0.0);
                let sag = match a.suspension {
                    Some(sp) if sp.stiffness > 0.0 => ramp * a.mass * G / sp.stiffness,
                    _ => 0.0,
                };
                AxleState { y: p.y - sag, vy }
            })
            .collect();
        veh.state.init = true;
    }

    // Substeps: resolve the fastest suspension.
    let mut rate: f64 = 0.0;
    for a in &veh.axles {
        if let Some(sp) = a.suspension {
            if a.mass > 0.0 && sp.stiffness > 0.0 {
                rate = rate
                    .max((sp.stiffness / a.mass).sqrt() / 0.1)
                    .max(sp.damping / a.mass / 0.5);
            }
        }
    }
    let n = ((dt * rate).ceil() as usize).clamp(4, 4000);
    let h = dt / n as f64;
    let mass = veh.mass();

    let mut avg_force = vec![0.0; n_ax];
    let mut lifted = vec![false; n_ax];
    let mut braking = false;
    let mut fell_at: Option<f64> = None;
    for i in 0..n {
        let tau = (i as f64 + 0.5) * h;
        let kin = veh.drive.at(t + tau);
        braking |= kin.braking;
        let transfer = veh.transfer(kin.deceleration);
        let places: Vec<Place> = veh
            .axles
            .iter()
            .map(|a| lane.locate(kin.s - a.offset, &broken))
            .collect();
        if places.contains(&Place::Gap) {
            fell_at = Some(t + tau);
            break;
        }
        for (j, a) in veh.axles.iter().enumerate() {
            let (p, vc) = contact(lane, v, places[j], tau);
            let weight = ramp * (a.mass * G + transfer[j]);
            let st = &mut veh.state.axles[j];
            let f = match a.suspension {
                Some(sp) if a.mass > 0.0 && sp.stiffness > 0.0 => {
                    let e = st.y - p.y;
                    let f = -sp.stiffness * e - sp.damping * (st.vy - vc);
                    let f = if f < 0.0 {
                        lifted[j] = true;
                        0.0
                    } else {
                        f
                    };
                    st.vy += h * (f - weight) / a.mass;
                    st.y += h * st.vy;
                    f
                }
                _ => {
                    st.y = p.y;
                    st.vy = vc;
                    weight.max(0.0)
                }
            };
            avg_force[j] += f * h / dt;
            if let Place::Deck(k, l) = places[j] {
                let mut force = Vec3::new(0.0, -f, 0.0);
                if kin.deceleration > 0.0 && mass > 0.0 {
                    force += dir * (ramp * a.mass * kin.deceleration);
                }
                let force = force * (scale * h / dt);
                out[lane.stations[k].node] += force * (1.0 - l);
                out[lane.stations[k + 1].node] += force * l;
            }
        }
    }

    // Rig at the end of the frame.
    let kin = veh.drive.at(t + dt);
    let wheels: Vec<WheelRig> = veh
        .axles
        .iter()
        .enumerate()
        .map(|(j, a)| {
            let place = lane.locate(kin.s - a.offset, &broken);
            let (p, _) = contact(lane, v, place, dt);
            let p = if place == Place::Gap {
                lane.start + dir * (kin.s - a.offset)
            } else {
                p
            };
            let bounce = match a.suspension {
                Some(sp) if a.mass > 0.0 && sp.stiffness > 0.0 => {
                    veh.state.axles[j].y - p.y + ramp * a.mass * G / sp.stiffness
                }
                _ => 0.0,
            };
            WheelRig {
                contact: p,
                force: avg_force[j] * scale,
                bounce,
                on_structure: matches!(place, Place::Deck(..)),
                lifted: lifted[j],
            }
        })
        .collect();
    let mut r = VehicleRig {
        name: veh.name.clone(),
        speed: kin.speed,
        braking,
        fallen: fell_at.is_some(),
        wheels,
        ..Default::default()
    };
    if fell_at.is_some() {
        r.speed = 0.0;
        for w in &mut r.wheels {
            w.force = 0.0;
            w.on_structure = false;
        }
    }
    finish_rig(&mut r, dir);
    if let Some(tf) = fell_at {
        veh.state.fallen = Some((tf, r.wheels.iter().map(|w| w.contact).collect()));
    }
    veh.state.rig = Some(r);
}
