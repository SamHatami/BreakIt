//! Real-time dynamics: substepped XPBD on corotational beams.
//!
//! Every node is a small rigid body (position + orientation). Every member is
//! six scalar compliant constraints between its end nodes, built from the
//! corotational Euler-Bernoulli beam:
//!
//! * axial stretch, stiffness EA/L
//! * twist, stiffness GJ/L
//! * per bending plane, the symmetric (θa + θb, stiffness 3EI/L) and
//!   antisymmetric (θa − θb, stiffness EI/L) modes. Together they reproduce the
//!   end moments Ma = EI/L (4θa + 2θb), Mb = EI/L (2θa + 4θb) of the linear beam,
//!   so a settled structure matches [`crate::solve_static`].
//!
//! θa and θb are the end rotations of each node's beam tangent relative to the
//! member chord, so rigid-body motion produces no strain (corotational).
//!
//! Plasticity follows the rest-shape idea used in position-based dynamics:
//! when an end moment exceeds the plastic moment, a plastic hinge rotation is
//! added at that end so the moment stays at capacity. Axial yield adds plastic
//! elongation the same way. Members break when accumulated plastic deformation
//! passes the material's capacity, or at first yield for brittle materials.
//!
//! Axial force reduces the plastic moment, Mp,N = Mp (1 − (N/Ny)²).
//! Simplifications for now: the two bending planes yield independently,
//! torsion stays elastic, and supports fix either all translations and/or all
//! rotations of a node.

use nalgebra::{Quaternion, UnitQuaternion};

use crate::frame::member_axes;
use crate::linsolve::{LinSolver, Policy, Stats, BLOCK_NNZ};
use crate::{Structure, Vec3};

/// How each substep's constraint problem is solved.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Solver {
    /// Classic XPBD: Gauss-Seidel sweeps over scalar constraints. Cheap per
    /// sweep, but stiff steel chains need thousands of sweeps per frame to
    /// reach the right stiffness (see `docs/solver-spike.md`).
    GaussSeidel,
    /// The same constraints solved together: one sparse Cholesky solve per
    /// Gauss-Newton iteration. Converges in one or two iterations even for
    /// stiff steel, so a handful of substeps per frame is enough.
    Direct,
}

#[derive(Clone, Debug)]
pub struct SimConfig {
    pub solver: Solver,
    /// Substeps per call to [`DynamicSim::step`].
    pub substeps: usize,
    /// Gauss-Seidel sweeps or Gauss-Newton iterations per substep.
    pub iterations: usize,
    /// Direct solver only: refactorise the system matrix at least every this
    /// many solves. 0 (the default) leaves it to the solver, which keeps an
    /// old factorisation as a preconditioner for as long as that is cheaper
    /// than a new one; 1 factorises every substep.
    pub refactor_every: usize,
    /// Direct solver only: relative accuracy of each linear solve. The old
    /// factorisation only preconditions conjugate gradients on the current
    /// matrix, so the answer is the current system's to this tolerance.
    pub linear_tolerance: f64,
    /// Direct solver only: refactorise once a solve needs more conjugate
    /// gradient iterations than this.
    pub refactor_after_iters: usize,
    pub gravity: Vec3,
    /// Linear velocity damping, 1/s. Keeps collapses readable and lets
    /// structures settle.
    pub damping: f64,
    /// Enables yield, plastic hinges and fracture.
    pub plasticity: bool,
}

impl Default for SimConfig {
    fn default() -> Self {
        Self {
            solver: Solver::Direct,
            substeps: 16,
            iterations: 1,
            refactor_every: 0,
            linear_tolerance: 1e-6,
            refactor_after_iters: 4,
            gravity: Vec3::new(0.0, -9.81, 0.0),
            damping: 0.5,
            plasticity: true,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BreakCause {
    /// Axial plastic strain passed the rupture strain.
    Rupture,
    /// A plastic hinge rotated past its capacity.
    Hinge,
    /// A brittle member reached its strength.
    Brittle,
}

#[derive(Clone, Debug, PartialEq)]
pub struct BreakEvent {
    pub member: usize,
    pub time: f64,
    pub cause: BreakCause,
}

#[derive(Clone, Debug)]
struct Body {
    x: Vec3,
    x0: Vec3,
    x_prev: Vec3,
    v: Vec3,
    q: UnitQuaternion<f64>,
    q_prev: UnitQuaternion<f64>,
    w: Vec3,
    x_tilde: Vec3,
    q_tilde: UnitQuaternion<f64>,
    inv_mass: f64,
    inv_inertia: f64,
    force: Vec3,
}

#[derive(Clone, Debug)]
struct Beam {
    a: usize,
    b: usize,
    l0: f64,
    /// Beam tangent and reference normal in the body frames of a and b.
    ta: Vec3,
    tb: Vec3,
    na: Vec3,
    nb: Vec3,
    k_axial: f64,
    k_torsion: f64,
    /// EI/L about local y and local z.
    ei_l: [f64; 2],
    n_yield: f64,
    /// Plastic moments about local y and z.
    m_plastic: [f64; 2],
    plastic_elongation: f64,
    /// Plastic hinge rotations [axis][end], axis 0 = y, 1 = z.
    hinge: [[f64; 2]; 2],
    plastic_strain: f64,
    hinge_rotation: f64,
    brittle: bool,
    rupture_strain: f64,
    hinge_capacity: f64,
    broken: bool,
    utilization: f64,
    /// Accumulated XPBD multipliers for this substep, one per constraint.
    lambda: [f64; 6],
}

/// Current corotational geometry of a beam.
struct Geom {
    e: Vec3,
    len: f64,
    /// Projector onto the plane normal to e.
    ta: Vec3,
    tb: Vec3,
    na: Vec3,
    nb: Vec3,
    /// Current local axes y and z.
    axes: [Vec3; 2],
}

pub struct DynamicSim {
    pub config: SimConfig,
    bodies: Vec<Body>,
    beams: Vec<Beam>,
    events: Vec<BreakEvent>,
    time: f64,
    direct: Option<DirectSystem>,
}

impl DynamicSim {
    pub fn new(s: &Structure, config: SimConfig) -> Self {
        let mut mass = vec![0.0; s.nodes.len()];
        let mut inertia = vec![0.0; s.nodes.len()];
        let mut beams = Vec::with_capacity(s.members.len());
        for (i, m) in s.members.iter().enumerate() {
            let l = s.member_length(i);
            let mm = m.material.density * m.section.area * l;
            for n in [m.a, m.b] {
                mass[n] += 0.5 * mm;
                inertia[n] += mm * l * l / 24.0;
            }
            let (x, y, _z) = member_axes(&s.nodes[m.a], &s.nodes[m.b]);
            let (e, g, fy) = (
                m.material.youngs_modulus,
                m.material.shear_modulus,
                m.material.yield_strength,
            );
            let sec = &m.section;
            beams.push(Beam {
                a: m.a,
                b: m.b,
                l0: l,
                ta: x,
                tb: x,
                na: y,
                nb: y,
                k_axial: e * sec.area / l,
                k_torsion: g * sec.j / l,
                ei_l: [e * sec.iy / l, e * sec.iz / l],
                n_yield: fy * sec.area,
                m_plastic: [fy * sec.zy, fy * sec.zz],
                plastic_elongation: 0.0,
                hinge: [[0.0; 2]; 2],
                plastic_strain: 0.0,
                hinge_rotation: 0.0,
                brittle: m.material.brittle,
                rupture_strain: m.material.rupture_strain,
                hinge_capacity: m.material.hinge_rotation_capacity,
                broken: false,
                utilization: 0.0,
                lambda: [0.0; 6],
            });
        }
        let mut bodies: Vec<Body> = s
            .nodes
            .iter()
            .enumerate()
            .map(|(i, &p)| Body {
                x: p,
                x0: p,
                x_prev: p,
                v: Vec3::zeros(),
                q: UnitQuaternion::identity(),
                q_prev: UnitQuaternion::identity(),
                w: Vec3::zeros(),
                x_tilde: p,
                q_tilde: UnitQuaternion::identity(),
                inv_mass: if mass[i] > 0.0 { 1.0 / mass[i] } else { 0.0 },
                inv_inertia: if inertia[i] > 0.0 {
                    1.0 / inertia[i]
                } else {
                    0.0
                },
                force: Vec3::zeros(),
            })
            .collect();
        for sup in &s.supports {
            if sup.fixes_translation() {
                bodies[sup.node].inv_mass = 0.0;
            }
            if sup.fixes_rotation() {
                bodies[sup.node].inv_inertia = 0.0;
            }
        }
        Self {
            config,
            bodies,
            beams,
            events: Vec::new(),
            time: 0.0,
            direct: None,
        }
    }

    /// Constant external force on a node (N), replacing any previous one.
    pub fn set_force(&mut self, node: usize, force: Vec3) {
        self.bodies[node].force = force;
    }

    pub fn time(&self) -> f64 {
        self.time
    }
    pub fn position(&self, node: usize) -> Vec3 {
        self.bodies[node].x
    }
    pub fn displacement(&self, node: usize) -> Vec3 {
        self.bodies[node].x - self.bodies[node].x0
    }
    pub fn orientation(&self, node: usize) -> UnitQuaternion<f64> {
        self.bodies[node].q
    }
    pub fn is_broken(&self, member: usize) -> bool {
        self.beams[member].broken
    }
    /// Utilisation from the last substep: (N/Ny)² + My/Mpy + Mz/Mpz (1.0 = yield).
    pub fn utilization(&self, member: usize) -> f64 {
        self.beams[member].utilization
    }
    /// Accumulated plastic hinge rotation of a member, rad.
    pub fn hinge_rotation(&self, member: usize) -> f64 {
        self.beams[member].hinge_rotation
    }
    pub fn break_events(&self) -> &[BreakEvent] {
        &self.events
    }
    /// Linear solver counters (direct solver only), for benchmarks.
    pub fn solver_stats(&self) -> Stats {
        self.direct
            .as_ref()
            .map(|d| d.lin.stats)
            .unwrap_or_default()
    }
    pub fn max_speed(&self) -> f64 {
        self.bodies.iter().map(|b| b.v.norm()).fold(0.0, f64::max)
    }

    /// Advance by `dt` seconds (one render frame).
    pub fn step(&mut self, dt: f64) {
        let h = dt / self.config.substeps as f64;
        for _ in 0..self.config.substeps {
            self.substep(h);
        }
    }

    /// Step at 60 Hz until the structure comes to rest or `max_time` passes.
    /// Returns true if it settled.
    pub fn settle(&mut self, max_time: f64, speed_tol: f64) -> bool {
        let start = self.time;
        while self.time - start < max_time {
            self.step(1.0 / 60.0);
            if self.time - start > 0.5 && self.max_speed() < speed_tol {
                return true;
            }
        }
        false
    }

    fn substep(&mut self, h: f64) {
        let damp = (-self.config.damping * h).exp();
        let g = self.config.gravity;
        let direct = self.config.solver == Solver::Direct;
        for b in &mut self.bodies {
            b.x_prev = b.x;
            b.q_prev = b.q;
            if b.inv_mass > 0.0 {
                let coast = b.v * damp;
                b.v = coast + h * (g + b.force * b.inv_mass);
                b.x_tilde = b.x + h * b.v;
                // XPBD projects from the full prediction x̃. The direct solver
                // instead starts from the force-free coast position and lets the
                // linear solve take in the forces: a large load on a light node
                // would otherwise throw x̃ a whole member length away, too far
                // for one Gauss-Newton step to recover.
                b.x = if direct { b.x + h * coast } else { b.x_tilde };
            } else {
                b.x_tilde = b.x;
            }
            if b.inv_inertia > 0.0 {
                b.w *= damp;
                b.q = rotate(b.q, b.w * h);
            }
            b.q_tilde = b.q;
        }
        match self.config.solver {
            Solver::GaussSeidel => {
                for bm in &mut self.beams {
                    bm.lambda = [0.0; 6];
                }
                for _ in 0..self.config.iterations {
                    for i in 0..self.beams.len() {
                        if !self.beams[i].broken {
                            self.solve_beam(i, h);
                        }
                    }
                }
            }
            Solver::Direct => {
                for _ in 0..self.config.iterations {
                    self.solve_direct(h);
                }
            }
        }
        for b in &mut self.bodies {
            b.v = (b.x - b.x_prev) / h;
            let dq = b.q * b.q_prev.inverse();
            let s = if dq.w < 0.0 { -2.0 } else { 2.0 };
            b.w = dq.imag() * s / h;
        }
        self.time += h;
        for i in 0..self.beams.len() {
            if !self.beams[i].broken {
                self.update_material(i);
            }
        }
    }

    fn geom(&self, i: usize) -> Geom {
        let bm = &self.beams[i];
        let (ba, bb) = (&self.bodies[bm.a], &self.bodies[bm.b]);
        let d = bb.x - ba.x;
        let len = d.norm();
        let e = d / len;
        let ta = ba.q * bm.ta;
        let tb = bb.q * bm.tb;
        let na = ba.q * bm.na;
        let nb = bb.q * bm.nb;
        let n = na + nb;
        let y = (n - e * e.dot(&n)).normalize();
        let z = e.cross(&y);
        Geom {
            e,
            len,
            ta,
            tb,
            na,
            nb,
            axes: [y, z],
        }
    }

    fn solve_beam(&mut self, i: usize, h: f64) {
        // Axial.
        let gm = self.geom(i);
        let bm = &self.beams[i];
        let c = gm.len - bm.l0 - bm.plastic_elongation;
        let k = bm.k_axial;
        self.apply(i, 0, c, -gm.e, gm.e, Vec3::zeros(), Vec3::zeros(), k, h);

        // Torsion.
        let gm = self.geom(i);
        let c = gm.na.cross(&gm.nb).dot(&gm.e);
        let gpa = gm.na.cross(&gm.nb.cross(&gm.e));
        let gpb = gm.nb.cross(&gm.e.cross(&gm.na));
        let k = self.beams[i].k_torsion;
        self.apply(i, 1, c, Vec3::zeros(), Vec3::zeros(), gpa, gpb, k, h);

        // Bending, two planes, symmetric and antisymmetric modes.
        for axis in 0..2 {
            for mode in 0..2 {
                let gm = self.geom(i);
                let bm = &self.beams[i];
                let u = gm.axes[axis];
                let p = |v: Vec3| v - gm.e * gm.e.dot(&v);
                let th_a = gm.e.cross(&gm.ta).dot(&u) - bm.hinge[axis][0];
                let th_b = gm.e.cross(&gm.tb).dot(&u) - bm.hinge[axis][1];
                let gxa_b = p(gm.ta.cross(&u)) / gm.len; // dθa/dxb
                let gxb_b = p(gm.tb.cross(&u)) / gm.len; // dθb/dxb
                let gpa = gm.ta.cross(&u.cross(&gm.e));
                let gpb = gm.tb.cross(&u.cross(&gm.e));
                let (c, gxb, gpa, gpb, k) = if mode == 0 {
                    (th_a + th_b, gxa_b + gxb_b, gpa, gpb, 3.0 * bm.ei_l[axis])
                } else {
                    (th_a - th_b, gxa_b - gxb_b, gpa, -gpb, bm.ei_l[axis])
                };
                self.apply(i, 2 + axis * 2 + mode, c, -gxb, gxb, gpa, gpb, k, h);
            }
        }
    }

    /// All six constraints of a beam at the current state:
    /// (value, stiffness, gradients [x_a, φ_a, x_b, φ_b]).
    fn beam_constraints(&self, i: usize) -> [(f64, f64, [Vec3; 4]); 6] {
        let gm = self.geom(i);
        let bm = &self.beams[i];
        let z = Vec3::zeros();
        let mut out = [(0.0, 0.0, [z; 4]); 6];
        out[0] = (
            gm.len - bm.l0 - bm.plastic_elongation,
            bm.k_axial,
            [-gm.e, z, gm.e, z],
        );
        out[1] = (
            gm.na.cross(&gm.nb).dot(&gm.e),
            bm.k_torsion,
            [
                z,
                gm.na.cross(&gm.nb.cross(&gm.e)),
                z,
                gm.nb.cross(&gm.e.cross(&gm.na)),
            ],
        );
        let p = |v: Vec3| v - gm.e * gm.e.dot(&v);
        for axis in 0..2 {
            let u = gm.axes[axis];
            let th_a = gm.e.cross(&gm.ta).dot(&u) - bm.hinge[axis][0];
            let th_b = gm.e.cross(&gm.tb).dot(&u) - bm.hinge[axis][1];
            let ga = p(gm.ta.cross(&u)) / gm.len;
            let gb = p(gm.tb.cross(&u)) / gm.len;
            let pa = gm.ta.cross(&u.cross(&gm.e));
            let pb = gm.tb.cross(&u.cross(&gm.e));
            out[2 + axis * 2] = (
                th_a + th_b,
                3.0 * bm.ei_l[axis],
                [-(ga + gb), pa, ga + gb, pb],
            );
            out[3 + axis * 2] = (th_a - th_b, bm.ei_l[axis], [-(ga - gb), pa, ga - gb, -pb]);
        }
        out
    }

    /// One Gauss-Newton step of the substep problem
    /// min ½|x − x̃|²_M / h² + Σ ½ k C(x)², solved with sparse Cholesky.
    fn solve_direct(&mut self, h: f64) {
        let mut sys = match self.direct.take() {
            Some(sys) => sys,
            None => match DirectSystem::new(&self.bodies, &self.beams) {
                Some(sys) => sys,
                None => return,
            },
        };
        let ih2 = 1.0 / (h * h);
        sys.lin.clear();
        for (bi, b) in self.bodies.iter().enumerate() {
            if let Some(d) = sys.trans[bi] {
                let m = ih2 / b.inv_mass;
                let r = b.x - b.x_tilde;
                for k in 0..3 {
                    sys.lin.add_diagonal(d + k, m);
                    sys.rhs[d + k] = -m * r[k];
                }
            }
            if let Some(d) = sys.rot[bi] {
                let m = ih2 / b.inv_inertia;
                let dq = b.q * b.q_tilde.inverse();
                let r = dq.imag() * if dq.w < 0.0 { -2.0 } else { 2.0 };
                for k in 0..3 {
                    sys.lin.add_diagonal(d + k, m);
                    sys.rhs[d + k] = -m * r[k];
                }
            }
        }
        for i in 0..self.beams.len() {
            if self.beams[i].broken {
                continue;
            }
            let (a, b) = (self.beams[i].a, self.beams[i].b);
            let slots = [sys.trans[a], sys.rot[a], sys.trans[b], sys.rot[b]];
            let mut kb = [0.0f64; BLOCK_NNZ];
            for (c, k, g) in self.beam_constraints(i) {
                let flat: [f64; 12] = std::array::from_fn(|j| g[j / 3][j % 3]);
                let mut t = 0;
                for r in 0..12 {
                    let kr = k * flat[r];
                    for col in 0..=r {
                        kb[t] += kr * flat[col];
                        t += 1;
                    }
                }
                for (s, slot) in slots.iter().enumerate() {
                    if let Some(d) = slot {
                        for k3 in 0..3 {
                            sys.rhs[d + k3] -= k * c * g[s][k3];
                        }
                    }
                }
            }
            sys.lin.add_block(i, &kb);
        }
        let policy = Policy {
            tolerance: self.config.linear_tolerance,
            refactor_after_iters: self.config.refactor_after_iters,
            max_iters: 4 * self.config.refactor_after_iters.max(2),
            max_age: self.config.refactor_every,
        };
        if sys.lin.solve(&mut sys.rhs, &mut sys.delta, &policy) {
            for (bi, b) in self.bodies.iter_mut().enumerate() {
                let dl = &sys.delta;
                if let Some(d) = sys.trans[bi] {
                    b.x += Vec3::new(dl[d], dl[d + 1], dl[d + 2]);
                }
                if let Some(d) = sys.rot[bi] {
                    b.q = rotate(b.q, Vec3::new(dl[d], dl[d + 1], dl[d + 2]));
                }
            }
        }
        self.direct = Some(sys);
    }

    /// One XPBD projection of a scalar constraint with stiffness `k`.
    #[allow(clippy::too_many_arguments)]
    fn apply(
        &mut self,
        i: usize,
        slot: usize,
        c: f64,
        gxa: Vec3,
        gxb: Vec3,
        gpa: Vec3,
        gpb: Vec3,
        k: f64,
        h: f64,
    ) {
        let (a, b) = (self.beams[i].a, self.beams[i].b);
        let (wa, ia) = (self.bodies[a].inv_mass, self.bodies[a].inv_inertia);
        let (wb, ib) = (self.bodies[b].inv_mass, self.bodies[b].inv_inertia);
        let alpha = 1.0 / (k * h * h);
        let denom = wa * gxa.norm_squared()
            + wb * gxb.norm_squared()
            + ia * gpa.norm_squared()
            + ib * gpb.norm_squared()
            + alpha;
        if denom <= alpha * (1.0 + 1e-12) {
            return; // both ends immovable for this constraint
        }
        let lambda = self.beams[i].lambda[slot];
        let dl = (-c - alpha * lambda) / denom;
        self.beams[i].lambda[slot] += dl;
        let ba = &mut self.bodies[a];
        ba.x += gxa * (wa * dl);
        if ia > 0.0 {
            ba.q = rotate(ba.q, gpa * (ia * dl));
        }
        let bb = &mut self.bodies[b];
        bb.x += gxb * (wb * dl);
        if ib > 0.0 {
            bb.q = rotate(bb.q, gpb * (ib * dl));
        }
    }

    fn update_material(&mut self, i: usize) {
        let gm = self.geom(i);
        let plastic = self.config.plasticity;
        let bm = &mut self.beams[i];
        let mut cause = None;

        let mut n = bm.k_axial * (gm.len - bm.l0 - bm.plastic_elongation);
        if plastic && n.abs() > bm.n_yield {
            if bm.brittle {
                cause = Some(BreakCause::Brittle);
            } else {
                let dp = (n.abs() - bm.n_yield) / bm.k_axial * n.signum();
                bm.plastic_elongation += dp;
                bm.plastic_strain += dp.abs() / bm.l0;
                n = bm.n_yield * n.signum();
                if bm.plastic_strain > bm.rupture_strain {
                    cause = Some(BreakCause::Rupture);
                }
            }
        }

        let ratio = (n.abs() / bm.n_yield).min(1.0);
        let mut util = ratio * ratio;
        for axis in 0..2 {
            let u = gm.axes[axis];
            let k4 = 4.0 * bm.ei_l[axis];
            let k2 = 2.0 * bm.ei_l[axis];
            let mut th = [
                gm.e.cross(&gm.ta).dot(&u) - bm.hinge[axis][0],
                gm.e.cross(&gm.tb).dot(&u) - bm.hinge[axis][1],
            ];
            let mut mmax: f64 = 0.0;
            for end in 0..2 {
                let other = 1 - end;
                let m = k4 * th[end] + k2 * th[other];
                // Axial force reduces the plastic moment: Mp,N = Mp (1 - n²),
                // exact for a rectangle and close for tubes.
                let ratio = (n.abs() / bm.n_yield).min(1.0);
                let mp = bm.m_plastic[axis] * (1.0 - ratio * ratio).max(0.05);
                if plastic && m.abs() > mp {
                    if bm.brittle {
                        cause = Some(BreakCause::Brittle);
                    } else {
                        let d = (m.abs() - mp) / k4 * m.signum();
                        bm.hinge[axis][end] += d;
                        th[end] -= d;
                        bm.hinge_rotation += d.abs();
                        if bm.hinge_rotation > bm.hinge_capacity {
                            cause = Some(BreakCause::Hinge);
                        }
                    }
                }
                mmax = mmax.max((k4 * th[end] + k2 * th[other]).abs());
            }
            util += mmax / bm.m_plastic[axis];
        }
        bm.utilization = util;

        if let Some(cause) = cause {
            bm.broken = true;
            self.events.push(BreakEvent {
                member: i,
                time: self.time,
                cause,
            });
        }
    }
}

/// DOF numbering and the linear solver for [`Solver::Direct`]. Built once:
/// broken beams keep their place in the pattern and contribute zeros.
struct DirectSystem {
    trans: Vec<Option<usize>>,
    rot: Vec<Option<usize>>,
    lin: LinSolver,
    rhs: Vec<f64>,
    delta: Vec<f64>,
}

impl DirectSystem {
    fn new(bodies: &[Body], beams: &[Beam]) -> Option<Self> {
        let mut ndof = 0;
        let mut next = |free: bool| {
            free.then(|| {
                ndof += 3;
                ndof - 3
            })
        };
        let mut trans = Vec::with_capacity(bodies.len());
        let mut rot = Vec::with_capacity(bodies.len());
        for b in bodies {
            trans.push(next(b.inv_mass > 0.0));
            rot.push(next(b.inv_inertia > 0.0));
        }
        let blocks: Vec<_> = beams
            .iter()
            .map(|bm| [trans[bm.a], rot[bm.a], trans[bm.b], rot[bm.b]])
            .collect();
        Some(Self {
            lin: LinSolver::new(ndof, &blocks)?,
            trans,
            rot,
            rhs: vec![0.0; ndof],
            delta: vec![0.0; ndof],
        })
    }
}

fn rotate(q: UnitQuaternion<f64>, phi: Vec3) -> UnitQuaternion<f64> {
    let qq = q.into_inner();
    let dq = Quaternion::from_imag(phi) * qq * 0.5;
    UnitQuaternion::from_quaternion(qq + dq)
}
