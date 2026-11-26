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
    /// Direct solver only: refactorise the system matrix every this many
    /// solves and reuse the factorisation in between. The matrix only changes
    /// as members rotate, so a factorisation a few substeps old still
    /// converges. 4 with 16 substeps passes every test, including buckling.
    pub refactor_every: usize,
    /// Direct solver only: check every step for sufficient decrease of the
    /// substep objective; refactorise early or shorten the step when it fails.
    pub step_check: bool,
    /// Direct solver only: relaxation time τ (s) of plastic flow. Stress above
    /// yield decays as exp(−t/τ), so a member overloaded by ΔN stretches
    /// plastically at ΔN / (k τ) whatever the substep count. The default,
    /// 1/960 s, is the rate the solver used to have at 16 substeps, when it
    /// returned all overstress to the yield surface once per substep.
    pub plastic_relaxation: f64,
    /// Direct solver only: let members yield inside the substep solve (with
    /// their tangent stiffness in the matrix) instead of solving elastically
    /// and returning the overstress afterwards. The old way limits plastic
    /// flow to one return per substep, so it slows down with fewer substeps.
    pub implicit_plasticity: bool,
    pub gravity: Vec3,
    /// Linear velocity damping, 1/s. Keeps collapses readable and lets
    /// structures settle.
    pub damping: f64,
    /// Enables yield, plastic hinges and fracture.
    pub plasticity: bool,
}

impl Default for SimConfig {
    fn default() -> Self {
        let env = |k: &str, d: usize| {
            std::env::var(k)
                .ok()
                .and_then(|v| v.parse().ok())
                .unwrap_or(d)
        };
        Self {
            solver: Solver::Direct,
            substeps: env("BK_SUB", 16),
            iterations: env("BK_IT", 1),
            refactor_every: env("BK_REF", 4),
            step_check: env("BK_CHECK", 1) != 0,
            plastic_relaxation: 1.0 / 960.0,
            implicit_plasticity: env("BK_IMPL", 1) != 0,
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
    /// Plastic moments reduced by the axial force of the previous substep,
    /// held fixed while a substep is solved.
    mp_now: [f64; 2],
    /// Which yield conditions were active at the last material update.
    yield_mask: u8,
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
    direct: Option<direct::Cache>,
    stats: SolverStats,
    /// A member started or stopped yielding, so its tangent stiffness in the
    /// factorised matrix is out of date.
    yield_changed: bool,
}

/// Elastic deformations of a beam, net of its committed plastic deformation.
struct Strains {
    axial: f64,
    twist: f64,
    /// End rotations [axis][end].
    th: [[f64; 2]; 2],
}

/// Internal forces of a beam for given strains, after the plastic return of
/// this substep, with their tangent stiffness and incremental energy.
struct Response {
    n: f64,
    kt_axial: f64,
    dp_axial: f64,
    /// End moments, tangent and plastic rotation increments per axis.
    m: [[f64; 2]; 2],
    kt: [[[f64; 2]; 2]; 2],
    dp: [[f64; 2]; 2],
    energy: f64,
    yield_mask: u8,
}

/// Viscoplastic return of one force: minimises over the plastic increment dp
/// ½ k (c − dp)² + fy |dp| + ½ k r dp², with r = τ/h. Returns
/// (force, tangent stiffness, energy, dp).
fn return_1d(k: f64, c: f64, fy: f64, r: f64) -> (f64, f64, f64, f64) {
    let trial = k * c;
    let excess = trial.abs() - fy;
    if excess.partial_cmp(&0.0) != Some(std::cmp::Ordering::Greater) {
        return (trial, k, 0.5 * k * c * c, 0.0);
    }
    let dp = excess / (k * (1.0 + r)) * trial.signum();
    let e = c - dp;
    (
        k * e,
        k * r / (1.0 + r),
        0.5 * k * e * e + fy * dp.abs() + 0.5 * k * r * dp * dp,
        dp,
    )
}

/// The same for the two end moments of one bending plane, which are coupled:
/// M = [[k4, k2], [k2, k4]] (θ − dp). Returns (M, tangent, energy, dp).
#[allow(clippy::type_complexity)]
fn return_bend(
    k4: f64,
    k2: f64,
    th: [f64; 2],
    mp: f64,
    r: f64,
) -> ([f64; 2], [[f64; 2]; 2], f64, [f64; 2]) {
    let trial = [k4 * th[0] + k2 * th[1], k2 * th[0] + k4 * th[1]];
    let mut act = [trial[0].abs() > mp, trial[1].abs() > mp];
    let mut sg = [trial[0].signum(), trial[1].signum()];
    let kd = k4 * (1.0 + r);
    let mut dp = [0.0; 2];
    let mut m = trial;
    if act[0] || act[1] {
        for _ in 0..4 {
            let f = [trial[0] - mp * sg[0], trial[1] - mp * sg[1]];
            dp = match act {
                [true, true] => {
                    let det = kd * kd - k2 * k2;
                    [(kd * f[0] - k2 * f[1]) / det, (kd * f[1] - k2 * f[0]) / det]
                }
                [true, false] => [f[0] / kd, 0.0],
                [false, true] => [0.0, f[1] / kd],
                [false, false] => [0.0; 2],
            };
            m = [
                trial[0] - k4 * dp[0] - k2 * dp[1],
                trial[1] - k2 * dp[0] - k4 * dp[1],
            ];
            let mut changed = false;
            for i in 0..2 {
                if act[i] && dp[i] * sg[i] < 0.0 {
                    act[i] = false;
                    changed = true;
                } else if !act[i] && m[i].abs() > mp * (1.0 + 1e-12) {
                    act[i] = true;
                    sg[i] = m[i].signum();
                    changed = true;
                }
            }
            if !changed {
                break;
            }
        }
    }
    // Tangent K − K_A (K_AA + η)⁻¹ K_Aᵀ over the active ends A.
    let kt = match act {
        [true, true] => {
            let det = kd * kd - k2 * k2;
            // K (K + ηI)⁻¹ ηI, with η = k4 r.
            let eta = k4 * r;
            let inv = [[kd / det, -k2 / det], [-k2 / det, kd / det]];
            let a = eta * (k4 * inv[0][0] + k2 * inv[1][0]);
            let b = eta * (k4 * inv[0][1] + k2 * inv[1][1]);
            [[a, b], [b, a]]
        }
        [true, false] => [
            [k4 - k4 * k4 / kd, k2 - k4 * k2 / kd],
            [k2 - k4 * k2 / kd, k4 - k2 * k2 / kd],
        ],
        [false, true] => [
            [k4 - k2 * k2 / kd, k2 - k4 * k2 / kd],
            [k2 - k4 * k2 / kd, k4 - k4 * k4 / kd],
        ],
        [false, false] => [[k4, k2], [k2, k4]],
    };
    let e = [th[0] - dp[0], th[1] - dp[1]];
    let mut energy = 0.5 * (k4 * (e[0] * e[0] + e[1] * e[1]) + 2.0 * k2 * e[0] * e[1]);
    if act[0] || act[1] {
        // Not when elastic: mp may be infinite there.
        energy += mp * (dp[0].abs() + dp[1].abs())
            + 0.5 * k4 * r * (dp[0] * dp[0] + dp[1] * dp[1]);
    }
    (m, kt, energy, dp)
}

/// Work counters of the direct solver since the simulation was created.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct SolverStats {
    pub substeps: usize,
    /// Numeric factorisations of the system matrix.
    pub factorizations: usize,
    /// Steps from a reused factorisation that failed the decrease test and
    /// were redone with a fresh one.
    pub stale_rejects: usize,
    /// Step halvings after a fresh-factorisation step failed the test.
    pub backtracks: usize,
}

/// Sufficient-decrease constant of the step check: accepts steps whose true
/// curvature is up to 2(1 − ARMIJO) = 1.5 times the factorised one.
const ARMIJO: f64 = 0.25;
const MAX_BACKTRACKS: usize = 4;

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
                mp_now: [fy * sec.zy, fy * sec.zz],
                yield_mask: 0,
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
            stats: SolverStats::default(),
            yield_changed: false,
        }
    }

    pub fn solver_stats(&self) -> SolverStats {
        self.stats
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
        self.stats.substeps += 1;
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
                self.update_material(i, h);
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

    /// τ/h of the plastic return: (inside the solve, at the material update).
    /// `None` inside the solve means the solve is elastic.
    fn plastic_ratio(&self, h: f64) -> (Option<f64>, f64) {
        let c = &self.config;
        if c.solver == Solver::Direct && c.implicit_plasticity {
            let r = c.plastic_relaxation.max(0.0) / h;
            (c.plasticity.then_some(r), r)
        } else {
            (None, 0.0)
        }
    }

    fn strains(&self, i: usize, gm: &Geom) -> Strains {
        let bm = &self.beams[i];
        let mut th = [[0.0; 2]; 2];
        for axis in 0..2 {
            let u = gm.axes[axis];
            th[axis] = [
                gm.e.cross(&gm.ta).dot(&u) - bm.hinge[axis][0],
                gm.e.cross(&gm.tb).dot(&u) - bm.hinge[axis][1],
            ];
        }
        Strains {
            axial: gm.len - bm.l0 - bm.plastic_elongation,
            twist: gm.na.cross(&gm.nb).dot(&gm.e),
            th,
        }
    }

    /// Internal forces for `st`. `r` = τ/h of the plastic return, `None` for
    /// an elastic response.
    fn response(&self, i: usize, st: &Strains, r: Option<f64>) -> Response {
        let bm = &self.beams[i];
        let r = r.filter(|_| !bm.brittle);
        let (fy, rr) = (
            if r.is_some() {
                bm.n_yield
            } else {
                f64::INFINITY
            },
            r.unwrap_or(0.0),
        );
        let (n, kt_axial, mut energy, dp_axial) = return_1d(bm.k_axial, st.axial, fy, rr);
        energy += 0.5 * bm.k_torsion * st.twist * st.twist;
        let mut out = Response {
            n,
            kt_axial,
            dp_axial,
            m: [[0.0; 2]; 2],
            kt: [[[0.0; 2]; 2]; 2],
            dp: [[0.0; 2]; 2],
            energy,
            yield_mask: (dp_axial != 0.0) as u8,
        };
        for axis in 0..2 {
            let mp = if r.is_some() {
                bm.mp_now[axis]
            } else {
                f64::INFINITY
            };
            let (k4, k2) = (4.0 * bm.ei_l[axis], 2.0 * bm.ei_l[axis]);
            let (m, kt, e, dp) = return_bend(k4, k2, st.th[axis], mp, rr);
            out.m[axis] = m;
            out.kt[axis] = kt;
            out.dp[axis] = dp;
            out.energy += e;
            for end in 0..2 {
                out.yield_mask |= ((dp[end] != 0.0) as u8) << (1 + axis * 2 + end);
            }
        }
        out
    }

    /// The six generalised forces of a beam at the current state and its
    /// energy: (force, tangent stiffness, gradients [x_a, φ_a, x_b, φ_b]).
    /// The nodal forces are −Σ force·gradient and the Gauss-Newton stiffness is
    /// Σ stiffness·gradient·gradientᵀ. Elastic: force = k·C and energy = Σ ½kC².
    #[allow(clippy::type_complexity)]
    fn beam_constraints(&self, i: usize, r: Option<f64>) -> ([(f64, f64, [Vec3; 4]); 6], f64) {
        let gm = self.geom(i);
        let bm = &self.beams[i];
        let st = self.strains(i, &gm);
        let resp = self.response(i, &st, r);
        let z = Vec3::zeros();
        let mut out = [(0.0, 0.0, [z; 4]); 6];
        out[0] = (resp.n, resp.kt_axial, [-gm.e, z, gm.e, z]);
        out[1] = (
            bm.k_torsion * st.twist,
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
            let ga = p(gm.ta.cross(&u)) / gm.len;
            let gb = p(gm.tb.cross(&u)) / gm.len;
            let pa = gm.ta.cross(&u.cross(&gm.e));
            let pb = gm.tb.cross(&u.cross(&gm.e));
            // The 2x2 tangent between the end moments as two scalar terms
            // along its eigenvectors. Elastic: θa + θb and θa − θb.
            let kt = resp.kt[axis];
            let ang = 0.5 * (2.0 * kt[0][1]).atan2(kt[0][0] - kt[1][1]);
            let (sn, cs) = ang.sin_cos();
            let m = resp.m[axis];
            for (j, v) in [[cs, sn], [-sn, cs]].into_iter().enumerate() {
                let lam =
                    kt[0][0] * v[0] * v[0] + 2.0 * kt[0][1] * v[0] * v[1] + kt[1][1] * v[1] * v[1];
                out[2 + axis * 2 + j] = (
                    m[0] * v[0] + m[1] * v[1],
                    lam.max(0.0),
                    [
                        -(ga * v[0] + gb * v[1]),
                        pa * v[0],
                        ga * v[0] + gb * v[1],
                        pb * v[1],
                    ],
                );
            }
        }
        (out, resp.energy)
    }

    /// Energy of a beam alone, for the step check.
    fn beam_energy(&self, i: usize, r: Option<f64>) -> f64 {
        let gm = self.geom(i);
        self.response(i, &self.strains(i, &gm), r).energy
    }

    /// The substep objective Φ = ½|x − x̃|²_M / h² + Σ beam energy at the
    /// current state (same residual definitions as `assemble_direct`).
    fn objective(&self, h: f64) -> f64 {
        let ih2 = 1.0 / (h * h);
        let mut phi = 0.0;
        for b in &self.bodies {
            if b.inv_mass > 0.0 {
                phi += 0.5 * ih2 / b.inv_mass * (b.x - b.x_tilde).norm_squared();
            }
            if b.inv_inertia > 0.0 {
                let dq = b.q * b.q_tilde.inverse();
                phi += 0.5 * ih2 / b.inv_inertia * 4.0 * dq.imag().norm_squared();
            }
        }
        let r = self.plastic_ratio(h).0;
        for i in 0..self.beams.len() {
            if !self.beams[i].broken {
                phi += self.beam_energy(i, r);
            }
        }
        phi
    }

    /// Right-hand side −∇Φ, the objective Φ, the mass diagonal and (only when
    /// `with_blocks`) the Gauss-Newton stiffness block of every live beam.
    #[allow(clippy::type_complexity)]
    fn assemble_direct(
        &self,
        cache: &direct::Cache,
        h: f64,
        with_blocks: bool,
    ) -> (
        Vec<f64>,
        f64,
        Vec<f64>,
        Vec<([Option<usize>; 4], [f64; 144])>,
    ) {
        let ih2 = 1.0 / (h * h);
        let mut diag = vec![0.0; cache.ndof];
        let mut rhs = vec![0.0; cache.ndof];
        let mut phi = 0.0;
        for (bi, b) in self.bodies.iter().enumerate() {
            if let Some(d) = cache.trans[bi] {
                let m = ih2 / b.inv_mass;
                let r = b.x - b.x_tilde;
                phi += 0.5 * m * r.norm_squared();
                for k in 0..3 {
                    diag[d + k] = m;
                    rhs[d + k] = -m * r[k];
                }
            }
            if let Some(d) = cache.rot[bi] {
                let m = ih2 / b.inv_inertia;
                let dq = b.q * b.q_tilde.inverse();
                let r = dq.imag() * if dq.w < 0.0 { -2.0 } else { 2.0 };
                phi += 0.5 * m * r.norm_squared();
                for k in 0..3 {
                    diag[d + k] = m;
                    rhs[d + k] = -m * r[k];
                }
            }
        }
        let r = self.plastic_ratio(h).0;
        let mut blocks = Vec::with_capacity(if with_blocks { self.beams.len() } else { 0 });
        for i in 0..self.beams.len() {
            if self.beams[i].broken {
                continue;
            }
            let (a, b) = (self.beams[i].a, self.beams[i].b);
            let slots = [cache.trans[a], cache.rot[a], cache.trans[b], cache.rot[b]];
            let mut kb = [[0.0f64; 144]; 1];
            let (terms, energy) = self.beam_constraints(i, r);
            phi += energy;
            for (f, k, g) in terms {
                if with_blocks {
                    let flat: [f64; 12] = std::array::from_fn(|j| g[j / 3][j % 3]);
                    for r in 0..12 {
                        for col in 0..=r {
                            kb[0][r * 12 + col] += k * flat[r] * flat[col];
                        }
                    }
                }
                for (s, slot) in slots.iter().enumerate() {
                    if let Some(d) = slot {
                        for k3 in 0..3 {
                            rhs[d + k3] -= f * g[s][k3];
                        }
                    }
                }
            }
            if with_blocks {
                // Mirror the lower triangle so any (row, col) lookup works.
                for r in 0..12 {
                    for col in r + 1..12 {
                        kb[0][r * 12 + col] = kb[0][col * 12 + r];
                    }
                }
                blocks.push((slots, kb[0]));
            }
        }
        (rhs, phi, diag, blocks)
    }

    /// Move every free body by `t`·`delta`, starting from `base` if given.
    fn apply_delta(
        &mut self,
        cache: &direct::Cache,
        delta: &[f64],
        t: f64,
        base: Option<&[(Vec3, UnitQuaternion<f64>)]>,
    ) {
        for (bi, b) in self.bodies.iter_mut().enumerate() {
            if let Some(base) = base {
                (b.x, b.q) = base[bi];
            }
            if let Some(d) = cache.trans[bi] {
                b.x += Vec3::new(delta[d], delta[d + 1], delta[d + 2]) * t;
            }
            if let Some(d) = cache.rot[bi] {
                b.q = rotate(b.q, Vec3::new(delta[d], delta[d + 1], delta[d + 2]) * t);
            }
        }
    }

    /// One Gauss-Newton step of the substep problem
    /// min Φ = ½|x − x̃|²_M / h² + Σ ½ k C(x)², solved with sparse Cholesky.
    ///
    /// The factorisation is reused between substeps. With `step_check` the step
    /// must then pass a sufficient-decrease test on Φ; a step from a stale
    /// factorisation that fails it is thrown away and redone with a fresh one,
    /// and a fresh step that fails is shortened by backtracking.
    fn solve_direct(&mut self, h: f64) {
        if self.direct.is_none() {
            self.direct = Some(direct::Cache::new(&self.bodies, &self.beams));
        }
        let mut cache = self.direct.take().unwrap();
        let check = self.config.step_check;
        let mut refactor = cache.needs_factor(self.config.refactor_every)
            || std::mem::take(&mut self.yield_changed);
        let base: Vec<(Vec3, UnitQuaternion<f64>)> = if check {
            self.bodies.iter().map(|b| (b.x, b.q)).collect()
        } else {
            Vec::new()
        };
        loop {
            let (rhs, phi0, diag, blocks) = self.assemble_direct(&cache, h, refactor);
            if refactor {
                self.stats.factorizations += 1;
                if cache.factor(&diag, &blocks).is_none() {
                    break;
                }
            }
            let Some(delta) = cache.back_solve(rhs.clone()) else {
                break;
            };
            self.apply_delta(&cache, &delta, 1.0, None);
            if !check {
                break;
            }
            // Armijo test. For a quadratic Φ the decrease is slope·(1 − ρ/2),
            // where ρ is the true curvature along the step over the curvature
            // the factorised matrix assumed: 1 for an exact Newton step, 2 at
            // the edge of divergence.
            let slope: f64 = rhs.iter().zip(&delta).map(|(r, d)| r * d).sum();
            let ok = |phi1: f64, t: f64| phi1 <= phi0 - ARMIJO * t * slope + 1e-10 * phi0.abs();
            if ok(self.objective(h), 1.0) {
                break;
            }
            if !refactor {
                // Stale matrix: undo and redo this step with a fresh one.
                self.stats.stale_rejects += 1;
                self.apply_delta(&cache, &delta, 0.0, Some(&base));
                refactor = true;
                continue;
            }
            // Fresh matrix and still too far: the problem is not quadratic
            // enough at this step length, so shorten the step.
            let mut t = 1.0;
            for _ in 0..MAX_BACKTRACKS {
                t *= 0.5;
                self.stats.backtracks += 1;
                self.apply_delta(&cache, &delta, t, Some(&base));
                if ok(self.objective(h), t) {
                    break;
                }
            }
            break;
        }
        self.direct = Some(cache);
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

    fn update_material(&mut self, i: usize, h: f64) {
        let gm = self.geom(i);
        let st = self.strains(i, &gm);
        let plastic = self.config.plasticity;
        // The same plastic return the solve used, now committed.
        let (in_solve, r) = self.plastic_ratio(h);
        let implicit = in_solve.is_some();
        let resp = self.response(i, &st, plastic.then_some(r));
        let bm = &mut self.beams[i];
        let mut cause = None;

        if plastic && bm.brittle && resp.n.abs() > bm.n_yield {
            cause = Some(BreakCause::Brittle);
        }
        if resp.dp_axial != 0.0 {
            bm.plastic_elongation += resp.dp_axial;
            bm.plastic_strain += resp.dp_axial.abs() / bm.l0;
            if bm.plastic_strain > bm.rupture_strain {
                cause = Some(BreakCause::Rupture);
            }
        }

        // Axial force reduces the plastic moment: Mp,N = Mp (1 - n²),
        // exact for a rectangle and close for tubes.
        let ratio = (resp.n.abs() / bm.n_yield).min(1.0);
        let reduce = (1.0 - ratio * ratio).max(0.05);
        let mut util = ratio * ratio;
        for axis in 0..2 {
            let mut mmax: f64 = 0.0;
            for end in 0..2 {
                let m = resp.m[axis][end];
                if plastic && bm.brittle && m.abs() > bm.m_plastic[axis] * reduce {
                    cause = Some(BreakCause::Brittle);
                }
                let d = resp.dp[axis][end];
                if d != 0.0 {
                    bm.hinge[axis][end] += d;
                    bm.hinge_rotation += d.abs();
                    if bm.hinge_rotation > bm.hinge_capacity {
                        cause = Some(BreakCause::Hinge);
                    }
                }
                mmax = mmax.max(m.abs());
            }
            util += mmax / bm.m_plastic[axis];
            bm.mp_now[axis] = bm.m_plastic[axis] * reduce;
        }
        bm.utilization = util;
        if bm.yield_mask != resp.yield_mask {
            bm.yield_mask = resp.yield_mask;
            // Only matters when the matrix holds tangent stiffnesses.
            self.yield_changed |= implicit;
        }
        let bm = &mut self.beams[i];

        if let Some(cause) = cause {
            bm.broken = true;
            self.direct = None; // sparsity pattern changed
            self.events.push(BreakEvent {
                member: i,
                time: self.time,
                cause,
            });
        }
    }
}

mod direct {
    use super::{Beam, Body};
    use faer::linalg::solvers::Solve;
    use faer::sparse::linalg::solvers::{Llt, SymbolicLlt};
    use faer::sparse::{SparseColMat, Triplet};
    use faer::{Mat, Side};

    /// DOF numbering and the symbolic Cholesky factorisation, reused until a
    /// member breaks and the sparsity pattern changes.
    pub struct Cache {
        pub ndof: usize,
        pub trans: Vec<Option<usize>>,
        pub rot: Vec<Option<usize>>,
        symbolic: Option<SymbolicLlt<usize>>,
        mat: Option<SparseColMat<usize, f64>>,
        pos: Vec<usize>,
        llt: Option<Llt<usize, f64>>,
        age: usize,
    }

    impl Cache {
        pub fn new(bodies: &[Body], _beams: &[Beam]) -> Self {
            let mut ndof = 0;
            let mut trans = Vec::with_capacity(bodies.len());
            let mut rot = Vec::with_capacity(bodies.len());
            for b in bodies {
                trans.push(if b.inv_mass > 0.0 {
                    ndof += 3;
                    Some(ndof - 3)
                } else {
                    None
                });
                rot.push(if b.inv_inertia > 0.0 {
                    ndof += 3;
                    Some(ndof - 3)
                } else {
                    None
                });
            }
            Self {
                ndof,
                trans,
                rot,
                symbolic: None,
                mat: None,
                pos: Vec::new(),
                llt: None,
                age: 0,
            }
        }

        pub fn needs_factor(&self, every: usize) -> bool {
            self.ndof > 0 && (self.llt.is_none() || self.age >= every.max(1))
        }

        pub fn back_solve(&mut self, rhs: Vec<f64>) -> Option<Vec<f64>> {
            let n = self.ndof;
            if n == 0 {
                return Some(rhs);
            }
            let llt = self.llt.as_ref()?;
            self.age += 1;
            let mut x = Mat::<f64>::from_fn(n, 1, |i, _| rhs[i]);
            llt.solve_in_place(x.as_mut());
            Some((0..n).map(|i| x[(i, 0)]).collect())
        }

        /// Factorise (diag + Σ blocks). Each block is a dense 12x12 over the four
        /// 3-DOF slots of a beam (None = fixed); only the lower triangle is used.
        pub fn factor(
            &mut self,
            diag: &[f64],
            blocks: &[([Option<usize>; 4], [f64; 144])],
        ) -> Option<()> {
            let n = self.ndof;
            // Entries in a fixed order: the diagonal, then each block's lower triangle.
            let mut idx: Vec<(usize, usize)> = Vec::new();
            let first = self.mat.is_none();
            let mut vals = Vec::with_capacity(n + blocks.len() * 78);
            for (i, &d) in diag.iter().enumerate() {
                if first {
                    idx.push((i, i));
                }
                vals.push(d);
            }
            for (slots, kb) in blocks {
                for (sr, r0) in slots.iter().enumerate() {
                    let Some(r0) = r0 else { continue };
                    for (sc, c0) in slots.iter().enumerate() {
                        let Some(c0) = c0 else { continue };
                        for i in 0..3 {
                            for j in 0..3 {
                                let (r, c) = (r0 + i, c0 + j);
                                if r >= c {
                                    if first {
                                        idx.push((r, c));
                                    }
                                    vals.push(kb[(sr * 3 + i) * 12 + sc * 3 + j]);
                                }
                            }
                        }
                    }
                }
            }
            if first {
                // Build the pattern once, then map every entry to its slot in
                // the CSC value array so later substeps only scatter values.
                let trip: Vec<_> = idx.iter().map(|&(r, c)| Triplet::new(r, c, 0.0)).collect();
                let mat = SparseColMat::<usize, f64>::try_new_from_triplets(n, n, &trip).ok()?;
                let sym = mat.symbolic();
                let (cp, ri) = (sym.col_ptr(), sym.row_idx());
                self.pos = idx
                    .iter()
                    .map(|&(r, c)| {
                        cp[c]
                            + ri[cp[c]..cp[c + 1]]
                                .binary_search(&r)
                                .expect("entry in pattern")
                    })
                    .collect();
                self.symbolic = SymbolicLlt::try_new(sym, Side::Lower).ok();
                self.mat = Some(mat);
            }
            let mat = self.mat.as_mut()?;
            {
                let v = mat.val_mut();
                v.fill(0.0);
                for (k, &p) in self.pos.iter().enumerate() {
                    v[p] += vals[k];
                }
            }
            self.llt = Some(
                Llt::try_new_with_symbolic(self.symbolic.clone()?, mat.as_ref(), Side::Lower)
                    .ok()?,
            );
            self.age = 0;
            Some(())
        }
    }
}

fn rotate(q: UnitQuaternion<f64>, phi: Vec3) -> UnitQuaternion<f64> {
    let qq = q.into_inner();
    let dq = Quaternion::from_imag(phi) * qq * 0.5;
    UnitQuaternion::from_quaternion(qq + dq)
}
