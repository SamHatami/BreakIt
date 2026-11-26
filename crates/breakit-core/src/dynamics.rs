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
//! The yield caps, stiffness changes and failure rules per material live in
//! [`crate::damage`].
//!
//! Axial force reduces the plastic moment, Mp,N = Mp (1 − (N/Ny)²).
//! Simplifications for now: the two bending planes yield independently,
//! torsion stays elastic, and supports fix either all translations and/or all
//! rotations of a node.

use nalgebra::{Quaternion, UnitQuaternion};

use crate::damage::{self, Committed, Law, LawCache, MatState, Mech, MemberDamage};
use crate::frame::member_axes;
use crate::joint::{bending_stiffness, Joint};
use crate::linsolve::{tri, LinSolver, Policy, Stats, BLOCK_NNZ};
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
    /// Direct solver only: the cost of a factorisation in conjugate gradient
    /// iterations. A new factorisation is computed once the old one has cost
    /// that many extra iterations. 0 (the default) estimates the cost from
    /// the structure.
    pub refactor_cost: usize,
    /// Direct solver only: check every step for sufficient decrease of the
    /// substep objective. A failing step (typically a yielding member that
    /// unloads and is suddenly much stiffer than its tangent) is shortened and
    /// followed by another iteration.
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
    /// Direct solver only: add the geometric stiffness of tensile axial forces
    /// to the matrix. Without it a slender member in high tension (a tie, a
    /// cable) is unstable sideways at any practical substep count.
    pub tension_stiffness: bool,
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
            substeps: 4,
            iterations: 1,
            refactor_every: 0,
            linear_tolerance: 1e-6,
            refactor_cost: 0,
            step_check: true,
            plastic_relaxation: 1.0 / 960.0,
            implicit_plasticity: true,
            tension_stiffness: true,
            gravity: Vec3::new(0.0, -9.81, 0.0),
            damping: 0.5,
            plasticity: true,
        }
    }
}

pub use crate::damage::BreakCause;

#[derive(Clone, Debug, PartialEq)]
pub struct BreakEvent {
    pub member: usize,
    pub time: f64,
    pub cause: BreakCause,
    /// Where along the member it failed: 0 = end a, 1 = end b, 0.5 for
    /// failures spread along the member (axial rupture, shear splits).
    pub position: f64,
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
    /// Axial stiffness EA/L in compression, and in tension until cracked.
    k_axial: f64,
    /// Axial stiffness in tension (lower once concrete cracks through).
    k_tension: f64,
    k_torsion: f64,
    /// EI/L about local y and local z.
    ei_l: [f64; 2],
    /// Axial yield force [tension, compression] set by the material for this
    /// substep; infinite where the material does not yield.
    n_cap: [f64; 2],
    /// Plastic moments about local y and z for this substep (reduced by the
    /// previous substep's axial force); infinite for no hinges.
    mp_now: [f64; 2],
    /// Which yield conditions were active at the last material update.
    yield_mask: u8,
    plastic_elongation: f64,
    /// Plastic hinge rotations [axis][end], axis 0 = y, 1 = z: the member's
    /// own plus its joints'.
    hinge: [[f64; 2]; 2],
    /// Connections at ends a and b.
    joints: [Joint; 2],
    /// The part of `hinge` that yielded in the joints rather than the member.
    joint_hinge: [[f64; 2]; 2],
    /// Plastic rotation booked against each joint's rotation capacity, rad.
    joint_rotation: [f64; 2],
    broken: bool,
    /// Material law, its history and the damage reported to renderers.
    law: Law,
    mat: MatState,
    damage: MemberDamage,
    /// Accumulated XPBD multipliers for this substep, one per constraint.
    lambda: [f64; 6],
}

impl Beam {
    /// End-moment stiffness of one bending plane, joints included.
    fn bending(&self, axis: usize) -> [[f64; 2]; 2] {
        bending_stiffness(
            self.ei_l[axis],
            [
                self.joints[0].stiffness[axis],
                self.joints[1].stiffness[axis],
            ],
        )
    }

    /// Plastic end moments of one bending plane: the member's, or a weaker
    /// joint's.
    fn end_mp(&self, axis: usize) -> [f64; 2] {
        [0, 1].map(|e| self.mp_now[axis].min(self.joints[e].moment_capacity[axis]))
    }
}

/// Eigenvectors and eigenvalues (clamped at zero) of a symmetric 2x2 matrix.
fn eigen2(k: [[f64; 2]; 2]) -> [([f64; 2], f64); 2] {
    let ang = 0.5 * (2.0 * k[0][1]).atan2(k[0][0] - k[1][1]);
    let (sn, cs) = ang.sin_cos();
    [[cs, sn], [-sn, cs]].map(|v| {
        let lam = k[0][0] * v[0] * v[0] + 2.0 * k[0][1] * v[0] * v[1] + k[1][1] * v[1] * v[1];
        (v, lam.max(0.0))
    })
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
    stats: SolverStats,
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
/// M = K (θ − dp), K symmetric (rigid ends: EI/L [[4, 2], [2, 4]]; joints
/// change it, and a pinned end has a zero row). Each end has its own plastic
/// moment `mp` (the member's, or a weaker joint's) and viscous term
/// η = K_ii r. Returns (M, tangent, energy, dp).
#[allow(clippy::type_complexity)]
fn return_bend(
    k: [[f64; 2]; 2],
    th: [f64; 2],
    mp: [f64; 2],
    r: f64,
) -> ([f64; 2], [[f64; 2]; 2], f64, [f64; 2]) {
    let trial = [
        k[0][0] * th[0] + k[0][1] * th[1],
        k[1][0] * th[0] + k[1][1] * th[1],
    ];
    let mut act = [trial[0].abs() > mp[0], trial[1].abs() > mp[1]];
    let mut sg = [trial[0].signum(), trial[1].signum()];
    let kd = [k[0][0] * (1.0 + r), k[1][1] * (1.0 + r)];
    let mut dp = [0.0; 2];
    let mut m = trial;
    if act[0] || act[1] {
        for _ in 0..4 {
            let f = [trial[0] - mp[0] * sg[0], trial[1] - mp[1] * sg[1]];
            dp = match act {
                [true, true] => {
                    let det = kd[0] * kd[1] - k[0][1] * k[0][1];
                    [
                        (kd[1] * f[0] - k[0][1] * f[1]) / det,
                        (kd[0] * f[1] - k[0][1] * f[0]) / det,
                    ]
                }
                [true, false] => [f[0] / kd[0], 0.0],
                [false, true] => [0.0, f[1] / kd[1]],
                [false, false] => [0.0; 2],
            };
            m = [
                trial[0] - k[0][0] * dp[0] - k[0][1] * dp[1],
                trial[1] - k[1][0] * dp[0] - k[1][1] * dp[1],
            ];
            let mut changed = false;
            for i in 0..2 {
                if act[i] && dp[i] * sg[i] < 0.0 {
                    act[i] = false;
                    changed = true;
                } else if !act[i] && m[i].abs() > mp[i] * (1.0 + 1e-12) {
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
    // Tangent K − K_A (K_AA + η_A)⁻¹ K_Aᵀ over the active ends A.
    let kt = match act {
        [true, true] => {
            let det = kd[0] * kd[1] - k[0][1] * k[0][1];
            let inv = [[kd[1] / det, -k[0][1] / det], [-k[0][1] / det, kd[0] / det]];
            let mut t = k;
            for a in 0..2 {
                for b in 0..2 {
                    for i in 0..2 {
                        for j in 0..2 {
                            t[a][b] -= k[a][i] * inv[i][j] * k[j][b];
                        }
                    }
                }
            }
            t
        }
        [true, false] | [false, true] => {
            let i = if act[0] { 0 } else { 1 };
            let mut t = k;
            for a in 0..2 {
                for b in 0..2 {
                    t[a][b] -= k[a][i] * k[i][b] / kd[i];
                }
            }
            t
        }
        [false, false] => k,
    };
    let e = [th[0] - dp[0], th[1] - dp[1]];
    let mut energy = 0.5 * (k[0][0] * e[0] * e[0] + k[1][1] * e[1] * e[1]) + k[0][1] * e[0] * e[1];
    for i in 0..2 {
        // Only yielding ends: mp may be infinite elsewhere.
        if dp[i] != 0.0 {
            energy += mp[i] * dp[i].abs() + 0.5 * k[i][i] * r * dp[i] * dp[i];
        }
    }
    (m, kt, energy, dp)
}

/// Work counters of the direct solver since the simulation was created.
#[derive(Clone, Copy, Debug, Default)]
pub struct SolverStats {
    pub substeps: usize,
    /// Linear solver work: solves, factorisations, CG iterations, timings.
    pub linear: Stats,
    /// Step halvings after a fresh-factorisation step failed the test.
    pub backtracks: usize,
    /// Gauss-Newton iterations added to a substep after a shortened step.
    pub extra_iterations: usize,
}

/// Sufficient-decrease constant of the step check: accepts steps whose true
/// curvature is up to 2(1 − ARMIJO) = 1.5 times the factorised one.
const ARMIJO: f64 = 0.25;
const MAX_BACKTRACKS: usize = 4;
/// Extra Gauss-Newton iterations allowed in a substep after a shortened step.
const MAX_RELINEARISE: usize = 2;

impl DynamicSim {
    pub fn new(s: &Structure, config: SimConfig) -> Self {
        let mut mass = vec![0.0; s.nodes.len()];
        let mut inertia = vec![0.0; s.nodes.len()];
        let mut beams = Vec::with_capacity(s.members.len());
        let mut laws = LawCache::default();
        for (i, m) in s.members.iter().enumerate() {
            let l = s.member_length(i);
            let mm = m.material.density * m.section.area * l;
            for n in [m.a, m.b] {
                mass[n] += 0.5 * mm;
                inertia[n] += mm * l * l / 24.0;
            }
            let (x, y, _z) = member_axes(&s.nodes[m.a], &s.nodes[m.b]);
            let (e, g) = (m.material.youngs_modulus, m.material.shear_modulus);
            let sec = &m.section;
            let (k_axial, ei_l) = (e * sec.area / l, [e * sec.iy / l, e * sec.iz / l]);
            let law = laws.get(&m.material, sec);
            let mech = Mech::new(&law, k_axial, ei_l, config.plasticity);
            beams.push(Beam {
                a: m.a,
                b: m.b,
                l0: l,
                ta: x,
                tb: x,
                na: y,
                nb: y,
                k_axial,
                k_tension: mech.k_tension,
                k_torsion: g * sec.j / l,
                ei_l,
                n_cap: mech.n_cap,
                mp_now: mech.mp,
                yield_mask: 0,
                plastic_elongation: 0.0,
                hinge: [[0.0; 2]; 2],
                joints: m.ends,
                joint_hinge: [[0.0; 2]; 2],
                joint_rotation: [0.0; 2],
                broken: false,
                mat: MatState::new(k_axial, ei_l),
                damage: MemberDamage::new(m.material.kind),
                law,
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
        }
    }

    pub fn solver_stats(&self) -> SolverStats {
        SolverStats {
            linear: self
                .direct
                .as_ref()
                .map(|d| d.lin.stats)
                .unwrap_or_default(),
            ..self.stats
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
    /// Utilisation from the last substep, 1.0 = at capacity. For steel,
    /// (N/Ny)² + My/Mpy + Mz/Mpz; other materials see [`crate::damage`].
    pub fn utilization(&self, member: usize) -> f64 {
        self.beams[member].damage.utilization
    }
    /// Accumulated plastic hinge rotation of a member over both ends, rad.
    pub fn hinge_rotation(&self, member: usize) -> f64 {
        self.beams[member].mat.hinge_rotation
    }
    /// Damage state of one member, for drawing it.
    pub fn damage(&self, member: usize) -> &MemberDamage {
        &self.beams[member].damage
    }
    /// Damage state of every member, in member order.
    pub fn damage_all(&self) -> impl ExactSizeIterator<Item = &MemberDamage> + '_ {
        self.beams.iter().map(|b| &b.damage)
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

        // Bending, two planes, along the eigenvectors of each plane's end
        // stiffness (rigid ends: the symmetric and antisymmetric modes).
        for axis in 0..2 {
            let kb = self.beams[i].bending(axis);
            for (mode, (v, k)) in eigen2(kb).into_iter().enumerate() {
                if k <= 0.0 {
                    continue; // a pinned end: nothing to resist
                }
                let gm = self.geom(i);
                let bm = &self.beams[i];
                let u = gm.axes[axis];
                let p = |v: Vec3| v - gm.e * gm.e.dot(&v);
                let th_a = gm.e.cross(&gm.ta).dot(&u) - bm.hinge[axis][0];
                let th_b = gm.e.cross(&gm.tb).dot(&u) - bm.hinge[axis][1];
                let gxa_b = p(gm.ta.cross(&u)) / gm.len; // dθa/dxb
                let gxb_b = p(gm.tb.cross(&u)) / gm.len; // dθb/dxb
                let gpa = gm.ta.cross(&u.cross(&gm.e)) * v[0];
                let gpb = gm.tb.cross(&u.cross(&gm.e)) * v[1];
                let c = th_a * v[0] + th_b * v[1];
                let gxb = gxa_b * v[0] + gxb_b * v[1];
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
        let th = std::array::from_fn(|axis| {
            let u = gm.axes[axis];
            [
                gm.e.cross(&gm.ta).dot(&u) - bm.hinge[axis][0],
                gm.e.cross(&gm.tb).dot(&u) - bm.hinge[axis][1],
            ]
        });
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
        // Tension and compression may differ in stiffness (cracked concrete)
        // and yield force; the energy stays continuous through zero.
        let sense = if st.axial > 0.0 { 0 } else { 1 };
        let k = if sense == 0 { bm.k_tension } else { bm.k_axial };
        let (fy, rr) = (
            if r.is_some() {
                bm.n_cap[sense]
            } else {
                f64::INFINITY
            },
            r.unwrap_or(0.0),
        );
        let (n, kt_axial, mut energy, dp_axial) = return_1d(k, st.axial, fy, rr);
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
                bm.end_mp(axis)
            } else {
                [f64::INFINITY; 2]
            };
            let (m, kt, e, dp) = return_bend(bm.bending(axis), st.th[axis], mp, rr);
            out.m[axis] = m;
            out.kt[axis] = kt;
            out.dp[axis] = dp;
            out.energy += e;
            for (end, d) in dp.iter().enumerate() {
                out.yield_mask |= ((*d != 0.0) as u8) << (1 + axis * 2 + end);
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
            let m = resp.m[axis];
            for (j, (v, lam)) in eigen2(resp.kt[axis]).into_iter().enumerate() {
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

    /// Assemble the Gauss-Newton matrix into `sys.lin` and the right-hand
    /// side −∇Φ into `sys.rhs`. Returns the objective Φ.
    fn assemble_direct(&self, sys: &mut DirectSystem, h: f64) -> f64 {
        let t0 = std::time::Instant::now();
        let ih2 = 1.0 / (h * h);
        let mut phi = 0.0;
        sys.lin.clear();
        for (bi, b) in self.bodies.iter().enumerate() {
            if let Some(d) = sys.trans[bi] {
                let m = ih2 / b.inv_mass;
                let r = b.x - b.x_tilde;
                phi += 0.5 * m * r.norm_squared();
                for k in 0..3 {
                    sys.lin.add_diagonal(d + k, m);
                    sys.rhs[d + k] = -m * r[k];
                }
            }
            if let Some(d) = sys.rot[bi] {
                let m = ih2 / b.inv_inertia;
                let dq = b.q * b.q_tilde.inverse();
                let r = dq.imag() * if dq.w < 0.0 { -2.0 } else { 2.0 };
                phi += 0.5 * m * r.norm_squared();
                for k in 0..3 {
                    sys.lin.add_diagonal(d + k, m);
                    sys.rhs[d + k] = -m * r[k];
                }
            }
        }
        let r = self.plastic_ratio(h).0;
        for i in 0..self.beams.len() {
            if self.beams[i].broken {
                continue;
            }
            let (a, b) = (self.beams[i].a, self.beams[i].b);
            let slots = [sys.trans[a], sys.rot[a], sys.trans[b], sys.rot[b]];
            let mut kb = [0.0f64; BLOCK_NNZ];
            let (terms, energy) = self.beam_constraints(i, r);
            phi += energy;
            for (f, k, g) in terms {
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
                            sys.rhs[d + k3] -= f * g[s][k3];
                        }
                    }
                }
            }
            if self.config.tension_stiffness {
                // Geometric stiffness of the axial force, (N/L)(I − e eᵀ) on
                // the relative translation of the ends: the restoring force of
                // a taut string. Tension only. Under compression the term is
                // negative and could make the matrix indefinite; leaving it
                // out there only makes the step slightly short.
                let (n, e) = (terms[0].0, terms[0].2[2]);
                if n > 0.0 {
                    let gk = n / (self.bodies[b].x - self.bodies[a].x).norm();
                    for r in 0..3 {
                        for col in 0..=r {
                            let v = gk * (if r == col { 1.0 } else { 0.0 } - e[r] * e[col]);
                            kb[tri(r, col)] += v;
                            kb[tri(6 + r, 6 + col)] += v;
                            kb[tri(6 + r, col)] -= v;
                            if r != col {
                                kb[tri(6 + col, r)] -= v;
                            }
                        }
                    }
                }
            }
            sys.lin.add_block(i, &kb);
        }
        sys.lin.stats.assembly_seconds += t0.elapsed().as_secs_f64();
        phi
    }

    /// Move every free body by `t`·`delta`, starting from `base` if given.
    fn apply_delta(
        &mut self,
        sys: &DirectSystem,
        t: f64,
        base: Option<&[(Vec3, UnitQuaternion<f64>)]>,
    ) {
        let delta = &sys.delta;
        for (bi, b) in self.bodies.iter_mut().enumerate() {
            if let Some(base) = base {
                (b.x, b.q) = base[bi];
            }
            if let Some(d) = sys.trans[bi] {
                b.x += Vec3::new(delta[d], delta[d + 1], delta[d + 2]) * t;
            }
            if let Some(d) = sys.rot[bi] {
                b.q = rotate(b.q, Vec3::new(delta[d], delta[d + 1], delta[d + 2]) * t);
            }
        }
    }

    /// One Gauss-Newton step of the substep problem
    /// min Φ = ½|x − x̃|²_M / h² + Σ beam energy.
    ///
    /// The linear system is solved to `linear_tolerance` by [`LinSolver`]. With
    /// `step_check` the step must also pass a sufficient-decrease test on Φ;
    /// a step that fails is shortened by backtracking and followed by another
    /// iteration.
    fn solve_direct(&mut self, h: f64) {
        let mut sys = match self.direct.take() {
            Some(sys) => sys,
            None => match DirectSystem::new(&self.bodies, &self.beams) {
                Some(sys) => sys,
                None => return,
            },
        };
        let check = self.config.step_check;
        let policy = Policy {
            tolerance: self.config.linear_tolerance,
            refactor_cost: self.config.refactor_cost,
            max_iters: 0,
            max_age: self.config.refactor_every,
        };
        let mut base = std::mem::take(&mut sys.base);
        let mut relinearised = 0;
        loop {
            if check {
                base.clear();
                base.extend(self.bodies.iter().map(|b| (b.x, b.q)));
            }
            let phi0 = self.assemble_direct(&mut sys, h);
            if !sys.lin.solve(&sys.rhs, &mut sys.delta, &policy) {
                break;
            }
            self.apply_delta(&sys, 1.0, None);
            if !check {
                break;
            }
            // Armijo test. For a quadratic Φ the decrease is slope·(1 − ρ/2),
            // where ρ is the true curvature along the step over the curvature
            // the matrix assumed: 1 for an exact Newton step, 2 at the edge
            // of divergence.
            let slope: f64 = sys.rhs.iter().zip(&sys.delta).map(|(r, d)| r * d).sum();
            let ok = |phi1: f64, t: f64| phi1 <= phi0 - ARMIJO * t * slope + 1e-10 * phi0.abs();
            if ok(self.objective(h), 1.0) {
                break;
            }
            // Φ is not quadratic enough over this step (typically a yielding
            // member unloads, and is five times stiffer again). Shorten the
            // step, then linearise again from there: stopping here would leave
            // the substep unconverged, and the leftover residual comes back
            // as velocity every substep.
            let mut t = 1.0;
            for _ in 0..MAX_BACKTRACKS {
                t *= 0.5;
                self.stats.backtracks += 1;
                self.apply_delta(&sys, t, Some(&base));
                if ok(self.objective(h), t) {
                    break;
                }
            }
            if relinearised == MAX_RELINEARISE {
                break;
            }
            relinearised += 1;
            self.stats.extra_iterations += 1;
        }
        sys.base = base;
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

    #[allow(clippy::needless_range_loop)]
    fn update_material(&mut self, i: usize, h: f64) {
        let gm = self.geom(i);
        let st = self.strains(i, &gm);
        let plastic = self.config.plasticity;
        // The same plastic return the solve used, now committed.
        let r = self.plastic_ratio(h).1;
        let resp = self.response(i, &st, plastic.then_some(r));
        let bm = &mut self.beams[i];
        bm.plastic_elongation += resp.dp_axial;
        // Rotation at an end where the joint is weaker than the member is the
        // joint yielding; the material model only sees the member's own.
        let mut member_dp = resp.dp;
        for axis in 0..2 {
            for end in 0..2 {
                let dp = resp.dp[axis][end];
                bm.hinge[axis][end] += dp;
                bm.joint_rotation[end] += dp.abs();
                if bm.joints[end].moment_capacity[axis] < bm.mp_now[axis] {
                    bm.joint_hinge[axis][end] += dp;
                    member_dp[axis][end] = 0.0;
                }
            }
        }
        let member_hinge: [[f64; 2]; 2] =
            std::array::from_fn(|a| [0, 1].map(|e| bm.hinge[a][e] - bm.joint_hinge[a][e]));
        bm.yield_mask = resp.yield_mask;
        let mut mech = Mech {
            k_axial: bm.k_axial,
            k_tension: bm.k_tension,
            ei_l: bm.ei_l,
            n_cap: bm.n_cap,
            mp: bm.mp_now,
        };
        let committed = Committed {
            n: resp.n,
            m: resp.m,
            dp_axial: resp.dp_axial,
            dp: member_dp,
            hinge: member_hinge,
            l0: bm.l0,
            len: gm.len,
        };
        let failure = damage::update(
            &bm.law,
            &mut bm.mat,
            &mut bm.damage,
            &mut mech,
            &committed,
            plastic,
        );
        bm.k_axial = mech.k_axial;
        bm.k_tension = mech.k_tension;
        bm.ei_l = mech.ei_l;
        bm.n_cap = mech.n_cap;
        bm.mp_now = mech.mp;

        // Connections: force and moment against their capacities, and the
        // plastic rotation at each end against the joint's rotation capacity.
        let mut failure = failure;
        let v = [
            (resp.m[0][0] + resp.m[0][1]) / gm.len,
            (resp.m[1][0] + resp.m[1][1]) / gm.len,
        ];
        for end in 0..2 {
            let j = &bm.joints[end];
            if j.is_rigid() {
                continue;
            }
            let u = j
                .utilization(resp.n, v, [resp.m[0][end], resp.m[1][end]])
                .max(bm.joint_rotation[end] / j.rotation_capacity);
            bm.damage.joint[end] = bm.damage.joint[end].max(u.min(1.0));
            let force = (resp.n * resp.n + v[0] * v[0] + v[1] * v[1]).sqrt();
            if failure.is_none()
                && (force > j.force_capacity || bm.joint_rotation[end] > j.rotation_capacity)
            {
                failure = Some((BreakCause::Connection, end as f64));
                bm.damage.state = damage::DamageState::Broken;
                bm.damage.damage = 1.0;
            }
        }

        if let Some((cause, position)) = failure {
            bm.broken = true;
            self.events.push(BreakEvent {
                member: i,
                time: self.time,
                cause,
                position,
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
    /// Scratch for the step check: body states before the step.
    base: Vec<(Vec3, UnitQuaternion<f64>)>,
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
            base: Vec::new(),
        })
    }
}

fn rotate(q: UnitQuaternion<f64>, phi: Vec3) -> UnitQuaternion<f64> {
    let qq = q.into_inner();
    let dq = Quaternion::from_imag(phi) * qq * 0.5;
    UnitQuaternion::from_quaternion(qq + dq)
}
