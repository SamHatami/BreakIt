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

use crate::contact::{ContactEvent, Ground, NodeContact};
use crate::damage::{self, Committed, Law, LawCache, MatState, Mech, MemberDamage};
use crate::frame::member_axes;
use crate::joint::{bending_stiffness, Joint};
use crate::linsolve::{tri, Backend, LinSolver, Policy, Stats, BLOCK_NNZ};
use crate::simd::SimdLevel;
use crate::{Structure, Vec3};

mod assemble;
mod beam;
mod direct;
mod ground;
mod plastic;
mod pool;
mod xpbd;

use direct::DirectSystem;

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
    /// Direct solver only: structures with at least this many members run
    /// assembly and the step check on the rayon thread pool, a thread per
    /// thousand members or so. Results do not depend on the thread count:
    /// they are the single-thread results bit for bit. `usize::MAX` keeps
    /// everything, the linear solver included, on the calling thread.
    ///
    /// The default is 2,048. On the 4-core cloud machine this was measured
    /// on, 4 threads took 15% off a frame at 2,200 to 5,800 members (2
    /// threads: 7 to 10% from 3,900, nothing at 2,200) and nothing at
    /// 1,300; a single-thread pool costs nothing either way.
    pub parallel_min_members: usize,
    /// Direct solver only: instruction set of the assembly and linear-solver
    /// kernels. The default is [`SimdLevel::from_env`];
    /// [`SimdLevel::Scalar`] runs the whole simulation on the scalar
    /// reference path.
    pub simd: SimdLevel,
    /// Direct solver only: which linear solver to use. The default picks per
    /// structure.
    pub linear_backend: Backend,
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
    /// Ground plane the structure lands on, or `None` for free fall.
    pub ground: Option<Ground>,
}

impl Default for SimConfig {
    fn default() -> Self {
        Self {
            solver: Solver::Direct,
            substeps: 4,
            iterations: 1,
            refactor_every: 0,
            parallel_min_members: 2048,
            simd: SimdLevel::from_env(),
            linear_backend: Backend::Auto,
            linear_tolerance: 1e-4,
            refactor_cost: 0,
            step_check: true,
            plastic_relaxation: 1.0 / 960.0,
            implicit_plasticity: true,
            tension_stiffness: true,
            gravity: Vec3::new(0.0, -9.81, 0.0),
            damping: 0.5,
            plasticity: true,
            ground: None,
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
    contacts: Vec<NodeContact>,
    contact_events: Vec<ContactEvent>,
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

impl DynamicSim {
    pub fn new(s: &Structure, config: SimConfig) -> Self {
        let mut mass = vec![0.0; s.nodes.len()];
        let mut inertia = vec![0.0; s.nodes.len()];
        let mut beams = Vec::with_capacity(s.members.len());
        let mut laws = LawCache::default();
        let mut contacts = vec![NodeContact::default(); s.nodes.len()];
        for (i, m) in s.members.iter().enumerate() {
            let l = s.member_length(i);
            let mm = m.material.density * m.section.area * l;
            for n in [m.a, m.b] {
                mass[n] += 0.5 * mm;
                inertia[n] += mm * l * l / 24.0;
            }
            let half = match m.section.shape {
                crate::Shape::Rect { b, h } => 0.25 * (b + h),
                crate::Shape::Tube { ro, .. } => ro,
            };
            for n in [m.a, m.b] {
                contacts[n].radius = contacts[n].radius.max(half);
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
            // Built here, not on the first step: the symbolic analysis of a
            // large structure costs several frames' worth of time.
            direct: (config.solver == Solver::Direct)
                .then(|| DirectSystem::new(&bodies, &beams, &config))
                .flatten(),
            config,
            bodies,
            beams,
            events: Vec::new(),
            contacts,
            contact_events: Vec::new(),
            time: 0.0,
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

    /// Adds a lumped mass (kg) to a free node, such as a tank or a machine
    /// the structure carries. It has weight under gravity and inertia in
    /// the solve. A negative `kg` takes a mass added earlier off again (a
    /// machine that fell off); the node keeps at least 1% of its mass.
    /// Does nothing at a node whose translation is fixed.
    pub fn add_point_mass(&mut self, node: usize, kg: f64) {
        let b = &mut self.bodies[node];
        if b.inv_mass > 0.0 {
            let m = 1.0 / b.inv_mass;
            b.inv_mass = 1.0 / (m + kg).max(0.01 * m);
        }
    }

    pub fn time(&self) -> f64 {
        self.time
    }
    pub fn position(&self, node: usize) -> Vec3 {
        self.bodies[node].x
    }
    /// Linear velocity of a node, m/s.
    pub fn velocity(&self, node: usize) -> Vec3 {
        self.bodies[node].v
    }
    /// Angular velocity of a node, rad/s.
    pub fn angular_velocity(&self, node: usize) -> Vec3 {
        self.bodies[node].w
    }
    /// Lumped translational mass of a node, kg: half the mass of every member
    /// meeting there. 0 for nodes whose translation is fixed by a support.
    pub fn node_mass(&self, node: usize) -> f64 {
        let im = self.bodies[node].inv_mass;
        if im > 0.0 {
            1.0 / im
        } else {
            0.0
        }
    }
    /// Number of nodes.
    pub fn node_count(&self) -> usize {
        self.bodies.len()
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
    /// Touch-downs on the ground since the start, oldest first.
    pub fn contact_events(&self) -> &[ContactEvent] {
        &self.contact_events
    }
    /// Ground normal force on a node at the end of the last substep, N.
    pub fn contact_force(&self, node: usize) -> f64 {
        self.contacts[node].normal_force
    }
    /// Gap between a node's contact sphere and the nearest ground surface, m
    /// (negative while pressed in), if there is a ground.
    pub fn clearance(&self, node: usize) -> Option<f64> {
        let g = self.config.ground.as_ref()?;
        let b = &self.bodies[node];
        Some(g.clearance(&self.contacts[node], b.x0, b.x))
    }
    pub fn max_speed(&self) -> f64 {
        self.bodies.iter().map(|b| b.v.norm()).fold(0.0, f64::max)
    }

    /// Advance by `dt` seconds (one render frame).
    pub fn step(&mut self, dt: f64) {
        let h = dt / self.config.substeps as f64;
        let pool = self.config.solver == Solver::Direct && self.runs_parallel();
        let mut substeps = || {
            for _ in 0..self.config.substeps {
                self.substep(h);
            }
        };
        if pool {
            // Enter the thread pool once per frame. A parallel loop started
            // from outside the pool hands its job over and blocks until a
            // worker wakes, every time; started from a worker it costs
            // nothing extra. A frame has a dozen or more such loops.
            rayon::join(substeps, || ());
        } else {
            substeps();
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
        let ground = self.config.ground.take();
        for (bi, b) in self.bodies.iter_mut().enumerate() {
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
                if let (Some(gr), true) = (&ground, direct) {
                    gr.clamp_start(&self.contacts[bi], b.x0, &mut b.x);
                }
            } else {
                b.x_tilde = b.x;
            }
            if b.inv_inertia > 0.0 {
                b.w *= damp;
                b.q = rotate(b.q, b.w * h);
            }
            b.q_tilde = b.q;
        }
        // Back in place for the solve, which reads it from the config.
        self.config.ground = ground;
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
                if let Some(gr) = &self.config.ground {
                    for (bi, b) in self.bodies.iter_mut().enumerate() {
                        if b.inv_mass > 0.0 {
                            gr.project(&self.contacts[bi], b.x0, &mut b.x, b.x_prev);
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
        if let Some(gr) = self.config.ground.take() {
            self.commit_contacts(&gr, h);
            self.config.ground = Some(gr);
        }
        for i in 0..self.beams.len() {
            if !self.beams[i].broken {
                self.update_material(i, h);
            }
        }
    }
}

fn rotate(q: UnitQuaternion<f64>, phi: Vec3) -> UnitQuaternion<f64> {
    let qq = q.into_inner();
    let dq = Quaternion::from_imag(phi) * qq * 0.5;
    UnitQuaternion::from_quaternion(qq + dq)
}
