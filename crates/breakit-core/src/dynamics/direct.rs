//! The direct path ([`Solver::Direct`](super::Solver)): assemble every member
//! into one sparse system per substep, solve it, and check the step against
//! the true energy with a backtracking line search.

use super::assemble::{AssembleSerial, BeamTerms, ComputeChunk};
use super::pool::ordered_chunks;
use super::*;

/// Sufficient-decrease constant of the step check: accepts steps whose true
/// curvature is up to 2(1 − ARMIJO) = 1.5 times the factorised one.
pub(super) const ARMIJO: f64 = 0.25;

pub(super) const MAX_BACKTRACKS: usize = 4;

/// Extra Gauss-Newton iterations allowed in a substep after a shortened step.
pub(super) const MAX_RELINEARISE: usize = 2;

/// DOF numbering and the linear solver for [`Solver::Direct`]. Built once:
/// broken beams keep their place in the pattern and contribute zeros.
pub(super) struct DirectSystem {
    trans: Vec<Option<usize>>,
    rot: Vec<Option<usize>>,
    pub(super) lin: LinSolver,
    pub(super) rhs: Vec<f64>,
    delta: Vec<f64>,
    /// Scratch for the step check: body states before the step.
    base: Vec<(Vec3, UnitQuaternion<f64>)>,
    /// Scratch for threaded assembly: every beam's contribution.
    terms: Vec<BeamTerms>,
}

/// Beams per parallel work item, and per partial sum of the step check.
pub(super) const PAR_CHUNK: usize = 64;

/// Threaded assembly asks for one more thread per this many beams (about
/// 0.2 ms of work), threaded step checks per [`OBJECTIVE_PER_HELPER`]
/// (the same time: a beam's energy alone costs less than half its block).
/// Waking a worker costs tens of microseconds on the calling thread, and
/// members a worker has read are slower for this thread to write to in
/// `update_material` afterwards, so more threads than this lose. Measured on
/// a 4-core cloud machine with towers and bridges of 1,300 to 5,800 members.
const ASSEMBLY_PER_HELPER: usize = 1024;
const OBJECTIVE_PER_HELPER: usize = 2048;

impl DirectSystem {
    pub(super) fn new(bodies: &[Body], beams: &[Beam], config: &SimConfig) -> Option<Self> {
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
        let mut lin = LinSolver::new(ndof, &blocks, config.linear_backend)?;
        lin.set_simd(config.simd);
        if config.parallel_min_members == usize::MAX {
            lin.set_parallel_min_nodes(usize::MAX);
        }
        Some(Self {
            lin,
            trans,
            rot,
            rhs: vec![0.0; ndof],
            delta: vec![0.0; ndof],
            base: Vec::new(),
            terms: Vec::new(),
        })
    }

    /// Subtract a beam's elastic force from the right-hand side.
    #[inline(always)]
    pub(super) fn add_force(&mut self, bm: &Beam, force: &[f64; 12]) {
        let slots = [
            self.trans[bm.a],
            self.rot[bm.a],
            self.trans[bm.b],
            self.rot[bm.b],
        ];
        for (s, slot) in slots.iter().enumerate() {
            if let Some(d) = slot {
                for k3 in 0..3 {
                    self.rhs[d + k3] -= force[s * 3 + k3];
                }
            }
        }
    }
}
impl DynamicSim {
    /// The substep objective Φ = ½|x − x̃|²_M / h² + Σ beam energy at the
    /// current state (same residual definitions as `assemble_direct`).
    ///
    /// Leaves every live member's length and response in `checked`.
    pub(super) fn objective(&self, h: f64, checked: &mut [(f64, Response)]) -> f64 {
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
        if let Some(gr) = &self.config.ground {
            for bi in 0..self.bodies.len() {
                if let Some(t) = self.contact_terms(gr, bi, h) {
                    phi += t.energy;
                }
            }
        }
        let r = self.plastic_ratio(h).0;
        // Summed per fixed-size chunk, then in chunk order, on one thread or
        // many: the same result on any number of threads.
        let n = self.beams.len();
        let chunk = |c: usize, out: &mut [(f64, Response)]| {
            let mut sum = 0.0;
            for (k, slot) in out.iter_mut().enumerate() {
                let i = c * PAR_CHUNK + k;
                if !self.beams[i].broken {
                    *slot = self.beam_response(i, r);
                    sum += slot.1.energy;
                }
            }
            sum
        };
        if self.runs_parallel() {
            let mut parts: Vec<_> = checked
                .chunks_mut(PAR_CHUNK)
                .map(|out| (0.0, out))
                .collect();
            let mut sum = 0.0;
            ordered_chunks(
                &mut parts,
                n / OBJECTIVE_PER_HELPER,
                |c, part| part.0 = chunk(c, part.1),
                |_, part| sum += part.0,
            );
            phi += sum;
        } else {
            phi += (checked.chunks_mut(PAR_CHUNK).enumerate())
                .map(|(c, out)| chunk(c, out))
                .sum::<f64>();
        }
        phi
    }

    pub(super) fn runs_parallel(&self) -> bool {
        self.beams.len() >= self.config.parallel_min_members
    }

    /// Assemble the Gauss-Newton matrix into `sys.lin` and the right-hand
    /// side −∇Φ into `sys.rhs`. Returns the objective Φ.
    pub(super) fn assemble_direct(&self, sys: &mut DirectSystem, h: f64) -> f64 {
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
        if let Some(gr) = &self.config.ground {
            // Ground contact: diagonal, non-negative, translation DOFs only,
            // so it needs no change to the sparsity pattern.
            for bi in 0..self.bodies.len() {
                if let (Some(d), Some(t)) = (sys.trans[bi], self.contact_terms(gr, bi, h)) {
                    phi += t.energy;
                    for k in 0..3 {
                        sys.lin.add_diagonal(d + k, t.hess[k]);
                        sys.rhs[d + k] -= t.grad[k];
                    }
                }
            }
        }
        let r = self.plastic_ratio(h).0;
        let arch = self.config.simd.arch();
        if self.runs_parallel() {
            // Per-beam work has no shared state: the blocks and forces are
            // computed in fixed chunks on whichever threads are there, and
            // added to the system by this thread in beam order. The result
            // does not depend on the thread count and is the single-thread
            // result bit for bit.
            let mut terms = std::mem::take(&mut sys.terms);
            terms.resize(self.beams.len(), BeamTerms::EMPTY);
            let mut chunks: Vec<&mut [BeamTerms]> = terms.chunks_mut(PAR_CHUNK).collect();
            ordered_chunks(
                &mut chunks,
                self.beams.len() / ASSEMBLY_PER_HELPER,
                |c, out| {
                    arch.dispatch(ComputeChunk {
                        sim: self,
                        r,
                        first: c * PAR_CHUNK,
                        out,
                    });
                },
                |c, out| {
                    for (k, out) in out.iter().enumerate() {
                        if out.live {
                            let i = c * PAR_CHUNK + k;
                            phi += out.energy;
                            sys.add_force(&self.beams[i], &out.force);
                            sys.lin.add_block(i, &out.block);
                        }
                    }
                },
            );
            sys.terms = terms;
        } else {
            phi += arch.dispatch(AssembleSerial { sim: self, r, sys });
        }
        sys.lin.stats.assembly_seconds += t0.elapsed().as_secs_f64();
        phi
    }

    /// Move every free body by `t`·`delta`, starting from `base` if given.
    pub(super) fn apply_delta(
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
    pub(super) fn solve_direct(&mut self, h: f64) {
        let mut sys = match self.direct.take() {
            Some(sys) => sys,
            None => match DirectSystem::new(&self.bodies, &self.beams, &self.config) {
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
        // Responses at the last state the step check evaluated; current
        // only while nothing has moved since.
        let mut checked = std::mem::take(&mut self.checked);
        checked.resize(self.beams.len(), (0.0, Response::default()));
        let mut checked_current = false;
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
            checked_current = false;
            if !check {
                break;
            }
            // Armijo test. For a quadratic Φ the decrease is slope·(1 − ρ/2),
            // where ρ is the true curvature along the step over the curvature
            // the matrix assumed: 1 for an exact Newton step, 2 at the edge
            // of divergence.
            let slope: f64 = sys.rhs.iter().zip(&sys.delta).map(|(r, d)| r * d).sum();
            let ok = |phi1: f64, t: f64| phi1 <= phi0 - ARMIJO * t * slope + 1e-10 * phi0.abs();
            let phi1 = self.objective(h, &mut checked);
            checked_current = true;
            if ok(phi1, 1.0) {
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
                if ok(self.objective(h, &mut checked), t) {
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
        self.checked = checked;
        self.checked_for = checked_current.then(|| self.plastic_ratio(h).0);
        self.direct = Some(sys);
    }
}
