//! The direct path ([`Solver::Direct`](super::Solver)): assemble every member
//! into one sparse system per substep, solve it, and check the step against
//! the true energy with a backtracking line search.

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
    rhs: Vec<f64>,
    delta: Vec<f64>,
    /// Scratch for the step check: body states before the step.
    base: Vec<(Vec3, UnitQuaternion<f64>)>,
    /// Scratch for assembly: every beam's contribution.
    terms: Vec<BeamTerms>,
}

/// One beam's contribution to the substep system.
#[derive(Clone)]
pub(super) struct BeamTerms {
    live: bool,
    energy: f64,
    /// Elastic force on [x_a, φ_a, x_b, φ_b].
    force: [f64; 12],
    /// Packed lower triangle of the 12x12 stiffness block.
    block: [f64; BLOCK_NNZ],
}

/// Beams per parallel work item.
pub(super) const PAR_CHUNK: usize = 64;

impl DirectSystem {
    pub(super) fn new(bodies: &[Body], beams: &[Beam], backend: Backend) -> Option<Self> {
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
            lin: LinSolver::new(ndof, &blocks, backend)?,
            trans,
            rot,
            rhs: vec![0.0; ndof],
            delta: vec![0.0; ndof],
            base: Vec::new(),
            terms: vec![
                BeamTerms {
                    live: false,
                    energy: 0.0,
                    force: [0.0; 12],
                    block: [0.0; BLOCK_NNZ],
                };
                beams.len()
            ],
        })
    }
}
impl DynamicSim {
    /// The substep objective Φ = ½|x − x̃|²_M / h² + Σ beam energy at the
    /// current state (same residual definitions as `assemble_direct`).
    pub(super) fn objective(&self, h: f64) -> f64 {
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
        let energy = |i: usize| {
            if self.beams[i].broken {
                0.0
            } else {
                self.beam_energy(i, r)
            }
        };
        if self.runs_parallel() {
            // Summed per fixed-size chunk, then in chunk order: the same
            // result on any number of threads.
            let n = self.beams.len();
            let parts: Vec<f64> = (0..n.div_ceil(PAR_CHUNK))
                .into_par_iter()
                .map(|c| {
                    (c * PAR_CHUNK..((c + 1) * PAR_CHUNK).min(n))
                        .map(energy)
                        .sum::<f64>()
                })
                .collect();
            phi += parts.iter().sum::<f64>();
        } else {
            phi += (0..self.beams.len()).map(energy).sum::<f64>();
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
        // Per-beam work has no shared state: compute every beam's block and
        // forces (on several cores for large structures), then scatter in
        // beam order so the result does not depend on the thread count.
        let compute = |(i, out): (usize, &mut BeamTerms)| {
            out.live = !self.beams[i].broken;
            if !out.live {
                return;
            }
            let (a, b) = (self.beams[i].a, self.beams[i].b);
            let kb = &mut out.block;
            *kb = [0.0; BLOCK_NNZ];
            out.force = [0.0; 12];
            let (terms, energy) = self.beam_constraints(i, r);
            out.energy = energy;
            for (f, k, g) in terms {
                let flat: [f64; 12] = std::array::from_fn(|j| g[j / 3][j % 3]);
                let mut t = 0;
                for r in 0..12 {
                    let kr = k * flat[r];
                    out.force[r] += f * flat[r];
                    for &fc in &flat[..=r] {
                        kb[t] += kr * fc;
                        t += 1;
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
        };
        if self.runs_parallel() {
            sys.terms
                .par_iter_mut()
                .enumerate()
                .with_min_len(PAR_CHUNK)
                .for_each(compute);
        } else {
            sys.terms.iter_mut().enumerate().for_each(compute);
        }
        for (i, out) in sys.terms.iter().enumerate() {
            if !out.live {
                continue;
            }
            phi += out.energy;
            let (a, b) = (self.beams[i].a, self.beams[i].b);
            let slots = [sys.trans[a], sys.rot[a], sys.trans[b], sys.rot[b]];
            for (s, slot) in slots.iter().enumerate() {
                if let Some(d) = slot {
                    for k3 in 0..3 {
                        sys.rhs[d + k3] -= out.force[s * 3 + k3];
                    }
                }
            }
            sys.lin.add_block(i, &out.block);
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
            None => {
                match DirectSystem::new(&self.bodies, &self.beams, self.config.linear_backend) {
                    Some(sys) => sys,
                    None => return,
                }
            }
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
}
