//! The direct path ([`Solver::Direct`](super::Solver)): assemble every member
//! into a sparse system per substep, solve it, and check the step against
//! the true energy with a backtracking line search. Pieces of the structure
//! that share nothing (islands) have a system each and go through this
//! side by side.

use super::assemble::{AssembleSerial, BeamTerms, ComputeChunk};
use super::pool::ordered_chunks;
use super::*;

/// Sufficient-decrease constant of the step check: accepts steps whose true
/// curvature is up to 2(1 − ARMIJO) = 1.5 times the factorised one.
pub(super) const ARMIJO: f64 = 0.25;

pub(super) const MAX_BACKTRACKS: usize = 4;

/// Extra Gauss-Newton iterations allowed in a substep after a shortened step.
pub(super) const MAX_RELINEARISE: usize = 2;

/// One linear system of [`Solver::Direct`]: a piece of the structure that
/// no live member ties to the rest, with the small debris that broke off
/// it. Broken beams keep their place in its pattern and contribute zeros.
pub(super) struct Group {
    /// Bodies of the group, ascending.
    bodies: Vec<u32>,
    /// Its beams (those live when it was built), ascending.
    beams: Vec<u32>,
    /// `beams` is every beam of the simulation, in order.
    whole: bool,
    pub(super) lin: LinSolver,
    pub(super) rhs: Vec<f64>,
    delta: Vec<f64>,
    /// Scratch for threaded assembly: every beam's contribution.
    terms: Vec<BeamTerms>,
    /// Per beam, its length and response at the last state the step check
    /// evaluated; current only while `checked_current`.
    checked: Vec<(f64, Response)>,
    checked_current: bool,
    // The substep in progress: whether the group still iterates, the
    // objective its current linearisation started from, and the
    // iterations added after shortened steps.
    active: bool,
    phi0: f64,
    relinearised: usize,
}

/// DOF numbering and the linear systems of [`Solver::Direct`]: one per
/// [`Group`]. Groups share nothing, so each is solved, checked and, if its
/// step fails, shortened and linearised again on its own.
pub(super) struct DirectSystem {
    /// DOF of each body's translation and rotation in its group's system.
    trans: Vec<Option<usize>>,
    rot: Vec<Option<usize>>,
    /// Block of each beam in its group's system.
    block: Vec<u32>,
    pub(super) groups: Vec<Group>,
    /// Break events when the groups were last checked against the breaks.
    events_seen: usize,
    /// Counters of the systems that have been replaced since.
    retired: Stats,
    /// Scratch for the step check: body states before the step.
    base: Vec<(Vec3, UnitQuaternion<f64>)>,
}

/// Where a group's bodies and beams go in its system.
#[derive(Clone, Copy)]
pub(super) struct Slots<'a> {
    trans: &'a [Option<usize>],
    rot: &'a [Option<usize>],
    pub(super) block: &'a [u32],
}

impl Slots<'_> {
    /// Subtract a beam's elastic force from the right-hand side.
    #[inline(always)]
    pub(super) fn add_force(&self, rhs: &mut [f64], bm: &Beam, force: &[f64; 12]) {
        let slots = [
            self.trans[bm.a],
            self.rot[bm.a],
            self.trans[bm.b],
            self.rot[bm.b],
        ];
        for (s, slot) in slots.iter().enumerate() {
            if let Some(d) = slot {
                for k3 in 0..3 {
                    rhs[d + k3] -= force[s * 3 + k3];
                }
            }
        }
    }
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

/// The pieces of `bodies` that the live ones of `beams` hold together, as
/// lists of bodies: each piece of at least `min_nodes` nodes, with
/// the smaller pieces added to the first. One list if there are fewer than
/// two such pieces.
fn pieces(
    bodies: &[u32],
    beams: &[u32],
    all_beams: &[Beam],
    min_nodes: usize,
    local: &mut [u32],
) -> Vec<Vec<u32>> {
    for (k, &b) in bodies.iter().enumerate() {
        local[b as usize] = k as u32;
    }
    let mut parent: Vec<u32> = (0..bodies.len() as u32).collect();
    fn find(parent: &mut [u32], mut x: u32) -> u32 {
        while parent[x as usize] != x {
            parent[x as usize] = parent[parent[x as usize] as usize];
            x = parent[x as usize];
        }
        x
    }
    for &i in beams {
        let bm = &all_beams[i as usize];
        if !bm.broken {
            let (a, b) = (
                find(&mut parent, local[bm.a]),
                find(&mut parent, local[bm.b]),
            );
            // The lower index stays the root: the outcome does not depend
            // on the order of the beams.
            parent[a.max(b) as usize] = a.min(b);
        }
    }
    let mut size = vec![0usize; bodies.len()];
    for k in 0..bodies.len() as u32 {
        size[find(&mut parent, k) as usize] += 1;
    }
    // Large pieces in the order of their first body.
    let mut piece_of = vec![u32::MAX; bodies.len()];
    let mut large = 0;
    for k in 0..bodies.len() {
        if parent[k] as usize == k && size[k] >= min_nodes {
            piece_of[k] = large;
            large += 1;
        }
    }
    if large < 2 {
        return vec![bodies.to_vec()];
    }
    let mut out = vec![Vec::new(); large as usize];
    for (k, &b) in bodies.iter().enumerate() {
        let root = find(&mut parent, k as u32) as usize;
        let piece = if piece_of[root] == u32::MAX {
            0
        } else {
            piece_of[root]
        };
        out[piece as usize].push(b);
    }
    out
}

impl Group {
    /// Number the unknowns of `bodies` and of the live beams among `beams`
    /// that join them, and write their slots. Returns the group's beams,
    /// its unknowns and the slots of each beam.
    fn number(
        bodies: &[u32],
        beams: &[u32],
        (all_bodies, all_beams): (&[Body], &[Beam]),
        (trans, rot, block): (&mut [Option<usize>], &mut [Option<usize>], &mut [u32]),
        in_group: &mut [bool],
    ) -> (Vec<u32>, usize, Vec<[Option<usize>; 4]>) {
        let mut ndof = 0;
        let mut next = |free: bool| {
            free.then(|| {
                ndof += 3;
                ndof - 3
            })
        };
        for &b in bodies {
            let body = &all_bodies[b as usize];
            trans[b as usize] = next(body.inv_mass > 0.0);
            rot[b as usize] = next(body.inv_inertia > 0.0);
            in_group[b as usize] = true;
        }
        let beams: Vec<u32> = (beams.iter().copied())
            .filter(|&i| {
                let bm = &all_beams[i as usize];
                !bm.broken && in_group[bm.a]
            })
            .collect();
        for &b in bodies {
            in_group[b as usize] = false;
        }
        let blocks = beams
            .iter()
            .enumerate()
            .map(|(k, &i)| {
                let bm = &all_beams[i as usize];
                block[i as usize] = k as u32;
                [trans[bm.a], rot[bm.a], trans[bm.b], rot[bm.b]]
            })
            .collect();
        (beams, ndof, blocks)
    }

    /// The group of a numbered system, or None if it has no unknowns.
    fn new(
        bodies: Vec<u32>,
        (beams, ndof, blocks): (Vec<u32>, usize, Vec<[Option<usize>; 4]>),
        all_beams: usize,
        config: &SimConfig,
    ) -> Option<Self> {
        let mut lin = LinSolver::new(ndof, &blocks, config.linear_backend)?;
        lin.set_simd(config.simd);
        if config.parallel_min_members == usize::MAX {
            lin.set_parallel_min_nodes(usize::MAX);
        }
        Some(Self {
            whole: beams.len() == all_beams,
            bodies,
            beams,
            lin,
            rhs: vec![0.0; ndof],
            delta: vec![0.0; ndof],
            terms: Vec::new(),
            checked: Vec::new(),
            checked_current: false,
            active: false,
            phi0: 0.0,
            relinearised: 0,
        })
    }
}

impl DirectSystem {
    pub(super) fn new(bodies: &[Body], beams: &[Beam], config: &SimConfig) -> Option<Self> {
        let mut sys = Self {
            trans: vec![None; bodies.len()],
            rot: vec![None; bodies.len()],
            block: vec![0; beams.len()],
            groups: Vec::new(),
            events_seen: 0,
            retired: Stats::default(),
            base: Vec::new(),
        };
        let all_bodies: Vec<u32> = (0..bodies.len() as u32).collect();
        let all_beams: Vec<u32> = (0..beams.len() as u32).collect();
        let mut local = vec![0u32; bodies.len()];
        let pieces = pieces(
            &all_bodies,
            &all_beams,
            beams,
            config.island_min_nodes,
            &mut local,
        );
        sys.add_groups(pieces, &all_beams, (bodies, beams, config));
        (!sys.groups.is_empty()).then_some(sys)
    }

    /// Add a group per piece. The symbolic analysis of each system is the
    /// expensive part: the pieces' are done side by side.
    fn add_groups(
        &mut self,
        pieces: Vec<Vec<u32>>,
        beams: &[u32],
        (all_bodies, all_beams, config): (&[Body], &[Beam], &SimConfig),
    ) {
        let mut in_group = vec![false; all_bodies.len()];
        let mut slots: Vec<_> = pieces
            .into_iter()
            .map(|piece| {
                let slots = (&mut self.trans[..], &mut self.rot[..], &mut self.block[..]);
                let numbered =
                    Group::number(&piece, beams, (all_bodies, all_beams), slots, &mut in_group);
                (Some((piece, numbered)), None)
            })
            .collect();
        let build = |slot: &mut (Option<_>, Option<Group>)| {
            if let Some((piece, numbered)) = slot.0.take() {
                slot.1 = Group::new(piece, numbered, all_beams.len(), config);
            }
        };
        if slots.len() > 1 && config.parallel_min_members != usize::MAX {
            let helpers = slots.len() - 1;
            // From inside the pool or not: this happens once per break-up.
            rayon::join(
                || ordered_chunks(&mut slots, helpers, |_, slot| build(slot), |_, _| {}),
                || (),
            );
        } else {
            slots.iter_mut().for_each(build);
        }
        self.groups.extend(slots.into_iter().filter_map(|s| s.1));
    }

    /// After breaks: give every piece that has come off a group, and is
    /// large enough, a system of its own.
    fn regroup(&mut self, events: usize, sim: (&[Body], &[Beam], &SimConfig)) {
        if events == self.events_seen {
            return;
        }
        self.events_seen = events;
        let min_nodes = sim.2.island_min_nodes;
        let mut local = vec![0u32; sim.0.len()];
        for g in std::mem::take(&mut self.groups) {
            if g.bodies.len() / 2 < min_nodes {
                self.groups.push(g);
                continue;
            }
            let pieces = pieces(&g.bodies, &g.beams, sim.1, min_nodes, &mut local);
            if pieces.len() < 2 {
                self.groups.push(g);
            } else {
                self.retired.add_counters(&g.lin.stats);
                self.add_groups(pieces, &g.beams, sim);
            }
        }
    }

    /// Counters of all systems, past and present, and the constants of the
    /// present ones.
    pub(super) fn stats(&self) -> Stats {
        let mut total = self.retired;
        for g in &self.groups {
            total.add_counters(&g.lin.stats);
            total.factor_entries += g.lin.stats.factor_entries;
            total.factor_flops += g.lin.stats.factor_flops;
            total.auto_refactor_cost = total.auto_refactor_cost.max(g.lin.stats.auto_refactor_cost);
        }
        total
    }
}

impl DynamicSim {
    pub(super) fn runs_parallel(&self) -> bool {
        self.beams.len() >= self.config.parallel_min_members
    }

    /// The substep objective Φ = ½|x − x̃|²_M / h² + Σ beam energy of one
    /// group at the current state (same residual definitions as
    /// `assemble_group`).
    ///
    /// Leaves every live member's length and response in `g.checked`.
    fn objective(&self, g: &mut Group, h: f64) -> f64 {
        let ih2 = 1.0 / (h * h);
        let mut phi = 0.0;
        for &bi in &g.bodies {
            let b = &self.bodies[bi as usize];
            if b.inv_mass > 0.0 {
                phi += 0.5 * ih2 / b.inv_mass * (b.x - b.x_tilde).norm_squared();
            }
            if b.inv_inertia > 0.0 {
                let dq = b.q * b.q_tilde.inverse();
                phi += 0.5 * ih2 / b.inv_inertia * 4.0 * dq.imag().norm_squared();
            }
        }
        if let Some(gr) = &self.config.ground {
            for &bi in &g.bodies {
                if let Some(t) = self.contact_terms(gr, bi as usize, h) {
                    phi += t.energy;
                }
            }
        }
        let r = self.plastic_ratio(h).0;
        // Summed per fixed-size chunk, then in chunk order, on one thread or
        // many: the same result on any number of threads.
        let n = g.beams.len();
        g.checked.resize(n, (0.0, Response::default()));
        let chunk = |beams: &[u32], out: &mut [(f64, Response)]| {
            let mut sum = 0.0;
            for (&i, slot) in beams.iter().zip(out) {
                if !self.beams[i as usize].broken {
                    *slot = self.beam_response(i as usize, r);
                    sum += slot.1.energy;
                }
            }
            sum
        };
        let chunks = (g.beams.chunks(PAR_CHUNK)).zip(g.checked.chunks_mut(PAR_CHUNK));
        if self.runs_parallel() {
            let mut parts: Vec<_> = chunks.map(|(beams, out)| (0.0, beams, out)).collect();
            let mut sum = 0.0;
            ordered_chunks(
                &mut parts,
                n / OBJECTIVE_PER_HELPER,
                |_, part| part.0 = chunk(part.1, part.2),
                |_, part| sum += part.0,
            );
            phi += sum;
        } else {
            phi += chunks.map(|(beams, out)| chunk(beams, out)).sum::<f64>();
        }
        g.checked_current = true;
        phi
    }

    /// Assemble the Gauss-Newton matrix of one group into `g.lin` and the
    /// right-hand side −∇Φ into `g.rhs`. Returns the group's objective Φ.
    fn assemble_group(&self, slots: Slots, g: &mut Group, h: f64) -> f64 {
        let t0 = std::time::Instant::now();
        let ih2 = 1.0 / (h * h);
        let mut phi = 0.0;
        g.lin.clear();
        for &bi in &g.bodies {
            let b = &self.bodies[bi as usize];
            if let Some(d) = slots.trans[bi as usize] {
                let m = ih2 / b.inv_mass;
                let r = b.x - b.x_tilde;
                phi += 0.5 * m * r.norm_squared();
                for k in 0..3 {
                    g.lin.add_diagonal(d + k, m);
                    g.rhs[d + k] = -m * r[k];
                }
            }
            if let Some(d) = slots.rot[bi as usize] {
                let m = ih2 / b.inv_inertia;
                let dq = b.q * b.q_tilde.inverse();
                let r = dq.imag() * if dq.w < 0.0 { -2.0 } else { 2.0 };
                phi += 0.5 * m * r.norm_squared();
                for k in 0..3 {
                    g.lin.add_diagonal(d + k, m);
                    g.rhs[d + k] = -m * r[k];
                }
            }
        }
        if let Some(gr) = &self.config.ground {
            // Ground contact: diagonal, non-negative, translation DOFs only,
            // so it needs no change to the sparsity pattern.
            for &bi in &g.bodies {
                let bi = bi as usize;
                if let (Some(d), Some(t)) = (slots.trans[bi], self.contact_terms(gr, bi, h)) {
                    phi += t.energy;
                    for k in 0..3 {
                        g.lin.add_diagonal(d + k, t.hess[k]);
                        g.rhs[d + k] -= t.grad[k];
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
            let Group {
                beams,
                terms,
                lin,
                rhs,
                ..
            } = g;
            terms.resize(beams.len(), BeamTerms::EMPTY);
            let mut chunks: Vec<_> = (beams.chunks(PAR_CHUNK))
                .zip(terms.chunks_mut(PAR_CHUNK))
                .collect();
            ordered_chunks(
                &mut chunks,
                beams.len() / ASSEMBLY_PER_HELPER,
                |_, (beams, out)| {
                    arch.dispatch(ComputeChunk {
                        sim: self,
                        r,
                        beams,
                        out,
                    });
                },
                |_, (beams, out)| {
                    for (&i, out) in beams.iter().zip(out.iter()) {
                        if out.live {
                            phi += out.energy;
                            slots.add_force(rhs, &self.beams[i as usize], &out.force);
                            lin.add_block(slots.block[i as usize] as usize, &out.block);
                        }
                    }
                },
            );
        } else {
            phi += arch.dispatch(AssembleSerial {
                sim: self,
                r,
                slots,
                beams: &g.beams,
                lin: &mut g.lin,
                rhs: &mut g.rhs,
            });
        }
        g.lin.stats.assembly_seconds += t0.elapsed().as_secs_f64();
        phi
    }

    /// Move every free body of a group by `t`·`delta`, starting from `base`
    /// if given.
    fn apply_delta(
        &mut self,
        slots: Slots,
        g: &mut Group,
        t: f64,
        base: Option<&[(Vec3, UnitQuaternion<f64>)]>,
    ) {
        let delta = &g.delta;
        for &bi in &g.bodies {
            let bi = bi as usize;
            let b = &mut self.bodies[bi];
            if let Some(base) = base {
                (b.x, b.q) = base[bi];
            }
            if let Some(d) = slots.trans[bi] {
                b.x += Vec3::new(delta[d], delta[d + 1], delta[d + 2]) * t;
            }
            if let Some(d) = slots.rot[bi] {
                b.q = rotate(b.q, Vec3::new(delta[d], delta[d + 1], delta[d + 2]) * t);
            }
        }
        g.checked_current = false;
    }

    /// One Gauss-Newton step of the substep problem
    /// min Φ = ½|x − x̃|²_M / h² + Σ beam energy,
    /// group by group.
    ///
    /// The linear systems are solved to `linear_tolerance` by [`LinSolver`],
    /// at the same time if there are several. With `step_check` a group's
    /// step must also pass a sufficient-decrease test on its Φ; a step that
    /// fails is shortened by backtracking and followed by another iteration
    /// of that group alone.
    pub(super) fn solve_direct(&mut self, h: f64) {
        let mut sys = match self.direct.take() {
            Some(sys) => sys,
            None => match DirectSystem::new(&self.bodies, &self.beams, &self.config) {
                Some(sys) => sys,
                None => return,
            },
        };
        sys.regroup(self.events.len(), (&self.bodies, &self.beams, &self.config));
        let check = self.config.step_check;
        let policy = Policy {
            tolerance: self.config.linear_tolerance,
            refactor_cost: self.config.refactor_cost,
            max_iters: 0,
            max_age: self.config.refactor_every,
        };
        let DirectSystem {
            trans,
            rot,
            block,
            groups,
            base,
            ..
        } = &mut sys;
        let slots = Slots { trans, rot, block };
        base.resize(
            self.bodies.len(),
            (Vec3::zeros(), UnitQuaternion::identity()),
        );
        for g in groups.iter_mut() {
            g.active = true;
            g.relinearised = 0;
            g.checked_current = false;
        }
        let par = self.runs_parallel();
        loop {
            let mut active: Vec<&mut Group> = groups.iter_mut().filter(|g| g.active).collect();
            if active.is_empty() {
                break;
            }
            for g in active.iter_mut() {
                if check {
                    for &bi in &g.bodies {
                        let b = &self.bodies[bi as usize];
                        base[bi as usize] = (b.x, b.q);
                    }
                }
                g.phi0 = self.assemble_group(slots, g, h);
            }
            // The groups' systems share nothing: solve them side by side.
            let solve = |g: &mut Group| g.active = g.lin.solve(&g.rhs, &mut g.delta, &policy);
            if par && active.len() > 1 {
                let helpers = active.len() - 1;
                ordered_chunks(&mut active, helpers, |_, g| solve(g), |_, _| {});
            } else {
                active.iter_mut().for_each(|g| solve(g));
            }
            for g in active {
                if !g.active {
                    continue;
                }
                self.apply_delta(slots, g, 1.0, None);
                if !check {
                    g.active = false;
                    continue;
                }
                // Armijo test. For a quadratic Φ the decrease is
                // slope·(1 − ρ/2), where ρ is the true curvature along the
                // step over the curvature the matrix assumed: 1 for an exact
                // Newton step, 2 at the edge of divergence.
                let slope: f64 = g.rhs.iter().zip(&g.delta).map(|(r, d)| r * d).sum();
                let phi0 = g.phi0;
                let ok = |phi1: f64, t: f64| phi1 <= phi0 - ARMIJO * t * slope + 1e-10 * phi0.abs();
                if ok(self.objective(g, h), 1.0) {
                    g.active = false;
                    continue;
                }
                // Φ is not quadratic enough over this step (typically a
                // yielding member unloads, and is five times stiffer again).
                // Shorten the step, then linearise again from there:
                // stopping here would leave the substep unconverged, and the
                // leftover residual comes back as velocity every substep.
                let mut t = 1.0;
                for _ in 0..MAX_BACKTRACKS {
                    t *= 0.5;
                    self.stats.backtracks += 1;
                    self.apply_delta(slots, g, t, Some(base));
                    if ok(self.objective(g, h), t) {
                        break;
                    }
                }
                if g.relinearised == MAX_RELINEARISE {
                    g.active = false;
                } else {
                    g.relinearised += 1;
                    self.stats.extra_iterations += 1;
                }
            }
        }
        // Hand the step check's responses to the material update.
        let current = groups.iter().all(|g| g.checked_current);
        if current {
            for g in groups.iter_mut() {
                if g.whole {
                    std::mem::swap(&mut self.checked, &mut g.checked);
                } else {
                    self.checked
                        .resize(self.beams.len(), (0.0, Response::default()));
                    for (&i, c) in g.beams.iter().zip(&g.checked) {
                        self.checked[i as usize] = *c;
                    }
                }
            }
        }
        self.checked_for = current.then(|| self.plastic_ratio(h).0);
        self.direct = Some(sys);
    }
}
