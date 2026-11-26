//! Linear algebra for the real-time solver: a symmetric positive definite
//! system made of a diagonal plus one dense 12x12 block per beam, solved
//! every substep.
//!
//! The sparsity pattern, the fill-reducing ordering and the symbolic
//! factorisation are built once and never change: a broken beam simply
//! contributes zeros. Each solve then
//!
//! 1. scatters the new values straight into the matrix storage, and
//! 2. solves with conjugate gradients preconditioned by the most recent
//!    Cholesky factorisation.
//!
//! Between substeps the matrix only changes by small member rotations and by
//! the occasional broken beam (a rank 6 change), so a factorisation that is
//! many substeps old is still an excellent preconditioner and CG converges to
//! the exact solution of the current system in a few iterations, each costing
//! one sparse product and one pair of triangular solves. A new numeric
//! factorisation is computed only when CG starts to need more iterations than
//! a refactorisation is worth.

use crate::blockchol::BlockChol;
use faer::dyn_stack::{MemBuffer, MemStack, StackReq};
use faer::linalg::cholesky::llt::factor::LltRegularization;
use faer::sparse::linalg::cholesky::{
    factorize_symbolic_cholesky, CholeskySymbolicParams, LltRef, SymbolicCholesky,
    SymbolicCholeskyRaw, SymmetricOrdering,
};
use faer::sparse::{SparseColMat, Triplet};
use faer::{ColMut, Conj, Par, Side};
use nalgebra::{Cholesky, Matrix6, Vector6};
use std::collections::HashMap;

/// Entries in the lower triangle of a symmetric 12x12 block.
pub const BLOCK_NNZ: usize = 78;

/// Index into a packed lower triangle (row-major, `r >= c`).
#[inline]
pub const fn tri(r: usize, c: usize) -> usize {
    r * (r + 1) / 2 + c
}

const ABSENT: u32 = u32::MAX;
const MIN_HOLD: usize = 4;
const MAX_HOLD: usize = 64;
/// `Backend::Auto` switches to block Jacobi when a scalar factorisation
/// would cost more than this many times the flops of one triangular solve
/// pair. Measured with the node-block factorisation: at 17 (a 3x3x8 bay
/// tower) Cholesky is 1.4 to 1.8 times faster per frame, at 43 (5x5x12) the
/// two are level, at 112 (8x8x16) block Jacobi is twice as fast in a
/// collapse.
const JACOBI_FILL_RATIO: f64 = 30.0;
const JACOBI_MAX_ITERS: usize = 500;
/// Scalar type of the block-Jacobi iteration.
type S = f32;

/// Which linear solver handles the substep systems.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Backend {
    /// Pick by the fill of the Cholesky factorisation: large 3D frames,
    /// whose factors fill in heavily, get `BlockJacobi`; small frames and
    /// chain-like structures such as truss bridges get `Cholesky`.
    #[default]
    Auto,
    /// Conjugate gradients preconditioned by the most recent sparse Cholesky
    /// factorisation, computed on 6x6 node blocks (`blockchol.rs`).
    Cholesky,
    /// Conjugate gradients preconditioned by the inverse of each node's 6x6
    /// block, warm-started from the previous solution. Never factorises, so
    /// its cost is linear in the member count and has no spikes.
    BlockJacobi,
    /// `Cholesky` with faer's scalar sparse factorisation, the
    /// implementation before the node-block one. Kept for comparison.
    CholeskyScalar,
}

/// How hard the solver works per call.
#[derive(Clone, Copy, Debug)]
pub struct Policy {
    /// CG stops when the residual norm has dropped below this fraction of
    /// the right-hand side's norm.
    pub tolerance: f64,
    /// Cost of a numeric factorisation, in CG iterations. Once the extra
    /// iterations spent because the factorisation is old add up to this, a
    /// new one is computed before the next solve. 0 = estimate it from the
    /// fill of the factorisation.
    pub refactor_cost: usize,
    /// Give up on CG at this many iterations: refactorise and solve directly.
    /// 0 = twice the refactor cost.
    pub max_iters: usize,
    /// Refactorise at least every this many solves (0 = no limit).
    pub max_age: usize,
}

/// Counters since construction, for benchmarks and tuning.
#[derive(Clone, Copy, Debug, Default)]
pub struct Stats {
    pub solves: usize,
    pub factorizations: usize,
    pub cg_iterations: usize,
    /// Solves where CG hit `max_iters` and a direct solve took over.
    pub fallbacks: usize,
    /// Seconds spent in numeric factorisation, in solves (CG and triangular
    /// solves) and, filled in by the caller, in assembling the matrix.
    pub factor_seconds: f64,
    pub solve_seconds: f64,
    pub assembly_seconds: f64,
    /// Constants of the Cholesky backends (0 for block Jacobi): scalar
    /// entries stored in the factor, the sum of squared column counts of
    /// the equivalent scalar factorisation, and the factorisation cost in
    /// CG iterations that `Policy::refactor_cost == 0` stands for.
    pub factor_entries: usize,
    pub factor_flops: f64,
    pub auto_refactor_cost: usize,
}

/// The matrix and the method behind a [`LinSolver`].
enum Engine {
    Block(BlockChol),
    Scalar(ScalarChol),
    Jacobi(BlockJacobi),
}

pub struct LinSolver {
    n: usize,
    engine: Engine,
    has_factor: bool,
    want_factor: bool,
    /// Estimated cost of a factorisation in CG iterations, from the fill.
    auto_refactor_cost: usize,
    age: usize,
    /// CG iterations of the first solve after the last factorisation, and
    /// the iterations spent beyond that since.
    base_iters: usize,
    wasted_iters: usize,
    /// While the matrix changes too fast for an old factorisation to pay
    /// (a violent collapse), factorise every solve for `direct_left` solves,
    /// then probe again; `direct_hold` is the length of the next such run.
    direct_left: usize,
    direct_hold: usize,
    /// Right-hand side and solution in the block engine's vector layout.
    bw: Vec<f64>,
    xw: Vec<f64>,
    r: Vec<f64>,
    z: Vec<f64>,
    p: Vec<f64>,
    ap: Vec<f64>,
    pub stats: Stats,
}

impl LinSolver {
    /// `blocks[i]` holds the first DOF of each of the four 3-DOF slots of
    /// beam `i` (translation a, rotation a, translation b, rotation b);
    /// `None` marks a fixed slot.
    pub fn new(n: usize, blocks: &[[Option<usize>; 4]], backend: Backend) -> Option<Self> {
        let mut stats = Stats::default();
        let mut auto_refactor_cost = 0;
        let engine = match backend {
            Backend::BlockJacobi => Engine::Jacobi(BlockJacobi::new(n, blocks)),
            Backend::CholeskyScalar => {
                let s = ScalarChol::new(n, blocks)?;
                stats.factor_entries = s.l_values.len();
                stats.factor_flops = s.factor_flops;
                auto_refactor_cost = s.auto_refactor_cost;
                Engine::Scalar(s)
            }
            Backend::Auto | Backend::Cholesky => {
                let layout = NodeLayout::new(n, blocks);
                let chol = BlockChol::new(&layout);
                let fill = chol.scalar_factor_flops / chol.scalar_solve_flops;
                if backend == Backend::Auto && fill > JACOBI_FILL_RATIO {
                    Engine::Jacobi(BlockJacobi::from_layout(layout))
                } else {
                    stats.factor_entries = chol.factor_entries();
                    stats.factor_flops = chol.scalar_factor_flops;
                    auto_refactor_cost =
                        chol.factor_cost_in_iterations().clamp(2.0, 40.0).round() as usize;
                    Engine::Block(chol)
                }
            }
        };
        stats.auto_refactor_cost = auto_refactor_cost;
        // Length of the vectors CG works on.
        let (m, mw) = match &engine {
            Engine::Block(c) => (c.dim(), c.dim()),
            Engine::Scalar(_) => (n, 0),
            Engine::Jacobi(_) => (0, 0),
        };
        Some(Self {
            n,
            engine,
            has_factor: false,
            want_factor: true,
            auto_refactor_cost,
            age: 0,
            base_iters: 0,
            wasted_iters: 0,
            direct_left: 0,
            direct_hold: MIN_HOLD,
            bw: vec![0.0; mw],
            xw: vec![0.0; mw],
            r: vec![0.0; m],
            z: vec![0.0; m],
            p: vec![0.0; m],
            ap: vec![0.0; m],
            stats,
        })
    }

    pub fn ndof(&self) -> usize {
        self.n
    }

    /// Start a new matrix: everything zero.
    pub fn clear(&mut self) {
        match &mut self.engine {
            Engine::Block(c) => c.clear(),
            Engine::Scalar(s) => s.mat.val_mut().fill(0.0),
            Engine::Jacobi(j) => j.clear(),
        }
    }

    #[inline]
    pub fn add_diagonal(&mut self, dof: usize, v: f64) {
        match &mut self.engine {
            Engine::Block(c) => c.add_diagonal(dof, v),
            Engine::Scalar(s) => {
                let p = s.diag_pos[dof] as usize;
                s.mat.val_mut()[p] += v;
            }
            Engine::Jacobi(j) => j.add_diagonal(dof, v),
        }
    }

    /// Add a beam's block, given as its packed lower triangle (see [`tri`]).
    #[inline]
    pub fn add_block(&mut self, block: usize, lower: &[f64; BLOCK_NNZ]) {
        match &mut self.engine {
            Engine::Block(c) => c.add_block(block, lower),
            Engine::Scalar(s) => {
                let vals = s.mat.val_mut();
                for (&p, &v) in s.block_pos[block].iter().zip(lower) {
                    if p != ABSENT {
                        vals[p as usize] += v;
                    }
                }
            }
            Engine::Jacobi(j) => j.add_block(block, lower),
        }
    }

    /// Ask for a fresh factorisation at the next solve.
    pub fn invalidate(&mut self) {
        self.want_factor = true;
    }

    /// Solve A x = b for the matrix assembled since the last [`clear`](Self::clear).
    /// Returns false if the matrix is
    /// not positive definite.
    pub fn solve(&mut self, b: &[f64], x: &mut [f64], policy: &Policy) -> bool {
        let t0 = std::time::Instant::now();
        let f0 = self.stats.factor_seconds;
        let ok = match &mut self.engine {
            Engine::Jacobi(j) => {
                self.stats.solves += 1;
                let iters = j.solve(b, x, policy.tolerance);
                self.stats.cg_iterations += iters.unwrap_or(0);
                iters.is_some()
            }
            Engine::Scalar(_) => self.solve_inner(b, x, policy),
            Engine::Block(c) => {
                // CG runs in the block engine's own vector layout.
                c.finish();
                let (mut bw, mut xw) = (std::mem::take(&mut self.bw), std::mem::take(&mut self.xw));
                c.load(b, &mut bw);
                let ok = self.solve_inner(&bw, &mut xw, policy);
                if let Engine::Block(c) = &self.engine {
                    c.store(&xw, x);
                }
                (self.bw, self.xw) = (bw, xw);
                ok
            }
        };
        self.stats.solve_seconds += t0.elapsed().as_secs_f64() - (self.stats.factor_seconds - f0);
        ok
    }

    fn solve_inner(&mut self, b: &[f64], x: &mut [f64], policy: &Policy) -> bool {
        let refactor_cost = if policy.refactor_cost == 0 {
            self.auto_refactor_cost
        } else {
            policy.refactor_cost
        };
        let max_iters = if policy.max_iters == 0 {
            2 * refactor_cost.max(3)
        } else {
            policy.max_iters
        };
        self.stats.solves += 1;
        if self.n == 0 {
            return true;
        }
        if self.direct_left > 0 {
            self.direct_left -= 1;
            self.want_factor = true;
        }
        let stale = self.has_factor
            && !self.want_factor
            && (policy.max_age == 0 || self.age < policy.max_age);
        if !stale {
            if !self.factor() {
                return false;
            }
            x.copy_from_slice(b);
            self.apply_factor(x);
            self.age = 1;
            return true;
        }
        self.age += 1;
        let iters = self.pcg(b, x, policy.tolerance, max_iters);
        self.stats.cg_iterations += iters.unwrap_or(max_iters);
        match iters {
            Some(k) => {
                if self.age == 2 {
                    self.base_iters = k;
                    if k >= refactor_cost {
                        self.hold_direct();
                    } else {
                        self.direct_hold = MIN_HOLD;
                    }
                }
                self.wasted_iters += k.saturating_sub(self.base_iters);
                if self.wasted_iters >= refactor_cost {
                    self.want_factor = true;
                }
                true
            }
            None => {
                // The old factorisation is no longer a useful preconditioner.
                self.stats.fallbacks += 1;
                self.hold_direct();
                if !self.factor() {
                    return false;
                }
                // r holds the residual of the current CG iterate x.
                self.z.copy_from_slice(&self.r);
                let mut z = std::mem::take(&mut self.z);
                self.apply_factor(&mut z);
                for (xi, zi) in x.iter_mut().zip(&z) {
                    *xi += zi;
                }
                self.z = z;
                self.age = 1;
                true
            }
        }
    }

    fn hold_direct(&mut self) {
        self.direct_left = self.direct_hold;
        self.direct_hold = (2 * self.direct_hold).min(MAX_HOLD);
    }

    fn factor(&mut self) -> bool {
        self.stats.factorizations += 1;
        let t0 = std::time::Instant::now();
        let ok = match &mut self.engine {
            Engine::Block(c) => c.factor(),
            Engine::Scalar(s) => s.factor(),
            Engine::Jacobi(_) => unreachable!("block Jacobi never factorises"),
        };
        self.stats.factor_seconds += t0.elapsed().as_secs_f64();
        self.has_factor = ok;
        self.want_factor = !ok;
        self.wasted_iters = 0;
        ok
    }

    /// v <- (L Lᵀ)⁻¹ v with the stored factorisation.
    fn apply_factor(&mut self, v: &mut [f64]) {
        match &mut self.engine {
            Engine::Block(c) => c.apply(v),
            Engine::Scalar(s) => s.apply(v),
            Engine::Jacobi(_) => unreachable!("block Jacobi never factorises"),
        }
    }

    /// y <- A v
    fn matvec(&self, v: &[f64], y: &mut [f64]) {
        match &self.engine {
            Engine::Block(c) => c.matvec(v, y),
            Engine::Scalar(s) => s.matvec(v, y),
            Engine::Jacobi(_) => unreachable!("block Jacobi has its own iteration"),
        }
    }

    /// Preconditioned CG from x = 0. Returns the iteration count, or None if
    /// it did not converge within `max_iters`; `self.r` then holds the
    /// residual of the returned x.
    fn pcg(&mut self, b: &[f64], x: &mut [f64], tolerance: f64, max_iters: usize) -> Option<usize> {
        let dot = |a: &[f64], b: &[f64]| a.iter().zip(b).map(|(x, y)| x * y).sum::<f64>();
        let (mut r, mut z, mut p, mut ap) = (
            std::mem::take(&mut self.r),
            std::mem::take(&mut self.z),
            std::mem::take(&mut self.p),
            std::mem::take(&mut self.ap),
        );
        let m = b.len();
        x.fill(0.0);
        r.copy_from_slice(b);
        let target = dot(b, b) * tolerance * tolerance;
        let mut result = if target > 0.0 { None } else { Some(0) };
        let mut rz = 0.0;
        if result.is_none() {
            for k in 1..=max_iters {
                z.copy_from_slice(&r);
                self.apply_factor(&mut z);
                let rz_new = dot(&r, &z);
                if k == 1 {
                    p.copy_from_slice(&z);
                } else {
                    let beta = rz_new / rz;
                    for i in 0..m {
                        p[i] = z[i] + beta * p[i];
                    }
                }
                rz = rz_new;
                self.matvec(&p, &mut ap);
                let pap = dot(&p, &ap);
                if pap <= 0.0 || !pap.is_finite() {
                    break;
                }
                let alpha = rz / pap;
                for i in 0..m {
                    x[i] += alpha * p[i];
                    r[i] -= alpha * ap[i];
                }
                if dot(&r, &r) <= target {
                    result = Some(k);
                    break;
                }
            }
        }
        (self.r, self.z, self.p, self.ap) = (r, z, p, ap);
        result
    }
}

/// faer's scalar sparse Cholesky on a CSC matrix: the pattern, the ordering
/// and the symbolic factorisation are built once.
struct ScalarChol {
    n: usize,
    /// Lower triangle of the system matrix; values are rewritten every solve.
    mat: SparseColMat<usize, f64>,
    diag_pos: Vec<u32>,
    /// Per block, the CSC value slot of each packed lower-triangle entry.
    block_pos: Vec<[u32; BLOCK_NNZ]>,
    symbolic: SymbolicCholesky<usize>,
    l_values: Vec<f64>,
    stack: MemBuffer,
    /// Sum of squared column counts of the factor.
    factor_flops: f64,
    auto_refactor_cost: usize,
}

impl ScalarChol {
    fn new(n: usize, blocks: &[[Option<usize>; 4]]) -> Option<Self> {
        let dof = |slots: &[Option<usize>; 4], l: usize| slots[l / 3].map(|d| d + l % 3);
        let mut trip = Vec::with_capacity(n + blocks.len() * BLOCK_NNZ);
        for i in 0..n {
            trip.push(Triplet::new(i, i, 0.0));
        }
        for slots in blocks {
            for r in 0..12 {
                for c in 0..=r {
                    if let (Some(gr), Some(gc)) = (dof(slots, r), dof(slots, c)) {
                        trip.push(Triplet::new(gr.max(gc), gr.min(gc), 0.0));
                    }
                }
            }
        }
        let mat = SparseColMat::<usize, f64>::try_new_from_triplets(n, n, &trip).ok()?;
        let sym = mat.symbolic();
        let (cp, ri) = (sym.col_ptr(), sym.row_idx());
        let find = |r: usize, c: usize| {
            (cp[c]
                + ri[cp[c]..cp[c + 1]]
                    .binary_search(&r)
                    .expect("entry in pattern")) as u32
        };
        let diag_pos = (0..n).map(|i| find(i, i)).collect();
        let block_pos = blocks
            .iter()
            .map(|slots| {
                let mut pos = [ABSENT; BLOCK_NNZ];
                for r in 0..12 {
                    for c in 0..=r {
                        if let (Some(gr), Some(gc)) = (dof(slots, r), dof(slots, c)) {
                            pos[tri(r, c)] = find(gr.max(gc), gr.min(gc));
                        }
                    }
                }
                pos
            })
            .collect();
        let symbolic = factorize_symbolic_cholesky(
            sym,
            Side::Lower,
            SymmetricOrdering::Amd,
            CholeskySymbolicParams::default(),
        )
        .ok()?;
        let (mut f_factor, mut f_solve) = (0.0f64, 2.0 * mat.compute_nnz() as f64);
        let mut count = |c: usize| {
            f_factor += (c * c) as f64;
            f_solve += 4.0 * c as f64;
        };
        match symbolic.raw() {
            SymbolicCholeskyRaw::Simplicial(s) => {
                for w in s.col_ptr().windows(2) {
                    count(w[1] - w[0]);
                }
            }
            SymbolicCholeskyRaw::Supernodal(s) => {
                for i in 0..s.n_supernodes() {
                    let width = s.supernode_end()[i] - s.supernode_begin()[i];
                    let below = s.supernode(i).pattern().len();
                    for j in 0..width {
                        count(below + width - j);
                    }
                }
            }
        }
        // Measured on frames of 100 to 5,000 members: per flop, a supernodal
        // factorisation runs about three times faster than the triangular
        // solves, a simplicial one slightly slower.
        let speed = match symbolic.raw() {
            SymbolicCholeskyRaw::Simplicial(_) => 1.2,
            SymbolicCholeskyRaw::Supernodal(_) => 0.33,
        };
        let auto_refactor_cost = (speed * f_factor / f_solve).clamp(2.0, 40.0).round() as usize;
        let par = Par::Seq;
        let req = StackReq::any_of(&[
            symbolic.factorize_numeric_llt_scratch::<f64>(par, Default::default()),
            symbolic.solve_in_place_scratch::<f64>(1, par),
        ]);
        Some(Self {
            n,
            l_values: vec![0.0; symbolic.len_val()],
            stack: MemBuffer::try_new(req).ok()?,
            mat,
            diag_pos,
            block_pos,
            symbolic,
            factor_flops: f_factor,
            auto_refactor_cost,
        })
    }

    fn factor(&mut self) -> bool {
        let stack = MemStack::new(&mut self.stack);
        self.symbolic
            .factorize_numeric_llt(
                &mut self.l_values,
                self.mat.as_ref(),
                Side::Lower,
                LltRegularization::default(),
                Par::Seq,
                stack,
                Default::default(),
            )
            .is_ok()
    }

    /// v <- (L Lᵀ)⁻¹ v with the stored factorisation.
    fn apply(&mut self, v: &mut [f64]) {
        let stack = MemStack::new(&mut self.stack);
        LltRef::new(&self.symbolic, &self.l_values).solve_in_place_with_conj(
            Conj::No,
            ColMut::from_slice_mut(v).as_mat_mut(),
            Par::Seq,
            stack,
        );
    }

    /// y <- A v using the stored lower triangle.
    fn matvec(&self, v: &[f64], y: &mut [f64]) {
        let sym = self.mat.symbolic();
        let (cp, ri) = (sym.col_ptr(), sym.row_idx());
        let vals = self.mat.val();
        y.fill(0.0);
        for c in 0..self.n {
            let vc = v[c];
            let mut acc = 0.0;
            // Sorted rows: the diagonal comes first in each column.
            let (start, end) = (cp[c], cp[c + 1]);
            y[c] += vals[start] * vc;
            for k in start + 1..end {
                let r = ri[k];
                y[r] += vals[k] * vc;
                acc += vals[k] * v[r];
            }
            y[c] += acc;
        }
    }
}

/// The nodes behind a DOF numbering: every pair of slots (translation,
/// rotation) that a beam end refers to is one node of six components, some
/// of which may be fixed.
pub(crate) struct NodeLayout {
    /// Compact DOF -> (node, component).
    pub loc: Vec<(u32, u8)>,
    /// Per node, which of its six components are unknowns.
    pub free: Vec<[bool; 6]>,
    /// Per beam: its end nodes (None = fully fixed end).
    pub ends: Vec<[Option<u32>; 2]>,
}

impl NodeLayout {
    pub fn new(n: usize, blocks: &[[Option<usize>; 4]]) -> Self {
        let mut ids: HashMap<(Option<usize>, Option<usize>), u32> = HashMap::new();
        let mut free = Vec::new();
        let mut loc = vec![(0u32, 0u8); n];
        let mut ends = Vec::with_capacity(blocks.len());
        for slots in blocks {
            let mut e = [None; 2];
            for end in 0..2 {
                let key = (slots[2 * end], slots[2 * end + 1]);
                if key == (None, None) {
                    continue;
                }
                let id = *ids.entry(key).or_insert_with(|| {
                    let id = free.len() as u32;
                    let mut f = [false; 6];
                    for (half, slot) in [key.0, key.1].into_iter().enumerate() {
                        if let Some(d) = slot {
                            for k in 0..3 {
                                f[half * 3 + k] = true;
                                loc[d + k] = (id, (half * 3 + k) as u8);
                            }
                        }
                    }
                    free.push(f);
                    id
                });
                e[end] = Some(id);
            }
            ends.push(e);
        }
        // DOFs that no beam touches (cannot happen for a node with mass, but
        // keep the numbering total): give each its own node.
        let mut seen = vec![false; n];
        for slots in blocks {
            for d in slots.iter().flatten() {
                for k in 0..3 {
                    seen[d + k] = true;
                }
            }
        }
        let mut d = 0;
        while d < n {
            if !seen[d] {
                let id = free.len() as u32;
                let mut f = [false; 6];
                for k in 0..3 {
                    f[k] = true;
                    loc[d + k] = (id, k as u8);
                    seen[d + k] = true;
                }
                free.push(f);
            }
            d += 3;
        }
        Self { loc, free, ends }
    }
}

/// Conjugate gradients on node blocks: the matrix is one 6x6 block per node
/// plus one 6x6 coupling block per beam, preconditioned by the inverse of the
/// node blocks. Fixed DOFs stay in the numbering with a unit diagonal.
struct BlockJacobi {
    /// Node blocks in double precision, as assembled; `diag` is the copy
    /// the iteration uses.
    diag64: Vec<Matrix6<f64>>,
    /// Compact DOF -> (node, component).
    loc: Vec<(u32, u8)>,
    free: Vec<[bool; 6]>,
    diag: Vec<Matrix6<S>>,
    inv: Vec<Matrix6<S>>,
    /// Per beam: its end nodes (None = fully fixed end).
    ends: Vec<[Option<u32>; 2]>,
    /// Per beam: coupling block, rows at end b, columns at end a.
    off: Vec<Matrix6<S>>,
    live: Vec<bool>,
    /// Previous solution, used as the starting guess.
    x: Vec<Vector6<S>>,
    b: Vec<Vector6<S>>,
    r: Vec<Vector6<S>>,
    z: Vec<Vector6<S>>,
    p: Vec<Vector6<S>>,
    ap: Vec<Vector6<S>>,
}

impl BlockJacobi {
    fn new(n: usize, blocks: &[[Option<usize>; 4]]) -> Self {
        Self::from_layout(NodeLayout::new(n, blocks))
    }

    fn from_layout(layout: NodeLayout) -> Self {
        let NodeLayout { loc, free, ends } = layout;
        let nn = free.len();
        let zero = vec![Vector6::zeros(); nn];
        Self {
            loc,
            free,
            diag64: vec![Matrix6::zeros(); nn],
            diag: vec![Matrix6::zeros(); nn],
            inv: vec![Matrix6::zeros(); nn],
            off: vec![Matrix6::zeros(); ends.len()],
            live: vec![false; ends.len()],
            ends,
            x: zero.clone(),
            b: zero.clone(),
            r: zero.clone(),
            z: zero.clone(),
            p: zero.clone(),
            ap: zero,
        }
    }

    fn clear(&mut self) {
        self.diag64.fill(Matrix6::zeros());
        self.live.fill(false);
    }

    #[inline]
    fn add_diagonal(&mut self, dof: usize, v: f64) {
        let (nd, k) = self.loc[dof];
        self.diag64[nd as usize][(k as usize, k as usize)] += v;
    }

    fn add_block(&mut self, beam: usize, lower: &[f64; BLOCK_NNZ]) {
        let [ea, eb] = self.ends[beam];
        for (end, o) in [(ea, 0), (eb, 6)] {
            let Some(nd) = end else { continue };
            let nd = nd as usize;
            let f = self.free[nd];
            let d = &mut self.diag64[nd];
            for r in 0..6 {
                for c in 0..=r {
                    if f[r] && f[c] {
                        let v = lower[tri(o + r, o + c)];
                        d[(r, c)] += v;
                        if r != c {
                            d[(c, r)] += v;
                        }
                    }
                }
            }
        }
        if let (Some(a), Some(b)) = (ea, eb) {
            let (fa, fb) = (self.free[a as usize], self.free[b as usize]);
            let m = &mut self.off[beam];
            for r in 0..6 {
                for c in 0..6 {
                    m[(r, c)] = if fb[r] && fa[c] {
                        lower[tri(6 + r, c)] as S
                    } else {
                        0.0
                    };
                }
            }
            self.live[beam] = true;
        }
    }

    fn matvec(&self, v: &[Vector6<S>], y: &mut [Vector6<S>]) {
        for ((y, d), v) in y.iter_mut().zip(&self.diag).zip(v) {
            *y = d * v;
        }
        for (i, e) in self.ends.iter().enumerate() {
            if !self.live[i] {
                continue;
            }
            let (a, b) = (e[0].unwrap() as usize, e[1].unwrap() as usize);
            let m = &self.off[i];
            y[b] += m * v[a];
            y[a] += m.tr_mul(&v[b]);
        }
    }

    /// Returns the CG iteration count, or None if a node block is not
    /// positive definite.
    fn solve(&mut self, rhs: &[f64], out: &mut [f64], tolerance: f64) -> Option<usize> {
        for i in 0..self.diag64.len() {
            let d = &mut self.diag64[i];
            for k in 0..6 {
                if !self.free[i][k] {
                    d[(k, k)] = 1.0;
                }
            }
            self.inv[i] = Cholesky::new(*d)?.inverse().cast::<S>();
            self.diag[i] = d.cast::<S>();
        }
        let dot = |a: &[Vector6<S>], b: &[Vector6<S>]| {
            a.iter().zip(b).map(|(x, y)| x.dot(y) as f64).sum::<f64>()
        };
        let (mut x, mut b, mut r, mut z, mut p, mut ap) = (
            std::mem::take(&mut self.x),
            std::mem::take(&mut self.b),
            std::mem::take(&mut self.r),
            std::mem::take(&mut self.z),
            std::mem::take(&mut self.p),
            std::mem::take(&mut self.ap),
        );
        b.fill(Vector6::zeros());
        for (d, &(nd, k)) in self.loc.iter().enumerate() {
            b[nd as usize][k as usize] = rhs[d] as S;
        }
        let target = dot(&b, &b) * tolerance * tolerance;
        let mut iters = 0;
        if target > 0.0 {
            // r = b − A x, with x the previous solution.
            self.matvec(&x, &mut ap);
            for i in 0..r.len() {
                r[i] = b[i] - ap[i];
            }
            let mut rz = 0.0;
            while dot(&r, &r) > target && iters < JACOBI_MAX_ITERS {
                for i in 0..r.len() {
                    z[i] = self.inv[i] * r[i];
                }
                let rz_new = dot(&r, &z);
                if iters == 0 {
                    p.copy_from_slice(&z);
                } else {
                    let beta = (rz_new / rz) as S;
                    for i in 0..p.len() {
                        p[i] = z[i] + p[i] * beta;
                    }
                }
                rz = rz_new;
                self.matvec(&p, &mut ap);
                let pap = dot(&p, &ap);
                if pap <= 0.0 || !pap.is_finite() {
                    break;
                }
                let alpha = (rz / pap) as S;
                for i in 0..x.len() {
                    x[i] += p[i] * alpha;
                    r[i] -= ap[i] * alpha;
                }
                iters += 1;
            }
        } else {
            x.fill(Vector6::zeros());
        }
        for (d, &(nd, k)) in self.loc.iter().enumerate() {
            out[d] = x[nd as usize][k as usize] as f64;
        }
        (self.x, self.b, self.r, self.z, self.p, self.ap) = (x, b, r, z, p, ap);
        Some(iters)
    }
}
