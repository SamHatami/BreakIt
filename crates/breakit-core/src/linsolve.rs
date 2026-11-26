//! Linear algebra for the real-time solver: a symmetric positive definite
//! system made of a diagonal plus one dense 12x12 block per beam, solved
//! every substep.
//!
//! The sparsity pattern, the fill-reducing ordering and the symbolic
//! factorisation are built once and never change: a broken beam simply
//! contributes zeros. Each solve then
//!
//! 1. scatters the new values straight into the CSC value array, and
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

use faer::dyn_stack::{MemBuffer, MemStack, StackReq};
use faer::linalg::cholesky::llt::factor::LltRegularization;
use faer::sparse::linalg::cholesky::{
    factorize_symbolic_cholesky, CholeskySymbolicParams, LltRef, SymbolicCholesky,
    SymmetricOrdering,
};
use faer::sparse::{SparseColMat, Triplet};
use faer::{ColMut, Conj, Par, Side};

/// Entries in the lower triangle of a symmetric 12x12 block.
pub const BLOCK_NNZ: usize = 78;

/// Index into a packed lower triangle (row-major, `r >= c`).
#[inline]
pub const fn tri(r: usize, c: usize) -> usize {
    r * (r + 1) / 2 + c
}

const ABSENT: u32 = u32::MAX;

/// How hard the solver works per call.
#[derive(Clone, Copy, Debug)]
pub struct Policy {
    /// CG stops when the residual norm has dropped below this fraction of
    /// the right-hand side's norm.
    pub tolerance: f64,
    /// Cost of a numeric factorisation, in CG iterations. Once the extra
    /// iterations spent because the factorisation is old add up to this, a
    /// new one is computed before the next solve.
    pub refactor_cost: usize,
    /// Give up on CG at this many iterations: refactorise and solve directly.
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
}

pub struct LinSolver {
    n: usize,
    /// Lower triangle of the system matrix; values are rewritten every solve.
    mat: SparseColMat<usize, f64>,
    diag_pos: Vec<u32>,
    /// Per block, the CSC value slot of each packed lower-triangle entry.
    block_pos: Vec<[u32; BLOCK_NNZ]>,
    symbolic: SymbolicCholesky<usize>,
    l_values: Vec<f64>,
    has_factor: bool,
    want_factor: bool,
    age: usize,
    /// CG iterations of the first solve after the last factorisation, and
    /// the iterations spent beyond that since.
    base_iters: usize,
    wasted_iters: usize,
    stack: MemBuffer,
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
    pub fn new(n: usize, blocks: &[[Option<usize>; 4]]) -> Option<Self> {
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
            (cp[c] + ri[cp[c]..cp[c + 1]].binary_search(&r).expect("entry in pattern")) as u32
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
            has_factor: false,
            want_factor: true,
            age: 0,
            base_iters: 0,
            wasted_iters: 0,
            r: vec![0.0; n],
            z: vec![0.0; n],
            p: vec![0.0; n],
            ap: vec![0.0; n],
            stats: Stats::default(),
        })
    }

    pub fn ndof(&self) -> usize {
        self.n
    }

    /// Start a new matrix: everything zero.
    pub fn clear(&mut self) {
        self.mat.val_mut().fill(0.0);
    }

    #[inline]
    pub fn add_diagonal(&mut self, dof: usize, v: f64) {
        let p = self.diag_pos[dof] as usize;
        self.mat.val_mut()[p] += v;
    }

    /// Add a beam's block, given as its packed lower triangle (see [`tri`]).
    #[inline]
    pub fn add_block(&mut self, block: usize, lower: &[f64; BLOCK_NNZ]) {
        let vals = self.mat.val_mut();
        for (&p, &v) in self.block_pos[block].iter().zip(lower) {
            if p != ABSENT {
                vals[p as usize] += v;
            }
        }
    }

    /// Ask for a fresh factorisation at the next solve.
    pub fn invalidate(&mut self) {
        self.want_factor = true;
    }

    /// Solve A x = b for the matrix assembled since the last [`clear`].
    /// `b` is overwritten with scratch data. Returns false if the matrix is
    /// not positive definite.
    pub fn solve(&mut self, b: &mut [f64], x: &mut [f64], policy: &Policy) -> bool {
        let t0 = std::time::Instant::now();
        let f0 = self.stats.factor_seconds;
        let ok = self.solve_inner(b, x, policy);
        self.stats.solve_seconds +=
            t0.elapsed().as_secs_f64() - (self.stats.factor_seconds - f0);
        ok
    }

    fn solve_inner(&mut self, b: &mut [f64], x: &mut [f64], policy: &Policy) -> bool {
        self.stats.solves += 1;
        if self.n == 0 {
            return true;
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
        let iters = self.pcg(b, x, policy);
        self.stats.cg_iterations += iters.unwrap_or(policy.max_iters);
        match iters {
            Some(k) => {
                if self.age == 2 {
                    self.base_iters = k;
                }
                self.wasted_iters += k.saturating_sub(self.base_iters);
                if self.wasted_iters >= policy.refactor_cost {
                    self.want_factor = true;
                }
                true
            }
            None => {
                // The old factorisation is no longer a useful preconditioner.
                self.stats.fallbacks += 1;
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

    fn factor(&mut self) -> bool {
        self.stats.factorizations += 1;
        let t0 = std::time::Instant::now();
        let stack = MemStack::new(&mut self.stack);
        let ok = self
            .symbolic
            .factorize_numeric_llt(
                &mut self.l_values,
                self.mat.as_ref(),
                Side::Lower,
                LltRegularization::default(),
                Par::Seq,
                stack,
                Default::default(),
            )
            .is_ok();
        self.stats.factor_seconds += t0.elapsed().as_secs_f64();
        self.has_factor = ok;
        self.want_factor = !ok;
        self.wasted_iters = 0;
        ok
    }

    /// v <- (L Lᵀ)⁻¹ v with the stored factorisation.
    fn apply_factor(&mut self, v: &mut [f64]) {
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

    /// Preconditioned CG from x = 0. Returns the iteration count, or None if
    /// it did not converge within `policy.max_iters`; `self.r` then holds the
    /// residual of the returned x.
    fn pcg(&mut self, b: &[f64], x: &mut [f64], policy: &Policy) -> Option<usize> {
        let dot = |a: &[f64], b: &[f64]| a.iter().zip(b).map(|(x, y)| x * y).sum::<f64>();
        let (mut r, mut z, mut p, mut ap) = (
            std::mem::take(&mut self.r),
            std::mem::take(&mut self.z),
            std::mem::take(&mut self.p),
            std::mem::take(&mut self.ap),
        );
        x.fill(0.0);
        r.copy_from_slice(b);
        let target = dot(b, b) * policy.tolerance * policy.tolerance;
        let mut result = if target > 0.0 { None } else { Some(0) };
        let mut rz = 0.0;
        if result.is_none() {
            for k in 1..=policy.max_iters {
                z.copy_from_slice(&r);
                self.apply_factor(&mut z);
                let rz_new = dot(&r, &z);
                if k == 1 {
                    p.copy_from_slice(&z);
                } else {
                    let beta = rz_new / rz;
                    for i in 0..self.n {
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
                for i in 0..self.n {
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
