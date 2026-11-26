//! Block-Jacobi conjugate gradients for the substep systems of
//! [`crate::linsolve`]: the backend for structures whose factorisation
//! fills in too much (large 3D grids).
//!
//! The matrix is one 6x6 block per node plus one 6x6 coupling block per
//! beam, preconditioned by the inverse of the node blocks. Fixed DOFs stay
//! in the numbering with a unit diagonal. Assembly and the node-block
//! inverses are in double precision; the iteration runs in single precision
//! from the previous solution.
//!
//! Layout, chosen for SIMD and threads (see [`crate::simd`]):
//!
//! * a node vector is padded from 6 to 8 entries, so it is one AVX register
//!   (two NEON registers) and whole-vector operations run over flat slices;
//! * a block is 36 entries, column-major, and a column is read as one
//!   8-entry register that overlaps the next column (the two extra lanes
//!   meet the zero padding of the vector, or are dropped), so a
//!   block-vector product is six fused multiply-adds;
//! * every node lists the coupling blocks of its row, each stored once and
//!   read transposed from one of its two nodes. Row `i` of a product then
//!   reads only what node `i` lists and writes only `y[i]`, so rows run on
//!   any number of threads without locks. Storing each block once keeps the
//!   matrix of a 4,000-member tower inside a 2 MB L2 cache; a transposed
//!   copy made the products cheaper but the solve slower from there up.
//!
//! Nodes are processed in fixed chunks of [`CHUNK`]. Each chunk reduces its
//! own dot products and the chunk results are added in order, so the answer
//! does not depend on the number of threads: the threaded path is
//! bit-identical to the single-threaded one.

use crate::blockchol::chol_inverse;
use crate::linsolve::{tri, NodeLayout, BLOCK_NNZ};
use crate::simd::SimdLevel;
use nalgebra::Matrix6;
use pulp::{Simd, WithSimd};
use rayon::prelude::*;

/// Scalar type of the iteration.
type F = f32;
/// Entries of a padded node vector.
const W: usize = 8;
/// Entries of a 6x6 block, column-major: entry (r, c) is at `c * 6 + r`.
const B: usize = 36;
/// Extra entries at the end of a block array, so the 8-entry read of the
/// last column of the last block stays in bounds.
const SLACK: usize = W - 6;
const MAX_ITERS: usize = 500;
/// Nodes per unit of work (and per partial sum of a dot product).
const CHUNK: usize = 32;
/// Fewest nodes for which the passes are handed to the thread pool. A pass
/// over 1,400 nodes (a 3,900-member tower) takes about 30 microseconds,
/// which is less than waking the workers costs: threads made that solve
/// 1.7 times slower on a 4-core cloud machine.
pub(crate) const PAR_MIN_NODES: usize = 4096;

pub(crate) struct BlockJacobi {
    /// Node blocks in double precision, as assembled; `diag` is the copy
    /// the iteration uses.
    diag64: Vec<Matrix6<f64>>,
    /// Compact DOF -> (node, component).
    loc: Vec<(u32, u8)>,
    free: Vec<[bool; 6]>,
    /// [`B`] entries per node, plus [`SLACK`].
    diag: Vec<F>,
    inv: Vec<F>,
    /// Per beam: its end nodes (None = fully fixed end).
    ends: Vec<[Option<u32>; 2]>,
    /// [`B`] entries per beam, plus [`SLACK`]: rows at end b, columns at
    /// end a.
    off: Vec<F>,
    live: Vec<bool>,
    /// Per node, the coupling blocks of its row as (other node, 2 beam +
    /// transposed): `row[row_ptr[i]..row_ptr[i + 1]]`. End b reads the
    /// block as stored, end a reads it transposed.
    row_ptr: Vec<u32>,
    row: Vec<(u32, u32)>,
    /// Previous solution, used as the starting guess. All vectors hold
    /// [`W`] entries per node.
    x: Vec<F>,
    b: Vec<F>,
    r: Vec<F>,
    z: Vec<F>,
    p: Vec<F>,
    ap: Vec<F>,
    /// Per chunk partial sums of the two dot products of a pass.
    sums: Vec<[f64; 2]>,
    arch: pulp::Arch,
    par_min_nodes: usize,
}

/// acc += col s over one padded column.
#[inline(always)]
fn fma_col<S: Simd>(simd: S, acc: &mut [F], col: &[F], s: F) {
    let (av, at) = S::as_mut_simd_f32s(acc);
    let (cv, ct) = S::as_simd_f32s(col);
    let sv = simd.splat_f32s(s);
    for (a, c) in av.iter_mut().zip(cv) {
        *a = simd.mul_add_e_f32s(*c, sv, *a);
    }
    for (a, c) in at.iter_mut().zip(ct) {
        *a += c * s;
    }
}

/// acc += m v for the block starting at `m[0]`. Lanes 6 and 7 of `acc` end
/// up with products of the next column: callers drop them.
#[inline(always)]
fn mul_acc<S: Simd>(simd: S, acc: &mut [F; W], m: &[F], v: &[F]) {
    let m = &m[..B + SLACK];
    for k in 0..6 {
        fma_col(simd, acc, &m[k * 6..k * 6 + W], v[k]);
    }
}

/// acc += mᵀ v for the block starting at `m[0]` and a padded vector `v`
/// (its lanes 6 and 7 are zero, which cancels the overlap).
#[inline(always)]
fn mul_t_acc<S: Simd>(simd: S, acc: &mut [F; 6], m: &[F], v: &[F]) {
    let m = &m[..B + SLACK];
    let (vv, vt) = S::as_simd_f32s(&v[..W]);
    for k in 0..6 {
        let (cv, ct) = S::as_simd_f32s(&m[k * 6..k * 6 + W]);
        let mut s = 0.0;
        for (c, v) in cv.iter().zip(vv) {
            s += simd.reduce_sum_f32s(simd.mul_f32s(*c, *v));
        }
        for (c, v) in ct.iter().zip(vt) {
            s += c * v;
        }
        acc[k] += s;
    }
}

/// y += a x over flat slices.
#[inline(always)]
fn axpy<S: Simd>(simd: S, y: &mut [F], a: F, x: &[F]) {
    fma_col(simd, y, x, a);
}

/// Running dot product: vector accumulators in single precision, reduced
/// to double once per chunk.
struct Dot<S: Simd> {
    acc: [S::f32s; W],
    tail: f64,
}

impl<S: Simd> Dot<S> {
    #[inline(always)]
    fn new(simd: S) -> Self {
        Self {
            acc: [simd.splat_f32s(0.0); W],
            tail: 0.0,
        }
    }

    /// Add a · b for two padded node vectors.
    #[inline(always)]
    fn add(&mut self, simd: S, a: &[F], b: &[F]) {
        let (av, at) = S::as_simd_f32s(a);
        let (bv, bt) = S::as_simd_f32s(b);
        for ((acc, a), b) in self.acc.iter_mut().zip(av).zip(bv) {
            *acc = simd.mul_add_e_f32s(*a, *b, *acc);
        }
        for (a, b) in at.iter().zip(bt) {
            self.tail += (a * b) as f64;
        }
    }

    #[inline(always)]
    fn sum(self, simd: S) -> f64 {
        let mut total = self.tail;
        for acc in self.acc {
            total += simd.reduce_sum_f32s(acc) as f64;
        }
        total
    }
}

/// ap = A p on the nodes of one chunk; returns p · ap over them.
struct Matvec<'a> {
    jac: &'a BlockJacobi,
    first: usize,
    p: &'a [F],
    ap: &'a mut [F],
}

impl WithSimd for Matvec<'_> {
    type Output = f64;

    #[inline(always)]
    fn with_simd<S: Simd>(self, simd: S) -> f64 {
        let Self { jac, first, p, ap } = self;
        let mut pap = Dot::new(simd);
        for (k, out) in ap.as_chunks_mut::<W>().0.iter_mut().enumerate() {
            let i = first + k;
            let pi = &p[i * W..(i + 1) * W];
            let mut acc = [0.0; W];
            let mut acc_t = [0.0; 6];
            mul_acc(simd, &mut acc, &jac.diag[i * B..], pi);
            let (s, e) = (jac.row_ptr[i] as usize, jac.row_ptr[i + 1] as usize);
            for &(j, block) in &jac.row[s..e] {
                let beam = block as usize / 2;
                if jac.live[beam] {
                    let j = j as usize;
                    let (m, pj) = (&jac.off[beam * B..], &p[j * W..(j + 1) * W]);
                    if block % 2 == 0 {
                        mul_acc(simd, &mut acc, m, pj);
                    } else {
                        mul_t_acc(simd, &mut acc_t, m, pj);
                    }
                }
            }
            for k in 0..6 {
                out[k] = acc[k] + acc_t[k];
            }
            pap.add(simd, pi, out);
        }
        pap.sum(simd)
    }
}

/// x += alpha p, r -= alpha ap, z = M⁻¹ r on the nodes of one chunk;
/// returns (r · r, r · z) over them. `step` is None for the first residual,
/// which only needs z and the two sums.
struct Update<'a> {
    /// Inverse node blocks from the chunk's first node on.
    inv: &'a [F],
    step: Option<(F, &'a [F], &'a [F], &'a mut [F])>,
    r: &'a mut [F],
    z: &'a mut [F],
}

impl WithSimd for Update<'_> {
    type Output = [f64; 2];

    #[inline(always)]
    fn with_simd<S: Simd>(self, simd: S) -> [f64; 2] {
        let Self { inv, step, r, z } = self;
        if let Some((alpha, p, ap, x)) = step {
            axpy(simd, x, alpha, p);
            axpy(simd, r, -alpha, ap);
        }
        let (mut rr, mut rz) = (Dot::new(simd), Dot::new(simd));
        let (r, z) = (r.as_chunks::<W>().0, z.as_chunks_mut::<W>().0);
        for (k, (ri, zi)) in r.iter().zip(z).enumerate() {
            let mut acc = [0.0; W];
            mul_acc(simd, &mut acc, &inv[k * B..], ri);
            zi[..6].copy_from_slice(&acc[..6]);
            rr.add(simd, ri, ri);
            rz.add(simd, ri, zi);
        }
        [rr.sum(simd), rz.sum(simd)]
    }
}

/// p = z + beta p over flat slices.
struct Direction<'a> {
    beta: F,
    z: &'a [F],
    p: &'a mut [F],
}

impl WithSimd for Direction<'_> {
    type Output = ();

    #[inline(always)]
    fn with_simd<S: Simd>(self, simd: S) {
        let (pv, pt) = S::as_mut_simd_f32s(self.p);
        let (zv, zt) = S::as_simd_f32s(self.z);
        let beta = simd.splat_f32s(self.beta);
        for (p, z) in pv.iter_mut().zip(zv) {
            *p = simd.mul_add_e_f32s(*p, beta, *z);
        }
        for (p, z) in pt.iter_mut().zip(zt) {
            *p = z + *p * self.beta;
        }
    }
}

/// Copy a 6x6 matrix into the single-precision block layout.
fn store(m: &Matrix6<f64>, out: &mut [F]) {
    for (o, v) in out.iter_mut().zip(m.iter()) {
        *o = *v as F;
    }
}

impl BlockJacobi {
    pub fn new(n: usize, blocks: &[[Option<usize>; 4]]) -> Self {
        Self::from_layout(NodeLayout::new(n, blocks))
    }

    pub fn from_layout(layout: NodeLayout) -> Self {
        let NodeLayout { loc, free, ends } = layout;
        let nn = free.len();
        let mut row_ptr = vec![0u32; nn + 1];
        for e in &ends {
            if let [Some(a), Some(b)] = *e {
                row_ptr[a as usize + 1] += 1;
                row_ptr[b as usize + 1] += 1;
            }
        }
        for i in 0..nn {
            row_ptr[i + 1] += row_ptr[i];
        }
        let mut next = row_ptr.clone();
        let mut row = vec![(0u32, 0u32); row_ptr[nn] as usize];
        for (i, e) in ends.iter().enumerate() {
            if let [Some(a), Some(b)] = *e {
                // The block maps a vector at a to a result at b.
                for (at, other, block) in [(b, a, 2 * i), (a, b, 2 * i + 1)] {
                    let slot = &mut next[at as usize];
                    row[*slot as usize] = (other, block as u32);
                    *slot += 1;
                }
            }
        }
        let zero = vec![0.0; nn * W];
        Self {
            loc,
            free,
            diag64: vec![Matrix6::zeros(); nn],
            diag: vec![0.0; nn * B + SLACK],
            inv: vec![0.0; nn * B + SLACK],
            off: vec![0.0; ends.len() * B + SLACK],
            live: vec![false; ends.len()],
            ends,
            row_ptr,
            row,
            x: zero.clone(),
            b: zero.clone(),
            r: zero.clone(),
            z: zero.clone(),
            p: zero.clone(),
            ap: zero,
            sums: vec![[0.0; 2]; nn.div_ceil(CHUNK)],
            arch: SimdLevel::from_env().arch(),
            par_min_nodes: PAR_MIN_NODES,
        }
    }

    /// Run the kernels on `level` from now on.
    pub fn set_simd(&mut self, level: SimdLevel) {
        self.arch = level.arch();
    }

    /// Hand the passes to the thread pool from this many nodes up
    /// (`usize::MAX`: never).
    pub fn set_par_min_nodes(&mut self, nodes: usize) {
        self.par_min_nodes = nodes;
    }

    pub fn clear(&mut self) {
        self.diag64.fill(Matrix6::zeros());
        self.live.fill(false);
    }

    #[inline]
    pub fn add_diagonal(&mut self, dof: usize, v: f64) {
        let (nd, k) = self.loc[dof];
        self.diag64[nd as usize][(k as usize, k as usize)] += v;
    }

    pub fn add_block(&mut self, beam: usize, lower: &[f64; BLOCK_NNZ]) {
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
            let m = &mut self.off[beam * B..(beam + 1) * B];
            for c in 0..6 {
                for r in 0..6 {
                    m[c * 6 + r] = if fb[r] && fa[c] {
                        lower[tri(6 + r, c)] as F
                    } else {
                        0.0
                    };
                }
            }
            self.live[beam] = true;
        }
    }

    /// Solve to a residual of `tolerance` times the right-hand side.
    /// Returns the CG iteration count, or None if a node block is not
    /// positive definite.
    pub fn solve(&mut self, rhs: &[f64], out: &mut [f64], tolerance: f64) -> Option<usize> {
        let nn = self.free.len();
        let par = nn >= self.par_min_nodes;
        let arch = self.arch;

        // Node blocks: unit diagonal on fixed components, then the inverse.
        let free = &self.free;
        let invert =
            |(((d, f), inv), diag): (((&mut Matrix6<f64>, &[bool; 6]), &mut [F]), &mut [F])| {
                for k in 0..6 {
                    if !f[k] {
                        d[(k, k)] = 1.0;
                    }
                }
                // d⁻¹ = wᵀ w with w the inverse of d's Cholesky factor.
                let a: &[f64; B] = d
                    .as_slice()
                    .try_into()
                    .expect("a 6x6 matrix has 36 entries");
                let Some(w) = chol_inverse(a) else {
                    return false;
                };
                for c in 0..6 {
                    for r in c..6 {
                        let mut v = 0.0;
                        for k in r..6 {
                            v += w[r * 6 + k] * w[c * 6 + k];
                        }
                        inv[c * 6 + r] = v as F;
                        inv[r * 6 + c] = v as F;
                    }
                }
                store(d, diag);
                true
            };
        let definite = if par {
            (self.diag64.par_iter_mut().zip(free.par_iter()))
                .zip(self.inv[..nn * B].par_chunks_mut(B))
                .zip(self.diag[..nn * B].par_chunks_mut(B))
                .with_min_len(CHUNK)
                .all(invert)
        } else {
            (self.diag64.iter_mut().zip(free.iter()))
                .zip(self.inv[..nn * B].chunks_mut(B))
                .zip(self.diag[..nn * B].chunks_mut(B))
                .all(invert)
        };
        if !definite {
            return None;
        }

        let (mut x, mut b, mut r, mut z, mut p, mut ap, mut sums) = (
            std::mem::take(&mut self.x),
            std::mem::take(&mut self.b),
            std::mem::take(&mut self.r),
            std::mem::take(&mut self.z),
            std::mem::take(&mut self.p),
            std::mem::take(&mut self.ap),
            std::mem::take(&mut self.sums),
        );
        b.fill(0.0);
        for (d, &(nd, k)) in self.loc.iter().enumerate() {
            b[nd as usize * W + k as usize] = rhs[d] as F;
        }
        let target = b.iter().map(|&v| (v * v) as f64).sum::<f64>() * tolerance * tolerance;

        let jac = &*self;
        // ap = A v; returns v · ap.
        let matvec = |v: &[F], ap: &mut [F], sums: &mut [[f64; 2]]| -> f64 {
            let body = |(c, (ap, s)): (usize, (&mut [F], &mut [f64; 2]))| {
                s[0] = arch.dispatch(Matvec {
                    jac,
                    first: c * CHUNK,
                    p: v,
                    ap,
                });
            };
            if par {
                (ap.par_chunks_mut(CHUNK * W).zip(sums.par_iter_mut()))
                    .enumerate()
                    .for_each(body);
            } else {
                (ap.chunks_mut(CHUNK * W).zip(sums.iter_mut()))
                    .enumerate()
                    .for_each(body);
            }
            sums.iter().map(|s| s[0]).sum()
        };
        // Optionally x += alpha p and r -= alpha ap, then z = M⁻¹ r; returns
        // (r · r, r · z).
        let update = |step: Option<(F, &[F], &[F], &mut [F])>,
                      r: &mut [F],
                      z: &mut [F],
                      sums: &mut [[f64; 2]]|
         -> (f64, f64) {
            let n = CHUNK * W;
            let chunks = sums.len();
            match step {
                Some((alpha, p, ap, x)) => {
                    let body = |((((((r, z), c), p), ap), x), s): (
                        (((((&mut [F], &mut [F]), usize), &[F]), &[F]), &mut [F]),
                        &mut [f64; 2],
                    )| {
                        *s = arch.dispatch(Update {
                            inv: &jac.inv[c * CHUNK * B..],
                            step: Some((alpha, p, ap, x)),
                            r,
                            z,
                        });
                    };
                    if par {
                        (r.par_chunks_mut(n).zip(z.par_chunks_mut(n)))
                            .zip(0..chunks)
                            .zip(p.par_chunks(n))
                            .zip(ap.par_chunks(n))
                            .zip(x.par_chunks_mut(n))
                            .zip(sums.par_iter_mut())
                            .for_each(body);
                    } else {
                        (r.chunks_mut(n).zip(z.chunks_mut(n)))
                            .zip(0..chunks)
                            .zip(p.chunks(n))
                            .zip(ap.chunks(n))
                            .zip(x.chunks_mut(n))
                            .zip(sums.iter_mut())
                            .for_each(body);
                    }
                }
                None => {
                    let body =
                        |(((r, z), c), s): (((&mut [F], &mut [F]), usize), &mut [f64; 2])| {
                            *s = arch.dispatch(Update {
                                inv: &jac.inv[c * CHUNK * B..],
                                step: None,
                                r,
                                z,
                            });
                        };
                    (r.chunks_mut(n).zip(z.chunks_mut(n)))
                        .zip(0..chunks)
                        .zip(sums.iter_mut())
                        .for_each(body);
                }
            }
            sums.iter()
                .fold((0.0, 0.0), |(rr, rz), s| (rr + s[0], rz + s[1]))
        };

        let mut iters = 0;
        if target > 0.0 {
            // r = b − A x, with x the previous solution.
            matvec(&x, &mut ap, &mut sums);
            for ((r, b), ap) in r.iter_mut().zip(&b).zip(&ap) {
                *r = b - ap;
            }
            let (mut rr, mut rz) = update(None, &mut r, &mut z, &mut sums);
            let mut rz_old = rz;
            while rr > target && iters < MAX_ITERS {
                if iters == 0 {
                    p.copy_from_slice(&z);
                } else {
                    arch.dispatch(Direction {
                        beta: (rz / rz_old) as F,
                        z: &z,
                        p: &mut p,
                    });
                }
                let pap = matvec(&p, &mut ap, &mut sums);
                if pap <= 0.0 || !pap.is_finite() {
                    break;
                }
                let alpha = (rz / pap) as F;
                rz_old = rz;
                (rr, rz) = update(Some((alpha, &p, &ap, &mut x)), &mut r, &mut z, &mut sums);
                iters += 1;
            }
        } else {
            x.fill(0.0);
        }
        for (d, &(nd, k)) in self.loc.iter().enumerate() {
            out[d] = x[nd as usize * W + k as usize] as f64;
        }
        (self.x, self.b, self.r, self.z, self.p, self.ap, self.sums) = (x, b, r, z, p, ap, sums);
        Some(iters)
    }
}
