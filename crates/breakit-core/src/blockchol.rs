//! Node-block sparse Cholesky for the substep systems of [`crate::linsolve`].
//!
//! The matrix couples whole nodes: every beam adds a dense 12x12 block over
//! its two end nodes. A scalar sparse factorisation spends most of its time
//! on per-entry indirection there, so this one works on 6x6 node blocks
//! throughout:
//!
//! * every node owns six unknowns (translation, rotation); a fixed half
//!   stays in the numbering with a unit diagonal and no coupling,
//! * the fill-reducing ordering, the elimination tree and the column
//!   patterns are those of the node graph, built once,
//! * the numeric factorisation is left-looking over block columns, with
//!   fixed-size 6x6 kernels the compiler unrolls and vectorises,
//! * the triangular solves and the matrix-vector product index once per
//!   block instead of once per entry.
//!
//! Vectors handed to [`BlockChol::apply`] and [`BlockChol::matvec`] are in
//! the solver's own layout: six entries per node, nodes in elimination
//! order. [`BlockChol::load`] and [`BlockChol::store`] convert from and to
//! the caller's compact DOF numbering.

use crate::linsolve::{tri, NodeLayout, BLOCK_NNZ};
use crate::simd::{madd, SimdLevel};
use faer::dyn_stack::{MemBuffer, MemStack};
use faer::sparse::linalg::amd;
use faer::sparse::SymbolicSparseColMatRef;
use pulp::{Simd, WithSimd};

const ABSENT: u32 = u32::MAX;

/// A 6x6 block, column-major: entry (r, c) is at `c * 6 + r`.
type B6 = [f64; 36];
type V6 = [f64; 6];
const ZERO: B6 = [0.0; 36];

/// The three blocks of the matrix a beam adds to when all twelve of its
/// DOFs are unknowns and its ends are different nodes: then its packed
/// block splits into fixed runs and needs no per-entry positions.
#[derive(Clone, Copy)]
struct Target {
    /// Node blocks of ends a and b; `a == ABSENT` marks a beam that goes
    /// through `beam_pos` instead.
    a: u32,
    b: u32,
    /// Index into the matrix of the block coupling the two ends.
    pair: u32,
    /// The coupling block is stored with end a's rows and end b's columns.
    flip: bool,
}

/// Add the lower triangle of one end's 6x6 block of a beam (`o` = 0 for end
/// a, 6 for end b) to that node's block.
#[inline(always)]
fn add_node(dst: &mut B6, lower: &[f64; BLOCK_NNZ], o: usize) {
    for r in 0..6 {
        let row = &lower[tri(o + r, o)..][..r + 1];
        for (c, v) in row.iter().enumerate() {
            dst[c * 6 + r] += v;
        }
    }
}

/// Add the coupling 6x6 block of a beam (rows at end b, columns at end a) to
/// the matrix block of the node pair; `flip` when that block is stored the
/// other way round.
#[inline(always)]
fn add_pair(dst: &mut B6, lower: &[f64; BLOCK_NNZ], flip: bool) {
    for r in 0..6 {
        let row = &lower[tri(6 + r, 0)..][..6];
        if flip {
            dst[r * 6..r * 6 + 6]
                .iter_mut()
                .zip(row)
                .for_each(|(d, v)| *d += v);
        } else {
            for c in 0..6 {
                dst[c * 6 + r] += row[c];
            }
        }
    }
}

// Every kernel below is generic over the instruction set and compiled once
// per set `pulp` dispatches to (see `crate::simd`); the scalar copy is the
// reference. The fixed 6x6 loops unroll and vectorise under each set.

/// c -= a bᵀ
#[inline(always)]
fn mul_sub_t<S: Simd>(c: &mut B6, a: &B6, b: &B6) {
    for j in 0..6 {
        let mut col: V6 = [0.0; 6];
        for k in 0..6 {
            let s = b[k * 6 + j];
            for i in 0..6 {
                col[i] = madd::<S>(col[i], a[k * 6 + i], s);
            }
        }
        for i in 0..6 {
            c[j * 6 + i] -= col[i];
        }
    }
}

/// b <- b wᵀ for lower triangular w.
#[inline(always)]
fn mul_lower_t<S: Simd>(b: &mut B6, w: &B6) {
    // Column c of the result needs columns k <= c of b: go downwards.
    for c in (0..6).rev() {
        let mut col: V6 = [0.0; 6];
        for k in 0..=c {
            let s = w[k * 6 + c];
            for i in 0..6 {
                col[i] = madd::<S>(col[i], b[k * 6 + i], s);
            }
        }
        b[c * 6..c * 6 + 6].copy_from_slice(&col);
    }
}

/// m x
#[inline(always)]
fn mul_vec<S: Simd>(m: &B6, x: &V6) -> V6 {
    let mut y: V6 = [0.0; 6];
    for k in 0..6 {
        let s = x[k];
        for i in 0..6 {
            y[i] = madd::<S>(y[i], m[k * 6 + i], s);
        }
    }
    y
}

/// mᵀ x
#[inline(always)]
fn mul_vec_t<S: Simd>(m: &B6, x: &V6) -> V6 {
    let mut y: V6 = [0.0; 6];
    for k in 0..6 {
        let mut acc = 0.0;
        for i in 0..6 {
            acc = madd::<S>(acc, m[k * 6 + i], x[i]);
        }
        y[k] = acc;
    }
    y
}

/// Inverse of the Cholesky factor of the symmetric block `d` (its lower
/// triangle is read), or None if `d` is not positive definite.
#[inline(always)]
pub(crate) fn chol_inverse(d: &B6) -> Option<B6> {
    let mut l = *d;
    let mut inv = [0.0; 6];
    for c in 0..6 {
        let mut p = l[c * 6 + c];
        for k in 0..c {
            p -= l[k * 6 + c] * l[k * 6 + c];
        }
        if !(p > 0.0 && p.is_finite()) {
            return None;
        }
        let root = p.sqrt();
        let ir = 1.0 / root;
        inv[c] = ir;
        l[c * 6 + c] = root;
        for r in c + 1..6 {
            let mut v = l[c * 6 + r];
            for k in 0..c {
                v -= l[k * 6 + r] * l[k * 6 + c];
            }
            l[c * 6 + r] = v * ir;
        }
    }
    let mut w = ZERO;
    for c in 0..6 {
        w[c * 6 + c] = inv[c];
        for r in c + 1..6 {
            let mut v = 0.0;
            for k in c..r {
                v += l[k * 6 + r] * w[c * 6 + k];
            }
            w[c * 6 + r] = -v * inv[r];
        }
    }
    Some(w)
}

/// Block column patterns of the factor for a given elimination order.
struct Symbolic {
    /// Elimination order: `order[new] = old`.
    order: Vec<u32>,
    col_ptr: Vec<u32>,
    /// Rows below the diagonal of each block column, ascending.
    row_idx: Vec<u32>,
    /// 6x6 block products in one numeric factorisation.
    factor_products: f64,
}

fn symbolic(adj: &[Vec<u32>], order: Vec<u32>) -> Symbolic {
    let nn = adj.len();
    let mut inv = vec![0u32; nn];
    for (new, &old) in order.iter().enumerate() {
        inv[old as usize] = new as u32;
    }
    let mut cols: Vec<Vec<u32>> = vec![Vec::new(); nn];
    let mut children: Vec<Vec<u32>> = vec![Vec::new(); nn];
    let mut mark = vec![ABSENT; nn];
    let mut col_ptr = Vec::with_capacity(nn + 1);
    let mut row_idx = Vec::new();
    let mut factor_products = 0.0;
    col_ptr.push(0);
    for j in 0..nn {
        let ju = j as u32;
        let mut s = Vec::new();
        mark[j] = ju;
        let mut add = |r: u32, s: &mut Vec<u32>| {
            if r > ju && mark[r as usize] != ju {
                mark[r as usize] = ju;
                s.push(r);
            }
        };
        for &old in &adj[order[j] as usize] {
            add(inv[old as usize], &mut s);
        }
        for &c in &children[j] {
            for &r in &cols[c as usize] {
                add(r, &mut s);
            }
        }
        s.sort_unstable();
        if let Some(&parent) = s.first() {
            children[parent as usize].push(ju);
        }
        let c = s.len() as f64;
        factor_products += (c + 1.0) * (c + 2.0) / 2.0;
        row_idx.extend_from_slice(&s);
        col_ptr.push(row_idx.len() as u32);
        cols[j] = s;
    }
    Symbolic {
        order,
        col_ptr,
        row_idx,
        factor_products,
    }
}

/// faer's approximate minimum degree on the node graph, as `order[new] = old`.
/// None if it ran out of memory.
fn amd_order(adj: &[Vec<u32>]) -> Option<Vec<u32>> {
    let nn = adj.len();
    let mut col_ptr = Vec::with_capacity(nn + 1);
    let mut row_idx = Vec::new();
    col_ptr.push(0usize);
    for a in adj {
        row_idx.extend(a.iter().map(|&r| r as usize));
        col_ptr.push(row_idx.len());
    }
    let pattern = SymbolicSparseColMatRef::new_checked(nn, nn, &col_ptr, None, &row_idx);
    let mut perm = vec![0usize; nn];
    let mut perm_inv = vec![0usize; nn];
    let mut mem = MemBuffer::try_new(amd::order_scratch::<usize>(nn, row_idx.len())).ok()?;
    amd::order(
        &mut perm,
        &mut perm_inv,
        pattern,
        amd::Control::default(),
        MemStack::new(&mut mem),
    )
    .ok()?;
    Some(perm.into_iter().map(|p| p as u32).collect())
}

pub(crate) struct BlockChol {
    nn: usize,
    /// Compact DOF -> index in the solver's vector layout.
    dof_pos: Vec<u32>,
    /// Flat indices into `a` of the diagonal entries of fixed DOFs.
    fixed: Vec<u32>,
    /// The matrix: `nn` node blocks (lower triangle assembled, mirrored by
    /// `finish`), then one block per coupled node pair, rows at the node
    /// eliminated later.
    a: Vec<B6>,
    /// Pairs sorted by column: `pair_ptr[j]..pair_ptr[j + 1]` are those
    /// whose earlier node is `j`.
    pair_ptr: Vec<u32>,
    pair_row: Vec<u32>,
    /// Slot of each pair's block in `l`.
    pair_l: Vec<u32>,
    /// Per beam, the flat index into `a` of each packed lower-triangle entry.
    beam_pos: Vec<[u32; BLOCK_NNZ]>,
    /// Per beam, its blocks when it takes the fast path of `add_block`.
    target: Vec<Target>,
    col_ptr: Vec<u32>,
    row_idx: Vec<u32>,
    /// Off-diagonal blocks of the factor, by column.
    l: Vec<B6>,
    /// Inverse of each diagonal block of the factor (lower triangular).
    w: Vec<B6>,
    // Scratch of the numeric factorisation.
    pos: Vec<u32>,
    head: Vec<u32>,
    next: Vec<u32>,
    cur: Vec<u32>,
    /// Instruction set the kernels run on (see [`crate::simd`]).
    arch: pulp::Arch,
    /// 6x6 block products in a factorisation, and block-times-vector
    /// products in one CG iteration (triangular solves plus the product
    /// with the matrix).
    factor_products: f64,
    iteration_products: f64,
    /// What a scalar factorisation with this ordering would cost: the sum of
    /// squared column counts, and the flops of a solve pair plus a product.
    pub scalar_factor_flops: f64,
    pub scalar_solve_flops: f64,
}

impl BlockChol {
    pub fn new(layout: &NodeLayout) -> Self {
        let nn = layout.free.len();
        let mut adj: Vec<Vec<u32>> = vec![Vec::new(); nn];
        for e in &layout.ends {
            if let [Some(a), Some(b)] = *e {
                if a != b {
                    adj[a as usize].push(b);
                    adj[b as usize].push(a);
                }
            }
        }
        for a in &mut adj {
            a.sort_unstable();
            a.dedup();
        }
        // An exact minimum degree ordering was also tried: the same fill on
        // the bridges, 5 to 15% more work on the towers.
        let order = amd_order(&adj).unwrap_or_else(|| (0..nn as u32).collect());
        let Symbolic {
            order,
            col_ptr,
            row_idx,
            factor_products,
        } = symbolic(&adj, order);
        let mut inv = vec![0u32; nn];
        for (new, &old) in order.iter().enumerate() {
            inv[old as usize] = new as u32;
        }

        // Coupled pairs, by column.
        let mut pair_ptr = Vec::with_capacity(nn + 1);
        let mut pair_row = Vec::new();
        let mut pair_l = Vec::new();
        pair_ptr.push(0);
        for j in 0..nn {
            let start = pair_row.len();
            pair_row.extend(
                adj[order[j] as usize]
                    .iter()
                    .map(|&o| inv[o as usize])
                    .filter(|&r| r as usize > j),
            );
            pair_row[start..].sort_unstable();
            let (s, e) = (col_ptr[j] as usize, col_ptr[j + 1] as usize);
            for &r in &pair_row[start..] {
                let k = row_idx[s..e].binary_search(&r).expect("pair in the factor");
                pair_l.push((s + k) as u32);
            }
            pair_ptr.push(pair_row.len() as u32);
        }
        let pair_of = |row: u32, col: u32| {
            let (s, e) = (
                pair_ptr[col as usize] as usize,
                pair_ptr[col as usize + 1] as usize,
            );
            s + pair_row[s..e].binary_search(&row).expect("pair exists")
        };

        let beam_pos = layout
            .ends
            .iter()
            .map(|e| {
                let mut pos = [ABSENT; BLOCK_NNZ];
                let node = |l: usize| e[l / 6].map(|n| inv[n as usize]);
                let is_free = |l: usize| e[l / 6].is_some_and(|n| layout.free[n as usize][l % 6]);
                for r in 0..12 {
                    for c in 0..=r {
                        if !(is_free(r) && is_free(c)) {
                            continue;
                        }
                        let (nr, nc) = (node(r).unwrap(), node(c).unwrap());
                        let (kr, kc) = (r % 6, c % 6);
                        let (block, row, col) = if nr == nc {
                            // Node block: only its lower triangle is kept.
                            (nr as usize, kr.max(kc), kr.min(kc))
                        } else if nr > nc {
                            (nn + pair_of(nr, nc), kr, kc)
                        } else {
                            (nn + pair_of(nc, nr), kc, kr)
                        };
                        pos[tri(r, c)] = (block * 36 + col * 6 + row) as u32;
                    }
                }
                pos
            })
            .collect();
        let target = layout
            .ends
            .iter()
            .map(|e| match *e {
                [Some(a), Some(b)]
                    if a != b
                        && layout.free[a as usize] == [true; 6]
                        && layout.free[b as usize] == [true; 6] =>
                {
                    let (na, nb) = (inv[a as usize], inv[b as usize]);
                    Target {
                        a: na,
                        b: nb,
                        pair: (nn + pair_of(na.max(nb), na.min(nb))) as u32,
                        flip: na > nb,
                    }
                }
                _ => Target {
                    a: ABSENT,
                    b: ABSENT,
                    pair: ABSENT,
                    flip: false,
                },
            })
            .collect();
        let dof_pos = layout
            .loc
            .iter()
            .map(|&(nd, k)| inv[nd as usize] * 6 + k as u32)
            .collect();
        let mut fixed = Vec::new();
        for (old, f) in layout.free.iter().enumerate() {
            for (k, &free) in f.iter().enumerate() {
                if !free {
                    fixed.push(inv[old] * 36 + (k * 7) as u32);
                }
            }
        }

        let (mut scalar_factor_flops, mut scalar_solve_flops) = (0.0, 0.0);
        for w in col_ptr.windows(2) {
            let c = (w[1] - w[0]) as usize;
            for t in 1..=6 {
                let count = (6 * c + t) as f64;
                scalar_factor_flops += count * count;
                scalar_solve_flops += 4.0 * count;
            }
        }
        scalar_solve_flops += 2.0 * (21 * nn + 36 * pair_row.len()) as f64;
        let iteration_products = (3 * nn + 2 * row_idx.len() + 2 * pair_row.len()) as f64;
        Self {
            nn,
            dof_pos,
            fixed,
            a: vec![ZERO; nn + pair_row.len()],
            pair_ptr,
            pair_row,
            pair_l,
            beam_pos,
            target,
            l: vec![ZERO; row_idx.len()],
            w: vec![ZERO; nn],
            col_ptr,
            row_idx,
            pos: vec![0; nn],
            head: vec![ABSENT; nn],
            next: vec![ABSENT; nn],
            cur: vec![0; nn],
            arch: SimdLevel::from_env().arch(),
            factor_products,
            iteration_products,
            scalar_factor_flops,
            scalar_solve_flops,
        }
    }

    /// Cost of a numeric factorisation in CG iterations. Measured on towers
    /// and bridges of 100 to 5,800 members (`bench --kernels`): a
    /// factorisation takes about 15 ns per block product plus 90 ns per
    /// node (its diagonal block and the column set-up), an iteration 5 to
    /// 10 ns per block-times-vector product.
    pub fn factor_cost_in_iterations(&self) -> f64 {
        2.1 * (self.factor_products + 6.0 * self.nn as f64) / self.iteration_products.max(1.0)
    }

    /// Length of the vectors `apply` and `matvec` work on.
    pub fn dim(&self) -> usize {
        6 * self.nn
    }

    /// Scalar entries stored in the factor.
    pub fn factor_entries(&self) -> usize {
        36 * self.l.len() + 21 * self.nn
    }

    pub fn clear(&mut self) {
        self.a.fill(ZERO);
    }

    #[inline]
    pub fn add_diagonal(&mut self, dof: usize, v: f64) {
        let p = self.dof_pos[dof] as usize;
        self.a[p / 6][p % 6 * 7] += v;
    }

    #[inline]
    pub fn add_block(&mut self, beam: usize, lower: &[f64; BLOCK_NNZ]) {
        let t = self.target[beam];
        if t.a != ABSENT {
            add_node(&mut self.a[t.a as usize], lower, 0);
            add_node(&mut self.a[t.b as usize], lower, 6);
            add_pair(&mut self.a[t.pair as usize], lower, t.flip);
            return;
        }
        let vals = self.a.as_flattened_mut();
        for (&p, &v) in self.beam_pos[beam].iter().zip(lower) {
            if p != ABSENT {
                vals[p as usize] += v;
            }
        }
    }

    /// Complete the assembled matrix: mirror the node blocks and put ones on
    /// the diagonal of fixed DOFs. Idempotent.
    pub fn finish(&mut self) {
        for d in &mut self.a[..self.nn] {
            for c in 0..6 {
                for r in c + 1..6 {
                    d[r * 6 + c] = d[c * 6 + r];
                }
            }
        }
        let vals = self.a.as_flattened_mut();
        for &p in &self.fixed {
            vals[p as usize] = 1.0;
        }
    }

    /// Compact DOF vector -> solver layout.
    pub fn load(&self, v: &[f64], out: &mut [f64]) {
        out.fill(0.0);
        for (&p, &x) in self.dof_pos.iter().zip(v) {
            out[p as usize] = x;
        }
    }

    /// Solver layout -> compact DOF vector.
    pub fn store(&self, v: &[f64], out: &mut [f64]) {
        for (&p, x) in self.dof_pos.iter().zip(out) {
            *x = v[p as usize];
        }
    }

    /// Run the kernels on `level` from now on. The default is
    /// [`SimdLevel::from_env`].
    pub fn set_simd(&mut self, level: SimdLevel) {
        self.arch = level.arch();
    }

    /// Factorise the matrix completed by [`finish`](Self::finish). Returns
    /// false if it is not positive definite.
    pub fn factor(&mut self) -> bool {
        struct Op<'a>(&'a mut BlockChol);
        impl WithSimd for Op<'_> {
            type Output = bool;
            #[inline(always)]
            fn with_simd<S: Simd>(self, _: S) -> bool {
                self.0.factor_generic::<S>()
            }
        }
        self.arch.dispatch(Op(self))
    }

    #[inline(always)]
    #[allow(clippy::needless_range_loop)]
    fn factor_generic<S: Simd>(&mut self) -> bool {
        let nn = self.nn;
        let (diag, pairs) = self.a.split_at(nn);
        self.head.fill(ABSENT);
        for k in 0..nn {
            let (s, e) = (self.col_ptr[k] as usize, self.col_ptr[k + 1] as usize);
            // Columns before k are finished; column k starts as that of A.
            let (done, rest) = self.l.split_at_mut(s);
            let col = &mut rest[..e - s];
            col.fill(ZERO);
            for (i, &r) in self.row_idx[s..e].iter().enumerate() {
                self.pos[r as usize] = i as u32;
            }
            for q in self.pair_ptr[k] as usize..self.pair_ptr[k + 1] as usize {
                col[self.pair_l[q] as usize - s] = pairs[q];
            }
            let mut d = diag[k];
            // Every earlier column j with a block in row k updates column k.
            let mut j = self.head[k];
            while j != ABSENT {
                let ju = j as usize;
                let next_j = self.next[ju];
                let c = self.cur[ju] as usize;
                let end = self.col_ptr[ju + 1] as usize;
                let b = &done[c];
                mul_sub_t::<S>(&mut d, b, b);
                for q in c + 1..end {
                    let t = self.pos[self.row_idx[q] as usize] as usize;
                    mul_sub_t::<S>(&mut col[t], &done[q], b);
                }
                if c + 1 < end {
                    let r = self.row_idx[c + 1] as usize;
                    self.cur[ju] = (c + 1) as u32;
                    self.next[ju] = self.head[r];
                    self.head[r] = j;
                }
                j = next_j;
            }
            let Some(w) = chol_inverse(&d) else {
                return false;
            };
            for b in col.iter_mut() {
                mul_lower_t::<S>(b, &w);
            }
            self.w[k] = w;
            if s < e {
                let r = self.row_idx[s] as usize;
                self.cur[k] = s as u32;
                self.next[k] = self.head[r];
                self.head[r] = k as u32;
            }
        }
        true
    }

    /// v <- (L Lᵀ)⁻¹ v with the stored factorisation.
    pub fn apply(&self, v: &mut [f64]) {
        struct Op<'a>(&'a BlockChol, &'a mut [f64]);
        impl WithSimd for Op<'_> {
            type Output = ();
            #[inline(always)]
            fn with_simd<S: Simd>(self, _: S) {
                self.0.apply_generic::<S>(self.1);
            }
        }
        self.arch.dispatch(Op(self, v));
    }

    #[inline(always)]
    fn apply_generic<S: Simd>(&self, v: &mut [f64]) {
        let (v, _) = v.as_chunks_mut::<6>();
        for j in 0..self.nn {
            let y = mul_vec::<S>(&self.w[j], &v[j]);
            v[j] = y;
            let (s, e) = (self.col_ptr[j] as usize, self.col_ptr[j + 1] as usize);
            for (m, &r) in self.l[s..e].iter().zip(&self.row_idx[s..e]) {
                let t = &mut v[r as usize];
                for k in 0..6 {
                    let x = -y[k];
                    for i in 0..6 {
                        t[i] = madd::<S>(t[i], m[k * 6 + i], x);
                    }
                }
            }
        }
        for j in (0..self.nn).rev() {
            let mut t = v[j];
            let (s, e) = (self.col_ptr[j] as usize, self.col_ptr[j + 1] as usize);
            for (m, &r) in self.l[s..e].iter().zip(&self.row_idx[s..e]) {
                let u = mul_vec_t::<S>(m, &v[r as usize]);
                for i in 0..6 {
                    t[i] -= u[i];
                }
            }
            v[j] = mul_vec_t::<S>(&self.w[j], &t);
        }
    }

    /// y <- A v
    pub fn matvec(&self, v: &[f64], y: &mut [f64]) {
        struct Op<'a>(&'a BlockChol, &'a [f64], &'a mut [f64]);
        impl WithSimd for Op<'_> {
            type Output = ();
            #[inline(always)]
            fn with_simd<S: Simd>(self, _: S) {
                self.0.matvec_generic::<S>(self.1, self.2);
            }
        }
        self.arch.dispatch(Op(self, v, y));
    }

    #[inline(always)]
    fn matvec_generic<S: Simd>(&self, v: &[f64], y: &mut [f64]) {
        let (v, _) = v.as_chunks::<6>();
        let (y, _) = y.as_chunks_mut::<6>();
        let (diag, pairs) = self.a.split_at(self.nn);
        y.fill([0.0; 6]);
        for j in 0..self.nn {
            let vj = &v[j];
            let mut acc = mul_vec::<S>(&diag[j], vj);
            let (s, e) = (self.pair_ptr[j] as usize, self.pair_ptr[j + 1] as usize);
            for (m, &r) in pairs[s..e].iter().zip(&self.pair_row[s..e]) {
                let r = r as usize;
                let u = mul_vec::<S>(m, vj);
                let t = mul_vec_t::<S>(m, &v[r]);
                for i in 0..6 {
                    y[r][i] += u[i];
                    acc[i] += t[i];
                }
            }
            for i in 0..6 {
                y[j][i] += acc[i];
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::BlockChol;
    use crate::linsolve::{tri, Backend, LinSolver, NodeLayout, Policy, BLOCK_NNZ};
    use crate::simd::SimdLevel;
    use nalgebra::{DMatrix, DVector};

    struct Rng(u64);
    impl Rng {
        fn next(&mut self) -> u64 {
            self.0 ^= self.0 << 13;
            self.0 ^= self.0 >> 7;
            self.0 ^= self.0 << 17;
            self.0
        }
        fn unit(&mut self) -> f64 {
            (self.next() >> 11) as f64 / (1u64 << 53) as f64
        }
        fn below(&mut self, n: usize) -> usize {
            (self.next() % n as u64) as usize
        }
    }

    /// Random symmetric positive semidefinite 12x12 block, packed.
    fn random_block(rng: &mut Rng) -> [f64; BLOCK_NNZ] {
        let scale = 10f64.powf(3.0 * rng.unit());
        let m = DMatrix::from_fn(12, 9, |_, _| rng.unit() - 0.5);
        let k = &m * m.transpose() * scale;
        let mut out = [0.0; BLOCK_NNZ];
        for r in 0..12 {
            for c in 0..=r {
                out[tri(r, c)] = k[(r, c)];
            }
        }
        out
    }

    /// Node pairs of a chain with a few long-range links, a 2D grid with
    /// diagonals, and a 3D grid.
    fn graphs(rng: &mut Rng) -> Vec<(usize, Vec<(usize, usize)>)> {
        let mut chain: Vec<_> = (0..39).map(|i| (i, i + 1)).collect();
        for _ in 0..8 {
            let (a, b) = (rng.below(40), rng.below(40));
            if a != b {
                chain.push((a, b));
            }
        }
        // A doubled beam between the same two nodes, in both directions.
        chain.push((3, 4));
        chain.push((5, 4));
        let mut grid = Vec::new();
        let at = |x: usize, y: usize| x * 6 + y;
        for x in 0..7 {
            for y in 0..6 {
                if x + 1 < 7 {
                    grid.push((at(x, y), at(x + 1, y)));
                }
                if y + 1 < 6 {
                    grid.push((at(x, y + 1), at(x, y)));
                }
                if x + 1 < 7 && y + 1 < 6 && (x + y) % 2 == 0 {
                    grid.push((at(x, y), at(x + 1, y + 1)));
                }
            }
        }
        let mut cube = Vec::new();
        let at = |x: usize, y: usize, z: usize| (x * 4 + y) * 4 + z;
        for x in 0..4 {
            for y in 0..4 {
                for z in 0..4 {
                    if x + 1 < 4 {
                        cube.push((at(x, y, z), at(x + 1, y, z)));
                    }
                    if y + 1 < 4 {
                        cube.push((at(x, y, z), at(x, y + 1, z)));
                    }
                    if z + 1 < 4 {
                        cube.push((at(x, y, z + 1), at(x, y, z)));
                    }
                }
            }
        }
        vec![(40, chain), (42, grid), (64, cube)]
    }

    #[test]
    fn block_cholesky_matches_dense_cholesky() {
        let mut rng = Rng(0x9E3779B97F4A7C15);
        for (nodes, pairs) in graphs(&mut rng) {
            for backend in [Backend::Cholesky, Backend::CholeskyScalar] {
                // Slots: some nodes lose their translation, their rotation
                // or both, as supports do.
                let mut n = 0;
                let mut slots = Vec::new();
                for _ in 0..nodes {
                    let kind = rng.below(10);
                    let mut next = |free: bool| {
                        free.then(|| {
                            n += 3;
                            n - 3
                        })
                    };
                    slots.push((next(kind != 0 && kind != 2), next(kind != 1 && kind != 2)));
                }
                let blocks: Vec<_> = pairs
                    .iter()
                    .map(|&(a, b)| [slots[a].0, slots[a].1, slots[b].0, slots[b].1])
                    .collect();
                let mut lin = LinSolver::new(n, &blocks, backend).unwrap();
                let policy = Policy {
                    tolerance: 1e-12,
                    refactor_cost: 1000,
                    max_iters: 1000,
                    max_age: 0,
                };
                // Three matrices on the same pattern: the first is solved
                // with a fresh factorisation, the later ones (more beams
                // broken, the others stiffer or softer) by CG on the stale
                // one.
                let base: Vec<_> = blocks.iter().map(|_| random_block(&mut rng)).collect();
                let mut broken = vec![false; blocks.len()];
                for round in 0..3 {
                    for b in broken.iter_mut() {
                        *b |= rng.below(8) == 0;
                    }
                    let mut dense = DMatrix::<f64>::zeros(n, n);
                    lin.clear();
                    for d in 0..n {
                        let v = 0.5 + rng.unit();
                        lin.add_diagonal(d, v);
                        dense[(d, d)] += v;
                    }
                    for (i, slots) in blocks.iter().enumerate() {
                        if broken[i] {
                            continue;
                        }
                        let scale = 0.7 + 0.6 * rng.unit();
                        let k = base[i].map(|v| v * scale);
                        lin.add_block(i, &k);
                        let dof = |l: usize| slots[l / 3].map(|d| d + l % 3);
                        for r in 0..12 {
                            for c in 0..=r {
                                if let (Some(gr), Some(gc)) = (dof(r), dof(c)) {
                                    dense[(gr, gc)] += k[tri(r, c)];
                                    if r != c {
                                        dense[(gc, gr)] += k[tri(r, c)];
                                    }
                                }
                            }
                        }
                    }
                    let b = DVector::from_fn(n, |_, _| rng.unit() - 0.5);
                    let want = dense.cholesky().expect("SPD").solve(&b);
                    let mut x = vec![0.0; n];
                    assert!(lin.solve(b.as_slice(), &mut x, &policy));
                    let err = (DVector::from_vec(x) - &want).norm() / want.norm();
                    assert!(
                        err < 1e-9,
                        "{backend:?}, {nodes} nodes, round {round}: relative error {err:e}"
                    );
                    assert_eq!(lin.stats.fallbacks, 0);
                    assert_eq!(lin.stats.factorizations, 1);
                }
            }
        }
    }

    /// The scalar reference kernels solve the system, and the native
    /// kernels agree with them to rounding.
    #[test]
    fn scalar_and_native_kernels_agree() {
        let mut rng = Rng(0xD1B54A32D192ED03);
        for (nodes, pairs) in graphs(&mut rng) {
            let n = 6 * nodes;
            let blocks: Vec<_> = pairs
                .iter()
                .map(|&(a, b)| [Some(6 * a), Some(6 * a + 3), Some(6 * b), Some(6 * b + 3)])
                .collect();
            let mut chol = BlockChol::new(&NodeLayout::new(n, &blocks));
            chol.clear();
            for d in 0..n {
                chol.add_diagonal(d, 0.5 + rng.unit());
            }
            for i in 0..blocks.len() {
                chol.add_block(i, &random_block(&mut rng));
            }
            chol.finish();
            let b: Vec<f64> = (0..n).map(|_| rng.unit() - 0.5).collect();
            let (mut bw, mut x, mut ax) = (vec![0.0; n], vec![0.0; n], vec![0.0; n]);
            chol.load(&b, &mut bw);
            chol.set_simd(SimdLevel::Scalar);
            assert!(chol.factor());
            x.copy_from_slice(&bw);
            chol.apply(&mut x);
            chol.matvec(&x, &mut ax);
            let norm = |v: &[f64]| v.iter().map(|x| x * x).sum::<f64>().sqrt();
            let res: Vec<f64> = ax.iter().zip(&bw).map(|(a, b)| a - b).collect();
            assert!(norm(&res) < 1e-10 * norm(&bw), "residual {:e}", norm(&res));
            // Both copies of the kernels agree to rounding.
            let mut x2 = bw.clone();
            chol.set_simd(SimdLevel::Native);
            assert!(chol.factor());
            chol.apply(&mut x2);
            let diff: Vec<f64> = x.iter().zip(&x2).map(|(a, b)| a - b).collect();
            assert!(norm(&diff) < 1e-10 * norm(&x));
        }
    }

    /// The fixed-run path of `add_block` puts every entry where the
    /// per-entry positions say.
    #[test]
    fn fast_add_block_matches_the_positions() {
        let mut rng = Rng(0xA0761D6478BD642F);
        let mut fast = 0;
        for (nodes, pairs) in graphs(&mut rng) {
            let blocks: Vec<_> = pairs
                .iter()
                .map(|&(a, b)| [Some(6 * a), Some(6 * a + 3), Some(6 * b), Some(6 * b + 3)])
                .collect();
            let mut chol = BlockChol::new(&NodeLayout::new(6 * nodes, &blocks));
            for beam in 0..blocks.len() {
                let block = random_block(&mut rng);
                chol.clear();
                chol.add_block(beam, &block);
                let mut expected = vec![0.0; chol.a.len() * 36];
                for (&p, &v) in chol.beam_pos[beam].iter().zip(&block) {
                    expected[p as usize] += v;
                }
                assert_eq!(chol.a.as_flattened(), &expected[..]);
                fast += (chol.target[beam].a != super::ABSENT) as usize;
            }
        }
        assert!(fast > 0, "no beam took the fast path");
    }

    #[test]
    fn indefinite_matrix_is_reported() {
        let blocks = [[Some(0), Some(3), Some(6), Some(9)]];
        let mut lin = LinSolver::new(12, &blocks, Backend::Cholesky).unwrap();
        lin.clear();
        for d in 0..12 {
            lin.add_diagonal(d, if d == 7 { -1.0 } else { 1.0 });
        }
        let policy = Policy {
            tolerance: 1e-10,
            refactor_cost: 0,
            max_iters: 0,
            max_age: 0,
        };
        let mut x = [0.0; 12];
        assert!(!lin.solve(&[1.0; 12], &mut x, &policy));
    }
}
