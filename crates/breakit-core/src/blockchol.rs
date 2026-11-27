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
//! * a long structure's nodes are cut into parts that couple only through
//!   a few separator nodes ordered last ([`dissect`]). The parts are then
//!   factorised, solved and multiplied independently, a thread each, and
//!   what they would write into separator rows goes to buffers of their
//!   own that are added up in part order. The split depends on the
//!   structure alone, so the result is the same bit for bit on any number
//!   of threads.
//!
//! Vectors handed to [`BlockChol::apply`] and [`BlockChol::matvec`] are in
//! the solver's own layout: six entries per node, nodes in elimination
//! order. [`BlockChol::load`] and [`BlockChol::store`] convert from and to
//! the caller's compact DOF numbering.

use crate::dynamics::pool::ordered_chunks;
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

/// An elimination order in which the first columns fall into parts that
/// do not couple to each other, only to the separator nodes ordered last.
struct Dissection {
    /// `order[new] = old`.
    order: Vec<u32>,
    /// Part `p` is the columns `bounds[p]..bounds[p + 1]`; the separators
    /// are the columns from the last bound on.
    bounds: Vec<u32>,
}

/// Breadth-first search from `start` over nodes whose `dist` is `ABSENT`:
/// sets their distance and returns them in the order reached.
fn bfs(adj: &[Vec<u32>], start: u32, dist: &mut [u32]) -> Vec<u32> {
    let mut queue = vec![start];
    dist[start as usize] = 0;
    let mut at = 0;
    while at < queue.len() {
        let n = queue[at] as usize;
        at += 1;
        for &m in &adj[n] {
            if dist[m as usize] == ABSENT {
                dist[m as usize] = dist[n] + 1;
                queue.push(m);
            }
        }
    }
    queue
}

/// Cut the node graph into up to `parts` parts of about equal size along
/// levels of a breadth-first search (few nodes wide on a long structure),
/// and order every part by minimum degree. None if no cut was found.
fn dissect(adj: &[Vec<u32>], parts: usize) -> Option<Dissection> {
    let nn = adj.len();
    // Levels, component after component, with an empty level in between:
    // the free cut.
    let mut level = vec![ABSENT; nn];
    let mut dist = vec![ABSENT; nn];
    let mut base = 0u32;
    for seed in 0..nn as u32 {
        if level[seed as usize] != ABSENT {
            continue;
        }
        // Start from an end of the component: the far node of the far node.
        let mut start = seed;
        for _ in 0..2 {
            let reached = bfs(adj, start, &mut dist);
            start = *reached.last().expect("the search reaches its start");
            for n in reached {
                dist[n as usize] = ABSENT;
            }
        }
        let reached = bfs(adj, start, &mut dist);
        let mut deepest = 0;
        for n in reached {
            level[n as usize] = base + dist[n as usize];
            deepest = deepest.max(dist[n as usize]);
        }
        base += deepest + 2;
    }
    let levels = base as usize;
    let mut size = vec![0usize; levels];
    for &l in &level {
        size[l as usize] += 1;
    }
    // The narrowest level near each equal share.
    let mut cuts: Vec<u32> = Vec::new();
    let slack = nn / (4 * parts);
    for i in 1..parts {
        let target = i * nn / parts;
        let mut best: Option<(usize, usize, usize)> = None;
        let mut before = 0;
        for (l, &sz) in size.iter().enumerate() {
            let mid = before + sz / 2;
            before += sz;
            let after_last = cuts.last().is_none_or(|&c| l > c as usize + 1);
            if l == 0 || !after_last || mid.abs_diff(target) > slack {
                continue;
            }
            let key = (sz, mid.abs_diff(target), l);
            if best.is_none_or(|b| key < b) {
                best = Some(key);
            }
        }
        if let Some((_, _, l)) = best {
            cuts.push(l as u32);
        }
    }
    if cuts.is_empty() {
        return None;
    }
    // Part of each node; a node of a cut level stays in the part below if
    // nothing ties it to the level above.
    let np = cuts.len() + 1;
    let mut members: Vec<Vec<u32>> = vec![Vec::new(); np];
    let mut separators: Vec<u32> = Vec::new();
    for n in 0..nn {
        let l = level[n];
        let below = cuts.partition_point(|&c| c < l);
        let on_cut = cuts.get(below) == Some(&l);
        if on_cut && adj[n].iter().any(|&m| level[m as usize] > l) {
            separators.push(n as u32);
        } else {
            members[below].push(n as u32);
        }
    }
    separators.sort_unstable_by_key(|&n| (level[n as usize], n));
    // Minimum degree inside each part, with the separator nodes the part
    // touches in its graph as one clique: the ordering then sees the fill
    // an elimination next to a separator causes and leaves those for last.
    let mut order = Vec::with_capacity(nn);
    let mut bounds = vec![0u32];
    let mut local = vec![ABSENT; nn];
    let mut is_sep = vec![false; nn];
    for &n in &separators {
        is_sep[n as usize] = true;
    }
    for part in &members {
        let mut nodes = part.clone();
        for &n in part {
            for &m in &adj[n as usize] {
                if is_sep[m as usize] && local[m as usize] == ABSENT {
                    local[m as usize] = 0;
                    nodes.push(m);
                }
            }
        }
        for (i, &n) in nodes.iter().enumerate() {
            local[n as usize] = i as u32;
        }
        let inner = part.len() as u32;
        let sub: Vec<Vec<u32>> = nodes
            .iter()
            .enumerate()
            .map(|(i, &n)| {
                let mut row: Vec<u32> = (adj[n as usize].iter())
                    .map(|&m| local[m as usize])
                    .filter(|&m| m != ABSENT)
                    .collect();
                if i as u32 >= inner {
                    row.retain(|&m| m < inner);
                    row.extend((inner..nodes.len() as u32).filter(|&m| m != i as u32));
                }
                row.sort_unstable();
                row
            })
            .collect();
        match amd_order(&sub) {
            Some(o) => {
                order.extend((o.into_iter().filter(|&i| i < inner)).map(|i| part[i as usize]))
            }
            None => order.extend_from_slice(part),
        }
        for &n in &nodes {
            local[n as usize] = ABSENT;
        }
        bounds.push(order.len() as u32);
    }
    order.extend_from_slice(&separators);
    Some(Dissection { order, bounds })
}

/// Most parts a matrix is split into. Four cores are what the estimate in
/// [`BlockChol::new`] assumes; more parts than that only pay on long
/// structures, where they are nearly free.
const MAX_PARTS: usize = 8;
/// Fewest nodes of a part worth a thread of its own.
const MIN_PART_NODES: usize = 1024;
/// Most block products a split factorisation may cost, over those of the
/// unsplit one.
const MAX_SPLIT_WORK: f64 = 1.25;

/// Where the blocks are: the factor's block columns, the matrix's coupled
/// pairs, and the parts the columns fall into.
///
/// A *region* is a run of columns that is worked on as a unit: one part,
/// or all separators together. Rows of a region's columns are numbered
/// locally: a row inside the region by its distance from the region's
/// first column, a separator row of a part's column after those, by its
/// index in the part's list of separators.
struct Pattern {
    nn: usize,
    /// Part `p` is the columns `bounds[p]..bounds[p + 1]`; the separators
    /// are the columns from the last bound on.
    bounds: Vec<u32>,
    col_ptr: Vec<u32>,
    /// Local row of each block of the factor; ascending within a column.
    row_loc: Vec<u32>,
    /// Per column, the index of its first block in a separator row of
    /// another region (the end of the column if it has none).
    sep_from: Vec<u32>,
    /// Pairs sorted by column: `pair_ptr[j]..pair_ptr[j + 1]` are those
    /// whose earlier node is `j`.
    pair_ptr: Vec<u32>,
    pair_row: Vec<u32>,
    /// Slot of each pair's block in the factor.
    pair_l: Vec<u32>,
    /// For a pair of a part's column with a separator: the separator's
    /// index in the part's list.
    pair_sep: Vec<u32>,
    /// Per separator column, the parts with a share in it, as (part, index
    /// in the part's list): `share_ptr[k - sep]..share_ptr[k - sep + 1]`.
    share_ptr: Vec<u32>,
    share: Vec<(u32, u32)>,
}

impl Pattern {
    /// First separator column.
    fn sep(&self) -> usize {
        *self.bounds.last().expect("there is always a bound") as usize
    }

    /// Columns of part `p`.
    fn part(&self, p: usize) -> (usize, usize) {
        (self.bounds[p] as usize, self.bounds[p + 1] as usize)
    }

    fn col(&self, j: usize) -> (usize, usize) {
        (self.col_ptr[j] as usize, self.col_ptr[j + 1] as usize)
    }
}

/// Scratch of the left-looking factorisation of one region.
struct Scratch {
    /// Local row -> index in the current column.
    pos: Vec<u32>,
    // Per local row, the list of finished columns whose next block is in
    // that row.
    head: Vec<u32>,
    next: Vec<u32>,
    cur: Vec<u32>,
}

impl Scratch {
    fn new(columns: usize, rows: usize) -> Self {
        Self {
            pos: vec![0; rows],
            head: vec![ABSENT; columns],
            next: vec![ABSENT; columns],
            cur: vec![0; columns],
        }
    }
}

/// What one part keeps to itself while the parts are worked on at the same
/// time: everything it would otherwise write into separator rows.
struct Part {
    /// Separator columns this part's columns have blocks in, ascending.
    seps: Vec<u32>,
    /// What eliminating this part takes off the separators' matrix (minus
    /// its Schur complement): `seps.len()`² blocks, column-major, lower
    /// triangle.
    schur: Vec<B6>,
    /// Slot in the factor of each block of `schur` below the diagonal;
    /// `ABSENT` where the factor has none, and the block is zero.
    schur_slot: Vec<u32>,
    /// This part's share of the separator rows of a vector.
    buf: Vec<V6>,
    scratch: Scratch,
    /// Whether the part's last factorisation went through.
    ok: bool,
}

/// Run `compute` on every slot, on the thread pool if `par`, and `consume`
/// in slot order on the calling thread. The same arithmetic either way.
fn run_parts<T, C, K>(par: bool, slots: &mut [T], compute: C, mut consume: K)
where
    T: Send,
    C: Fn(usize, &mut T) + Sync,
    K: FnMut(usize, &mut T),
{
    if par {
        ordered_chunks(slots, slots.len() - 1, compute, consume);
    } else {
        for (p, slot) in slots.iter_mut().enumerate() {
            compute(p, slot);
            consume(p, slot);
        }
    }
}

#[inline(always)]
fn add_to(t: &mut V6, u: &V6) {
    for i in 0..6 {
        t[i] += u[i];
    }
}

/// t -= m y
#[inline(always)]
fn sub_mul<S: Simd>(t: &mut V6, m: &B6, y: &V6) {
    for k in 0..6 {
        let x = -y[k];
        for i in 0..6 {
            t[i] = madd::<S>(t[i], m[k * 6 + i], x);
        }
    }
}

/// Left-looking factorisation of the block columns `start..end`, which
/// take updates only from each other. `l` and `w` are the region's own
/// slices of the factor. `extra(k, d, col, s)` may add to the diagonal
/// block and the column of `k` (whose first block is slot `s`) before the
/// region's earlier columns are taken off. False if the matrix is not
/// positive definite.
#[inline(always)]
#[allow(clippy::too_many_arguments)]
fn factor_region<S: Simd>(
    pat: &Pattern,
    diag: &[B6],
    pairs: &[B6],
    (start, end): (usize, usize),
    l: &mut [B6],
    w: &mut [B6],
    sc: &mut Scratch,
    mut extra: impl FnMut(usize, &mut B6, &mut [B6], usize),
) -> bool {
    let len = end - start;
    let base = pat.col_ptr[start] as usize;
    sc.head.fill(ABSENT);
    for k in start..end {
        let (s, e) = pat.col(k);
        // Columns before k are finished; column k starts as that of A.
        let (done, rest) = l.split_at_mut(s - base);
        let col = &mut rest[..e - s];
        col.fill(ZERO);
        for (i, &r) in pat.row_loc[s..e].iter().enumerate() {
            sc.pos[r as usize] = i as u32;
        }
        for q in pat.pair_ptr[k] as usize..pat.pair_ptr[k + 1] as usize {
            col[pat.pair_l[q] as usize - s] = pairs[q];
        }
        let mut d = diag[k];
        extra(k, &mut d, col, s);
        // Every earlier column j with a block in row k updates column k.
        let kl = k - start;
        let mut j = sc.head[kl];
        while j != ABSENT {
            let ju = j as usize;
            let next_j = sc.next[ju];
            let c = sc.cur[ju] as usize;
            let end_j = pat.col_ptr[start + ju + 1] as usize;
            let b = &done[c - base];
            mul_sub_t::<S>(&mut d, b, b);
            for q in c + 1..end_j {
                let t = sc.pos[pat.row_loc[q] as usize] as usize;
                mul_sub_t::<S>(&mut col[t], &done[q - base], b);
            }
            if c + 1 < end_j {
                let r = pat.row_loc[c + 1] as usize;
                // Rows past the region are the separators': not ours.
                if r < len {
                    sc.cur[ju] = (c + 1) as u32;
                    sc.next[ju] = sc.head[r];
                    sc.head[r] = j;
                }
            }
            j = next_j;
        }
        let Some(wk) = chol_inverse(&d) else {
            return false;
        };
        for b in col.iter_mut() {
            mul_lower_t::<S>(b, &wk);
        }
        w[kl] = wk;
        if s < e {
            let r = pat.row_loc[s] as usize;
            if r < len {
                sc.cur[kl] = s as u32;
                sc.next[kl] = sc.head[r];
                sc.head[r] = kl as u32;
            }
        }
    }
    true
}

/// What the finished columns `start..end` of a part take off the
/// separators' matrix: `schur[c2 * m + c1] = -Σ L(c1, j) L(c2, j)ᵀ`.
#[inline(always)]
fn schur_region<S: Simd>(
    pat: &Pattern,
    (start, end): (usize, usize),
    l: &[B6],
    m: usize,
    schur: &mut [B6],
) {
    let len = end - start;
    let base = pat.col_ptr[start] as usize;
    schur.fill(ZERO);
    for j in start..end {
        let e = pat.col_ptr[j + 1] as usize;
        for q2 in pat.sep_from[j] as usize..e {
            let c2 = pat.row_loc[q2] as usize - len;
            let b = &l[q2 - base];
            for q1 in q2..e {
                let c1 = pat.row_loc[q1] as usize - len;
                mul_sub_t::<S>(&mut schur[c2 * m + c1], &l[q1 - base], b);
            }
        }
    }
}

/// Forward substitution through the columns `start..end`: `v` is the
/// region's slice of the vector, and what the columns take off separator
/// rows of other regions goes to `buf`.
#[inline(always)]
fn forward_region<S: Simd>(
    pat: &Pattern,
    (start, end): (usize, usize),
    l: &[B6],
    w: &[B6],
    v: &mut [V6],
    buf: &mut [V6],
) {
    let len = end - start;
    for j in start..end {
        let y = mul_vec::<S>(&w[j], &v[j - start]);
        v[j - start] = y;
        let (s, e) = pat.col(j);
        let f = pat.sep_from[j] as usize;
        for q in s..f {
            sub_mul::<S>(&mut v[pat.row_loc[q] as usize], &l[q], &y);
        }
        for q in f..e {
            sub_mul::<S>(&mut buf[pat.row_loc[q] as usize - len], &l[q], &y);
        }
    }
}

/// Backward substitution through the columns `start..end`; `buf` holds
/// the finished separator rows of other regions.
#[inline(always)]
fn backward_region<S: Simd>(
    pat: &Pattern,
    (start, end): (usize, usize),
    l: &[B6],
    w: &[B6],
    v: &mut [V6],
    buf: &[V6],
) {
    let len = end - start;
    for j in (start..end).rev() {
        let mut t = v[j - start];
        let (s, e) = pat.col(j);
        let f = pat.sep_from[j] as usize;
        for q in s..f {
            let u = mul_vec_t::<S>(&l[q], &v[pat.row_loc[q] as usize]);
            for i in 0..6 {
                t[i] -= u[i];
            }
        }
        for q in f..e {
            let u = mul_vec_t::<S>(&l[q], &buf[pat.row_loc[q] as usize - len]);
            for i in 0..6 {
                t[i] -= u[i];
            }
        }
        v[j - start] = mul_vec_t::<S>(&w[j], &t);
    }
}

/// The columns `start..end` of y += A v: `y` is the region's slice, and
/// what belongs in separator rows of other regions goes to `buf`.
#[inline(always)]
fn product_region<S: Simd>(
    pat: &Pattern,
    (start, end): (usize, usize),
    diag: &[B6],
    pairs: &[B6],
    v: &[V6],
    y: &mut [V6],
    buf: &mut [V6],
) {
    for j in start..end {
        let vj = &v[j];
        let mut acc = mul_vec::<S>(&diag[j], vj);
        for q in pat.pair_ptr[j] as usize..pat.pair_ptr[j + 1] as usize {
            let r = pat.pair_row[q] as usize;
            let m = &pairs[q];
            let u = mul_vec::<S>(m, vj);
            let t = mul_vec_t::<S>(m, &v[r]);
            if r < end {
                add_to(&mut y[r - start], &u);
            } else {
                add_to(&mut buf[pat.pair_sep[q] as usize], &u);
            }
            add_to(&mut acc, &t);
        }
        add_to(&mut y[j - start], &acc);
    }
}

// The kernels above, each as one dispatch to the instruction set. A part's
// work is dispatched on the thread that runs it.

struct FactorPart<'a> {
    pat: &'a Pattern,
    diag: &'a [B6],
    pairs: &'a [B6],
    region: (usize, usize),
    l: &'a mut [B6],
    w: &'a mut [B6],
    part: &'a mut Part,
}

impl WithSimd for FactorPart<'_> {
    type Output = ();
    #[inline(always)]
    fn with_simd<S: Simd>(self, _: S) {
        let Self {
            pat,
            diag,
            pairs,
            region,
            l,
            w,
            part,
        } = self;
        let no_extra = |_: usize, _: &mut B6, _: &mut [B6], _: usize| {};
        part.ok = factor_region::<S>(pat, diag, pairs, region, l, w, &mut part.scratch, no_extra);
        if part.ok {
            schur_region::<S>(pat, region, l, part.seps.len(), &mut part.schur);
        }
    }
}

struct FactorSeparators<'a> {
    pat: &'a Pattern,
    diag: &'a [B6],
    pairs: &'a [B6],
    l: &'a mut [B6],
    w: &'a mut [B6],
    parts: &'a [Part],
    scratch: &'a mut Scratch,
}

impl WithSimd for FactorSeparators<'_> {
    type Output = bool;
    #[inline(always)]
    fn with_simd<S: Simd>(self, _: S) -> bool {
        let Self {
            pat,
            diag,
            pairs,
            l,
            w,
            parts,
            scratch,
        } = self;
        let sep = pat.sep();
        // The separators' matrix is that of A less every part's share,
        // taken in part order.
        let shares = |k: usize, d: &mut B6, col: &mut [B6], s: usize| {
            let (from, to) = (pat.share_ptr[k - sep], pat.share_ptr[k - sep + 1]);
            for &(p, lk) in &pat.share[from as usize..to as usize] {
                let (part, lk) = (&parts[p as usize], lk as usize);
                let m = part.seps.len();
                let u = &part.schur[lk * m..(lk + 1) * m];
                let slot = &part.schur_slot[lk * m..(lk + 1) * m];
                for i in 0..36 {
                    d[i] += u[lk][i];
                }
                for li in lk + 1..m {
                    if slot[li] != ABSENT {
                        let t = &mut col[slot[li] as usize - s];
                        for i in 0..36 {
                            t[i] += u[li][i];
                        }
                    }
                }
            }
        };
        factor_region::<S>(pat, diag, pairs, (sep, pat.nn), l, w, scratch, shares)
    }
}

/// One direction of the triangular solves on one part.
struct SolvePart<'a> {
    pat: &'a Pattern,
    region: (usize, usize),
    l: &'a [B6],
    w: &'a [B6],
    v: &'a mut [V6],
    buf: &'a mut [V6],
    forward: bool,
}

impl WithSimd for SolvePart<'_> {
    type Output = ();
    #[inline(always)]
    fn with_simd<S: Simd>(self, _: S) {
        if self.forward {
            forward_region::<S>(self.pat, self.region, self.l, self.w, self.v, self.buf);
        } else {
            backward_region::<S>(self.pat, self.region, self.l, self.w, self.v, self.buf);
        }
    }
}

/// Both triangular solves on the separators, between the parts' two.
struct SolveSeparators<'a> {
    pat: &'a Pattern,
    l: &'a [B6],
    w: &'a [B6],
    v: &'a mut [V6],
}

impl WithSimd for SolveSeparators<'_> {
    type Output = ();
    #[inline(always)]
    fn with_simd<S: Simd>(self, _: S) {
        let region = (self.pat.sep(), self.pat.nn);
        forward_region::<S>(self.pat, region, self.l, self.w, self.v, &mut []);
        backward_region::<S>(self.pat, region, self.l, self.w, self.v, &[]);
    }
}

struct ProductRegion<'a> {
    pat: &'a Pattern,
    region: (usize, usize),
    diag: &'a [B6],
    pairs: &'a [B6],
    v: &'a [V6],
    y: &'a mut [V6],
    buf: &'a mut [V6],
}

impl WithSimd for ProductRegion<'_> {
    type Output = ();
    #[inline(always)]
    fn with_simd<S: Simd>(self, _: S) {
        product_region::<S>(
            self.pat,
            self.region,
            self.diag,
            self.pairs,
            self.v,
            self.y,
            self.buf,
        );
    }
}

/// Split `v`, which covers the columns of all parts, into one slice per
/// part, each with its part.
fn part_slots<'a, T>(
    pat: &Pattern,
    mut v: &'a mut [T],
    parts: &'a mut [Part],
) -> Vec<(&'a mut [T], &'a mut Part)> {
    let mut slots = Vec::with_capacity(parts.len());
    for (p, part) in parts.iter_mut().enumerate() {
        let (start, end) = pat.part(p);
        let (mine, rest) = std::mem::take(&mut v).split_at_mut(end - start);
        v = rest;
        slots.push((mine, part));
    }
    slots
}

pub(crate) struct BlockChol {
    /// Compact DOF -> index in the solver's vector layout.
    dof_pos: Vec<u32>,
    /// Flat indices into `a` of the diagonal entries of fixed DOFs.
    fixed: Vec<u32>,
    /// The matrix: a block per node (lower triangle assembled, mirrored by
    /// `finish`), then one block per coupled node pair, rows at the node
    /// eliminated later.
    a: Vec<B6>,
    /// Per beam, the flat index into `a` of each packed lower-triangle entry.
    beam_pos: Vec<[u32; BLOCK_NNZ]>,
    /// Per beam, its blocks when it takes the fast path of `add_block`.
    target: Vec<Target>,
    pat: Pattern,
    /// Off-diagonal blocks of the factor, by column.
    l: Vec<B6>,
    /// Inverse of each diagonal block of the factor (lower triangular).
    w: Vec<B6>,
    parts: Vec<Part>,
    sep_scratch: Scratch,
    /// Give every part its own thread from this many nodes up.
    par_min_nodes: usize,
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

/// Separator columns each part's columns have blocks in, ascending.
fn part_separators(sym: &Symbolic, bounds: &[u32]) -> Vec<Vec<u32>> {
    let sep = *bounds.last().expect("there is always a bound");
    bounds
        .windows(2)
        .map(|b| {
            let (s, e) = (sym.col_ptr[b[0] as usize], sym.col_ptr[b[1] as usize]);
            let mut seps: Vec<u32> = (sym.row_idx[s as usize..e as usize].iter())
                .copied()
                .filter(|&r| r >= sep)
                .collect();
            seps.sort_unstable();
            seps.dedup();
            seps
        })
        .collect()
}

impl BlockChol {
    pub fn new(layout: &NodeLayout) -> Self {
        Self::with_parts(layout, MAX_PARTS, MIN_PART_NODES, MAX_SPLIT_WORK)
    }

    /// As [`new`](Self::new), splitting the matrix into at most `max_parts`
    /// parts of at least `min_part_nodes` nodes, if that costs at most
    /// `max_work` times the block products of the unsplit factorisation.
    pub fn with_parts(
        layout: &NodeLayout,
        max_parts: usize,
        min_part_nodes: usize,
        max_work: f64,
    ) -> Self {
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
        let mut sym = symbolic(&adj, order);
        let mut bounds = vec![0, nn as u32];
        // Parts that do not couple to each other can be factorised and
        // solved at the same time, at the price of more fill next to the
        // separators. Take the split that is cheapest on four cores, if any
        // is: on a bridge the separators are a handful of nodes and four
        // parts cost 15% more products; a 3D grid has no small separator.
        // The split depends on the structure alone, never on the machine,
        // so that the results do not either.
        let whole = sym.factor_products;
        let mut least = whole;
        let mut parts = 2;
        while parts <= max_parts && nn / parts >= min_part_nodes.max(1) {
            if let Some(cut) = dissect(&adj, parts) {
                let trial = symbolic(&adj, cut.order);
                let sep = *cut.bounds.last().expect("there is always a bound") as usize;
                let serial: f64 = (trial.col_ptr[sep..].windows(2))
                    .map(|w| {
                        let c = (w[1] - w[0]) as f64;
                        (c + 1.0) * (c + 2.0) / 2.0
                    })
                    .sum();
                let threads = (cut.bounds.len() - 1).min(4) as f64;
                let cost = serial + (trial.factor_products - serial) / threads;
                let shares: usize = (part_separators(&trial, &cut.bounds).iter())
                    .map(|s| s.len() * s.len())
                    .sum();
                if trial.factor_products <= max_work * whole
                    && shares <= 2 * trial.row_idx.len()
                    && cost < least
                {
                    least = cost;
                    bounds = cut.bounds;
                    sym = trial;
                }
            }
            parts *= 2;
        }
        let part_seps = part_separators(&sym, &bounds);
        let Symbolic {
            order,
            col_ptr,
            row_idx,
            factor_products,
        } = sym;
        let sep = *bounds.last().expect("there is always a bound") as usize;
        let mut inv = vec![0u32; nn];
        for (new, &old) in order.iter().enumerate() {
            inv[old as usize] = new as u32;
        }

        // Local rows of the factor's blocks.
        let mut row_loc = vec![0u32; row_idx.len()];
        let mut sep_from = vec![0u32; nn];
        for (p, seps) in part_seps.iter().enumerate() {
            let (start, end) = (bounds[p] as usize, bounds[p + 1] as usize);
            for j in start..end {
                let (s, e) = (col_ptr[j] as usize, col_ptr[j + 1] as usize);
                sep_from[j] = e as u32;
                for q in s..e {
                    let r = row_idx[q];
                    row_loc[q] = if (r as usize) < sep {
                        r - start as u32
                    } else {
                        sep_from[j] = sep_from[j].min(q as u32);
                        let at = seps.binary_search(&r).expect("listed separator");
                        (end - start + at) as u32
                    };
                }
            }
        }
        for j in sep..nn {
            let (s, e) = (col_ptr[j] as usize, col_ptr[j + 1] as usize);
            sep_from[j] = e as u32;
            for q in s..e {
                row_loc[q] = row_idx[q] - sep as u32;
            }
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
        let mut pair_sep = vec![0u32; pair_row.len()];
        for (p, seps) in part_seps.iter().enumerate() {
            let (from, to) = (bounds[p] as usize, bounds[p + 1] as usize);
            for q in pair_ptr[from] as usize..pair_ptr[to] as usize {
                if pair_row[q] as usize >= sep {
                    let at = seps.binary_search(&pair_row[q]);
                    pair_sep[q] = at.expect("listed separator") as u32;
                }
            }
        }
        let pair_of = |row: u32, col: u32| {
            let (s, e) = (
                pair_ptr[col as usize] as usize,
                pair_ptr[col as usize + 1] as usize,
            );
            s + pair_row[s..e].binary_search(&row).expect("pair exists")
        };

        // Each part's share of the separators.
        let mut share_ptr = vec![0u32];
        let mut share = Vec::new();
        for k in sep..nn {
            for (p, seps) in part_seps.iter().enumerate() {
                if let Ok(at) = seps.binary_search(&(k as u32)) {
                    share.push((p as u32, at as u32));
                }
            }
            share_ptr.push(share.len() as u32);
        }
        let parts: Vec<Part> = (part_seps.into_iter().enumerate())
            .map(|(p, seps)| {
                let m = seps.len();
                let mut schur_slot = vec![ABSENT; m * m];
                for (lk, &k) in seps.iter().enumerate() {
                    let (s, e) = (
                        col_ptr[k as usize] as usize,
                        col_ptr[k as usize + 1] as usize,
                    );
                    for (li, r) in seps.iter().enumerate().skip(lk + 1) {
                        if let Ok(at) = row_idx[s..e].binary_search(r) {
                            schur_slot[lk * m + li] = (s + at) as u32;
                        }
                    }
                }
                let columns = (bounds[p + 1] - bounds[p]) as usize;
                Part {
                    schur: vec![ZERO; m * m],
                    schur_slot,
                    buf: vec![[0.0; 6]; m],
                    scratch: Scratch::new(columns, columns + m),
                    ok: true,
                    seps,
                }
            })
            .collect();

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
            dof_pos,
            fixed,
            a: vec![ZERO; nn + pair_row.len()],
            beam_pos,
            target,
            l: vec![ZERO; row_loc.len()],
            w: vec![ZERO; nn],
            parts,
            sep_scratch: Scratch::new(nn - sep, nn - sep),
            pat: Pattern {
                nn,
                bounds,
                col_ptr,
                row_loc,
                sep_from,
                pair_ptr,
                pair_row,
                pair_l,
                pair_sep,
                share_ptr,
                share,
            },
            par_min_nodes: 0,
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
        2.1 * (self.factor_products + 6.0 * self.pat.nn as f64) / self.iteration_products.max(1.0)
    }

    /// Length of the vectors `apply` and `matvec` work on.
    pub fn dim(&self) -> usize {
        6 * self.pat.nn
    }

    /// Scalar entries stored in the factor.
    pub fn factor_entries(&self) -> usize {
        36 * self.l.len() + 21 * self.pat.nn
    }

    /// Parts the matrix is split into: they are factorised and solved
    /// independently of each other. One if it is not split.
    pub fn parts(&self) -> usize {
        self.parts.len()
    }

    /// Give every part its own thread when the matrix has at least this
    /// many nodes; `usize::MAX` keeps everything on the calling thread.
    /// The result is the same either way.
    pub fn set_par_min_nodes(&mut self, nodes: usize) {
        self.par_min_nodes = nodes;
    }

    /// Whether factorisations, solves and products use the thread pool.
    pub fn runs_parallel(&self) -> bool {
        self.parts.len() > 1 && self.pat.nn >= self.par_min_nodes
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
        for d in &mut self.a[..self.pat.nn] {
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
    ///
    /// The parts go first, each on its own thread if there are several:
    /// their columns, and what those take off the separators. The
    /// separators follow on the calling thread.
    pub fn factor(&mut self) -> bool {
        let par = self.runs_parallel();
        let (pat, arch) = (&self.pat, self.arch);
        let (diag, pairs) = self.a.split_at(pat.nn);
        let sep = pat.sep();
        let (l_parts, l_sep) = self.l.split_at_mut(pat.col_ptr[sep] as usize);
        let (w_parts, w_sep) = self.w.split_at_mut(sep);
        let mut slots = Vec::with_capacity(self.parts.len());
        let (mut l_rest, mut w_rest) = (l_parts, w_parts);
        for (p, part) in self.parts.iter_mut().enumerate() {
            let (start, end) = pat.part(p);
            let blocks = (pat.col_ptr[end] - pat.col_ptr[start]) as usize;
            let (l, rest) = std::mem::take(&mut l_rest).split_at_mut(blocks);
            l_rest = rest;
            let (w, rest) = std::mem::take(&mut w_rest).split_at_mut(end - start);
            w_rest = rest;
            slots.push((l, w, part));
        }
        let mut ok = true;
        run_parts(
            par,
            &mut slots,
            |p, (l, w, part)| {
                arch.dispatch(FactorPart {
                    pat,
                    diag,
                    pairs,
                    region: pat.part(p),
                    l,
                    w,
                    part,
                });
            },
            |_, (_, _, part)| ok &= part.ok,
        );
        drop(slots);
        ok && (sep == pat.nn
            || arch.dispatch(FactorSeparators {
                pat,
                diag,
                pairs,
                l: l_sep,
                w: w_sep,
                parts: &self.parts,
                scratch: &mut self.sep_scratch,
            }))
    }

    /// v <- (L Lᵀ)⁻¹ v with the stored factorisation.
    pub fn apply(&mut self, v: &mut [f64]) {
        let par = self.runs_parallel();
        let (pat, arch, l, w) = (&self.pat, self.arch, &self.l[..], &self.w[..]);
        let sep = pat.sep();
        let (v, _) = v.as_chunks_mut::<6>();
        let (v_parts, v_sep) = v.split_at_mut(sep);
        let mut slots = part_slots(pat, v_parts, &mut self.parts);
        // Forward through the parts, then their shares of the separator
        // rows in part order.
        run_parts(
            par,
            &mut slots,
            |p, (v, part)| {
                part.buf.fill([0.0; 6]);
                arch.dispatch(SolvePart {
                    pat,
                    region: pat.part(p),
                    l,
                    w,
                    v,
                    buf: &mut part.buf,
                    forward: true,
                });
            },
            |_, (_, part)| {
                for (&r, b) in part.seps.iter().zip(&part.buf) {
                    add_to(&mut v_sep[r as usize - sep], b);
                }
            },
        );
        if sep < pat.nn {
            arch.dispatch(SolveSeparators {
                pat,
                l,
                w,
                v: v_sep,
            });
        }
        let v_sep = &*v_sep;
        run_parts(
            par,
            &mut slots,
            |p, (v, part)| {
                for (&r, b) in part.seps.iter().zip(&mut part.buf) {
                    *b = v_sep[r as usize - sep];
                }
                arch.dispatch(SolvePart {
                    pat,
                    region: pat.part(p),
                    l,
                    w,
                    v,
                    buf: &mut part.buf,
                    forward: false,
                });
            },
            |_, _| {},
        );
    }

    /// y <- A v
    pub fn matvec(&mut self, v: &[f64], y: &mut [f64]) {
        let par = self.runs_parallel();
        let (pat, arch) = (&self.pat, self.arch);
        let (diag, pairs) = self.a.split_at(pat.nn);
        let sep = pat.sep();
        let (v, _) = v.as_chunks::<6>();
        let (y, _) = y.as_chunks_mut::<6>();
        let (y_parts, y_sep) = y.split_at_mut(sep);
        y_sep.fill([0.0; 6]);
        let mut slots = part_slots(pat, y_parts, &mut self.parts);
        run_parts(
            par,
            &mut slots,
            |p, (y, part)| {
                y.fill([0.0; 6]);
                part.buf.fill([0.0; 6]);
                arch.dispatch(ProductRegion {
                    pat,
                    region: pat.part(p),
                    diag,
                    pairs,
                    v,
                    y,
                    buf: &mut part.buf,
                });
            },
            |_, (_, part)| {
                for (&r, b) in part.seps.iter().zip(&part.buf) {
                    add_to(&mut y_sep[r as usize - sep], b);
                }
            },
        );
        if sep < pat.nn {
            arch.dispatch(ProductRegion {
                pat,
                region: (sep, pat.nn),
                diag,
                pairs,
                v,
                y: y_sep,
                buf: &mut [],
            });
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

    /// A ladder of `rungs` rungs of `width` nodes, braced: long and thin,
    /// so it splits into parts with small separators.
    fn ladder(width: usize, rungs: usize) -> (usize, Vec<(usize, usize)>) {
        let at = |x: usize, y: usize| x * width + y;
        let mut pairs = Vec::new();
        for x in 0..rungs {
            for y in 0..width {
                if y + 1 < width {
                    pairs.push((at(x, y), at(x, y + 1)));
                }
                if x + 1 < rungs {
                    pairs.push((at(x + 1, y), at(x, y)));
                    if y + 1 < width && (x + y) % 3 == 0 {
                        pairs.push((at(x, y), at(x + 1, y + 1)));
                    }
                }
            }
        }
        (width * rungs, pairs)
    }

    /// A split and an unsplit factorisation of one random matrix on the
    /// graph: (split, whole, right-hand side in compact numbering).
    fn split_and_whole(
        nodes: usize,
        pairs: &[(usize, usize)],
        rng: &mut Rng,
    ) -> (BlockChol, BlockChol, Vec<f64>) {
        let n = 6 * nodes;
        let blocks: Vec<_> = pairs
            .iter()
            .map(|&(a, b)| [Some(6 * a), Some(6 * a + 3), Some(6 * b), Some(6 * b + 3)])
            .collect();
        let layout = NodeLayout::new(n, &blocks);
        let mut split = BlockChol::with_parts(&layout, 8, 12, 4.0);
        let mut whole = BlockChol::with_parts(&layout, 1, 12, 4.0);
        split.clear();
        whole.clear();
        for d in 0..n {
            let v = 0.5 + rng.unit();
            split.add_diagonal(d, v);
            whole.add_diagonal(d, v);
        }
        for i in 0..blocks.len() {
            let k = random_block(rng);
            split.add_block(i, &k);
            whole.add_block(i, &k);
        }
        split.finish();
        whole.finish();
        let b = (0..n).map(|_| rng.unit() - 0.5).collect();
        (split, whole, b)
    }

    /// (A⁻¹ b, A b) in compact numbering.
    fn solve_and_multiply(chol: &mut BlockChol, b: &[f64]) -> (Vec<f64>, Vec<f64>) {
        let n = b.len();
        let (mut bw, mut yw) = (vec![0.0; n], vec![0.0; n]);
        let (mut x, mut y) = (vec![0.0; n], vec![0.0; n]);
        chol.load(b, &mut bw);
        chol.matvec(&bw, &mut yw);
        chol.store(&yw, &mut y);
        assert!(chol.factor());
        chol.apply(&mut bw);
        chol.store(&bw, &mut x);
        (x, y)
    }

    fn distance(a: &[f64], b: &[f64]) -> f64 {
        let norm = |v: &[f64]| v.iter().map(|x| x * x).sum::<f64>().sqrt();
        let diff: Vec<f64> = a.iter().zip(b).map(|(a, b)| a - b).collect();
        norm(&diff) / norm(a)
    }

    /// A matrix split into parts gives the solution and the product of the
    /// unsplit one, on both instruction sets.
    #[test]
    fn split_matrix_matches_the_whole() {
        let mut rng = Rng(0xA0761D6478BD642F);
        for (width, rungs, least_parts) in [(3, 60, 4), (2, 150, 4), (5, 40, 2)] {
            let (nodes, pairs) = ladder(width, rungs);
            let (mut split, mut whole, b) = split_and_whole(nodes, &pairs, &mut rng);
            assert!(
                split.parts() >= least_parts,
                "{width}x{rungs}: only {} parts",
                split.parts()
            );
            assert_eq!(whole.parts(), 1);
            let (x0, y0) = solve_and_multiply(&mut whole, &b);
            for level in [SimdLevel::Scalar, SimdLevel::Native] {
                split.set_simd(level);
                let (x, y) = solve_and_multiply(&mut split, &b);
                let (dx, dy) = (distance(&x0, &x), distance(&y0, &y));
                assert!(
                    dx < 1e-10,
                    "{width}x{rungs} on {level:?}: solution off by {dx:e}"
                );
                assert!(
                    dy < 1e-13,
                    "{width}x{rungs} on {level:?}: product off by {dy:e}"
                );
            }
        }
    }

    /// The parts on separate threads give the single-thread result bit for
    /// bit, on any number of threads.
    #[test]
    fn threaded_parts_are_bit_identical() {
        let mut rng = Rng(0xE7037ED1A0B428DB);
        let (nodes, pairs) = ladder(3, 80);
        let (mut split, _, b) = split_and_whole(nodes, &pairs, &mut rng);
        assert!(split.parts() >= 4);
        split.set_par_min_nodes(usize::MAX);
        assert!(!split.runs_parallel());
        let single = solve_and_multiply(&mut split, &b);
        split.set_par_min_nodes(0);
        assert!(split.runs_parallel());
        for threads in [1, 2, 3, 8] {
            let pool = rayon::ThreadPoolBuilder::new()
                .num_threads(threads)
                .build()
                .expect("a thread pool");
            for _ in 0..20 {
                let threaded = pool.install(|| solve_and_multiply(&mut split, &b));
                assert!(single == threaded, "{threads} threads change the result");
            }
        }
    }

    /// A matrix that is not positive definite is reported from a part and
    /// from the separators alike.
    #[test]
    fn split_matrix_reports_an_indefinite_block() {
        let mut rng = Rng(0x8EBC6AF09C88C6E3);
        let (nodes, pairs) = ladder(3, 60);
        let (mut split, _, _) = split_and_whole(nodes, &pairs, &mut rng);
        assert!(split.factor());
        let sep = split.pat.sep();
        assert!(sep < nodes);
        for column in [0, sep / 2, sep, nodes - 1] {
            let saved = split.a[column];
            split.a[column][0] = -1.0e9;
            assert!(!split.factor(), "column {column} went unnoticed");
            split.a[column] = saved;
            assert!(split.factor());
        }
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
