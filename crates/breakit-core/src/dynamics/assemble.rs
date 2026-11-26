//! Element assembly kernels of the direct path: one member's forces and its
//! 12x12 Gauss-Newton block, written once over [`pulp::Simd`] and run on
//! [`SimConfig::simd`] (see [`crate::simd`]), one dispatch per chunk of
//! members.
//!
//! Geometry, strains and the material response are the scalar code of
//! `beam.rs` and `plastic.rs`, called as they are: only the products that
//! follow them (K = Σ k g gᵀ and f = Σ f g) are vectorised.

use super::*;
use crate::simd::madd;
use pulp::{Simd, WithSimd};

/// One beam's contribution to the substep system.
#[derive(Clone)]
pub(super) struct BeamTerms {
    pub(super) live: bool,
    pub(super) energy: f64,
    /// Elastic force on [x_a, φ_a, x_b, φ_b].
    pub(super) force: [f64; 12],
    /// Packed lower triangle of the 12x12 stiffness block.
    pub(super) block: [f64; BLOCK_NNZ],
}

impl BeamTerms {
    pub(super) const EMPTY: Self = Self {
        live: false,
        energy: 0.0,
        force: [0.0; 12],
        block: [0.0; BLOCK_NNZ],
    };
}

/// The first `W` entries of row `r` of Σ w gᵀ, where `w[t]` is `g[t]` times
/// the stiffness of term `t`. `W` is a multiple of the vector width, so the
/// loop compiles to whole vectors; entries past the diagonal are computed
/// and thrown away.
#[inline(always)]
fn row<S: Simd, const W: usize>(w: &[[f64; 12]; 6], g: &[[f64; 12]; 6], r: usize) -> [f64; W] {
    let mut acc = [0.0; W];
    for t in 0..6 {
        let wr = w[t][r];
        for c in 0..W {
            acc[c] = madd::<S>(acc[c], wr, g[t][c]);
        }
    }
    acc
}

impl DynamicSim {
    /// Forces and stiffness block of beam `i` at the current state. `r` is
    /// the plastic ratio of [`DynamicSim::plastic_ratio`].
    #[inline(always)]
    pub(super) fn beam_terms<S: Simd>(&self, i: usize, r: Option<f64>, out: &mut BeamTerms) {
        out.live = !self.beams[i].broken;
        if !out.live {
            return;
        }
        let (terms, energy) = self.beam_constraints(i, r);
        out.energy = energy;
        // Gradients as flat rows of 12, and the same times the stiffness.
        let mut g = [[0.0; 12]; 6];
        let mut w = [[0.0; 12]; 6];
        let mut force = [0.0; 12];
        for (t, (f, k, grad)) in terms.iter().enumerate() {
            for j in 0..12 {
                g[t][j] = grad[j / 3][j % 3];
            }
            for j in 0..12 {
                w[t][j] = k * g[t][j];
                force[j] = madd::<S>(force[j], *f, g[t][j]);
            }
        }
        out.force = force;
        // Rank-6 update, lower triangle only, a row at a time in whole
        // vectors of 4. A row's padding lands on the start of the next row,
        // which is written after it.
        let kb = &mut out.block;
        for r in 0..4 {
            kb[tri(r, 0)..][..4].copy_from_slice(&row::<S, 4>(&w, &g, r));
        }
        for r in 4..8 {
            kb[tri(r, 0)..][..8].copy_from_slice(&row::<S, 8>(&w, &g, r));
        }
        for r in 8..12 {
            kb[tri(r, 0)..][..12].copy_from_slice(&row::<S, 12>(&w, &g, r));
        }
        if self.config.tension_stiffness {
            // Geometric stiffness of the axial force, (N/L)(I − e eᵀ) on
            // the relative translation of the ends: the restoring force of
            // a taut string. Tension only. Under compression the term is
            // negative and could make the matrix indefinite; leaving it
            // out there only makes the step slightly short.
            let (n, e) = (terms[0].0, terms[0].2[2]);
            if n > 0.0 {
                let (a, b) = (self.beams[i].a, self.beams[i].b);
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
    }
}

/// Fills `out[k]` with the terms of beam `first + k`.
pub(super) struct ComputeChunk<'a> {
    pub(super) sim: &'a DynamicSim,
    pub(super) r: Option<f64>,
    pub(super) first: usize,
    pub(super) out: &'a mut [BeamTerms],
}

impl WithSimd for ComputeChunk<'_> {
    type Output = ();
    #[inline(always)]
    fn with_simd<S: Simd>(self, _: S) {
        for (k, out) in self.out.iter_mut().enumerate() {
            self.sim.beam_terms::<S>(self.first + k, self.r, out);
        }
    }
}

/// The single-thread path: every beam computed and added to the system
/// straight away, in beam order. Returns the sum of the beam energies.
pub(super) struct AssembleSerial<'a> {
    pub(super) sim: &'a DynamicSim,
    pub(super) r: Option<f64>,
    pub(super) sys: &'a mut DirectSystem,
}

impl WithSimd for AssembleSerial<'_> {
    type Output = f64;
    #[inline(always)]
    fn with_simd<S: Simd>(self, _: S) -> f64 {
        let mut phi = 0.0;
        let mut terms = BeamTerms::EMPTY;
        for i in 0..self.sim.beams.len() {
            self.sim.beam_terms::<S>(i, self.r, &mut terms);
            if terms.live {
                phi += terms.energy;
                self.sys.add_force(&self.sim.beams[i], &terms.force);
                self.sys.lin.add_block(i, &terms.block);
            }
        }
        phi
    }
}
