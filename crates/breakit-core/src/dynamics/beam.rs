//! Per-member corotational beam: end-node geometry, strains, the elastic-plastic
//! response, and the six compliant constraints and energy of one member.

use super::plastic::{return_1d, return_bend};
use super::*;

/// Eigenvectors and eigenvalues (clamped at zero) of a symmetric 2x2 matrix.
pub(super) fn eigen2(k: [[f64; 2]; 2]) -> [([f64; 2], f64); 2] {
    let ang = 0.5 * (2.0 * k[0][1]).atan2(k[0][0] - k[1][1]);
    let (sn, cs) = ang.sin_cos();
    [[cs, sn], [-sn, cs]].map(|v| {
        let lam = k[0][0] * v[0] * v[0] + 2.0 * k[0][1] * v[0] * v[1] + k[1][1] * v[1] * v[1];
        (v, lam.max(0.0))
    })
}
impl DynamicSim {
    pub(super) fn geom(&self, i: usize) -> Geom {
        let bm = &self.beams[i];
        let (ba, bb) = (&self.bodies[bm.a], &self.bodies[bm.b]);
        let d = bb.x - ba.x;
        let len = d.norm();
        let e = d / len;
        let ta = ba.q * bm.ta;
        let tb = bb.q * bm.tb;
        let na = ba.q * bm.na;
        let nb = bb.q * bm.nb;
        let n = na + nb;
        let y = (n - e * e.dot(&n)).normalize();
        let z = e.cross(&y);
        Geom {
            e,
            len,
            ta,
            tb,
            na,
            nb,
            axes: [y, z],
        }
    }

    pub(super) fn strains(&self, i: usize, gm: &Geom) -> Strains {
        let bm = &self.beams[i];
        let th = std::array::from_fn(|axis| {
            let u = gm.axes[axis];
            [
                gm.e.cross(&gm.ta).dot(&u) - bm.hinge[axis][0],
                gm.e.cross(&gm.tb).dot(&u) - bm.hinge[axis][1],
            ]
        });
        Strains {
            axial: gm.len - bm.l0 - bm.plastic_elongation,
            twist: gm.na.cross(&gm.nb).dot(&gm.e),
            th,
        }
    }

    /// Internal forces for `st`. `r` = τ/h of the plastic return, `None` for
    /// an elastic response.
    pub(super) fn response(&self, i: usize, st: &Strains, r: Option<f64>) -> Response {
        let bm = &self.beams[i];
        // Tension and compression may differ in stiffness (cracked concrete)
        // and yield force; the energy stays continuous through zero.
        let sense = if st.axial > 0.0 { 0 } else { 1 };
        let k = if sense == 0 { bm.k_tension } else { bm.k_axial };
        let (fy, rr) = (
            if r.is_some() {
                bm.n_cap[sense]
            } else {
                f64::INFINITY
            },
            r.unwrap_or(0.0),
        );
        let (n, kt_axial, mut energy, dp_axial) = return_1d(k, st.axial, fy, rr);
        energy += 0.5 * bm.k_torsion * st.twist * st.twist;
        let mut out = Response {
            n,
            kt_axial,
            dp_axial,
            m: [[0.0; 2]; 2],
            kt: [[[0.0; 2]; 2]; 2],
            dp: [[0.0; 2]; 2],
            energy,
            yield_mask: (dp_axial != 0.0) as u8,
        };
        for axis in 0..2 {
            let mp = if r.is_some() {
                bm.end_mp(axis)
            } else {
                [f64::INFINITY; 2]
            };
            let (m, kt, e, dp) = return_bend(bm.bending(axis), st.th[axis], mp, rr);
            out.m[axis] = m;
            out.kt[axis] = kt;
            out.dp[axis] = dp;
            out.energy += e;
            for (end, d) in dp.iter().enumerate() {
                out.yield_mask |= ((*d != 0.0) as u8) << (1 + axis * 2 + end);
            }
        }
        out
    }

    /// The six generalised forces of a beam at the current state and its
    /// energy: (force, tangent stiffness, gradients [x_a, φ_a, x_b, φ_b]).
    /// The nodal forces are −Σ force·gradient and the Gauss-Newton stiffness is
    /// Σ stiffness·gradient·gradientᵀ. Elastic: force = k·C and energy = Σ ½kC².
    #[allow(clippy::type_complexity)]
    pub(super) fn beam_constraints(
        &self,
        i: usize,
        r: Option<f64>,
    ) -> ([(f64, f64, [Vec3; 4]); 6], f64) {
        let gm = self.geom(i);
        let bm = &self.beams[i];
        let st = self.strains(i, &gm);
        let resp = self.response(i, &st, r);
        let z = Vec3::zeros();
        let mut out = [(0.0, 0.0, [z; 4]); 6];
        out[0] = (resp.n, resp.kt_axial, [-gm.e, z, gm.e, z]);
        out[1] = (
            bm.k_torsion * st.twist,
            bm.k_torsion,
            [
                z,
                gm.na.cross(&gm.nb.cross(&gm.e)),
                z,
                gm.nb.cross(&gm.e.cross(&gm.na)),
            ],
        );
        let p = |v: Vec3| v - gm.e * gm.e.dot(&v);
        for axis in 0..2 {
            let u = gm.axes[axis];
            let ga = p(gm.ta.cross(&u)) / gm.len;
            let gb = p(gm.tb.cross(&u)) / gm.len;
            let pa = gm.ta.cross(&u.cross(&gm.e));
            let pb = gm.tb.cross(&u.cross(&gm.e));
            // The 2x2 tangent between the end moments as two scalar terms
            // along its eigenvectors. Elastic: θa + θb and θa − θb.
            let m = resp.m[axis];
            for (j, (v, lam)) in eigen2(resp.kt[axis]).into_iter().enumerate() {
                out[2 + axis * 2 + j] = (
                    m[0] * v[0] + m[1] * v[1],
                    lam.max(0.0),
                    [
                        -(ga * v[0] + gb * v[1]),
                        pa * v[0],
                        ga * v[0] + gb * v[1],
                        pb * v[1],
                    ],
                );
            }
        }
        (out, resp.energy)
    }

    /// Length and response of a beam alone, for the step check (which
    /// needs the energy) and the material update after it.
    pub(super) fn beam_response(&self, i: usize, r: Option<f64>) -> (f64, Response) {
        let gm = self.geom(i);
        (gm.len, self.response(i, &self.strains(i, &gm), r))
    }
}
