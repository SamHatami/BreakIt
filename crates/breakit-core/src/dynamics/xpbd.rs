//! The Gauss-Seidel XPBD path ([`Solver::GaussSeidel`](super::Solver)): one
//! member at a time, positional corrections applied to its end nodes.

use super::beam::eigen2;
use super::*;

impl DynamicSim {
    pub(super) fn solve_beam(&mut self, i: usize, h: f64) {
        // Axial.
        let gm = self.geom(i);
        let bm = &self.beams[i];
        let c = gm.len - bm.l0 - bm.plastic_elongation;
        let k = bm.k_axial;
        self.apply(i, 0, c, -gm.e, gm.e, Vec3::zeros(), Vec3::zeros(), k, h);

        // Torsion.
        let gm = self.geom(i);
        let c = gm.na.cross(&gm.nb).dot(&gm.e);
        let gpa = gm.na.cross(&gm.nb.cross(&gm.e));
        let gpb = gm.nb.cross(&gm.e.cross(&gm.na));
        let k = self.beams[i].k_torsion;
        self.apply(i, 1, c, Vec3::zeros(), Vec3::zeros(), gpa, gpb, k, h);

        // Bending, two planes, along the eigenvectors of each plane's end
        // stiffness (rigid ends: the symmetric and antisymmetric modes).
        for axis in 0..2 {
            let kb = self.beams[i].bending(axis);
            for (mode, (v, k)) in eigen2(kb).into_iter().enumerate() {
                if k <= 0.0 {
                    continue; // a pinned end: nothing to resist
                }
                let gm = self.geom(i);
                let bm = &self.beams[i];
                let u = gm.axes[axis];
                let p = |v: Vec3| v - gm.e * gm.e.dot(&v);
                let th_a = gm.e.cross(&gm.ta).dot(&u) - bm.hinge[axis][0];
                let th_b = gm.e.cross(&gm.tb).dot(&u) - bm.hinge[axis][1];
                let gxa_b = p(gm.ta.cross(&u)) / gm.len; // dθa/dxb
                let gxb_b = p(gm.tb.cross(&u)) / gm.len; // dθb/dxb
                let gpa = gm.ta.cross(&u.cross(&gm.e)) * v[0];
                let gpb = gm.tb.cross(&u.cross(&gm.e)) * v[1];
                let c = th_a * v[0] + th_b * v[1];
                let gxb = gxa_b * v[0] + gxb_b * v[1];
                self.apply(i, 2 + axis * 2 + mode, c, -gxb, gxb, gpa, gpb, k, h);
            }
        }
    }

    /// One XPBD projection of a scalar constraint with stiffness `k`.
    #[allow(clippy::too_many_arguments)]
    pub(super) fn apply(
        &mut self,
        i: usize,
        slot: usize,
        c: f64,
        gxa: Vec3,
        gxb: Vec3,
        gpa: Vec3,
        gpb: Vec3,
        k: f64,
        h: f64,
    ) {
        let (a, b) = (self.beams[i].a, self.beams[i].b);
        let (wa, ia) = (self.bodies[a].inv_mass, self.bodies[a].inv_inertia);
        let (wb, ib) = (self.bodies[b].inv_mass, self.bodies[b].inv_inertia);
        let alpha = 1.0 / (k * h * h);
        let denom = wa * gxa.norm_squared()
            + wb * gxb.norm_squared()
            + ia * gpa.norm_squared()
            + ib * gpb.norm_squared()
            + alpha;
        if denom <= alpha * (1.0 + 1e-12) {
            return; // both ends immovable for this constraint
        }
        let lambda = self.beams[i].lambda[slot];
        let dl = (-c - alpha * lambda) / denom;
        self.beams[i].lambda[slot] += dl;
        let ba = &mut self.bodies[a];
        ba.x += gxa * (wa * dl);
        if ia > 0.0 {
            ba.q = rotate(ba.q, gpa * (ia * dl));
        }
        let bb = &mut self.bodies[b];
        bb.x += gxb * (wb * dl);
        if ib > 0.0 {
            bb.q = rotate(bb.q, gpb * (ib * dl));
        }
    }
}
