//! Member end forces and support reactions from a running [`DynamicSim`].
//!
//! The solver stores, per member, the axial force `N` (tension positive) and
//! the end moments `m[axis][end]` conjugate to the end rotations
//! `θ = (e × t)·u` about the member's current axes `u = y, z`. Differentiating
//! the member energy with respect to the end positions gives the force the
//! member puts on its nodes:
//!
//! ```text
//! F_a = N e − Σ_axis (m_a + m_b) (u × e) / L,     F_b = −F_a
//! ```
//!
//! (a turn of the chord by φ about `u` lowers both end rotations by φ). A
//! support's reaction balances the members meeting it and the load applied
//! there. Torsion puts no force on the nodes.

use breakit_core::loads::{Geometry, View};
use breakit_core::{DynamicSim, Structure, Vec3};

/// Force member `m` puts on its end nodes (a, b), N, world frame. Zero for a
/// broken member.
pub fn member_end_forces(
    s: &Structure,
    sim: &DynamicSim,
    geometry: &Geometry,
    m: usize,
) -> (Vec3, Vec3) {
    let view = View {
        structure: s,
        sim,
        geometry,
        time: sim.time(),
    };
    let Some(mn) = view.member(m) else {
        return (Vec3::zeros(), Vec3::zeros());
    };
    let d = sim.damage(m);
    let mut fa = mn.x * d.axial;
    for (axis, u) in [mn.y, mn.z].into_iter().enumerate() {
        let sum = d.moment[axis][0] + d.moment[axis][1];
        fa -= u.cross(&mn.x) * (sum / mn.length);
    }
    (fa, -fa)
}

/// Reaction at every supported node, N: what the support pushes on the
/// structure with. `applied` is the external force on each node (may be
/// shorter than the node list).
pub fn reactions(
    s: &Structure,
    sim: &DynamicSim,
    geometry: &Geometry,
    applied: &[Vec3],
) -> Vec<(usize, Vec3)> {
    let mut on_node = vec![Vec3::zeros(); s.nodes.len()];
    let supported: Vec<usize> = s.supports.iter().map(|sp| sp.node).collect();
    for (i, mem) in s.members.iter().enumerate() {
        if !(supported.contains(&mem.a) || supported.contains(&mem.b)) {
            continue;
        }
        let (fa, fb) = member_end_forces(s, sim, geometry, i);
        on_node[mem.a] += fa;
        on_node[mem.b] += fb;
    }
    let mut out: Vec<(usize, Vec3)> = Vec::new();
    for n in supported {
        if out.iter().any(|(k, _)| *k == n) {
            continue;
        }
        let ext = applied.get(n).copied().unwrap_or_else(Vec3::zeros);
        out.push((n, -(on_node[n] + ext)));
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use breakit_core::{Material, Section, SimConfig, Support};

    fn steel_tube() -> (Material, Section) {
        (Material::steel(), Section::tube(0.0571, 0.005))
    }

    fn settle(s: &Structure, loads: &[(usize, Vec3)]) -> DynamicSim {
        let mut sim = DynamicSim::new(s, SimConfig::default());
        for &(n, f) in loads {
            sim.set_force(n, f);
        }
        sim.settle(20.0, 1e-4);
        sim
    }

    #[test]
    fn cantilever_reaction_carries_the_tip_load() {
        let (mat, sec) = steel_tube();
        let mut s = Structure::new();
        let a = s.add_node(Vec3::zeros());
        let b = s.add_node(Vec3::new(2.0, 0.0, 0.0));
        s.add_member(a, b, mat, sec);
        s.add_support(Support::fixed(a));
        let p = 5_000.0;
        let sim = settle(&s, &[(b, Vec3::new(0.0, -p, 0.0))]);
        let g = Geometry::new(&s);
        let r = reactions(&s, &sim, &g, &[]);
        assert_eq!(r.len(), 1);
        let tip_weight = sim.node_mass(b) * 9.81;
        let want = p + tip_weight;
        let got = r[0].1;
        assert!(
            (got.y - want).abs() < 0.03 * want,
            "reaction {got:?}, want {want} up"
        );
        assert!(got.x.abs() < 0.03 * want && got.z.abs() < 0.03 * want);
    }

    #[test]
    fn simple_span_splits_a_midspan_load() {
        let (mat, sec) = steel_tube();
        let mut s = Structure::new();
        let n: Vec<usize> = (0..5)
            .map(|i| s.add_node(Vec3::new(i as f64, 0.0, 0.0)))
            .collect();
        for w in n.windows(2) {
            s.add_member(w[0], w[1], mat.clone(), sec.clone());
        }
        s.add_support(Support::pinned(n[0]));
        s.add_support(Support::pinned(n[4]));
        // Off-centre load: 3/4 of it to the near support.
        let p = 4_000.0;
        let sim = settle(&s, &[(n[1], Vec3::new(0.0, -p, 0.0))]);
        let g = Geometry::new(&s);
        let r = reactions(&s, &sim, &g, &[]);
        let weight: f64 = (0..5).map(|i| sim.node_mass(i)).sum::<f64>() * 9.81;
        let total: f64 = r.iter().map(|(_, f)| f.y).sum();
        assert!(
            (total - (p + weight)).abs() < 0.03 * (p + weight),
            "total {total}, want {}",
            p + weight
        );
        let near = r.iter().find(|(k, _)| *k == n[0]).unwrap().1.y;
        let far = r.iter().find(|(k, _)| *k == n[4]).unwrap().1.y;
        assert!(near > far, "near {near} should carry more than far {far}");
        let want_near = 0.75 * p + 0.5 * weight;
        assert!(
            (near - want_near).abs() < 0.05 * want_near,
            "near {near}, want {want_near}"
        );
    }
}
