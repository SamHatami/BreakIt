//! Static FEM against textbook beam formulas.
mod common;
use breakit_core::{solve_static, Material, NodeLoad, Section, Structure, Support, Vec3};
use common::*;

#[test]
fn cantilever_tip_load_matches_pl3_over_3ei() {
    let (s, ids) = cantilever(2.0, 10, Material::steel());
    let p = 1000.0;
    let r = solve_static(
        &s,
        &[NodeLoad::force(
            *ids.last().unwrap(),
            Vec3::new(0.0, -p, 0.0),
        )],
    )
    .unwrap();
    let ei = 210e9 * tube().iz;
    let expected = p * 8.0 / (3.0 * ei);
    let got = -r.displacements[*ids.last().unwrap()].y;
    assert!(
        rel_err(got, expected) < 1e-9,
        "got {got}, expected {expected}"
    );
    // Root moment = P L, tip moment = 0.
    assert!(rel_err(r.member_forces[0].mz[0].abs(), p * 2.0) < 1e-9);
    assert!(r.member_forces[9].mz[1].abs() < 1e-6);
}

#[test]
fn cantilever_sideways_load_uses_iy() {
    let mut s = Structure::new();
    let sec = Section::rect(0.05, 0.2); // stiff vertically, weak sideways
    let ids = s.add_line(
        Vec3::zeros(),
        Vec3::new(2.0, 0.0, 0.0),
        4,
        &Material::steel(),
        &sec,
    );
    s.add_support(Support::fixed(ids[0]));
    let p = 500.0;
    let tip = *ids.last().unwrap();
    let r = solve_static(&s, &[NodeLoad::force(tip, Vec3::new(0.0, 0.0, p))]).unwrap();
    let expected = p * 8.0 / (3.0 * 210e9 * sec.iy);
    assert!(rel_err(r.displacements[tip].z, expected) < 1e-9);
}

#[test]
fn simply_supported_midspan_load_matches_pl3_over_48ei() {
    let mut s = Structure::new();
    let ids = s.add_line(
        Vec3::zeros(),
        Vec3::new(4.0, 0.0, 0.0),
        8,
        &Material::steel(),
        &tube(),
    );
    // Pin one end; the other end is a pin too, with torsion held at the first end
    // so the model has no free spin about its own axis.
    let mut left = Support::pinned(ids[0]);
    left.fixed[3] = true;
    s.add_support(left);
    s.add_support(Support::pinned(ids[8]));
    let p = 2000.0;
    let r = solve_static(&s, &[NodeLoad::force(ids[4], Vec3::new(0.0, -p, 0.0))]).unwrap();
    let expected = p * 64.0 / (48.0 * 210e9 * tube().iz);
    assert!(rel_err(-r.displacements[ids[4]].y, expected) < 1e-9);
}

#[test]
fn axial_bar_matches_pl_over_ea() {
    let mut s = Structure::new();
    let ids = s.add_line(
        Vec3::zeros(),
        Vec3::new(0.0, 3.0, 0.0),
        3,
        &Material::steel(),
        &tube(),
    );
    s.add_support(Support::fixed(ids[0]));
    let p = 50_000.0;
    let r = solve_static(&s, &[NodeLoad::force(ids[3], Vec3::new(0.0, p, 0.0))]).unwrap();
    let expected = p * 3.0 / (210e9 * tube().area);
    assert!(rel_err(r.displacements[ids[3]].y, expected) < 1e-9);
    assert!(
        rel_err(r.member_forces[0].axial, p) < 1e-9,
        "tension is positive"
    );
}

#[test]
fn torsion_matches_tl_over_gj() {
    let (s, ids) = cantilever(2.0, 2, Material::steel());
    let t = 300.0;
    let tip = ids[2];
    let r = solve_static(
        &s,
        &[NodeLoad {
            node: tip,
            force: Vec3::zeros(),
            moment: Vec3::new(t, 0.0, 0.0),
        }],
    )
    .unwrap();
    let expected = t * 2.0 / (81e9 * tube().j);
    assert!(rel_err(r.rotations[tip].x, expected) < 1e-9);
}

#[test]
fn unsupported_structure_is_rejected() {
    let mut s = Structure::new();
    s.add_line(
        Vec3::zeros(),
        Vec3::new(1.0, 0.0, 0.0),
        2,
        &Material::steel(),
        &tube(),
    );
    assert!(solve_static(&s, &[]).is_err());
}

#[test]
fn utilization_reaches_one_at_plastic_moment() {
    let (s, ids) = cantilever(2.0, 4, Material::steel());
    let mp = 355e6 * tube().zz;
    let r = solve_static(
        &s,
        &[NodeLoad::force(ids[4], Vec3::new(0.0, -mp / 2.0, 0.0))],
    )
    .unwrap();
    assert!(rel_err(r.member_forces[0].utilization, 1.0) < 1e-9);
}

#[test]
fn a_raft_on_springs_sinks_by_load_over_stiffness() {
    // A square frame held only by springs at its corners (a float on still
    // water): the same load on every corner sinks each by P / k.
    let mut s = Structure::new();
    let ids: Vec<usize> = [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0)]
        .iter()
        .map(|&(x, z)| s.add_node(Vec3::new(x, 0.0, z)))
        .collect();
    for i in 0..4 {
        s.add_member(ids[i], ids[(i + 1) % 4], Material::steel(), tube());
    }
    s.add_member(ids[0], ids[2], Material::steel(), tube());
    let k = 1.0e5;
    let springs: Vec<_> = ids
        .iter()
        .map(|&n| breakit_core::NodeSpring {
            node: n,
            stiffness: Vec3::new(k, k, k),
        })
        .collect();
    let p = 1000.0;
    let loads: Vec<_> = ids
        .iter()
        .map(|&n| NodeLoad::force(n, Vec3::new(0.0, -p, 0.0)))
        .collect();
    assert!(solve_static(&s, &loads).is_err());
    let r = breakit_core::solve_static_with_springs(&s, &loads, &springs).unwrap();
    for &n in &ids {
        assert!(rel_err(-r.displacements[n].y, p / k) < 1e-9);
    }
}
