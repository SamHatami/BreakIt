//! Real-time solver checked against the static solve and textbook behaviour.
mod common;
use breakit_core::{
    solve_static, BreakCause, DynamicSim, Material, NodeLoad, Section, SimConfig, Structure,
    Support, Vec3,
};
use common::*;
use std::f64::consts::PI;

/// Apply `force` at `node`, ramped linearly over `ramp` seconds (a suddenly
/// applied load would overshoot to about twice its static effect), then hold
/// until `total` seconds have passed.
fn ramp_load(sim: &mut DynamicSim, node: usize, force: Vec3, ramp: f64, total: f64) {
    let frames = (total * 60.0) as usize;
    for f in 0..frames {
        let t = f as f64 / 60.0;
        sim.set_force(node, force * (t / ramp).min(1.0));
        sim.step(1.0 / 60.0);
    }
}

fn quiet(plasticity: bool) -> SimConfig {
    SimConfig {
        gravity: Vec3::zeros(),
        damping: 8.0,
        plasticity,
        ..Default::default()
    }
}

#[test]
fn settled_cantilever_matches_static_solve() {
    let (s, ids) = cantilever(2.0, 10, Material::steel());
    let tip = *ids.last().unwrap();
    let f = Vec3::new(0.0, -1000.0, 0.0);
    let stat = solve_static(&s, &[NodeLoad::force(tip, f)]).unwrap();
    let mut sim = DynamicSim::new(&s, quiet(false));
    sim.set_force(tip, f);
    assert!(sim.settle(10.0, 1e-6), "did not settle");
    let (d, e) = (sim.displacement(tip).y, stat.displacements[tip].y);
    assert!(rel_err(d, e) < 0.01, "xpbd {d:e} vs static {e:e}");
}

#[test]
fn self_weight_matches_static_solve() {
    let (s, ids) = cantilever(3.0, 12, Material::steel());
    let tip = *ids.last().unwrap();
    let stat = solve_static(&s, &s.self_weight_loads(9.81)).unwrap();
    let mut sim = DynamicSim::new(
        &s,
        SimConfig {
            damping: 8.0,
            plasticity: false,
            ..Default::default()
        },
    );
    assert!(sim.settle(10.0, 1e-6));
    let (d, e) = (sim.displacement(tip).y, stat.displacements[tip].y);
    assert!(rel_err(d, e) < 0.01, "xpbd {d:e} vs static {e:e}");
}

#[test]
fn sideways_load_on_portal_frame_matches_static_solve() {
    // Two columns and a beam: exercises vertical members, joints and both bending planes.
    let mat = Material::steel();
    let sec = Section::rect(0.08, 0.12);
    let mut s = Structure::new();
    let left = s.add_line(Vec3::zeros(), Vec3::new(0.0, 3.0, 0.0), 4, &mat, &sec);
    let right = s.add_line(
        Vec3::new(4.0, 0.0, 0.0),
        Vec3::new(4.0, 3.0, 0.0),
        4,
        &mat,
        &sec,
    );
    let mut prev = left[4];
    for i in 1..4 {
        let n = s.add_node(Vec3::new(i as f64, 3.0, 0.0));
        s.add_member(prev, n, mat.clone(), sec.clone());
        prev = n;
    }
    s.add_member(prev, right[4], mat.clone(), sec.clone());
    s.add_support(Support::fixed(left[0]));
    s.add_support(Support::fixed(right[0]));
    let loads = [
        NodeLoad::force(left[4], Vec3::new(5000.0, 0.0, 2000.0)),
        NodeLoad::force(right[4], Vec3::new(0.0, -2_000.0, 0.0)),
    ];
    let stat = solve_static(&s, &loads).unwrap();
    let mut sim = DynamicSim::new(&s, quiet(false));
    for l in &loads {
        sim.set_force(l.node, l.force);
    }
    assert!(sim.settle(10.0, 1e-6));
    for n in [left[4], right[4], left[2]] {
        let (d, e) = (sim.displacement(n), stat.displacements[n]);
        assert!(
            (d - e).norm() < 0.01 * e.norm(),
            "node {n}: xpbd {d:?} vs static {e:?}"
        );
    }
}

#[test]
fn column_buckles_above_euler_load_only() {
    let mat = Material::steel().elastic_only();
    let sec = tube();
    let l = 3.0;
    let p_cr = PI * PI * 210e9 * sec.iz / (4.0 * l * l); // fixed base, free top
    let sway = |ratio: f64| {
        let mut s = Structure::new();
        let ids = s.add_line(Vec3::zeros(), Vec3::new(0.0, l, 0.0), 12, &mat, &sec);
        s.add_support(Support::fixed(ids[0]));
        let mut sim = DynamicSim::new(
            &s,
            SimConfig {
                damping: 2.0,
                ..quiet(false)
            },
        );
        // Axial load plus a 0.1% lateral nudge as the imperfection.
        ramp_load(
            &mut sim,
            ids[12],
            Vec3::new(0.001 * p_cr, -ratio * p_cr, 0.0),
            2.0,
            5.0,
        );
        sim.displacement(ids[12]).x
    };
    let below = sway(0.8);
    let above = sway(1.2);
    // Below P_cr the nudge is amplified about 1/(1 - 0.8) = 5 times: millimetres.
    assert!(below.abs() < 0.02, "below {below}");
    // Above P_cr the column folds over: tens of centimetres.
    assert!(above.abs() > 0.3, "above {above}");
}

#[test]
fn plastic_hinge_forms_at_collapse_load_and_then_snaps() {
    let l = 2.0;
    let mp = 355e6 * tube().zz;
    let p_collapse = mp / l;
    let run = |ratio: f64, seconds: f64| {
        let (s, ids) = cantilever(l, 8, Material::steel());
        let mut sim = DynamicSim::new(&s, quiet(true));
        ramp_load(
            &mut sim,
            ids[8],
            Vec3::new(0.0, -ratio * p_collapse, 0.0),
            1.0,
            seconds,
        );
        sim
    };
    let safe = run(0.8, 2.0);
    assert_eq!(
        safe.hinge_rotation(0),
        0.0,
        "no yield at 80% of collapse load"
    );
    assert!(safe.break_events().is_empty());

    let over = run(1.3, 4.0);
    let ev = over.break_events();
    assert!(!ev.is_empty(), "130% load should snap the root");
    assert_eq!(ev[0].member, 0, "the root member fails first");
    assert_eq!(ev[0].cause, BreakCause::Hinge);
}

#[test]
fn steel_tie_ruptures_after_yielding() {
    let mut s = Structure::new();
    let ids = s.add_line(
        Vec3::zeros(),
        Vec3::new(0.0, -2.0, 0.0),
        2,
        &Material::steel(),
        &tube(),
    );
    s.add_support(Support::fixed(ids[0]));
    let ny = 355e6 * tube().area;
    let mut sim = DynamicSim::new(&s, quiet(true));
    sim.set_force(ids[2], Vec3::new(0.0, -1.1 * ny, 0.0));
    for _ in 0..240 {
        sim.step(1.0 / 60.0);
    }
    let ev = sim.break_events();
    assert!(!ev.is_empty());
    assert_eq!(ev[0].cause, BreakCause::Rupture);
}

#[test]
fn timber_snaps_without_yielding() {
    let mut s = Structure::new();
    let sec = Section::rect(0.05, 0.1);
    let ids = s.add_line(
        Vec3::zeros(),
        Vec3::new(2.0, 0.0, 0.0),
        4,
        &Material::timber(),
        &sec,
    );
    s.add_support(Support::fixed(ids[0]));
    let p = 1.2 * 24e6 * sec.zz / 2.0;
    let mut sim = DynamicSim::new(&s, quiet(true));
    sim.set_force(ids[4], Vec3::new(0.0, -p, 0.0));
    for _ in 0..120 {
        sim.step(1.0 / 60.0);
    }
    let ev = sim.break_events();
    assert!(!ev.is_empty());
    assert_eq!(ev[0].cause, BreakCause::Brittle);
    assert_eq!(sim.hinge_rotation(ev[0].member), 0.0);
}

#[test]
fn broken_pieces_fall_freely() {
    let (s, ids) = cantilever(2.0, 4, Material::timber());
    let mut sim = DynamicSim::new(
        &s,
        SimConfig {
            damping: 0.0,
            ..Default::default()
        },
    );
    sim.set_force(ids[4], Vec3::new(0.0, -1e6, 0.0));
    for _ in 0..60 {
        sim.step(1.0 / 60.0);
    }
    assert!(!sim.break_events().is_empty());
    assert!(
        sim.displacement(ids[4]).y < -1.0,
        "tip should have fallen away"
    );
}

#[test]
fn axial_tension_lowers_the_plastic_moment() {
    // Short and stiff, so the tension barely straightens the beam (P-delta).
    let l = 0.5;
    let (ny, mp) = (355e6 * tube().area, 355e6 * tube().zz);
    let hinge = |axial: f64| {
        let (s, ids) = cantilever(l, 8, Material::steel());
        let mut sim = DynamicSim::new(&s, quiet(true));
        // 80% of Mp at the root from the vertical load, plus axial tension.
        ramp_load(
            &mut sim,
            ids[8],
            Vec3::new(axial * ny, -0.8 * mp / l, 0.0),
            1.0,
            2.0,
        );
        sim.hinge_rotation(0)
    };
    assert_eq!(hinge(0.0), 0.0, "bending alone stays elastic");
    // (0.6)² + 0.8 > 1, so the same moment yields once 60% of Ny is added.
    assert!(hinge(0.6) > 0.0, "combined load should form a hinge");
}

#[test]
fn short_stiff_members_survive_a_sudden_heavy_load() {
    // 62 mm steel elements with a 25.6 kN load dropped on in one frame: a
    // light, very stiff tip node. Regression test for the solver throwing the
    // tip a whole element length in one substep.
    let mut s = Structure::new();
    let mat = Material::steel().elastic_only();
    let ids = s.add_line(Vec3::zeros(), Vec3::new(0.5, 0.0, 0.0), 8, &mat, &tube());
    s.add_support(Support::fixed(ids[0]));
    let f = Vec3::new(0.0, -25_600.0, 0.0);
    let stat = solve_static(&s, &[NodeLoad::force(ids[8], f)]).unwrap();
    let mut sim = DynamicSim::new(&s, quiet(false));
    sim.set_force(ids[8], f);
    assert!(sim.settle(5.0, 1e-6));
    let (d, e) = (sim.displacement(ids[8]).y, stat.displacements[ids[8]].y);
    assert!(rel_err(d, e) < 0.01, "xpbd {d:e} vs static {e:e}");
}
