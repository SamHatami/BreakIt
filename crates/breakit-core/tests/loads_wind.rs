//! Wind loads against hand calculations and the static solve.
mod common;
use breakit_core::loads::wind::{cylinder_drag, rect_drag, Profile, Wind, AIR_DENSITY};
use breakit_core::{
    solve_static, DynamicSim, LoadCase, LoadKind, LoadSet, Material, NodeLoad, Section, SimConfig,
    Structure, Support, Vec3, Window,
};
use common::*;

fn quiet() -> SimConfig {
    SimConfig {
        gravity: Vec3::zeros(),
        damping: 8.0,
        plasticity: false,
        ..Default::default()
    }
}

fn steady(wind: Wind) -> LoadSet {
    LoadSet::new(vec![LoadCase::new(
        "wind",
        Window::new(0.0, None, 0.0),
        LoadKind::Wind(wind),
    )])
}

/// Total wind force on a structure at rest.
fn total_force(s: &Structure, wind: Wind) -> (Vec3, LoadSet) {
    let mut loads = steady(wind);
    let mut sim = DynamicSim::new(s, quiet());
    loads.apply(s, &mut sim, 1.0 / 60.0);
    (loads.totals()[0], loads)
}

fn horizontal_member(section: &Section, length: f64, segments: usize) -> Structure {
    let mut s = Structure::new();
    let ids = s.add_line(
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(0.0, 1.0, length),
        segments,
        &Material::steel(),
        section,
    );
    s.add_support(Support::fixed(ids[0]));
    s
}

#[test]
fn drag_coefficient_tables() {
    assert_eq!(cylinder_drag(5e4), 1.2);
    assert!(cylinder_drag(4e5) < 0.55, "drag crisis");
    assert_eq!(rect_drag(1.0), 2.1);
    assert_eq!(rect_drag(2.0), 1.65);
    assert_eq!(rect_drag(10.0), 0.9);
    assert_eq!(rect_drag(0.65), 2.4);
}

#[test]
fn tube_drag_matches_hand_calculation() {
    // D = 0.1 m, U = 10 m/s: Re = 6.7e4, C_d = 1.2. q = ½ ρ C_d D U².
    let s = horizontal_member(&tube(), 2.0, 4);
    let (f, _) = total_force(&s, Wind::new(10.0, Vec3::x()));
    let expected = 0.5 * AIR_DENSITY * 1.2 * 0.1 * 100.0 * 2.0;
    assert!(rel_err(f.x, expected) < 1e-9, "{} vs {expected}", f.x);
    assert!(f.y.abs() < 1e-9 && f.z.abs() < 1e-9, "{f:?}");
}

#[test]
fn rectangle_drag_uses_depth_ratio_and_face() {
    // b = 0.1 along local z (world -x for a member along +z), h = 0.2 up.
    // Wind along x meets the 0.2 m tall face; depth along the flow 0.1, so
    // d/b = 0.5.
    let sec = Section::rect(0.1, 0.2);
    let s = horizontal_member(&sec, 3.0, 3);
    let (f, _) = total_force(&s, Wind::new(15.0, Vec3::x()));
    let expected = 0.5 * AIR_DENSITY * rect_drag(0.5) * 0.2 * 225.0 * 3.0;
    assert!(rel_err(f.x, expected) < 1e-9, "{} vs {expected}", f.x);
    // Wind from above would meet the 0.1 m face with d/b = 2.
    assert!((rect_drag(0.5) - 2.35).abs() < 0.01);
}

#[test]
fn power_law_profile_integrates_over_a_mast() {
    // Vertical mast of height H: total = ½ ρ C_d D U² H / (1 + 2α).
    let (h, alpha, u) = (4.0, 0.16, 12.0);
    let mut s = Structure::new();
    let ids = s.add_line(
        Vec3::zeros(),
        Vec3::new(0.0, h, 0.0),
        16,
        &Material::steel(),
        &tube(),
    );
    s.add_support(Support::fixed(ids[0]));
    let mut w = Wind::new(u, Vec3::z());
    w.profile = Profile::PowerLaw {
        ref_height: h,
        alpha,
    };
    let (f, _) = total_force(&s, w);
    let expected = 0.5 * AIR_DENSITY * 1.2 * 0.1 * u * u * h / (1.0 + 2.0 * alpha);
    assert!(rel_err(f.z, expected) < 0.01, "{} vs {expected}", f.z);
}

#[test]
fn leeward_member_is_shielded() {
    // Two parallel members 4 D apart in the wind, and one off to the side.
    let mut s = Structure::new();
    let (m, t) = (Material::steel(), tube());
    let mut line = |x: f64, y: f64| {
        let ids = s.add_line(Vec3::new(x, y, 0.0), Vec3::new(x, y, 2.0), 4, &m, &t);
        s.add_support(Support::fixed(ids[0]));
    };
    line(0.0, 1.0);
    line(0.4, 1.0);
    line(0.4, 2.0);
    let per_line = |shield: bool| {
        let mut w = Wind::new(10.0, Vec3::x());
        w.shielding = shield;
        let (_, loads) = total_force(&s, w);
        let f = loads.forces();
        let sum = |r: std::ops::Range<usize>| r.map(|n| f[n].x).sum::<f64>();
        (sum(0..5), sum(5..10), sum(10..15))
    };
    let (front, back, side) = per_line(true);
    let (front0, back0, _) = per_line(false);
    assert!(rel_err(front, front0) < 1e-9);
    assert!(rel_err(back0, front0) < 1e-9);
    assert!(
        rel_err(side, front0) < 1e-9,
        "member beside is not shielded"
    );
    let psi = back / front;
    assert!(psi > 0.1 && psi < 0.7, "shielding factor {psi}");
}

#[test]
fn jet_profile_is_a_soft_top_hat() {
    let mut w = Wind::new(10.0, Vec3::x());
    w.profile = Profile::Jet {
        center: Vec3::new(0.0, 1.0, 0.0),
        half_width: 1.0,
        half_height: 0.5,
        edge: 0.2,
    };
    assert_eq!(w.profile_factor(Vec3::new(3.0, 1.0, 0.5)), 1.0);
    assert_eq!(w.profile_factor(Vec3::new(0.0, 1.0, 1.2)), 0.0);
    let edge = w.profile_factor(Vec3::new(0.0, 1.0, 1.0));
    assert!((edge - 0.5).abs() < 1e-9, "{edge}");
    assert_eq!(w.profile_factor(Vec3::new(0.0, 1.7, 0.0)), 0.0);
}

#[test]
fn settled_cantilever_under_wind_matches_static_solve() {
    let s = horizontal_member(&tube(), 2.5, 10);
    let tip = s.nodes.len() - 1;
    let mut w = Wind::new(30.0, Vec3::x());
    w.vortex_shedding = false;
    let mut loads = LoadSet::new(vec![LoadCase::new(
        "gale",
        Window::new(0.0, None, 1.0),
        LoadKind::Wind(w),
    )]);
    let mut sim = DynamicSim::new(&s, quiet());
    for _ in 0..(6 * 60) {
        loads.step(&s, &mut sim, 1.0 / 60.0);
    }
    let nodal: Vec<NodeLoad> = loads
        .forces()
        .iter()
        .enumerate()
        .map(|(n, f)| NodeLoad::force(n, *f))
        .collect();
    let stat = solve_static(&s, &nodal).unwrap();
    let (d, e) = (sim.displacement(tip).x, stat.displacements[tip].x);
    assert!(e > 1e-4, "load too small to test: {e}");
    assert!(rel_err(d, e) < 0.02, "dynamic {d:e} vs static {e:e}");
}

#[test]
fn sign_panel_loads_its_frame_through_the_load_set() {
    use breakit_core::loads::wind::panels::PanelSpec;
    // 6 x 3 m sign facing a 40 m/s wind on a frame of four posts.
    let mut s = Structure::new();
    let corners = [
        Vec3::new(0.0, 5.0, -3.0),
        Vec3::new(0.0, 5.0, 3.0),
        Vec3::new(0.0, 8.0, 3.0),
        Vec3::new(0.0, 8.0, -3.0),
    ];
    let (m, t) = (Material::steel(), tube());
    let ids: Vec<usize> = corners.iter().map(|c| s.add_node(*c)).collect();
    for k in 0..4 {
        s.add_member(ids[k], ids[(k + 1) % 4], m.clone(), t.clone());
    }
    for &id in &ids[..2] {
        s.add_support(Support::fixed(id));
    }
    let mut w = Wind::new(40.0, Vec3::x());
    w.vortex_shedding = false;
    w.panels
        .push(PanelSpec::sign("billboard", corners.to_vec()));
    let mut with = steady(w.clone());
    w.panels.clear();
    let mut without = steady(w);
    let mut sim = DynamicSim::new(&s, quiet());
    with.apply(&s, &mut sim, 1.0 / 60.0);
    without.apply(&s, &mut sim, 1.0 / 60.0);
    let panel = with.totals()[0] - without.totals()[0];
    let expected = 0.5 * AIR_DENSITY * 1600.0 * 1.8 * 18.0;
    assert!(
        rel_err(panel.x, expected) < 1e-6,
        "{} vs {expected}",
        panel.x
    );
    let rig = with.rig_state(sim.time());
    assert_eq!(rig.wind[0].panels.len(), 1);
    assert!(rel_err(rig.wind[0].panels[0].net_pressure.abs(), expected / 18.0) < 1e-6);
}
