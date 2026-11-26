//! Wind loads against hand calculations and the static solve.
mod common;
use breakit_core::loads::wind::{cylinder_drag, rect_drag, Gusts, Profile, Wind, AIR_DENSITY};
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
fn gusts_have_the_target_intensity_and_convect_downwind() {
    let mut s = Structure::new();
    s.add_line(
        Vec3::zeros(),
        Vec3::new(10.0, 0.0, 0.0),
        1,
        &Material::steel(),
        &tube(),
    );
    let mut w = Wind::new(20.0, Vec3::x());
    w.gusts = Some(Gusts {
        intensity: 0.15,
        length_scale: 30.0,
        seed: 3,
    });
    w.prepare(&s);
    let p = Vec3::new(0.0, 1.0, 0.0);
    let n = 200_000;
    let dt = 0.01;
    let (mut sum, mut sum2) = (0.0, 0.0);
    for i in 0..n {
        let u = w.velocity(p, i as f64 * dt, 1.0).x;
        sum += u;
        sum2 += u * u;
    }
    let mean = sum / n as f64;
    let std = (sum2 / n as f64 - mean * mean).sqrt();
    assert!(rel_err(mean, 20.0) < 0.02, "mean {mean}");
    assert!(rel_err(std, 0.15 * 20.0) < 0.1, "std {std}");
    // Frozen turbulence: 5 m downwind sees the same gust 5/U s later.
    let q = p + Vec3::new(5.0, 0.0, 0.0);
    for t in [1.0, 7.3, 22.0] {
        let a = w.velocity(p, t, 1.0).x;
        let b = w.velocity(q, t + 5.0 / 20.0, 1.0).x;
        assert!((a - b).abs() < 1e-9, "{a} vs {b}");
    }
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
fn vortex_shedding_drives_a_cross_wind_force_at_the_strouhal_frequency() {
    // Large slow tube: f = St U / D = 0.2 * 2 / 0.4 = 1 Hz, below the fade.
    let sec = Section::tube(0.2, 0.01);
    let s = horizontal_member(&sec, 2.0, 2);
    let mut loads = steady(Wind::new(2.0, Vec3::x()));
    let mut sim = DynamicSim::new(
        &s,
        SimConfig {
            gravity: Vec3::zeros(),
            ..quiet()
        },
    );
    let mut ys = Vec::new();
    for _ in 0..(4 * 60) {
        loads.apply(&s, &mut sim, 1.0 / 60.0);
        ys.push(loads.totals()[0].y);
        sim.step(1.0 / 60.0);
    }
    let amp = ys.iter().fold(0.0f64, |m, y| m.max(y.abs()));
    let expected = 0.5 * AIR_DENSITY * 0.25 * 0.4 * 4.0 * 2.0;
    assert!(
        rel_err(amp, expected) < 0.1,
        "lift amplitude {amp} vs {expected}"
    );
    // Count zero crossings: 2 per period, 1 Hz, 4 s.
    let crossings = ys
        .windows(2)
        .filter(|w| w[0].signum() != w[1].signum())
        .count();
    assert!((6..=10).contains(&crossings), "{crossings} crossings");
}
