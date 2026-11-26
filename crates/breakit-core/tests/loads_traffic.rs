//! Vehicle loads and impactors against hand calculations and the static solve.
mod common;
use breakit_core::loads::impact::Impactor;
use breakit_core::loads::traffic::{Axle, Drive, LaneLine, Spring, Traffic, Vehicle};
use breakit_core::loads::G;
use breakit_core::{
    solve_static, DynamicSim, LoadCase, LoadKind, LoadSet, Material, NodeLoad, Section, SimConfig,
    Structure, Support, Vec3, Window,
};
use common::*;
use std::f64::consts::PI;

const DT: f64 = 1.0 / 60.0;

fn quiet() -> SimConfig {
    SimConfig {
        gravity: Vec3::zeros(),
        damping: 8.0,
        plasticity: false,
        ..Default::default()
    }
}

/// 10 m simply supported steel beam of ten 1 m members along +X at y = 0.
/// Pinned both ends; one end also holds the torsion so the static solve
/// has no mechanism.
fn beam() -> (Structure, Vec<usize>) {
    let mut s = Structure::new();
    let ids = s.add_line(
        Vec3::zeros(),
        Vec3::new(10.0, 0.0, 0.0),
        10,
        &Material::steel(),
        &Section::rect(0.2, 0.4),
    );
    s.add_support(Support {
        node: ids[0],
        fixed: [true, true, true, true, false, false],
    });
    s.add_support(Support::pinned(ids[10]));
    (s, ids)
}

/// Lane along the beam with 5 m approach and exit ramps: arc 5 is node 0.
fn lane() -> LaneLine {
    LaneLine::new(Vec3::new(-5.0, 0.0, 0.0), Vec3::new(15.0, 0.0, 0.0), 0.3)
}

fn traffic_set(vehicles: Vec<Vehicle>, ramp: f64) -> LoadSet {
    LoadSet::new(vec![LoadCase::new(
        "Traffic",
        Window::new(0.0, None, ramp),
        LoadKind::Traffic(Traffic::new(vec![lane()], vehicles)),
    )])
}

fn traffic(loads: &LoadSet) -> &Traffic {
    match &loads.cases[0].kind {
        LoadKind::Traffic(t) => t,
        _ => unreachable!(),
    }
}

#[test]
fn lane_picks_nodes_along_line_in_order() {
    let mat = Material::steel();
    let sec = tube();
    let mut s = Structure::new();
    // Deck nodes added out of order, one slightly off the line.
    let xs = [3.0, 0.0, 2.0, 1.0, 4.0];
    let ids: Vec<usize> = xs
        .iter()
        .map(|&x| s.add_node(Vec3::new(x, 0.0, if x == 2.0 { 0.2 } else { 0.0 })))
        .collect();
    let off = s.add_node(Vec3::new(2.5, 0.0, 0.8)); // beside the lane
    let beyond = s.add_node(Vec3::new(4.6, 0.0, 0.0)); // past the end of the line
    let twin = s.add_node(Vec3::new(3.0, 0.0, 0.0)); // coincident with node at x = 3
    let below = s.add_node(Vec3::new(1.5, -1.0, 0.0)); // truss chord below
    let by_x = |x: f64| ids[xs.iter().position(|&v| v == x).unwrap()];
    for x in [0.0, 1.0, 2.0, 3.0] {
        s.add_member(by_x(x), by_x(x + 1.0), mat.clone(), sec.clone());
    }
    s.add_member(by_x(2.0), off, mat.clone(), sec.clone());
    s.add_member(by_x(4.0), beyond, mat.clone(), sec.clone());
    s.add_member(twin, below, mat.clone(), sec.clone());
    let mut t = Traffic::new(
        vec![LaneLine::new(Vec3::zeros(), Vec3::new(4.5, 0.0, 0.0), 0.3)],
        vec![],
    );
    t.prepare(&s);
    let got = t.lane_nodes(0);
    let want: Vec<usize> = [0.0, 1.0, 2.0, 3.0, 4.0].iter().map(|&x| by_x(x)).collect();
    assert_eq!(got, want, "lane nodes");
    let lane = t.lane(0).unwrap();
    assert_eq!(lane.stations[3].others, vec![twin]);
    assert!(lane.segments.iter().all(|m| m.len() == 1));
    println!("lane nodes {got:?} (off-lane {off}, beyond {beyond}, below {below} left out)");
}

#[test]
fn parked_vehicle_splits_axles_by_lever_rule() {
    let (s, ids) = beam();
    // Front axle at arc 11.3 (x = 6.3), rear 3 m behind (x = 3.3).
    let stiff = |m: f64| Some(Spring::tuned(m, 50.0, 0.1));
    let truck = Vehicle::new(
        "Dump truck",
        0,
        vec![
            Axle {
                suspension: stiff(6000.0),
                ..Axle::new(0.0, 6000.0)
            },
            Axle {
                suspension: stiff(4000.0),
                ..Axle::new(3.0, 4000.0)
            },
        ],
        Drive::parked(11.3),
    );
    let mut loads = traffic_set(vec![truck], 0.0);
    let mut sim = DynamicSim::new(&s, quiet());
    loads.apply(&s, &mut sim, DT);
    let f = loads.forces();
    let total: Vec3 = f.iter().sum();
    let w = 10_000.0 * G;
    println!("total {:.1} N vs weight {w:.1} N", -total.y);
    assert!(rel_err(-total.y, w) < 1e-6);
    let expect = [
        (ids[6], 0.7 * 6000.0 * G),
        (ids[7], 0.3 * 6000.0 * G),
        (ids[3], 0.7 * 4000.0 * G),
        (ids[4], 0.3 * 4000.0 * G),
    ];
    for (n, e) in expect {
        println!("node {n}: {:.1} N vs {e:.1} N", -f[n].y);
        assert!(rel_err(-f[n].y, e) < 1e-6, "node {n}");
    }
    assert!(f.iter().all(|v| v.x == 0.0 && v.z == 0.0));
    let rig = &loads.rig_state(sim.time()).vehicles[0];
    assert!(rig.wheels.iter().all(|w| w.on_structure) && !rig.fallen);
}

fn midspan_static(s: &Structure, ids: &[usize], mass: f64) -> f64 {
    let stat = solve_static(
        s,
        &[NodeLoad::force(ids[5], Vec3::new(0.0, -mass * G, 0.0))],
    )
    .unwrap();
    stat.displacements[ids[5]].y
}

#[test]
fn parked_axle_settles_to_static_deflection() {
    let (s, ids) = beam();
    let mass = 10_000.0;
    let truck = Vehicle::new(
        "Coil hauler",
        0,
        vec![Axle::new(0.0, mass)],
        Drive::parked(10.0),
    );
    let mut loads = traffic_set(vec![truck], 1.0);
    let mut sim = DynamicSim::new(&s, quiet());
    for _ in 0..(8.0 / DT) as usize {
        loads.step(&s, &mut sim, DT);
    }
    let d = sim.displacement(ids[5]).y;
    let e = midspan_static(&s, &ids, mass);
    println!("midspan: dynamic {d:e} m, static {e:e} m");
    assert!(rel_err(d, e) < 0.015, "dynamic {d:e} vs static {e:e}");
}

#[test]
fn slow_crossing_peaks_at_static_deflection() {
    let (s, ids) = beam();
    let mass = 10_000.0;
    // Front axle starts 2 m before the deck at 1 m/s.
    let truck = Vehicle::new(
        "Dump truck",
        0,
        vec![Axle::new(0.0, mass)],
        Drive::cruise(1.0, 3.0),
    );
    let mut loads = traffic_set(vec![truck], 0.0);
    let mut sim = DynamicSim::new(&s, quiet());
    let mut peak: f64 = 0.0;
    let mut t_peak = 0.0;
    for _ in 0..(16.0 / DT) as usize {
        loads.step(&s, &mut sim, DT);
        let d = -sim.displacement(ids[5]).y;
        if d > peak {
            peak = d;
            t_peak = sim.time();
        }
    }
    let e = -midspan_static(&s, &ids, mass);
    println!(
        "peak midspan {peak:e} m at t = {t_peak:.2} s (axle at midspan at 7 s), static {e:e} m"
    );
    assert!(rel_err(peak, e) < 0.05);
    assert!((t_peak - 7.0).abs() < 0.5);
    // Off the far end: back to rest.
    assert!(sim.displacement(ids[5]).y.abs() < 0.01 * e);
    let rig = &loads.rig_state(sim.time()).vehicles[0];
    assert!(!rig.wheels[0].on_structure && !rig.fallen);
}

#[test]
fn braking_pushes_deck_forward_with_m_a() {
    let (s, _ids) = beam();
    let (m1, m2) = (7000.0, 5000.0);
    let a = 4.0;
    let factor = 1.5;
    // 3 m/s, braking at arc 13 (x = 8): stops 1.125 m later, both axles on deck.
    let truck = Vehicle::new(
        "Mobile crane",
        0,
        vec![Axle::new(0.0, m1), Axle::new(3.5, m2)],
        Drive::cruise(3.0, 7.0).braking(13.0, a),
    )
    .with_cg_height(1.5);
    let mut loads = traffic_set(vec![truck], 0.0);
    loads.cases[0].factor = factor;
    let mut sim = DynamicSim::new(&s, quiet());
    let mut checked = 0;
    let mut max_front: f64 = 0.0;
    while sim.time() < 3.5 {
        let t = sim.time();
        loads.step(&s, &mut sim, DT);
        let total = loads.totals()[0];
        // Frames entirely inside the braking phase (2 s .. 2.75 s).
        if t > 2.0 + 1e-9 && t + DT < 2.75 {
            let e = factor * (m1 + m2) * a;
            assert!(rel_err(total.x, e) < 1e-9, "t {t}: {} vs {e}", total.x);
            assert!(loads.rig_state(t).vehicles[0].braking);
            checked += 1;
            max_front = max_front.max(loads.rig_state(t).vehicles[0].wheels[0].force);
        }
        if t > 2.8 {
            assert!(total.x.abs() < 1e-9, "after stop {}", total.x);
        }
    }
    let front_static = factor * m1 * G;
    println!(
        "braking force {:.1} N = M a ({checked} frames); front axle dive {:.1} N vs static {front_static:.1} N",
        factor * (m1 + m2) * a,
        max_front
    );
    assert!(checked > 30);
    assert!(max_front > 1.05 * front_static, "no dive");
    let rig = &loads.rig_state(sim.time()).vehicles[0];
    assert!((rig.wheels[0].contact.x - (8.0 + 9.0 / 8.0)).abs() < 1e-3);
    assert_eq!(rig.speed, 0.0);
}

#[test]
fn vehicle_falls_through_gap() {
    // Two cantilevers with a missing 1 m member between x = 5 and x = 6.
    let mat = Material::steel();
    let sec = Section::rect(0.2, 0.4);
    let mut s = Structure::new();
    let left = s.add_line(Vec3::zeros(), Vec3::new(5.0, 0.0, 0.0), 5, &mat, &sec);
    let right = s.add_line(
        Vec3::new(6.0, 0.0, 0.0),
        Vec3::new(11.0, 0.0, 0.0),
        5,
        &mat,
        &sec,
    );
    s.add_support(Support::fixed(left[0]));
    s.add_support(Support::fixed(right[5]));
    let lane = LaneLine::new(Vec3::new(-5.0, 0.0, 0.0), Vec3::new(16.0, 0.0, 0.0), 0.3);
    let truck = Vehicle::new(
        "Dump truck",
        0,
        vec![Axle::new(0.0, 2000.0)],
        Drive::cruise(2.0, 0.0),
    );
    let mut loads = LoadSet::new(vec![LoadCase::new(
        "Traffic",
        Window::new(0.0, None, 0.0),
        LoadKind::Traffic(Traffic::new(vec![lane], vec![truck])),
    )]);
    let mut sim = DynamicSim::new(&s, quiet());
    let mut fell = None;
    while sim.time() < 8.0 {
        loads.step(&s, &mut sim, DT);
        if fell.is_some() {
            assert_eq!(loads.totals()[0].norm(), 0.0);
        }
        let r = &loads.rig_state(sim.time()).vehicles[0];
        if r.fallen && fell.is_none() {
            fell = Some(r.wheels[0].contact.x);
        }
    }
    let x = fell.expect("did not fall");
    println!("fell at x = {x:.3} m");
    assert!((5.0..5.1).contains(&x));
    assert_eq!(traffic(&loads).lane_nodes(0).len(), 12);
}

fn impact_set(i: Impactor) -> LoadSet {
    LoadSet::new(vec![LoadCase::new(
        "Impact",
        Window::new(0.0, None, 0.0),
        LoadKind::Impact(i),
    )])
}

fn impactor(loads: &LoadSet) -> &Impactor {
    match &loads.cases[0].kind {
        LoadKind::Impact(i) => i,
        _ => unreachable!(),
    }
}

#[test]
fn impact_on_rigid_node_matches_spring_mass_contact() {
    let (s, ids) = cantilever(2.0, 10, Material::steel());
    let (m, v0, r) = (1000.0, 2.0, 0.1);
    let cart = Impactor::cart(
        "Runaway cart",
        m,
        r,
        Vec3::new(-0.3, 0.0, 0.0),
        Vec3::new(v0, 0.0, 0.0),
    )
    .undamped();
    let k = cart.pad_stiffness;
    let mut loads = impact_set(cart);
    let mut sim = DynamicSim::new(&s, quiet());
    while sim.time() < 0.5 {
        loads.step(&s, &mut sim, DT);
    }
    let rig = loads.rig_state(sim.time()).impactors[0].clone();
    assert_eq!(rig.events.len(), 1, "{:?}", rig.events);
    let ev = &rig.events[0];
    assert_eq!(ev.node, ids[0]);
    let peak = v0 * (k * m).sqrt();
    let dur = PI * (m / k).sqrt();
    println!(
        "peak {:.0} N vs {peak:.0} N, duration {:.4} s vs {dur:.4} s, rebound {:.4} m/s vs {v0}, impact speed {:.3}",
        ev.peak_force, ev.duration, -rig.velocity.x, ev.impact_speed
    );
    assert!(rel_err(ev.peak_force, peak) < 0.02);
    assert!(rel_err(ev.duration, dur) < 0.03);
    assert!(rel_err(-rig.velocity.x, v0) < 0.02);
    assert!(rel_err(ev.impulse, 2.0 * m * v0) < 0.02);
    assert!(!rig.in_contact);
    assert!((impactor(&loads).energy() - 0.5 * m * v0 * v0).abs() < 1e-9);
}

#[test]
fn impulse_on_free_body_equals_impactor_momentum_change() {
    // Free 2 m steel bar along X, no supports, no damping.
    let mut s = Structure::new();
    let ids = s.add_line(
        Vec3::zeros(),
        Vec3::new(2.0, 0.0, 0.0),
        2,
        &Material::steel(),
        &Section::rect(0.2, 0.2),
    );
    let (m, v0) = (1000.0, 2.0);
    let i = Impactor::new(
        "Drop weight",
        m,
        0.1,
        Vec3::new(-0.25, 0.02, 0.0),
        Vec3::new(v0, 0.0, 0.0),
    );
    let mut loads = impact_set(i);
    let mut sim = DynamicSim::new(
        &s,
        SimConfig {
            damping: 0.0,
            ..quiet()
        },
    );
    while sim.time() < 0.6 {
        loads.step(&s, &mut sim, DT);
    }
    let bar: Vec3 = ids
        .iter()
        .map(|&n| sim.velocity(n) * sim.node_mass(n))
        .sum();
    let mass_bar: f64 = ids.iter().map(|&n| sim.node_mass(n)).sum();
    let dp = (impactor(&loads).velocity_now() - Vec3::new(v0, 0.0, 0.0)) * m;
    println!(
        "bar momentum {bar:?}, impactor change {dp:?} (bar {mass_bar:.0} kg, impactor now {:?})",
        impactor(&loads).velocity_now()
    );
    assert!(
        (bar + dp).norm() < 1e-6 * (m * v0),
        "momentum not conserved"
    );
    assert!(bar.x > 0.5 * m * v0);
}

#[test]
fn dropped_weight_energy_includes_fall() {
    // Cantilever 1 m above the table.
    let mut s = Structure::new();
    let ids = s.add_line(
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(2.0, 1.0, 0.0),
        10,
        &Material::steel(),
        &tube(),
    );
    s.add_support(Support::fixed(ids[0]));
    let mut i = Impactor::dropped("Drop weight", 500.0, 0.2, Vec3::new(1.0, 3.0, 0.0));
    i.prepare(&s);
    let e = i.energy();
    let hand = 500.0 * G * (3.0 - 1.0 - 0.2);
    println!("energy {e:.0} J vs {hand:.0} J");
    assert!(rel_err(e, hand) < 1e-9);
    let mut loads = impact_set(i);
    let mut sim = DynamicSim::new(&s, quiet());
    let mut hit = 0.0;
    while sim.time() < 1.0 {
        loads.step(&s, &mut sim, DT);
        let r = &loads.rig_state(sim.time()).impactors[0];
        if hit == 0.0 && !r.events.is_empty() {
            hit = r.events[0].impact_speed;
        }
    }
    let v = (2.0 * G * 1.8).sqrt();
    println!("impact speed {hit:.3} m/s vs {v:.3} m/s");
    assert!(rel_err(hit, v) < 0.02);
}
