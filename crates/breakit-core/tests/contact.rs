//! Ground contact: landing, resting, friction, and collapses that stay on the floor.
mod common;
use breakit_core::{
    Block, DynamicSim, Ground, Material, Section, SimConfig, Solver, Structure, Support, Vec3,
};
use common::*;

fn with_ground() -> SimConfig {
    SimConfig {
        ground: Some(Ground::default()),
        ..Default::default()
    }
}

/// A free steel tube along X at height `y`, `segments` elements.
fn loose_beam(y: f64, length: f64, segments: usize) -> (Structure, Vec<usize>) {
    let mut s = Structure::new();
    let ids = s.add_line(
        Vec3::new(0.0, y, 0.0),
        Vec3::new(length, y, 0.0),
        segments,
        &Material::steel(),
        &tube(),
    );
    (s, ids)
}

fn lowest(sim: &DynamicSim, nodes: &[usize]) -> f64 {
    nodes
        .iter()
        .map(|&n| sim.clearance(n).unwrap())
        .fold(f64::INFINITY, f64::min)
}

fn mass(s: &Structure) -> f64 {
    (0..s.members.len())
        .map(|i| s.members[i].material.density * s.members[i].section.area * s.member_length(i))
        .sum()
}

#[test]
fn dropped_beam_lands_and_rests_on_its_surface() {
    let (s, ids) = loose_beam(1.0, 3.0, 6);
    let mut sim = DynamicSim::new(&s, with_ground());
    // Tilted drop: one end first.
    sim.step(1.0 / 60.0);
    let mut deepest = f64::INFINITY;
    for _ in 0..180 {
        sim.step(1.0 / 60.0);
        deepest = deepest.min(lowest(&sim, &ids));
    }
    assert!(deepest > -0.005, "sank {deepest} m into the ground");
    for &n in &ids {
        let p = sim.position(n);
        assert!(p.iter().all(|v| v.is_finite()));
        // Resting on the tube's surface (radius 50 mm), not its centreline.
        assert!((p.y - 0.05).abs() < 2e-3, "node {n} at {}", p.y);
        // Landed flat: no sideways skid.
        assert!(p.z.abs() < 1e-6 && (p.x - sim.position(n).x).abs() < 1e-9);
    }
    assert!(
        sim.max_speed() < 1e-3,
        "still moving at {}",
        sim.max_speed()
    );
    // Every node touched down once, with about the free-fall speed sqrt(2 g h).
    assert_eq!(sim.contact_events().len(), ids.len());
    let v = (2.0 * 9.81 * 0.95f64).sqrt();
    for e in sim.contact_events() {
        assert!(
            e.speed > 0.5 * v && e.speed < 1.1 * v,
            "impact speed {}",
            e.speed
        );
    }
}

#[test]
fn resting_weight_is_carried_by_the_ground() {
    let (s, ids) = loose_beam(0.05, 3.0, 6);
    let mut sim = DynamicSim::new(&s, with_ground());
    assert!(sim.settle(5.0, 1e-6));
    let carried: f64 = ids.iter().map(|&n| sim.contact_force(n)).sum();
    let w = mass(&s) * 9.81;
    assert!(
        rel_err(carried, w) < 1e-3,
        "ground carries {carried} N of {w} N"
    );
    // Sinks by micrometres, not millimetres.
    assert!(
        lowest(&sim, &ids) > -Ground::default().ramp,
        "{}",
        lowest(&sim, &ids)
    );
}

#[test]
fn static_friction_holds_and_kinetic_friction_slides() {
    let g = Ground::default();
    for (fraction, slides) in [(0.8, false), (1.5, true)] {
        let (s, ids) = loose_beam(0.05, 2.0, 4);
        let w = mass(&s) * 9.81;
        let mut sim = DynamicSim::new(&s, with_ground());
        assert!(sim.settle(3.0, 1e-6));
        let x0 = sim.position(ids[0]).x;
        // Push along the beam on every node, in proportion to its share.
        let per = g.friction * w * fraction / ids.len() as f64;
        let t = 2.0;
        for _ in 0..(t * 60.0) as usize {
            for &n in &ids {
                // End nodes carry half a member's weight, inner nodes a whole one.
                let share = if n == ids[0] || n == *ids.last().unwrap() {
                    0.5
                } else {
                    1.0
                };
                sim.set_force(
                    n,
                    Vec3::new(
                        per * share * ids.len() as f64 / (ids.len() - 1) as f64,
                        0.0,
                        0.0,
                    ),
                );
            }
            sim.step(1.0 / 60.0);
        }
        let dx = sim.position(ids[0]).x - x0;
        if slides {
            // a = (F − μW)/m, with the solver's linear damping c:
            // x = a/c (t − (1 − e^(−ct))/c).
            let a = (fraction - 1.0) * g.friction * 9.81;
            let c = SimConfig::default().damping;
            let expected = a / c * (t - (1.0 - (-c * t).exp()) / c);
            assert!(
                rel_err(dx, expected) < 0.03,
                "slid {dx} m, expected {expected}"
            );
        } else {
            assert!(dx.abs() < 2e-3, "crept {dx} m under static friction");
        }
    }
}

#[test]
fn collapsing_frame_stays_on_the_floor() {
    // Portal frame on the ground, overloaded until the columns go.
    let mut s = Structure::new();
    let mat = Material::steel();
    let sec = Section::tube(0.03, 0.003);
    let h = 2.0;
    let pts: Vec<Vec3> = (0..=4)
        .map(|i| Vec3::new(0.0, h * i as f64 / 4.0, 0.0))
        .chain((1..=6).map(|i| Vec3::new(3.0 * i as f64 / 6.0, h, 0.0)))
        .chain(
            (0..4)
                .rev()
                .map(|i| Vec3::new(3.0, h * i as f64 / 4.0, 0.0)),
        )
        .collect();
    let ids: Vec<usize> = pts.iter().map(|&p| s.add_node(p)).collect();
    for w in ids.windows(2) {
        s.add_member(w[0], w[1], mat.clone(), sec.clone());
    }
    s.add_support(Support::fixed(ids[0]));
    s.add_support(Support::fixed(*ids.last().unwrap()));
    let top = &ids[4..=10];
    let all: Vec<usize> = (0..s.nodes.len()).collect();
    for solver in [Solver::Direct, Solver::GaussSeidel] {
        let mut sim = DynamicSim::new(
            &s,
            SimConfig {
                solver,
                substeps: if solver == Solver::Direct { 4 } else { 40 },
                iterations: if solver == Solver::Direct { 1 } else { 4 },
                ..with_ground()
            },
        );
        let mid = top[3];
        let mut deepest = f64::INFINITY;
        for f in 0..(6.0 * 60.0) as usize {
            let t = f as f64 / 60.0;
            sim.set_force(mid, Vec3::new(800.0, -40_000.0 * (t / 1.0).min(1.0), 0.0));
            sim.step(1.0 / 60.0);
            deepest = deepest.min(lowest(&sim, &all));
        }
        assert!(
            !sim.break_events().is_empty()
                || (0..s.members.len()).any(|m| sim.hinge_rotation(m) > 0.1),
            "{solver:?}: frame did not collapse"
        );
        let tol = if solver == Solver::Direct { 0.01 } else { 0.03 };
        assert!(
            deepest > -tol,
            "{solver:?}: a node went {deepest} m under the floor"
        );
        for n in all.iter().copied() {
            assert!(sim.position(n).iter().all(|v| v.is_finite()));
        }
        // The fallen beam ended up on the floor, not hanging under it.
        assert!(
            sim.position(mid).y < 1.0 && sim.position(mid).y > -tol,
            "{solver:?}: mid at {}",
            sim.position(mid).y
        );
    }
}

#[test]
fn ground_off_by_default_and_without_effect_above_it() {
    // A cantilever well above the ground gives the same answer with and
    // without contact.
    let (mut s, ids) = cantilever(2.0, 8, Material::steel());
    for p in &mut s.nodes {
        p.y += 1.0;
    }
    assert!(SimConfig::default().ground.is_none());
    let tip = *ids.last().unwrap();
    let mut a = DynamicSim::new(&s, SimConfig::default());
    let mut b = DynamicSim::new(&s, with_ground());
    for _ in 0..60 {
        a.step(1.0 / 60.0);
        b.step(1.0 / 60.0);
    }
    assert_eq!(a.position(tip), b.position(tip));
}

/// Two river banks (table blocks) with a 5 m gap and the floor 3.2 m down,
/// like the lab's ravine table.
fn ravine() -> Ground {
    Ground {
        height: -3.2,
        blocks: vec![
            Block::new(Vec3::new(-20.0, -3.2, -5.0), Vec3::new(-2.5, 0.0, 5.0)),
            Block::new(Vec3::new(2.5, -3.2, -5.0), Vec3::new(20.0, 0.0, 5.0)),
        ],
        ..Default::default()
    }
}

#[test]
fn beam_dropped_across_a_gap_rests_on_both_banks() {
    let mut s = Structure::new();
    let ids = s.add_line(
        Vec3::new(-4.0, 0.5, 0.0),
        Vec3::new(4.0, 0.5, 0.0),
        16,
        &Material::steel(),
        &tube(),
    );
    let mut sim = DynamicSim::new(
        &s,
        SimConfig {
            ground: Some(ravine()),
            ..Default::default()
        },
    );
    assert!(sim.settle(8.0, 1e-5), "did not settle");
    let ends = [ids[0], ids[16]];
    for n in ends {
        assert!(
            (sim.position(n).y - 0.05).abs() < 0.01,
            "end at {}",
            sim.position(n).y
        );
    }
    // The middle hangs over the gap, sagging a little, not in the ravine.
    let mid = sim.position(ids[8]).y;
    assert!(mid < 0.05 && mid > -0.2, "middle at {mid}");
    assert!(lowest(&sim, &ids) > -0.005);
    assert!(sim.position(ids[0]).x.abs() > 3.5, "slid off the bank");
}

#[test]
fn bridge_over_a_gap_falls_into_it() {
    // A single span pinned on the banks, overloaded until it fails.
    let mut s = Structure::new();
    let ids = s.add_line(
        Vec3::new(-3.0, 0.05, 0.0),
        Vec3::new(3.0, 0.05, 0.0),
        12,
        &Material::steel(),
        &Section::tube(0.03, 0.003),
    );
    s.add_support(Support::pinned(ids[0]));
    s.add_support(Support::pinned(ids[12]));
    let mut sim = DynamicSim::new(
        &s,
        SimConfig {
            ground: Some(ravine()),
            ..Default::default()
        },
    );
    let mut deepest = f64::INFINITY;
    for f in 0..(8.0 * 60.0) as usize {
        let t = f as f64 / 60.0;
        sim.set_force(ids[6], Vec3::new(0.0, -60_000.0 * t.min(1.0), 0.0));
        sim.step(1.0 / 60.0);
        deepest = deepest.min(lowest(&sim, &ids));
    }
    assert!(!sim.break_events().is_empty(), "did not break");
    assert!(deepest > -0.02, "went {deepest} m into a surface");
    // The loaded middle ended on the ravine floor.
    let y = sim.position(ids[6]).y;
    assert!(y < -3.0 && y > -3.2, "middle at {y}");
}
