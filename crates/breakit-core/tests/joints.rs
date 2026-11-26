//! Member end joints: pinned and semi-rigid ends against textbook results,
//! the dynamic solver against the static solve, and connection failure.
mod common;
use breakit_core::{
    solve_static, BreakCause, DamageState, DynamicSim, Joint, JointKind, Material, MaterialKind,
    NodeLoad, Section, SimConfig, Structure, Support, Vec3,
};
use common::*;

fn quiet(plasticity: bool) -> SimConfig {
    SimConfig {
        gravity: Vec3::zeros(),
        damping: 8.0,
        plasticity,
        ..Default::default()
    }
}

fn ramp_load(sim: &mut DynamicSim, node: usize, force: Vec3, ramp: f64, total: f64) {
    let frames = (total * 60.0) as usize;
    for f in 0..frames {
        let t = f as f64 / 60.0;
        sim.set_force(node, force * (t / ramp).min(1.0));
        sim.step(1.0 / 60.0);
    }
}

fn ei() -> f64 {
    210e9 * tube().iz
}

/// Beam of span `l` along X between two clamped supports, two members,
/// with `ends` at the supports. Returns (structure, midspan node).
fn clamped_beam(l: f64, ends: Joint) -> (Structure, usize) {
    let mut s = Structure::new();
    let ids = s.add_line(
        Vec3::zeros(),
        Vec3::new(l, 0.0, 0.0),
        2,
        &Material::steel(),
        &tube(),
    );
    s.add_support(Support::fixed(ids[0]));
    s.add_support(Support::fixed(ids[2]));
    s.set_ends(0, ends, Joint::RIGID);
    s.set_ends(1, Joint::RIGID, ends);
    (s, ids[1])
}

#[test]
fn pinned_ends_turn_a_clamped_beam_into_a_simply_supported_one() {
    let (l, p) = (4.0, 1000.0);
    let load = |n| [NodeLoad::force(n, Vec3::new(0.0, -p, 0.0))];
    let (rigid, mid) = clamped_beam(l, Joint::RIGID);
    let (pinned, _) = clamped_beam(l, Joint::PINNED);
    let d_rigid = solve_static(&rigid, &load(mid)).unwrap().displacements[mid].y;
    let res = solve_static(&pinned, &load(mid)).unwrap();
    let d_pinned = res.displacements[mid].y;
    assert!(rel_err(-d_rigid, p * l.powi(3) / (192.0 * ei())) < 1e-6);
    assert!(rel_err(-d_pinned, p * l.powi(3) / (48.0 * ei())) < 1e-6);
    // No moment at the pins, PL/4 at midspan.
    assert!(res.member_forces[0].mz[0].abs() < 1e-6 * p * l);
    assert!(rel_err(res.member_forces[0].mz[1].abs(), p * l / 4.0) < 1e-6);
}

#[test]
fn semi_rigid_root_adds_the_spring_rotation() {
    // Cantilever on a rotational spring: δ = PL³/3EI + PL²/k.
    let (l, p) = (2.0, 1000.0);
    let k = 2.0 * ei() / l;
    let (mut s, ids) = cantilever(l, 4, Material::steel());
    let spring = Joint {
        stiffness: [k; 2],
        ..Joint::RIGID
    };
    s.set_ends(0, spring, Joint::RIGID);
    let tip = ids[4];
    let res = solve_static(&s, &[NodeLoad::force(tip, Vec3::new(0.0, -p, 0.0))]).unwrap();
    let want = p * l.powi(3) / (3.0 * ei()) + p * l * l / k;
    assert!(rel_err(-res.displacements[tip].y, want) < 1e-6);
}

#[test]
fn dynamic_solver_matches_static_with_pinned_and_semi_rigid_ends() {
    let f = Vec3::new(0.0, -1000.0, 0.0);
    let (pinned, mid) = clamped_beam(4.0, Joint::PINNED);
    let semi = {
        let (mut s, ids) = cantilever(2.0, 4, Material::steel());
        let j = Joint {
            stiffness: [2.0 * ei() / 2.0; 2],
            ..Joint::RIGID
        };
        s.set_ends(0, j, Joint::RIGID);
        (s, ids[4])
    };
    for (s, node) in [(pinned, mid), semi] {
        let stat = solve_static(&s, &[NodeLoad::force(node, f)]).unwrap();
        let mut sim = DynamicSim::new(&s, quiet(false));
        sim.set_force(node, f);
        assert!(sim.settle(10.0, 1e-6), "did not settle");
        let (d, e) = (sim.displacement(node).y, stat.displacements[node].y);
        assert!(rel_err(d, e) < 0.01, "dynamic {d:e} vs static {e:e}");
    }
}

#[test]
fn pinned_truss_carries_load_axially() {
    // Triangle with pins everywhere: no bending, textbook bar forces.
    let mut s = Structure::new();
    let (mat, sec) = (Material::steel(), tube());
    let a = s.add_node(Vec3::zeros());
    let b = s.add_node(Vec3::new(2.0, 0.0, 0.0));
    let c = s.add_node(Vec3::new(1.0, 1.0, 0.0));
    for (i, j) in [(a, b), (a, c), (b, c)] {
        let m = s.add_member(i, j, mat.clone(), sec.clone());
        s.set_ends(m, Joint::PINNED, Joint::PINNED);
    }
    s.add_support(Support::pinned(a));
    s.add_support(Support {
        node: b,
        fixed: [false, true, true, false, false, false],
    });
    s.add_support(Support {
        node: c,
        fixed: [false, false, true, false, false, false],
    });
    let p = 10_000.0;
    let res = solve_static(&s, &[NodeLoad::force(c, Vec3::new(0.0, -p, 0.0))]).unwrap();
    let mf = &res.member_forces;
    assert!(rel_err(mf[0].axial, p / 2.0) < 1e-6, "tie {}", mf[0].axial);
    for m in [1, 2] {
        assert!(rel_err(mf[m].axial, -p / 2f64.sqrt()) < 1e-6);
    }
    for f in mf {
        for e in 0..2 {
            assert!(f.my[e].abs() + f.mz[e].abs() < 1e-6 * p);
        }
    }
}

/// Portal frame: two 3 m columns and a 4 m beam, bases pinned, every member
/// end connected by `joint`.
fn portal(joint: Joint, brace: bool) -> (Structure, usize) {
    let mut s = Structure::new();
    let (mat, sec) = (Material::steel(), Section::rect(0.08, 0.12));
    let n = [
        s.add_node(Vec3::zeros()),
        s.add_node(Vec3::new(0.0, 3.0, 0.0)),
        s.add_node(Vec3::new(4.0, 3.0, 0.0)),
        s.add_node(Vec3::new(4.0, 0.0, 0.0)),
    ];
    let mut pairs = vec![(n[0], n[1]), (n[1], n[2]), (n[2], n[3])];
    if brace {
        pairs.push((n[0], n[2]));
    }
    for (i, j) in pairs {
        let m = s.add_member(i, j, mat.clone(), sec.clone());
        s.set_ends(m, joint, joint);
    }
    s.add_support(Support::pinned(n[0]));
    s.add_support(Support::pinned(n[3]));
    (s, n[1])
}

#[test]
fn all_pinned_portal_is_a_mechanism_until_braced() {
    let load = |n| [NodeLoad::force(n, Vec3::new(1000.0, 0.0, 0.0))];
    let (open, top) = portal(Joint::PINNED, false);
    assert!(solve_static(&open, &load(top)).is_err());
    let (braced, top) = portal(Joint::PINNED, true);
    let res = solve_static(&braced, &load(top)).unwrap();
    assert!(res.displacements[top].x.abs() < 1e-3);

    // The dynamic solver agrees: the open frame sways over, the braced one holds.
    let sway = |s: &Structure, top| {
        let mut sim = DynamicSim::new(s, quiet(false));
        ramp_load(&mut sim, top, Vec3::new(1000.0, 0.0, 0.0), 0.5, 3.0);
        sim.displacement(top).x
    };
    let (open, top) = portal(Joint::PINNED, false);
    assert!(sway(&open, top) > 0.3, "open frame should sway over");
    let (braced, top) = portal(Joint::PINNED, true);
    let d = sway(&braced, top);
    assert!(rel_err(d, res.displacements[top].x) < 0.02, "braced {d:e}");
}

#[test]
fn pin_shears_off_before_the_member_yields() {
    // Hanging steel tie pinned at the top. The pin carries 0.6 of the
    // member's yield force.
    let mut s = Structure::new();
    let (mat, sec) = (Material::steel(), tube());
    let ids = s.add_line(Vec3::zeros(), Vec3::new(0.0, -2.0, 0.0), 2, &mat, &sec);
    s.add_support(Support::fixed(ids[0]));
    let pin = JointKind::Pinned.joint(&mat, &sec, 1.0);
    s.set_ends(0, pin, Joint::RIGID);
    let ny = 355e6 * sec.area;
    let run = |ratio: f64| {
        let mut sim = DynamicSim::new(&s, quiet(true));
        ramp_load(&mut sim, ids[2], Vec3::new(0.0, -ratio * ny, 0.0), 1.0, 2.0);
        sim
    };
    let safe = run(0.5);
    assert!(safe.break_events().is_empty());
    assert!(safe.damage(0).joint[0] > 0.7, "{}", safe.damage(0).joint[0]);
    let over = run(0.7);
    let ev = over.break_events();
    assert_eq!(ev.len(), 1);
    assert_eq!(
        (ev[0].member, ev[0].cause, ev[0].position),
        (0, BreakCause::Connection, 0.0)
    );
    assert_eq!(
        over.damage(0).plastic_strain,
        0.0,
        "the member never yielded"
    );
}

/// Steel tube cantilever, 2 m in 8 members, with `root` at the clamp and a
/// tip load of `ratio` times the member's plastic collapse load Mp/L.
fn cantilever_on(root: JointKind, ratio: f64, seconds: f64) -> DynamicSim {
    let (l, mat, sec) = (2.0, Material::steel(), tube());
    let (mut s, ids) = cantilever(l, 8, mat.clone());
    s.set_ends(0, root.joint(&mat, &sec, l / 8.0), Joint::RIGID);
    let p = ratio * 355e6 * sec.zz / l;
    let mut sim = DynamicSim::new(&s, quiet(true));
    ramp_load(&mut sim, ids[8], Vec3::new(0.0, -p, 0.0), 1.0, seconds);
    sim
}

#[test]
fn bolted_joint_yields_before_the_member_and_then_lets_go() {
    // The bolted joint carries 0.45 Mp.
    let safe = cantilever_on(JointKind::Bolted, 0.4, 2.0);
    assert!(safe.break_events().is_empty());
    assert_eq!(safe.damage(0).state, DamageState::Intact);

    let over = cantilever_on(JointKind::Bolted, 0.55, 4.0);
    let ev = over.break_events();
    assert!(!ev.is_empty(), "the joint should fail");
    assert_eq!(
        (ev[0].member, ev[0].cause, ev[0].position),
        (0, BreakCause::Connection, 0.0)
    );
    assert_eq!(
        over.hinge_rotation(0),
        0.0,
        "the member itself never yielded"
    );
}

#[test]
fn weld_cracks_before_the_member_hinge_would_snap() {
    // Rigid: a plastic hinge forms and snaps at 0.08 rad. Welded: the weld
    // cracks at 0.025 rad of rotation at its face.
    let rigid = cantilever_on(JointKind::Rigid, 1.3, 4.0);
    assert_eq!(rigid.break_events()[0].cause, BreakCause::Hinge);
    let welded = cantilever_on(JointKind::Welded, 1.3, 4.0);
    let ev = welded.break_events();
    assert_eq!(
        (ev[0].member, ev[0].cause, ev[0].position),
        (0, BreakCause::Connection, 0.0)
    );
    assert!(ev[0].time < rigid.break_events()[0].time);
    let held = cantilever_on(JointKind::Welded, 0.8, 2.0);
    assert!(held.break_events().is_empty(), "below yield the weld holds");
}

#[test]
fn static_solve_reports_joint_utilization() {
    let (l, mat, sec) = (2.0, Material::steel(), tube());
    let (mut s, ids) = cantilever(l, 8, mat.clone());
    let bolt = JointKind::Bolted.joint(&mat, &sec, l / 8.0);
    s.set_ends(0, bolt, Joint::RIGID);
    let p = 0.5 * bolt.moment_capacity[1] / l;
    let res = solve_static(&s, &[NodeLoad::force(ids[8], Vec3::new(0.0, -p, 0.0))]).unwrap();
    let u = res.member_forces[0].joint_utilization;
    assert!(rel_err(u[0], 0.5) < 1e-3, "{u:?}");
    assert_eq!(u[1], 0.0);
}

#[test]
fn joint_kinds_fit_their_materials() {
    assert!(JointKind::Welded.suits(MaterialKind::Steel));
    assert!(!JointKind::Welded.suits(MaterialKind::Timber));
    assert!(JointKind::Dowelled.suits(MaterialKind::Timber));
    assert!(JointKind::Monolithic.suits(MaterialKind::Concrete));
    assert!(!JointKind::Bolted.suits(MaterialKind::Concrete));
}
