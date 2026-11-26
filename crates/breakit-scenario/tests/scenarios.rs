//! Scenario data, limits, test runs and grading, on sample designs.
mod common;

use breakit_core::{Material, NodeLoad, Section, SimConfig, Structure, Support, Vec3};
use breakit_scenario::*;
use common::{mast, warren};

fn bridge() -> Scenario {
    builtin()
        .into_iter()
        .find(|s| s.id == "traffic_bridge")
        .unwrap()
}

fn tower() -> Scenario {
    builtin()
        .into_iter()
        .find(|s| s.id == "quake_tower")
        .unwrap()
}

fn grade(s: &Scenario, st: &Structure) -> Report {
    evaluate(
        s,
        st,
        &[],
        &CostModel::default(),
        PlayMode::Challenge,
        SimConfig::default(),
    )
    .unwrap_or_else(|v| panic!("blocked: {v:?}"))
}

fn limits(s: &Scenario, st: &Structure, mode: PlayMode) -> Vec<Violation> {
    check_limits(s, st, &CostModel::default().estimate(st), mode)
}

#[test]
fn builtin_scenarios_parse_and_round_trip() {
    let all = builtin();
    assert_eq!(all.len(), BUILTIN.len());
    for s in &all {
        assert_eq!(&Scenario::from_ron(&s.to_ron()).unwrap(), s);
        assert!(!s.anchors.is_empty() && s.duration > 0.0);
        let mut ids: Vec<_> = all.iter().map(|s| &s.id).collect();
        ids.dedup();
        assert_eq!(ids.len(), all.len());
    }
}

#[test]
fn a_sensible_bridge_is_allowed_and_passes() {
    let s = bridge();
    let st = warren(0.03, 0.003, 1.0);
    assert!(limits(&s, &st, PlayMode::Challenge).is_empty());
    let r = grade(&s, &st);
    print!("{}", r.summary());
    assert!(r.passed, "{:?}", r.failure);
    assert!(r.stars >= 2 && r.score > 600, "{} {}", r.stars, r.score);
    assert!(r.safety_margin > 1.0);
    // The timeline has both convoys and the pass.
    let started = r
        .timeline
        .iter()
        .filter(|e| matches!(e.what, Happening::EventStarted { .. }))
        .count();
    assert_eq!(started, 2);
    assert!(r.timeline.iter().any(|e| e.what == Happening::Passed));
    // The proof test ran to collapse at some factor above 1.
    let f = r.reserve_factor.unwrap();
    assert!(f > 1.0 && f < 5.0, "{f}");
}

#[test]
fn a_flimsy_bridge_collapses_and_scores_nothing() {
    let r = grade(&bridge(), &warren(0.015, 0.0015, 1.0));
    assert!(!r.passed);
    assert_eq!((r.score, r.stars), (0, 0));
    assert!(r.collapse_time.is_some() && r.first_break.is_some());
    assert!(r.criteria.iter().any(|c| !c.bonus && !c.ok));
}

#[test]
fn overbuilding_passes_but_scores_lower() {
    let s = bridge();
    let right = grade(&s, &warren(0.03, 0.003, 1.0));
    let heavy = grade(&s, &warren(0.05, 0.005, 1.0));
    assert!(heavy.passed && right.passed);
    assert!(heavy.safety_margin > s.scoring.margin_band[1]);
    assert!(heavy.parts.margin < right.parts.margin);
    assert!(heavy.parts.cost < right.parts.cost);
    assert!(
        heavy.score < right.score,
        "{} vs {}",
        heavy.score,
        right.score
    );
}

#[test]
fn limits_report_each_problem_in_plain_words() {
    let s = bridge();
    let mut st = warren(0.03, 0.003, 1.0);
    // Off the build area.
    let far = st.add_node(Vec3::new(0.0, 5.0, 0.0));
    st.add_member(far, 7, Material::steel(), Section::tube(0.03, 0.003));
    // A support between the banks, and a fixed one on a pin bearing.
    st.add_support(Support::pinned(3));
    st.supports[0] = Support::fixed(0);
    // Concrete is not allowed here.
    st.add_member(1, 8, Material::concrete(), Section::rect(0.2, 0.2));
    let v = limits(&s, &st, PlayMode::Challenge);
    let has = |f: fn(&Violation) -> bool| v.iter().any(f);
    assert!(has(
        |v| matches!(v, Violation::OutOfVolume { node } if *node == 13)
    ));
    assert!(has(|v| matches!(
        v,
        Violation::SupportOffAnchor { node: 3 }
    )));
    assert!(has(|v| matches!(
        v,
        Violation::AnchorTooWeak { node: 0, .. }
    )));
    assert!(has(|v| matches!(
        v,
        Violation::MaterialNotAllowed {
            material: MaterialClass::Concrete,
            ..
        }
    )));
    for x in &v {
        let line = x.to_string();
        assert!(line.ends_with('.') && line.len() > 10, "{line}");
    }
    // Sandbox drops the caps.
    assert!(limits(&s, &st, PlayMode::Sandbox).is_empty());
}

#[test]
fn budget_and_deck_gaps_block_the_test() {
    let s = bridge();
    let heavy = warren(0.08, 0.01, 1.0);
    let v = limits(&s, &heavy, PlayMode::Challenge);
    assert!(
        v.iter().any(|v| matches!(v, Violation::OverBudget { .. })),
        "{v:?}"
    );
    let r = evaluate(
        &s,
        &heavy,
        &[],
        &CostModel::default(),
        PlayMode::Challenge,
        SimConfig::default(),
    );
    assert!(r.is_err());

    // A deck with 3 m bays: wheels would fall between nodes, even in sandbox.
    let mut st = Structure::new();
    let ids = st.add_line(
        Vec3::new(-3.0, 0.0, 0.0),
        Vec3::new(3.0, 0.0, 0.0),
        2,
        &Material::steel(),
        &Section::tube(0.05, 0.005),
    );
    st.add_support(Support::pinned(ids[0]));
    st.add_support(Support::pinned(ids[2]));
    let v = limits(&s, &st, PlayMode::Sandbox);
    assert!(
        matches!(v[..], [Violation::DeckGap { gap, .. }] if (gap - 3.0).abs() < 1e-9),
        "{v:?}"
    );
}

#[test]
fn bridge_trusses_either_side_of_the_lane_share_the_traffic() {
    // Two planar trusses at z = -1 and z = +1, tied by cross beams.
    let s = bridge();
    let mut st = Structure::new();
    let (m, sec) = (Material::steel(), Section::tube(0.03, 0.003));
    let mut halves = Vec::new();
    for z in [-1.0, 1.0] {
        let mut one = warren(0.03, 0.003, 1.0);
        for p in &mut one.nodes {
            p.z = z;
        }
        let off = st.nodes.len();
        st.nodes.extend(one.nodes);
        for mm in one.members {
            st.add_member(mm.a + off, mm.b + off, mm.material, mm.section);
        }
        for sup in one.supports {
            st.add_support(Support::pinned(sup.node + off));
        }
        halves.push(off);
    }
    for i in 0..7 {
        st.add_member(halves[0] + i, halves[1] + i, m.clone(), sec.clone());
    }
    let b = Binding::new(&s, &st);
    let deck = b.deck.as_ref().unwrap();
    assert_eq!(deck.stations.len(), 7);
    assert!(deck.stations.iter().all(|st| st.others.len() == 1));
    let cost = CostModel::default().estimate(&st);
    let s_free = Scenario {
        limits: Limits {
            budget: None,
            ..s.limits.clone()
        },
        ..s.clone()
    };
    let r = evaluate(
        &s_free,
        &st,
        &[],
        &CostModel::default(),
        PlayMode::Challenge,
        SimConfig::default(),
    )
    .unwrap();
    assert!(r.passed);
    assert!((r.bill.total() - cost.total()).abs() < 1e-9);
    // Half the traffic per truss: well under the single truss's utilisation.
    let single = grade(&s, &warren(0.03, 0.003, 1.0));
    assert!(r.peak_utilization < 0.75 * single.peak_utilization);
}

#[test]
fn forces_at_matches_the_convoy_on_the_deck() {
    let s = bridge();
    let st = warren(0.03, 0.003, 1.0);
    let b = Binding::new(&s, &st);
    let total = |t: f64| -> f64 { b.forces_at(&s, t).iter().map(|f| -f.y).sum() };
    assert_eq!(total(0.0), 0.0);
    // First truck fully on the 6 m deck: front at 4.0 m + both axles on.
    let t = 0.5 + 4.0 / 4.0 + 0.1;
    let on = total(t);
    // Lever rule beyond the last node sends nothing to the ground here,
    // since deck nodes sit at both lane ends.
    assert!((on - 74_000.0).abs() < 1.0, "{on}");
    // After the test the proof test scales the deck stand-in load.
    let early = total(s.duration + 1.5);
    let late = total(s.duration + 3.0);
    assert!(late > early && early > 0.0);
    assert!(Binding::factor_at(&s, s.duration + 2.0) > 1.0);
}

#[test]
fn the_tank_is_carried_and_shaken() {
    let s = tower();
    let st = mast(0.025, 0.0025, 2);
    let r = grade(&s, &st);
    print!("{}", r.summary());
    assert!(r.passed);
    let drift = r.payload_drift[0].1;
    assert!(drift > 0.002 && drift < 0.1, "{drift}");
    // Without a node near the tank the test cannot run.
    let mut low = st.clone();
    low.nodes.last_mut().unwrap().y = 5.0;
    let v = limits(&s, &low, PlayMode::Challenge);
    assert!(v
        .iter()
        .any(|v| matches!(v, Violation::PayloadUnattached { .. })));
}

#[test]
fn sandbox_runs_player_loads_and_is_unranked() {
    let s = bridge();
    let st = warren(0.03, 0.003, 1.0);
    let extra = [NodeLoad::force(3, Vec3::new(0.0, -20_000.0, 0.0))];
    let costs = CostModel::default();
    let with = evaluate(
        &s,
        &st,
        &extra,
        &costs,
        PlayMode::Sandbox,
        SimConfig::default(),
    )
    .unwrap();
    let without = evaluate(
        &s,
        &st,
        &[],
        &costs,
        PlayMode::Sandbox,
        SimConfig::default(),
    )
    .unwrap();
    assert!(!with.ranked);
    assert!(with.peak_utilization > without.peak_utilization);
    // Challenge mode ignores player loads.
    let ch = evaluate(
        &s,
        &st,
        &extra,
        &costs,
        PlayMode::Challenge,
        SimConfig::default(),
    )
    .unwrap();
    assert!((ch.peak_utilization - without.peak_utilization).abs() < 1e-9);
}

#[test]
fn joint_kinds_change_the_price() {
    let st = warren(0.03, 0.003, 1.0);
    let c = CostModel::default();
    let bolted = c.estimate(&st);
    let pinned = c.estimate_with_joints(&st, &vec![[JointClass::Pinned; 2]; st.members.len()]);
    assert!((bolted.joints - 2.0 * pinned.joints).abs() < 1e-9);
    assert_eq!(bolted.materials, pinned.materials);
}

fn plant() -> Scenario {
    builtin()
        .into_iter()
        .find(|s| s.id == "plant_platform")
        .unwrap()
}

#[test]
fn a_braced_platform_keeps_the_pump_on() {
    let s = plant();
    let r = grade(&s, &common::platform(0.04, 0.004, true));
    print!("{}", r.summary());
    assert!(r.passed, "{:?}", r.failure);
    let (_, pump) = &r.payloads[0];
    assert!(pump.lost.is_none());
    assert!(pump.sag > 0.001 && pump.tilt_deg < 0.5);
    assert!(r
        .criteria
        .iter()
        .any(|c| c.label == "pump skid stays on" && c.ok && !c.bonus));
}

#[test]
fn a_swaying_platform_shakes_the_pump_off_and_fails() {
    let s = plant();
    let r = grade(&s, &common::platform(0.06, 0.005, false));
    assert!(!r.passed);
    assert!(r.failure.as_deref().unwrap().contains("pump skid came off"));
    let lost = r
        .timeline
        .iter()
        .find_map(|e| match &e.what {
            Happening::PayloadLost {
                name, mass, radius, ..
            } => Some((name.clone(), *mass, *radius)),
            _ => None,
        })
        .expect("a PayloadLost event for the impact hand-off");
    assert_eq!(lost.0, "pump skid");
    assert_eq!(lost.1, 2500.0);
    assert!(lost.2 > 0.5);
}

#[test]
fn equipment_needs_a_node_under_every_foot() {
    let s = plant();
    let mut st = common::platform(0.04, 0.004, true);
    // Move the node under one pump foot away.
    let foot = st
        .nodes
        .iter()
        .position(|p| (p - Vec3::new(1.0, 3.0, 0.5)).norm() < 1e-9)
        .unwrap();
    st.nodes[foot].x = 1.25;
    let v = limits(&s, &st, PlayMode::Challenge);
    assert!(
        v.iter().any(
            |v| matches!(v, Violation::PayloadUnattached { payload } if payload == "pump skid")
        ),
        "{v:?}"
    );
}

#[test]
fn strict_limits_fail_and_loose_ones_only_cost_score() {
    let st = common::platform(0.04, 0.004, true);
    let mut s = plant();
    s.payloads[0].limits.sag = Some(0.001);
    let loose = grade(&s, &st);
    assert!(loose.passed);
    assert!(loose
        .criteria
        .iter()
        .any(|c| c.label == "pump skid sag" && !c.ok && c.bonus));
    s.payloads[0].limits.strict = true;
    let strict = grade(&s, &st);
    assert!(!strict.passed);
    assert!(strict.failure.unwrap().contains("sank"));
}

#[test]
fn weak_bolts_tear_out_under_the_shaking() {
    let mut s = plant();
    s.payloads[1].mount = Mount::Bolted { capacity: 50.0 };
    let r = grade(&s, &common::platform(0.04, 0.004, true));
    assert!(r.payloads[1].1.bolts_failed);
    assert!(r
        .timeline
        .iter()
        .any(|e| matches!(e.what, Happening::BoltsFailed { payload: 1, .. })));
}

fn scenario(id: &str) -> Scenario {
    builtin().into_iter().find(|s| s.id == id).unwrap()
}

#[test]
fn the_fan_wall_loads_the_billboard_and_a_slim_tower_blows_over() {
    let s = scenario("sign_wind");
    let b = Binding::load_set(&s, s.settle);
    assert!(b
        .cases
        .iter()
        .any(|c| matches!(c.kind, breakit_core::loads::LoadKind::Wind(_))));
    let good = grade(&s, &common::sign_tower(0.035, 0.0035, 2));
    print!("{}", good.summary());
    assert!(good.passed, "{:?}", good.failure);
    // ½ ρ C_d A U² on the 18 m² board at 40 m/s is about 32 kN: the
    // tower's members must feel it.
    assert!(good.peak_utilization > 0.3, "{}", good.peak_utilization);
    let bad = grade(&s, &common::sign_tower(0.02, 0.002, 2));
    assert!(!bad.passed);
}

#[test]
fn the_surge_pushes_the_jetty_legs() {
    let s = scenario("flood_jetty");
    let braced = grade(&s, &common::jetty(0.03, 0.003, true));
    print!("{}", braced.summary());
    assert!(braced.passed, "{:?}", braced.failure);
    // Unbraced legs sway far more under the same surge.
    let sway = grade(&s, &common::jetty(0.06, 0.005, false));
    let drift = |r: &Report| r.payloads[0].1.drift;
    assert!(
        drift(&sway) > 1.5 * drift(&braced),
        "{} vs {}",
        drift(&sway),
        drift(&braced)
    );
}
