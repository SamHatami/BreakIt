use breakit_core::{solve_static, DynamicSim, NodeLoad, SimConfig, Structure, Support, Vec3};
use breakit_parts::*;

fn rel_err(a: f64, b: f64) -> f64 {
    (a - b).abs() / b.abs()
}

#[test]
fn standard_catalog_loads_and_validates() {
    let c = Catalog::standard();
    assert!(c.parts.len() >= 12);
    assert!(c.families.len() >= 8);
    for p in &c.parts {
        assert!(p.choice.joints_suit_material(), "{}", p.id);
    }
}

#[test]
fn catalog_round_trips_through_ron() {
    let c = Catalog::standard();
    let text = ron::ser::to_string_pretty(&c, Default::default()).unwrap();
    assert_eq!(Catalog::from_ron(&text).unwrap(), c);
}

#[test]
fn standard_sections_match_published_values() {
    // Published values include root radii, which the profiles leave out, so
    // allow a few percent.
    let c = Catalog::standard();
    let size = |fam: &str, name: &str| {
        c.family(fam)
            .unwrap()
            .sizes
            .iter()
            .find(|s| s.name == name)
            .unwrap()
            .profile
            .section()
    };
    let ipe200 = size("steel-ipe", "IPE 200");
    assert!(rel_err(ipe200.area, 28.48e-4) < 0.06, "{}", ipe200.area);
    assert!(rel_err(ipe200.iz, 1943e-8) < 0.07, "{}", ipe200.iz);
    let chs = size("steel-chs", "CHS 114.3x5");
    assert!(rel_err(chs.area, 17.2e-4) < 0.01, "{}", chs.area);
    assert!(rel_err(chs.iz, 257e-8) < 0.01, "{}", chs.iz);
    let joist = size("timber-sawn", "45x220");
    assert!(rel_err(joist.iz, 0.045 * 0.22f64.powi(3) / 12.0) < 1e-6);
}

#[test]
fn cost_is_mass_times_price_plus_joints() {
    let c = Catalog::standard();
    let p = c.part("steel-ibeam-m").unwrap();
    let mass = p.choice.mass(4.0);
    let want = mass * 2.5 + 2.0 * 40.0;
    assert!(rel_err(c.prices.member(&p.choice, 4.0), want) < 1e-12);
    // A stiffer joint costs more than a pin on the same member.
    let mut pinned = p.choice;
    pinned.joints = [JointSpec::Pinned; 2];
    assert!(c.prices.member(&pinned, 4.0) < c.prices.member(&p.choice, 4.0));
}

#[test]
fn sandbox_limits_refuse_sections_outside_the_beam_model() {
    let c = Catalog::standard();
    let l = &c.limits;
    let steel = |profile| MemberChoice {
        material: MaterialSpec::Steel(SteelSpec::S355),
        profile,
        joints: [JointSpec::Welded; 2],
    };
    assert!(l
        .check(
            &steel(ProfileSpec::Rhs {
                b: 300.0,
                h: 300.0,
                t: 6.0
            }),
            3.0
        )
        .is_ok());
    assert!(matches!(
        l.check(
            &steel(ProfileSpec::Rhs {
                b: 600.0,
                h: 600.0,
                t: 4.0
            }),
            3.0
        ),
        Err(LimitError::WallTooThin { .. })
    ));
    assert!(matches!(
        l.check(
            &steel(ProfileSpec::I {
                h: 300.0,
                b: 300.0,
                tw: 6.0,
                tf: 3.0
            }),
            3.0
        ),
        Err(LimitError::FlangeTooThin { .. })
    ));
    assert!(matches!(
        l.check(&steel(ProfileSpec::Chs { d: 50.0, t: 1.0 }), 3.0),
        Err(LimitError::TooThin { .. })
    ));
    assert!(matches!(
        l.check(&steel(ProfileSpec::Chs { d: 20.0, t: 3.0 }), 6.0),
        Err(LimitError::TooSlender { .. })
    ));
    assert!(matches!(
        l.check(&steel(ProfileSpec::Chs { d: 50.0, t: 30.0 }), 1.0),
        Err(LimitError::BadShape)
    ));
    let welded_timber = MemberChoice {
        material: MaterialSpec::Timber(TimberSpec::C24, 0.0),
        profile: ProfileSpec::Rect { b: 45.0, h: 220.0 },
        joints: [JointSpec::Welded, JointSpec::Pinned],
    };
    assert_eq!(
        l.check(&welded_timber, 3.0),
        Err(LimitError::JointUnsuitable(JointSpec::Welded))
    );
    let concrete_i = MemberChoice {
        material: MaterialSpec::ReinforcedConcrete(ConcreteSpec::C30, 0.01, 40.0),
        profile: ProfileSpec::I {
            h: 400.0,
            b: 200.0,
            tw: 10.0,
            tf: 15.0,
        },
        joints: [JointSpec::Monolithic; 2],
    };
    assert_eq!(l.check(&concrete_i, 3.0), Err(LimitError::BadShape));
}

#[test]
fn family_choices_use_their_grade_size_and_default_joint() {
    let c = Catalog::standard();
    let f = c.family("steel-ipe").unwrap();
    let choice = f.choice(2, 2);
    assert_eq!(choice.material, MaterialSpec::Steel(SteelSpec::S460));
    assert_eq!(choice.profile, f.sizes[2].profile);
    assert_eq!(choice.joints, [JointSpec::Bolted; 2]);
    assert!(Layer::Catalog < Layer::Engineer && Layer::Engineer < Layer::Sandbox);
}

/// Portal frame from catalog parts: two columns and a beam, fixed bases.
fn portal(c: &Catalog, column: &str, beam: &str) -> (Structure, usize) {
    let mut s = Structure::new();
    let n: Vec<usize> = [(0.0, 0.0), (0.0, 3.0), (5.0, 3.0), (5.0, 0.0)]
        .iter()
        .map(|&(x, y)| s.add_node(Vec3::new(x, y, 0.0)))
        .collect();
    for (a, b, id) in [(0, 1, column), (1, 2, beam), (3, 2, column)] {
        let choice = c.part(id).unwrap().choice;
        let len = (s.nodes[n[b]] - s.nodes[n[a]]).norm();
        let r = choice.resolve(len);
        let m = s.add_member(n[a], n[b], r.material, r.section);
        s.set_ends(m, r.joints[0], r.joints[1]);
    }
    s.add_support(Support::fixed(n[0]));
    s.add_support(Support::fixed(n[3]));
    (s, n[1])
}

#[test]
fn catalog_parts_build_structures_both_solvers_agree_on() {
    let c = Catalog::standard();
    for (column, beam) in [
        ("steel-column", "steel-ibeam-m"),
        ("timber-post", "glulam-beam"),
        ("rc-column", "rc-beam"),
        ("steel-tube-l", "steel-tube-m"),
    ] {
        let (s, top) = portal(&c, column, beam);
        let f = Vec3::new(2000.0, 0.0, 0.0);
        let stat = solve_static(&s, &[NodeLoad::force(top, f)]).unwrap();
        let mut sim = DynamicSim::new(
            &s,
            SimConfig {
                gravity: Vec3::zeros(),
                damping: 8.0,
                plasticity: false,
                ..Default::default()
            },
        );
        sim.set_force(top, f);
        assert!(sim.settle(10.0, 1e-6), "{column}/{beam} did not settle");
        let (d, e) = (sim.displacement(top).x, stat.displacements[top].x);
        assert!(
            rel_err(d, e) < 0.02,
            "{column}/{beam}: dynamic {d:e} vs static {e:e}"
        );
    }
}
