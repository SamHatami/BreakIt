//! Concrete, timber and composite material models against textbook values.
mod common;
use breakit_core::concrete::RcSection;
use breakit_core::material::{hankinson, Model};
use breakit_core::{
    solve_static, BreakCause, DamageState, DynamicSim, Material, NodeLoad, Section, SimConfig,
    Structure, Support, Vec3,
};
use common::*;

fn quiet() -> SimConfig {
    SimConfig {
        gravity: Vec3::zeros(),
        damping: 8.0,
        ..Default::default()
    }
}

/// Ramp a force on `node` over `ramp` seconds, then hold to `total`.
fn ramp(sim: &mut DynamicSim, node: usize, force: Vec3, ramp: f64, total: f64) {
    for f in 0..(total * 60.0) as usize {
        let t = f as f64 / 60.0;
        sim.set_force(node, force * (t / ramp).min(1.0));
        sim.step(1.0 / 60.0);
    }
}

/// Cantilever along +X clamped at x = 0, load along -Y at the tip.
fn cantilever_of(
    l: f64,
    segments: usize,
    mat: &Material,
    sec: &Section,
) -> (Structure, Vec<usize>) {
    let mut s = Structure::new();
    let ids = s.add_line(Vec3::zeros(), Vec3::new(l, 0.0, 0.0), segments, mat, sec);
    s.add_support(Support::fixed(ids[0]));
    (s, ids)
}

/// Vertical member hanging down (tie) or standing up (column) from a clamp.
fn vertical(l: f64, segments: usize, mat: &Material, sec: &Section) -> (Structure, Vec<usize>) {
    let mut s = Structure::new();
    let ids = s.add_line(Vec3::zeros(), Vec3::new(0.0, l, 0.0), segments, mat, sec);
    s.add_support(Support::fixed(ids[0]));
    (s, ids)
}

fn concrete_props(m: &Material) -> &breakit_core::material::ConcreteProps {
    match &m.model {
        Model::Concrete(c) => c,
        _ => unreachable!(),
    }
}

/// 300 x 500 mm beam, 1 % steel, 40 mm to bar centres: bending about z.
fn rc_beam() -> (Material, Section, RcSection) {
    let mat = Material::reinforced_concrete();
    let sec = Section::rect(0.3, 0.5);
    let rc = RcSection::new(&sec, mat.youngs_modulus, concrete_props(&mat));
    (mat, sec, rc)
}

// ---------------------------------------------------------------- sections

#[test]
fn rc_section_matches_hand_calculation() {
    let (_, _, rc) = rc_beam();
    let ax = &rc.axes[1];
    let (b, h, d, d2) = (0.3, 0.5, 0.46, 0.04);
    let as_half = 0.5 * 0.01 * b * h; // 750 mm² per face
    let (fck, fy, es, ec) = (30e6, 500e6, 200e9, 33e9);

    // Squash load, EN 1992: f_ck (A − As) + Es ε_c2 As.
    let n0 = fck * (b * h - 2.0 * as_half) + 400e6 * 2.0 * as_half;
    assert!(rel_err(rc.squash_load, n0) < 1e-12);
    // Cracking moment from the gross section.
    assert!(rel_err(ax.cracking_moment, 2.9e6 * b * h * h / 6.0) < 1e-12);

    // Cracked transformed section: first moment of area about the neutral
    // axis vanishes.
    let n = es / ec;
    let x = ax.cracked_neutral_axis;
    let first_moment = b * x * x / 2.0 + (n - 1.0) * as_half * (x - d2) - n * as_half * (d - x);
    assert!(first_moment.abs() < 1e-9, "{first_moment}");
    assert!(ax.cracked_inertia < 0.3 * ax.gross_inertia);

    // Ultimate moment, worked by hand: tension bars yield, compression bars
    // stay elastic. 0.8 x b fck + As' Es ε_cu (x − d')/x = As fy is the
    // quadratic 0.8 b fck x² + (As' Es ε_cu − As fy) x − As' Es ε_cu d' = 0.
    let k = as_half * es * 0.0035;
    let (qa, qb, qc) = (0.8 * b * fck, k - as_half * fy, -k * d2);
    let xu = (-qb + (qb * qb - 4.0 * qa * qc).sqrt()) / (2.0 * qa);
    assert!((xu - 0.044585).abs() < 1e-5, "x_u {xu}");
    let cc = 0.8 * xu * b * fck;
    let cs = k * (xu - d2) / xu;
    let mu = cc * (h / 2.0 - 0.4 * xu) + cs * (h / 2.0 - d2) + as_half * fy * (d - h / 2.0);
    assert!(rel_err(mu, 164.6e3) < 1e-3, "textbook M_u {mu}");
    assert!(
        rel_err(ax.ultimate_moment, mu) < 1e-6,
        "{}",
        ax.ultimate_moment
    );
    // The interaction curve passes through the pure-bending point.
    assert!(rel_err(ax.moment_capacity(0.0), mu) < 0.01);
    // Moderate compression raises the capacity, tension lowers it.
    assert!(ax.moment_capacity(0.2 * n0) > 1.3 * mu);
    assert!(ax.moment_capacity(-0.5 * rc.tension_yield) < 0.8 * mu);
    assert_eq!(ax.moment_capacity(1.01 * n0), 0.0);

    // EN 1992 (6.2.a) shear resistance without links.
    let kk: f64 = 1.0 + (200.0f64 / 460.0).sqrt();
    let rho = as_half / (b * d);
    let v = 0.18 * kk * (100.0 * rho * 30.0f64).cbrt() * 1e6 * b * d;
    assert!(rel_err(ax.shear_capacity(0.0, rc.area, fck), v) < 1e-12);
    assert!(rel_err(v, 104.5e3) < 0.01, "V_Rc {v}");
}

#[test]
fn hankinson_grain_angle() {
    use std::f64::consts::FRAC_PI_2;
    let along = Material::timber();
    let across = Material::timber_at_grain_angle(FRAC_PI_2);
    assert!(rel_err(along.youngs_modulus, 11e9) < 1e-12);
    assert!(rel_err(across.youngs_modulus, 0.37e9) < 1e-9);
    // f_t at 15° to the grain is under a third of f_t0.
    let f15 = hankinson(14.5e6, 0.4e6, 15f64.to_radians());
    assert!(f15 < 14.5e6 / 3.0 && f15 > 0.4e6);
}

// ---------------------------------------------------------------- concrete

#[test]
fn cracked_rc_cantilever_softens_by_branson() {
    // 3 m cantilever loaded to ~60 % of M_u at the root: cracked, not yielded.
    let (mat, sec, rc) = rc_beam();
    let ax = &rc.axes[1];
    let (l, segs) = (3.0, 6);
    let p = 0.6 * ax.ultimate_moment / l;
    let (s, ids) = cantilever_of(l, segs, &mat, &sec);
    let mut sim = DynamicSim::new(&s, quiet());
    ramp(&mut sim, ids[segs], Vec3::new(0.0, -p, 0.0), 1.0, 4.0);
    assert!(sim.break_events().is_empty());
    let root = sim.damage(0);
    assert_eq!(root.state, DamageState::Cracked);
    assert!(root.crack[0] > 0.0 && root.crack[0] < 1.0);
    assert!(root.stiffness < 0.8);
    assert_eq!(sim.hinge_rotation(0), 0.0);

    // Oracle: static solve with every element's E scaled by Branson's
    // Ie/Ig for its peak (root-end) moment.
    let mut soft = s.clone();
    for (i, m) in soft.members.iter_mut().enumerate() {
        let ma = p * (l - i as f64 * l / segs as f64);
        let r = (ax.cracking_moment / ma).min(1.0).powi(3);
        m.material.youngs_modulus *= r + (1.0 - r) * ax.cracked_inertia / ax.gross_inertia;
    }
    let load = [NodeLoad::force(ids[segs], Vec3::new(0.0, -p, 0.0))];
    let want = solve_static(&soft, &load).unwrap().displacements[ids[segs]].y;
    let rigid = solve_static(&s, &load).unwrap().displacements[ids[segs]].y;
    let got = sim.displacement(ids[segs]).y;
    assert!(
        got < 1.5 * rigid,
        "cracking should soften: {got} vs {rigid}"
    );
    assert!(rel_err(got, want) < 0.05, "dynamic {got} vs Branson {want}");
}

#[test]
fn rc_cantilever_holds_below_mu_and_fails_by_crushing_above() {
    let (mat, sec, rc) = rc_beam();
    let ax = &rc.axes[1];
    let l = 3.0;
    let run = |ratio: f64, secs: f64| {
        let (s, ids) = cantilever_of(l, 6, &mat, &sec);
        let mut sim = DynamicSim::new(&s, quiet());
        let p = ratio * ax.ultimate_moment / l;
        // Load slowly so the hinge stage is visible before the crush.
        let mut yielded_before_break = false;
        for f in 0..(secs * 60.0) as usize {
            let t = f as f64 / 60.0;
            sim.set_force(ids[6], Vec3::new(0.0, -p * (t / 2.0).min(1.0), 0.0));
            sim.step(1.0 / 60.0);
            if sim.break_events().is_empty() && sim.damage(0).state == DamageState::Yielded {
                yielded_before_break = true;
            }
        }
        (sim, yielded_before_break)
    };
    let (safe, _) = run(0.95, 4.0);
    assert!(safe.break_events().is_empty());
    assert_eq!(safe.hinge_rotation(0), 0.0, "no hinge below M_u");

    let (over, yielded) = run(1.15, 6.0);
    let ev = over.break_events();
    assert!(!ev.is_empty(), "over M_u the root hinge should crush");
    assert_eq!((ev[0].member, ev[0].cause), (0, BreakCause::Crushing));
    assert_eq!(ev[0].position, 0.0);
    assert!(
        yielded,
        "bars yield (visible hinge) before the concrete crushes"
    );
}

#[test]
fn plain_concrete_breaks_at_first_crack_where_rc_survives() {
    let sec = Section::rect(0.3, 0.5);
    let mcr = 2.9e6 * 0.3 * 0.25 / 6.0;
    let l = 3.0;
    let run = |mat: Material, ratio: f64| {
        let (s, ids) = cantilever_of(l, 6, &mat, &sec);
        let mut sim = DynamicSim::new(&s, quiet());
        ramp(
            &mut sim,
            ids[6],
            Vec3::new(0.0, -ratio * mcr / l, 0.0),
            1.0,
            3.0,
        );
        sim
    };
    assert!(run(Material::concrete(), 0.9).break_events().is_empty());
    let plain = run(Material::concrete(), 1.1);
    assert_eq!(plain.break_events()[0].cause, BreakCause::Brittle);
    assert_eq!(plain.break_events()[0].position, 0.0);
    let rc = run(Material::reinforced_concrete(), 1.1);
    assert!(rc.break_events().is_empty());
    assert_eq!(rc.damage(0).state, DamageState::Cracked);
}

#[test]
fn rc_tie_cracks_then_bars_carry_the_load_then_snap() {
    let mat = Material::reinforced_concrete();
    let sec = Section::rect(0.3, 0.3);
    let rc = RcSection::new(&sec, mat.youngs_modulus, concrete_props(&mat));
    let l = 2.0;
    let run = |force: f64, secs: f64| {
        let (s, ids) = vertical(l, 2, &mat, &sec);
        let mut sim = DynamicSim::new(&s, quiet());
        ramp(&mut sim, ids[2], Vec3::new(0.0, force, 0.0), 1.0, secs);
        (sim, ids)
    };
    assert!(rc.tension_cracking < 0.8 * rc.tension_yield);
    let n = 0.8 * rc.tension_yield;
    let (sim, ids) = run(n, 4.0);
    assert!(sim.break_events().is_empty());
    let d = sim.damage(0);
    assert_eq!(d.state, DamageState::Cracked);
    assert!(d.tension_crack > 0.5);
    // Cracked through: only the bars stretch, δ = N L / (Es As).
    let want = n * l / (200e9 * rc.steel_area);
    let got = sim.displacement(ids[2]).y;
    assert!(rel_err(got, want) < 0.03, "{got} vs {want}");

    let (snap, _) = run(1.1 * rc.tension_yield, 6.0);
    assert_eq!(snap.break_events()[0].cause, BreakCause::RebarRupture);

    // Plain concrete breaks when it cracks.
    let (s, ids) = vertical(l, 2, &Material::concrete(), &sec);
    let mut plain = DynamicSim::new(&s, quiet());
    ramp(
        &mut plain,
        ids[2],
        Vec3::new(0.0, 1.1 * rc.tension_cracking, 0.0),
        1.0,
        2.0,
    );
    assert_eq!(plain.break_events()[0].cause, BreakCause::Brittle);
}

#[test]
fn concrete_column_crushes_at_squash_load() {
    let mat = Material::reinforced_concrete();
    let sec = Section::rect(0.3, 0.3);
    let rc = RcSection::new(&sec, mat.youngs_modulus, concrete_props(&mat));
    let run = |ratio: f64, secs: f64| {
        let (s, ids) = vertical(1.0, 2, &mat, &sec);
        let mut sim = DynamicSim::new(&s, quiet());
        let mut crushed_intact = false;
        for f in 0..(secs * 60.0) as usize {
            let t = f as f64 / 60.0;
            let p = ratio * rc.squash_load * (t / 1.0).min(1.0);
            sim.set_force(ids[2], Vec3::new(0.0, -p, 0.0));
            sim.step(1.0 / 60.0);
            if sim.break_events().is_empty() && sim.damage(0).state == DamageState::Crushed {
                crushed_intact = true;
            }
        }
        (sim, crushed_intact)
    };
    let (safe, _) = run(0.9, 2.0);
    assert!(safe.break_events().is_empty());
    assert!(safe.damage(0).state < DamageState::Crushed);
    let (over, crushed) = run(1.05, 3.0);
    assert_eq!(over.break_events()[0].cause, BreakCause::Crushing);
    assert!(crushed, "crushing shows before the column gives way");
}

#[test]
fn short_rc_cantilever_without_links_fails_in_shear() {
    // 0.8 m long: flexure needs M_u / L ≈ 206 kN, shear gives out at ~105 kN.
    let (mat, sec, rc) = rc_beam();
    let v_rc = rc.axes[1].shear_capacity(0.0, rc.area, rc.fck);
    let run = |ratio: f64| {
        let (s, ids) = cantilever_of(0.8, 4, &mat, &sec);
        let mut sim = DynamicSim::new(&s, quiet());
        ramp(
            &mut sim,
            ids[4],
            Vec3::new(0.0, -ratio * v_rc, 0.0),
            1.0,
            2.5,
        );
        sim
    };
    assert!(run(0.9).break_events().is_empty());
    let ev = run(1.1).break_events().to_vec();
    assert_eq!(ev[0].cause, BreakCause::ConcreteShear);
    assert_eq!(ev[0].position, 0.0, "diagonal crack at the support");
}

// ------------------------------------------------------------------ timber

#[test]
fn timber_tie_is_much_weaker_off_grain() {
    let sec = Section::rect(0.05, 0.1);
    let n = 0.8 * 14.5e6 * sec.area;
    let run = |mat: Material| {
        let (s, ids) = vertical(1.0, 2, &mat, &sec);
        let mut sim = DynamicSim::new(&s, quiet());
        ramp(&mut sim, ids[2], Vec3::new(0.0, n, 0.0), 1.0, 2.0);
        sim
    };
    assert!(run(Material::timber()).break_events().is_empty());
    let off = run(Material::timber_at_grain_angle(15f64.to_radians()));
    assert_eq!(off.break_events()[0].cause, BreakCause::Splinter);
}

#[test]
fn short_deep_timber_beam_splits_along_the_grain() {
    // 100 x 300 mm, 0.3 m: shear capacity f_v A / 1.5 = 80 kN, bending 120 kN.
    let sec = Section::rect(0.1, 0.3);
    let vcap = 4e6 * sec.area / 1.5;
    let run = |ratio: f64| {
        let (s, ids) = cantilever_of(0.3, 3, &Material::timber(), &sec);
        let mut sim = DynamicSim::new(&s, quiet());
        ramp(
            &mut sim,
            ids[3],
            Vec3::new(0.0, -ratio * vcap, 0.0),
            1.0,
            2.0,
        );
        sim
    };
    assert!(run(0.9).break_events().is_empty());
    assert_eq!(run(1.1).break_events()[0].cause, BreakCause::TimberSplit);
}

#[test]
fn timber_post_crushes_gradually() {
    let sec = Section::rect(0.1, 0.1);
    let nc = 21e6 * sec.area;
    let (s, ids) = vertical(0.5, 2, &Material::timber(), &sec);
    let mut sim = DynamicSim::new(&s, quiet());
    let mut crushed_intact = false;
    for f in 0..(4.0 * 60.0) as usize {
        let t = f as f64 / 60.0;
        sim.set_force(ids[2], Vec3::new(0.0, -1.05 * nc * t.min(1.0), 0.0));
        sim.step(1.0 / 60.0);
        if sim.break_events().is_empty() && sim.damage(0).state == DamageState::Crushed {
            crushed_intact = true;
        }
    }
    assert!(crushed_intact, "crushing is visible before failure");
    assert_eq!(sim.break_events()[0].cause, BreakCause::Crushing);
}

#[test]
fn static_utilization_is_one_at_timber_bending_strength() {
    let sec = Section::rect(0.05, 0.1);
    let (s, ids) = cantilever_of(2.0, 4, &Material::timber(), &sec);
    let p = 24e6 * sec.wz / 2.0;
    let r = solve_static(&s, &[NodeLoad::force(ids[4], Vec3::new(0.0, -p, 0.0))]).unwrap();
    assert!(rel_err(r.member_forces[0].utilization, 1.0) < 1e-9);
}

// --------------------------------------------------------------- composite

#[test]
fn cfrp_is_linear_until_it_fails_suddenly() {
    // Compression fibres govern in bending: M = X_c W.
    let sec = Section::rect(0.05, 0.1);
    let l = 2.0;
    let p = 1400e6 * sec.wz / l;
    let run = |ratio: f64| {
        let (s, ids) = cantilever_of(l, 6, &Material::cfrp(), &sec);
        let mut sim = DynamicSim::new(&s, quiet());
        ramp(&mut sim, ids[6], Vec3::new(0.0, -ratio * p, 0.0), 1.0, 2.5);
        (sim, s, ids)
    };
    let (safe, s, ids) = run(0.6);
    assert!(safe.break_events().is_empty());
    assert_eq!(safe.damage(0).state, DamageState::Intact, "no warning");
    assert!(rel_err(safe.damage(0).utilization, 0.6) < 0.03);
    let stat = solve_static(
        &s,
        &[NodeLoad::force(ids[6], Vec3::new(0.0, -0.6 * p, 0.0))],
    )
    .unwrap();
    let (got, want) = (safe.displacement(ids[6]).y, stat.displacements[ids[6]].y);
    assert!(rel_err(got, want) < 0.02, "linear: {got} vs {want}");

    let (over, _, _) = run(1.1);
    let ev = over.break_events();
    assert_eq!((ev[0].member, ev[0].cause), (0, BreakCause::FibreRupture));
}

#[test]
fn cfrp_delaminates_in_interlaminar_shear() {
    // 50 x 100 mm. ILSS gives out at V = 70 MPa · A / 1.5 = 233 kN.
    let sec = Section::rect(0.05, 0.1);
    let v = 70e6 * sec.area / 1.5;
    let run = |l: f64, ratio: f64| {
        let (s, ids) = cantilever_of(l, 2, &Material::cfrp(), &sec);
        let mut sim = DynamicSim::new(&s, quiet());
        ramp(&mut sim, ids[2], Vec3::new(0.0, -ratio * v, 0.0), 1.0, 2.0);
        sim
    };
    assert_eq!(run(0.2, 0.9).damage(0).state, DamageState::Intact);
    // Short enough that the halved section still carries the moment.
    let held = run(0.2, 1.05);
    assert!(held.break_events().is_empty());
    let d = held.damage(0);
    assert_eq!(d.state, DamageState::Delaminated);
    assert_eq!(d.delamination, 0.5);
    assert!(d.stiffness <= 0.25 + 1e-12);
    // Longer: after delaminating, the halves break in bending.
    let snapped = run(0.3, 1.05);
    assert_eq!(snapped.break_events()[0].cause, BreakCause::FibreRupture);
}

#[test]
fn cfrp_laminate_loses_stiffness_to_matrix_cracking() {
    let sec = Section::tube(0.05, 0.005);
    let l = 1.0;
    let p = 500e6 * sec.wz / l;
    let run = |ratio: f64| {
        let (s, ids) = cantilever_of(l, 4, &Material::cfrp_laminate(), &sec);
        let mut sim = DynamicSim::new(&s, quiet());
        ramp(&mut sim, ids[4], Vec3::new(0.0, -ratio * p, 0.0), 1.0, 2.5);
        sim
    };
    assert_eq!(run(0.4).damage(0).state, DamageState::Intact);
    let cracked = run(0.8);
    assert!(cracked.break_events().is_empty());
    let d = cracked.damage(0);
    assert_eq!(d.state, DamageState::Cracked);
    assert!(d.stiffness < 0.99 && d.stiffness > 0.8, "{}", d.stiffness);
}

#[test]
fn grades_match_the_default_materials_and_order_by_strength() {
    use breakit_core::material::Model;
    use breakit_core::{ConcreteClass, SteelGrade, TimberClass};
    // The default constructors are one of the grades.
    let s355 = Material::steel_grade(SteelGrade::S355);
    assert_eq!(s355.model, Material::steel().model);
    let c24 = Material::timber_class(TimberClass::C24, 0.3);
    assert_eq!(c24.model, Material::timber_at_grain_angle(0.3).model);
    assert_eq!(
        c24.youngs_modulus,
        Material::timber_at_grain_angle(0.3).youngs_modulus
    );
    let c30 = Material::reinforced_concrete_class(ConcreteClass::C30, 0.01, 0.04);
    assert_eq!(c30.model, Material::reinforced_concrete().model);
    assert_eq!(c30.density, Material::reinforced_concrete().density);

    let fy = |g| match Material::steel_grade(g).model {
        Model::Ductile(p) => p.yield_strength,
        _ => unreachable!(),
    };
    assert!(
        fy(SteelGrade::S235) < fy(SteelGrade::S355) && fy(SteelGrade::S355) < fy(SteelGrade::S460)
    );
    let fm = |c| match Material::timber_class(c, 0.0).model {
        Model::Timber(t) => t.bending,
        _ => unreachable!(),
    };
    assert!(
        fm(TimberClass::C16) < fm(TimberClass::C24)
            && fm(TimberClass::C24) < fm(TimberClass::GL28h)
    );
    let fck = |c| match Material::concrete_class(c).model {
        Model::Concrete(p) => p.compression,
        _ => unreachable!(),
    };
    assert!(
        fck(ConcreteClass::C25) < fck(ConcreteClass::C30)
            && fck(ConcreteClass::C30) < fck(ConcreteClass::C40)
    );
}
