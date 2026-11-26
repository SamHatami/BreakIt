//! Unsteady wind: turbulence statistics, vortex shedding and lock-in,
//! galloping, against textbook results.
mod common;
use breakit_core::loads::wind::shedding::WakeOscillator;
use breakit_core::loads::wind::{galloping_coefficient, Gusts, Wind, AIR_DENSITY, GALLOPING_A1};
use breakit_core::{
    DynamicSim, LoadCase, LoadKind, LoadSet, Material, Section, SimConfig, Structure, Support,
    Vec3, Window,
};
use std::f64::consts::{PI, TAU};

fn steady(wind: Wind) -> LoadSet {
    LoadSet::new(vec![LoadCase::new(
        "wind",
        Window::new(0.0, None, 0.0),
        LoadKind::Wind(wind),
    )])
}

fn wind_of(loads: &LoadSet) -> &Wind {
    match &loads.cases[0].kind {
        LoadKind::Wind(w) => w,
        _ => unreachable!("case 0 is wind"),
    }
}

fn horizontal(section: &Section, length: f64, segments: usize) -> Structure {
    let mut s = Structure::new();
    let ids = s.add_line(
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(0.0, 1.0, length),
        segments,
        &Material::steel(),
        section,
    );
    s.add_support(Support::fixed(ids[0]));
    s.add_support(Support::fixed(*ids.last().unwrap()));
    s
}

fn still() -> SimConfig {
    SimConfig {
        gravity: Vec3::zeros(),
        plasticity: false,
        ..Default::default()
    }
}

#[test]
fn shedding_frequency_follows_the_strouhal_number() {
    // EN 1991-1-4 Table E.1: tube D = 0.4 m at 2 m/s, St 0.18, 0.9 Hz;
    // square 0.2 m at 5 m/s, St 0.12, 3 Hz.
    for (sec, u, f) in [
        (Section::tube(0.2, 0.01), 2.0, 0.9),
        (Section::rect(0.2, 0.2), 5.0, 3.0),
    ] {
        let s = horizontal(&sec, 2.0, 2);
        let mut loads = steady(Wind::new(u, Vec3::x()));
        let mut sim = DynamicSim::new(&s, still());
        loads.apply(&s, &mut sim, 1.0 / 60.0);
        let fs = wind_of(&loads)
            .member_aero()
            .next()
            .unwrap()
            .shedding_frequency;
        assert!((fs - f).abs() < 1e-9, "{fs} vs {f}");
    }
}

#[test]
fn fixed_cylinder_sheds_with_its_lift_amplitude_and_frequency() {
    // A fixed 0.4 m tube at 2 m/s: C_L swings by C_L0 = 0.3 at 0.9 Hz.
    let s = horizontal(&Section::tube(0.2, 0.01), 1.0, 1);
    let mut loads = steady(Wind::new(2.0, Vec3::x()));
    let mut sim = DynamicSim::new(&s, still());
    let mut cl = Vec::new();
    for f in 0..(20 * 60) {
        loads.step(&s, &mut sim, 1.0 / 60.0);
        if f >= 10 * 60 {
            cl.push(
                wind_of(&loads)
                    .member_aero()
                    .next()
                    .unwrap()
                    .lift_coefficient,
            );
        }
    }
    let peak = cl.iter().fold(0.0f64, |m, c| m.max(c.abs()));
    assert!((peak - 0.3).abs() < 0.02, "C_L amplitude {peak}");
    let crossings = cl
        .windows(2)
        .filter(|w| w[0].signum() != w[1].signum())
        .count();
    assert!(
        (17..=19).contains(&crossings),
        "{crossings} crossings in 10 s"
    );
}

/// Cross-wind response of a spring-mounted cylinder (per unit length) with
/// the wake oscillator at St = 0.2, at reduced velocity `ur` = U / (f_n D). Returns the
/// steady amplitude over D and the response frequency over f_n.
fn sdof(ur: f64) -> (f64, f64) {
    let (d, m, zeta, fnat) = (0.1, 1.0, 0.0005, 2.0);
    let wn = TAU * fnat;
    let u = ur * fnat * d;
    let omega = TAU * 0.2 * u / d;
    let (mut y, mut yd, mut ydd) = (0.0f64, 0.0f64, 0.0);
    let mut w = WakeOscillator::default();
    let h = 1.0 / 2000.0;
    let steps = (120.0 / h) as usize;
    let (mut peak, mut cross, mut last) = (0.0f64, 0, 0.0f64);
    for i in 0..steps {
        w.advance(h, omega, ydd, d);
        // Shedding lift (C_L0 / 2) q plus quasi-steady drag damping, as in
        // the member model.
        let f = 0.5 * AIR_DENSITY * u * u * d * 0.15 * w.q - 0.5 * AIR_DENSITY * u * d * 1.2 * yd;
        ydd = (f - 2.0 * zeta * wn * m * yd - wn * wn * m * y) / m;
        yd += h * ydd;
        y += h * yd;
        if i > steps / 2 {
            peak = peak.max(y.abs());
            if last.signum() != y.signum() {
                cross += 1;
            }
            last = y;
        }
    }
    (peak / d, cross as f64 / 2.0 / 60.0 / fnat)
}

#[test]
fn spring_mounted_cylinder_locks_in_near_the_strouhal_resonance() {
    // Lock-in: around U_r = 1 / St = 5 the cylinder vibrates at its own
    // natural frequency with an amplitude of a sizeable fraction of D
    // (Griffin plot range for Scruton ≈ 0.5); away from it the shedding
    // runs at St U / D and the motion is tiny.
    let mut locked = Vec::new();
    for ur in [3.0, 4.0, 5.0, 5.5, 7.0, 10.0] {
        let (a, f) = sdof(ur);
        if a > 0.05 {
            assert!((f - 1.0).abs() < 0.02, "locked at U_r {ur} but f/fn {f}");
            assert!(a < 1.5, "amplitude must self-limit: {a}");
            locked.push(ur);
        } else {
            // Free shedding: the motion follows f_s = St U / D = 0.2 U_r f_n.
            assert!((f - 0.2 * ur).abs() < 0.05, "U_r {ur}: f/fn {f}");
            assert!(a < 0.01, "U_r {ur}: A/D {a}");
        }
    }
    assert_eq!(locked, vec![5.0, 5.5], "lock-in band");
}

/// Damping ratio and frequency of a structure's free cross-wind (z)
/// vibration at `tip` in the solver, from the log decrement. The implicit
/// integrator adds numerical damping of about ω h / 2 per substep h, so the
/// aeroelastic checks use the damping the solver really has.
fn free_decay(s: &Structure, tip: usize, cfg: &SimConfig) -> (f64, f64) {
    let mut sim = DynamicSim::new(s, cfg.clone());
    sim.set_force(tip, Vec3::new(0.0, 0.0, 5.0));
    for _ in 0..30 {
        sim.step(1.0 / 60.0);
    }
    sim.set_force(tip, Vec3::zeros());
    let (mut peaks, mut times) = (Vec::new(), Vec::new());
    let (mut last, mut rising) = (0.0f64, false);
    for f in 0..(30 * 60) {
        sim.step(1.0 / 60.0);
        let z = sim.displacement(tip).z;
        if rising && z < last {
            peaks.push(last);
            times.push(f as f64 / 60.0);
        }
        rising = z > last;
        last = z;
    }
    let n = peaks.len().min(8);
    assert!(n >= 3, "too few cycles to measure damping");
    let delta = (peaks[0] / peaks[n - 1]).ln() / (n - 1) as f64;
    let f = (n - 1) as f64 / (times[n - 1] - times[0]);
    (delta / TAU, f)
}

/// Light, slender tube mast: first mode `f1 = 3.516 / (2π L²) √(EI/m)`.
fn mast(length: f64) -> (Structure, usize, f64, f64) {
    let sec = Section::tube(0.05, 0.001);
    let mat = Material::steel();
    let mut s = Structure::new();
    let ids = s.add_line(Vec3::zeros(), Vec3::new(0.0, length, 0.0), 12, &mat, &sec);
    s.add_support(Support::fixed(ids[0]));
    let m = mat.density * sec.area;
    let f1 = 3.516 / (TAU * length * length) * (mat.youngs_modulus * sec.iz / m).sqrt();
    (s, *ids.last().unwrap(), f1, 0.1)
}

/// Solver settings for the mast: no added damping, 16 substeps to keep the
/// integrator's numerical damping near a real steel mast's (0.3 %).
fn mast_config() -> SimConfig {
    SimConfig {
        damping: 0.0,
        substeps: 16,
        ..still()
    }
}

/// Steady cross-wind tip amplitude of the mast in wind `u`, m.
fn mast_response(u: f64) -> (f64, f64) {
    let (s, tip, _, _) = mast(10.0);
    let mut w = Wind::new(u, Vec3::x());
    w.galloping = false;
    let mut loads = steady(w);
    let mut sim = DynamicSim::new(&s, mast_config());
    let (mut peak, mut cross, mut last) = (0.0f64, 0, 0.0f64);
    let frames = 60 * 60;
    for f in 0..frames {
        loads.step(&s, &mut sim, 1.0 / 60.0);
        if f >= frames - 20 * 60 {
            let z = sim.displacement(tip).z;
            peak = peak.max(z.abs());
            if last.signum() != z.signum() {
                cross += 1;
            }
            last = z;
        }
    }
    (peak, cross as f64 / 2.0 / 20.0)
}

#[test]
fn slender_mast_locks_in_at_its_critical_wind_speed() {
    // EN 1991-1-4 E.1.3.1: v_crit = f1 D / St, St = 0.18. A 10 m, 100 mm
    // steel tube with 1 mm wall: f1 ≈ 1 Hz, v_crit ≈ 0.55 m/s.
    let (s, tip, f1, d) = mast(10.0);
    let (zeta, f_sim) = free_decay(&s, tip, &mast_config());
    assert!(
        (f_sim / f1 - 1.0).abs() < 0.03,
        "solver f1 {f_sim} vs beam theory {f1}"
    );
    let v_crit = f1 * d / 0.18;
    let (at, f_at) = mast_response(1.05 * v_crit);
    let (below, _) = mast_response(0.6 * v_crit);
    let (above, _) = mast_response(1.6 * v_crit);
    // EN-style resonance estimate for a uniform member, without the
    // aerodynamic damping and mode shape: y/D = C_lat / (4π St² Sc).
    let m = 7850.0 * Section::tube(0.05, 0.001).area;
    let sc = 4.0 * PI * zeta * m / (AIR_DENSITY * d * d);
    let estimate = 0.3 / (4.0 * PI * 0.18 * 0.18 * sc) * d;
    assert!(
        at > 0.2 * estimate && at < 3.0 * estimate,
        "lock-in {at} m vs estimate {estimate} m (Sc {sc:.1})"
    );
    assert!(
        at > 4.0 * below && at > 4.0 * above,
        "{below} / {at} / {above}"
    );
    assert!(
        (f_at / f1 - 1.0).abs() < 0.05,
        "responds at {f_at} Hz, f1 {f1} Hz"
    );
}

#[test]
fn galloping_coefficient_has_parkinson_slope_and_den_hartog_sign() {
    let h = 1e-6;
    let slope = (galloping_coefficient(h) - galloping_coefficient(0.0)) / h;
    assert!((slope - GALLOPING_A1).abs() < 1e-3, "{slope}");
    assert!(galloping_coefficient(0.1) > 0.0);
    assert!(
        galloping_coefficient(0.3) < 0.0,
        "restoring at large angles"
    );
}

fn gallop_config() -> SimConfig {
    SimConfig {
        damping: 0.0,
        substeps: 16,
        ..still()
    }
}

fn timber_cantilever() -> (Structure, usize) {
    let mut s = Structure::new();
    let ids = s.add_line(
        Vec3::zeros(),
        Vec3::new(0.0, 6.0, 0.0),
        12,
        &Material::timber().elastic_only(),
        &Section::rect(0.1, 0.1),
    );
    s.add_support(Support::fixed(ids[0]));
    (s, ids[12])
}

/// Square timber cantilever, kicked sideways, in wind `u`; ratio of the
/// late to the early cross-wind amplitude.
fn galloping_growth(u: f64) -> f64 {
    let (s, tip) = timber_cantilever();
    let mut w = Wind::new(u, Vec3::x());
    w.vortex_shedding = false;
    let mut loads = steady(w);
    let mut sim = DynamicSim::new(&s, gallop_config());
    let (mut early, mut late) = (0.0f64, 0.0f64);
    for f in 0..(40 * 60) {
        loads.apply(&s, &mut sim, 1.0 / 60.0);
        if f < 6 {
            // A short, small sideways push: well inside the galloping limit
            // cycle, so it grows above onset.
            let mut forces = loads.forces().to_vec();
            forces[tip].z += 5.0;
            for (n, fz) in forces.iter().enumerate() {
                sim.set_force(n, *fz);
            }
        }
        sim.step(1.0 / 60.0);
        let z = sim.displacement(tip).z.abs();
        if (5 * 60..10 * 60).contains(&f) {
            early = early.max(z);
        }
        if f >= 35 * 60 {
            late = late.max(z);
        }
    }
    late / early
}

#[test]
fn square_cantilever_gallops_above_the_den_hartog_speed_only() {
    // Den Hartog onset: U_g = 4 m ζ ω / (ρ b A₁), with ζ the damping the
    // solver really has (measured by free decay).
    let mat = Material::timber();
    let sec = Section::rect(0.1, 0.1);
    let m = mat.density * sec.area;
    let (s, tip) = timber_cantilever();
    let (zeta, f) = free_decay(&s, tip, &gallop_config());
    let ug = 4.0 * m * zeta * TAU * f / (AIR_DENSITY * 0.1 * GALLOPING_A1);
    let below = galloping_growth(0.6 * ug);
    let above = galloping_growth(1.6 * ug);
    assert!(below < 0.7, "decays below onset ({ug:.2} m/s): {below}");
    assert!(above > 1.5, "grows above onset ({ug:.2} m/s): {above}");
}

#[test]
fn turbulence_has_atmospheric_intensities_and_spatial_coherence() {
    let mut s = Structure::new();
    s.add_line(
        Vec3::new(0.0, 10.0, 0.0),
        Vec3::new(1.0, 10.0, 0.0),
        1,
        &Material::steel(),
        &Section::tube(0.05, 0.005),
    );
    let (u, l, i) = (20.0, 30.0, 0.15);
    let mut w = Wind::new(u, Vec3::x());
    w.gusts = Some(Gusts {
        intensity: i,
        length_scale: l,
        seed: 5,
    });
    w.prepare(&s);
    let p = Vec3::new(0.0, 10.0, 0.0);
    let dt = 0.05;
    let n = 40_000;
    let at = |q: Vec3| -> Vec<Vec3> { (0..n).map(|k| w.velocity(q, k as f64 * dt, 1.0)).collect() };
    let base = at(p);
    let stats = |a: &[Vec3], c: usize| {
        let m = a.iter().map(|v| v[c]).sum::<f64>() / n as f64;
        let s = (a.iter().map(|v| (v[c] - m).powi(2)).sum::<f64>() / n as f64).sqrt();
        (m, s)
    };
    let (mu, su) = stats(&base, 0);
    let (_, sv) = stats(&base, 2);
    let (_, sw) = stats(&base, 1);
    assert!((mu - u).abs() < 0.03 * u, "mean {mu}");
    // One record is a finite sample of the field: ±20 %.
    assert!((su / (i * u) - 1.0).abs() < 0.2, "σ_u {su}");
    assert!((sv / su - 0.75).abs() < 0.2, "σ_v/σ_u {}", sv / su);
    assert!((sw / su - 0.5).abs() < 0.15, "σ_w/σ_u {}", sw / su);
    // Correlation of u between the point and one across the wind.
    let corr = |off: f64| {
        let other = at(p + Vec3::new(0.0, 0.0, off));
        let (m2, s2) = stats(&other, 0);
        base.iter()
            .zip(&other)
            .map(|(a, b)| (a.x - mu) * (b.x - m2))
            .sum::<f64>()
            / n as f64
            / (su * s2)
    };
    let (c1, c2, c3) = (corr(0.05 * l), corr(0.5 * l), corr(2.0 * l));
    assert!(c1 > 0.85, "close points move together: {c1}");
    assert!(
        c2 < c1 && c3 < c2,
        "coherence falls with distance: {c1} {c2} {c3}"
    );
    assert!(c3.abs() < 0.3, "far points are unrelated: {c3}");
}
