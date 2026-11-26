//! Earthquake loads against hand calculations and the static solve.
mod common;
use breakit_core::loads::seismic::{GroundMotion, Quake, Synthetic};
use breakit_core::loads::G;
use breakit_core::{
    solve_static, DynamicSim, LoadCase, LoadKind, LoadSet, Material, NodeLoad, SimConfig,
    Structure, Support, Vec3, Window,
};
use common::*;
use std::f64::consts::TAU;

fn mast(h: f64, segments: usize) -> Structure {
    let mut s = Structure::new();
    let ids = s.add_line(
        Vec3::zeros(),
        Vec3::new(0.0, h, 0.0),
        segments,
        &Material::steel(),
        &tube(),
    );
    s.add_support(Support::fixed(ids[0]));
    s
}

fn case(q: Quake) -> LoadSet {
    LoadSet::new(vec![LoadCase::new(
        "quake",
        Window::new(0.0, None, 0.0),
        LoadKind::Quake(q),
    )])
}

#[test]
fn base_shear_is_free_mass_times_ground_acceleration() {
    let s = mast(2.0, 8);
    let mut sim = DynamicSim::new(&s, SimConfig::default());
    let mut loads = case(Quake::new(GroundMotion::Harmonic {
        amplitude: 3.0,
        frequency: 2.0,
        duration: 5.0,
        direction: Vec3::x(),
    }));
    // Step to a known time, then compare the applied force with −M a_g.
    for _ in 0..40 {
        loads.step(&s, &mut sim, 1.0 / 60.0);
    }
    loads.apply(&s, &mut sim, 1.0 / 60.0);
    let LoadKind::Quake(q) = &loads.cases[0].kind else {
        unreachable!()
    };
    let a = q.acceleration(sim.time());
    let free_mass: f64 = (0..s.nodes.len()).map(|n| sim.node_mass(n)).sum();
    let total_mass = 7850.0 * tube().area * 2.0;
    // The support node keeps half of the bottom member's mass.
    assert!(rel_err(free_mass, total_mass * (1.0 - 1.0 / 16.0)) < 1e-9);
    let shear = loads.totals()[0];
    assert!(a.x.abs() > 1.0, "{a:?}");
    assert!(
        (shear + a * free_mass).norm() < 1e-9 * free_mass,
        "{shear:?}"
    );
}

#[test]
fn slow_shaking_of_a_stiff_mast_matches_the_static_inertia_load() {
    // 2 m steel tube mast, first mode about 24 Hz. Shaken at 1 Hz the
    // dynamic amplification is 1/(1 − (1/24)²) ≈ 1.002, so the peak relative
    // tip displacement is the static deflection under −m a_peak.
    let s = mast(2.0, 10);
    let tip = s.nodes.len() - 1;
    let amp = 0.5 * G;
    let mut loads = case(Quake::new(GroundMotion::Harmonic {
        amplitude: amp,
        frequency: 1.0,
        duration: 6.0,
        direction: Vec3::x(),
    }));
    let cfg = SimConfig {
        gravity: Vec3::zeros(),
        damping: 2.0,
        plasticity: false,
        ..Default::default()
    };
    let mut sim = DynamicSim::new(&s, cfg);
    let mut peak: f64 = 0.0;
    for _ in 0..(6 * 60) {
        loads.step(&s, &mut sim, 1.0 / 60.0);
        peak = peak.max(sim.displacement(tip).x.abs());
    }
    let loads_static: Vec<NodeLoad> = (0..s.nodes.len())
        .map(|n| NodeLoad::force(n, Vec3::new(-sim.node_mass(n) * amp, 0.0, 0.0)))
        .collect();
    let stat = solve_static(&s, &loads_static).unwrap();
    let e = stat.displacements[tip].x.abs();
    assert!(rel_err(peak, e) < 0.03, "peak {peak:e} vs static {e:e}");
}

#[test]
fn response_spectrum_of_a_sine_matches_the_sdof_steady_state() {
    // Ground a = A sin ωt, oscillator at ω_n = 2ω, ζ = 5 %: relative
    // displacement amplitude A / (ω_n² √((1 − r²)² + (2ζr)²)), r = 0.5.
    let (a, f) = (2.0, 1.0);
    let mut q = Quake::new(GroundMotion::Harmonic {
        amplitude: a,
        frequency: f,
        duration: 30.0,
        direction: Vec3::x(),
    });
    let zeta = 0.05;
    let psa = q.response_spectrum(&[0.5], zeta)[0];
    let r: f64 = 0.5;
    let expected = a / ((1.0 - r * r).powi(2) + (2.0 * zeta * r).powi(2)).sqrt();
    assert!(rel_err(psa, expected) < 0.03, "{psa} vs {expected}");
}

#[test]
fn synthetic_quake_hits_its_target_and_replays_from_its_seed() {
    let make = |seed| {
        let mut q = Quake::new(GroundMotion::Synthetic(Synthetic {
            pga: 0.35 * G,
            duration: 8.0,
            ground_frequency: 2.5,
            cross_ratio: 0.85,
            seed,
            ..Default::default()
        }));
        q.prepare();
        q
    };
    let (q1, q2, q3) = (make(4), make(4), make(5));
    let main = q1.samples().1.iter().map(|a| a.x.abs()).fold(0.0, f64::max);
    assert!(rel_err(main, 0.35 * G) < 1e-9, "main PGA {main}");
    let cross = q1.samples().1.iter().map(|a| a.z.abs()).fold(0.0, f64::max);
    assert!(
        cross > 0.5 * 0.35 * G && cross < 1.3 * 0.35 * G,
        "cross {cross}"
    );
    assert_eq!(q1.samples().1, q2.samples().1);
    assert_ne!(q1.samples().1, q3.samples().1);
    assert!(
        q1.duration() > 8.0 && q1.duration() < 16.0,
        "{}",
        q1.duration()
    );
}

#[test]
fn synthetic_spectrum_peaks_near_the_ground_period() {
    let mut q = Quake::new(GroundMotion::Synthetic(Synthetic {
        pga: 0.3 * G,
        duration: 15.0,
        ground_frequency: 2.5,
        seed: 11,
        ..Default::default()
    }));
    let periods: Vec<f64> = (1..=40).map(|i| 0.05 * i as f64).collect();
    let sa = q.response_spectrum(&periods, 0.05);
    let (imax, smax) = sa
        .iter()
        .enumerate()
        .fold((0, 0.0), |b, (i, &v)| if v > b.1 { (i, v) } else { b });
    let tp = periods[imax];
    assert!((0.15..=0.8).contains(&tp), "peak at {tp} s");
    // Typical amplification of a 5 % damped spectrum: 2 to 3.5 times PGA.
    let amp = smax / (0.3 * G);
    assert!((1.8..=4.0).contains(&amp), "amplification {amp}");
}

#[test]
fn table_motion_is_bounded_and_consistent_with_its_acceleration() {
    let mut q = Quake::new(GroundMotion::Synthetic(Synthetic {
        pga: 0.4 * G,
        duration: 10.0,
        seed: 2,
        ..Default::default()
    }));
    q.prepare();
    let (dt, acc) = q.samples();
    let n = acc.len();
    let d: Vec<f64> = (0..n)
        .map(|i| q.table(i as f64 * dt, 1.0).displacement.x)
        .collect();
    let peak_d = d.iter().fold(0.0f64, |m, v| m.max(v.abs()));
    // A 0.4 g, 2.5 Hz quake moves the ground by centimetres, not metres.
    assert!(
        peak_d > 0.005 && peak_d < 0.5,
        "peak table displacement {peak_d}"
    );
    assert_eq!(q.table(0.0, 1.0).displacement, Vec3::zeros());
    // d is the exact double integral of the piecewise linear a, so its
    // second difference is (a[i-1] + 4 a[i] + a[i+1]) / 6 (rms over the record).
    let (mut err, mut norm) = (0.0, 0.0);
    for i in 1..n - 1 {
        let dd = (d[i + 1] - 2.0 * d[i] + d[i - 1]) / (dt * dt);
        let a = (acc[i - 1].x + 4.0 * acc[i].x + acc[i + 1].x) / 6.0;
        err += (dd - a).powi(2);
        norm += acc[i].x.powi(2);
    }
    assert!((err / norm).sqrt() < 1e-6, "{}", (err / norm).sqrt());
}

#[test]
fn peer_at2_records_parse() {
    let text = "PEER NGA STRONG MOTION DATABASE RECORD\n\
                TEST 1940, STATION 0, 180\n\
                ACCELERATION TIME SERIES IN UNITS OF G\n\
                NPTS=    5, DT=   .0200 SEC\n\
                0.0 0.1 -0.2\n 0.05  0.0\n";
    let GroundMotion::Record { dt, accel } = GroundMotion::from_peer_at2(text, Vec3::z()).unwrap()
    else {
        panic!()
    };
    assert_eq!(dt, 0.02);
    assert_eq!(accel.len(), 5);
    assert!((accel[2].z + 0.2 * G).abs() < 1e-12);
    let _ = TAU;
}
