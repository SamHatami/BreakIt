//! The solver against wave theory: dispersion, the wavemaker, a solitary
//! wave, a dam break, shoaling, run-up, breaking, and the basics (rest
//! stays at rest, volume is conserved). Tolerances say how close the model
//! is, not how close floats are.

mod common;

use std::f64::consts::{PI, TAU};

use breakit_core::loads::water::wave_number;
use breakit_water::paddle::{model_phase_speed, model_wave_number};
use breakit_water::{Config, Grid, Program, Solver, WaterSim};
use common::*;

/// Period of the first seiche of a closed basin.
fn seiche_period(length: f64, depth: f64, dispersive: bool) -> f64 {
    let dx = 0.05;
    let mut s = Solver::new(
        Grid::covering((0.0, length), (0.0, dx), dx),
        Config {
            dispersive,
            manning: 0.0,
            ..Config::default()
        },
    );
    s.set_bed(|_, _| -depth);
    let k = PI / length;
    s.set_surface(|x, _| 0.002 * (k * x).cos());
    let mut gauge = vec![];
    while s.time() < 40.0 {
        s.advance(1.0 / 240.0);
        gauge.push((s.time(), s.surface()[0]));
    }
    let up = upcrossings(&gauge);
    assert!(up.len() >= 4, "no oscillation");
    (up[up.len() - 1] - up[0]) / (up.len() - 1) as f64
}

#[test]
fn seiche_period_follows_linear_dispersion() {
    for (length, depth) in [(12.0, 1.0), (6.0, 2.0), (4.0, 2.0), (3.0, 2.0)] {
        let k = PI / length;
        let kd: f64 = k * depth;
        let exact = TAU / (G * k * kd.tanh()).sqrt();
        let model = TAU / (k * model_phase_speed(k, depth, G));
        let got = seiche_period(length, depth, true);
        assert!(
            rel(got, model) < 0.005,
            "kd {kd:.2}: {got} vs model {model}"
        );
        assert!(rel(got, exact) < 0.03, "kd {kd:.2}: {got} vs Airy {exact}");
        // Without the non-hydrostatic pressure the shorter waves run far
        // too fast: this is what the pressure solve buys.
        if kd > 1.0 {
            let hydrostatic = seiche_period(length, depth, false);
            assert!(rel(hydrostatic, exact) > 0.12, "kd {kd:.2}");
        }
    }
}

#[test]
fn paddle_makes_waves_of_the_asked_height_and_speed() {
    for (depth, period, height) in [
        (2.0, 4.0, 0.1),
        (2.0, 3.0, 0.1),
        (1.0, 3.0, 0.05),
        (2.6, 3.0, 0.2),
    ] {
        // Long enough that nothing comes back from the far end in time.
        let mut sim = flume(160.0, depth, 0.1, false, None);
        let program = Program::Regular {
            height,
            period,
            ramp: 2.0 * period,
        };
        let omega = TAU / period;
        let k = model_wave_number(omega, depth, G).expect("below the cut-off");
        let c = model_phase_speed(k, depth, G);
        let (ia, ib) = (200, 230);
        let (mut a, mut b) = (vec![], vec![]);
        let t_end = 23.0 / c * 2.0 + 9.0 * period;
        assert!(
            t_end < (160.0 + 137.0) / (G * depth).sqrt(),
            "reflection would be back"
        );
        let mut t = 0.0;
        while t < t_end {
            sim.advance(DT, drive(&program, t, depth));
            t += DT;
            if t > t_end - 5.0 * period {
                a.push((t, sim.solver.surface()[ia]));
                b.push((t, sim.solver.surface()[ib]));
            }
        }
        let range = |s: &[(f64, f64)]| {
            s.iter().map(|p| p.1).fold(f64::MIN, f64::max)
                - s.iter().map(|p| p.1).fold(f64::MAX, f64::min)
        };
        let got = range(&a);
        assert!(
            rel(got, height) < 0.05,
            "d {depth} T {period}: height {got} vs {height}"
        );
        // Phase at each gauge from the Fourier component at the paddle
        // frequency over whole periods; the phase lag gives the wave number.
        let phase = |s: &[(f64, f64)]| {
            let n = (4.0 * period / DT).round() as usize;
            let (mut re, mut im) = (0.0, 0.0);
            for (t, z) in &s[s.len() - n..] {
                re += z * (omega * t).cos();
                im += z * (omega * t).sin();
            }
            im.atan2(re)
        };
        let lag = (phase(&b) - phase(&a)).rem_euclid(TAU);
        let speed = omega / (lag / ((ib - ia) as f64 * 0.1));
        let exact = omega / wave_number(omega, depth);
        assert!(
            rel(speed, c) < 0.02,
            "d {depth} T {period}: speed {speed} vs model {c}"
        );
        assert!(
            rel(speed, exact) < 0.05,
            "d {depth} T {period}: speed {speed} vs Airy {exact}"
        );
    }
}

#[test]
fn solitary_wave_keeps_its_shape_and_speed() {
    for (depth, height) in [(2.0, 0.4), (2.6, 0.8)] {
        let mut sim = flume(90.0, depth, 0.1, true, None);
        let program = Program::Solitary { height, at: 4.0 };
        let mut t = 0.0;
        let mut peaks = vec![];
        while t < 16.0 {
            sim.advance(DT, drive(&program, t, depth));
            t += DT;
            if t > 8.0 {
                let s = sim.solver.surface();
                let i = (0..s.len())
                    .max_by(|a, b| s[*a].total_cmp(&s[*b]))
                    .expect("cells");
                // Sub-cell crest position from a parabola through the top.
                let (l, m, r) = (s[i - 1], s[i], s[i + 1]);
                let x = (i as f64 + 0.5 * (l - r) / (l - 2.0 * m + r)) * 0.1;
                peaks.push((t, x, m));
            }
        }
        let (first, last) = (peaks[0], peaks[peaks.len() - 1]);
        let speed = (last.1 - first.1) / (last.0 - first.0);
        let theory = (G * (depth + height)).sqrt();
        assert!(rel(speed, theory) < 0.025, "speed {speed} vs {theory}");
        for p in &peaks {
            assert!(rel(p.2, height) < 0.12, "crest {} at t {}", p.2, p.0);
        }
        assert_eq!(sim.solver.stats().breaking_cells, 0);
    }
}

/// Stoker's dam break on a wet bed: (middle depth, bore speed).
fn stoker(h_left: f64, h_right: f64) -> (f64, f64) {
    let f = |hm: f64| {
        2.0 * ((G * h_left).sqrt() - (G * hm).sqrt())
            - (hm - h_right) * (G * (hm + h_right) / (2.0 * hm * h_right)).sqrt()
    };
    let (mut lo, mut hi) = (h_right, h_left);
    for _ in 0..80 {
        let mid = 0.5 * (lo + hi);
        if f(mid) > 0.0 {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    let hm = 0.5 * (lo + hi);
    let um = 2.0 * ((G * h_left).sqrt() - (G * hm).sqrt());
    (hm, hm * um / (hm - h_right))
}

#[test]
fn dam_break_bore_matches_stoker() {
    for dispersive in [false, true] {
        let dx = 0.1;
        let mut s = Solver::new(
            Grid::covering((0.0, 80.0), (0.0, dx), dx),
            Config {
                dispersive,
                manning: 0.0,
                ..Config::default()
            },
        );
        s.set_bed(|_, _| -1.0);
        s.set_surface(|x, _| if x < 40.0 { 1.0 } else { 0.0 });
        let (hm, speed) = stoker(2.0, 1.0);
        while s.time() < 5.0 {
            s.advance(DT);
        }
        let front = |s: &Solver| {
            let z = s.surface();
            let level = 0.5 * (hm - 1.0);
            let i = (400..800).rev().find(|i| z[*i] > level).expect("a bore");
            (i as f64 + (z[i] - level) / (z[i] - z[i + 1])) * dx
        };
        let x5 = front(&s);
        // The plateau between the fan and the bore. With dispersion the
        // front is an undular bore whose mean is still Stoker's level.
        let z = s.surface();
        let from = ((x5 - 9.0) / dx) as usize;
        let to = ((x5 - 3.0) / dx) as usize;
        let plateau = z[from..to].iter().sum::<f64>() / (to - from) as f64 + 1.0;
        while s.time() < 7.0 {
            s.advance(DT);
        }
        let got = (front(&s) - x5) / 2.0;
        assert!(
            rel(got, speed) < 0.03,
            "dispersive {dispersive}: bore {got} vs {speed}"
        );
        assert!(
            rel(plateau, hm) < 0.03,
            "dispersive {dispersive}: depth {plateau} vs {hm}"
        );
    }
}

#[test]
fn water_at_rest_stays_at_rest_and_volume_is_kept() {
    let dx = 0.1;
    let mut s = Solver::new(
        Grid::covering((0.0, 12.0), (0.0, 6.0), dx),
        Config::default(),
    );
    // A sloping beach, a submerged bar and an island.
    s.set_bed(|x, z| {
        let island = 0.9 * (-((x - 5.0).powi(2) + (z - 3.0).powi(2)) / 1.5).exp();
        let bar = if (2.0..2.6).contains(&x) { 0.5 } else { 0.0 };
        -1.2 + island + bar + (x - 8.0).max(0.0) * 0.5
    });
    s.fill_to(-0.5, |_, _| true);
    let v0 = s.volume();
    for _ in 0..240 {
        s.advance(DT);
    }
    let fastest = s
        .face_u()
        .iter()
        .chain(s.face_v())
        .fold(0.0_f64, |a, u| a.max(u.abs()));
    assert!(fastest < 1e-9, "still water moved at {fastest} m/s");
    // Now slosh it over the bar and up the beach.
    s.set_surface(|x, _| -0.5 + 0.4 * (-(x - 1.0).powi(2)).exp());
    let v1 = s.volume();
    let mut peak: f64 = 0.0;
    for _ in 0..1200 {
        s.advance(DT);
        peak = s.face_u().iter().fold(peak, |a, u| a.max(u.abs()));
        assert!(s.depth().iter().all(|h| *h >= 0.0 && h.is_finite()));
    }
    assert!(peak > 0.3, "the hump should have run out");
    assert!(
        rel(s.volume(), v1) < 1e-9,
        "volume {} -> {}",
        v1,
        s.volume()
    );
    assert!(rel(v1, v0) > 1e-3);
}

#[test]
fn long_waves_shoal_by_greens_law() {
    // From 2 m to 0.5 m over a gentle slope, long low waves.
    let (d0, d1) = (2.0, 0.5);
    let dx = 0.1;
    let mut solver = Solver::new(
        Grid::covering((0.0, 160.0), (0.0, dx), dx),
        Config {
            manning: 0.0,
            ..Config::default()
        },
    );
    solver.set_bed(|x, _| -d0 + ((x - 30.0) / 60.0).clamp(0.0, 1.0) * (d0 - d1));
    solver.fill_to(0.0, |_, _| true);
    solver.set_boundary(breakit_water::Side::XMin, breakit_water::Boundary::Piston);
    solver.set_sponge(|x, _| 2.0 * ((x - 130.0) / 30.0).max(0.0).powi(2));
    let mut sim = WaterSim::new(solver, false);
    let program = Program::Regular {
        height: 0.02,
        period: 8.0,
        ramp: 16.0,
    };
    let (mut lo, mut hi) = ([0.0_f64; 2], [0.0_f64; 2]);
    let mut t = 0.0;
    while t < 110.0 {
        sim.advance(DT, drive(&program, t, d0));
        t += DT;
        if t > 70.0 {
            for (k, i) in [200, 1100].into_iter().enumerate() {
                let z = sim.solver.surface()[i];
                lo[k] = lo[k].min(z);
                hi[k] = hi[k].max(z);
            }
        }
    }
    let ratio = (hi[1] - lo[1]) / (hi[0] - lo[0]);
    // Linear shoaling: H ∝ 1/√(group speed); Green's law (d0/d1)^¼ is its
    // long-wave limit.
    let omega = TAU / 8.0;
    let cg = |d: f64| {
        let k = wave_number(omega, d);
        0.5 * omega / k * (1.0 + 2.0 * k * d / (2.0 * k * d).sinh())
    };
    let linear = (cg(d0) / cg(d1)).sqrt();
    assert!(rel(ratio, linear) < 0.06, "shoaling {ratio} vs {linear}");
    assert!(rel(ratio, (d0 / d1).powf(0.25)) < 0.12);
}

#[test]
fn solitary_run_up_matches_synolakis() {
    // Synolakis (1987): non-breaking run-up on a plane beach,
    // R/d = 2.831 √(cot β) (H/d)^(5/4).
    let (d, cot, h) = (1.0, 19.85, 0.03);
    let dx = 0.05;
    let toe = 30.0;
    let mut sim = flume(toe + 1.4 * cot, d, dx, false, Some((toe, 1.0 / cot)));
    let program = Program::Solitary { height: h, at: 8.0 };
    let mut t = 0.0;
    let mut run_up = f64::MIN;
    let mut at_toe: f64 = 0.0;
    while t < 45.0 {
        sim.advance(DT, drive(&program, t, d));
        t += DT;
        let (bed, depth) = (sim.solver.bed(), sim.solver.depth());
        at_toe = at_toe.max(sim.solver.surface()[(25.0 / dx) as usize]);
        let wet_top = (0..bed.len())
            .rev()
            .find(|i| depth[*i] > 0.004)
            .expect("water");
        run_up = run_up.max(bed[wet_top] + depth[wet_top]);
    }
    // The wave that arrives is what counts; the paddle's is within 15 %.
    assert!(rel(at_toe, h) < 0.15, "wave at the toe {at_toe} vs {h}");
    let theory = d * 2.831 * cot.sqrt() * (at_toe / d).powf(1.25);
    assert!(rel(run_up, theory) < 0.1, "run-up {run_up} vs {theory}");
}

#[test]
fn waves_break_on_the_beach_and_lose_their_energy() {
    let depth = 2.0;
    let mut sim = flume(40.0, depth, 0.1, false, Some((20.0, 1.0 / 6.0)));
    let program = Program::Regular {
        height: 0.5,
        period: 3.5,
        ramp: 4.0,
    };
    let mut t = 0.0;
    let (mut most_breaking, mut crest_offshore, mut crest_surf, mut run_up) =
        (0, 0.0_f64, 0.0_f64, f64::MIN);
    let mut break_depth: f64 = 0.0;
    while t < 40.0 {
        let stats = sim.advance(DT, drive(&program, t, depth));
        t += DT;
        most_breaking = most_breaking.max(stats.breaking_cells);
        let s = &sim.solver;
        crest_offshore = crest_offshore.max(s.surface()[100]);
        for i in 200..400 {
            if s.depth()[i] > 0.01 {
                crest_surf = crest_surf.max(s.surface()[i]);
                run_up = run_up.max(s.surface()[i]);
            }
            if s.breaking()[i] {
                break_depth = break_depth.max(-s.bed()[i]);
            }
        }
    }
    assert!(most_breaking >= 3, "nothing broke");
    // Shoaling steepens the crest before it breaks...
    assert!(
        crest_surf > 1.25 * crest_offshore,
        "{crest_surf} vs {crest_offshore}"
    );
    // ...in water about as deep as the wave is high (H/d of 0.5 to 1.3).
    assert!(
        break_depth < 1.3 && break_depth > 0.0,
        "broke at depth {break_depth}"
    );
    // What arrives at the top of the beach is a fraction of what a
    // standing wave against a wall would reach.
    assert!(run_up > 0.05 && run_up < 0.8, "run-up {run_up}");
    assert!(
        sim.foam().iter().any(|f| *f > 0.5),
        "breaking should leave foam"
    );
    // The reflected wave is small: offshore crests stay near the incident
    // height instead of doubling.
    assert!(
        crest_offshore < 0.5 * 0.5 * 1.5,
        "offshore crest {crest_offshore}"
    );
}
