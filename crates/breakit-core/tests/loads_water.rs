//! Water loads checked against hand calculations and the static solve.
mod common;
use breakit_core::loads::water::{piston_height_to_stroke, wave_number, Water};
use breakit_core::loads::{Geometry, View};
use breakit_core::{
    solve_static, DynamicSim, LoadCase, LoadKind, LoadSet, Material, NodeLoad, Section, SimConfig,
    Structure, Support, Vec3, Window,
};
use common::*;
use std::f64::consts::PI;

const G: f64 = 9.81;
const RHO: f64 = 1000.0;

fn quiet() -> SimConfig {
    SimConfig {
        gravity: Vec3::zeros(),
        damping: 8.0,
        plasticity: false,
        ..Default::default()
    }
}

/// No ramp: the case is at full strength from t = 0.
fn instant(w: Water) -> LoadSet {
    LoadSet::new(vec![LoadCase::new(
        "Water",
        Window::new(0.0, None, 0.0),
        LoadKind::Water(w),
    )])
}

/// Net water force on `s` at rest, at t = 0.
fn total_force(s: &Structure, w: Water) -> Vec3 {
    let mut loads = instant(w);
    let mut sim = DynamicSim::new(s, quiet());
    loads.apply(s, &mut sim, 1.0 / 60.0);
    loads.totals()[0]
}

fn column(height: f64, segments: usize, sec: &Section) -> (Structure, Vec<usize>) {
    let mut s = Structure::new();
    let ids = s.add_line(
        Vec3::zeros(),
        Vec3::new(0.0, height, 0.0),
        segments,
        &Material::steel(),
        sec,
    );
    s.add_support(Support::fixed(ids[0]));
    (s, ids)
}

#[test]
fn dispersion_limits() {
    // Deep: L = g T² / 2π.
    let t = 1.0;
    let k = wave_number(2.0 * PI / t, 50.0);
    let (l, e) = (2.0 * PI / k, G * t * t / (2.0 * PI));
    println!("deep L {l} vs {e}");
    assert!(rel_err(l, e) < 1e-9);
    // Shallow: L = T √(g d), kd ≈ 0.03 so the tanh correction is ~kd²/6.
    let (t, d) = (20.0, 0.1);
    let k = wave_number(2.0 * PI / t, d);
    let (l, e) = (2.0 * PI / k, t * (G * d).sqrt());
    println!("shallow L {l} vs {e}");
    assert!(rel_err(l, e) < 1e-3);
    // Intermediate: residual of the dispersion relation.
    let (w, d) = (2.0 * PI / 1.5, 0.5);
    let k = wave_number(w, d);
    assert!((G * k * (k * d).tanh() - w * w).abs() < 1e-10);
}

#[test]
fn biesel_limits() {
    let deep = piston_height_to_stroke(20.0);
    println!("deep H/S {deep}");
    assert!((deep - 2.0).abs() < 1e-9);
    let huge = piston_height_to_stroke(1000.0);
    assert!((huge - 2.0).abs() < 1e-12);
    let kd = 1e-3;
    let shallow = piston_height_to_stroke(kd);
    println!("shallow H/S {shallow} vs kd {kd}");
    assert!(rel_err(shallow, kd) < 1e-3);
}

#[test]
fn buoyancy_of_submerged_horizontal_tube() {
    let sec = tube();
    let (ro, l) = (0.05, 2.0);
    let mut s = Structure::new();
    let ids = s.add_line(
        Vec3::new(0.0, 0.5, 0.0),
        Vec3::new(l, 0.5, 0.0),
        4,
        &Material::steel(),
        &sec,
    );
    s.add_support(Support::fixed(ids[0]));
    let f = total_force(&s, Water::current(1.0, Vec3::zeros()));
    let e = RHO * G * PI * ro * ro * l;
    println!("buoyancy {} vs {e}", f.y);
    assert!(rel_err(f.y, e) < 1e-12);
    assert!(f.x.abs() < 1e-9 && f.z.abs() < 1e-9);

    // Flooded tube: only the steel displaces water.
    let mut w = Water::current(1.0, Vec3::zeros());
    w.sealed = false;
    let f = total_force(&s, w);
    let e = RHO * G * sec.area * l;
    assert!(rel_err(f.y, e) < 1e-12);
}

#[test]
fn buoyancy_of_half_submerged_vertical_tube() {
    let ro = 0.05;
    for segments in [1, 4] {
        let (s, _) = column(1.0, segments, &tube());
        let f = total_force(&s, Water::current(0.5, Vec3::zeros()));
        let e = RHO * G * PI * ro * ro * 0.5;
        println!("{segments} segments: buoyancy {} vs {e}", f.y);
        assert!(rel_err(f.y, e) < 1e-12);
    }
}

#[test]
fn current_drag_on_submerged_column() {
    let (h, u) = (1.0, 0.5);
    // Tube: Cd 1, D = 2 ro.
    let (s, _) = column(h, 4, &tube());
    let f = total_force(&s, Water::current(2.0, Vec3::new(u, 0.0, 0.0)));
    let e = 0.5 * RHO * 1.0 * 0.1 * h * u * u;
    println!("tube drag {} vs {e}", f.x);
    assert!(rel_err(f.x, e) < 1e-12);
    assert!(f.z.abs() < 1e-9);
    // Square: Cd 2, D = b, flow along z this time.
    let b = 0.08;
    let (s, _) = column(h, 4, &Section::rect(b, b));
    let f = total_force(&s, Water::current(2.0, Vec3::new(0.0, 0.0, u)));
    let e = 0.5 * RHO * 2.0 * b * h * u * u;
    println!("square drag {} vs {e}", f.z);
    assert!(rel_err(f.z, e) < 1e-12);
    // A rectangle presents its b face to flow along local y and its h face
    // to flow along local z. For a vertical column local y is world x.
    let (bb, hh) = (0.05, 0.12);
    let (s, _) = column(h, 4, &Section::rect(bb, hh));
    let fx = total_force(&s, Water::current(2.0, Vec3::new(u, 0.0, 0.0))).x;
    let fz = total_force(&s, Water::current(2.0, Vec3::new(0.0, 0.0, u))).z;
    let (ex, ez) = (
        0.5 * RHO * 2.0 * h * u * u * bb,
        0.5 * RHO * 2.0 * h * u * u * hh,
    );
    println!("rect drag x {fx} vs {ex}, z {fz} vs {ez}");
    assert!(rel_err(fx, ex) < 1e-12 && rel_err(fz, ez) < 1e-12);
}

#[test]
fn drag_damps_a_moving_member() {
    // Still water, a column moving through it: drag opposes the motion.
    let (s, ids) = column(1.0, 4, &tube());
    let mut loads = instant(Water::current(2.0, Vec3::zeros()));
    let mut sim = DynamicSim::new(&s, quiet());
    sim.set_force(*ids.last().unwrap(), Vec3::new(500.0, 0.0, 0.0));
    for _ in 0..3 {
        sim.step(1.0 / 60.0);
    }
    let vx = sim.velocity(*ids.last().unwrap()).x;
    assert!(vx > 0.0);
    loads.apply(&s, &mut sim, 1.0 / 60.0);
    assert!(loads.totals()[0].x < 0.0, "{}", loads.totals()[0].x);
}

#[test]
fn airy_inertia_on_small_cylinder() {
    let (d, h, t) = (0.5, 0.005, 1.5);
    let ro = 0.01;
    let sec = Section::tube(ro, 0.002);
    let (s, _) = column(d, 20, &sec);
    let mut w = Water::airy(d, h, t, Vec3::x());
    w.coefficients.tube_cd = 0.0;
    w.samples = 16;
    let omega = 2.0 * PI / t;
    let k = wave_number(omega, d);
    let sim = DynamicSim::new(&s, quiet());
    let geometry = Geometry::new(&s);
    // At t = T/4 the phase at x = 0 is −π/2: η = 0 at the column (the
    // surface is at the still level) and the acceleration is at its peak.
    let tq = t / 4.0;
    let view = View {
        structure: &s,
        sim: &sim,
        geometry: &geometry,
        time: tq,
    };
    let mut out = vec![Vec3::zeros(); s.nodes.len()];
    w.apply(&view, tq, 1.0, 1.0, 1.0 / 60.0, &mut out);
    let f: Vec3 = out.iter().sum();
    let e = RHO * 2.0 * PI * ro * ro * (h / 2.0) * omega * omega / k;
    println!("inertia {} vs {e} (kd {})", f.x, k * d);
    assert!(rel_err(-f.x, e) < 0.01);
}

#[test]
fn airy_surface_and_paddle() {
    let (d, h, t) = (0.4, 0.05, 1.2);
    let w = Water::airy(d, h, t, Vec3::x());
    // Crest at the origin at t = 0, trough half a period later.
    assert!((w.surface_elevation(0.0, 0.0, 0.0) - (d + h / 2.0)).abs() < 1e-12);
    assert!((w.surface_elevation(0.0, 0.0, t / 2.0) - (d - h / 2.0)).abs() < 1e-12);
    // Dry above the crest, moving below it.
    assert_eq!(
        w.water_velocity(Vec3::new(0.0, d + h, 0.0), 0.0),
        Vec3::zeros()
    );
    // Wheeler: the crest moves at the linear surface velocity a ω coth kd.
    let k = wave_number(2.0 * PI / t, d);
    let u = w
        .water_velocity(Vec3::new(0.0, d + h / 2.0 - 1e-9, 0.0), 0.0)
        .x;
    let e = h / 2.0 * 2.0 * PI / t / (k * d).tanh();
    assert!(rel_err(u, e) < 1e-6, "{u} vs {e}");
    // Paddle stroke from Biésel.
    let r = w.rig(0.3, 1.0);
    let stroke = h / piston_height_to_stroke(k * d);
    let amp = (r.paddle_position.powi(2) + (r.paddle_velocity / (2.0 * PI / t)).powi(2)).sqrt();
    assert!(rel_err(2.0 * amp, stroke) < 1e-12);
    assert!((r.wave_height - h).abs() < 1e-12 && (r.depth - d).abs() < 1e-12);
}

#[test]
fn solitary_wave_celerity_and_paddle() {
    let (d, h) = (0.3, 0.06);
    let mut w = Water::solitary(d, h, -3.0, Vec3::x());
    let c = (G * (d + h)).sqrt();
    let crest = |t: f64| {
        let mut best = (f64::MIN, 0.0);
        for i in 0..40001 {
            let x = -5.0 + i as f64 * 1e-3 * 0.5;
            let y = w.surface_elevation(x, 0.0, t);
            if y > best.0 {
                best = (y, x);
            }
        }
        best
    };
    let (y0, x0) = crest(0.5);
    let (y1, x1) = crest(1.5);
    let speed = x1 - x0;
    println!("celerity {speed} vs {c}, crest {} vs {}", y1, d + h);
    assert!(rel_err(speed, c) < 1e-3);
    assert!((y0 - (d + h)).abs() < 1e-6 && (y1 - (d + h)).abs() < 1e-6);
    // Depth-uniform velocity under the crest, c H / (d + H).
    let u = w.water_velocity(Vec3::new(x1, 0.1, 0.0), 1.5).x;
    assert!(rel_err(u, c * h / (d + h)) < 1e-5);

    // Goring paddle: it pushes out the wave volume ∫η dξ = 2H/K over the
    // depth, so the stroke is 2H/(K d) = √(16 H d / 3), and it ends at rest.
    w.prepare();
    let kappa = (3.0 * h / (4.0 * d.powi(3))).sqrt();
    let s = Structure::new();
    let sim = DynamicSim::new(&s, quiet());
    let geometry = Geometry::new(&s);
    let mut out = vec![];
    let mut tt = 0.0;
    let mut vmax: f64 = 0.0;
    while tt < 6.0 {
        let view = View {
            structure: &s,
            sim: &sim,
            geometry: &geometry,
            time: tt,
        };
        w.apply(&view, tt, 1.0, 1.0, 1.0 / 60.0, &mut out);
        vmax = vmax.max(w.rig(tt, 1.0).paddle_velocity);
        tt += 1.0 / 60.0;
    }
    let r = w.rig(tt, 1.0);
    let e = 2.0 * h / (kappa * d);
    println!(
        "stroke {} vs ~{e}, vmax {vmax} vs {}",
        r.paddle_position,
        c * h / (d + h)
    );
    assert!(rel_err(r.paddle_position, e) < 0.05);
    assert!(r.paddle_velocity.abs() < 1e-3);
    assert!(rel_err(vmax, c * h / (d + h)) < 0.01);
}

#[test]
fn flood_rises_with_time_and_ramp() {
    let w = Water::flood(0.8, 10.0);
    assert_eq!(w.still_level(0.0), 0.0);
    assert!((w.still_level(5.0) - 0.4).abs() < 1e-12);
    assert!((w.still_level(20.0) - 0.8).abs() < 1e-12);
    assert!((w.sea(20.0, 0.5).still_level() - 0.4).abs() < 1e-12);
    assert_eq!(w.intensity(), (0.8, "m"));
    assert_eq!(
        Water::airy(0.5, 0.1, 1.0, Vec3::x()).intensity(),
        (0.1, "m")
    );
}

#[test]
fn column_in_current_settles_to_static_deflection() {
    let sec = tube();
    let (s, ids) = column(2.0, 8, &sec);
    let top = *ids.last().unwrap();
    let mut loads = LoadSet::new(vec![LoadCase::new(
        "River",
        Window::new(0.0, None, 1.0),
        LoadKind::Water(Water::current(2.5, Vec3::new(2.0, 0.0, 0.0))),
    )]);
    let mut sim = DynamicSim::new(&s, quiet());
    let mut settled = false;
    for f in 0..60 * 20 {
        loads.step(&s, &mut sim, 1.0 / 60.0);
        if f > 180 && sim.max_speed() < 1e-7 {
            settled = true;
            break;
        }
    }
    assert!(settled, "did not settle");
    loads.apply(&s, &mut sim, 1.0 / 60.0);
    let nodal: Vec<NodeLoad> = loads
        .forces()
        .iter()
        .enumerate()
        .map(|(n, f)| NodeLoad::force(n, *f))
        .collect();
    let stat = solve_static(&s, &nodal).unwrap();
    let (dd, e) = (sim.displacement(top).x, stat.displacements[top].x);
    let drag = loads.totals()[0].x;
    println!(
        "tip {dd:e} vs static {e:e}, drag {drag} vs {}",
        0.5 * RHO * 0.1 * 2.0 * 4.0
    );
    assert!(rel_err(drag, 0.5 * RHO * 1.0 * 0.1 * 2.0 * 4.0) < 1e-3);
    assert!(rel_err(dd, e) < 0.02);
}

/// A free horizontal member along x with its axis at `y`.
fn log(y: f64, mat: &Material, sec: &Section) -> Structure {
    let mut s = Structure::new();
    s.add_line(Vec3::new(-1.0, y, 0.0), Vec3::new(1.0, y, 0.0), 2, mat, sec);
    s
}

#[test]
fn buoyancy_grows_smoothly_as_a_horizontal_tube_sinks_in() {
    // Water surface at y = 1; a tube of radius 0.05, 2 m long.
    let (ro, len) = (0.05, 2.0);
    let full = RHO * G * PI * ro * ro * len;
    let lift = |rise: f64| {
        let s = log(1.0 - rise, &Material::steel(), &tube());
        total_force(&s, Water::current(1.0, Vec3::zeros())).y
    };
    assert_eq!(lift(-ro), 0.0);
    assert!(rel_err(lift(0.0), 0.5 * full) < 1e-12);
    assert!(rel_err(lift(ro), full) < 1e-12);
    assert!(rel_err(lift(3.0 * ro), full) < 1e-12);
    // Half a radius in: the circle segment below the chord.
    let t: f64 = 0.5;
    let segment = ((-t).acos() + t * (1.0 - t * t).sqrt()) / PI;
    assert!(rel_err(lift(0.5 * ro), segment * full) < 1e-12);
}

#[test]
fn a_timber_log_floats_at_its_draft() {
    let (b, level) = (0.2, 1.0);
    let timber = Material::timber();
    let s = log(level + 0.05, &timber, &Section::rect(b, b));
    let mut loads = instant(Water::current(level, Vec3::zeros()));
    loads.prepare(&s);
    let mut sim = DynamicSim::new(
        &s,
        SimConfig {
            plasticity: false,
            ..Default::default()
        },
    );
    for _ in 0..(8.0 * 60.0) as usize {
        loads.step(&s, &mut sim, 1.0 / 60.0);
    }
    // Archimedes: the wet share of the section is the density ratio.
    let wet = timber.density / RHO;
    let expected = level - (wet - 0.5) * b;
    for n in 0..3 {
        let y = sim.position(n).y;
        println!("node {n}: axis at {y}, expected {expected}");
        assert!((y - expected).abs() < 0.005);
        let v = sim.velocity(n).norm();
        assert!(v < 0.02, "still bobbing at {v} m/s");
    }
}

#[test]
fn a_damaged_or_open_tube_floods() {
    // A wide thin tube floats when sealed and sinks when the water is let in.
    let sec = Section::tube(0.3, 0.004);
    let s = log(0.5, &Material::steel(), &sec);
    let weight = 7850.0 * sec.area * 2.0 * G;
    let sealed = total_force(&s, Water::current(1.0, Vec3::zeros())).y;
    let open = total_force(
        &s,
        Water {
            sealed: false,
            ..Water::current(1.0, Vec3::zeros())
        },
    )
    .y;
    println!("weight {weight}, lift sealed {sealed}, open {open}");
    assert!(sealed > 2.0 * weight);
    assert!(open < 0.2 * weight);
}

#[test]
fn broken_timber_floats() {
    // A stick under water, torn apart by a pull on its free end: the
    // pieces come up and float instead of falling through the water.
    let (b, len, level) = (0.03, 2.0, 1.0);
    let mut s = Structure::new();
    let ids = s.add_line(
        Vec3::new(0.0, 0.3, 0.0),
        Vec3::new(len, 0.3, 0.0),
        2,
        &Material::timber(),
        &Section::rect(b, b),
    );
    s.add_support(Support::fixed(ids[0]));
    let mut loads = instant(Water::current(level, Vec3::zeros()));
    loads.static_loads = vec![NodeLoad::force(ids[2], Vec3::new(2.0e5, 0.0, 0.0))];
    loads.prepare(&s);
    let mut sim = DynamicSim::new(&s, SimConfig::default());
    let dt = 1.0 / 240.0;
    let mut steps = 0;
    while sim.break_events().is_empty() {
        loads.step(&s, &mut sim, dt);
        steps += 1;
        assert!(steps < 240, "the stick should have snapped");
    }
    loads.static_loads.clear();
    for _ in 0..(10.0 / dt) as usize {
        loads.step(&s, &mut sim, dt);
    }
    let broken: Vec<bool> = (0..2).map(|m| sim.is_broken(m)).collect();
    let y = sim.position(ids[2]).y;
    println!("broken {broken:?}, free end at y = {y}");
    assert!((y - level).abs() < 0.05, "the free end should float: {y}");
}
