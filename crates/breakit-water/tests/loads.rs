//! The simulated water feeding the core load model: the force on a pile
//! against the same load model on analytic Airy waves, the wake behind a
//! pile, and a core water case driving the paddle.

mod common;

use breakit_core::loads::water::Water;
use breakit_core::loads::{Geometry, View};
use breakit_core::{
    DynamicSim, LoadCase, LoadKind, LoadSet, Material, Section, SimConfig, Structure, Support,
    Vec3, Window,
};
use breakit_water::{Config, Drive, FlumeSetup, WaterSim};
use common::*;

/// A stiff vertical pile from the bed to above the water at `x`.
fn pile(x: f64, bed: f64) -> Structure {
    let mut s = Structure::new();
    let ids = s.add_line(
        Vec3::new(x, bed, 0.5),
        Vec3::new(x, 1.5, 0.5),
        12,
        &Material::steel(),
        &Section::tube(0.2, 0.02),
    );
    s.add_support(Support::fixed(ids[0]));
    s
}

fn water_case(water: Water) -> LoadSet {
    LoadSet::new(vec![LoadCase::new(
        "Waves",
        Window::new(0.0, None, 0.0),
        LoadKind::Water(water),
    )])
}

/// Peak horizontal force on the pile over the last `count` wave periods.
fn peak_force(loads: &LoadSet, record: &mut Vec<f64>) {
    record.push(loads.totals()[0].x);
}

#[test]
fn pile_force_matches_the_load_model_on_airy_waves() {
    let (bed, period, height) = (-2.0, 3.5, 0.2);
    let depth = -bed;
    let x_pile = 20.0;
    let structure = pile(x_pile, bed);
    let mut case = Water::airy(0.0, height, period, Vec3::x());
    case.bed = bed;

    // Reference: the core's analytic waves.
    let mut analytic = water_case(case.clone());
    analytic.prepare(&structure);
    let mut sim_a = DynamicSim::new(&structure, SimConfig::default());
    let mut fa = vec![];
    for _ in 0..(4.0 * period / DT) as usize {
        analytic.step(&structure, &mut sim_a, DT);
        peak_force(&analytic, &mut fa);
    }

    // The same case on the simulated water, the paddle driven by the case.
    let mut water = WaterSim::flume(
        FlumeSetup {
            x: (0.0, 140.0),
            z: (0.3, 0.7),
            dx: 0.1,
            bed,
            still_level: 0.0,
            beach: None,
            outlet: false,
        },
        Config::default(),
    );
    case.paddle = 0.0;
    let mut simulated = water_case(case.clone());
    simulated.prepare(&structure);
    let mut sim_s = DynamicSim::new(&structure, SimConfig::default());
    let mut fs = vec![];
    let (mut crest, mut trough) = (0.0_f64, 0.0_f64);
    let t_end = 16.0 * period;
    let mut t = 0.0;
    while t < t_end {
        // Fade the paddle in over two periods.
        let ramp = (t / (2.0 * period)).min(1.0);
        water.advance(DT, Drive::from_case(&case, t, ramp));
        if let LoadKind::Water(w) = &mut simulated.cases[0].kind {
            w.field = Some(water.field());
        }
        simulated.step(&structure, &mut sim_s, DT);
        t += DT;
        if t > t_end - 4.0 * period {
            peak_force(&simulated, &mut fs);
            let level = water.field().column(x_pile, 0.5).expect("wet").0;
            crest = crest.max(level);
            trough = trough.min(level);
        }
    }
    let peak = |f: &[f64]| f.iter().fold(0.0_f64, |a, x| a.max(x.abs()));
    let (pa, ps) = (peak(&fa), peak(&fs));
    assert!(depth > 0.0 && pa > 100.0, "the reference should push: {pa}");
    // The rescaled stroke gives the wave height the case asks for...
    let got = crest - trough;
    assert!(rel(got, height) < 0.08, "wave height {got} vs {height}");
    // ...and the force on the pile agrees with the analytic model.
    assert!(
        rel(ps, pa) < 0.1,
        "pile force {ps} N simulated vs {pa} N analytic"
    );
}

#[test]
fn a_pile_in_a_current_leaves_a_wake() {
    let bed = -1.5;
    let structure = pile(8.0, bed);
    let mut water = WaterSim::flume(
        FlumeSetup {
            x: (0.0, 20.0),
            z: (-1.0, 2.0),
            dx: 0.1,
            bed,
            still_level: 0.0,
            beach: None,
            outlet: true,
        },
        Config::default(),
    );
    let geometry = Geometry::new(&structure);
    let sim = DynamicSim::new(&structure, SimConfig::default());
    let mut t = 0.0;
    while t < 20.0 {
        let current = 1.5 * (t / 5.0_f64).min(1.0);
        let view = View {
            structure: &structure,
            sim: &sim,
            geometry: &geometry,
            time: t,
        };
        water.couple(&view, 1.0, 2.0);
        water.advance(
            DT,
            Drive {
                current,
                ..Drive::default()
            },
        );
        t += DT;
    }
    let field = water.field();
    let behind = field.flow(8.6, 0.5).0;
    let beside = field.flow(8.6, -0.7).0;
    let upstream = field.flow(3.0, 0.5).0;
    assert!(rel(upstream, 1.5) < 0.1, "current {upstream}");
    assert!(
        behind < 0.8 * beside,
        "no wake: {behind} behind, {beside} beside"
    );
    // The water piles up in front of the pile and drops behind it.
    let front = field.column(7.8, 0.5).expect("wet").0;
    let back = field.column(8.3, 0.5).expect("wet").0;
    assert!(
        front > back + 0.005,
        "no set-up: {front} in front, {back} behind"
    );
}

#[test]
fn a_dropped_log_splashes_and_floats() {
    let (bed, b) = (-2.0, 0.2);
    let timber = Material::timber();
    let mut structure = Structure::new();
    structure.add_line(
        Vec3::new(4.0, 1.5, 1.0),
        Vec3::new(6.0, 1.5, 1.0),
        2,
        &timber,
        &Section::rect(b, b),
    );
    let mut water = WaterSim::flume(
        FlumeSetup {
            x: (0.0, 10.0),
            z: (0.0, 2.0),
            dx: 0.1,
            bed,
            still_level: 0.0,
            beach: None,
            outlet: false,
        },
        Config::default(),
    );
    let mut case = Water::current(0.0, Vec3::zeros());
    case.bed = bed;
    let mut loads = water_case(case);
    loads.prepare(&structure);
    let geometry = Geometry::new(&structure);
    let mut sim = DynamicSim::new(&structure, SimConfig::default());
    let (mut most_spray, mut ring) = (0usize, 0.0_f64);
    let mut t = 0.0;
    while t < 10.0 {
        water.couple(
            &View {
                structure: &structure,
                sim: &sim,
                geometry: &geometry,
                time: t,
            },
            1.0,
            2.0,
        );
        water.advance(DT, Drive::default());
        if let LoadKind::Water(w) = &mut loads.cases[0].kind {
            w.field = Some(water.field());
        }
        loads.step(&structure, &mut sim, DT);
        t += DT;
        most_spray = most_spray.max(water.droplets().len());
        // The water around the log is pushed about by the splash.
        let level = water.field().column(5.0, 1.2).expect("wet").0;
        ring = ring.max(level.abs());
    }
    assert!(most_spray >= 20, "no splash: {most_spray} droplets");
    assert!(ring > 0.005, "the splash left the water flat: {ring} m");
    // Archimedes: the wet share of the section is the density ratio.
    let expected = -(timber.density / 1000.0 - 0.5) * b;
    for n in 0..3 {
        let y = sim.position(n).y;
        assert!(
            (y - expected).abs() < 0.02,
            "node {n} at {y}, expected {expected}"
        );
    }
}

#[test]
fn a_pontoon_raft_carries_its_load_through_waves() {
    use breakit_core::loads::water::wet_fraction;
    let bed = -2.5;
    // Two large pontoons (CHS 813x16) 3 m long, 1 m apart, tied by three
    // cross tubes, with 600 kg of cargo on the middle one.
    let (ro, wall) = (0.4065, 0.016);
    let (pontoon, tie) = (Section::tube(ro, wall), Section::tube(0.057, 0.005));
    let steel = Material::steel();
    let mut s = Structure::new();
    let left = s.add_line(
        Vec3::new(20.0, 0.0, 0.5),
        Vec3::new(23.0, 0.0, 0.5),
        2,
        &steel,
        &pontoon,
    );
    let right = s.add_line(
        Vec3::new(20.0, 0.0, 1.5),
        Vec3::new(23.0, 0.0, 1.5),
        2,
        &steel,
        &pontoon,
    );
    for (a, b) in left.iter().zip(&right) {
        s.add_member(*a, *b, steel.clone(), tie.clone());
    }
    let cargo = 600.0;
    let mass = steel.density * (6.0 * pontoon.area + 3.0 * tie.area) + cargo;
    // Archimedes for the pontoons (the ties are under water with them).
    let lift_needed = mass - 1000.0 * 3.0 * std::f64::consts::PI * 0.057 * 0.057;
    let full = 1000.0 * 6.0 * std::f64::consts::PI * ro * ro;
    let share = lift_needed / full;
    assert!(share < 0.95, "the raft should float: {share}");
    let (mut lo, mut hi) = (-ro, ro);
    for _ in 0..50 {
        let mid = 0.5 * (lo + hi);
        if wet_fraction(mid, ro, true) < share {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    let draft_axis = -lo;

    let (period, height) = (4.0, 0.3);
    let mut water = WaterSim::flume(
        FlumeSetup {
            x: (0.0, 120.0),
            z: (0.0, 2.0),
            dx: 0.2,
            bed,
            still_level: 0.0,
            beach: None,
            outlet: false,
        },
        Config::default(),
    );
    let mut case = Water::airy(0.0, height, period, Vec3::x());
    case.bed = bed;
    case.paddle = 0.0;
    let mut loads = water_case(case.clone());
    loads.prepare(&s);
    let geometry = Geometry::new(&s);
    let mut sim = DynamicSim::new(&s, SimConfig::default());
    sim.add_point_mass(left[1], 0.5 * cargo);
    sim.add_point_mass(right[1], 0.5 * cargo);
    let centre = |sim: &DynamicSim| 0.5 * (sim.position(left[1]).y + sim.position(right[1]).y);
    let mut t = 0.0;
    let (mut still, mut top, mut bottom, mut tilt) = (0.0, f64::MIN, f64::MAX, 0.0_f64);
    // Still water first, then waves from 12 s on.
    while t < 12.0 + 12.0 * period {
        water.couple(
            &View {
                structure: &s,
                sim: &sim,
                geometry: &geometry,
                time: t,
            },
            1.0,
            2.0,
        );
        let waves = t - 12.0;
        let drive = if waves > 0.0 {
            Drive::from_case(&case, waves, (waves / (2.0 * period)).min(1.0))
        } else {
            Drive::default()
        };
        water.advance(DT, drive);
        if let LoadKind::Water(w) = &mut loads.cases[0].kind {
            w.field = Some(water.field());
        }
        loads.step(&s, &mut sim, DT);
        t += DT;
        if (t - 12.0).abs() < 0.5 * DT {
            still = centre(&sim);
        }
        if waves > 8.0 * period {
            top = top.max(centre(&sim));
            bottom = bottom.min(centre(&sim));
            tilt = tilt.max((sim.position(left[1]).y - sim.position(right[1]).y).abs());
        }
    }
    println!("axis at {still} in still water, expected {draft_axis}");
    assert!((still - draft_axis).abs() < 0.03);
    // A long wave lifts the raft with it: it heaves about the wave height
    // around its still-water draft, and stays level across.
    let heave = top - bottom;
    println!("heave {heave} in {height} m waves, tilt {tilt}");
    assert!(heave > 0.6 * height && heave < 1.5 * height);
    assert!((0.5 * (top + bottom) - draft_axis).abs() < 0.06);
    assert!(tilt < 0.03);
    assert!(sim.break_events().is_empty());
}
