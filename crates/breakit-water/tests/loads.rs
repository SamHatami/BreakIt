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
