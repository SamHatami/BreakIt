//! Cost of evaluating scenario loads per frame on a braced lattice tower.
//! `cargo run --release --example loads_bench`
use breakit_core::loads::{
    seismic::Quake,
    water::Water,
    wind::{Gusts, Wind},
};
use breakit_core::{
    DynamicSim, LoadCase, LoadKind, LoadSet, Material, Section, SimConfig, Structure, Support,
    Vec3, Window,
};
use std::time::Instant;

fn main() {
    // n x n x h grid of 1 m cells with columns, beams both ways.
    let (n, h) = (6usize, 12usize);
    let mut s = Structure::new();
    let id = |i: usize, j: usize, k: usize| (k * (n + 1) + j) * (n + 1) + i;
    for k in 0..=h {
        for j in 0..=n {
            for i in 0..=n {
                s.add_node(Vec3::new(i as f64, k as f64, j as f64));
            }
        }
    }
    let (m, sec) = (Material::steel(), Section::tube(0.05, 0.005));
    for k in 0..=h {
        for j in 0..=n {
            for i in 0..=n {
                if i < n {
                    s.add_member(id(i, j, k), id(i + 1, j, k), m.clone(), sec.clone());
                }
                if j < n {
                    s.add_member(id(i, j, k), id(i, j + 1, k), m.clone(), sec.clone());
                }
                if k < h {
                    s.add_member(id(i, j, k), id(i, j, k + 1), m.clone(), sec.clone());
                }
                if k == 0 {
                    s.add_support(Support::fixed(id(i, j, k)));
                }
            }
        }
    }
    println!("{} nodes, {} members", s.nodes.len(), s.members.len());
    let w = Window::new(0.0, None, 1.0);
    let mut wind = Wind::new(30.0, Vec3::x());
    wind.gusts = Some(Gusts::default());
    let cases = [
        ("wind", LoadKind::Wind(wind)),
        (
            "quake",
            LoadKind::Quake(Quake::synthetic(0.3, 10.0, Vec3::x(), 1)),
        ),
        (
            "waves",
            LoadKind::Water(Water::airy(3.0, 0.5, 2.0, Vec3::x())),
        ),
    ];
    for (name, kind) in cases {
        let mut loads = LoadSet::new(vec![LoadCase::new(name, w.clone(), kind)]);
        let mut sim = DynamicSim::new(&s, SimConfig::default());
        loads.prepare(&s);
        let frames = 120;
        let mut t_loads = 0.0;
        let t0 = Instant::now();
        for _ in 0..frames {
            let a = Instant::now();
            loads.apply(&s, &mut sim, 1.0 / 60.0);
            t_loads += a.elapsed().as_secs_f64();
            sim.step(1.0 / 60.0);
        }
        let total = t0.elapsed().as_secs_f64();
        println!(
            "{name:6} loads {:.3} ms/frame, solver {:.2} ms/frame",
            1e3 * t_loads / frames as f64,
            1e3 * (total - t_loads) / frames as f64
        );
    }
}
