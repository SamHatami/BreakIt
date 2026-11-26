use breakit_core::*;
fn main() {
    let sec = Section::tube(0.05, 0.005);
    for over in [1.02, 1.1, 1.5] {
        for sub in [64, 16, 8, 4, 2, 1] {
            let mut s = Structure::new();
            let ids = s.add_line(
                Vec3::zeros(),
                Vec3::new(0.0, -2.0, 0.0),
                2,
                &Material::steel(),
                &sec,
            );
            s.add_support(Support::fixed(ids[0]));
            let ny = 355e6 * sec.area;
            let mut sim = DynamicSim::new(
                &s,
                SimConfig {
                    gravity: Vec3::zeros(),
                    damping: 8.0,
                    substeps: sub,
                    ..Default::default()
                },
            );
            sim.set_force(ids[2], Vec3::new(0.0, -over * ny, 0.0));
            for _ in 0..2400 {
                sim.step(1.0 / 60.0);
                if !sim.break_events().is_empty() {
                    break;
                }
            }
            println!(
                "over {over} sub {sub}: rupture at t = {:?} {:?}",
                sim.break_events().first().map(|e| e.time),
                sim.solver_stats()
            );
        }
    }
    // Cantilever hinge: time to snap at 1.3x the collapse load.
    for sub in [64, 16, 8, 4, 2, 1] {
        let mut s = Structure::new();
        let ids = s.add_line(
            Vec3::zeros(),
            Vec3::new(2.0, 0.0, 0.0),
            8,
            &Material::steel(),
            &sec,
        );
        s.add_support(Support::fixed(ids[0]));
        let p = 355e6 * sec.zz / 2.0;
        let mut sim = DynamicSim::new(
            &s,
            SimConfig {
                gravity: Vec3::zeros(),
                damping: 8.0,
                substeps: sub,
                ..Default::default()
            },
        );
        for f in 0..600 {
            let t = f as f64 / 60.0;
            sim.set_force(ids[8], Vec3::new(0.0, -1.3 * p * t.min(1.0), 0.0));
            sim.step(1.0 / 60.0);
            if !sim.break_events().is_empty() {
                break;
            }
        }
        println!(
            "hinge sub {sub}: snap at t = {:?} member {:?} {:?}",
            sim.break_events().first().map(|e| e.time),
            sim.break_events().first().map(|e| e.member),
            sim.solver_stats()
        );
    }
}
