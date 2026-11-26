use breakit_core::*;
fn main() {
    let mut s = Structure::new();
    let ids = s.add_line(
        Vec3::zeros(),
        Vec3::new(2.0, 0.0, 0.0),
        10,
        &Material::steel(),
        &Section::tube(0.05, 0.005),
    );
    s.add_support(Support::fixed(ids[0]));
    let tip = ids[10];
    let f = Vec3::new(0.0, -1000.0, 0.0);
    let e = solve_static(&s, &[NodeLoad::force(tip, f)])
        .unwrap()
        .displacements[tip]
        .y;
    let subs: usize = std::env::args()
        .nth(1)
        .map(|a| a.parse().unwrap())
        .unwrap_or(40);
    let mut sim = DynamicSim::new(
        &s,
        SimConfig {
            gravity: Vec3::zeros(),
            damping: 8.0,
            plasticity: false,
            solver: if std::env::args().nth(3).as_deref() == Some("gs") {
                Solver::GaussSeidel
            } else {
                Solver::Direct
            },
            substeps: subs,
            iterations: std::env::args()
                .nth(2)
                .map(|a| a.parse().unwrap())
                .unwrap_or(1),
            ..Default::default()
        },
    );
    sim.set_force(tip, f);
    for fr in 0..600 {
        sim.step(1.0 / 60.0);
        if fr % 60 == 0 {
            println!(
                "t={:.1} tip={:.4e} static={:.4e} vmax={:.3e} z={:.2e}",
                sim.time(),
                sim.displacement(tip).y,
                e,
                sim.max_speed(),
                sim.displacement(tip).z
            );
        }
    }
}
