use breakit_core::*;
fn main() {
    let ratio: f64 = std::env::args()
        .nth(1)
        .map(|a| a.parse().unwrap())
        .unwrap_or(0.0);
    let subs: usize = std::env::args()
        .nth(2)
        .map(|a| a.parse().unwrap())
        .unwrap_or(8);
    let it: usize = std::env::args()
        .nth(3)
        .map(|a| a.parse().unwrap())
        .unwrap_or(1);
    let sec = Section::tube(0.05, 0.005);
    let mut s = Structure::new();
    let ids = s.add_line(
        Vec3::zeros(),
        Vec3::new(0.0, 3.0, 0.0),
        12,
        &Material::steel().elastic_only(),
        &sec,
    );
    s.add_support(Support::fixed(ids[0]));
    let pcr = std::f64::consts::PI.powi(2) * 210e9 * sec.iz / 36.0;
    let f = Vec3::new(0.001 * pcr, -ratio * pcr, 0.0);
    let st = solve_static(&s, &[NodeLoad::force(12, f)])
        .unwrap()
        .displacements[12]
        .x;
    let mut sim = DynamicSim::new(
        &s,
        SimConfig {
            gravity: Vec3::zeros(),
            damping: 2.0,
            plasticity: false,
            substeps: subs,
            iterations: it,
            ..Default::default()
        },
    );
    for fr in 0..600 {
        let t = fr as f64 / 60.0;
        sim.set_force(12, f * (t / 2.0).min(1.0));
        sim.step(1.0 / 60.0);
        if fr % 60 == 59 {
            println!(
                "t={:.0} x={:.4e} lin_static={:.4e} amp_expected={:.4e} y={:.3e}",
                t + 1.0 / 60.0,
                sim.displacement(12).x,
                st,
                st / (1.0 - ratio),
                sim.displacement(12).y
            );
        }
    }
}
