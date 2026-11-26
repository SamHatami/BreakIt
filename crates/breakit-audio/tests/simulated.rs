//! End to end: a real `DynamicSim` run, heard through the listener and the
//! mixer. Nothing here is scripted; the sound is whatever the solver did.

use breakit_audio::{BreakCause, Command, Mixer, StructureListener};
use breakit_core::{DynamicSim, Material, Section, SimConfig, Structure, Support, Vec3};

const SR: f32 = 48_000.0;

fn peak(x: &[f32]) -> f32 {
    x.iter().fold(0.0f32, |m, v| m.max(v.abs()))
}

/// Steps `sim` at 60 Hz for `seconds`, with `load(t)` on `node`, and returns
/// the commands the listener sent and the audio.
fn run(
    structure: &Structure,
    node: usize,
    seconds: f64,
    load: impl Fn(f64) -> Vec3,
) -> (Vec<Command>, Vec<f32>) {
    let mut sim = DynamicSim::new(structure, SimConfig::default());
    let mut listener = StructureListener::new();
    let mut mixer = Mixer::new(SR);
    let mut commands = Vec::new();
    let mut audio = Vec::new();
    let dt = 1.0 / 60.0;
    for frame in 0..(seconds / dt) as usize {
        sim.set_force(node, load(frame as f64 * dt));
        sim.step(dt);
        listener.update(&sim, structure, dt as f32, [0.0; 3], |c| {
            commands.push(c.clone());
            mixer.handle(c);
        });
        audio.extend(mixer.render_seconds(dt as f32));
    }
    (commands, audio)
}

#[test]
fn a_steel_tie_pulled_apart_is_heard_as_it_yields_and_ruptures() {
    let section = Section::tube(0.05, 0.005);
    let mut s = Structure::new();
    let ids = s.add_line(
        Vec3::zeros(),
        Vec3::new(0.0, -2.0, 0.0),
        2,
        &Material::steel(),
        &section,
    );
    s.add_support(Support::fixed(ids[0]));
    let yield_force = 355e6 * section.area;
    // Ramp to 110% of yield over two seconds, then hold.
    let (commands, audio) = run(&s, ids[2], 5.0, |t| {
        Vec3::new(0.0, -1.1 * yield_force * (t / 2.0).min(1.0), 0.0)
    });
    assert!(audio.iter().all(|x| x.is_finite()));

    let strains = commands
        .iter()
        .filter(|c| matches!(c, Command::Strain { .. }))
        .count();
    assert!(strains > 20, "loading was not reported: {strains}");

    let breaks: Vec<_> = commands
        .iter()
        .filter_map(|c| match c {
            Command::Break { info, .. } => Some(*info),
            _ => None,
        })
        .collect();
    assert!(!breaks.is_empty(), "the tie never broke");
    let b = breaks[0];
    assert_eq!(b.cause, BreakCause::Rupture);
    // At yield, a 1 m tube holds σ²/2E · A · L = 355e6² / 420e9 · A J.
    let at_yield = (355e6f64.powi(2) / (2.0 * 210e9) * section.area) as f32;
    assert!(
        b.energy > 0.5 * at_yield && b.energy < 3.0 * at_yield,
        "energy {} J, yield energy {at_yield} J",
        b.energy
    );
    assert!(b.axial > 0.9, "a tie's energy is axial: {}", b.axial);
    assert!((b.body.length - 1.0).abs() < 0.05);
    // The break is the loudest thing in the run.
    assert!(peak(&audio) > 0.2, "{}", peak(&audio));
}

#[test]
fn a_structure_at_rest_under_load_is_silent() {
    let section = Section::tube(0.05, 0.005);
    let mut s = Structure::new();
    let ids = s.add_line(
        Vec3::zeros(),
        Vec3::new(2.0, 0.0, 0.0),
        4,
        &Material::steel(),
        &section,
    );
    s.add_support(Support::fixed(ids[0]));
    // A third of the collapse load, applied gently and held.
    let p = 355e6 * section.zz / 2.0 / 3.0;
    let (commands, audio) = run(&s, ids[4], 6.0, |t| {
        Vec3::new(0.0, -p * (t / 1.0).min(1.0), 0.0)
    });
    assert!(!commands.iter().any(|c| matches!(c, Command::Break { .. })));
    // Once it has settled, nothing.
    let settled = &audio[(4.5 * SR as f64) as usize * 2..];
    assert!(peak(settled) < 2e-3, "{}", peak(settled));
}
