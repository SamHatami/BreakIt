//! Renders the sound of real solver runs to WAV: nothing is scripted but the
//! load. A truss is loaded at mid-span until it fails, once in steel and
//! once in timber, and what you hear is what `StructureListener` made of the
//! solver's utilisation, damage and break events. A third run puts a truss
//! of thin steel tubes in a rising, gusty wind: the members sing at their
//! vortex shedding rates, and ring when a gust carries that rate through
//! one of their own modes.
//!
//! ```text
//! cargo run --release -p breakit-audio --example render_sim -- <out_dir>
//! ```

use std::path::PathBuf;

use breakit_audio::listen::wind_tones;
use breakit_audio::wav::write_wav;
use breakit_audio::{AmbientKind, Command, Mixer, StructureListener};
use breakit_core::loads::wind::{Gusts, Wind};
use breakit_core::loads::{LoadCase, LoadKind, LoadSet, Window};
use breakit_core::{DynamicSim, Material, Section, SimConfig, Structure, Support, Vec3};

const SR: f32 = 48_000.0;

/// A Warren truss spanning `span` metres with `bays` bays, pinned at both
/// ends. Returns the structure and the bottom-chord node at mid-span.
fn truss(span: f64, bays: usize, material: &Material, section: &Section) -> (Structure, usize) {
    let mut s = Structure::new();
    let bay = span / bays as f64;
    let height = 0.8 * bay;
    let bottom: Vec<usize> = (0..=bays)
        .map(|i| s.add_node(Vec3::new(i as f64 * bay - span / 2.0, 0.0, 0.0)))
        .collect();
    let top: Vec<usize> = (0..bays)
        .map(|i| s.add_node(Vec3::new((i as f64 + 0.5) * bay - span / 2.0, height, 0.0)))
        .collect();
    for i in 0..bays {
        s.add_member(bottom[i], bottom[i + 1], material.clone(), section.clone());
        s.add_member(bottom[i], top[i], material.clone(), section.clone());
        s.add_member(top[i], bottom[i + 1], material.clone(), section.clone());
        if i + 1 < bays {
            s.add_member(top[i], top[i + 1], material.clone(), section.clone());
        }
    }
    s.add_support(Support::pinned(bottom[0]));
    s.add_support(Support::pinned(bottom[bays]));
    (s, bottom[bays / 2])
}

/// Loads the truss at `rate` newtons per second until `seconds` are up.
fn render(structure: &Structure, node: usize, rate: f64, seconds: f64) -> (Vec<f32>, usize, f64) {
    let mut sim = DynamicSim::new(structure, SimConfig::default());
    let mut listener = StructureListener::new();
    let mut mixer = Mixer::new(SR);
    mixer.handle(Command::Listener {
        position: [0.0, 1.5, 6.0],
        right: [1.0, 0.0, 0.0],
        up: [0.0, 1.0, 0.0],
    });
    mixer.handle(Command::Ambient {
        id: 0,
        kind: AmbientKind::Room,
        level: 1.0,
        place: None,
    });
    let mut audio = Vec::new();
    let dt = 1.0 / 60.0;
    let mut first_break = f64::NAN;
    for frame in 0..(seconds / dt) as usize {
        let t = frame as f64 * dt;
        // Stop adding load once something has broken, and let it fall.
        let load = if first_break.is_nan() { t } else { first_break } * rate;
        sim.set_force(node, Vec3::new(0.0, -load, 0.0));
        sim.step(dt);
        if first_break.is_nan() && !sim.break_events().is_empty() {
            first_break = t;
        }
        listener.update(&sim, structure, dt as f32, [0.0; 3], |c| mixer.handle(c));
        audio.extend(mixer.render_seconds(dt as f32));
    }
    (audio, sim.break_events().len(), first_break)
}

/// The truss, held along its bottom chord, in a wind across it that rises
/// to `speed` m/s over `rise` seconds.
fn render_wind(structure: &Structure, speed: f64, rise: f64, seconds: f64) -> (Vec<f32>, f32, f32) {
    let mut sim = DynamicSim::new(structure, SimConfig::default());
    let mut wind = Wind::new(speed, Vec3::z());
    wind.gusts = Some(Gusts::default());
    let mut loads = LoadSet::new(vec![LoadCase::new(
        "Wind",
        Window::new(0.0, None, rise),
        LoadKind::Wind(wind),
    )]);
    loads.prepare(structure);
    let mut listener = StructureListener::new();
    let mut mixer = Mixer::new(SR);
    mixer.handle(Command::Listener {
        position: [0.0, 1.5, 6.0],
        right: [1.0, 0.0, 0.0],
        up: [0.0, 1.0, 0.0],
    });
    mixer.handle(Command::Ambient {
        id: 0,
        kind: AmbientKind::Room,
        level: 1.0,
        place: None,
    });
    let mut audio = Vec::new();
    let dt = 1.0 / 60.0;
    let (mut low, mut high) = (f32::INFINITY, 0.0f32);
    for _ in 0..(seconds / dt) as usize {
        loads.step(structure, &mut sim, dt);
        listener.update(&sim, structure, dt as f32, [0.0; 3], |c| mixer.handle(c));
        if let LoadKind::Wind(wind) = &loads.cases[0].kind {
            wind_tones(&sim, structure, wind, [0.0; 3], |c| {
                if let Command::Wind { frequency, .. } = &c {
                    low = low.min(*frequency);
                    high = high.max(*frequency);
                }
                mixer.handle(c)
            });
        }
        audio.extend(mixer.render_seconds(dt as f32));
    }
    (audio, low, high)
}

fn main() {
    let dir = PathBuf::from(
        std::env::args()
            .nth(1)
            .unwrap_or_else(|| "audio-demo".into()),
    );
    std::fs::create_dir_all(&dir).expect("create output dir");
    let runs = [
        (
            "sim-steel-truss",
            Material::steel(),
            Section::tube(0.03, 0.003),
        ),
        (
            "sim-timber-truss",
            Material::timber(),
            Section::rect(0.06, 0.09),
        ),
    ];
    for (name, material, section) in runs {
        let (structure, node) = truss(6.0, 6, &material, &section);
        // Find the load that breaks it in about five seconds: try a rate,
        // then rescale.
        let (_, _, t) = render(&structure, node, 40e3, 30.0);
        let rate = if t.is_nan() { 400e3 } else { 40e3 * t / 5.0 };
        let (audio, breaks, t) = render(&structure, node, rate, 9.0);
        let path = dir.join(format!("{name}.wav"));
        write_wav(&path, &audio, 2, SR as u32).expect("write wav");
        let peak = audio.iter().fold(0.0f32, |m, x| m.max(x.abs()));
        println!(
            "{name}: first break at {t:.2} s under {:.0} kN, {breaks} members broke, peak {peak:.2} -> {}",
            rate * t / 1e3,
            path.display()
        );
    }

    // Wind: thin tubes, so the shedding is in the audible range.
    let (mut structure, _) = truss(6.0, 6, &Material::steel(), &Section::tube(0.016, 0.002));
    for i in 1..6 {
        structure.add_support(Support::fixed(i));
    }
    let (audio, low, high) = render_wind(&structure, 32.0, 10.0, 16.0);
    let path = dir.join("sim-wind-steel-truss.wav");
    write_wav(&path, &audio, 2, SR as u32).expect("write wav");
    let peak = audio.iter().fold(0.0f32, |m, x| m.max(x.abs()));
    println!(
        "sim-wind-steel-truss: shedding heard from {low:.0} to {high:.0} Hz, peak {peak:.2} -> {}",
        path.display()
    );
}
