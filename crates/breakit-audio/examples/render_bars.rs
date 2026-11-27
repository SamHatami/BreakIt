//! What pure physical modelling gives for the textbook case: two hollow
//! steel bars, free in the air, knocked together. Nothing is recorded. Each
//! bar rings at the bending and wall modes of a free tube of its size, the
//! contact is a Hertz pulse, and the level of each mode follows how well it
//! radiates. Then the same knock with timber posts and with carbon tubes.
//!
//! ```text
//! cargo run --release -p breakit-audio --example render_bars -- <out_dir>
//! ```

use std::path::PathBuf;

use breakit_audio::body::{Drive, Ends};
use breakit_audio::material::{
    break_voice, capacity_energy, contact_time, impact_voice, BreakInfo,
};
use breakit_audio::modal::{Bank, Hit, ModalVoice};
use breakit_audio::wav::write_wav;
use breakit_audio::{Body, BreakCause, Voice};
use breakit_core::{Material, Section};

const SR: f32 = 48_000.0;

/// Two free bars of `material` and `section`, `la` and `lb` metres long,
/// meeting at `speed` m/s, with contact modulus `e` (Pa).
fn knock(
    material: &Material,
    section: &Section,
    (la, lb): (f64, f64),
    speed: f32,
    e: f32,
    seed: u32,
) -> Vec<f32> {
    let free = |l: f64| Body {
        ends: Ends::Free,
        ..Body::member(material, section, l)
    };
    let (a, b) = (free(la), free(lb));
    // The lighter bar sets the contact; they meet edge to edge.
    let mass = a.mass().min(b.mass());
    let tau = contact_time(mass, speed, 0.005, e).clamp(0.05e-3, 6e-3);
    let len = (tau * SR).round().max(1.0) as u16;
    // One is struck near its end, the other a third of the way along.
    let banks = vec![
        Bank::new(&a.modes(Drive::transverse(0.08), 40, seed), SR),
        Bank::new(&b.modes(Drive::transverse(0.33), 40, seed ^ 7), SR),
    ];
    let hits = (0..2)
        .map(|to| Hit {
            at: 0,
            len,
            to,
            step: false,
            amp: 1.0,
            direct: if to == 0 { 0.6 } else { 0.0 },
        })
        .collect();
    let mut voice = ModalVoice::new(banks, hits, 0.5, 0.6, 9_000.0, SR);
    let mut out = Vec::new();
    let mut block = [0.0f32; 128];
    while out.len() < (3.0 * SR) as usize {
        block.fill(0.0);
        let alive = voice.render(&mut block);
        out.extend_from_slice(&block);
        if !alive {
            break;
        }
    }
    out.resize((3.0 * SR) as usize, 0.0);
    out
}

/// A loose bar of `material` and `section`, `length` m long, landing on the
/// floor at `speed` m/s, as the game plays it.
fn drop(material: &Material, section: &Section, length: f64, speed: f32, seed: u32) -> Vec<f32> {
    let body = Body::member(material, section, length);
    let mut out = Vec::new();
    if let Some(mut voice) = impact_voice(&body, speed, body.mass(), 1.0, SR, seed) {
        let mut block = [0.0f32; 128];
        while out.len() < (3.0 * SR) as usize {
            block.fill(0.0);
            let alive = voice.render(&mut block);
            out.extend_from_slice(&block);
            if !alive {
                break;
            }
        }
    }
    out.resize((3.0 * SR) as usize, 0.0);
    out
}

/// A steel member `length` m long pulled apart at `load` of its capacity.
fn snap(section: &Section, length: f64, load: f32, seed: u32) -> Vec<f32> {
    let body = Body::member(&Material::steel(), section, length);
    let info = BreakInfo {
        body,
        cause: BreakCause::Rupture,
        at: 0.5,
        energy: capacity_energy(&body) * load * load,
        axial: 0.95,
    };
    let mut out = Vec::new();
    if let Some(mut voice) = break_voice(&info, 1.0, SR, seed) {
        let mut block = [0.0f32; 128];
        while out.len() < (3.0 * SR) as usize {
            block.fill(0.0);
            let alive = voice.render(&mut block);
            out.extend_from_slice(&block);
            if !alive {
                break;
            }
        }
    }
    out.resize((3.0 * SR) as usize, 0.0);
    out
}

fn main() {
    let dir = PathBuf::from(
        std::env::args()
            .nth(1)
            .unwrap_or_else(|| "audio-demo".into()),
    );
    std::fs::create_dir_all(&dir).expect("create output dir");
    let steel = Material::steel();
    let mut audio = Vec::new();
    // Hollow steel bars: three knocks, harder each time, then thinner and
    // shorter bars, then timber posts and carbon tubes for comparison.
    let tube = Section::tube(0.024, 0.003);
    for (i, speed) in [0.5f32, 1.5, 4.0].into_iter().enumerate() {
        audio.extend(knock(
            &steel,
            &tube,
            (1.5, 1.0),
            speed,
            1.1e11,
            3 + i as u32,
        ));
    }
    audio.extend(knock(
        &steel,
        &Section::tube(0.012, 0.002),
        (0.6, 0.4),
        2.0,
        1.1e11,
        11,
    ));
    audio.extend(knock(
        &steel,
        &Section::tube(0.05, 0.005),
        (3.0, 2.0),
        2.0,
        1.1e11,
        12,
    ));
    audio.extend(knock(
        &Material::timber(),
        &Section::rect(0.08, 0.08),
        (1.5, 1.0),
        2.0,
        1.5e9,
        13,
    ));
    audio.extend(knock(
        &Material::cfrp(),
        &Section::tube(0.02, 0.002),
        (1.5, 1.0),
        2.0,
        5.0e9,
        14,
    ));
    let peak = audio.iter().fold(0.0f32, |m, x| m.max(x.abs())).max(1e-6);
    for x in &mut audio {
        *x *= 0.7 / peak;
    }
    // The same bars let go above the floor.
    let mut drops = Vec::new();
    for (i, (section, length, speed)) in [
        (tube.clone(), 1.5, 3.0f32),
        (tube.clone(), 1.0, 5.0),
        (Section::tube(0.012, 0.002), 0.5, 4.0),
        (Section::tube(0.05, 0.005), 3.0, 4.0),
    ]
    .into_iter()
    .enumerate()
    {
        drops.extend(drop(&steel, &section, length, speed, 21 + i as u32));
    }
    drops.extend(drop(
        &Material::timber(),
        &Section::rect(0.08, 0.08),
        1.5,
        4.0,
        31,
    ));
    let peak = drops.iter().fold(0.0f32, |m, x| m.max(x.abs())).max(1e-6);
    for x in &mut drops {
        *x *= 0.7 / peak;
    }
    write_wav(&dir.join("drops.wav"), &drops, 1, SR as u32).expect("write wav");
    // And pulled apart in a frame.
    let mut snaps = Vec::new();
    for (i, (section, length)) in [
        (tube.clone(), 1.5),
        (tube.clone(), 3.0),
        (Section::tube(0.012, 0.002), 1.0),
        (Section::tube(0.05, 0.005), 4.0),
    ]
    .into_iter()
    .enumerate()
    {
        snaps.extend(snap(&section, length, 1.0, 41 + i as u32));
    }
    let peak = snaps.iter().fold(0.0f32, |m, x| m.max(x.abs())).max(1e-6);
    for x in &mut snaps {
        *x *= 0.7 / peak;
    }
    write_wav(&dir.join("snaps.wav"), &snaps, 1, SR as u32).expect("write wav");
    let path = dir.join("bars.wav");
    write_wav(&path, &audio, 1, SR as u32).expect("write wav");
    println!(
        "bars: {:.1} s -> {}",
        audio.len() as f32 / SR,
        path.display()
    );
}
