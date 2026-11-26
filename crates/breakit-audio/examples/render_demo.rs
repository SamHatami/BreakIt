//! Renders listening samples of every sound family to WAV files.
//!
//! ```text
//! cargo run --release -p breakit-audio --example render_demo -- <out_dir> [scene...]
//! ```
//!
//! Scenes: stress, breaks, impacts, geometry, ambience, rigs, ui, collapse, and one
//! story per material: steel, timber, concrete, composite (all by default).
//! Each scene is a scripted sequence of the same commands the game
//! sends, so what you hear is what the game plays.

use std::path::{Path, PathBuf};

use breakit_audio::material::capacity_energy;
use breakit_audio::wav::write_wav;
use breakit_audio::{
    AmbientKind, Body, BreakCause, BreakInfo, Command, DamageState, MaterialKind, Mixer, Place,
    RigKind, UiSound,
};
use breakit_core::{Material, Section};

const SR: f32 = 48_000.0;

const MATERIALS: [MaterialKind; 4] = [
    MaterialKind::Steel,
    MaterialKind::Timber,
    MaterialKind::Concrete,
    MaterialKind::Composite,
];

/// A timeline of commands, rendered in 10 ms steps.
struct Script {
    events: Vec<(f32, Command)>,
    /// Commands re-sent every step while t is in [start, end): continuous
    /// parameters as a function of time.
    ramps: Vec<(f32, f32, Box<dyn Fn(f32) -> Command>)>,
    length: f32,
}

impl Script {
    fn new(length: f32) -> Self {
        Self {
            events: Vec::new(),
            ramps: Vec::new(),
            length,
        }
    }

    fn at(&mut self, t: f32, c: Command) -> &mut Self {
        self.events.push((t, c));
        self
    }

    fn ramp(&mut self, start: f32, end: f32, f: impl Fn(f32) -> Command + 'static) -> &mut Self {
        self.ramps.push((start, end, Box::new(f)));
        self
    }

    fn render(mut self) -> Vec<f32> {
        let mut m = Mixer::new(SR);
        m.handle(Command::Listener {
            position: [0.0, 1.5, 6.0],
            right: [1.0, 0.0, 0.0],
            up: [0.0, 1.0, 0.0],
        });
        self.events.sort_by(|a, b| a.0.total_cmp(&b.0));
        let step = 0.01;
        let mut out = Vec::new();
        let mut next = 0;
        let mut t = 0.0;
        while t < self.length {
            while next < self.events.len() && self.events[next].0 <= t {
                m.handle(self.events[next].1.clone());
                next += 1;
            }
            for (a, b, f) in &self.ramps {
                if t >= *a && t < *b {
                    m.handle(f(t));
                }
            }
            out.extend(m.render_seconds(step));
            t += step;
        }
        out
    }
}

fn place(x: f32) -> Place {
    Place::at([x, 1.0, 0.0])
}

/// A typical member of a material, `length` metres long.
fn member(m: MaterialKind, length: f32) -> Body {
    Body::typical(m, length)
}

/// A break that releases `load`² of the member's capacity energy.
fn snap(body: Body, cause: BreakCause, at: f32, axial: f32, load: f32, x: f32) -> Command {
    Command::Break {
        info: BreakInfo {
            body,
            cause,
            at,
            energy: capacity_energy(&body) * load * load,
            axial,
        },
        place: place(x),
    }
}

fn onset(body: Body, state: DamageState, load: f32, x: f32) -> Command {
    Command::Damage {
        body,
        state,
        energy: capacity_energy(&body) * load * load,
        place: place(x),
    }
}

/// A piece of bounding radius `radius` landing at `speed`.
fn land(m: MaterialKind, radius: f32, speed: f32, x: f32) -> Command {
    let body = Body::fragment(m, radius);
    Command::Impact {
        body,
        speed,
        mass: body.mass(),
        place: place(x),
    }
}

/// Loads member `id` from `load.0` to `load.1` over `span`; damage follows
/// the highest load so far, as the solver reports it.
fn load_ramp(s: &mut Script, id: u32, body: Body, span: (f32, f32), load: (f32, f32), x: f32) {
    let ((t0, t1), (u0, u1)) = (span, load);
    s.ramp(t0, t1, move |t| {
        let u = u0 + (u1 - u0) * (t - t0) / (t1 - t0);
        Command::Strain {
            id,
            body,
            utilization: u,
            damage: u.max(u0).min(1.0),
            place: place(x),
        }
    });
}

fn stress() -> Script {
    // Each material in turn: one member loaded from nothing to capacity in
    // five seconds, eased off, and loaded again (quieter: nothing new
    // cracks until the old peak is passed).
    let mut s = Script::new(4.0 * 10.0);
    for (i, &m) in MATERIALS.iter().enumerate() {
        let t0 = i as f32 * 10.0;
        let body = member(m, 3.0);
        let x = -1.0 + i as f32 * 0.7;
        load_ramp(&mut s, i as u32, body, (t0, t0 + 5.0), (0.0, 1.0), x);
        s.ramp(t0 + 5.0, t0 + 6.5, move |t| Command::Strain {
            id: i as u32,
            body,
            utilization: 1.0 - 0.4 * (t - t0 - 5.0),
            damage: 1.0,
            place: place(x),
        });
        s.ramp(t0 + 6.5, t0 + 8.0, move |t| Command::Strain {
            id: i as u32,
            body,
            utilization: 0.4 + 0.4 * (t - t0 - 6.5),
            damage: 1.0,
            place: place(x),
        });
        s.at(t0 + 9.5, Command::ResetStructure);
    }
    s
}

fn breaks() -> Script {
    // (material, cause, where along the member, axial share of the energy)
    let all: [(MaterialKind, BreakCause, f32, f32); 13] = [
        (MaterialKind::Steel, BreakCause::Rupture, 0.5, 0.95),
        (MaterialKind::Steel, BreakCause::Hinge, 0.1, 0.1),
        (MaterialKind::Steel, BreakCause::Connection, 0.0, 0.6),
        (MaterialKind::Timber, BreakCause::Splinter, 0.5, 0.2),
        (MaterialKind::Timber, BreakCause::TimberSplit, 0.5, 0.1),
        (MaterialKind::Timber, BreakCause::Crushing, 0.5, 0.9),
        (MaterialKind::Concrete, BreakCause::Brittle, 0.5, 0.2),
        (MaterialKind::Concrete, BreakCause::ConcreteShear, 0.2, 0.1),
        (MaterialKind::Concrete, BreakCause::Crushing, 0.5, 0.9),
        (MaterialKind::Concrete, BreakCause::RebarRupture, 0.5, 0.2),
        (MaterialKind::Composite, BreakCause::FibreRupture, 0.5, 0.9),
        (MaterialKind::Composite, BreakCause::Delamination, 0.5, 0.1),
        (MaterialKind::Composite, BreakCause::Connection, 1.0, 0.5),
    ];
    let mut s = Script::new(all.len() as f32 * 3.0 + 1.0);
    for (i, &(m, c, at, axial)) in all.iter().enumerate() {
        s.at(
            0.3 + i as f32 * 3.0,
            snap(
                member(m, 3.0),
                c,
                at,
                axial,
                1.0,
                ((i % 3) as f32 - 1.0) * 1.5,
            ),
        );
    }
    s
}

/// The same events on members of different size: nothing changes but the
/// numbers the solver already has. A steel tie snapping at 1, 3 and 8 m; a
/// thin and a heavy steel tube; a timber post and a timber plank; pieces of
/// 0.1, 0.4 and 1.5 m landing.
fn geometry() -> Script {
    let mut s = Script::new(34.0);
    let mut t = 0.3;
    for len in [1.0, 3.0, 8.0] {
        let body = member(MaterialKind::Steel, len);
        s.at(t, snap(body, BreakCause::Rupture, 0.5, 0.95, 1.0, 0.0));
        t += 3.0;
    }
    for (ro, wall) in [(0.02, 0.002), (0.12, 0.012)] {
        let body = Body::member(&Material::steel(), &Section::tube(ro, wall), 3.0);
        s.at(t, snap(body, BreakCause::Hinge, 0.5, 0.1, 1.0, 0.0));
        t += 3.0;
    }
    for (b, h) in [(0.2, 0.2), (0.2, 0.03)] {
        let body = Body::member(&Material::timber(), &Section::rect(b, h), 3.0);
        s.at(t, snap(body, BreakCause::Splinter, 0.5, 0.2, 1.0, 0.0));
        t += 3.0;
    }
    for m in [
        MaterialKind::Steel,
        MaterialKind::Timber,
        MaterialKind::Concrete,
    ] {
        for r in [0.05, 0.2, 0.75] {
            s.at(t, land(m, r, 4.0, 0.0));
            t += 1.4;
        }
    }
    s
}

fn impacts() -> Script {
    let mut s = Script::new(4.0 * 3.0 + 1.0);
    for (i, &m) in MATERIALS.iter().enumerate() {
        let t0 = 0.2 + i as f32 * 3.0;
        // A big piece, then small bits.
        s.at(t0, land(m, 0.6, 6.0, 0.0));
        for k in 0..8 {
            let tk = t0 + 0.6 + k as f32 * 0.13 + (k * k) as f32 * 0.01;
            s.at(
                tk,
                land(
                    m,
                    0.12 - 0.01 * k as f32,
                    4.0 - k as f32 * 0.4,
                    -1.0 + k as f32 * 0.3,
                ),
            );
        }
    }
    s
}

fn ambience() -> Script {
    let mut s = Script::new(20.0);
    s.at(
        0.0,
        Command::Ambient {
            id: 0,
            kind: AmbientKind::Room,
            level: 1.0,
            place: None,
        },
    );
    s.at(
        0.0,
        Command::Ambient {
            id: 1,
            kind: AmbientKind::Hvac,
            level: 1.0,
            place: Some(Place::scaled([-300.0, 200.0, -200.0], 0.05)),
        },
    );
    s.at(
        0.0,
        Command::Ambient {
            id: 2,
            kind: AmbientKind::ElectricalHum,
            level: 1.0,
            place: Some(Place::scaled([100.0, 220.0, 0.0], 0.05)),
        },
    );
    s.at(
        0.0,
        Command::Ambient {
            id: 3,
            kind: AmbientKind::Machinery,
            level: 1.0,
            place: Some(Place::scaled([400.0, 0.0, -300.0], 0.05)),
        },
    );
    s
}

fn rigs() -> Script {
    let mut s = Script::new(32.0);
    // Power unit idles under everything.
    s.ramp(0.0, 31.0, |_| Command::Rig {
        id: 0,
        kind: RigKind::PowerUnit,
        drive: 0.4,
        speed: 0.0,
        place: Place::scaled([-80.0, -15.0, -60.0], 0.05),
    });
    // Fan wall spins up to a gusty 25 m/s and back down.
    s.ramp(0.0, 8.0, |t| {
        let up = (t / 3.0).min(1.0) * (1.0 - ((t - 6.0) / 2.0).clamp(0.0, 1.0));
        let gust = 1.0 + 0.15 * (t * 1.3).sin() * (t * 0.7).sin();
        Command::Rig {
            id: 1,
            kind: RigKind::WindFan,
            drive: up,
            speed: 25.0 * up * gust,
            place: Place::at([-4.0, 1.0, -1.0]),
        }
    });
    // Shake table: a quake-like burst of shaking.
    s.ramp(8.0, 16.0, |t| {
        let tt = t - 8.0;
        let env = (tt / 1.0).min(1.0) * (-(tt - 2.0).max(0.0) / 2.5).exp();
        let v = env * 0.4 * ((tt * 11.0).sin() + 0.6 * (tt * 17.3).sin() + 0.3 * (tt * 29.0).sin());
        Command::Rig {
            id: 2,
            kind: RigKind::ShakeTable,
            drive: env,
            speed: v,
            place: Place::at([0.0, -0.3, 0.0]),
        }
    });
    // Wave paddle: regular strokes, 0.7 Hz.
    s.ramp(16.0, 24.0, |t| {
        let tt = t - 16.0;
        let env = (tt / 1.5).min(1.0);
        Command::Rig {
            id: 3,
            kind: RigKind::WavePaddle,
            drive: env * 0.8,
            speed: env * 0.6 * (tt * 0.7 * std::f32::consts::TAU).cos(),
            place: Place::at([3.0, 0.0, -2.0]),
        }
    });
    // Cart rolls across, then the winch lifts a drop weight.
    s.ramp(24.0, 28.0, |t| {
        let tt = t - 24.0;
        let v = 2.0 * (tt / 0.8).min(1.0) * (1.0 - ((tt - 3.0) / 1.0).clamp(0.0, 1.0));
        Command::Rig {
            id: 4,
            kind: RigKind::Cart,
            drive: 0.7,
            speed: v,
            place: Place::at([-3.0 + 1.6 * tt, 0.2, 0.0]),
        }
    });
    s.ramp(28.0, 31.0, |t| {
        let tt = t - 28.0;
        let v = 0.5 * (tt / 0.5).min(1.0) * (1.0 - ((tt - 2.3) / 0.5).clamp(0.0, 1.0));
        Command::Rig {
            id: 5,
            kind: RigKind::Winch,
            drive: 0.8,
            speed: v,
            place: Place::at([0.0, 5.0, 0.0]),
        }
    });
    s
}

fn ui() -> Script {
    let seq = [
        UiSound::Hover,
        UiSound::Hover,
        UiSound::PlaceNode,
        UiSound::Hover,
        UiSound::PlaceNode,
        UiSound::PlaceBeam,
        UiSound::Select,
        UiSound::Support,
        UiSound::Delete,
        UiSound::Undo,
        UiSound::Invalid,
        UiSound::TestStart,
        UiSound::TestStop,
        UiSound::Pass,
        UiSound::Fail,
    ];
    let mut s = Script::new(seq.len() as f32 * 0.9 + 2.0);
    for (i, &u) in seq.iter().enumerate() {
        s.at(0.2 + i as f32 * 0.9, Command::Ui(u));
    }
    s
}

/// A short scripted collapse: a steel and timber frame loads up, yields,
/// breaks, and the pieces land.
fn collapse() -> Script {
    let mut s = Script::new(12.0);
    s.at(
        0.0,
        Command::Ambient {
            id: 0,
            kind: AmbientKind::Room,
            level: 1.0,
            place: None,
        },
    );
    s.at(
        0.0,
        Command::Ambient {
            id: 1,
            kind: AmbientKind::Hvac,
            level: 0.8,
            place: Some(Place::scaled([-300.0, 200.0, -200.0], 0.05)),
        },
    );
    s.at(0.0, Command::Ui(UiSound::TestStart));
    // A small mixed truss taken to failure: six members load up at
    // different rates, a timber diagonal goes first, its load lands on the
    // steel, a chord hinges and a tie snaps.
    let steel = member(MaterialKind::Steel, 3.0);
    let timber = member(MaterialKind::Timber, 2.5);
    load_ramp(&mut s, 0, timber, (1.0, 5.3), (0.1, 1.0), 0.8);
    load_ramp(&mut s, 1, timber, (1.0, 6.5), (0.1, 0.9), 0.2);
    load_ramp(&mut s, 2, steel, (1.0, 5.3), (0.1, 0.6), -0.5);
    load_ramp(&mut s, 2, steel, (5.3, 5.8), (0.6, 1.0), -0.5);
    load_ramp(&mut s, 3, steel, (1.0, 5.8), (0.1, 0.5), 0.2);
    load_ramp(&mut s, 3, steel, (5.8, 6.1), (0.5, 1.0), 0.2);
    load_ramp(&mut s, 4, steel, (1.0, 6.5), (0.0, 0.7), -1.2);
    s.at(3.8, onset(timber, DamageState::Cracked, 0.8, 0.8));
    s.at(4.9, onset(steel, DamageState::Yielded, 0.9, -0.5));
    s.at(5.3, snap(timber, BreakCause::Splinter, 0.45, 0.3, 1.0, 0.8));
    s.at(5.8, snap(steel, BreakCause::Hinge, 0.1, 0.1, 1.0, -0.5));
    s.at(6.1, snap(steel, BreakCause::Rupture, 0.6, 0.95, 1.0, 0.2));
    s.at(
        6.4,
        snap(timber, BreakCause::Connection, 1.0, 0.4, 0.8, 0.2),
    );
    s.at(7.0, Command::ResetStructure);
    let hits = [
        (6.7, 7.0, 0.7),
        (6.75, 5.0, 0.2),
        (6.9, 3.0, 0.3),
        (7.05, 5.0, 0.8),
        (7.1, 4.0, 0.15),
        (7.3, 2.5, 0.6),
        (7.45, 3.0, 0.1),
        (7.6, 2.0, 0.25),
        (7.9, 1.5, 0.5),
    ];
    for (i, &(t, v, r)) in hits.iter().enumerate() {
        let mat = if i % 2 == 0 {
            MaterialKind::Steel
        } else {
            MaterialKind::Timber
        };
        s.at(t, land(mat, r, v, -1.0 + i as f32 * 0.3));
    }
    s.at(10.0, Command::Ui(UiSound::Fail));
    s
}

/// One material from loaded to broken: two members load up, damage starts,
/// they break in that material's typical ways, the pieces land.
fn material_story(m: MaterialKind) -> Script {
    let (onset_state, first, second, piece) = match m {
        MaterialKind::Steel => (
            DamageState::Yielded,
            (BreakCause::Hinge, 0.1, 0.1),
            (BreakCause::Rupture, 0.5, 0.95),
            0.7,
        ),
        MaterialKind::Timber => (
            DamageState::Cracked,
            (BreakCause::Splinter, 0.5, 0.2),
            (BreakCause::TimberSplit, 0.4, 0.1),
            0.6,
        ),
        MaterialKind::Concrete => (
            DamageState::Cracked,
            (BreakCause::ConcreteShear, 0.2, 0.1),
            (BreakCause::Crushing, 0.5, 0.9),
            0.4,
        ),
        MaterialKind::Composite => (
            DamageState::Delaminated,
            (BreakCause::FibreRupture, 0.5, 0.9),
            (BreakCause::Delamination, 0.5, 0.1),
            0.5,
        ),
    };
    let body = member(m, 3.0);
    let mut s = Script::new(9.0);
    s.at(
        0.0,
        Command::Ambient {
            id: 0,
            kind: AmbientKind::Room,
            level: 1.0,
            place: None,
        },
    );
    load_ramp(&mut s, 0, body, (0.0, 3.8), (0.1, 1.0), -0.3);
    load_ramp(&mut s, 1, body, (0.0, 3.8), (0.1, 0.7), 0.6);
    load_ramp(&mut s, 1, body, (3.8, 4.6), (0.7, 1.0), 0.6);
    s.at(2.3, onset(body, onset_state, 0.7, -0.3));
    if m == MaterialKind::Concrete {
        s.at(3.2, onset(body, DamageState::Crushed, 0.9, 0.4));
    }
    s.at(3.8, snap(body, first.0, first.1, first.2, 1.0, -0.3));
    s.at(4.6, snap(body, second.0, second.1, second.2, 1.0, 0.6));
    s.at(5.0, Command::ResetStructure);
    let hits = [
        (5.3, 6.0, 1.0),
        (5.45, 4.0, 0.3),
        (5.7, 2.5, 1.0),
        (5.8, 3.5, 0.2),
        (6.1, 1.5, 0.3),
        (6.4, 1.0, 0.15),
    ];
    for (i, &(t, v, frac)) in hits.iter().enumerate() {
        s.at(t, land(m, piece * frac, v, -1.0 + 0.4 * i as f32));
    }
    s
}

fn steel() -> Script {
    material_story(MaterialKind::Steel)
}
fn timber() -> Script {
    material_story(MaterialKind::Timber)
}
fn concrete() -> Script {
    material_story(MaterialKind::Concrete)
}
fn composite() -> Script {
    material_story(MaterialKind::Composite)
}

fn main() {
    let mut args = std::env::args().skip(1);
    let dir = PathBuf::from(args.next().unwrap_or_else(|| "audio-demo".into()));
    let wanted: Vec<String> = args.collect();
    std::fs::create_dir_all(&dir).expect("create output dir");
    let scenes: [(&str, fn() -> Script); 12] = [
        ("stress", stress),
        ("breaks", breaks),
        ("impacts", impacts),
        ("geometry", geometry),
        ("ambience", ambience),
        ("rigs", rigs),
        ("ui", ui),
        ("collapse", collapse),
        ("steel", steel),
        ("timber", timber),
        ("concrete", concrete),
        ("composite", composite),
    ];
    for (name, make) in scenes {
        if !wanted.is_empty() && !wanted.iter().any(|w| w == name) {
            continue;
        }
        let t = std::time::Instant::now();
        let script = make();
        let length = script.length;
        let audio = script.render();
        let path: &Path = &dir.join(format!("{name}.wav"));
        write_wav(path, &audio, 2, SR as u32).expect("write wav");
        let peak = audio.iter().fold(0.0f32, |m, x| m.max(x.abs()));
        let rms = (audio.iter().map(|x| x * x).sum::<f32>() / audio.len().max(1) as f32).sqrt();
        println!(
            "{name:>9}: {length:5.1} s rendered in {:6.1} ms ({:5.1}x real time), peak {:.2}, rms {:.3} -> {}",
            t.elapsed().as_secs_f32() * 1e3,
            length / t.elapsed().as_secs_f32(),
            peak,
            rms,
            path.display()
        );
    }
}
