//! Renders listening samples of every sound family to WAV files.
//!
//! ```text
//! cargo run --release -p breakit-audio --example render_demo -- <out_dir> [scene...]
//! ```
//!
//! Scenes: stress, breaks, impacts, ambience, rigs, ui, collapse (all by
//! default). Each scene is a scripted sequence of the same commands the game
//! sends, so what you hear is what the game plays.

use std::path::{Path, PathBuf};

use breakit_audio::wav::write_wav;
use breakit_audio::{
    AmbientKind, BreakCause, Command, DamageState, MaterialKind, Mixer, Place, RigKind, UiSound,
};

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

fn stress() -> Script {
    // Each material in turn: load ramps from nothing past capacity.
    let mut s = Script::new(4.0 * 7.0);
    for (i, &m) in MATERIALS.iter().enumerate() {
        let t0 = i as f32 * 7.0;
        s.ramp(t0, t0 + 6.0, move |t| {
            let u = ((t - t0) / 5.0).min(1.2);
            Command::Stress {
                material: m,
                load: u,
                rate: 0.2,
                members: 6.0,
                place: place(-1.0 + i as f32 * 0.7),
            }
        });
        s.at(t0 + 6.0, Command::ResetStructure);
        // Damage onsets along the way.
        let state = match m {
            MaterialKind::Steel => DamageState::Yielded,
            MaterialKind::Timber | MaterialKind::Concrete => DamageState::Cracked,
            MaterialKind::Composite => DamageState::Delaminated,
        };
        s.at(
            t0 + 3.5,
            Command::Damage {
                material: m,
                state,
                size: 60.0,
                place: place(0.0),
            },
        );
        if m == MaterialKind::Concrete {
            s.at(
                t0 + 4.8,
                Command::Damage {
                    material: m,
                    state: DamageState::Crushed,
                    size: 300.0,
                    place: place(0.0),
                },
            );
        }
    }
    s
}

fn breaks() -> Script {
    let all: [(MaterialKind, BreakCause, f32); 12] = [
        (MaterialKind::Steel, BreakCause::Rupture, 40.0),
        (MaterialKind::Steel, BreakCause::Hinge, 40.0),
        (MaterialKind::Steel, BreakCause::Hinge, 400.0),
        (MaterialKind::Timber, BreakCause::Splinter, 30.0),
        (MaterialKind::Timber, BreakCause::TimberSplit, 30.0),
        (MaterialKind::Timber, BreakCause::Crushing, 30.0),
        (MaterialKind::Concrete, BreakCause::Brittle, 300.0),
        (MaterialKind::Concrete, BreakCause::ConcreteShear, 300.0),
        (MaterialKind::Concrete, BreakCause::Crushing, 300.0),
        (MaterialKind::Concrete, BreakCause::RebarRupture, 300.0),
        (MaterialKind::Composite, BreakCause::FibreRupture, 10.0),
        (MaterialKind::Composite, BreakCause::Delamination, 10.0),
    ];
    let mut s = Script::new(all.len() as f32 * 3.0 + 1.0);
    for (i, &(m, c, size)) in all.iter().enumerate() {
        s.at(
            0.3 + i as f32 * 3.0,
            Command::Break {
                material: m,
                cause: c,
                size,
                place: place(((i % 3) as f32 - 1.0) * 1.5),
            },
        );
    }
    s
}

fn impacts() -> Script {
    let mut s = Script::new(4.0 * 3.0 + 1.0);
    for (i, &m) in MATERIALS.iter().enumerate() {
        let t0 = 0.2 + i as f32 * 3.0;
        // A big piece, a bounce, then small bits.
        s.at(
            t0,
            Command::Impact {
                material: m,
                speed: 6.0,
                size: 40.0,
                place: place(0.0),
            },
        );
        s.at(
            t0 + 0.35,
            Command::Impact {
                material: m,
                speed: 2.0,
                size: 40.0,
                place: place(0.1),
            },
        );
        for k in 0..8 {
            let tk = t0 + 0.5 + k as f32 * 0.13 + (k * k) as f32 * 0.01;
            s.at(
                tk,
                Command::Impact {
                    material: m,
                    speed: 4.0 - k as f32 * 0.4,
                    size: 2.0,
                    place: place(-1.0 + k as f32 * 0.3),
                },
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
    s.ramp(1.0, 7.0, |t| {
        let u = ((t - 1.0) / 4.0).min(1.3);
        Command::Stress {
            material: MaterialKind::Steel,
            load: u,
            rate: 0.25,
            members: 8.0,
            place: place(-0.5),
        }
    });
    s.ramp(1.0, 5.3, |t| {
        let u = ((t - 1.0) / 3.5).min(1.2);
        Command::Stress {
            material: MaterialKind::Timber,
            load: u,
            rate: 0.3,
            members: 4.0,
            place: place(0.8),
        }
    });
    s.at(
        3.8,
        Command::Damage {
            material: MaterialKind::Timber,
            state: DamageState::Cracked,
            size: 30.0,
            place: place(0.8),
        },
    );
    s.at(
        4.2,
        Command::Damage {
            material: MaterialKind::Steel,
            state: DamageState::Yielded,
            size: 40.0,
            place: place(-0.5),
        },
    );
    s.at(
        5.3,
        Command::Break {
            material: MaterialKind::Timber,
            cause: BreakCause::Splinter,
            size: 30.0,
            place: place(0.8),
        },
    );
    s.at(
        5.8,
        Command::Break {
            material: MaterialKind::Steel,
            cause: BreakCause::Hinge,
            size: 40.0,
            place: place(-0.5),
        },
    );
    s.at(
        6.1,
        Command::Break {
            material: MaterialKind::Steel,
            cause: BreakCause::Rupture,
            size: 40.0,
            place: place(0.2),
        },
    );
    s.at(7.0, Command::ResetStructure);
    let hits = [
        (6.7, 7.0, 30.0),
        (6.75, 5.0, 3.0),
        (6.9, 3.0, 5.0),
        (7.05, 2.0, 30.0),
        (7.1, 4.0, 2.0),
        (7.3, 1.5, 2.0),
        (7.6, 1.0, 1.0),
    ];
    for (i, &(t, v, m)) in hits.iter().enumerate() {
        let mat = if i % 2 == 0 {
            MaterialKind::Steel
        } else {
            MaterialKind::Timber
        };
        s.at(
            t,
            Command::Impact {
                material: mat,
                speed: v,
                size: m,
                place: place(-1.0 + i as f32 * 0.3),
            },
        );
    }
    s.at(10.0, Command::Ui(UiSound::Fail));
    s
}

fn main() {
    let mut args = std::env::args().skip(1);
    let dir = PathBuf::from(args.next().unwrap_or_else(|| "audio-demo".into()));
    let wanted: Vec<String> = args.collect();
    std::fs::create_dir_all(&dir).expect("create output dir");
    let scenes: [(&str, fn() -> Script); 7] = [
        ("stress", stress),
        ("breaks", breaks),
        ("impacts", impacts),
        ("ambience", ambience),
        ("rigs", rigs),
        ("ui", ui),
        ("collapse", collapse),
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
