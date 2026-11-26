//! Scripted capture for headless checks: set `BREAKIT_CAPTURE=<dir>` and
//! the game walks the whole loop on its own (register, brief, build a
//! truss on sheet B-01, test, report), saves a screenshot of each screen,
//! pulls the camera back for a shot of the lab and exits. Used by agents and CI to see the game without a person at
//! the screen, e.g. `xvfb-run cargo run -p breakit-game`.
//!
//! `BREAKIT_NO_UI=1` hides the UI for clean scene shots.
//! `BREAKIT_THEME=<n>` starts with theme preset n, for comparing
//! looks. `BREAKIT_TRUSS=steel` builds the web in steel instead of timber;
//! `BREAKIT_TRUSS=weak` lays a bare timber plank instead, which breaks.
//!
//! `BREAKIT_FILM=<fps>` films the test: the clock advances exactly one
//! frame of that rate per rendered frame, however slowly the machine draws,
//! and every frame of the test is saved to `<dir>/film/NNNNN.png` (join them
//! with ffmpeg). `BREAKIT_FILM_CAM=distance,yaw,pitch[,focus_y]` places the
//! camera for it.

use std::path::PathBuf;

use bevy::app::AppExit;
use bevy::prelude::*;
use bevy::render::view::screenshot::{save_to_disk, Screenshot};

use crate::analysis::Simulation;
use crate::build::BuildTool;
use crate::camera::OrbitCamera;
use crate::design::{BeamKind, Design};
use crate::flow::{Screen, UiAction};
use crate::scenario::{Sheets, TestRecord};
use crate::ui::theme::Themes;

pub struct CapturePlugin;

impl Plugin for CapturePlugin {
    fn build(&self, app: &mut App) {
        if let Ok(theme) = std::env::var("BREAKIT_THEME") {
            if let Ok(i) = theme.parse::<usize>() {
                let mut t = Themes::default();
                t.active = i.min(t.presets.len() - 1);
                app.insert_resource(t);
            }
        }
        if let Ok(dir) = std::env::var("BREAKIT_CAPTURE") {
            let _ = std::fs::create_dir_all(&dir);
            if let Some(fps) = film_fps() {
                let _ = std::fs::create_dir_all(PathBuf::from(&dir).join("film"));
                app.insert_resource(bevy::time::TimeUpdateStrategy::ManualDuration(
                    std::time::Duration::from_secs_f64(1.0 / fps),
                ));
            }
            app.insert_resource(CaptureDir(PathBuf::from(dir)))
                .add_systems(Last, capture_script);
            if std::env::var("BREAKIT_NO_UI").is_ok() {
                app.add_systems(Last, hide_ui);
            }
        }
    }
}

#[derive(Resource)]
struct CaptureDir(PathBuf);

/// Frame rate of `BREAKIT_FILM`, if filming.
fn film_fps() -> Option<f64> {
    let fps: f64 = std::env::var("BREAKIT_FILM").ok()?.parse().ok()?;
    (fps > 0.0).then_some(fps)
}

/// `BREAKIT_NO_UI=1`: hides every UI root, for clean shots of the scene
/// (backgrounds for UI mock-ups).
fn hide_ui(
    mut roots: Query<&mut Visibility, (With<Node>, Without<ChildOf>)>,
    mut perf: ResMut<crate::perf::Perf>,
    mut window: Single<&mut Window>,
) {
    perf.visible = false;
    if window.resolution.width() != 1600.0 {
        window.resolution.set(1600.0, 900.0);
    }
    for mut v in &mut roots {
        if *v != Visibility::Hidden {
            *v = Visibility::Hidden;
        }
    }
}

#[derive(Default)]
struct Script {
    frame: u32,
    /// Frame the current step started on.
    since: u32,
    step: u32,
    /// Film frames saved, and how many of them since the first break.
    filmed: u32,
    after_break: u32,
}

#[allow(clippy::too_many_arguments)]
fn capture_script(
    mut commands: Commands,
    dir: Res<CaptureDir>,
    mut s: Local<Script>,
    screen: Res<State<Screen>>,
    sheets: Res<Sheets>,
    mut design: ResMut<Design>,
    mut tool: ResMut<BuildTool>,
    rec: Res<TestRecord>,
    sim: Option<Res<Simulation>>,
    mut actions: MessageWriter<UiAction>,
    mut exit: MessageWriter<AppExit>,
    mut orbit: Single<&mut OrbitCamera>,
) {
    s.frame += 1;
    let waited = s.frame - s.since;
    let shot = |commands: &mut Commands, name: &str| {
        let path = dir.0.join(name);
        commands
            .spawn(Screenshot::primary_window())
            .observe(save_to_disk(path));
    };
    let advance = |s: &mut Script| {
        s.step += 1;
        s.since = s.frame;
    };
    match s.step {
        // Shaders compile on the first frames.
        0 if waited >= 90 => {
            // The first bridge sheet; select it in case another is open.
            let i = sheets.all.iter().position(|s| s.id == "B-01").unwrap_or(0);
            actions.write(UiAction::SelectSheet(i));
            advance(&mut s);
        }
        1 if waited >= 20 => {
            shot(&mut commands, "1-register.png");
            advance(&mut s);
        }
        2 if waited >= 10 => {
            actions.write(UiAction::OpenBrief);
            advance(&mut s);
        }
        3 if waited >= 30 && *screen.get() == Screen::Brief => {
            shot(&mut commands, "2-brief.png");
            actions.write(UiAction::StartBuild);
            advance(&mut s);
        }
        4 if waited >= 20 && *screen.get() == Screen::Build => {
            let (chord, web) = match std::env::var("BREAKIT_TRUSS").as_deref() {
                Ok("steel") => (
                    BeamKind::part("steel-tube-m"),
                    BeamKind::part("steel-tube-m"),
                ),
                _ => (
                    BeamKind::part("steel-tube-m"),
                    BeamKind::part("timber-joist"),
                ),
            };
            if std::env::var("BREAKIT_TRUSS").as_deref() == Ok("weak") {
                build_plank(&mut design);
            } else {
                build_truss(&mut design, chord, web);
            }
            tool.selected_beam = Some(design.beams.len() - 1);
            advance(&mut s);
        }
        5 if waited >= 40 => {
            shot(&mut commands, "3-build.png");
            advance(&mut s);
        }
        6 if waited >= 10 => {
            if film_fps().is_some() {
                let cam = std::env::var("BREAKIT_FILM_CAM").unwrap_or_default();
                let v: Vec<f32> = cam.split(',').filter_map(|x| x.parse().ok()).collect();
                orbit.distance = v.first().copied().unwrap_or(9.5);
                orbit.yaw = v.get(1).copied().unwrap_or(0.55);
                orbit.pitch = v.get(2).copied().unwrap_or(0.3);
                orbit.focus.y = v.get(3).copied().unwrap_or(0.1);
            }
            actions.write(UiAction::RunTest);
            advance(&mut s);
        }
        7 if film_fps().is_some() => {
            // Every frame of the test, then a few seconds of the wreck.
            let name = format!("film/{:05}.png", s.filmed);
            shot(&mut commands, &name);
            s.filmed += 1;
            orbit.yaw += 0.0012;
            if rec.first_break.is_some() {
                s.after_break += 1;
            }
            let fps = film_fps().unwrap_or(30.0);
            if s.after_break as f64 > 9.0 * fps || *screen.get() == Screen::Report {
                s.step = 12;
                s.since = s.frame;
            }
        }
        7 => {
            let t = sim.map(|s| s.run().time()).unwrap_or(0.0);
            let after_break = rec.first_break.is_some_and(|b| t > b as f64 + 0.15);
            if *screen.get() == Screen::Test && (after_break || t > 7.0) {
                shot(&mut commands, "4-test.png");
                // Play the rest (and the proof test) fast.
                actions.write(UiAction::SimFaster);
                advance(&mut s);
            }
        }
        8 if *screen.get() == Screen::Report => advance(&mut s),
        9 if waited >= 30 => {
            shot(&mut commands, "5-report.png");
            advance(&mut s);
        }
        10 if waited >= 30 => {
            orbit.distance = 70.0;
            orbit.pitch = 0.22;
            orbit.yaw = 0.75;
            advance(&mut s);
        }
        11 if waited >= 40 => {
            shot(&mut commands, "6-lab.png");
            advance(&mut s);
        }
        12 if waited >= 30 => {
            exit.write(AppExit::Success);
        }
        _ => {}
    }
}

/// A Warren truss over sheet B-01's 6 m span: `chord` top and bottom,
/// `web` diagonals.
fn build_truss(d: &mut Design, chord: BeamKind, web: BeamKind) {
    let bottom: Vec<usize> = (-3..=3)
        .map(|x| d.add_node(Vec3::new(x as f32, 0.0, 0.0)))
        .collect();
    let top: Vec<usize> = (-3..3)
        .map(|x| d.add_node(Vec3::new(x as f32 + 0.5, 1.0, 0.0)))
        .collect();
    for w in bottom.windows(2) {
        d.add_beam(w[0], w[1], chord);
    }
    for w in top.windows(2) {
        d.add_beam(w[0], w[1], chord);
    }
    for (i, &t) in top.iter().enumerate() {
        d.add_beam(bottom[i], t, web);
        d.add_beam(t, bottom[i + 1], web);
    }
}

/// A bare timber plank across the span, which breaks under the vehicles.
fn build_plank(d: &mut Design) {
    let nodes: Vec<usize> = (-3..=3)
        .map(|x| d.add_node(Vec3::new(x as f32, 0.0, 0.0)))
        .collect();
    for w in nodes.windows(2) {
        d.add_beam(w[0], w[1], BeamKind::part("timber-joist"));
    }
}
