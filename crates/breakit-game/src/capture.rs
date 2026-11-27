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
//! `BREAKIT_SHEET=F-01` walks the flume sheet with a jetty instead.

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
            let id = std::env::var("BREAKIT_SHEET").unwrap_or_else(|_| "B-01".into());
            let i = sheets.all.iter().position(|s| s.id == id).unwrap_or(0);
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
            if std::env::var("BREAKIT_SHEET").as_deref() == Ok("F-01") {
                build_jetty(&mut design, chord);
            } else if std::env::var("BREAKIT_TRUSS").as_deref() == Ok("weak") {
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
            actions.write(UiAction::RunTest);
            advance(&mut s);
        }
        7 => {
            let t = sim.map(|s| s.time()).unwrap_or(0.0);
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

/// A jetty for sheet F-01: four legs from the footings on the flume bed
/// to a deck frame under the pump house's feet.
fn build_jetty(d: &mut Design, kind: BeamKind) {
    let corners = [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)];
    let feet: Vec<usize> = corners
        .iter()
        .map(|&(x, z)| d.add_node(Vec3::new(x, 1.0, 0.5 * z)))
        .collect();
    let mid: Vec<usize> = corners
        .iter()
        .map(|&(x, z)| d.add_node(Vec3::new(1.25 * x, -1.1, z)))
        .collect();
    for (i, &(x, z)) in corners.iter().enumerate() {
        let base = d.add_node(Vec3::new(1.5 * x, -3.2, 1.5 * z));
        d.add_beam(base, mid[i], kind);
        d.add_beam(mid[i], feet[i], kind);
        d.add_beam(feet[i], feet[(i + 1) % 4], kind);
        d.add_beam(mid[i], mid[(i + 1) % 4], kind);
        d.add_beam(mid[i], feet[(i + 1) % 4], kind);
    }
    d.add_beam(feet[0], feet[2], kind);
}
