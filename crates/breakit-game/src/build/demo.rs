//! Scripted build-mode walk-through for headless checks and PR clips: set
//! `BREAKIT_BUILD_CAPTURE=<dir>` and the game opens the tower sheet, builds
//! a braced frame with the mouse (moving the cursor, clicking, holding
//! Shift, right-clicking to cancel), saves a screenshot at each step and,
//! with `BREAKIT_BUILD_CLIP=1`, every other frame into `<dir>/clip/` for
//! `ffmpeg`. Then it exits.
//!
//! `xvfb-run -a -s "-screen 0 1280x720x24" cargo run --release -p breakit-game`

use std::path::PathBuf;

use bevy::app::AppExit;
use bevy::input::InputSystems;
use bevy::prelude::*;
use bevy::render::view::screenshot::{save_to_disk, Screenshot};
use bevy::window::PrimaryWindow;

use super::Tool;
use crate::camera::OrbitCamera;
use crate::flow::{Screen, UiAction};
use crate::scenario::Sheets;

pub struct BuildDemoPlugin;

impl Plugin for BuildDemoPlugin {
    fn build(&self, app: &mut App) {
        if let Ok(dir) = std::env::var("BREAKIT_BUILD_CAPTURE") {
            let dir = PathBuf::from(dir);
            let _ = std::fs::create_dir_all(dir.join("clip"));
            app.insert_resource(DemoDir {
                dir,
                clip: std::env::var("BREAKIT_BUILD_CLIP").is_ok(),
            })
            .add_systems(PreUpdate, run_demo.after(InputSystems));
        }
    }
}

#[derive(Resource)]
struct DemoDir {
    dir: PathBuf,
    clip: bool,
}

#[derive(Clone, Copy, Debug)]
enum Step {
    Wait(u32),
    /// Camera yaw, pitch, distance, focus height.
    Cam(f32, f32, f32, f32),
    /// Glide the cursor to a world point.
    Hover(Vec3),
    Click,
    RightClick,
    Shift(bool),
    SetTool(Tool),
    Shot(&'static str),
}

fn script() -> Vec<Step> {
    use Step::*;
    let v = Vec3::new;
    let (a, b, c, d) = (
        v(-1.0, 0.0, -1.0),
        v(1.0, 0.0, -1.0),
        v(1.0, 0.0, 1.0),
        v(-1.0, 0.0, 1.0),
    );
    let up = |p: Vec3| p + Vec3::Y * 2.0;
    let mut s = vec![Cam(0.55, 0.32, 11.0, 1.2), Wait(20), Hover(a), Wait(25)];
    s.push(Shot("1-foundation.png"));
    // First column: click the foundation, hold Shift, go up.
    s.extend([Click, Wait(8), Shift(true), Hover(up(a)), Wait(20)]);
    s.push(Shot("2-shift-vertical.png"));
    s.extend([Click, Shift(false), Wait(8), RightClick, Wait(10)]);
    // The other columns, each tied to the last.
    for (foot, prev) in [(b, a), (c, b), (d, c)] {
        s.extend([Hover(foot), Click, Wait(6), Shift(true), Hover(up(foot))]);
        s.extend([
            Wait(6),
            Click,
            Shift(false),
            Wait(6),
            Hover(up(prev)),
            Wait(6),
        ]);
        s.extend([Click, Wait(6), RightClick, Wait(8)]);
    }
    s.extend([
        Hover(up(d)),
        Click,
        Wait(6),
        Hover(up(a)),
        Wait(6),
        Click,
        RightClick,
    ]);
    // Turn to the side: the work plane turns with the view.
    s.extend([
        Cam(1.35, 0.25, 11.0, 1.2),
        Wait(25),
        Hover(b),
        Click,
        Wait(6),
    ]);
    s.extend([Hover(v(1.0, 1.0, 0.0)), Wait(25)]);
    s.push(Shot("3-side-plane.png"));
    s.extend([Hover(up(c)), Wait(6), Click, Wait(6), RightClick, Wait(10)]);
    // Select a member: it is outlined over the scene.
    s.extend([Cam(0.55, 0.32, 11.0, 1.2), Wait(25)]);
    s.extend([
        Hover(v(0.0, 2.0, -1.0)),
        Wait(10),
        Click,
        Hover(v(0.4, 1.0, 1.5)),
        Wait(25),
    ]);
    s.push(Shot("4-selected.png"));
    // The support tool says where supports can go.
    s.extend([RightClick, SetTool(Tool::Support), Hover(up(a)), Wait(25)]);
    s.push(Shot("5-support.png"));
    s.extend([Hover(a), Wait(25)]);
    s.push(Shot("6-support-ok.png"));
    s.push(Wait(20));
    s
}

#[derive(Default)]
struct Run {
    frame: u32,
    started: bool,
    step: usize,
    wait: u32,
    /// Cursor glide: from, to (pixels), frames done.
    glide: Option<(Vec2, Vec2, u32)>,
    release: Option<MouseButton>,
    clip_frame: u32,
    steps: Vec<Step>,
}

const GLIDE_FRAMES: u32 = 10;

/// Whether this frame runs a shot step.
fn shot_now(run: &Run) -> bool {
    run.wait == 0 && run.glide.is_none() && matches!(run.steps.get(run.step), Some(Step::Shot(_)))
}

#[allow(clippy::too_many_arguments)]
fn run_demo(
    mut commands: Commands,
    dir: Res<DemoDir>,
    mut run: Local<Run>,
    screen: Res<State<Screen>>,
    sheets: Res<Sheets>,
    mut actions: MessageWriter<UiAction>,
    mut exit: MessageWriter<AppExit>,
    mut mouse: ResMut<ButtonInput<MouseButton>>,
    mut keys: ResMut<ButtonInput<KeyCode>>,
    mut window: Single<&mut Window, With<PrimaryWindow>>,
    camera: Single<(&Camera, &GlobalTransform, &mut OrbitCamera)>,
) {
    if run.steps.is_empty() {
        run.steps = script();
    }
    run.frame += 1;
    if let Some(b) = run.release.take() {
        mouse.release(b);
    }
    // Shaders compile on the first frames, then open the tower sheet.
    if !run.started {
        match run.frame {
            90 => {
                let i = sheets.all.iter().position(|s| s.id == "Q-01").unwrap_or(0);
                actions.write(UiAction::SelectSheet(i));
            }
            100 => {
                actions.write(UiAction::OpenBrief);
            }
            130 => {
                actions.write(UiAction::StartBuild);
            }
            f if f > 150 && *screen.get() == Screen::Build => run.started = true,
            _ => {}
        }
        return;
    }
    // One screenshot a frame: a step's shot, else a clip frame.
    if dir.clip && run.frame.is_multiple_of(2) && !shot_now(&run) {
        let path = dir
            .dir
            .join("clip")
            .join(format!("f{:04}.png", run.clip_frame));
        run.clip_frame += 1;
        commands
            .spawn(Screenshot::primary_window())
            .observe(save_to_disk(path));
    }
    let (cam, cam_tf, mut orbit) = camera.into_inner();

    if let Some((from, to, n)) = run.glide {
        let t = (n + 1) as f32 / GLIDE_FRAMES as f32;
        let e = t * t * (3.0 - 2.0 * t);
        window.set_cursor_position(Some(from.lerp(to, e)));
        run.glide = (n + 1 < GLIDE_FRAMES).then_some((from, to, n + 1));
        return;
    }
    if run.wait > 0 {
        run.wait -= 1;
        return;
    }
    let Some(step) = run.steps.get(run.step).copied() else {
        exit.write(AppExit::Success);
        return;
    };
    run.step += 1;
    match step {
        Step::Wait(n) => run.wait = n,
        Step::Cam(yaw, pitch, dist, y) => {
            orbit.yaw = yaw;
            orbit.pitch = pitch;
            orbit.distance = dist;
            orbit.focus = Vec3::new(0.0, y, 0.0);
            run.wait = 3;
        }
        Step::Hover(p) => {
            if let Ok(to) = cam.world_to_viewport(cam_tf, p) {
                let from = window.cursor_position().unwrap_or(to);
                run.glide = Some((from, to, 0));
            }
        }
        Step::Click => {
            mouse.press(MouseButton::Left);
            run.release = Some(MouseButton::Left);
        }
        Step::RightClick => {
            mouse.press(MouseButton::Right);
            run.release = Some(MouseButton::Right);
        }
        Step::Shift(on) => {
            if on {
                keys.press(KeyCode::ShiftLeft);
            } else {
                keys.release(KeyCode::ShiftLeft);
            }
        }
        Step::SetTool(t) => {
            actions.write(UiAction::SetTool(t));
        }
        Step::Shot(name) => {
            commands
                .spawn(Screenshot::primary_window())
                .observe(save_to_disk(dir.dir.join(name)));
            run.wait = 4;
        }
    }
}
