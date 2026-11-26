//! On-screen text: mode, controls, the physics read-out and, top right, the
//! frame rate. The frame rate is also logged every five seconds, so a run on
//! any machine leaves a performance record in its console output.

use bevy::diagnostic::{DiagnosticsStore, FrameTimeDiagnosticsPlugin};
use bevy::prelude::*;
use breakit_render::RenderStats;

use crate::analysis::{DeflectionScale, FrameState};
use crate::build::BuildTool;
use crate::design::Design;
use crate::Mode;

pub struct HudPlugin;

impl Plugin for HudPlugin {
    fn build(&self, app: &mut App) {
        if !app.is_plugin_added::<FrameTimeDiagnosticsPlugin>() {
            app.add_plugins(FrameTimeDiagnosticsPlugin::default());
        }
        app.add_systems(Startup, spawn_hud)
            .add_systems(Update, (update_hud, update_perf));
    }
}

#[derive(Component)]
struct HudText;

#[derive(Component)]
struct PerfText;

fn spawn_hud(mut commands: Commands) {
    commands.spawn((
        Text::new(""),
        TextFont::from_font_size(15.0),
        TextColor(Color::srgb(0.92, 0.92, 0.92)),
        Node {
            position_type: PositionType::Absolute,
            top: px(10),
            left: px(12),
            ..default()
        },
        HudText,
    ));
    commands.spawn((
        Text::new(""),
        TextFont::from_font_size(15.0),
        TextColor(Color::srgb(0.75, 0.9, 0.75)),
        TextLayout::justify(Justify::Right),
        Node {
            position_type: PositionType::Absolute,
            top: px(10),
            right: px(12),
            ..default()
        },
        PerfText,
    ));
}

/// One line of performance numbers: smoothed frame rate and frame time, how
/// many member instances are drawn, and the CPU cost of updating them.
pub fn perf_line(fps: f64, frame_ms: f64, stats: &RenderStats) -> String {
    format!(
        "{fps:.0} fps  {frame_ms:.1} ms\n{} members drawn, {} debris, {} particles\nvisual update {:.2} ms",
        stats.slots, stats.debris, stats.particles, stats.scene_update_ms
    )
}

/// Refreshes the frame-rate read-out four times a second and logs it every
/// five seconds.
fn update_perf(
    time: Res<Time<Real>>,
    diagnostics: Res<DiagnosticsStore>,
    stats: Res<RenderStats>,
    mode: Res<State<Mode>>,
    mut text: Single<&mut Text, With<PerfText>>,
    mut next: Local<(f32, f32)>,
) {
    let now = time.elapsed_secs();
    if now < next.0 {
        return;
    }
    next.0 = now + 0.25;
    let get = |path: &bevy::diagnostic::DiagnosticPath| {
        diagnostics
            .get(path)
            .and_then(|d| d.smoothed())
            .unwrap_or(0.0)
    };
    let fps = get(&FrameTimeDiagnosticsPlugin::FPS);
    let frame_ms = get(&FrameTimeDiagnosticsPlugin::FRAME_TIME);
    text.0 = perf_line(fps, frame_ms, &stats);
    if now >= next.1 {
        next.1 = now + 5.0;
        info!("perf [{:?}]: {}", mode.get(), text.0.replace('\n', " | "));
    }
}

const BUILD_HELP: &str = "\
LMB add node / select, clicks chain beams   Esc deselect
S support   F load   X delete node   M material   C clear
Tab plane   Q/E plane down/up   D deflection view
RMB orbit   MMB or Shift+RMB pan   wheel zoom
Space simulate";

const SIM_HELP: &str = "Space or R back to build   RMB orbit   wheel zoom";

fn update_hud(
    mode: Res<State<Mode>>,
    design: Res<Design>,
    state: Res<FrameState>,
    tool: Res<BuildTool>,
    scale: Res<DeflectionScale>,
    mut text: Single<&mut Text, With<HudText>>,
) {
    let mut s = String::new();
    match mode.get() {
        Mode::Build => {
            s.push_str("BUILD\n");
            s.push_str(&format!(
                "material: {}   plane: {} at {:+.1} m\n",
                tool.kind.label(),
                tool.plane.label(),
                tool.level
            ));
            s.push_str(&format!(
                "{} nodes, {} beams, {:.1} m of beam\n",
                design.nodes.len(),
                design.beams.len(),
                design.total_length()
            ));
            match &state.error {
                Some(e) => s.push_str(&format!("static check: {e}\n")),
                None if !design.beams.is_empty() => s.push_str(&format!(
                    "static check: max utilisation {:.0}%, max deflection {:.1} mm{}\n",
                    state.max_utilization() * 100.0,
                    state.max_displacement * 1000.0,
                    if scale.0 > 0.0 {
                        format!(" (drawn x{:.0})", scale.0)
                    } else {
                        String::new()
                    }
                )),
                None => {}
            }
            s.push('\n');
            s.push_str(BUILD_HELP);
        }
        Mode::Simulate => {
            s.push_str("SIMULATE (gravity + loads)\n");
            s.push_str(&format!(
                "max utilisation {:.0}%   broken beams {}\n",
                state.max_utilization() * 100.0,
                state.broken_count()
            ));
            for line in state.log.iter().rev().take(6) {
                s.push_str(line);
                s.push('\n');
            }
            s.push('\n');
            s.push_str(SIM_HELP);
        }
    }
    if text.0 != s {
        text.0 = s;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn perf_line_reports_rate_and_counts() {
        let stats = RenderStats {
            slots: 5100,
            remnants: 0,
            debris: 12,
            particles: 300,
            scene_update_ms: 0.41,
        };
        let line = perf_line(59.6, 16.78, &stats);
        assert!(line.starts_with("60 fps  16.8 ms"));
        assert!(line.contains("5100 members drawn"));
        assert!(line.contains("visual update 0.41 ms"));
    }
}
