//! On-screen text: mode, controls and the physics read-out.

use bevy::prelude::*;

use crate::analysis::{DeflectionScale, FrameState};
use crate::build::BuildTool;
use crate::design::Design;
use crate::Mode;

pub struct HudPlugin;

impl Plugin for HudPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, spawn_hud)
            .add_systems(Update, update_hud);
    }
}

#[derive(Component)]
struct HudText;

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
}

const BUILD_HELP: &str = "\
LMB add node / select, clicks chain beams   Esc deselect
S support   F load   X delete node   M material   C clear
Tab plane   Q/E plane down/up   D deflection view   T table   F3 frame time
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
