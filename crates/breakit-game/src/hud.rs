//! On-screen text: mode, controls and the physics read-out.

use bevy::prelude::*;

use breakit_scenario::{Phase, PlayMode};

use crate::analysis::{DeflectionScale, FrameState, Simulation};
use crate::build::BuildTool;
use crate::design::Design;
use crate::test_scenario::{ActiveScenario, LastReport, ScenarioCheck};
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
N next scenario   B challenge / sandbox
RMB orbit   MMB or Shift+RMB pan   wheel zoom
Space run the test";

const SIM_HELP: &str = "Space or R back to build   RMB orbit   wheel zoom";

#[allow(clippy::too_many_arguments)]
fn update_hud(
    mode: Res<State<Mode>>,
    design: Res<Design>,
    state: Res<FrameState>,
    tool: Res<BuildTool>,
    scale: Res<DeflectionScale>,
    active: Res<ActiveScenario>,
    check: Res<ScenarioCheck>,
    report: Res<LastReport>,
    sim: Option<Res<Simulation>>,
    mut text: Single<&mut Text, With<HudText>>,
) {
    let mut s = String::new();
    match active.get() {
        Some(sc) => {
            s.push_str(&format!(
                "{} {}{}\n",
                sc.sheet,
                sc.title,
                match active.mode {
                    PlayMode::Challenge => "",
                    PlayMode::Sandbox => "  (sandbox)",
                }
            ));
        }
        None => s.push_str("free play (no scenario)\n"),
    }
    match mode.get() {
        Mode::Build => {
            if let Some(sc) = active.get() {
                s.push_str(&format!("{}\n", sc.brief));
                let budget = sc
                    .limits
                    .budget
                    .map(|b| format!(" / {b:.0}"))
                    .unwrap_or_default();
                s.push_str(&format!(
                    "cost {:.0}{budget} (full marks at {:.0})   weight {:.0} kg\n",
                    check.bill.total(),
                    sc.scoring.target_cost,
                    check.bill.mass
                ));
                if check.blocks_test() {
                    s.push_str("can't run the test yet:\n");
                    for v in check.violations.iter().take(5) {
                        s.push_str(&format!("  {v}\n"));
                    }
                    if check.violations.len() > 5 {
                        s.push_str(&format!("  and {} more\n", check.violations.len() - 5));
                    }
                }
            }
            if let Some(r) = &report.0 {
                s.push_str("\nlast test:\n");
                s.push_str(&r.summary());
            }
            s.push_str("\nBUILD\n");
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
            match sim.as_ref().and_then(|s| s.run()) {
                Some(run) => {
                    let phase = match run.phase() {
                        Phase::Settling => "self weight settling".to_string(),
                        Phase::Test => {
                            format!("test {:.1} / {:.1} s", run.time(), run.scenario.duration)
                        }
                        Phase::ProofTest => format!("proof test, load x {:.2}", run.load_factor()),
                        Phase::Decided => "decided".to_string(),
                    };
                    s.push_str(&format!("TEST: {phase}\n"));
                    if let Some(r) = &report.0 {
                        s.push_str(&r.summary());
                    }
                }
                None => s.push_str("SIMULATE (gravity + loads)\n"),
            }
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
