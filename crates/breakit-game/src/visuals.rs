//! Hands the design and each frame's physics to `breakit-render`, which
//! draws the beams: bent along their curves, damaged, and broken.
//!
//! Build mode shows the utilisation heatmap on straight (or exaggerated)
//! members; simulate mode shows the materials with the solver's real
//! rotations, hinges, damage and breaks.

use bevy::prelude::*;
use breakit_render::breakit_mesh::MemberStyle;
use breakit_render::{BreakitRenderPlugin, MemberFrame, RenderSettings, StructureVisual};

use crate::analysis::{BreakitSet, FrameState, Simulation};
use crate::build::BuildTool;
use crate::design::Design;
use crate::testrun::Heatmap;
use crate::Mode;

pub struct VisualsPlugin;

impl Plugin for VisualsPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(BreakitRenderPlugin)
            .add_systems(Update, feed_renderer.after(BreakitSet::Physics));
    }
}

/// Rebuilds the visual structure when the design or the mode changes, and
/// passes on this frame's physics.
#[allow(clippy::too_many_arguments)]
fn feed_renderer(
    mut commands: Commands,
    design: Res<Design>,
    state: Res<FrameState>,
    mode: Res<State<Mode>>,
    sim: Option<Res<Simulation>>,
    tool: Res<BuildTool>,
    heat: Res<Heatmap>,
    visual: Option<ResMut<StructureVisual>>,
    mut frame: ResMut<MemberFrame>,
    mut settings: ResMut<RenderSettings>,
    mut built_for: Local<Option<(u64, bool)>>,
) {
    let simulating = *mode.get() == Mode::Simulate && sim.is_some();
    if design.beams.is_empty() {
        if visual.is_some() {
            commands.remove_resource::<StructureVisual>();
        }
        *built_for = None;
        return;
    }

    // A new simulation starts from an unbroken structure, so entering or
    // leaving simulate mode rebuilds as well.
    let key = (design.revision, simulating);
    let mut fresh = None;
    if *built_for != Some(key) || visual.is_none() {
        let styles: Vec<MemberStyle> = design.beams.iter().map(|b| b.kind.style()).collect();
        fresh = Some(StructureVisual::new(&design.to_structure(), &styles));
        *built_for = Some(key);
    }
    let mut existing = visual;
    let current: &mut StructureVisual = match (&mut fresh, &mut existing) {
        (Some(v), _) => v,
        (None, Some(v)) => &mut *v,
        (None, None) => return,
    };

    frame.0 = match &sim {
        Some(sim) if simulating => current.frame_from_sim(sim.sim()),
        _ => {
            let utilization: Vec<f32> = state.members.iter().map(|m| m.utilization).collect();
            let positions = if state.positions.len() == design.nodes.len() {
                &state.positions
            } else {
                &design.nodes
            };
            current.frame_from_positions(positions, &utilization)
        }
    };

    let stress_view = !simulating || heat.0;
    let highlighted: Vec<usize> = match (tool.hovered, tool.hovered_beam.or(tool.selected_beam)) {
        (Some(h), _) if !simulating => design
            .beams
            .iter()
            .enumerate()
            .filter(|(_, b)| b.a == h || b.b == h)
            .map(|(i, _)| i)
            .collect(),
        (None, Some(b)) if !simulating => vec![b],
        _ => Vec::new(),
    };
    if settings.stress_view != stress_view || settings.highlighted != highlighted {
        settings.stress_view = stress_view;
        settings.highlighted = highlighted;
    }

    if let Some(v) = fresh {
        commands.insert_resource(v);
    }
}
