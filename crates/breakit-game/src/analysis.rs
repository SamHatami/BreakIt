//! The bridge between the game and `breakit-core`.
//!
//! Everything the renderer and HUD need from physics goes through
//! [`FrameState`]: where each node is drawn and how hard each member is
//! working. Build mode fills it from the static solve, simulate mode from
//! the real-time `DynamicSim`. Add new physics read-outs here, not in the
//! render code.

use bevy::prelude::*;
use breakit_core::{solve_static, Block, BreakCause, DynamicSim, Ground, NodeLoad, SimConfig};
use breakit_lab::PlayTable;
use breakit_scenario::TestRun;

use crate::design::{from_core, to_core, Design};
use crate::flow::Screen;
use crate::scenario::Sheets;
use crate::Mode;

pub struct AnalysisPlugin;

/// Systems that update [`FrameState`] (and step [`Simulation`]) run in this
/// set in `Update`. Order visual or gameplay systems `.after(BreakitSet::Physics)`.
#[derive(SystemSet, Debug, Clone, PartialEq, Eq, Hash)]
pub enum BreakitSet {
    Physics,
}

impl Plugin for AnalysisPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<FrameState>()
            .init_resource::<DeflectionScale>()
            .init_resource::<SimClock>()
            .add_systems(
                Update,
                (
                    static_analysis.run_if(in_state(Mode::Build)),
                    step_simulation.run_if(in_state(Mode::Simulate)),
                )
                    .in_set(BreakitSet::Physics),
            )
            .add_systems(OnEnter(Screen::Test), start_simulation)
            .add_systems(OnExit(Mode::Simulate), stop_simulation);
    }
}

/// Per-member read-out for display.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct MemberState {
    /// Fraction of capacity in use: 0 is unloaded, 1 is at yield / plastic
    /// moment.
    pub utilization: f32,
    pub broken: bool,
    /// Axial force in newtons, tension positive. Only known in build mode.
    pub axial: Option<f32>,
    /// Fraction of each end joint's capacity in use (end a, end b); 0 for
    /// rigid joints.
    pub joints: [f32; 2],
}

/// What the renderer draws this frame.
#[derive(Resource, Debug, Default)]
pub struct FrameState {
    /// Display position per design node.
    pub positions: Vec<Vec3>,
    /// One entry per design beam.
    pub members: Vec<MemberState>,
    /// Static solve failure, e.g. an unsupported or wobbly structure.
    pub error: Option<String>,
    /// Largest real (unscaled) displacement, metres.
    pub max_displacement: f32,
    /// Real (unscaled) static displacement per node; empty when unsolved or
    /// simulating.
    pub displacements: Vec<Vec3>,
    /// Break messages in the order they happened.
    pub log: Vec<String>,
    /// Design revision this state was computed for.
    pub revision: Option<u64>,
}

impl FrameState {
    pub fn max_utilization(&self) -> f32 {
        self.members
            .iter()
            .filter(|m| !m.broken)
            .map(|m| m.utilization)
            .fold(0.0, f32::max)
    }
}

/// Visual exaggeration of static deflections in build mode.
#[derive(Resource, Debug, Clone, Copy)]
pub struct DeflectionScale(pub f32);

impl Default for DeflectionScale {
    fn default() -> Self {
        Self(0.0)
    }
}

/// How fast the test runs: a multiple of real time, or paused. Set by the
/// test-run controls and by the automatic slow motion on a first break.
#[derive(Resource, Debug, Clone, Copy, PartialEq)]
pub struct SimClock {
    pub speed: f32,
    pub paused: bool,
}

impl Default for SimClock {
    fn default() -> Self {
        Self {
            speed: 1.0,
            paused: false,
        }
    }
}

/// Speeds the slower / faster keys step through.
pub const SPEEDS: [f32; 5] = [0.125, 0.25, 0.5, 1.0, 2.0];

/// Gravity used everywhere, m/s^2.
pub const GRAVITY: f32 = 9.81;

/// Live real-time simulation, present only in a test and its report: the
/// sheet's graded [`TestRun`], which applies the scenario's loads (and in
/// the sandbox the player's own).
#[derive(Resource)]
pub struct Simulation {
    run: Box<TestRun>,
    seen_breaks: usize,
}

impl Simulation {
    pub fn sim(&self) -> &DynamicSim {
        self.run.sim()
    }

    pub fn run(&self) -> &TestRun {
        &self.run
    }
}

/// Computes the static response of the design when it, the sheet, or the
/// deflection scale changes. The check previews the sheet's loads (see
/// [`crate::scenario::Scenario::static_loads`]).
pub fn static_analysis(
    design: Res<Design>,
    scale: Res<DeflectionScale>,
    sheets: Res<Sheets>,
    mut state: ResMut<FrameState>,
) {
    if state.revision == Some(design.revision) && !scale.is_changed() && !sheets.is_changed() {
        return;
    }
    let extra: Vec<NodeLoad> = sheets.current().static_loads(&design);
    *state = compute_static(&design, scale.0, &extra);
}

/// Pure static analysis: self weight, the player's loads and `extra`.
pub fn compute_static(design: &Design, scale: f32, extra: &[NodeLoad]) -> FrameState {
    let mut state = FrameState {
        positions: design.nodes.clone(),
        members: vec![MemberState::default(); design.beams.len()],
        revision: Some(design.revision),
        ..Default::default()
    };
    if design.beams.is_empty() {
        return state;
    }
    let structure = design.to_structure();
    let mut loads = structure.self_weight_loads(GRAVITY as f64);
    loads.extend(design.node_loads());
    loads.extend_from_slice(extra);
    match solve_static(&structure, &loads) {
        Ok(r) => {
            state.displacements = r.displacements.iter().map(|&d| from_core(d)).collect();
            for (i, p) in state.positions.iter_mut().enumerate() {
                let u = state.displacements[i];
                state.max_displacement = state.max_displacement.max(u.length());
                *p += u * scale;
            }
            for (m, f) in state.members.iter_mut().zip(&r.member_forces) {
                m.utilization = f.utilization as f32;
                m.axial = Some(f.axial as f32);
                m.joints = f.joint_utilization.map(|u| u as f32);
            }
        }
        Err(e) => state.error = Some(e.to_string()),
    }
    state
}

/// What collapsing parts land on: the play table's contact boxes (banks,
/// slabs, channel beds), over a floor at the basin bottom so nothing falls
/// forever if an insert leaves a hole.
pub fn table_ground(table: &PlayTable) -> Ground {
    Ground {
        height: -breakit_lab::table::BASIN_DEPTH as f64,
        blocks: table
            .contact_boxes()
            .into_iter()
            .map(|(lo, hi)| Block::new(to_core(lo), to_core(hi)))
            .collect(),
        ..Default::default()
    }
}

fn start_simulation(
    mut commands: Commands,
    design: Res<Design>,
    sheets: Res<Sheets>,
    table: Res<PlayTable>,
    mut state: ResMut<FrameState>,
    mut clock: ResMut<SimClock>,
    mut next: ResMut<NextState<Screen>>,
) {
    *clock = SimClock::default();
    let structure = design.to_structure();
    let config = SimConfig {
        gravity: to_core(Vec3::new(0.0, -GRAVITY, 0.0)),
        ground: Some(table_ground(&table)),
        ..Default::default()
    };
    let s = sheets.current();
    let bill = design.bill(&structure, &sheets.costs);
    let run = match TestRun::new(
        &s.def,
        &structure,
        &design.node_loads(),
        bill,
        s.mode,
        config,
    ) {
        Ok(run) => run,
        Err(v) => {
            // Build mode doesn't offer the test while limits fail.
            warn!("test blocked: {v:?}");
            next.set(Screen::Build);
            return;
        }
    };
    *state = FrameState {
        positions: design.nodes.clone(),
        members: vec![MemberState::default(); design.beams.len()],
        revision: None,
        ..Default::default()
    };
    commands.insert_resource(Simulation {
        run: Box::new(run),
        seen_breaks: 0,
    });
}

fn stop_simulation(mut commands: Commands, mut state: ResMut<FrameState>) {
    commands.remove_resource::<Simulation>();
    // Forces the static view to recompute.
    state.revision = None;
}

/// Fixed physics step. The sim advances in 1/60 s steps however fast the
/// game renders, catching up at most a few steps per frame.
const SIM_DT: f64 = 1.0 / 60.0;

fn step_simulation(
    time: Res<Time>,
    clock: Res<SimClock>,
    mut acc: Local<f64>,
    sim: Option<ResMut<Simulation>>,
    design: Res<Design>,
    mut state: ResMut<FrameState>,
) {
    let Some(mut sim) = sim else { return };
    let dt = if clock.paused {
        0.0
    } else {
        time.delta_secs_f64() * clock.speed as f64
    };
    *acc = (*acc + dt).min(8.0 * SIM_DT);
    while *acc >= SIM_DT {
        if sim.run.is_done() {
            *acc = 0.0;
            break;
        }
        sim.run.step(SIM_DT);
        *acc -= SIM_DT;
    }

    let s = sim.sim();
    for (i, p) in state.positions.iter_mut().enumerate() {
        *p = from_core(s.position(i));
    }
    for (i, m) in state.members.iter_mut().enumerate() {
        m.utilization = s.utilization(i) as f32;
        m.broken = s.is_broken(i);
        m.joints = s.damage(i).joint.map(|u| u as f32);
    }

    let events = s.break_events();
    let new: Vec<String> = events[sim.seen_breaks..]
        .iter()
        .map(|e| {
            let beam = &design.beams[e.member];
            format!(
                "t={:.2}s: {} beam {} broke ({})",
                e.time,
                beam.kind.label(),
                e.member,
                cause_label(&e.cause)
            )
        })
        .collect();
    sim.seen_breaks = events.len();
    state.log.extend(new);
}

pub fn cause_label(c: &BreakCause) -> String {
    format!("{c:?}").to_lowercase()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::design::{BeamKind, SupportKind};

    #[test]
    fn starter_bridge_spans_the_default_table_gap() {
        let d = Design::starter_bridge();
        let mut sim = DynamicSim::new(
            &d.to_structure(),
            SimConfig {
                ground: Some(table_ground(&PlayTable::default())),
                ..Default::default()
            },
        );
        sim.step(1.0 / 60.0);
        // Only the supported ends sit on the banks; the span is over the gap.
        for n in 0..d.nodes.len() {
            if d.supports[n].is_none() {
                assert!(
                    sim.clearance(n).unwrap() > 0.1,
                    "node {n} touches the table"
                );
            }
        }
    }

    #[test]
    fn static_state_reports_utilization_and_sag() {
        let d = Design::starter_bridge();
        let s = compute_static(&d, 1.0, &[]);
        assert!(s.error.is_none(), "{:?}", s.error);
        assert_eq!(s.members.len(), d.beams.len());
        assert!(s.max_utilization() > 0.0);
        assert!(s.max_displacement > 0.0);
        assert!(s.positions[3].y < d.nodes[3].y);
        println!(
            "starter bridge: max utilization {:.2}, max deflection {:.4} m",
            s.max_utilization(),
            s.max_displacement
        );
    }

    #[test]
    fn unsupported_design_reports_an_error() {
        let mut d = Design::default();
        let a = d.add_node(Vec3::ZERO);
        let b = d.add_node(Vec3::X);
        d.add_beam(a, b, BeamKind::part("steel-tube-m"));
        let s = compute_static(&d, 1.0, &[]);
        assert!(s.error.is_some());
    }

    #[test]
    fn cantilever_runs_in_real_time_sim_without_nans() {
        let mut d = Design::default();
        let a = d.add_node(Vec3::ZERO);
        let b = d.add_node(Vec3::X * 2.0);
        d.add_beam(a, b, BeamKind::part("steel-tube-m"));
        d.set_support(a, Some(SupportKind::Fixed));
        let mut sim = DynamicSim::new(&d.to_structure(), SimConfig::default());
        for _ in 0..60 {
            sim.step(SIM_DT);
        }
        let tip = from_core(sim.position(b));
        assert!(tip.is_finite());
        assert!(tip.y < 0.0, "tip should sag under self weight, got {tip}");
    }
}
