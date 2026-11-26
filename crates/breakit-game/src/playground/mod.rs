//! The playground: build anything on any table, then run the real-time
//! solver while adding, switching and tuning loads. No scoring, no limits.
//!
//! The bench settings and the running simulation live in the engine-free
//! `breakit-playground` crate; this module is the game side: the run's life
//! cycle on [`Screen::Playground`], the keys, picking members and nodes,
//! the panel ([`panel`]), the debug overlays ([`overlay`]) and saved files
//! ([`files`]). Every button and key arrives as a [`PlayAction`] inside
//! [`UiAction::Playground`]. See `docs/playground.md`.

mod capture;
mod files;
mod overlay;
mod panel;

use bevy::prelude::*;
use bevy::window::PrimaryWindow;
use breakit_core::SimConfig;
use breakit_lab::PlayTable;
use breakit_playground::{Env, Live, Param, PointLoad, Probe, Setup, Trace};
use breakit_render::StructureVisual;

use crate::analysis::{
    table_ground, BreakitSet, FrameState, MemberState, SimClock, Simulation, GRAVITY, SIM_DT,
    SPEEDS,
};
use crate::build::segment_distance;
use crate::camera::OrbitCamera;
use crate::design::{to_core, Design};
use crate::flow::{Screen, UiAction};
use crate::testrun::Heatmap;
use crate::ui::pointer::Pointer;

pub struct PlaygroundPlugin;

impl Plugin for PlaygroundPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<Bench>()
            .add_plugins(capture::plugin)
            .add_systems(OnEnter(Screen::Playground), (enter, panel::spawn).chain())
            .add_systems(OnExit(Screen::Playground), leave)
            .add_systems(
                PreUpdate,
                keyboard
                    .after(crate::flow::keyboard_actions)
                    .run_if(in_state(Screen::Playground)),
            )
            .add_systems(
                Update,
                (
                    (pick, apply_actions, push_setup)
                        .chain()
                        .before(BreakitSet::Physics),
                    (sample_trace, overlay::draw).after(BreakitSet::Physics),
                )
                    .run_if(in_state(Screen::Playground)),
            )
            .add_systems(
                PostUpdate,
                (panel::refresh, panel::update_readouts)
                    .chain()
                    .run_if(in_state(Screen::Playground))
                    .before(crate::ui::theme::apply_theme),
            );
    }
}

/// Switchable loads.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Load {
    Wind,
    Gusts,
    Quake,
    Water,
}

/// Debug overlays that switch on and off.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Show {
    Heatmap,
    Forces,
    Reactions,
}

/// Everything the playground's buttons and keys can ask for.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum PlayAction {
    Toggle(Load),
    Step(Param, i8),
    NextQuakeMotion,
    NewRecord,
    NextWaves,
    SendTruck,
    DropWeight,
    /// Clicking a node places a point load (instead of picking it).
    PointTool,
    ClearPointLoads,
    Show(Show),
    /// Next deformation exaggeration.
    NextDeform,
    /// Next quantity on the graph for the picked member or node.
    NextProbe,
    Unpick,
    Pause,
    StepFrame,
    Slower,
    Faster,
    /// Start the run again from the build, keeping the bench.
    Restart,
    /// Back to building.
    Exit,
    NextTable,
    Save(u8),
    Load(u8),
}

/// Deformation exaggeration steps; 1 draws no ghost.
pub const DEFORM: [f32; 4] = [1.0, 10.0, 50.0, 200.0];

/// Seconds of history on the graph.
pub const TRACE_SECONDS: f64 = 10.0;

/// What the player picked.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Pick {
    Node(usize),
    Member(usize),
}

/// The playground's state between runs: the bench and the view settings.
#[derive(Resource, Debug)]
pub struct Bench {
    pub setup: Setup,
    pub forces: bool,
    pub reactions: bool,
    /// Index into [`DEFORM`].
    pub deform: usize,
    pub pick: Option<Pick>,
    pub trace: Option<Trace>,
    pub point_tool: bool,
    /// Built-in table index.
    pub table: usize,
    /// A short message for the status line, and when it was set (real s).
    pub note: Option<(String, f64)>,
}

impl Default for Bench {
    fn default() -> Self {
        Self {
            setup: Setup::default(),
            forces: true,
            reactions: true,
            deform: 0,
            pick: None,
            trace: None,
            point_tool: false,
            table: 1,
            note: None,
        }
    }
}

impl Bench {
    pub fn deform_scale(&self) -> f32 {
        DEFORM[self.deform % DEFORM.len()]
    }

    fn say(&mut self, msg: impl Into<String>, now: f64) {
        self.note = Some((msg.into(), now));
    }
}

/// What the water and paddle are like on a table.
fn env_for(table: &PlayTable) -> Env {
    if table.insert.name() == "wave flume" {
        Env {
            water_bed: -3.2,
            paddle: -13.6,
        }
    } else {
        Env {
            water_bed: -(breakit_lab::table::BASIN_DEPTH as f64),
            paddle: -13.6,
        }
    }
}

/// Index of the built-in table the lab shows.
fn table_index(table: &PlayTable) -> usize {
    (0..4)
        .find(|&i| PlayTable::builtin(i).insert.name() == table.insert.name())
        .unwrap_or(1)
}

/// Builds a fresh run of the design on the table, with the bench as set.
fn start_run(
    commands: &mut Commands,
    design: &Design,
    table: &PlayTable,
    bench: &Bench,
    state: &mut FrameState,
) {
    let structure = design.to_structure();
    let config = SimConfig {
        gravity: to_core(Vec3::new(0.0, -GRAVITY, 0.0)),
        ground: Some(table_ground(table)),
        ..Default::default()
    };
    let live =
        Live::new(structure, config, &bench.setup, env_for(table)).with_loads(design.node_loads());
    *state = FrameState {
        positions: design.nodes.clone(),
        members: vec![MemberState::default(); design.beams.len()],
        revision: None,
        ..Default::default()
    };
    commands.insert_resource(Simulation::playground(live));
    // A new run starts unbroken: have the renderer rebuild the members.
    commands.remove_resource::<StructureVisual>();
}

fn enter(
    mut commands: Commands,
    design: Res<Design>,
    table: Res<PlayTable>,
    mut bench: ResMut<Bench>,
    mut state: ResMut<FrameState>,
    mut clock: ResMut<SimClock>,
    mut heat: ResMut<Heatmap>,
) {
    *clock = SimClock::default();
    heat.0 = true;
    bench.table = table_index(&table);
    bench.pick = None;
    bench.trace = None;
    bench.point_tool = false;
    let n = design.nodes.len();
    bench.setup.point_loads.retain(|p| p.node < n);
    start_run(&mut commands, &design, &table, &bench, &mut state);
}

fn leave(mut heat: ResMut<Heatmap>) {
    heat.0 = false;
}

/// The playground's keys.
fn keyboard(keys: Res<ButtonInput<KeyCode>>, mut out: MessageWriter<UiAction>) {
    use KeyCode as K;
    let ctrl = keys.any_pressed([K::ControlLeft, K::ControlRight, K::SuperLeft, K::SuperRight]);
    let shift = keys.any_pressed([K::ShiftLeft, K::ShiftRight]);
    let mut send = |a: PlayAction| {
        out.write(UiAction::Playground(a));
    };
    if ctrl {
        for (k, slot) in [(K::Digit1, 1), (K::Digit2, 2), (K::Digit3, 3)] {
            if keys.just_pressed(k) {
                send(if shift {
                    PlayAction::Load(slot)
                } else {
                    PlayAction::Save(slot)
                });
            }
        }
        return;
    }
    let map = [
        (K::Space, PlayAction::Pause),
        (K::KeyN, PlayAction::StepFrame),
        (K::Comma, PlayAction::Slower),
        (K::Period, PlayAction::Faster),
        (K::KeyR, PlayAction::Restart),
        (K::Escape, PlayAction::Exit),
        (K::Digit1, PlayAction::Toggle(Load::Wind)),
        (K::Digit2, PlayAction::Toggle(Load::Quake)),
        (K::Digit3, PlayAction::Toggle(Load::Water)),
        (K::KeyV, PlayAction::SendTruck),
        (K::KeyB, PlayAction::DropWeight),
        (K::KeyP, PlayAction::PointTool),
        (K::KeyH, PlayAction::Show(Show::Heatmap)),
        (K::KeyA, PlayAction::Show(Show::Forces)),
        (K::KeyX, PlayAction::Show(Show::Reactions)),
        (K::KeyD, PlayAction::NextDeform),
        (K::KeyG, PlayAction::NextProbe),
        (K::KeyT, PlayAction::NextTable),
        (K::F5, PlayAction::Save(1)),
        (K::F9, PlayAction::Load(1)),
    ];
    for (k, a) in map {
        if keys.just_pressed(k) {
            send(a);
        }
    }
    if keys.just_pressed(K::KeyF) {
        out.write(UiAction::FrameView);
    }
}

/// Screen-space pick radius for nodes and members, px.
const PICK_NODE_PX: f32 = 14.0;
const PICK_MEMBER_PX: f32 = 8.0;

/// Left click in the scene: picks a node or member, or with the point-load
/// tool puts a load on the node under the cursor.
fn pick(
    buttons: Res<ButtonInput<MouseButton>>,
    window: Single<&Window, With<PrimaryWindow>>,
    camera: Single<(&Camera, &GlobalTransform), With<OrbitCamera>>,
    pointer: Res<Pointer>,
    design: Res<Design>,
    state: Res<FrameState>,
    time: Res<Time<Real>>,
    mut bench: ResMut<Bench>,
) {
    if !buttons.just_pressed(MouseButton::Left) || pointer.over_ui {
        return;
    }
    let Some(px) = window.cursor_position() else {
        return;
    };
    let (camera, tf) = *camera;
    let positions = if state.positions.len() == design.nodes.len() {
        &state.positions
    } else {
        &design.nodes
    };
    let screen: Vec<Option<Vec2>> = positions
        .iter()
        .map(|&p| camera.world_to_viewport(tf, p).ok())
        .collect();
    let node = screen
        .iter()
        .enumerate()
        .filter_map(|(i, s)| Some((i, s.as_ref()?.distance(px))))
        .filter(|(_, d)| *d <= PICK_NODE_PX)
        .min_by(|a, b| a.1.total_cmp(&b.1))
        .map(|(i, _)| i);
    let member = || {
        design
            .beams
            .iter()
            .enumerate()
            .filter_map(|(i, b)| {
                let d = segment_distance(px, screen[b.a]?, screen[b.b]?);
                (d <= PICK_MEMBER_PX).then_some((i, d))
            })
            .min_by(|a, b| a.1.total_cmp(&b.1))
            .map(|(i, _)| i)
    };
    let now = time.elapsed_secs_f64();
    if bench.point_tool {
        if let Some(n) = node {
            let f = bench.setup.point_load;
            bench.setup.point_loads.push(PointLoad {
                node: n,
                force: [0.0, -f, 0.0],
            });
            bench.say(format!("{:.0} kN on node {n}", f / 1000.0), now);
        }
        return;
    }
    let picked = node.map(Pick::Node).or_else(|| member().map(Pick::Member));
    if picked != bench.pick {
        bench.pick = picked;
        bench.trace = picked.map(|p| {
            let probe = match p {
                Pick::Node(n) => Probe::node(n)[0],
                Pick::Member(m) => Probe::member(m)[0],
            };
            Trace::new(probe, TRACE_SECONDS)
        });
    }
}

fn apply_actions(
    mut commands: Commands,
    mut actions: MessageReader<UiAction>,
    mut bench: ResMut<Bench>,
    mut clock: ResMut<SimClock>,
    mut heat: ResMut<Heatmap>,
    mut sim: Option<ResMut<Simulation>>,
    mut next: ResMut<NextState<Screen>>,
    mut design: ResMut<Design>,
    mut history: ResMut<crate::history::History>,
    mut table: ResMut<PlayTable>,
    mut state: ResMut<FrameState>,
    time: Res<Time<Real>>,
) {
    let now = time.elapsed_secs_f64();
    let mut restart = false;
    for a in actions.read() {
        let UiAction::Playground(a) = *a else {
            continue;
        };
        let s = &mut bench.setup;
        match a {
            PlayAction::Toggle(Load::Wind) => s.wind.on = !s.wind.on,
            PlayAction::Toggle(Load::Gusts) => s.wind.gusts = !s.wind.gusts,
            PlayAction::Toggle(Load::Quake) => s.quake.on = !s.quake.on,
            PlayAction::Toggle(Load::Water) => s.water.on = !s.water.on,
            PlayAction::Step(p, dir) => {
                p.step(s, dir);
            }
            PlayAction::NextQuakeMotion => s.quake.motion = s.quake.motion.next(),
            PlayAction::NewRecord => s.quake.seed = s.quake.seed.wrapping_add(1),
            PlayAction::NextWaves => s.water.waves = s.water.waves.next(),
            PlayAction::SendTruck => {
                let r = sim
                    .as_mut()
                    .and_then(|s| s.live_mut())
                    .map(|l| l.send_truck());
                match r {
                    Some(Ok(())) => {
                        let t = &bench.setup.traffic;
                        let msg = format!("{:.0} t truck sent at {:.0} m/s", t.mass, t.speed);
                        bench.say(msg, now);
                    }
                    Some(Err(e)) => bench.say(format!("No truck: {e}"), now),
                    None => {}
                }
            }
            PlayAction::DropWeight => {
                let target = match bench.pick {
                    Some(Pick::Node(n)) => Some(n),
                    Some(Pick::Member(m)) => design.beams.get(m).map(|b| b.a),
                    None => None,
                };
                if let Some(l) = sim.as_mut().and_then(|s| s.live_mut()) {
                    l.drop_weight(target);
                    let d = &bench.setup.drop;
                    let msg = format!("{:.1} t dropped from {:.1} m", d.mass / 1000.0, d.height);
                    bench.say(msg, now);
                }
            }
            PlayAction::PointTool => bench.point_tool = !bench.point_tool,
            PlayAction::ClearPointLoads => s.point_loads.clear(),
            PlayAction::Show(Show::Heatmap) => heat.0 = !heat.0,
            PlayAction::Show(Show::Forces) => bench.forces = !bench.forces,
            PlayAction::Show(Show::Reactions) => bench.reactions = !bench.reactions,
            PlayAction::NextDeform => bench.deform = (bench.deform + 1) % DEFORM.len(),
            PlayAction::NextProbe => {
                if let Some(t) = &mut bench.trace {
                    *t = Trace::new(t.probe.next(), TRACE_SECONDS);
                }
            }
            PlayAction::Unpick => {
                bench.pick = None;
                bench.trace = None;
            }
            PlayAction::Pause => clock.paused = !clock.paused,
            PlayAction::StepFrame => {
                clock.paused = true;
                if let Some(l) = sim.as_mut().and_then(|s| s.live_mut()) {
                    l.set_setup(&bench.setup);
                    l.step(SIM_DT);
                }
            }
            PlayAction::Slower | PlayAction::Faster => {
                let i = SPEEDS
                    .iter()
                    .position(|v| (*v - clock.speed).abs() < 1e-3)
                    .unwrap_or(3);
                let j = if a == PlayAction::Slower {
                    i.saturating_sub(1)
                } else {
                    (i + 1).min(SPEEDS.len() - 1)
                };
                clock.speed = SPEEDS[j];
                clock.paused = false;
            }
            PlayAction::Restart => restart = true,
            PlayAction::Exit => next.set(Screen::Build),
            PlayAction::NextTable => {
                bench.table = (bench.table + 1) % 4;
                *table = PlayTable::builtin(bench.table);
                restart = true;
            }
            PlayAction::Save(slot) => {
                let msg = match files::save(slot, &design, bench.table, &bench.setup) {
                    Ok(p) => format!("Saved slot {slot}: {}", p.display()),
                    Err(e) => format!("Save failed: {e}"),
                };
                bench.say(msg, now);
            }
            PlayAction::Load(slot) => match files::load(slot) {
                Ok(f) => {
                    let mut d = files::design_from(&f.design);
                    d.revision = design.revision + 1;
                    *design = d;
                    history.reset(&design);
                    bench.setup = f.setup;
                    bench.table = f.table % 4;
                    *table = PlayTable::builtin(bench.table);
                    bench.pick = None;
                    bench.trace = None;
                    bench.say(format!("Loaded slot {slot}"), now);
                    restart = true;
                }
                Err(e) => bench.say(format!("Slot {slot}: {e}"), now),
            },
        }
    }
    if restart {
        *clock = SimClock {
            paused: clock.paused,
            ..SimClock::default()
        };
        if let Some(t) = &mut bench.trace {
            *t = Trace::new(t.probe, TRACE_SECONDS);
        }
        start_run(&mut commands, &design, &table, &bench, &mut state);
    }
}

/// Hands changed bench settings to the running simulation.
fn push_setup(bench: Res<Bench>, sim: Option<ResMut<Simulation>>) {
    let Some(mut sim) = sim else { return };
    let Some(live) = sim.live_mut() else { return };
    if live.setup() != &bench.setup {
        live.set_setup(&bench.setup);
    }
}

fn sample_trace(mut bench: ResMut<Bench>, sim: Option<Res<Simulation>>) {
    let Some(live) = sim.as_ref().and_then(|s| s.live()) else {
        return;
    };
    if let Some(t) = &mut bench.trace {
        t.sample(live);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tables_are_found_again_and_the_flume_has_its_bed() {
        for i in 0..4 {
            assert_eq!(table_index(&PlayTable::builtin(i)), i);
        }
        assert_eq!(env_for(&PlayTable::builtin(2)).water_bed, -3.2);
        assert!(env_for(&PlayTable::builtin(0)).water_bed < -3.2);
    }

    #[test]
    fn the_starter_bridge_runs_on_the_bench() {
        let d = Design::starter_bridge();
        let table = PlayTable::builtin(0);
        let mut setup = Setup::default();
        setup.wind.on = true;
        let config = SimConfig {
            ground: Some(table_ground(&table)),
            ..Default::default()
        };
        let mut live =
            Live::new(d.to_structure(), config, &setup, env_for(&table)).with_loads(d.node_loads());
        live.send_truck().unwrap();
        for _ in 0..60 {
            live.step(SIM_DT);
        }
        assert!(live.sim().position(3).iter().all(|v| v.is_finite()));
        assert_eq!(live.reactions().len(), 2);
    }
}
