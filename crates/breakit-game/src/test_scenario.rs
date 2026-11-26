//! The active test scenario: its limits checked live in build mode, its
//! loads driving simulate mode, and the graded report afterwards.
//!
//! The scenario data, the test run and the grading live in the engine-free
//! `breakit-scenario` crate; this module only holds the choice, keeps the
//! pre-test check current and draws the scenario's guides (build area,
//! foundations, lane, payloads, vehicles) as gizmos until the play table
//! and report screens replace them.
//!
//! Keys: N cycles scenarios (and free play), B toggles challenge / sandbox.

use bevy::prelude::*;
use breakit_scenario::{
    builtin, check_limits, Bill, CostModel, PlayMode, Report, Scenario, Violation,
};

use crate::analysis::Simulation;
use crate::design::{from_core, Design};
use crate::Mode;

pub struct TestScenarioPlugin;

impl Plugin for TestScenarioPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(ActiveScenario::default())
            .init_resource::<ScenarioCheck>()
            .init_resource::<LastReport>()
            .add_systems(
                Update,
                (
                    (choose_scenario, update_check)
                        .chain()
                        .run_if(in_state(Mode::Build)),
                    draw_scenario_guides,
                ),
            );
    }
}

/// Which scenario the player is building for. `index: None` is free play:
/// no limits, player loads only, as before scenarios existed.
#[derive(Resource, Debug, Clone)]
pub struct ActiveScenario {
    pub all: Vec<Scenario>,
    pub index: Option<usize>,
    pub mode: PlayMode,
    pub costs: CostModel,
}

impl Default for ActiveScenario {
    fn default() -> Self {
        Self {
            all: builtin(),
            index: Some(0),
            mode: PlayMode::Challenge,
            costs: CostModel::default(),
        }
    }
}

impl ActiveScenario {
    pub fn get(&self) -> Option<&Scenario> {
        self.index.map(|i| &self.all[i])
    }
}

/// The pre-test check for the current design.
#[derive(Resource, Debug, Default, Clone)]
pub struct ScenarioCheck {
    pub bill: Bill,
    pub violations: Vec<Violation>,
    revision: Option<(u64, Option<usize>, PlayMode)>,
}

impl ScenarioCheck {
    pub fn blocks_test(&self) -> bool {
        !self.violations.is_empty()
    }
}

/// The report of the last decided test run.
#[derive(Resource, Debug, Default, Clone)]
pub struct LastReport(pub Option<Report>);

fn choose_scenario(keys: Res<ButtonInput<KeyCode>>, mut active: ResMut<ActiveScenario>) {
    if keys.just_pressed(KeyCode::KeyN) {
        let n = active.all.len();
        active.index = match active.index {
            None if n > 0 => Some(0),
            Some(i) if i + 1 < n => Some(i + 1),
            _ => None,
        };
    }
    if keys.just_pressed(KeyCode::KeyB) {
        active.mode = match active.mode {
            PlayMode::Challenge => PlayMode::Sandbox,
            PlayMode::Sandbox => PlayMode::Challenge,
        };
    }
}

fn update_check(
    design: Res<Design>,
    active: Res<ActiveScenario>,
    mut check: ResMut<ScenarioCheck>,
) {
    let key = (design.revision, active.index, active.mode);
    if check.revision == Some(key) {
        return;
    }
    let structure = design.to_structure();
    let bill = active.costs.estimate(&structure);
    let violations = match active.get() {
        Some(s) if !design.beams.is_empty() => check_limits(s, &structure, &bill, active.mode),
        Some(_) => vec![Violation::NoSupports],
        None => Vec::new(),
    };
    *check = ScenarioCheck {
        bill,
        violations,
        revision: Some(key),
    };
}

fn v3(p: [f64; 3]) -> Vec3 {
    Vec3::new(p[0] as f32, p[1] as f32, p[2] as f32)
}

const GUIDE: Color = Color::srgba(0.88, 0.65, 0.15, 0.6);
const ANCHOR: Color = Color::srgb(0.88, 0.65, 0.15);
const LANE: Color = Color::srgba(0.9, 0.9, 0.9, 0.5);
const VEHICLE: Color = Color::srgb(0.85, 0.3, 0.2);

fn draw_scenario_guides(
    mut gizmos: Gizmos,
    active: Res<ActiveScenario>,
    mode: Res<State<Mode>>,
    sim: Option<Res<Simulation>>,
) {
    let Some(s) = active.get() else { return };
    let building = *mode.get() == Mode::Build;
    if building && active.mode == PlayMode::Challenge {
        let (lo, hi) = (v3(s.limits.volume.min), v3(s.limits.volume.max));
        gizmos.cube(
            Transform::from_translation((lo + hi) * 0.5).with_scale(hi - lo),
            GUIDE,
        );
    }
    for a in &s.anchors {
        let p = v3(a.position);
        gizmos.sphere(Isometry3d::from_translation(p), 0.12, ANCHOR);
        gizmos.line(p, p - Vec3::Y * 0.3, ANCHOR);
    }
    if let Some(d) = &s.deck {
        gizmos.line(v3(d.from), v3(d.to), LANE);
    }
    for p in &s.payloads {
        let at = v3(p.at);
        gizmos.cube(
            Transform::from_translation(at + Vec3::Y * 0.4).with_scale(Vec3::splat(0.8)),
            if building { GUIDE } else { VEHICLE },
        );
    }
    // Vehicles ride on the deflected deck: lift them onto the nearest
    // deck node's current height.
    if let Some(run) = sim.as_ref().and_then(|s| s.run()) {
        let deck = run.binding().deck.as_ref();
        for v in run.vehicles() {
            let mid = v.front - v.length * 0.5;
            let base = deck
                .and_then(|d| {
                    d.stations
                        .iter()
                        .min_by(|a, b| (a.s - mid).abs().total_cmp(&(b.s - mid).abs()))
                        .map(|st| from_core(run.sim().displacement(st.nodes[0])).y)
                })
                .unwrap_or(0.0)
                .max(-1.0);
            let dir = from_core(v.dir);
            let centre =
                from_core(v.position) - dir * (v.length as f32 * 0.5) + Vec3::Y * (0.6 + base);
            gizmos.cube(
                Transform::from_translation(centre)
                    .looking_to(dir, Vec3::Y)
                    .with_scale(Vec3::new(1.6, 1.2, v.length as f32 + 1.0)),
                VEHICLE,
            );
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use breakit_core::SimConfig;
    use breakit_scenario::evaluate;

    #[test]
    fn starter_bridge_is_a_legal_first_attempt_at_the_default_scenario() {
        let active = ActiveScenario::default();
        let scenario = active.get().unwrap();
        let design = Design::starter_bridge();
        let s = design.to_structure();
        let bill = active.costs.estimate(&s);
        assert_eq!(
            check_limits(scenario, &s, &bill, PlayMode::Challenge),
            vec![]
        );
        let r = evaluate(
            scenario,
            &s,
            &[],
            &active.costs,
            PlayMode::Challenge,
            SimConfig::default(),
        )
        .unwrap();
        // Heavy for the job: it passes, but the margin is far above the band.
        assert!(r.passed && r.stars >= 1);
        assert!(r.safety_margin > scenario.scoring.margin_band[1]);
    }
}
