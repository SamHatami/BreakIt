//! Scenarios for BreakIt: what each test asks of the player's structure,
//! the test run that applies it in the real-time solver, and the grading.
//!
//! - [`Scenario`]: data loaded from RON. Hard limits (build volume,
//!   foundations, budget, mass, materials), payloads, a load timeline
//!   (convoys, deck loads, ground shaking), a proof test, bonus objectives
//!   and scoring weights.
//! - [`check_limits`]: everything that blocks the test, as plain sentences.
//! - [`TestRun`]: steps `breakit_core::DynamicSim` under the scenario's
//!   loads and records peak utilisation, sag, first damage, breaks and
//!   collapse. [`evaluate`] runs one headless.
//! - [`Report`]: pass or fail, cost / weight / margin / objective scores,
//!   0..1000 score and 0..3 stars, report rows and the timeline.
//!
//! Like `breakit-core`, this crate has no engine dependency.

pub mod cost;
pub mod limits;
pub mod loads;
pub mod report;
pub mod run;
pub mod scenario;

pub use cost::{Bill, CostModel, JointClass, JointPrices};
pub use limits::{check_limits, PlayMode, Violation};
pub use loads::{Binding, DeckNodes, VehiclePose};
pub use report::{Criterion, Report};
pub use run::{evaluate, Happening, Phase, TestRun, TimelineEntry};
pub use scenario::*;

/// The scenarios that ship with the game, as (file name, RON text).
pub const BUILTIN: &[(&str, &str)] = &[
    (
        "traffic_bridge.ron",
        include_str!("../scenarios/traffic_bridge.ron"),
    ),
    (
        "quake_tower.ron",
        include_str!("../scenarios/quake_tower.ron"),
    ),
];

/// Parses every built-in scenario. Panics on a malformed file, which the
/// crate's tests catch.
pub fn builtin() -> Vec<Scenario> {
    BUILTIN
        .iter()
        .map(|(name, text)| {
            Scenario::from_ron(text).unwrap_or_else(|e| panic!("scenarios/{name}: {e}"))
        })
        .collect()
}
