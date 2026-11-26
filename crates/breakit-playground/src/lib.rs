//! The playground: a free load bench for trying structures out.
//!
//! No scoring and no limits. The player builds on any table, then runs the
//! real-time solver while adding, switching and tuning loads: wind, a shake
//! table, water and waves, trucks, dropped weights, gravity and point loads.
//! This crate is the engine-free half: the bench settings ([`Setup`]), the
//! running simulation that follows them ([`Live`]), member end forces and
//! support reactions ([`forces`]), the graph's probes ([`probe`]), and saved
//! files ([`saved`]). The game draws the panel, overlays and graph.
//!
//! See `docs/playground.md`.

pub mod forces;
pub mod live;
pub mod probe;
pub mod saved;
pub mod setup;

pub use live::{deck_lane, Env, Live, SendError};
pub use probe::{Probe, Trace};
pub use saved::{FileError, SavedBeam, SavedDesign, SavedPlayground, SavedSupport};
pub use setup::{
    heading_vector, DropSetup, Param, PointLoad, QuakeMotion, QuakeSetup, Setup, TrafficSetup,
    WaterSetup, WaveKind, WindSetup,
};
