//! Real-time water for the wave flume and the flood scenarios.
//!
//! [`Solver`] is a depth-averaged, non-hydrostatic shallow-water solver: a
//! paddle makes waves that travel at the speed linear wave theory gives
//! them, shoal, break into bores, run up a beach, and flood dry ground.
//! See `docs/water.md` for the model, its validation and its limits.

pub mod field;
pub mod grid;
pub mod paddle;
pub mod sim;
pub mod solver;

pub use field::Field;
pub use grid::Grid;
pub use paddle::Program;
pub use sim::{Drive, Droplet, FlumeSetup, WaterSim, Whitewater};
pub use solver::{Boundary, Config, Side, Solver, StepStats};
