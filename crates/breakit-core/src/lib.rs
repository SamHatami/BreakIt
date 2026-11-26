//! Structural solver for BreakIt.
//!
//! The crate models a structure as a 3D frame of Euler-Bernoulli beam members
//! joined at nodes, and offers two solvers that share that model:
//!
//! * [`statics::solve_static`]: a linear stiffness-matrix (FEM) solve. Exact for
//!   small deflections; used for the build-mode stress heatmap and as the test
//!   oracle for the dynamic solver.
//! * [`dynamics::DynamicSim`]: a real-time, substepped XPBD solver with
//!   corotational beams, plastic hinges, axial yield and fracture.
//!
//! Members fail by material: steel yields and tears, timber splinters, splits
//! and crushes, concrete cracks and crushes (rebar holds it together after
//! cracking), composites break suddenly or delaminate. [`damage`] has the
//! rules and the per-member [`MemberDamage`] state renderers draw from;
//! [`concrete`] the sectional analysis behind reinforced concrete.
//!
//! Scenario loads (wind, earthquakes, waves and floods, traffic, impacts)
//! live in [`loads`]: models that turn a scenario into nodal forces every
//! frame and report the state of the lab rig that produces them.
//!
//! Units are SI throughout (m, kg, s, N, Pa). The world is Y-up, matching Bevy.
//! The crate has no game-engine dependency on purpose.

pub mod concrete;
pub mod damage;
pub mod dynamics;
pub mod frame;
pub mod linsolve;
pub mod loads;
pub mod material;
pub mod model;
pub mod section;
pub mod statics;

pub use damage::{DamageState, MemberDamage};
pub use dynamics::{BreakCause, BreakEvent, DynamicSim, SimConfig, Solver, SolverStats};
pub use material::{Material, MaterialKind};
pub use loads::{LoadCase, LoadKind, LoadSet, RigState, Window};
pub use model::{Member, NodeLoad, Structure, Support};
pub use section::{Section, Shape};
pub use statics::{solve_static, MemberForces, StaticResult};

/// nalgebra vector type used across the crate.
pub type Vec3 = nalgebra::Vector3<f64>;
