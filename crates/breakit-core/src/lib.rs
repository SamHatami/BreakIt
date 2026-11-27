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
//! Member ends connect through a [`Joint`]: rigid, pinned or semi-rigid,
//! with its own strength, so a connection can fail before its member.
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

mod blockchol;
pub mod concrete;
pub mod contact;
pub mod damage;
pub mod dynamics;
pub mod frame;
mod jacobi;
pub mod joint;
pub mod linsolve;
pub mod loads;
pub mod material;
pub mod model;
pub mod section;
pub mod simd;
pub mod statics;

pub use contact::{Block, ContactEvent, Ground};
pub use damage::{DamageState, MemberDamage};
pub use dynamics::{BreakCause, BreakEvent, DynamicSim, SimConfig, Solver, SolverStats};
pub use joint::{Joint, JointKind};
pub use loads::{LoadCase, LoadKind, LoadSet, RigState, Window};
pub use material::{ConcreteClass, Material, MaterialKind, SteelGrade, TimberClass};
pub use model::{Member, NodeLoad, NodeSpring, Structure, Support};
pub use section::{Section, Shape};
pub use simd::SimdLevel;
pub use statics::{solve_static, solve_static_with_springs, MemberForces, StaticResult};

/// nalgebra vector type used across the crate.
pub type Vec3 = nalgebra::Vector3<f64>;
