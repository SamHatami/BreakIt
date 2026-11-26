//! Procedural, deformable geometry for BreakIt. No engine dependency.
//!
//! The visual model follows the structural model one to one:
//!
//! * [`profile::Profile`]: the parametric cross-section of a member. Both the
//!   visual mesh and the solver's [`breakit_core::Section`] come from it.
//! * [`mesh::member_template`]: one shared mesh per profile, in *member space*
//!   (`s` along the member, `(y, z)` across). It is uploaded once.
//! * [`pose::MemberPose`]: where a member is this frame (end positions, end
//!   frames, elastic end tangents, plastic hinge stubs). The renderer packs it
//!   into a [`pose::GpuMember`] and the vertex shader bends the template
//!   along the member's cubic Hermite curve, so deformation costs ~144 bytes
//!   per member per frame on the CPU, not a re-mesh.
//! * [`state`]: material family, per-member damage, and the adapter from
//!   [`breakit_core::DynamicSim`].
//! * [`fracture`], [`debris`], [`particles`]: what happens when a member
//!   breaks: procedurally split, material-specific pieces that become cheap
//!   visual-only rigid bodies, plus sparks, dust, splinters and fibres.
//!
//! Everything here is plain Rust and testable headless; `breakit-render`
//! is the Bevy side.

pub mod debris;
pub mod fracture;
pub mod mesh;
pub mod particles;
pub mod pose;
pub mod profile;
pub mod state;

pub use debris::{DebrisConfig, DebrisWorld};
pub use fracture::{fracture, BreakInput, DebrisPiece, Fracture, ParticleBurst};
pub use mesh::{member_template, MeshData, TemplateOptions};
pub use particles::ParticlePool;
pub use pose::{GpuMember, MemberPose};
pub use profile::Profile;
pub use state::{
    MaterialKind, MemberStyle, MemberVisualState, VisualBreak, VisualFrame, VisualStructure,
};
