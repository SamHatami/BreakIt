# Notes for agents

- Physics lives in `crates/breakit-core` and must not depend on Bevy or any engine.
- Every solver change must keep `cargo test --release` green; the static FEM is the oracle for the dynamic solver.
- Units are SI, world is Y-up.
- Pin the Bevy version when the game crate is added; check that release's migration guide and examples before writing Bevy code.
- Current status and open problems: `docs/solver-spike.md`.
- Material and damage models: `docs/materials.md`.
