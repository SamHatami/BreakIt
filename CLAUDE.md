# Notes for agents

- Read `CONTRIBUTING.md` first: checks to run before pushing, crate dependency direction, who owns which files, and the shared files to touch as little as possible.
- CI denies all warnings: run `cargo fmt --all` and `cargo clippy --workspace --all-targets -- -D warnings` before every push. The toolchain is pinned in `rust-toolchain.toml`.
- Physics lives in `crates/breakit-core` and must not depend on Bevy or any engine.
- Every solver change must keep `cargo test --release` green; the static FEM is the oracle for the dynamic solver.
- Units are SI, world is Y-up.
- Pin the Bevy version when the game crate is added; check that release's migration guide and examples before writing Bevy code.
- Current status and open problems: `docs/solver-spike.md`.
- Material and damage models: `docs/materials.md`.
- Ground and table contact: `docs/contact.md`.
- Scenarios, limits and scoring: `docs/scenarios.md`. Scenario data and grading live in the engine-free `breakit-scenario` crate.
- Member end joints (pinned, bolted, welded...) and connection failure: `docs/joints.md`.
- Building parts (catalog, standard sections, grades, costs, sandbox limits): `docs/parts.md`, data in `crates/breakit-parts/data/parts.ron`.
- Member visuals (GPU deformation, damage looks, breaking, joints and node hubs): `docs/visuals.md`. `GpuMember` in `breakit-mesh` and `Member` in `breakit-render/src/member.wesl` must stay field-for-field identical.
- Scenario loads (wind, quake, water, vehicles, impacts) and their rig hooks: `docs/loads.md`.
- The hall and play table: `docs/lab.md`. The structure is full size; the hall is drawn `model_scale` (20) times larger so the table reads as a 1:20 model.
- Sound: `docs/audio.md`; the model behind structure sounds: `docs/sound-physics.md`. The simulation decides every structure sound (which event, when, how loud, what pitch, how long) and the member's own modes colour it; the timbre of a crack or clang comes from recordings in `crates/breakit-audio/samples/` (one folder per kind of event, see its README). Do not go back to computed clicks ringing bare modes: Sam heard that as glass. Wind tones are fully computed. `breakit_audio::StructureListener` turns a `DynamicSim` into commands; the game only adds debris, rigs, UI and the camera in `breakit-game/src/sound.rs`.
