# Contributing

BreakIt is written mostly by coding agents working in parallel, one task per
branch. These rules exist so those branches merge without fights. They apply
to people too.

## Before you push

```
cargo fmt --all
cargo clippy --workspace --all-targets -- -D warnings
cargo test --release --no-default-features -p breakit-core -p breakit-mesh -p breakit-parts -p breakit-scenario -p breakit-audio
cargo test -p breakit-game -p breakit-render -p breakit-lab
```

CI runs exactly these (plus `cargo doc` with warnings denied). The compiler
is pinned in `rust-toolchain.toml`, so your local clippy matches CI's.
Building the Bevy crates on Linux needs `libudev-dev libwayland-dev
libxkbcommon-dev libasound2-dev pkg-config`.

## Branches and PRs

- Branch from the latest `main`, not from another open branch. Stacked
  branches turn one conflict into several.
- One topic per PR. Keep refactors of shared files (see below) in their own
  small PR, so they land fast and everyone rebases once.
- Before opening a PR and again before it merges: `git fetch origin main`,
  merge or rebase onto it, and re-run the checks above.
- `Cargo.lock` conflicts: take `main`'s version, then run `cargo check
  --workspace` to re-add your dependencies. Never hand-merge it.

## Crates and dependency direction

```
breakit-core       solver, materials, damage, joints, loads    no engine
  ^
  +-- breakit-scenario   briefs, test runs, scoring            no engine
  +-- breakit-audio      synthesised sound                     no engine (cpal behind `device`)
  +-- breakit-mesh       member geometry, fracture, debris     no engine (glam)
        ^
        +-- breakit-parts    part catalog, sections, costs     no engine
        +-- breakit-render   Bevy renderer for members         Bevy
breakit-lab        hall and play tables                         Bevy, no breakit deps
  ^
breakit-game       the app: modes, UI, input, wiring            Bevy, everything
```

- Arrows only point up. The engine-free crates (core, mesh, parts, scenario,
  audio) never depend on Bevy or on a crate below them; `breakit-lab` stays independent of the solver.
- Logic that can be tested without a window goes in an engine-free crate.
  Game-side code that holds pure data and maths still belongs there if it
  imports `bevy` only for `Vec3` or `Resource`.
- A new crate goes under `crates/`, joins `members` in the root
  `Cargo.toml`, uses the `workspace` fields for edition, license, repository
  and `rust-version`, and ends with `[lints] workspace = true`. CI's clippy, fmt and docs cover it
  automatically; add it to the `cargo test` list of the matching job in
  `.github/workflows/ci.yml` (engine-free: `core` job, Bevy: `game` job).
- Shared dependency versions go in `[workspace.dependencies]`. Bevy is pinned
  to `=0.20.1` in every crate that uses it; upgrade all of them in one PR.

## Who owns what

Each area has one owner at a time: the thread or person whose task it is.
Work inside your area freely. To change another area, keep the change
minimal and say so in the PR description, or ask that area's owner to do it.

| Area | Paths | Doc |
| --- | --- | --- |
| Solver | `breakit-core/src/{statics,dynamics,linsolve,frame,model}.rs` | `docs/solver-spike.md` |
| Materials and damage | `breakit-core/src/{material,section,concrete,damage}.rs` | `docs/materials.md` |
| Joints | `breakit-core/src/joint.rs` | `docs/joints.md` |
| Loads | `breakit-core/src/loads/` | `docs/loads.md` |
| Scenarios and scoring | `crates/breakit-scenario/` | `docs/scenarios.md` |
| Member visuals | `crates/breakit-mesh/`, `crates/breakit-render/` | `docs/visuals.md` |
| Building parts | `crates/breakit-parts/` | `docs/parts.md` |
| Ground contact | `breakit-core/src/contact.rs` | `docs/contact.md` |
| Sound | `crates/breakit-audio/`, `breakit-game/src/sound.rs` | `docs/audio.md` |
| Lab and play tables | `crates/breakit-lab/` | `docs/lab.md` |
| Game UI and flow | `breakit-game/src/ui/`, `flow.rs`, `history.rs` | `docs/game-ui.md` |
| Game shell | the rest of `breakit-game/src/` | `README.md` |

## Shared files: the usual sources of conflicts

Several areas touch these. Change them as little as possible, and prefer
adding a new file to growing one of these.

- `breakit-core/src/dynamics.rs`: add a feature in its own module and call it
  from `DynamicSim` with a few lines; do not paste a subsystem into this file.
  Accessors other crates need (`node_mass`, forces, ...) are added once, by the
  solver owner, and reused.
- `lib.rs` module lists and `pub use` lists, `breakit-game/src/main.rs`
  plugin list: add one line in alphabetical order, never reorder or reformat
  neighbouring lines.
- In `breakit-game`, each feature is a Bevy `Plugin` in its own module and
  `main.rs` only adds it. Systems go in the feature's plugin, not in `main.rs`.
- `CLAUDE.md`, `README.md`, `.github/workflows/ci.yml`, root `Cargo.toml`:
  only edit them when your change needs it. A new doc goes in `docs/` and in
  the table above.

## Code conventions

- Units are SI, the world is Y-up. Name units in docs (`/// Span, m.`).
- `breakit-core` uses `f64` and nalgebra (`Vec3`); `breakit-mesh` and the Bevy
  crates use `f32` and glam. Convert at the boundary, in one place.
- Errors: fallible public functions in the engine-free crates return
  `Result<_, SomeError>`, where the error type implements `Display` and
  `std::error::Error`. `panic!`, `unwrap` and `expect` are for broken
  invariants only, with a message that says which invariant. Game code may
  `expect` on start-up data it ships with.
- No `unsafe`: the workspace denies it. The one exception is the AVX2/FMA
  dispatch in `breakit-core/src/blockchol.rs`, which opts out with a
  module-level `#![expect(unsafe_code, reason = ...)]` and a `// SAFETY:`
  comment on every block. A new exception needs the same, and the solver
  owner's agreement first.
- Lint exceptions: `#[expect(clippy::lint_name, reason = "...")]` on the
  smallest item that needs it. No crate-wide `allow` for new code.
- Docs: every public item in `breakit-core`, `breakit-mesh` and
  `breakit-scenario` gets a `///` comment that says what it is and its unit.
  Module docs (`//!`) say what the module is for and how it fits.
- Tests: unit tests in a `#[cfg(test)] mod tests` at the bottom of the file;
  cross-module and physics-validation tests in the crate's `tests/`; shared
  test helpers in `tests/common/mod.rs`. Solver changes keep the static FEM
  oracle tests green in release mode.
- Examples are for benchmarks and visual checks (`examples/`), not for logic
  that tests should cover.
- `GpuMember` in `breakit-mesh` and `Member` in
  `breakit-render/src/member.wesl` stay field-for-field identical.
