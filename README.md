# BreakIt

A structure-builder game where real structural mechanics decide whether your build survives.
Plan: https://claude.ai/artifact/LE9dveFS17PTMrkqGWwSvT

## Layout

- `crates/breakit-core`: the structural solver. Pure Rust, no game-engine dependency.
  - `statics`: linear 3D frame FEM (direct stiffness). Exact for small deflections; build-mode heatmap and test oracle.
  - `dynamics`: real-time corotational beam solver with plastic hinges, axial yield and fracture.
- `crates/breakit-game`: the Bevy app (Bevy pinned to `=0.20.1`, Rust 1.97.1+).
  - `design`: the player's nodes, beams, supports and loads; `to_structure()` is the only hand-off to the solver.
  - `analysis`: `FrameState`, the one physics read-out the renderer uses. Static solve in build mode, `DynamicSim` in simulate mode, in `BreakitSet::Physics`.
  - `flow`, `scenario`, `testrun`, `guides`, `history`: screens and the play loop, the game's view of `breakit-scenario` sheets, the test on screen, scenario guides, undo/redo.
  - `camera`, `build`, `render`, `ui`: orbit camera, build tool, beam/node drawing coloured by utilisation, game UI on a swappable theme layer (`docs/game-ui.md`).
  - `capture`: `BREAKIT_CAPTURE=<dir>` walks the whole loop and screenshots each screen, then exits (for headless checks).

## Run

```
cargo test --release
cargo run --release --example bench -- --sizes s,m,l --static   # speed and accuracy on game-sized structures
cargo run --release --example probe -- 8 1        # direct solver, 8 substeps, 1 iteration
cargo run --release --example probe -- 40 1 gs    # plain XPBD Gauss-Seidel, for comparison
```

Game:

```
cargo run -p breakit-game                      # pick a sheet, build, Space runs the test; ? or Esc > Controls lists the keys
# Linux needs: libudev-dev libwayland-dev libxkbcommon-dev (and libxkbcommon-x11-0 at runtime under X11)
BREAKIT_CAPTURE=/tmp/shots xvfb-run cargo run -p breakit-game   # headless screenshots (needs a Vulkan driver, e.g. mesa lavapipe)
```

Solver status: see `docs/solver-spike.md`. Game UI, flow and controls: `docs/game-ui.md`.
