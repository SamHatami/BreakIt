# BreakIt

A structure-builder game where real structural mechanics decide whether your build survives.
Plan: https://claude.ai/artifact/LE9dveFS17PTMrkqGWwSvT

## Layout

- `crates/breakit-core`: the structural solver. Pure Rust, no game-engine dependency.
  - `statics`: linear 3D frame FEM (direct stiffness). Exact for small deflections; build-mode heatmap and test oracle.
  - `dynamics`: real-time corotational beam solver with plastic hinges, axial yield and fracture.
- `crates/breakit-game` (next): Bevy app, one pinned Bevy release.

## Run

```
cargo test --release
cargo run --release --example bench -- --sizes s,m,l --static   # speed and accuracy on game-sized structures
cargo run --release --example probe -- 8 1        # direct solver, 8 substeps, 1 iteration
cargo run --release --example probe -- 40 1 gs    # plain XPBD Gauss-Seidel, for comparison
```

Status: see `docs/solver-spike.md`.
