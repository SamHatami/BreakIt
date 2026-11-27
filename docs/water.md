# Water

The water on the play table is simulated, and the same simulated water is drawn and pushes on the structure. Four pieces:

| Crate | What |
| --- | --- |
| `breakit-water` (engine-free) | the solver, the paddle, foam and spray, the hook into the core load model |
| `breakit-water-gpu` (wgpu, no Bevy) | the solver as compute passes, read back into the CPU solver |
| `breakit-water-render` (Bevy) | the surface mesh and shader, the spray, quality presets |
| `breakit-game/src/water.rs` | sets the water up from the play table and the sheet's water case, steps it with the physics |

## The model

`Solver` is a one-layer non-hydrostatic shallow-water solver (Stelling and Zijlema 2003) on a staggered grid: water level and depth-averaged vertical velocity in the cells, horizontal velocities on the faces, one pressure unknown per cell.

- **Why not plain shallow water:** it has no dispersion. Every wave travels at `sqrt(g d)`, so a paddle makes bores instead of wave trains. The pressure term gives `c² = g d / (1 + (k d)² / 4)`, within 3 % of linear theory up to `k d = 2` (in the 2.6 m deep flume: periods from about 2.3 s). Shorter waves are too slow, and there is a cut-off at `ω = 2 sqrt(g / d)` above which the paddle only makes a local disturbance.
- **Why not particles or a 3D grid:** a height field at 20 cm cells costs about 2 ms a frame on one core; a 3D solver at the same resolution is two orders of magnitude more, and the load model only needs depth-resolved velocities, which the depth-averaged solution gives well for the waves it can carry. What a height field cannot do: overturning crests, air pockets, water falling off the table. Breaking waves are bores with foam and spray on them.
- **Scheme:** Stelling and Duinmeijer's momentum-conserving advection with upwind face depths, so bores and dam breaks have the right speed and height, wetting and drying need no special cases, and volume is conserved to rounding. The pressure is a projection: a five-point symmetric system solved by conjugate gradients with a modified incomplete Cholesky preconditioner, warm-started.
- **Breaking:** where the surface rises faster than `0.6 sqrt(g d)` the cell goes hydrostatic until it falls below `0.3 sqrt(g d)` (hydrostatic front approximation), which turns a steep crest into a bore that dissipates like one. The same flag feeds the foam and the spray.
- **Boundaries:** walls, a piston paddle (the face moves, as a velocity boundary), and a radiating outlet that holds a level and a current (reflects about 5 % of a 4 s wave).
- **The structure in the water:** `WaterSim::couple` gives every wet member's drag back to the water as a sink of momentum in its cell, so piles leave wakes and the water sets up in front of them.

`WaterSim` wraps the solver with the foam cover (made by breaking and by wakes, carried with the flow, decaying), spray droplets (ballistic, thrown by breaking crests and by fast water at members) and a `Field` snapshot.

## Forces on the structure

`breakit_core::loads::water::WaterField` is the hook: a `Water` load case with `field: Some(..)` takes surface level, velocity and acceleration at any point from the simulated water instead of from its analytic waves, and applies the same Morison drag, inertia and buoyancy. The depth profile under the surface is rebuilt from the depth-averaged solution (uniform horizontal velocity, linear vertical velocity). `Drive::from_case` turns the case's paddle motion, current and still level into the solver's boundary conditions, rescaling the stroke with the solver's own wavemaker function so the waves have the height the case asks for.

The scenario's grading therefore depends on the simulated water. It is deterministic on one machine; across machines (and on a future GPU backend) expect differences in the last digits, so anything scored from it needs a tolerance.

## Floating, flooding and debris

The same load case carries everything that floats:

- **Draft.** Buoyancy and the Morison terms scale with the share of the section that is under the local surface (a circle segment for a tube, a strip for a rectangle), so a member lying in the surface sinks in until it displaces its own weight, and rides the waves.
- **What displaces what.** A closed tube displaces its whole outline while it is sound; every other section displaces only its material. Timber (420 kg/m³) floats, a steel I-beam sinks.
- **Flooding.** A tube whose damage passes `FLOOD_DAMAGE` (0.75) lets the water in and from then on displaces only its steel. A broken member floods at once. A pontoon bridge therefore sinks where it is damaged.
- **Debris.** Each half of a broken member goes with its end node and keeps its (flooded) buoyancy and drag there, so timber pieces come up and float and steel sinks at a terminal speed.
- **Settling.** Drag is quadratic and hardly damps a small bob, so a section in the surface also gets a wave-radiation damping (`RADIATION`, an estimate).
- **Splashes.** `WaterSim::couple` notices nodes dropping into the water faster than 1 m/s and calls `WaterSim::splash`: spray, a patch of foam, and a dent in the surface. `splash` is public for things that are not members (dropped equipment).
- **Pontoons.** `steel-pontoon-s` (CHS 406.4x8) and `steel-pontoon-l` (CHS 813x16) in the parts catalog: about 50 and 200 kg of spare lift per metre.

- **Static check.** `Water::static_support(structure, t)` gives the still-water view of the same model for a static solve: the buoyant force on every node at the built position, and a vertical `NodeSpring` per node with the water-plane stiffness `ρ g × waterline area`.

Not covered: the water does not feel a floating body's displacement (only its drag). The static solver does not take the springs yet (the scenario side is adding that), so until then a structure held up only by floats has no static solution in build mode.

## Validation

`cargo test --release -p breakit-water`:

| Test | Against | Tolerance |
| --- | --- | --- |
| standing wave period in a basin | the model's dispersion relation; linear theory | 0.5 %; 3 % |
| paddle wave height and phase speed | piston wavemaker theory | 5 % |
| solitary wave from Goring's paddle motion | speed `sqrt(g (d + H))`; height kept along the flume | 2.5 %; 12 % |
| dam break | Stoker's solution: bore speed and height | 3 % |
| still water over an uneven bed stays still; volume | exact | 1e-9 |
| shoaling on a slope | linear shoaling; Green's law | 6 %; 12 % |
| solitary wave run-up | Synolakis (1987) | 10 % |
| waves on a beach | they break, in shallow water, and run up | |
| force on a pile in paddle waves | the core's Morison model on Airy waves | 10 % |
| pile in a current | wake behind, set-up in front | |
| dropped timber log | splashes, then floats at its Archimedes draft | 2 cm |
| pontoon raft with cargo | Archimedes draft in still water; heaves with 4 s waves, stays level, nothing breaks | 3 cm |

## Drawing it

`WaterRenderPlugin` draws the `LiveWater` resource. The surface is one mesh over the solver grid, rebuilt on the CPU each frame; vertices carry depth, flow, foam, breaking and curvature. `water.wesl` extends `StandardMaterial`:

- refraction and depth colour through Bevy's screen-space transmission, with the refracted path length as thickness;
- reflections of the hall from the environment map and lamps;
- ripples carried by the flow, rougher where the water is fast or breaking;
- caustics on what is seen through the surface, focused by crests;
- foam from the simulated cover, and a lace along a moving shoreline.

`WaterQuality` follows the lab's F4 presets: low is alpha-blended without ripples or spray, medium adds refraction and ripples, high adds caustics and a finer surface, ultra the finest. In the game the solver cell is 25 cm on low, 20 cm on medium and high, 15 cm on ultra on the CPU, and 20, 15 and 10 cm on the GPU.

`cargo run --release -p breakit-water-render --example flume` is a flume in the lab with a beach and a pier: 1 waves, 2 surge, 3 flood, V views. `-- --capture <dir>` and `-- --clip <dir> --view beach --frames 100 --fps 20` save stills and frames headless (under `xvfb-run`).

## Cost

One core, `cargo run --release -p breakit-water --example water_bench` (28 m by 6 m flume, 3.5 s waves breaking on a beach):

| Cell | Cells | ms per 60 Hz frame |
| --- | --- | --- |
| 20 cm | 4 140 | 2.1 |
| 15 cm | 7 360 | 5.7 |
| 10 cm | 16 560 | 18.0 |

Two thirds of it is the pressure solve, and it is scalar code. `PressureSolver::RedBlack` (16 sweeps) instead of conjugate gradients takes the 10 cm grid from 18 to about 12 ms; the GPU backend below is the way to finer grids.

## On the GPU

`breakit-water-gpu` runs the same substep as compute passes (`solver.wgsl`, to be kept in step with `Solver::substep` formula by formula). `GpuSolver::step` goes where `Solver::advance` would: `WaterSim::advance_with(dt, drive, |solver, dt| gpu.step(device, queue, solver, dt))`.

- **What differs from the CPU reference.** The arithmetic is `f32`. The pressure system is relaxed by a fixed number of red-black over-relaxation sweeps (16, factor 1.6, warm-started from the last substep) because conjugate gradients with an incomplete Cholesky factor is a chain through every cell and does not run in parallel; `PressureSolver::RedBlack` is the same relaxation on the CPU and is what the GPU is tested against. The outflow limiter scales all faces from the state before it instead of cell after cell.
- **The CPU solver is the mirror.** The state lives on the GPU. Each step copies it into a staging buffer that is mapped without waiting, and whatever has arrived is loaded into the CPU `Solver` (`Solver::load_state`). Foam, spray, `Field`, the loads on the structure and the renderer read that solver as before, one step late: at 60 Hz the water the structure feels and the water drawn are 17 ms behind the water computed. The substep is sized from the mirror with a 15 % margin for that. `GpuSolver::sync` waits for the GPU (tests, captures).
- **What goes up each step.** Bed, sources, sponge and blockage (four floats per cell, twice), the paddle and the boundaries. A splash (`Solver::displace`) goes up as extra source over the step, so the state never has to come back down first. Edits that replace the state (`fill_to`, `set_surface`, `set_bed`, `set_velocity_x`; `Solver::revision` counts them) upload it whole and drop read-backs still in flight.
- **Size.** Eight storage buffers of `(nx + 1)(nz + 1)` entries, 16 bytes each (the pressure 4). A substep is 9 dispatches plus two per sweep, 41 in all, each its own compute pass because each reads what the one before wrote.

**Tolerance, not equality.** Different GPUs round differently (fused multiply-add, `pow`, `sqrt`), so nothing may depend on the water being the same bit for bit: tests compare within tolerances, and limits scored from water need a margin. Measured here on a software Vulkan adapter (`cargo test --release -p breakit-water-gpu`), GPU against the CPU with the same relaxation:

| Case | Surface difference, rms | at most |
| --- | --- | --- |
| Flume, 0.3 m waves breaking on a beach, 14 s | 0.6 µm | 2 µm |
| Basin with a current, a pile, an open side and a splash, 4 s | 23 µm | 0.3 mm |
| Flume with the mirror lagging against waiting every step, 8 s | 38 µm | 74 µm |

The red-black relaxation itself differs from the conjugate-gradient reference by 0.3 mm rms and 1.7 mm at most in 0.6 m waves (CPU against CPU). The tests pass without checking anything where there is no adapter; CI installs Mesa's software Vulkan so they do run there.

**In the game** the CPU is the default. F6 or `BREAKIT_WATER=gpu` steps the water on the GPU the game draws with (`WaterBackend`); the frame-time read-out (F3) says where the water runs and on what grid. The grid follows the backend the next time the water is set up (a running test keeps its grid and only changes where it is stepped).

**Not timed on real hardware.** Everything here was built and checked on a software adapter, which says it is right and nothing about how fast it is. To find out: on the flume table, run a test with waves, open the read-out (F3) and compare the frame time with F6 off and on. Until someone has, the CPU stays the default.

## Open

- The GPU backend has not been timed on a GPU (see above). Foam, spray and the surface mesh are still built on the CPU from the mirror each step, which is what will limit grids much finer than 10 cm.
- The flume's walls are infinitely high for the water: a crest higher than the table top does not spill over it.
- Dropped equipment (scenario payloads) is not yet pushed by the water; `Field::column` and `WaterField::kinematics` are there for it.
- Waves shorter than about three water depths do not travel in this model, so a splash stirs the water where it lands but sends out no rings; ripple rings would be a shader effect.
- The ravine's stream is still the lab's flat water.
- The paddle the lab draws does not move with the solver's paddle yet (`LiveWater::sim.drive()` has its position).
