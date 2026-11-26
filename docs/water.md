# Water

The water on the play table is simulated, and the same simulated water is drawn and pushes on the structure. Three pieces:

| Crate | What |
| --- | --- |
| `breakit-water` (engine-free) | the solver, the paddle, foam and spray, the hook into the core load model |
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

Not covered: the water does not feel a floating body's displacement (only its drag), and the static check in build mode knows nothing about buoyancy, so a structure held up only by floats has no static solution there.

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

`WaterQuality` follows the lab's F4 presets: low is alpha-blended without ripples or spray, medium adds refraction and ripples, high adds caustics and a finer surface, ultra the finest. In the game the solver cell is 25 cm on low, 20 cm on medium and high, 15 cm on ultra.

`cargo run --release -p breakit-water-render --example flume` is a flume in the lab with a beach and a pier: 1 waves, 2 surge, 3 flood, V views. `-- --capture <dir>` and `-- --clip <dir> --view beach --frames 100 --fps 20` save stills and frames headless (under `xvfb-run`).

## Cost

One core, `cargo run --release -p breakit-water --example water_bench` (28 m by 6 m flume, 3.5 s waves breaking on a beach):

| Cell | Cells | ms per 60 Hz frame |
| --- | --- | --- |
| 20 cm | 4 140 | 2.1 |
| 15 cm | 7 360 | 5.7 |
| 10 cm | 16 560 | 18.0 |

Two thirds of it is the pressure solve. It is scalar code; a GPU compute backend behind the same `Field` interface, checked against this solver within a tolerance, is the planned way to finer grids.

## Open

- The flume's walls are infinitely high for the water: a crest higher than the table top does not spill over it.
- Dropped equipment (scenario payloads) is not yet pushed by the water; `Field::column` and `WaterField::kinematics` are there for it.
- Waves shorter than about three water depths do not travel in this model, so a splash stirs the water where it lands but sends out no rings; ripple rings would be a shader effect.
- The ravine's stream is still the lab's flat water.
- The paddle the lab draws does not move with the solver's paddle yet (`LiveWater::sim.drive()` has its position).
