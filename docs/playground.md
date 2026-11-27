# Playground

A free load bench for trying structures out and seeing how the physics
behaves. No scoring, no budget, no limits, any table. Build as in the
sandbox, then run the real-time solver and add, switch and tune loads while
it runs.

Open it from the test register: **Sandbox** tab, sheet **PG Playground**,
then *Open playground*. That opens the build screen (sandbox rules: place
supports and point loads yourself, `T` cycles the table). `Space` starts the
playground run instead of a graded test. `Esc` in the run goes back to the
build; the bench keeps its settings.

## The bench (left)

| Load | Settings | Model (`breakit_core::loads`) |
| --- | --- | --- |
| Wind `1` | speed, direction, gusts on/off, turbulence | `Wind`: drag on every member, shielding, vortex shedding; Kaimal gusts |
| Shake table `2` | sine / synthetic earthquake / sine sweep, peak (g), frequency, direction, new record | `Quake`: `Harmonic`, `Synthetic` (Kanai-Tajimi, 20 s, with cross and vertical components), `Sweep` (0.5 Hz up to the set frequency over 30 s) |
| Water `3` | level, current, waves calm / regular / surge, height, period | `Water`: Morison loads and buoyancy, Airy waves, a solitary wave from the paddle |
| Traffic `V` | truck mass, speed, axles; send | `Traffic`: sprung axles along a lane found on the structure |
| Drop weight `B` | mass, height; drop on the picked node or member, else the middle | `Impactor::dropped` |
| Gravity | 0 to 5 g | extra `(g - 1) m g` on every free node |
| Point loads `P` | size; click nodes to place, clear | constant node forces |

Switching wind, shaking or water off and on fades their load in or out over
1 s rather than switching it at once (a sudden load overshoots about 2x).
Wind speed and gravity follow their settings with a 0.5 s time constant.
Changing any other setting of a load rebuilds that load only; the others
keep their state. Trucks and weights are new load cases each time they are
sent.

The truck lane is the level with the most nodes, along x through their
middle, with an 8 m approach ramp at each end (`breakit_playground::deck_lane`).
Water uses the flume's bed (-3.2 m) on the wave flume and the basin floor
elsewhere; the paddle is at x = -13.6 m.

## Time (bottom)

`Space` run / pause, `N` one 1/60 s step (pauses), `,` `.` slower and faster
(1/8 to 2x), `R` restart from the build with the same bench, `T` next table
(restarts), `F` frame the structure.

## Overlays and inspector (right)

- `H` stress colours on the members (utilisation ramp).
- `A` applied forces: one arrow per loaded node, the largest 1.6 m long.
- `X` support reactions: arrows and a value in kN beside each support.
- `D` deformation ghost: the deflected shape exaggerated 10, 50 or 200 times,
  coloured by utilisation.
- Click a member or node to inspect it. Members show utilisation, axial
  force, largest end moment, damage state, damage, plastic strain and joint
  utilisation; nodes show displacement, vertical movement, speed, applied
  force, reaction and lumped mass.
- The graph under the inspector plots one quantity of the picked member or
  node over the last 10 s, bars from the zero line. `G` picks the next
  quantity (members: utilisation, axial, end moment; nodes: displacement,
  vertical, speed).

Reactions and member end forces come from `breakit_playground::forces`: the
force a member puts on its ends is `F_a = N e - sum over axes of (m_a + m_b)
(u x e) / L`, `F_b = -F_a`, from the solver's axial force and end moments,
and a support's reaction balances the members meeting it and the load
applied there. Tests check it against a cantilever and an off-centre
simple span.

## Saving

`F5` saves and `F9` loads slot 1 (`Ctrl+1..3` save, `Ctrl+Shift+1..3` load
slots 1 to 3). A slot holds the design, the table and the bench as RON
(`breakit_playground::SavedPlayground`), in a `playground` folder next to
the `logs` folder, or in `BREAKIT_PLAYGROUND_DIR`.

## Where things live

- `crates/breakit-playground` (engine-free): `Setup` and `Param` stepping,
  `Live` (the simulation that follows the bench), `forces`, `Probe` and
  `Trace`, saved files.
- `crates/breakit-game/src/playground/`: the screen's life cycle and keys
  (`mod.rs`), panel (`panel.rs`), gizmo overlays and rig stand-ins
  (`overlay.rs`), save slots (`files.rs`), scripted screenshots
  (`capture.rs`, `BREAKIT_PLAYGROUND_CAPTURE=<dir>`).
- Shared hooks: `Screen::Playground` and `UiAction::Playground` in
  `flow.rs`, `Scenario::playground` in `scenario.rs`, and `Simulation` in
  `analysis.rs` holds either a test run or a playground run.

## Open points

- The rigs are gizmo stand-ins (wind streaks, water lines, truck boxes,
  weight spheres). When the visible rigs and the water renderer land, they
  should read `Live::rig_state()` and `Live::table_offset()` the same way as
  in a test. The shake table offset is not drawn yet.
- Only built-in tables; scenario tables can join the `T` cycle when
  scenarios choose their own.
