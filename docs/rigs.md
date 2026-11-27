# The load rigs

Every scenario load comes from a rig you can see. The forces are the load models in `breakit-core::loads` (`docs/loads.md`); the rigs are drawn by `crates/breakit-lab/src/rigs/` and driven from the running test by `crates/breakit-game/src/rigs.rs`.

| Rig | File | What it shows |
| --- | --- | --- |
| Fan wall | `rigs/fan.rs` | 5 by 4 ducted fans on a cart: rotors spin up with the fan drive and blur, smoke streamers from a rake are carried downstream at the wind speed and wander more in gusts, wool tufts on a wire lift with the dynamic pressure, a running beacon, a console with an emergency stop |
| Shake table | `rigs/shake.rs` | Four servo-hydraulic actuators on the table surround whose rods follow the platform, the platform itself and what stands on it moving with the table displacement, an accelerometer, and an instrument cart whose screen draws the table acceleration over the last 8 s |
| Wave paddle | `rigs/paddle.rs` | A piston plate hanging from a carriage that drives along racked rails on the channel banks. Its wet face is the water's upstream wall |
| Model vehicles | `rigs/truck.rs` | Box truck, dump truck, coil hauler and mobile crane built around the axles the load model gives them: each wheel set sits where its tyre presses on the deck and rolls with the speed, the body rides on the suspension bounce and pitches with it, brake lamps light |
| Rig park | `rigs/park.rs` | The marked bay where the fan wall and the instrument cart wait and roll in from, the paddle's trolley (it is lifted over), spare vehicles, test masses, a guard rail |

## How a rig is driven

The lab does not depend on the solver. The game writes one resource:

```rust
pub struct LabRigs {
    pub setup: RigSetup, // which rigs the scenario needs: they travel to the table or back to the park when it changes
    pub live: RigLive,   // this frame: wind, table, flume, vehicles, and the test time
}
```

`RigLive` mirrors `breakit_core::loads::RigState` field for field in `f32` and world units; `breakit-game/src/rigs.rs::to_lab` is the conversion. Things that integrate (rotor angle, wheel roll, smoke, the seismograph) advance with `RigLive::time`, so they slow down, pause and stop with the simulation.

In the game, `setup_for` reads the sheet before the test (so the rig is at the table while you build) and `setup_from` adds any rig whose load is on. The solver works in the shake table's frame, so `visuals.rs` adds the table displacement to the drawn structure, and the lab moves the platform (`ShakePlatform` parts of the table, built into `TableTop::platform`).

The state comes straight from the run: `TestRun::rig_state()` reports every load case of the sheet (vehicles with their wheel contacts and bounce, the shake table's displacement, the fan wall, the flume and its paddle), and `rigs.rs::to_lab` copies it into `RigLive`. The playground's free bench reports its rigs the same way (`Live::rig_state()`), so they follow the loads placed there too. Before a test, `setup_for` builds the same load set from the sheet (`Binding::load_set`) so the right rigs are already at the table.

## Sizes

The structure is full size and the hall is drawn 20 times larger (`docs/lab.md`). The rigs are real hardware next to a 1:20 model, so they are built directly in world units at 20 units to the metre: the fan wall is 0.7 m wide with 14 cm fans, the instrument cart is 0.5 m wide. The vehicles are models like the structure: a truck with a 3.4 m wheelbase is 3.4 world units long.

## Replacing a rig with a hero model

Each rig is a root entity with one child per moving part. A part's `Role` says what moves it. To replace the procedural geometry, put a glTF binary in `assets/props/` under the slot's file name. The game loads whatever is there at startup (`RigModels::from_folder`); a slot without a file keeps its procedural rig.

| Slot | File | Frame (world units, y up) | Nodes that are driven |
| --- | --- | --- | --- |
| Fan wall | `fan_wall.glb` | Origin at table-top level under the middle of the outlet face, +x downstream. The hall floor is 17.2 below. Fan `i` is at `fan::fan_centre(i)` | `rotor.0` .. `rotor.19` spin about their x, `tuft.0` .. `tuft.12` swing about their z, `lamp_on`, `lamp_off` |
| Instrument cart | `instrument_cart.glb` | Origin at table-top level under the cart, screen facing +z, centred 5.6 up, 9 by 5.4 | none (the trace is drawn over the screen) |
| Shake actuators | `shake_actuators.glb` | The world's: origin in the middle of the platform. Push axes in `shake::actuator_axes` | `rod.0` .. `rod.3` slide along their x, `platform` rides the table |
| Wave paddle | `wave_paddle.glb` | Origin in the middle of the table top, +x the wave direction, plate face at x = -13.6 | `plate` slides along x, `lamp_on`, `lamp_off` |
| Vehicles | `truck_box.glb`, `truck_dump.glb`, `truck_coil.glb`, `truck_crane.glb` | Origin on the ground under the front axle, +x forward | `body` rides the bounce, `wheel.0` .. `wheel.n` (front first) follow the contacts and roll about their z, `brake_light` |

Node names are matched without case, and anything after a second dot is ignored (`Wheel.1.L` and `Wheel.1.R` both roll with axle 1). A model's driven nodes keep their rest pose from the file; the drive is applied on top. The smoke, the blade blur and the seismograph trace are effects and stay with a hero model.

This path has unit tests for the name matching only: no hero model has been loaded through it yet. A model truck has the axle spacing it was made with, so its scenario must use those axles.

## Running it

`cargo run --release -p breakit-lab --example lab -- --rig wind` (or `quake`, `flume`, `trucks`) calls a rig and runs it on made-up signals; R steps through them, V changes the view (`rig`, `rigs` and `park` look at the rigs). Add `--capture <dir> --views rig,play` for screenshots.

## Open points

- Smoke streamers go straight through the structure: they do not know it is there. Bending them round members needs the structure's shape passed to the lab.
- Table motion is true to the solver: a 0.4 g, 2 Hz shake moves the table 25 mm at full size, which is a small movement on screen. The seismograph carries the read-out.
- Vehicles are a fixed 1.3 wide. Scenarios with a wider lane could pass a width.
- No drawn impactors yet (loose equipment falling, `RigState::impactors`).
- Displacement sensors under bridge nodes, wind tufts on the structure and mass blocks on a tower need the structure's nodes and belong with the scenarios.
- The crane hook does not carry the paddle when it is lifted over.
