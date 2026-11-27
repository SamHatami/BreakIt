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

The state comes straight from the run: `TestRun::rig_state()` reports every load case of the sheet (vehicles with their wheel contacts and bounce, the shake table's displacement, the fan wall, the flume and its paddle), and `rigs.rs::to_lab` copies it into `RigLive`. The playground's free bench reports its rigs the same way (`Live::rig_state()`), so they follow the loads placed there too. Before a test, `setup_for` builds the same load set from the sheet (`Binding::load_set`) so the right rigs are already at the table. With simulated water on the table, the paddle plate takes its stroke from the water solver (`LiveWater.sim.drive()`), whose upstream edge it is, not from the load model's `FlumeRig`: the two differ slightly for regular waves.

## Sizes

The structure is full size and the hall is drawn `LabSettings::model_scale` times larger (`docs/lab.md`). The procedural rigs are built directly in world units, sized by what they act on: the fan array by the jet a wind scenario needs, the paddle by the flume, the actuators by the shake platform. They take the hall floor from `LabSettings::floor_y()`. The rig park needs the floor within 30 world units of the table kept clear. The vehicles are models like the structure: a truck with a 3.4 m wheelbase is 3.4 world units long.

## Hero models

The models in `assets/props/` (made by Sam, metres, +y up, standing on y = 0) replace the procedural geometry. They are built into `breakit-lab` (`rigs/hero.rs`), so a release needs no asset folder. To change one, overwrite its file with the same name, node names and frame, and rebuild. `HeroProps(false)` inserted before the lab starts (`--procedural` in the lab example) gives the procedural rigs back.

| File | Used as | Size | Moving nodes |
| --- | --- | --- | --- |
| `industrial_fan_wall.glb` | Fan wall. Blows along its +z | Real lab hardware: `model_scale` times its metres, standing on the hall floor. At 1:10 it is 27 high and 26 wide beside the 28 x 16 table | `fan_rotor_1` .. `fan_rotor_9` spin about their z |
| `hydraulic_actuator.glb` | The four shake actuators. Barrel along +y from the base pin | Fitted to the 3.2 from reaction bracket to platform edge | `piston_rod` slides along y with the platform |
| `shake_table.glb` | Platform of the shake table insert (the insert's own platform is hidden) | Platen edge fitted to `ShakeTable::size`, top flush with the table | `table_platen` rides the table motion |
| `model_dump_truck.glb`, `model_coil_hauler.glb`, `model_mobile_crane.glb` | Vehicles whose scenario name contains "tipper" or "dump", "coil", "crane". +x forward | Fitted so the first and last axle of the model stand on the first and last axle load | `chassis` rides the bounce; the wheels (children of the chassis) stand on the deck and roll about z |
| `overhead_gantry_crane.glb` | A second crane in the hall, at `hero::GANTRY_AT` (lab metres), spanning the hall | `model_scale` times its metres | none yet (`trolley`, `hook` are there) |

What the code relies on in each file (axle positions, rotor positions, node names) is in `rigs/hero.rs` and checked against the files by its tests, so a re-exported model that moved an axle fails a test instead of drawing wrongly.

A truck model keeps its proportions: it is about twice as long as it is wide, 6 times its file size for a 2.5 m wheelbase. Scenarios should give convoys the models' axle spacing and a deck wide enough (tyres are 0.44 of the model's 0.42 to 0.61 wheelbase). A middle axle of the model keeps its place on the model and follows the deck between the loads. There is no box truck model: other vehicles are the procedural box truck.

The smoke, the blade blur and the seismograph trace are effects and stay with a hero model; the fan wall's tufts and beacon and the vehicles' brake lamps are only on the procedural rigs.

### Adding another model

Each rig is a root entity with one child per moving part. A part's `Role` says what moves it. A new file goes in `assets/props/`, in `hero::FILES`, and under its slot's `RigSlot::file_stem`; the caller of `spawn_rig` gives the model's frame in the rig's frame. Nodes are bound by name, either the names below or the aliases in `hero::alias`. A node that does not turn or slide about the role's usual axis gets an `Axis`.

| Rig | Frame | Driven nodes |
| --- | --- | --- |
| Fan wall | Origin at table-top level under the middle of the outlet face, +x downstream | `rotor.N` spin about their x, `tuft.N` swing about their z, `lamp_on`, `lamp_off` |
| Shake actuators | Table frame | `rod.N` slide along their x, `platform` rides the table |
| Wave paddle | Table frame | `plate` slides along x |
| Vehicles | Origin on the ground under the front axle, +x forward | `body` rides the bounce, `wheel.0` .. `wheel.n` (front first) follow the contacts and roll about their z, `brake_light` |

Node names are matched without case, and anything after a second dot is ignored (`Wheel.1.L` and `Wheel.1.R` both roll with axle 1). A model's driven nodes keep their rest pose from the file; the drive is applied on top.

## Running it

`cargo run --release -p breakit-lab --example lab -- --rig wind` (or `quake`, `flume`, `trucks`) calls a rig and runs it on made-up signals; R steps through them, V changes the view (`rig`, `rigs`, `park` and `bay` look at the rigs, `crane` at the gantry crane). Add `--capture <dir> --views rig,play` for screenshots, `--scale 10` to draw the hall at another scale, `--procedural` to leave the hero models out.

## Open points

- Smoke streamers go straight through the structure: they do not know it is there. Bending them round members needs the structure's shape passed to the lab.
- Table motion is true to the solver: a 0.4 g, 2 Hz shake moves the table 25 mm at full size, which is a small movement on screen. The seismograph carries the read-out.
- The procedural box truck is a fixed 1.3 wide and there is no hero model for it; beside the hero trucks it looks small.
- The spare vehicles in the park are still the procedural ones.
- The instrument cart and the paddle have no hero model.
- The gantry crane stands still: its `trolley` and `hook` nodes are not driven.
- No drawn impactors yet (loose equipment falling, `RigState::impactors`).
- Displacement sensors under bridge nodes, wind tufts on the structure and mass blocks on a tower need the structure's nodes and belong with the scenarios.
- The crane hook does not carry the paddle when it is lifted over.
