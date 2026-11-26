# The lab scene

`crates/breakit-lab` (Bevy, same pin as the game) draws the world around the structure: a structures and hydraulics test hall with the play table in it. Everything is generated in code; there are no assets to author or re-export.

## Scale: a full-size structure on a model table

The solver works in SI units and the structure is full size: the starter bridge is 6 m long, and 6 world units. The fiction is that it is a 1:20 model on a table in a real hall. So the hall is drawn `LabSettings::model_scale` (20) times larger than its real size: a 60 m hall is 1,200 world units long, a 0.86 m high table stands 17.2 world units above the floor.

- World origin is the middle of the table top. The table's playing surface is at y = 0, where the solver's ground is.
- `LabSettings::lab_to_world` maps lab metres to world units; `floor_y()` is the hall floor in world units.
- Depth of field uses a sensor scaled by the same factor (`sensor_height = 18.66 mm x 20`), so the blur is what a real camera would give looking at a real 1:20 model. That is what makes the table read as a miniature and lets the hall go soft behind it.

## What is in the hall

`hall.rs` (lab metres, floor at y = 0, length along x):

- 10 bays of 6 m, 30 m clear span, eaves 11 m, ridge 13.5 m. Portal frames are I-section columns and rafters with haunches, apex and base plates and anchor nuts.
- Purlins, side rails, cross-bracing in the end bays, wind posts in the gables.
- Profiled sheeting with ribs, a precast panel band at the bottom, a band of high windows, roof lights in every other bay, a large window in the far gable, a roller door with hazard-striped bollards at the near end.
- Overhead crane: runway beams on column brackets, double box-girder bridge, end carriages, trolley, ropes and hook block.
- 20 high-bay lamps hung above the crane.
- Mezzanine with a lit control room, handrails and a stair.
- Floor: saw-cut joints, painted walkways, a hatched keep-clear box, a steel strong floor with anchor points.

`props.rs`: a glass-walled wave flume with piston paddle, absorber beach and wave-gauge carriage; a reaction frame with an actuator and a concrete beam specimen; a column test rig; a small shake table; two hydraulic power units; tanks with ladders; pumps; pipe runs in service colours with flanges, brackets and drops; cable trays; electrical cabinets with indicator lamps; pallet racking; benches, a tool chest, gas bottles, test cubes, a steel stillage, timber, a scissor lift, a pallet truck and cones. No people. The dressing is seeded, so it is the same every run. A test keeps the table's patch of floor clear.

Small emissive things (indicator lamps, lit windows, lamp faces) are there on purpose: out of focus they become bokeh highlights.

`LabAnchors` lists where things are (power units, pumps, crane hook, flume paddle, control room, cabinets, test rigs, lamps) in world units, for sound and scenario rigs, since the scenery is merged into a few meshes and has no per-prop entities.

## The play table and scenarios

The table is a model basin: frame, rim with scale ticks every metre of play space, legs, shelf. What fills the basin comes from a `TableInsert`:

```rust
pub trait TableInsert: Send + Sync + 'static {
    fn name(&self) -> &str;
    fn build(&self, top: &mut TableTop); // fill y = -BASIN_DEPTH..0 inside top.half
}
commands.insert_resource(PlayTable::new(Ravine { gap: 5.0, depth: 3.2 }));
```

`TableTop::surface` adds a block of table top with the grid on it (minor 0.5 m, the build snap, major every 2 m, aligned to the world origin so lines are where nodes snap); `grid` and `contour_bank` (stacked foam layers, like an architect's terrain model) are there for terrain. Built in: `Ravine` (default: banks and a stream, sized so the starter bridge's supports sit on the banks), `Flat`, `Flume` (a water channel along x) and `ShakeTable` (a platform flush with the surface). Changing `PlayTable` rebuilds the table next frame. In the game, T cycles the built-ins in build mode until scenarios choose their own.

## Cost

Measured headless (`cargo test` builds and counts; frame times on the container's software rasteriser are not meaningful for a GPU):

| | |
|---|---|
| hall, dressing and ravine table | 64 k triangles in 37 meshes (one per material), generated once at startup |
| per-frame CPU | none for the hall; the lens system only copies the focus distance |
| lights | 1 shadowed directional light (cascades sized for the table, hall geometry does not cast), 20 point lights without shadows |
| post | HDR, bloom, TonyMcMapface, distance haze, bokeh depth of field |

On a GPU the likely costs, in order: the bokeh depth of field (switch `LabSettings::dof` to `Some(DepthOfFieldMode::Gaussian)` or `None`), the 20 point lights (`lamp_lights: false` keeps the lamps glowing without lighting), then shadows. Static geometry at this size is not a concern.

The game's frame-time read-out (top right, F3 toggles) shows the average and worst frame over two seconds of real time, the CPU time of physics and of the member visuals, and the lab's triangle and mesh counts. `BREAKIT_PERF_LOG=1` prints it once a second.

## Running it

- `cargo run --release -p breakit-game`: the game in the lab. T table, F3 read-out.
- `cargo run --release -p breakit-lab --example lab`: the lab alone with a stand-in truss. 1-4 table, V next view, B depth of field mode, [ and ] f-number. `-- --capture <dir>` saves every view and table and exits; `-- --bench` prints frame times with bokeh, gaussian and no depth of field.
- `BREAKIT_CAPTURE=<dir>` on the game also saves `lab.png`, a pulled-back view.

## Open points

- The load rigs (fan wall, shake platform actuators, flume paddle on the table, traffic carts) from the concept canvas are not built yet. They belong next to the scenarios that drive them; build them in lab metres and scale by `model_scale`.
- To move the table with a shake rig, expose a table root entity (today the table parts are separate entities at the origin).
- Looks were tuned on a software rasteriser; exposure, lamp intensity and the f-number want a pass on a real GPU.
- No LOD or culling groups for the hall: it is merged per material, so it is all drawn or none. Fine at 60 k triangles.
- Water is static.
