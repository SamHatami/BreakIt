# The lab scene

`crates/breakit-lab` (Bevy, same pin as the game) draws the world around the structure: a structures and hydraulics test hall with the play table in it. Everything is generated in code; there are no assets to author or re-export.

## Scale: a full-size structure on a model table

The solver works in SI units and the structure is full size: the starter bridge is 6 m long, and 6 world units. The fiction is that it is a 1:10 model on a large test table in a real hall. So the hall is drawn `LabSettings::model_scale` (10) times larger than its real size: a 60 m hall is 600 world units long, a 0.86 m high table stands 8.6 world units above the floor, and the 28 x 16 table is 2.8 x 1.6 m.

It was 1:20 until Sam's first playtest: the models read as tiny miniatures in a huge hall, and the depth of field (which scales with the model, see below) blurred much of the table. 1:10 makes the test models read twice as large against the hall.

- World origin is the middle of the table top. The table's playing surface is at y = 0, where the solver's ground is.
- `LabSettings::lab_to_world` maps lab metres to world units; `floor_y()` is the hall floor in world units.
- Depth of field uses a sensor scaled by the same factor (`sensor_height = 18.66 mm x 10`), so the blur behind the table is what a real camera would give looking at a real 1:10 model. The table itself is always sharp: see "Depth of field" below.
- The rig park and the parked rigs stand on a patch of floor round the table that the dressing keeps clear: `LabSettings::clear_floor_half`, the table plus `CLEAR_FLOOR_MARGIN` (30 world units) on each side. The rigs are sized by the structure, so the margin is in world units and does not change with the model scale.

## What is in the hall

`hall.rs` (lab metres, floor at y = 0, length along x):

- 10 bays of 6 m, 30 m clear span, eaves 11 m, ridge 13.5 m. Portal frames are I-section columns and rafters with haunches, apex and base plates and anchor nuts.
- Purlins, side rails, cross-bracing in the end bays, wind posts in the gables.
- Profiled sheeting with ribs, a precast panel band at the bottom, a band of high windows, roof lights in every other bay, a large window in the far gable, a roller door with hazard-striped bollards at the near end.
- Overhead crane: runway beams on column brackets, double box-girder bridge, end carriages, trolley, ropes and hook block.
- 20 high-bay lamps hung above the crane.
- Mezzanine with a lit control room, handrails and a stair.
- Yellow guard rails round the rig bays, drain gratings along the flume, a cantilever rack of steel sections, silos.
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

`TableTop::surface` adds a block of table top with the grid on it (minor 0.5 m, the build snap, major every 2 m, aligned to the world origin so lines are where nodes snap); `grid` and `contour_bank` (stacked foam layers, like an architect's terrain model) are there for terrain. Built in: `Ravine` (default: banks and a stream, sized so the starter bridge's supports sit on the banks), `Flat`, `Flume` (a water channel along x) and `ShakeTable` (a platform flush with the surface). Changing `PlayTable` rebuilds the table next frame. `contact_boxes` gives the solver's ground contact the same shape as axis-aligned boxes (the default is a flat slab), so the lab crate does not depend on `breakit-core`. In the game, T cycles the built-ins in build mode until scenarios choose their own.

## The look: light, reflections, lens

`look.rs` owns everything that is not geometry. The reference is a photograph of an architect's model in a working hall on an overcast afternoon: cool light from the roof lights, warm pools under the high-bay lamps, a real lens.

- **Reflections and ambient light** come from an environment map of the hall that is painted, not rendered (`envmap.rs`). The hall is a box with a pitched roof, so what the table sees in any direction follows from `HallSpec`: floor with lamp pools, precast band, window band, roof lights, lamps, crane bridge. It is filtered on the CPU once at startup (about 10 ms at 128 texels) into a mip chain for glossy reflections and a small cube for diffuse light. This is why stainless tanks and wet surfaces show the lamps, and why undersides are warmer and darker than tops. If the hall's layout changes, `HallRadiance::at` has to follow.
- **Daylight** is one cool directional light with cascaded shadows sized for the table.
- **Lamps** are warm spot lights pointing down, nearest the table first. They are driven harder than real fittings would be next to daylight (`LAMP_GAIN`) so the pools read.
- **Who shadows what**: the hall is on render layer 1 (`HALL_LAYER`), the table and structure on layer 0, and lab cameras see both. The daylight only takes shadows from layer 0, because the roof would otherwise shade everything; the lamps take them from both. Anything you add to the hall should go on layer 1, anything on the table on layer 0.
- **Ambient occlusion** reaches further the further away the camera is focused (`ao_radius`): 0.8 m in the joints of the structure up close, 0.6 m of hall when pulled back, where it grounds the machinery.
- **Camera**: exposure fixed at EV 9.2, TonyMcMapface tonemapping, a light grade (a touch of contrast, cooler shadows, warmer highlights), bloom only above a threshold so nothing glows that would not in a real hall, a soft vignette, distance haze, and the depth of field from the scaled sensor.

### Depth of field

A real lens blurs in front of and behind what it is focused on, so a lens focused on the middle of the table blurs its near and far edges: on the first playtest the blur covered much of the play area. The lab keeps the whole table sharp instead:

- The lens focuses on the far edge of the table (and 10 world units above it, room for towers and rig gantries), measured as view depth from the camera, or on the orbit focus if that is further (`look::sharp_to`).
- Nothing nearer than the focus is blurred. Bevy's depth of field has no setting for that, so the lab swaps in its own copy of Bevy's shader, `src/dof.wesl`, which differs only in `calculate_circle_of_confusion` (`max(depth - focus, 0)` instead of `abs(depth - focus)`). `look::override_dof_shader` replaces the embedded bytes at `bevy_post_process/dof/dof.wesl` in `LabPlugin::finish`, before the render pipelines load it. When the Bevy pin moves, copy the new `dof.wesl` and make the same one-line change.
- Behind the table the hall goes soft as a real lens would make it. At the default view with f/2 the far end of the hall has a blur of about 4 px at 900 lines, against 20 px or more before. `LabSettings::f_stops` sets it.

### Surfaces

`surface.rs` and `surface.wesl` give the scenery its materials: `StandardMaterial` lighting with the surface worked out per pixel from noise laid out in metres, so there are no textures and the generated meshes need no UVs. Each `Mat` maps to a `SurfaceKind` (`kind_of`); glowing things and rubber stay plain.

| kind | what it does |
|---|---|
| concrete | pour-to-pour clouds, stains, pores, and a power-floated floor that is burnished in patches, so it takes a dull reflection of the lamps |
| paint | tone drift, grime running down vertical faces, dust on anything facing up, a few chips to primer and rust |
| galvanised | zinc spangle in colour and roughness |
| stainless | brushed streaks, which stretch the lamp highlights on the tanks |
| sheeting | sheet-to-sheet shade, dirt runs |
| plate | mill scale, scuffed bright in drag marks (the strong floor) |
| timber | veneer figure |
| foam | fine cells and the drag lines of the hot wire on cut faces |
| table top | a worn dark mat with fine scratches |
| glass | reflection grows towards glancing angles, a film of dust |
| water | moving wave trains and ripple in the normal, reflection grows towards glancing angles |

Detail fades out before it gets smaller than a pixel (`resolved` in the shader), so nothing shimmers at a distance. Things that are real-size are patterned in real metres wherever they stand, including the table's frame; foam, the table top and the water in the basin are part of the model and patterned in its metres.

`LabWater { amplitude, wavelength, speed, heading }` sets the waves on the table's water (world units). It is the hook for the wave and flood rigs: today it only moves the shading, the water surface itself is still a flat quad.

`Look::surface_detail` turns all of it off (the Low preset does), leaving the same materials plain.

### Quality presets

`LabSettings::set_quality(LabQuality::..)` picks a `Look`; F4 steps through them at run time (`LabSettings::quality_key`, `None` to take the key back). Every knob is also a field of `LabSettings::look`.

| | Low | Medium | High (default) | Ultra |
|---|---|---|---|---|
| anti-aliasing | FXAA | SMAA | SMAA | TAA + sharpening |
| ambient occlusion | off | medium | high | ultra |
| contact shadows | off | off | on | on |
| depth of field | gaussian | gaussian | bokeh | bokeh |
| shadow maps | 1024, 2 cascades | 2048, 3 | 2048, 3 | 4096, 4 |
| lamps lighting / shadowed | 6 / 0 | 12 / 0 | 20 / 4 | 20 / 8 |
| reflection map | 32 | 64 | 128 | 256 |
| procedural surfaces | off | on | on | on |

TAA is only on Ultra on purpose: the member shader does not write motion vectors for its own bending, so a member that moves fast can leave a faint trail. SMAA has nothing temporal in it.

## Cost

Measured headless (`cargo test` builds and counts; frame times on the container's software rasteriser are not meaningful for a GPU):

| | |
|---|---|
| hall and dressing | 66 k triangles in 29 meshes (one per material), generated once at startup |
| table (ravine) | 9 k triangles in 8 meshes, rebuilt only when the table changes |
| per-frame CPU | none for the hall; the lens system only copies the focus distance |
| lights (High) | 1 directional light with 3 cascades, 20 spot lights of which 4 are shadowed |
| passes (High) | depth and normal prepass, ambient occlusion, SMAA, bloom, bokeh depth of field, one post pass for the vignette |

None of this has been timed on a GPU. The presets exist so that it can be: open the read-out (F3), press F4 and compare. `cargo run --release -p breakit-lab --example lab -- --bench` prints frame times for all four. What each step up adds, most expensive first as far as can be judged without hardware: the bokeh depth of field, ambient occlusion (it also brings the depth and normal prepass, which draws the scene once more), the shadowed lamps (each draws the hall's 66 k triangles into its own shadow map), then the unshadowed lamps and contact shadows.

In the game with the starter bridge, the read-out showed physics at about 1 ms and the member visuals at 0.05 to 0.08 ms of CPU per frame.

The game's frame-time read-out (top right, F3 toggles) shows the average and worst frame over two seconds of real time, the CPU time of physics and of the member visuals, the lab's triangle and mesh counts and the quality preset. `BREAKIT_PERF_LOG=1` prints it once a second.

## Running it

- `cargo run --release -p breakit-game`: the game in the lab. T table, F3 read-out, F4 quality.
- `cargo run --release -p breakit-lab --example lab`: the lab alone with a stand-in truss. 1-4 table, V next view, F4 quality, B depth of field mode, [ and ] f-number. `-- --quality low|medium|high|ultra` starts on a preset. `-- --capture <dir>` saves every view and table and exits (`--views play,hall` for some of them); `-- --bench` prints frame times for each preset.
- `BREAKIT_CAPTURE=<dir>` on the game also saves `lab.png`, a pulled-back view.

## Open points

- The load rigs and the rig park are in `rigs/` and have their own page, `docs/rigs.md`. The shake platform is built into `TableTop::platform` and spawned as `ShakePlatform` parts, which the shake rig moves.
- Looks were tuned on a software rasteriser; exposure, lamp intensity and the f-number want a pass on a real GPU.
- The reflection map is taken from the middle of the table and has no parallax: reflections are right in direction, not in position. Good enough for curved steel and rippled water; a large flat mirror would give it away.
- Ambient occlusion can speckle faintly on far walls when the camera is focused close and the background is sharp (depth of field off). With depth of field on, the blur hides it.
- No LOD or culling groups for the hall: it is merged per material, so it is all drawn or none. Fine at 60 k triangles.
- The lab's own water is a flat quad that moves in its shading only. A flume's water is simulated and drawn by `breakit-water-render` instead (`docs/water.md`): `PlayTable::water()` describes the basin, and `PlayTable::without_still_water()` leaves the quad out.
- The grid on the table is still geometry, which aliases at a distance; drawing it in the table-top shader would fix that.
- Hero props (crane, actuators, model trucks) are still procedural stand-ins; they are to be replaced by glTF models.
