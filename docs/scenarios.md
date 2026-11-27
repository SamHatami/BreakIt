# Scenarios: limits and scoring

Crate: `crates/breakit-scenario` (engine-free, depends only on `breakit-core`). Scenario files: `crates/breakit-scenario/scenarios/*.ron`, listed in `BUILTIN` in `src/lib.rs`.

## Rules (agreed with Sam, 2026-11-26)

Hard limits stop runaway builds; a scored evaluation rewards building "just big enough".

**Hard limits** (`check_limits`, each a plain sentence via `Display`; any one blocks the test):

- Build volume: every node inside the scenario's box.
- Foundations: supports only at the scenario's anchors. A `Pinned` anchor refuses a support that fixes rotation.
- Budget cap and mass cap.
- Allowed materials.
- What the loads need: a deck with no gap over `max_gap` (wheels fall through) and a node near each payload.
- Self weight is not a rule: the real solver collapses an oversized structure on its own.

**Sandbox** (`PlayMode::Sandbox`) drops volume, anchors, budget, mass and materials; keeps the deck and payload checks; applies the player's own loads; the report is unranked.

**Pass or fail**: survive `duration` seconds of the load timeline. Collapse is a watched node (deck and payload nodes, or every free node) moving more than `collapse.displacement`, or more than `collapse.max_broken_fraction` of members breaking.

**Score** (pass only, 0 to 1000, weights per scenario):

| Part | Full marks | Falls off |
| --- | --- | --- |
| Cost | at or under `target_cost` | linearly to 0.25 at the budget |
| Weight | at or under `target_mass` | linearly to 0.25 at the mass cap |
| Safety margin = 1 / peak utilisation | inside `margin_band` (1.25 to 2.0 in both scenarios) | below: to 0 at a margin of 1; above: `hi / margin` (overbuilt) |
| Objectives | all met | fraction met |

Stars: 1 for a pass, 2 and 3 at score fractions `stars` (0.6, 0.85).

**Objectives** (bonus): `DeckDeflection` (span/300), `PayloadDrift`, `NoBreaks`, `GracefulFailure` (seconds from first damage to collapse in the proof test), `Reserve` (proof test load factor).

**Proof test** (`overload`): after a pass, scenario loads scale up by `rate` per second (traffic replaced by a uniform deck load) until collapse or `max_factor`. Gives the reserve factor and the graceful-failure warning time; it never turns a pass into a fail.

## Loads

The load physics is `breakit_core::loads` (see `docs/loads.md`); `breakit-scenario` only maps scenario events onto it. `Binding::load_set(scenario, settle)` builds a core `LoadSet` whose windows are in solver time (`settle` seconds ahead of test time), and `TestRun` applies it every frame, then adds what the core has no model for.

- `Convoy`: a core `Traffic` case. Each vehicle has sprung (quarter-car) axles driving along the deck lane, so trucks bounce on a flexible deck. The lane is `deck_lane(deck)`: a core `LaneLine::strip` that captures nodes within `width` sideways and `height` vertically of the lane line and shares each station's load between its nodes (two trusses), so scenario files never name node ids.
- `GroundShake`: a core `Quake` with harmonic ground motion, a(t) = peak sin(2 pi f t) with fades; every free node feels -m a. `TestRun::table_offset` gives the table displacement for drawing, `Binding::ground_accel` the acceleration the equipment model uses.
- `Wind`: a core `Wind` case (fan wall jet, Kaimal gusts, shielding, vortex shedding) on every member. Payloads with a `size` also catch it: `½ ρ C_d A |v| v` on the box face (`drag`, default 1.2; 1.8 for a sign board) with the core gusting velocity at the item's centre, applied at `height` above the feet with its overturning couple, so bolts can tear and resting items tip. The fan wall parks upstream along `-direction`.
- `Water`: a core `Water` case in the flume: bed, still level (filled over `ramp`), current and a `Surge` (solitary wave) or `Regular` (Airy) waves from the paddle at `paddle` (default -13.6 m, the lab flume's upstream wall). Morison drag and inertia plus buoyancy on every submerged member; payloads feel no water.
- `DeckLoad` (uniform N/m along the lane), the proof test's deck stand-in and sandbox player loads stay in `Binding::deck_forces`: the core has no uniform deck load.
- Proof test: the traffic cases are switched off (`factor` 0) and the deck stand-in takes over; the `LoadSet` factor follows the proof ramp.
- Floats: any sealed tube in a `Water` case displaces water (core `Water`, see `docs/water.md`). The build-mode static check sees still water as a spring bed: `Binding::static_support` gives the buoyant force at the built position and a vertical water-plane spring per node in the surface (core `Water::static_support`), plus a soft horizontal spring (`FLOAT_DRIFT_STIFFNESS`) so a raft held only by floats still solves; the game passes them to `solve_static_with_springs`. Start the water case before the clock (`start: -settle`, `ramp: 0`) when the structure floats, so the water is there while its weight comes on.
- `Binding::forces_at(scenario, t)` is a static preview (axle loads by the lever rule, no bounce or shaking) for build mode's static check and the brief. The test itself runs the core models.
- `TestRun::rig_state()` and `vehicles()` give the rigs (trucks, table) for drawing; before the test, `Binding::load_set` tells the lab which rigs to roll in.
- Payloads are point masses on the nearest node (`DynamicSim::add_point_mass`), so their weight comes from gravity and they have inertia when shaken.
- Self weight is ramped in over `settle` seconds before the clock starts (sudden gravity overshoots to twice its static effect).

## Shipped scenarios (tuned with `cargo run --release -p breakit-scenario --example probe`)

| Sheet | Scenario | Sample design | Result |
| --- | --- | --- | --- |
| B-01 | Culvert road bridge, 6 m, two 7.5 t dump trucks then an 18 t coil hauler (axles as on the truck models, 2.7 m road) | Warren truss, steel tube 50x5 (the game's starter bridge) | pass, 1 star, 588 (overbuilt) |
| | | 30x3 tube | pass, 3 stars, 917 |
| | | 25x2.5 tube | collapses at 8.8 s (the coil hauler) |
| | | 20x2 tube | collapses at 8.0 s (coil hauler) |
| P-01 | Pump platform on the shake table: 2.5 t pump skid resting on friction 0.35 (critical), 400 kg cabinet bolted, 0.3 g at 2.5 Hz | Braced platform, 40x4 tube | pass, 3 stars, 871: pump sag 13.6 mm, tilt 0.1° |
| | | Unbraced (sway) platform, 60x5 tube | the deck amplifies the shaking, the pump slides off at 2.1 s: fail |
| Q-01 | Water tower on the shake table: 4 t tank at 6 m, 0.4 g at 1.5 Hz | 2-level braced mast, 25x2.5 tube | pass, 2 stars, 782 |

| W-01 | Billboard in the fan wall: 6 x 3 m board (1.2 t, bolted, C_d 1.8) 8 m up, 40 m/s with 15 % gusts along +x | 4-leg tower, 2 storeys, 30x3 tube | pass, 3 stars, 982, util 0.54 |
| | | 35x3.5 tube | pass, 2 stars, 839 |
| | | 40x4 tube | pass, 2 stars, 714 (heavy) |
| | | 25x2.5 tube | survives yielding (util 1.06), 2 stars, 750 |
| | | 20x2 tube | blows over at 3.5 s |
| F-01 | Pump jetty in the flume: 3 t pump house resting (friction 0.35) on a deck 1 m up, legs from the bed at -3.2 m, 2.6 m deep water, 1.5 m/s current, 1.8 m surge whose crest reaches the deck | X-braced legs, 30x3 tube | pass, 3 stars, 916, util 0.70 |
| | | X-braced, 40x4 | pass, 2 stars, 739 |
| | | X-braced, 25x2.5 | yields (util 1.19), pump drifts 89 mm, 1 star |
| | | Unbraced legs, 60x5 | pass, 2 stars, 799, pump drifts 74 mm |
| F-02 | Floating footbridge: 6.3 m path across the flume, moored only by two pins on the near bank (the far bank takes nothing), 2 kN/m crowd, 0.6 m/s current, 0.4 m waves along the flume | Two large pontoons (813x16, 4.8 m) at the middle and far end, frame 30x3 | pass, 3 stars, sag 248 mm |
| | | Same pontoons at stations 2 and 5 | holds, path sinks 563 mm at the free end: 2 stars |
| | | Three small pontoons (406x8) | not enough lift, sinks at 2.7 s |
| | | No pontoons | swings down about the pins at once |

Q-01 is easy for a braced tower; the budget is what bites. Make it harder with a heavier tank, a lower frequency nearer the tower's own, or fewer footings once the part catalog has thinner sections.

## In the game

The game's screens play these scenarios: see `docs/game-ui.md`. Build shows the blocking problems and RUN TEST only starts when there are none; the test screen steps a `TestRun` and the report screen shows its `Report`. Anchor capacity (pull-out, sliding) waits for foundation components.

### Briefs and the site (after Sam's first playtest, 2026-11-27)

Sam could not tell what to build from the briefs ("1.6 m something", coordinates, "flume"), did not know what the yellow orbs were, and found no road for the trucks. So:

- A brief is one or two plain sentences of story; `steps` lists what to build, in the order a player does it, in words that point at what is drawn ("the dashed road line", "the orange foot marks", "the foundations"). No coordinates and no lab jargon: say "wave channel", not "flume". The brief screen numbers the steps.
- `breakit-game/src/site.rs` draws the site as things, with a short caption on each while building:
  - foundations: a concrete block per anchor, an orange pin across it for a pin, an orange plate for a fixed one ("3 foundations (pins): support here");
  - the road (or footpath, when no trucks drive): approach roads on the banks, and a road surface laid on the deck nodes the player builds, following them in the test and dropping out where the members under it break. While building, the lane's strip shows where nodes count as road nodes, with red stretches where they are more than `max_gap` apart;
  - equipment: each payload as a box of its `size` on its feet (see-through while building, solid in the test, falling when the run loses it). Its feet are the sheet's locked nodes, one per foot, so the player sees exactly what to carry.
- `guides.rs` keeps the build-area box and the vehicles' outlines.
- Each register tab sits on a table that has its foundations: shaking sheets (Towers, Equipment) on the shake table, wind sheets on open ground in front of the fan wall, bridges on the ravine. A test checks every foundation stands on solid table.
- The violation messages name what is drawn ("the red stretch of the road", "its orange foot marks").
