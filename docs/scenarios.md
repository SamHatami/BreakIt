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
- `DeckLoad` (uniform N/m along the lane), the proof test's deck stand-in and sandbox player loads stay in `Binding::deck_forces`: the core has no uniform deck load.
- Proof test: the traffic cases are switched off (`factor` 0) and the deck stand-in takes over; the `LoadSet` factor follows the proof ramp.
- `Binding::forces_at(scenario, t)` is a static preview (axle loads by the lever rule, no bounce or shaking) for build mode's static check and the brief. The test itself runs the core models.
- `TestRun::rig_state()` and `vehicles()` give the rigs (trucks, table) for drawing; before the test, `Binding::load_set` tells the lab which rigs to roll in.
- Payloads are point masses on the nearest node (`DynamicSim::add_point_mass`), so their weight comes from gravity and they have inertia when shaken.
- Self weight is ramped in over `settle` seconds before the clock starts (sudden gravity overshoots to twice its static effect).

## Shipped scenarios (tuned with `cargo run --release -p breakit-scenario --example probe`)

| Sheet | Scenario | Sample design | Result |
| --- | --- | --- | --- |
| B-01 | Culvert road bridge, 6 m, two 7.5 t trucks then an 18 t tipper | Warren truss, steel tube 50x5 (the game's starter bridge) | pass, 1 star, 588 (overbuilt) |
| | | 30x3 tube | pass, 3 stars, 917 |
| | | 25x2.5 tube | collapses at 9.6 s (the bouncing tipper) |
| | | 20x2 tube | collapses at 8.8 s (tipper) |
| P-01 | Pump platform on the shake table: 2.5 t pump skid resting on friction 0.35 (critical), 400 kg cabinet bolted, 0.3 g at 2.5 Hz | Braced platform, 40x4 tube | pass, 3 stars, 871: pump sag 13.6 mm, tilt 0.1° |
| | | Unbraced (sway) platform, 60x5 tube | the deck amplifies the shaking, the pump slides off at 2.1 s: fail |
| Q-01 | Water tower on the shake table: 4 t tank at 6 m, 0.4 g at 1.5 Hz | 2-level braced mast, 25x2.5 tube | pass, 2 stars, 782 |

Q-01 is easy for a braced tower; the budget is what bites. Make it harder with a heavier tank, a lower frequency nearer the tower's own, or fewer footings once the part catalog has thinner sections.

## In the game

The game's screens play these scenarios: see `docs/game-ui.md`. Build shows the blocking problems and RUN TEST only starts when there are none; the test screen steps a `TestRun` and the report screen shows its `Report`. Build volume, anchors, lane, payloads and vehicles are gizmos (`breakit-game/src/guides.rs`) until the play table draws them. Anchor capacity (pull-out, sliding) waits for foundation components.
