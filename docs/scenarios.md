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

`Binding::forces_at(scenario, t)` is a pure function from test time to one force per node; the game, the headless `evaluate` and any chart recorder see the same loads.

- `Convoy`: vehicles with axle loads driving along the deck lane. Each axle is split between the deck stations either side (lever rule). Deck nodes are found by geometry (within `width` sideways and `height` vertically of the lane line), so scenario files never name node ids; nodes at the same station (two trusses) share the load.
- `DeckLoad`: uniform N/m along the lane, ramped.
- `GroundShake`: a(t) = peak sin(2 pi f t) with fades; every free node feels -m a, using `DynamicSim::node_mass`.
- Payloads are point masses on the nearest node (`DynamicSim::add_point_mass`), so their weight comes from gravity and they have inertia when shaken.
- Self weight is ramped in over `settle` seconds before the clock starts (sudden gravity overshoots to twice its static effect).

The "Simulate wind, quake and wave loads" work is adding richer load models in `breakit-core`; the plan is to replace the `Event` enum with those load cases (they carry name, start, duration and a factor for the proof test).

## Cost

`CostModel`: price per kg per material (steel 2.5, timber 1.2, concrete 0.25, composite 30) plus a price per member end by joint kind (pinned 20, dowelled 30, bolted 40, monolithic 60, welded 80, rigid 80; from the components design). `estimate_with_joints` takes the joint kind per member end; `estimate` prices every end as bolted. Joints dominate small structures at these prices.

## Shipped scenarios (tuned with `cargo run --release -p breakit-scenario --example probe`)

| Sheet | Scenario | Sample design | Result |
| --- | --- | --- | --- |
| B-01 | Culvert road bridge, 6 m, two 7.5 t trucks then an 18 t tipper | Warren truss, steel tube 50x5 (the game's starter bridge) | pass, 1 star, margin 9.7 (overbuilt) |
| | | 30x3 tube | pass, 2 stars, 786, margin 1.65 |
| | | 25x2.5 tube | pass, yields (margin 0.95), sag 118 mm |
| | | 20x2 tube | collapses at 8.7 s (tipper) |
| Q-01 | Water tower on the shake table: 4 t tank at 6 m, 0.4 g at 1.5 Hz | 2-level braced mast, 25x2.5 tube | pass, 2 stars, 783, tank sway 12 mm |

Q-01 is easy for a braced tower; the budget is what bites. Make it harder with a heavier tank, a lower frequency nearer the tower's own, or fewer footings once the part catalog has thinner sections.

## In the game

The game's screens play these scenarios: see `docs/game-ui.md`. Build shows the blocking problems and RUN TEST only starts when there are none; the test screen steps a `TestRun` and the report screen shows its `Report`. Build volume, anchors, lane, payloads and vehicles are gizmos (`breakit-game/src/guides.rs`) until the play table draws them. Anchor capacity (pull-out, sliding) waits for foundation components.
