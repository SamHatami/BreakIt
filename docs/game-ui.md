# Game UI and flow

The concept canvas (wireframes, high-fidelity screens, flows, input map,
stress colours): https://claude.ai/artifact/Mr17fNnWSXoMxXciR6Yy65

The look is "Hairline", picked by Sam from three mock-ups
(`/mnt/project-files/ui-directions/`): no panels, type straight on the
scene over soft top and bottom gradients, light weights for values, one
warm accent (active tool, primary action, playhead). The 3D lab stays the
hero. Everything visual still goes through one theme layer.

## Loop

`Screen` in `flow.rs` is the one state machine:

    Register -> Brief -> Build -> Test -> Report -> (Revise -> Build | Next sheet -> Brief | Register)

`Mode` (Build or Simulate) is computed from `Screen`: Test and Report run
the dynamic solver, the rest show the static solve.

- **Register**: the test sheets, one tab per group (Bridges, Towers) and a
  sandbox. The first sheet of each group is open; the next unlocks when
  the one before it has a passed revision.
- **Brief**: the scenario's brief, its load timeline, the hard limits
  and what scores full marks. F1 shows it again as an overlay in build.
- **Build**: what blocks the test (`check_limits`, in its own sentences),
  cost and mass against the caps, the static check under the sheet's
  heaviest moment, cost meter, worst member, tool rail, member inspector,
  parts drawer and the RUN TEST button, which only runs when nothing
  blocks. A node placed on an anchor gets that foundation's support;
  payload points are locked nodes.
- **Test**: a `breakit_scenario::TestRun` applies the loads and decides
  the outcome, then runs the proof test after a pass. The chart recorder
  plots worst-member utilisation with a band per event. The first break
  drops to 1/4 speed for 2.5 s. The report opens once the outcome has
  played out.
- **Report**: pass or fail stamp, grade (stars, score, safety margin),
  the proof test's reserve and warning time, the scenario report's
  criterion rows, the failure sequence (or after a pass, what the proof
  test broke first), one sentence on why, and the revision list. Those
  members stay pinned in build after Revise.

Scenarios, limits, loads and grading come from the engine-free
`breakit-scenario` crate (`docs/scenarios.md`). `scenario.rs` here is the
game's view of them: register tabs, anchors, load-case bands, the live
check (`ScenarioCheck`) and the static preview loads. Cost everywhere is
the scenario `CostModel`'s bill.

## Controls

All input goes through `UiAction` messages: keys (`flow::keyboard_actions`)
and clicks (`ui/pointer.rs`) write the same actions.

| Input | Action |
| --- | --- |
| RMB drag / MMB or Shift+RMB / wheel | orbit / pan / zoom |
| F | frame selection, or everything |
| LMB | use the tool: place, chain, select |
| B S X L | draw, support, delete, load (sandbox only) |
| 1 2 | steel, timber (also changes the selected beam) |
| Del | delete selected beam or hovered node |
| Ctrl+Z / Ctrl+Y | undo / redo (whole-design snapshots, `history.rs`) |
| Tab, Q / E | drawing plane, move plane down / up |
| D | exaggerate deflection |
| C | clear to the sheet's start |
| T | next play table (until sheets choose their own) |
| F3 | frame-time read-out (hidden by default) |
| F6 | water on the CPU / on the GPU (`docs/water.md`) |
| Space | run test; in a test, pause |
| , . | test slower / faster |
| H | stress heatmap during a test |
| F1 / Esc | brief / pause or back |
| ? | controls card |
| F10 | next theme preset |

## Theme layer

`ui/theme.rs`. Widgets never name a colour or size. They carry a `Skin`
(background and border as `Swatch` roles) and text carries an `Ink`
(`TextRole` and `Swatch`). `apply_theme` resolves those against the active
`UiTheme` whenever the theme or the node changes.

A look is a `UiTheme` preset: colours per swatch, text size and weight per
role, corner radius, the edge-gradient colour (`EdgeScrim`) and a text
shadow for type over the scene. `hairline` is the default; `slate` puts the
same layout on opaque dark cards and is there to keep screens from
depending on one look. F10 cycles them; `BREAKIT_THEME=<n>` starts on
preset n.

The font is Inter Tight (SIL Open Font License, `crates/breakit-game/fonts/`),
a variable face embedded in the binary, so every weight comes from one
file. Labels and key hints are tracked capitals (`widgets::caps`).

The stress ramp in the UI (blue, amber, orange, red at 50/80/100%) is the
colour-blind-safe one from the canvas. The member shader in
`breakit-render` still uses its own ramp, so the two differ for now.

## Headless check

`BREAKIT_CAPTURE=<dir>` walks the whole loop on sheet B-01 and saves
`1-register.png` to `6-lab.png`. `BREAKIT_TRUSS=weak` lays a bare
timber plank instead of a truss, to see the failure path.
