# Game UI and flow

The concept canvas (wireframes, high-fidelity screens, flows, input map,
stress colours): https://claude.ai/artifact/Mr17fNnWSXoMxXciR6Yy65

The look is not settled. Everything visual goes through one theme layer, so
the structure below can stay while the look changes.

## Loop

`Screen` in `flow.rs` is the one state machine:

    Register -> Brief -> Build -> Test -> Report -> (Revise -> Build | Next sheet -> Brief | Register)

`Mode` (Build or Simulate) is computed from `Screen`: Test and Report run
the dynamic solver, the rest show the static solve.

- **Register**: the test sheets. A sheet unlocks when the one before it
  has a passed revision.
- **Brief**: site drawing, load cases, criteria. F1 shows it again as an
  overlay in build.
- **Build**: the sheet's criteria checked live against the static solve,
  cost meter, worst member, tool rail, member inspector, parts drawer, and
  the RUN TEST button. Anchors and load points from the sheet are locked.
- **Test**: load cases run in order (`scenario.rs` applies them). The chart
  recorder plots worst-member utilisation over time. The first break drops
  to 1/4 speed for 2.5 s; the test ends 3 s after it, or when the cases
  finish.
- **Report**: pass or fail stamp, each criterion measured against
  required, the failure sequence, one sentence on why, and the revision
  list. Failed members stay pinned in build after Revise.

`scenario.rs` holds the sheets (B-01 to B-03 and a sandbox) until a
`breakit-scenario` crate takes over.

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

A new look is a new `UiTheme` preset: colours per swatch, text sizes per
role, corner radius. Four exist to compare directions: `drafting` (paper
and ink), `slate` (dark, rounded, cyan), `blueprint` (cyanotype, white
linework) and `hangar` (black plates, heavy borders, hazard yellow). F10
cycles them; `BREAKIT_THEME=<n>` starts on preset n.

The stress ramp in the UI (blue, amber, orange, red at 50/80/100%) is the
colour-blind-safe one from the canvas. The member shader in
`breakit-render` still uses its own ramp, so the two differ for now.

## Headless check

`BREAKIT_CAPTURE=<dir>` walks the whole loop on sheet B-02 and saves
`1-register.png` to `5-report.png`. `BREAKIT_TRUSS=weak` lays a bare
timber plank instead of a truss, to see the failure path.
