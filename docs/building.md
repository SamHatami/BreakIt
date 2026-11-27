# Building in build mode

How the player places nodes, beams and supports in 3D, and what the game
shows them while they do. Code: `crates/breakit-game/src/build.rs` (tools,
clicks, actions), `build/aim.rs` (work plane and axis lock, plain maths with
tests), `build/aids.rs` (previews, selection, hints), `build/demo.rs`
(scripted walk-through).

## Where a click lands

Every frame `update_cursor` works out an `Aim`, in this order:

1. **A node** within 14 px of the cursor on screen, wherever it is in 3D.
   Connecting to existing nodes is how beams cross between planes.
2. **A foundation** (sheet anchor) with no node on it yet. A node placed
   there takes the foundation's support.
3. **Axis lock**, with Shift held while a chain is open: the point on the
   x, y or z line through the chain's last node that lands closest to the
   cursor, snapped to the grid. Moving the mouse up gives a column, sideways
   a beam straight out of the plane.
4. **A member** within 8 px (only with no chain open), to select or delete.
5. **The work plane**: the grid point where the cursor ray meets it.

## The work plane

- It passes through the chain's last node, so a beam drawn from a node
  stays in that node's plane, and stays there after the chain ends.
- **Auto** (default): it faces the camera. Looking from the front gives the
  front plane (XY), from the side the side plane (YZ), from steeply above
  the floor plane (XZ). There is some hysteresis so it doesn't flicker
  while orbiting. To build in 3D, orbit (right-drag) and keep clicking.
- **Tab** cycles auto, floor, front, side. **Q / E** step the plane by one
  grid cell along its normal.
- A 8 m patch of grid is drawn around the cursor; a locked plane gets an
  outline.

## Mouse and keys

- Left click acts by the tool. Draw: click a node or empty space to start,
  each further click adds a beam (chain); click a member to select it.
- **Right click** (press and release without dragging) always cancels: it
  ends the chain, else drops the selected member, else returns to Draw.
  Right-drag still orbits. Esc does the same as before (end chain,
  deselect, pause sheet).
- Support tool: click a node to cycle none, pinned, fixed. On sheets with
  foundations, supports only go on them (the scenario rejects others), so
  other nodes are refused with a red hint. Clicking a free foundation puts
  a node there.

## What the player sees

- **Next step** under the scene (`aids::guide_text`): changes with the tool,
  the chain and the selection, e.g. "Start at a yellow foundation" on an
  empty sheet.
- **Cursor hint** (`aids::cursor_hint`): a small box by the cursor saying
  what a click does here (beam length, axis, coordinates, support type),
  red when the click won't work (outside the build area, no foundation).
- **On-top gizmos** (`OnTop`, `OnTopThin` gizmo groups with a depth bias of
  -1, so members never hide them): the selected member's outline in the
  accent colour, the hovered member's in blue (red with Delete), rings on
  the chain node and the hovered node, the preview beam, the locked axis,
  a drop line to the table showing height, a ghost of the next support,
  and the build area's box when the cursor leaves it. Free foundations
  pulse until the first beam.
- Members: the selected one always glows (`RenderSettings::highlighted`),
  plus the hovered one or those at a hovered node. Nodes: the chain node is
  drawn larger in the accent colour, the hovered one in blue.

## Scripted walk-through

`BREAKIT_BUILD_CAPTURE=<dir>` opens sheet Q-01 and builds a braced tower
frame with simulated mouse input (glides, clicks, Shift, right clicks),
saving a screenshot at each step; `BREAKIT_BUILD_CLIP=1` also saves every
other frame to `<dir>/clip/`. Use a release build under `xvfb-run`.
