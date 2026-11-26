# Ground contact (2026-11-26)

`breakit-core/src/contact.rs`. Off by default; turn it on with `SimConfig::ground = Some(Ground { .. })`. The game builds it in `analysis::table_ground()` from the lab table's `PlayTable::contact_boxes()` (read when the simulation starts, so T in build mode changes it), over a floor at the basin bottom (y = -4).

## What touches what

- The ground is a floor plane at `Ground::height` plus any number of static axis-aligned `Block`s (table blocks, banks, abutments, piers).
- Every free node is a sphere of radius half its largest member section (rect: (b + h)/4, tube: outer radius), scaled by `radius_scale`. A node built closer to a surface than that touches at its built position, so beams drawn on the table do not pop up.
- Members are straight between nodes for contact. Member-member contact is not done: collapsing parts can pass through each other.
- Supports are not touched (they have no translation unknowns).

## How it is solved

Contact is part of the direct solver's substep objective, so the Gauss-Newton matrix, the step check and the linear solver all see it, and the gradient is exact for the energy (finite-difference tests in `contact.rs`).

- Normal: one-sided penalty on the penetration d. Stiffness κ = `stiffness` (20) times the node's (mass + load/g)/h². It ramps in linearly over the first `ramp` (2 mm), so the energy is C² and the matrix changes smoothly as nodes land. All additions are diagonal and non-negative on translation unknowns: no sparsity change, both linear backends unaffected.
- Friction: Coulomb (μ = 0.6) as an elastic-plastic tangential spring to an anchor on the surface, holding up to μN after `stick_slip` (1 mm), then the anchor is dragged. N is the previous substep's normal force (lagged); the anchor is fixed within a substep, so the energy is exact for the solve. No creep under static friction.
- Landing: the substep start position is never put deeper than `ramp`, so a fast node lands in one substep instead of starting deep in the stiff region. After the substep, outward velocity that only undoes last substep's penetration is removed, so the penalty does not bounce nodes. Landings are perfectly inelastic.
- Gauss-Seidel solver: position projection with friction (push-out, tangential motion cancelled up to μ times the push).

## Read-outs

`DynamicSim::contact_events()` (node, time, approach speed, mass) for each touch-down faster than 0.05 m/s, for sound and dust. `contact_force(node)` is the ground normal force, `clearance(node)` the gap to the nearest surface.

## Numbers

`cargo run --release --example floor -- --sizes s,m,l` (braced steel tower pushed until it breaks, 600 frames at 60 Hz, 4 substeps, single thread, noisy shared machine):

| Tower | Members | Run | ms/frame mean / p99 / max | Deepest mm | Touch-downs | Breaks |
|---|---|---|---|---|---|---|
| 2x2x4 | 100 | no floor | 0.34 / 0.59 / 1.15 | falls 113 m | 0 | 17 |
| | | floor | 0.28 / 0.64 / 1.03 | 2.9 | 24 | 18 |
| | | standing on floor, no supports | 0.30 / 0.45 / 1.17 | 2.4 | 0 | 0 |
| 3x3x8 | 368 | no floor | 1.01 / 1.64 / 5.16 | falls 113 m | 0 | 29 |
| | | floor | 1.07 / 1.72 / 4.95 | 4.5 | 34 | 29 |
| | | standing | 1.04 / 1.73 / 4.88 | 3.8 | 0 | 0 |
| 5x5x12 | 1,272 | no floor | 4.06 / 6.63 / 19.5 | falls 108 m | 0 | 50 |
| | | floor | 4.13 / 6.02 / 19.0 | 4.1 | 59 | 53 |
| | | standing | 3.58 / 5.74 / 18.2 | 5.6 | 0 | 0 |

Contact costs nothing measurable. A free-standing tower's column bases sink 4 to 6 mm, because the penalty scales with the node's own mass, not the weight it carries. Making κ follow the lagged normal force cut that to 0.5 mm but cost 2 to 4 times the CG iterations, so it is not done.

## Open

- Member-member contact (piles of collapsed members interpenetrate).
- Sloped terrain (the lab's contour banks are boxes here).
- Moving ground (shake table in the world frame); the quake loads run in the table frame, so the static ground is right for them.
- Broken halves drawn by `breakit-mesh` hang rigidly from their node and can still poke into the ground.
