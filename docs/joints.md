# Joints (2026-11-26)

Every member end has a `Joint` (`crates/breakit-core/src/joint.rs`), set with
`Structure::set_ends(m, a, b)`. The default is `Joint::RIGID`, which behaves
exactly like the solver did before joints existed. Design: the "Building
components and control depth" doc, where joints are the main gameplay
choice between cost, stiffness and how a part fails.

## Model

- **Stiffness**: a rotational spring per bending axis in series with the
  member. Member flexibility L/(6EI) [[2, −1], [−1, 2]] plus 1/k on each
  diagonal, inverted (`bending_stiffness`). A pinned end (k = 0) is condensed
  out exactly, so its row and column are exactly zero. Torsion always passes
  through: a pin is a ball joint in bending only, so a pinned-pinned member
  cannot spin about its own axis.
- **Strength**: an end moment capacity per axis. Where it is below the
  member's plastic moment the joint yields instead of the member; that hinge
  rotation is booked to the joint, not to the material model.
- **Failure** (`BreakCause::Connection`, `position` = 0 or 1 for the end):
  resultant of axial and shear force over `force_capacity`, or plastic
  rotation at that end (joint and member hinge alike) over
  `rotation_capacity`.
- `MemberDamage.joint[end]` is the peak connection utilisation 0..1, and the
  static solve reports `MemberForces.joint_utilization`.

The static solve builds each bending block as Bᵀ K B from the same
`bending_stiffness`, so it remains the oracle. Nodes where only pins meet can
be free to rotate about an axis no member resists; the static solve gives
rotational DOFs a 1e-10 relative spring in that case instead of reporting a
mechanism. Translational mechanisms (an unbraced pinned frame) are still
reported as unstable.

## Presets (`JointKind::joint(material, section, length)`)

Stiffness classes follow EN 1993-1-8 5.2.2 (pinned below 0.5 EI/L, rigid
above 25 EI/L). Capacities are game values relative to the member's tension
capacity N and moment capacity M (`member_capacity`), not code resistances.

| Kind | Materials | Stiffness | Moment cap | Rotation cap | Force cap |
| --- | --- | --- | --- | --- | --- |
| Rigid | all | rigid | member | none | none |
| Pinned | all | 0 | – | none | 0.6 N |
| Bolted | steel, timber, CFRP | 4 EI/L | 0.45 M | 0.06 rad | 0.8 N |
| Welded | steel | rigid | member | 0.025 rad | none |
| Monolithic | concrete | rigid | member | none | none |
| Dowelled | timber | 1.5 EI/L | 0.3 M | 0.04 rad | 0.5 N |

## Validation (`tests/joints.rs`, 10 tests)

- Pinned ends turn a clamped beam (PL³/192EI) into a simply supported one
  (PL³/48EI), zero moment at the pins, PL/4 at midspan (static, exact).
- Cantilever on a rotational spring: PL³/3EI + PL²/k (static, exact).
- Dynamic solver within 1 % of the static solve for both.
- Pinned triangle truss: bar forces P/2 and P/√2, no bending.
- Pinned portal frame without a brace is a mechanism in both solvers; with a
  brace the dynamic sway matches the static solve within 2 %.
- Pinned tie: holds at 0.5 Ny, the pin shears at 0.7 Ny before the steel
  yields.
- Bolted cantilever root: holds at 0.4 Mp/L, the joint yields and fails at
  0.55 Mp/L with the member still elastic.
- Welded root at 1.3 Mp/L cracks at the weld before the rigid case's
  plastic hinge snaps; holds at 0.8.

## Not yet

Joints per member end only (no node-level joint object). Joint stiffness does
not degrade as it yields beyond the plastic cap. No torsion release. Joint
meshes and a broken-connection look belong to `breakit-mesh`.
