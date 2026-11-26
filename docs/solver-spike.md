# Solver spike: status and findings (2026-11-26)

## Results so far

- Static FEM: 7 textbook tests pass exactly (cantilever PL^3/3EI both planes, simply supported PL^3/48EI, axial PL/EA, torsion TL/GJ, instability detection, utilisation = 1 at plastic moment).
- Plain XPBD (Gauss-Seidel over scalar constraints) has a correct formulation (converges to the static answer, 0.2% at 200 substeps x 10 sweeps) but is far too soft at real-time budgets for steel: 40 substeps x 1 sweep gives 35x the static deflection on a 10-element cantilever, and the accurate budget costs ~11 ms/frame for only 10 members.
- Direct solver (same constraints, one sparse Cholesky Gauss-Newton step per substep, faer): 8 substeps x 1 iteration lands within 0.02% of the static solve at ~0.2 ms/frame for 10 members. This is now the default (`Solver::Direct`).

Probe: 2 m steel tube cantilever (r 50 mm, t 5 mm), 10 elements, 1 kN tip load, static tip deflection 7.522 mm.

| Solver | Substeps x iters | Tip deflection | Error |
| --- | --- | --- | --- |
| XPBD GS | 40 x 1 | 260 mm | +3360% |
| XPBD GS | 300 x 3 | 7.81 mm | +3.8% |
| XPBD GS | 200 x 10 | 7.536 mm | +0.2% |
| Direct | 8 x 1 | 7.521 mm | -0.02% |
| Direct | 4 x 1 | 7.443 mm | -1.1% |

## Open (3 of 8 dynamics tests failing)

1. `plastic_hinge_forms_at_collapse_load_and_then_snaps`: at 80% of the collapse load a break event still fires; likely the dynamic overshoot of a suddenly applied load (up to 2x static) or the linear interaction check. Ramp the load in tests, then re-check.
2. `sideways_load_on_portal_frame_matches_static_solve`: mismatch on a frame with vertical columns and out-of-plane load; check torsion constraint sign/gradient and the vertical-member local axes.
3. `column_buckles_above_euler_load_only`: thresholds not met; Gauss-Newton has no geometric stiffness, check whether the column buckles over the 4 s window and tune.

Next after these: performance benchmark at ~2,000 members, then the Bevy prototype crate.
