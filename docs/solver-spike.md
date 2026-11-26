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

## Fixed since the first push

All 17 tests pass (7 static, 10 dynamic).

- Damping was applied to the external-force velocity increment, making every settled deflection about 1.7% too small. Damping now applies before forces are added.
- Suddenly applied loads overshoot to about 2x their static effect, which made the "safe" plastic and buckling cases fail. Tests now ramp loads (`ramp_load`); the game should ramp scenario loads too.
- Portal frame test used a 20 kN column load, so the real P-delta effect in the dynamic solver made it differ from the linear static solve by ~1%. Vertical load reduced to 2 kN for that comparison.
- Compression stability: the Gauss-Newton Hessian has no geometric stiffness. At 8 substeps x 1 iteration a column at 50% of its Euler load oscillates chaotically; at 16 x 1 it matches the theoretical P-delta amplification 1/(1 - P/Pcr) within 1% (4.97 vs 4.93 mm), and above Pcr it buckles. Default is now 16 substeps x 1 iteration.

## Robustness fix: start Gauss-Newton from the force-free coast position

A large load on a light node (short members) used to throw the predicted position x̃ a whole member length in one substep, too far for one Gauss-Newton step: a 0.5 m cantilever of 62 mm elements under 25.6 kN ended 4 m *above* its support. The direct solver now starts each substep from x + h·v and puts the forces into the linear solve (x̃ still holds them in the objective). Effects:

- Short stiff members are exact under sudden loads (new regression test).
- The column at 0.5 Pcr is stable even at 8 substeps.
- A factorisation can now be reused for several substeps without diverging.

## Performance (`cargo run --release --example bench -- 120`)

3D truss bridge, 484 nodes, 1,565 steel members, 16 substeps x 1 iteration, single thread:

| Change | ms per 60 Hz frame |
| --- | --- |
| Rebuild CSC from triplets every substep | 152 |
| Reuse sparsity pattern, scatter values only | 44 |
| After the coast-start fix: refactorise every 4 substeps (new default) | 13.4 |
| Refactorise every 8 substeps (all tests pass) | 9.5 |

Per substep, the numeric Cholesky is ~1.5 ms and assembly ~0.5 ms. 8 substeps with factor reuse failed the 1.2 Pcr buckling test (NaN) at this point; see the next section for the cause and the fix. The static solve is now sparse too: 1.9 s to 19 ms at this size.

## Fewer substeps: 16 to 4 per frame (default now 4)

Three separate things depended on the substep count. None of them was the missing compressive geometric stiffness, the first-order quaternion update or the single Gauss-Newton iteration: with a fresh factorisation every substep, the true curvature along the step over the curvature the matrix assumed (ρ) stayed between 0.974 and 1.0 through the whole 1.2 Pcr fold-over at 8 substeps.

1. **Reused factorisation while members rotate.** The NaN at 1.2 Pcr came only from `refactor_every` > 1. A stale matrix holds the axial stiffness EA/L along the old member axis; after the member has turned by θ the new axial direction sees only m/h² plus bending plus `EA/L·cos²θ`, so the step overshoots once `EA/L·sin²θ` exceeds roughly `m/h²` plus the bending stiffness. Fewer substeps shrink m/h² fourfold per halving and let the member turn further between factorisations. Measured: at 8 substeps ρ first passed 2 (the objective went up) with the matrix 3 substeps old and a chord rotation of 0.064 rad; at 16 substeps the largest rotation in a stale window was 0.041 rad and ρ never passed 1.16. Fix (`step_check`): every step must pass an Armijo test on the substep objective. A failing step from a reused factorisation is redone with a fresh one; a failing fresh step is halved and followed by one more iteration.
2. **Plastic flow was one return per substep.** The solve was elastic and the overstress was returned to the yield surface afterwards, so a tie at 1.1 Ny ruptured after 0.23 / 0.93 / 1.85 / 3.70 / 7.40 s at 64 / 16 / 8 / 4 / 2 substeps. Fix (`implicit_plasticity`, `plastic_relaxation`): the plastic return runs inside the solve with its tangent stiffness in the matrix, as a viscoplastic law with relaxation time τ = 1/960 s, which is the rate the old scheme had at 16 substeps. Rupture is now at 0.924 to 0.950 s from 64 substeps down to 1.
3. **No tension geometric stiffness.** A slender tie in high tension (6 mm tube, 4 m, 0.9 Ny) went NaN even at 16 substeps, because its sideways stiffness is almost all N/L and Gauss-Newton leaves that out. Fix (`tension_stiffness`): add (N/L)(I − e eᵀ) for members in tension. Compression is left out on purpose: the term is negative there, and omitting it only shortens the step by the 0 to 3% measured above.

Existing tests against substeps (the 10 dynamic tests; `column` = `column_buckles_above_euler_load_only`, `tie` = `steel_tie_ruptures_after_yielding`):

| Fixes | 16 | 8 | 4 | 2 | 1 |
| --- | --- | --- | --- | --- | --- |
| none | pass | column NaN | column NaN | column NaN, tie | column NaN, tie |
| refactorise every substep | pass | pass | pass | tie | tie |
| step check | pass | pass | pass | tie | tie |
| tension stiffness | pass | pass | pass | tie | tie |
| implicit plasticity | pass | column NaN | column NaN | column NaN | column NaN |
| step check + implicit plasticity | pass | pass | pass | pass | pass |
| all three | pass | pass | pass | pass | pass |

Tension stiffness alone passes the buckling test by luck: the test only asks for more than 0.3 m of sway, and without the step check the column ends up metres to 10^35 m away depending on load and substeps. With the step check the tip ends at the same place (1.9557 m at 1.2 Pcr) for every substep count and every `refactor_every` tried (1, 4, 16).

Bench (`--sizes s,m --static`, minimum of 4 runs; "16 old" has all three fixes off):

| Structure | Config | Elastic ms/frame | ms/substep | Probe vs static | Collapse ms/frame | Breaks | First break s |
| --- | --- | --- | --- | --- | --- | --- | --- |
| tower 3x3x8, 368 members | 16 old | 8.35 | 0.522 | +0.29% | 7.93 | 28 | 1.208 |
| | 16 | 8.69 | 0.543 | +0.29% | 9.07 | 28 | 1.206 |
| | 8 | 4.27 | 0.534 | +0.29% | 4.64 | 29 | 1.208 |
| | 4 | 2.09 | 0.522 | +0.29% | 2.50 | 29 | 1.208 |
| | 2 | 1.06 | 0.530 | +0.29% | 1.39 | 27 | 1.208 |
| | 1 | 0.55 | 0.550 | +0.29% | 0.77 | 27 | 1.217 |
| bridge 20 panels, 488 members | 16 old | 4.96 | 0.310 | +1.73% | 5.04 | 37 | 1.676 |
| | 16 | 5.40 | 0.338 | +1.73% | 6.07 | 40 | 1.677 |
| | 8 | 2.73 | 0.341 | +1.73% | 3.24 | 43 | 1.681 |
| | 4 | 1.37 | 0.343 | +1.73% | 1.75 | 35 | 1.687 |
| | 2 | 0.67 | 0.335 | +1.73% | 1.09 | 28 | 1.692 |
| | 1 | 0.35 | 0.350 | +1.73% | 1.83 | 119 | 1.417 |

The settled displacement does not depend on the substep count (it is where the forces balance, whatever matrix got there). The cost per substep is up 0 to 10%. Break counts in a collapse are chaotic: a one-substep lag in the reduced plastic moment alone moved "16 old" on the bridge from 34 to 37. Within that noise 4 and 2 substeps match 16; 1 substep does not (119 breaks, first break 0.26 s early on the bridge), so 1 is not usable.

Not verified: transient accuracy at 4 substeps beyond these numbers. Implicit Euler damps vibration and rotation more as the substep grows, and that has not been measured against a reference.

## Next

1. Headroom below 16 ms/frame (solver-speed thread): parallel assembly, fewer scheduled refactorisations (`refactor_every` 16 passes every test now that the step check catches the bad substeps).
2. Done: sparse static solve; axial-bending interaction Mp,N = Mp (1 - n²). Open: biaxial interaction, plastic torsion.
