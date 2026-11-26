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

All 17 solver tests pass (7 static, 10 dynamic). Material models and their 15 tests: `docs/materials.md`.

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

Per substep, the numeric Cholesky is ~1.5 ms and assembly ~0.5 ms. 8 substeps with factor reuse fails the 1.2 Pcr buckling test (NaN), so 16 substeps stays the default. The static solve is now sparse too: 1.9 s to 19 ms at this size.

## Next

1. Headroom below 16 ms/frame (solver-speed thread): parallel assembly, adaptive refactorisation, fewer substeps.
2. Done: sparse static solve; axial-bending interaction Mp,N = Mp (1 - n²). Open: biaxial interaction, plastic torsion.
