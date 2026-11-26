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

All 15 tests pass (7 static, 8 dynamic).

- Damping was applied to the external-force velocity increment, making every settled deflection about 1.7% too small. Damping now applies before forces are added.
- Suddenly applied loads overshoot to about 2x their static effect, which made the "safe" plastic and buckling cases fail. Tests now ramp loads (`ramp_load`); the game should ramp scenario loads too.
- Portal frame test used a 20 kN column load, so the real P-delta effect in the dynamic solver made it differ from the linear static solve by ~1%. Vertical load reduced to 2 kN for that comparison.
- Compression stability: the Gauss-Newton Hessian has no geometric stiffness. At 8 substeps x 1 iteration a column at 50% of its Euler load oscillates chaotically; at 16 x 1 it matches the theoretical P-delta amplification 1/(1 - P/Pcr) within 1% (4.97 vs 4.93 mm), and above Pcr it buckles. Default is now 16 substeps x 1 iteration.

## Performance (`cargo run --release --example bench -- 120`)

3D truss bridge, 484 nodes, 1,565 steel members, 16 substeps x 1 iteration, single thread:

| Change | ms per 60 Hz frame |
| --- | --- |
| Rebuild CSC from triplets every substep | 152 |
| Reuse sparsity pattern, scatter values only | 44 |
| Also reuse the factorisation for 4 substeps | 18 (but diverges under heavy compression) |

Per substep at 44 ms/frame: numeric Cholesky ~1.5 ms, assembly ~0.5 ms. Factor reuse (`SimConfig::refactor_every`) exists but defaults to 1, because a stale factorisation makes the 0.8 Pcr column test produce NaN even at 2. The static solve is still dense (1.9 s at this size) and must move to the sparse path.

## Next

1. Reach 16 ms/frame at ~1,500 members: parallel assembly, fewer substeps once geometric stiffness (tension part, or a PD-safe approximation) is in the Hessian, or factor reuse made safe (e.g. refactor when the residual grows).
2. Sparse static solve.
3. Axial-bending interaction in the yield check; plastic torsion.
