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

## Linear solver rewrite (`src/linsolve.rs`) and current numbers

`cargo run --release --example bench -- --sizes s,m,l,xl --static` (the bench was replaced: a braced tower frame and a truss bridge, each run elastic and to collapse; the older `-- 120` bridge quoted above no longer exists).

What changed in the linear algebra:

- faer's default global parallelism made these factorisations 2.5 to 4 times slower (thread hand-off costs more than the work at a few thousand unknowns). The solver now calls the low-level faer API with `Par::Seq` and preallocated workspace.
- The sparsity pattern and symbolic factorisation are built once and survive breaks: a broken member contributes zeros.
- An old Cholesky factorisation is only a preconditioner for conjugate gradients on the current matrix, so every solve is exact for the current matrix to `linear_tolerance` (default 1e-6). This replaces "reuse the factorisation as if it were current", which needed the step check to catch divergence. A break is a rank 6 change, which CG absorbs in a few iterations.
- When to refactorise is a cost decision: the factorisation cost in CG iterations is estimated from the fill, and a new one is computed once the old one has wasted that many iterations. When even a one-solve-old factorisation needs that many (violent collapse, many members changing yield state), the solver factorises every solve for a while and then probes again.

Single thread, 60 Hz frames, 4 substeps, shared 4-core cloud machine with other jobs running (timings noisy, roughly ±20%):

| Structure | Members | Unknowns | Elastic ms/frame (mean / p99) | Collapse ms/frame (mean / p99 / max) | Settled probe vs static FEM |
| --- | --- | --- | --- | --- | --- |
| tower 2x2x4 | 100 | 270 | 0.33 / 0.82 | 0.46 / 0.75 / 1.2 | +0.11% |
| bridge 8 panels | 100 | 216 | 0.30 / 0.64 | 0.36 / 0.59 / 1.0 | +0.08% |
| tower 3x3x8 | 368 | 864 | 1.5 / 3.0 | 2.0 / 4.7 / 6.3 | +0.29% |
| bridge 20 panels | 488 | 1,968 | 1.7 / 2.5 | 2.7 / 6.3 / 7.8 | +1.7% |
| tower 5x5x12 | 1,272 | 2,808 | 5.8 / 14 | 9.6 / 17 / 19 | +0.56% |
| bridge 60 panels | 2,172 | 10,152 | 9.4 / 16 | 15 / 33 / 39 | not run |
| tower 8x8x16 | 3,856 | 8,262 | 29 / 69 | 55 / 156 / 211 | not run |
| bridge 120 panels | 5,776 | 28,896 | 25 / 47 | 46 / 114 / 118 | not run |

The gap to static FEM is real geometric nonlinearity (the bridge sags 94 mm under the push), not solver error: it is identical at 16 substeps and with a fresh factorisation every solve.

Same benchmark, tower 5x5x12 and bridge 60 panels, elastic mean ms/frame, by stage:

| Stage | Tower (1,272) | Bridge (2,172) |
| --- | --- | --- |
| First spike commit, 8 substeps, refactor every substep | 251 | 64 |
| Pattern reuse, stale factor for 4 substeps, 16 substeps (785c6c7) | 70 | not measured |
| + sequential faer, fixed pattern, CG on stale factor, 16 substeps | 20 | 35 |
| + 4 substeps (step check, implicit plasticity, tension stiffness) | 5.8 | 9.4 |

Where the time goes now (tower 5x5x12, elastic, per frame): assembly 2.1 ms, solves 2.9 ms, factorisation 0.2 ms, everything else (integration, step-check objective, material update) about 0.6 ms.

## Second linear backend: block-Jacobi CG, picked per structure

A side study of factorisation-free solvers found that conjugate gradients preconditioned by each node's 6x6 block, warm-started from the previous solve, beats sparse Cholesky on 3D frames (whose factors fill in heavily) and loses on chain-like bridges (ill-conditioned along the span, 60 to 160 iterations in a collapse). `linsolve::Backend::Auto` (the default, `SimConfig::linear_backend`) therefore picks by the fill of the factorisation: block Jacobi when a factorisation costs more than 12 triangular-solve pairs in flops, Cholesky otherwise. In the bench that sends the three larger towers to block Jacobi and every bridge to Cholesky. The default `linear_tolerance` is now 1e-4: settled displacements and break counts in the bench are the same as at 1e-6.

Elastic / collapse mean ms per frame (collapse max in brackets), 4 substeps, single thread:

| Structure | Members | Cholesky backend | Block Jacobi backend | Auto picks |
| --- | --- | --- | --- | --- |
| tower 3x3x8 | 368 | 1.5 / 2.2 (7.0) | 1.5 / 1.4 (4.7) | Jacobi |
| tower 5x5x12 | 1,272 | 5.5 / 9.5 (20) | 5.6 / 5.0 (16) | Jacobi |
| tower 8x8x16 | 3,856 | 29 / 55 (211) | 18 / 18 (58) | Jacobi |
| bridge 20 panels | 488 | 1.7 / 2.6 (7.0) | 3.2 / 3.2 (9.7) | Cholesky |
| bridge 60 panels | 2,172 | 9.5 / 13 (39) | 16 / 26 (83) | Cholesky |
| bridge 120 panels | 5,776 | 33 / 44 (149) | 50 / 128 (495) | Cholesky |

Also measured in that study and not adopted: symmetric block Gauss-Seidel preconditioning (half the iterations at twice the cost each), Chebyshev iteration, matrix-free products, and rayon over the sparse product (slower than serial at these sizes on a shared 4-core machine). A separate f32 block-sparse implementation of the same method was about twice as fast per iteration as the f64 one adopted here; that is available headroom.

## Follow-up: 32-bit block Jacobi, leaner assembly, optional threads

- The block-Jacobi iteration now runs in f32 (node blocks are assembled and inverted in f64, then cast). Same iteration counts, displacements and break counts; the solve part is 10 to 25% faster at 3,856 members and unchanged at 1,272, where everything already fits in cache.
- Assembly computes each beam's block and forces first and scatters afterwards. That alone cut assembly by about a quarter (tower 5x5x12: 2.1 to 1.5 ms per frame).
- The same split lets assembly and the step-check objective run on the rayon pool (`SimConfig::parallel_min_members`), with results independent of the thread count. It is off by default: on the 4-core cloud machine used here, 4 threads gained at most 15% on the two largest cases and nothing or a loss below that. Worth re-measuring on a desktop CPU before turning it on. The material update was left serial because other threads are editing it.

Current defaults, single thread, mean ms per frame elastic / collapse (collapse max): tower 1,272 members 4.6 / 4.2 (17); bridge 2,172 members 8.2 / 13 (44); tower 3,856 members 15 / 14 (61); bridge 5,776 members 23 / 41 (129).

## Node-block Cholesky (`src/blockchol.rs`)

faer's scalar sparse Cholesky ran at about 1.5 GFlop/s on the bridges: the factor of a chain-like structure is too sparse for supernodes, and the simplicial code pays an indirection per entry. `Backend::Cholesky` now factorises on 6x6 node blocks instead (the faer path is kept as the hidden `Backend::CholeskyScalar`, bench configs `scalar` and `scalar-1e-4`):

- Every node owns six unknowns; a fixed translation or rotation stays in the numbering with a unit diagonal. The ordering is faer's AMD on the node graph, the symbolic factorisation is by block columns, both built once.
- The numeric factorisation is left-looking over block columns with fixed-size 6x6 kernels; the inverse of each diagonal block's factor is stored, so the triangular solves and the product with the matrix are branch-free and index once per block.
- The kernels are compiled twice, for the baseline instruction set and for AVX2 with FMA, chosen at start-up (three small `unsafe` calls). The two copies differ in the last bits, as faer's kernels do between CPUs.
- The refactorisation policy, the CG loop and the statistics are unchanged and shared by both paths. The cost of a factorisation in CG iterations is now estimated from block counts; it comes out at 3 on the bridges (it was 4), so in a collapse the solver refactorises more often and iterates less.

Kernel times in microseconds (`bench --kernels`, minimum of 3 runs, single thread; a direct solve is a triangular solve pair, a CG iteration adds the product with the matrix and the vector updates):

| Structure | Members | Factor entries (faer / blocks) | Factorisation (faer / blocks) | Direct solve (faer / blocks) | CG iteration (faer / blocks) |
| --- | --- | --- | --- | --- | --- |
| tower 3x3x8 | 368 | 46,356 / 46,356 | 1,422 / 144 | 51 / 13 | 53 / 16 |
| bridge 20 panels | 488 | 35,466 / 36,120 | 456 / 67 | 43 / 13 | 64 / 19 |
| tower 5x5x12 | 1,272 | 385,884 / 286,704 | 3,866 / 2,013 | 225 / 112 | 197 / 132 |
| bridge 60 panels | 2,172 | 173,622 / 174,492 | 2,170 / 321 | 222 / 83 | 367 / 141 |
| tower 8x8x16 | 3,856 | 2,057,940 / 1,567,260 | 22,925 / 26,708 | 1,245 / 927 | 1,359 / 935 |
| bridge 120 panels | 5,776 | 481,218 / 482,304 | 5,883 / 1,016 | 684 / 375 | 1,099 / 492 |

On the bridges the fill is the same and the factorisation is 6 to 7 times faster, the solves 2 to 2.7 times. On tower 8x8x16, whose factor is dense enough for faer's supernodal kernels, the block factorisation is 15% slower and the solves 30% faster; that structure goes to block Jacobi anyway.

Frames, `linear_tolerance` 1e-4, mean ms per frame elastic / collapse (collapse p99 / max), minimum of 3 runs. Settled probe displacements and break counts are identical between the two paths in all six cases.

| Structure | faer scalar | Node blocks |
| --- | --- | --- |
| tower 3x3x8 | 1.15 / 1.49 (4.0 / 5.5) | 0.71 / 0.81 (1.3 / 1.7) |
| bridge 20 panels | 1.40 / 2.09 (5.6 / 6.5) | 0.99 / 1.15 (2.6 / 3.0) |
| tower 5x5x12 | 3.92 / 9.04 (20 / 23) | 3.15 / 4.71 (8.2 / 9.9) |
| bridge 60 panels | 8.95 / 12.2 (28 / 36) | 5.52 / 7.29 (16 / 20) |
| tower 8x8x16 | 19.0 / 35.6 (100 / 106) | 15.5 / 29.1 (74 / 85) |
| bridge 120 panels | 23.3 / 37.4 (95 / 121) | 18.8 / 18.3 (44 / 52) |

`Backend::Auto` still decides by fill, now computed from the block structure, and the threshold moved from 12 to 30: tower 3x3x8 (fill 17) now gets Cholesky, which is 1.4 to 1.8 times faster there than block Jacobi; tower 5x5x12 (43) is a tie (Cholesky 3.2 / 4.7, Jacobi 4.7 / 4.2) and stays on Jacobi for its lower collapse p99; tower 8x8x16 (112) is clearly Jacobi's.

On the bridges assembly is now the largest part of a frame (bridge 60: 3.0 of 5.5 ms elastic). What is left in the solver: the triangular solves on bridge 120 are bound by memory (a 3.9 MB factor), where a single-precision copy of the factor for preconditioning would help; supernodes would be needed to beat faer on dense factors.

## Next

1. Speed: on bridges assembly is now the largest part of a frame (about half), so that is the next thing to cut; the remaining collapse spikes are step-check retries, each paying an assembly and a factorisation. Bridge 120's solves are memory-bound (a single-precision copy of the factor for preconditioning is untried); threads need re-measuring on a desktop CPU. Large 3D grids (tower 8x8x16) are limited by fill in the factorisation; a nested-dissection ordering or a looser `linear_tolerance` (1e-4 gave the same displacements and break counts in the bench) are the next levers there.
2. Done: sparse static solve; axial-bending interaction Mp,N = Mp (1 - n²). Open: biaxial interaction, plastic torsion.
