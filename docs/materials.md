# Material models (2026-11-26)

Every member is still a corotational beam in the solver. Once per substep the
material model (`src/damage.rs`) reads the member's stretch and end rotations
and adjusts its rest shape (axial offset, plastic hinge rotations) and its
stiffness so the internal forces follow the material law. Stiffness only goes
down, so a factorisation the direct solver reuses for a few substeps
over-estimates stiffness and stays stable. Strengths are characteristic
values without safety factors.

| Material | Constructor | Warning before failure | Failure modes (`BreakCause`) |
| --- | --- | --- | --- |
| Steel S355 | `Material::steel()` | Yield, plastic hinges (kinks) | `Rupture`, `Hinge` |
| Timber C24 | `Material::timber()`, `timber_at_grain_angle(θ)` | Fibre cracking near capacity, crushing in compression | `Splinter` (tension, bending), `TimberSplit` (shear along grain), `Crushing` |
| Plain concrete C30/37 | `Material::concrete()` | None in tension | `Brittle` (first crack), `Crushing`, `ConcreteShear` |
| Reinforced concrete C30/37 + B500B | `Material::reinforced_concrete()`, `reinforced_concrete_with(ratio, cover)` | Cracks, stiffness loss, bar yield (hinges), crushing | `Crushing`, `RebarRupture`, `ConcreteShear` |
| CFRP profile (UD) | `Material::cfrp()` | None | `FibreRupture`, delamination then `FibreRupture` |
| CFRP laminate (quasi-isotropic) | `Material::cfrp_laminate()` | Matrix cracking (up to 20 % stiffness loss) | `FibreRupture` |

## Rules

- **Steel**: unchanged. Mp,N = Mp (1 − (N/Ny)²), hinge snap at 0.08 rad total, rupture at 15 % plastic strain.
- **Timber**: Hankinson's formula for grain angle (E, f_t, f_c, f_m). EN 1995 interaction σt/ft + σm/fm ≤ 1 and (σc/fc)² + σm/fm ≤ 1 on the elastic section modulus. Shear τ = 1.5 V/A against f_v. Compression past f_c crushes plastically up to 2 % strain.
- **Concrete** (`src/concrete.rs`, sectional analysis once per member type): bars in the four corners. Cracking moment from the gross section; cracked transformed inertia; Branson's Ie for the softened bending stiffness; rectangular stress block interaction curve M(N) for hinges; hinge capacity (φu − φy) d/2; squash load f_ck (A − As) + Es ε_c2 As with crushing over ε_cu − ε_c2; EN 1992 V_Rc without links. Axial tension after cracking is carried by the bars only (Es As), and bars snap at 2.5 % average plastic strain.
- **Composite**: linear to failure on max fibre stress (Xt tension, Xc compression). Interlaminar shear (1.5 V/A, 2 V/A for tubes) over the ILSS delaminates a bending plane: bending stiffness × 0.25 (rectangles) or × 0.5 (tubes), section modulus halved, no further ILSS check. Laminates lose stiffness linearly from first-ply failure (50 % of ultimate) to ultimate.

## For renderers

`DynamicSim::damage(i)` / `damage_all()` return `MemberDamage`: `kind`, `state` (Intact < Cracked < Yielded < Delaminated < Crushed < Broken, never goes back), `damage` 0..1, `utilization`, `hinge[axis][end]`, `plastic_strain`, `crack[end]`, `tension_crack`, `crush[end]`, `delamination`, `moment[axis][end]`, `axial`, `stiffness`. `BreakEvent.position` is 0 at end a, 1 at end b, 0.5 for failures along the member.

## Validation (`tests/materials.rs`, 15 tests)

- RC section by hand: squash load, cracking moment, cracked neutral axis, M_u = 164.6 kN·m for 300 × 500 mm at 1 % (hand-solved quadratic), V_Rc = 104.5 kN.
- Cracked RC cantilever matches a static solve with Branson-softened elements within 5 %.
- RC cantilever holds at 0.95 M_u, yields then crushes at 1.15 M_u; plain concrete breaks at 1.1 M_cr where RC only cracks.
- RC tie: cracked elongation = N L / (Es As) within 3 %, bars snap above As fy; plain tie breaks at f_ctm A.
- RC column crushes at 1.05 N0 (visible crushing first), holds 0.9 N0. Short RC cantilever without links fails in shear at 1.1 V_Rc.
- Timber: tension tie at 0.8 f_t0 holds along grain, splinters at 15° grain; short deep beam splits in shear at 1.1 f_v A / 1.5; post crushes gradually.
- CFRP: linear (deflection within 2 % of static) and no damage at 60 %, fibre rupture at 110 %; delamination at 1.05 × ILSS capacity survives on a short member and breaks a longer one; laminate matrix cracking lowers stiffness at 80 %.

## Performance

Material update per 60 Hz frame on the 1,565-member bench bridge: steel 1.6 ms, reinforced concrete 2.7 ms, timber 2.2 ms, CFRP 2.1 ms (frames 14 to 17 ms total, single thread). Concrete laws are shared between members with the same material and section, and the interaction curve is sampled at 64 even steps in N for O(1) lookup; per-member tables made concrete 13.5 ms from cache misses.

## Simplifications

Biaxial bending axes yield independently. No links (stirrups) in concrete, no tension stiffening, no confinement. Symmetric corner reinforcement only. Damaged bending stiffness does not recover when cracks close. Timber grain angle is fixed per material. Composite delamination happens once per plane.
