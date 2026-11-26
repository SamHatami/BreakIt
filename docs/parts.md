# Building parts (2026-11-26)

`crates/breakit-parts` is the engine-free part catalog: what the player can
build with, at the three control layers Sam picked (design doc "Building
components and control depth"). Data in `crates/breakit-parts/data/parts.ron`,
dimensions in mm.

## One description per member

Every layer ends in a `MemberChoice { material, profile, joints }`.
`MemberChoice::resolve(length)` returns the `breakit-core` `Material`,
`Section` and two `Joint`s, plus the `breakit-mesh` `Profile` that is drawn.
Games and saves store the `MemberChoice` (serde), never solver numbers.

| Layer | What the player picks | Source |
| --- | --- | --- |
| Catalog | A `Part` by id ("steel-ibeam-m": IPE 200, S355, bolted) | `Catalog::parts` |
| Engineer | A `Family` (IPE, HEA, CHS, SHS, round bar, sawn timber, glulam, RC, CFRP tube), a size, a grade, a joint per end | `Catalog::families`, `Family::choice` |
| Sandbox | Any `ProfileSpec` and `MaterialSpec` | checked by `Catalog::limits` |

## Materials and grades

`MaterialSpec` maps onto new core constructors: `Material::steel_grade`
(S235, S355, S460), `timber_class` (C16, C24 per EN 338, GL28h per
EN 14080, with grain angle), `concrete_class` and `reinforced_concrete_class`
(C25/30, C30/37, C40/50 per EN 1992-1-1 table 3.1), plus CFRP and CFRP
laminate. S355, C24 and RC C30/37 equal the old default constructors.

## Costs

`Prices::member(choice, length)` = mass × price per kg (by grade) + a cost
per member end by joint type: pinned 20, dowelled 30, bolted 40, monolithic
60, welded 80, rigid 80 (sandbox). Steel 2.3 to 2.9 per kg, timber 1.0 to
1.8, concrete 0.22 to 0.3, RC 0.42 to 0.5, CFRP 25 to 30. Game values,
agreed with the scenario thread's cost model.

## Sandbox limits

`Limits::check` refuses: walls, flanges, webs or bars under 3 mm; anything
over 2 m; hollow sections with d/t or b/t over 60 (local buckling, which the
beam model does not see); I flanges with b/2tf over 30 or webs with hw/tw
over 120; L/r over 300 (round bars are exempt as ties); concrete that is not
a solid rectangle (the RC model puts bars in its corners); joints that do
not suit the material (welded timber).

`Catalog::validate` runs on load: unique ids, a price for every material and
joint used, and every part and every family size × grade × joint inside the
limits.

## Not yet

Cables (tension only), decks, supports and foundations, springs and dampers
are later phases. The palette and inspector UI waits for the UI flow PR.
