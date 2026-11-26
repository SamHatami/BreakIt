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
per member end by joint type, in installed kr (Sam's call): pinned 400,
dowelled 600, bolted 800, monolithic 1 200, welded 1 600, rigid 1 600
(sandbox). Per kg: steel 46 to 58, timber 20 to 36, concrete 4.4 to 6, RC
8.4 to 10, CFRP 500 to 600. Kept consistent with the scenario crate's cost
model.

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

## Automatic joints

`auto_joint(choice, others, allowed)` picks a member end's joint from its
material and the materials of the other members at that node: timber to
timber dowelled, concrete to concrete monolithic, steel and mixed bolted,
concrete to anything else on a pinned bearing, solid round bars pinned. The
pick is always one of the family's joints. Steel defaults to bolted rather
than welded because joints are priced per member end: an all-welded starter
truss costs about 90 000 kr against the bridge sheet's 54 000 kr budget.
Welding is stiffer and the player can choose it.

## Node detailing

`node::detail_node` looks at every member end at a node and gives each one a
joint type (unless the player set it), a physical detail, plate sizes and
factors on the joint's numbers:

- 3+ steel members share a **gusset** in their plane (normal, reach from the
  node, standard thickness). Reach grows with the deepest member and the
  shallowest angle; thickness with the thickest wall and the member count.
- A steel member square (75 degrees or more) to a continuous run gets a
  pinned **fin plate**, unless the player picked another joint.
- Two steel members in line: bolted **splice**, twice the knee stiffness.
  At an angle: **end plates**. Alone (a support): **base plate**.
- Timber: dowelled **slotted-in plate**. Concrete: **cast** with concrete,
  else a **bearing**. Round bars and two-member pins: **clevis**.
- Under 30 degrees to the nearest member the bolt group sits off the axis:
  capacity falls linearly to half at 0 degrees. Five or more members: 0.9.

The game multiplies the resolved joint's force and moment capacity by
`capacity` and its finite stiffness by `stiffness`, so the solver sees the
node.

## In the game (breakit-game)

- `design::BeamKind(MemberChoice)` is what each `Beam` is built from;
  `Beam.joints: [Option<JointSpec>; 2]` holds the player's override per end,
  `None` = automatic. `Design::node_detail(node)` and `Design::end_details(i)` give the
  node detailing, `Design::joints(i)` the actual joints, and `to_structure`
  resolves the choice at the member's length, scales by the detail and calls
  `set_ends`. `Design::bill` prices joints by kind.
- Palette (`build_drawer`): material tabs (keys 1 to 4) and the catalog parts
  of that material. A scenario's material limits filter the parts.
- Inspector (`build_inspector`): standard size (`[` `]` or the arrows), grade
  (G), joint at each end (click: automatic, then each allowed type), section
  and joint utilisation (`MemberState.joints` from
  `MemberForces.joint_utilization` in build, `MemberDamage.joint` in the
  test), cost including joints.
- Visuals: `MemberStyle.profile` is the real section. The strong axis is
  local y (up) in both the solver (`iz`) and the mesh (`h`).

## Not yet

Cables (tension only), decks, supports and foundations, springs and dampers
are later phases. Engineer controls are open to everyone until the unlock
model is decided; Sandbox custom dimensions have no UI yet.
