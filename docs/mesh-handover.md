# Deformable meshes: handover notes (2026-11-26)

Branch `claude/project-thread-bmebfu` (off `physics-spike`). Crate split: `breakit-mesh` (Bevy-free) and `breakit-render` (Bevy =0.20.1, currently a stub).

## Written and tested (`cargo test --release -p breakit-mesh`, 14 of 15 pass)

- `profile.rs`: `Profile {Rect, Round, Tube, BoxSection, IBeam}`; outline loops with hard-edge normals, cap triangulation, `Profile::section()` gives the solver `Section` from the same dimensions (IPE 300 within 5% of tables). The I-beam cap is three rectangles because ear clipping broke on its collinear corners.
- `mesh.rs`: `member_template(profile)` in member space, position = (s, y, z) with s in 0..1, rings cosine-clustered near the ends for hinge kinks. A volume test checks it is closed.
- `pose.rs`: `MemberPose`. Centreline is the cubic Hermite through the end nodes with the beam end tangents (exact for elastic Euler-Bernoulli beams without span loads). Section frame = slerp of end frames rotated onto the tangent. A plastic hinge is a straight stub following the node (length = depth x smoothstep on hinge angle), then a kink; elastic tangent = rot(u, -hinge) x node tangent, sign checked against `dynamics.rs` (theta = (e x t).u - hinge). `deform()` is the CPU twin of the planned vertex shader. `GpuMember` = 9 vec4 (144 B). `rest_frame` matches `frame::member_axes` (tested).
- `state.rs`: `MaterialKind {Steel, Timber, Concrete, ReinforcedConcrete, Composite}`, `MemberStyle {profile, kind}`, `MemberVisualState`, `VisualFrame`, `VisualStructure::capture(&DynamicSim)` (estimates cracking from utilisation until the materials thread lands), `gpu_member` packing.
- `breakit-core`: added read-only `DynamicSim::plastic_hinges(member) -> [[f64; 2]; 2]`.
- `fracture.rs`: procedural per-material pieces from the deformed pose at break time. Steel: necked jagged tear + sparks. Timber: jagged face + splinter slivers. Concrete: rough faces, chunk zone, bent rebar stubs for RC, dust. Composite: delaminated splayed plies + fibres. Tests: finite, positioned, steel volume conserved, rebar only for RC.
- `debris.rs`: ground-plane rigid bodies with corner contacts and sleep. **Failing test**: small concrete chunks jitter (|w| 0.1-0.9 rad/s) and never sleep. Cause: corner-by-corner impulses with sphere inertia. Fix: box inertia tensor, or Avian/Rapier.
- `particles.rs`: ring-buffer pool, sparks/dust/splinters/fibres, tested.

## Not done

- No benchmarks yet.
- `breakit-render` is a stub. Bevy 0.20.1 builds here after `apt-get install pkg-config libwayland-dev libudev-dev libxkbcommon-dev`; `mesa-vulkan-drivers` gives lavapipe for headless screenshots.
- Planned Bevy path: `ExtendedMaterial<StandardMaterial, BeamExt>` with `#[storage(100, read_only)] Handle<ShaderBuffer>` of `GpuMember`; one entity per member with `MeshTag(member index)` sharing one mesh + material per (profile, kind) so Bevy auto-instances; custom vertex + prepass vertex shader (0.20 uses `.wesl`; reuse `decompress_vertex` from `forward_io` / `prepass::io`, positions may be compressed); push `VERTEX_OUTPUT_INSTANCE_INDEX` in `specialize` so the fragment can read damage by tag; set each entity's `Aabb` per frame from `MemberPose::bounds` (Bezier hull). References at tag v0.20.1: `examples/shader/automatic_instancing.rs`, `storage_buffer.rs`, `extended_material.rs`, `crates/bevy_pbr/src/render/mesh.wesl`, `prepass/prepass.wesl`.

## Agreed APIs

- Game shell (branch `claude/project-thread-kczrhu`): inserts `StructureVisual` per design change (`MemberStyle` from its `BeamKind`: SteelTube -> `Tube(0.05, 0.005)`, Timber -> `Rect(0.1, 0.15)`), overwrites a `VisualFrame` resource after `BreakitSet::Physics` (`from_sim` in Simulate, `from_positions` in Build). Render systems run in `PostUpdate`. It disables its cylinder renderer once `BreakitRenderPlugin` exists. Its bridge: `Simulation { sim }` resource, `FrameState`.
- Materials (branch `claude/project-thread-xyip9z`): `Material.kind: MaterialKind {Steel, Timber, Concrete, Composite}` + `reinforced()`; `DynamicSim::damage(member) -> MemberDamage {state, damage, utilization, hinge [[f64;2];2], plastic_strain, crack [f64;2], tension_crack, crush [f64;2], delamination, moment [[f64;2];2]}`; `BreakEvent` gains `position` (0..1) and more causes. When it lands, `capture()` should read it and `plastic_hinges()` can go.
