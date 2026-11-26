# Scenario loads (`breakit-core/src/loads`)

Every load in a scenario comes from a visible lab rig, but the forces are the
engineering models of the real thing. All of it is Bevy-free and lives in
`breakit-core::loads`; the module docs of each file give the formulas.

| Rig | Module | Model |
| --- | --- | --- |
| Fan wall / wind tunnel | `wind/` | Quasi-steady drag on the velocity relative to each member (C_d by section and Reynolds number), mean profiles, a 3D von Kármán turbulence field with spatial coherence, shielding, a wake oscillator per member for vortex shedding and lock-in, quasi-steady galloping of rectangles, pressures on signs and plates, and aeroelastic checks. See *Wind dynamics* below |
| Shake table | `seismic.rs` | Uniform base excitation in the table frame: `−m a_g(t)` on every free node (exact for a rigid table). Synthetic Kanai-Tajimi / Clough-Penzien quakes scaled to a PGA, PEER `.AT2` records, sine and sweep tests. Zero-phase 0.1 Hz high-pass and drift removal give a bounded table displacement that matches the applied acceleration. Response spectrum for the brief |
| Wave flume | `water.rs` | Morison inertia + drag on submerged member parts, buoyancy (sealed or flooded sections), flood level rise with a current, Airy waves with Wheeler stretching, solitary wave for a surge, piston paddle motion (Biésel transfer function, Goring for the solitary wave) |
| Model vehicles | `traffic.rs` | Lanes resolved from a line in the scenario to the player's deck nodes, axle loads split between nodes, quarter-car suspension per axle (bounce, lift-off), braking force and load transfer, vehicles fall through gaps |
| Crash / dropped object | `impact.rs` | Rigid impactor with a padded contact against member surfaces (capsules of half the section depth), split to the end nodes, integrated inside the frame and applied as the frame-averaged force (impulse conserved), contact events for sound and visuals |

Members float by the share of their section under the local surface; closed tubes displace their outline until damaged, broken pieces keep their buoyancy and drag (`docs/water.md`, "Floating, flooding and debris"). A water case can take its kinematics from the simulated water instead of the analytic waves: set `Water::field` (trait `WaterField`); see `docs/water.md`.

## Using it

```rust
let mut loads = LoadSet::new(vec![
    LoadCase::new("Gale", Window::new(0.0, None, 2.0), LoadKind::Wind(Wind::new(25.0, Vec3::x()))),
    LoadCase::new("Quake", Window::new(5.0, None, 0.0), LoadKind::Quake(Quake::synthetic(0.35, 10.0, Vec3::x(), 7))),
]);
loads.prepare(&structure);          // at test start: lanes, geometry, ground motion
loop {
    loads.step(&structure, &mut sim, 1.0 / 60.0);   // sets every node's force, then steps
    let rigs = loads.rig_state(sim.time());          // fan drive, table motion, paddle, vehicles, impactors
    let offset = loads.table_offset(sim.time());     // add to every drawn position during a quake
}
```

- `LoadSet` owns all nodal forces (`set_force` replaces), so constant scenario
  loads go in `LoadSet::static_loads`.
- `LoadSet::factor` scales every case (proof tests); `LoadCase::factor` one case.
- `Window { start, duration, ramp }`: smooth ramp in and out. Suddenly applied
  loads overshoot about 2x; use a ramp unless the shock is the point. Quakes
  carry their own envelope, so use ramp 0 there.
- `LoadCase::intensity()` gives the headline number for the brief (m/s, g, m, kN, kJ).
- Equipment that comes loose mid-run: `LoadSet::add_case(&structure, LoadCase::new(name, Window::new(now, None, 0.0), LoadKind::Impact(Impactor::released(name, mass, radius, position, velocity))))`. It falls, hits the deck or the floor, and the other cases keep their state.
- `totals()` gives the net force per case each frame (base shear, total drag)
  for the chart recorder.
- Serde derives on all scenario data behind the `serde` feature.

## Limits

- Distributed loads are lumped to member ends. A member's span moment from a
  load between its nodes is not seen by the solver, so long members under
  wind or water should be split (`add_line`).
- Forces are evaluated once per frame and held over the substeps; content
  above a few Hz is sampled at 60 Hz (vortex shedding fades out above 4–8 Hz,
  gusts stop at 5 Hz).
- No wave slamming, diffraction or breaking; no added mass on the structure's
  own acceleration; vehicles are moving sprung masses, not bodies with pitch.
- Cost on a 1,680-member lattice tower (`cargo run --release --example
  loads_bench`, 4 cores): wind with gusts, shedding and shielding 0.35
  ms/frame, waves 0.3 ms, quake 0.004 ms, against ~10 ms for the solver.

## Wind dynamics (`loads/wind/`)

Full CFD around a structure costs minutes per second of flow, so the wind
uses the reduced models of wind engineering. Each is validated against a
textbook case in `tests/loads_wind_dynamics.rs` and `tests/loads_wind_checks.rs`.

| Effect | Model | Check |
| --- | --- | --- |
| Turbulence (`turbulence.rs`) | Random Fourier modes over the von Kármán spectrum (Kraichnan; Smirnov et al. 2001), divergence-free, convected with the mean wind, σ_v = 0.75 σ_u, σ_w = 0.5 σ_u, σ_u constant with height (EN 1991-1-4) | Intensities, correlation falling with separation |
| Vortex shedding (`shedding.rs`) | van der Pol wake oscillator per member (Facchinetti et al. 2004), St by section (EN Table E.1), C_L0 by section and Re, spanwise correlation length from the motion (EN Table E.4) | Strouhal frequency, C_L amplitude, lock-in band of a spring-mounted cylinder at U_r ≈ 5, lock-in of a 1 Hz mast at v_crit = f D / St |
| Galloping | Quasi-steady cross-wind coefficient of rectangles at small angles (slope A₁ = 2.69, Parkinson and Smith), scaled by EN Table E.7 | A square cantilever gallops above the Den Hartog speed and not below |
| Signs and plates (`panels.rs`) | Flat-plate normal force with EN 1991-1-4 §7.4.3 eccentricity | Total force and torsion of a sign |
| Checks (`checks.rs`) | Rayleigh first frequency, v_crit, Scruton, galloping onset, Selberg flutter | Hand calculations |

For visuals and sound, `Wind::velocity(p, t, ramp)` gives the flow anywhere
(streamers, tufts), `Wind::member_aero()` gives each member's flow,
dynamic pressure, shedding frequency, lift and lock-in, and
`WindRig::tones` lists the loudest Aeolian tones (frequency, strength,
member).

**Cost and the hot-path rules.** Wind is a hot path (`CONTRIBUTING.md`):

- The turbulence sum over modes is a SIMD kernel (`turbulence.rs`) with its
  own vector cosine (Cody-Waite reduction, Taylor polynomial to r²⁴, within
  2e-12 of `f64::cos`); the scalar level is the reference. 5x faster than
  scalar on AVX2 (gate: 2x).
- Members and node gusts run on the rayon pool through
  `dynamics::pool::ordered_chunks` (the calling thread works too, so a
  sleeping pool costs nothing) from `wind::PARALLEL_MIN_MEMBERS` (768)
  members, with each chunk writing only its own members and forces added in
  member order: bit-identical on any thread count
  (`tests/loads_wind_exec.rs`, and the gate). `Wind::set_execution` picks
  the SIMD level and the threshold (`usize::MAX` = one thread).
- The member loop itself is scalar: per member it is a branchy mix of
  section type, Reynolds number, galloping and the wake oscillator. Drag
  uses two-point Gauss along the member (exact for a cubic), shape-only
  coefficients are cached at prepare, and the shielding grid uses a cheap
  hash (it is rebuilt every 0.25 s).

Measured on the 4-core cloud machine with the solver stepping between
frames (median frame of `LoadSet::apply`, before → after this pass):

| Structure | Steady wind, 4 threads | Steady, 1 thread | Gusts, 4 threads | Gusts, 1 thread |
| --- | --- | --- | --- | --- |
| tower 5x5x12 (1,272 members) | 0.23 → 0.20 ms | 0.46 → 0.30 ms | 0.44 → 0.24 ms | 1.02 → 0.40 ms |
| bridge 60 panels (2,172 members) | 0.39 → 0.31 ms | 0.74 → 0.57 ms | 1.07 → 0.41 ms | 2.65 → 0.70 ms |

Threads broke even at about 500 members and did not pay below; members
cost about 0.25 µs each on one thread.

**Solver damping.** The implicit integrator adds numerical damping of about
ζ ≈ ω h / 2 per substep `h`: about 2 % at 2 Hz with the default 4
substeps, against 0.2–0.5 % for real steel. That damps lock-in and galloping
out of stiff structures at default settings. The tests measure the solver's
real damping by free decay and use 16 substeps.

**A GPU lattice-Boltzmann flow just for looks: not worth it yet.** A coarse
D3Q19 grid around the model (say 128×64×64, 0.5 M cells) runs at a few
milliseconds per step on a mid-range GPU and needs several steps per frame.
At that resolution the flow is a low Reynolds number flow: a 50 mm member is
under one cell, so it shows big laminar eddies that do not match the forces
the structure feels, and it costs as much as the solver. Particles and
ribbons advected through `Wind::velocity`, with Kármán-street sprites behind
members phased by `member_aero().lift_coefficient`, show the same things
(gusts, wakes, shedding at the right frequency) for almost nothing and
always agree with the forces. Revisit if a scenario needs flow around a
large bluff body (a building block) to be seen.

