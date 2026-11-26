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
  loads_bench`): wind with gusts and shielding 0.9 ms/frame, waves 0.3 ms,
  quake 0.005 ms, against ~16 ms for the solver.

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

Cost on the 1,680-member tower (`cargo run --release --example
loads_bench`, 4 cores): 0.67 ms/frame with turbulence, shedding and
galloping (1.3 ms on one thread), 0.36 ms for drag and shielding alone.

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

