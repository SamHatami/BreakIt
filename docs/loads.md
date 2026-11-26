# Scenario loads (`breakit-core/src/loads`)

Every load in a scenario comes from a visible lab rig, but the forces are the
engineering models of the real thing. All of it is Bevy-free and lives in
`breakit-core::loads`; the module docs of each file give the formulas.

| Rig | Module | Model |
| --- | --- | --- |
| Fan wall / wind tunnel | `wind.rs` | Quasi-steady drag `½ρC_d d|v_n|v_n` on the velocity relative to each member, C_d by section (cylinder drag crisis with Reynolds number, EN 1991-1-4 rectangle coefficients by depth ratio), uniform / power-law / log-law / fan-jet profile, Kaimal gusts convected as frozen turbulence, grid-based shielding of leeward members, vortex-shedding cross-wind force at the Strouhal frequency |
| Shake table | `seismic.rs` | Uniform base excitation in the table frame: `−m a_g(t)` on every free node (exact for a rigid table). Synthetic Kanai-Tajimi / Clough-Penzien quakes scaled to a PGA, PEER `.AT2` records, sine and sweep tests. Zero-phase 0.1 Hz high-pass and drift removal give a bounded table displacement that matches the applied acceleration. Response spectrum for the brief |
| Wave flume | `water.rs` | Morison inertia + drag on submerged member parts, buoyancy (sealed or flooded sections), flood level rise with a current, Airy waves with Wheeler stretching, solitary wave for a surge, piston paddle motion (Biésel transfer function, Goring for the solitary wave) |
| Model vehicles | `traffic.rs` | Lanes resolved from a line in the scenario to the player's deck nodes, axle loads split between nodes, quarter-car suspension per axle (bounce, lift-off), braking force and load transfer, vehicles fall through gaps |
| Crash / dropped object | `impact.rs` | Rigid impactor with a padded contact against nodes, integrated inside the frame and applied as the frame-averaged force (impulse conserved), contact events for sound and visuals |

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
