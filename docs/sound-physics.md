# Physically based structure sound

How BreakIt computes the sound of members straining, breaking and landing from the simulation, what it is based on, and where it stops being physics.

## The idea

A sound is a **force** acting on a **body**. The body is a member; its vibration modes follow from its geometry and material. The force is whatever just happened to it: it was hit, or it was carrying load and suddenly is not, or a joint slipped, or a crack jumped. Both halves come from numbers the solver already has, so a long thin steel tie and a short fat concrete column sound different without anyone having tuned either.

```
solver                     listen.rs (StructureListener)   breakit-audio
member E, G, rho, A, I, L  -> Body ---------------------->  body.rs    modes: frequency, decay, gain
utilisation, damage        -> Command::Strain ----------->  material.rs  force events
break cause, position,     -> Command::Break  ----------->     |
  axial force, end moments    (elastic energy, axial share)    v
debris speed, size (game)  -> Command::Impact ----------->  modal.rs   modes driven by the force -> sound
```

## Recorded timbre (`timbre.rs`)

The first version computed everything, and it sounded like glass. Two reasons. A member was an ideal bar on its own, which rings at a few clean pitches for seconds, exactly as a glass does. And the solver steps tens of times a second, so it cannot say what the milliseconds of a crack are like; short computed clicks stood in for them, and the ear heard clinks.

So now:

- What an event sounds like comes from a recording of that kind of event (`samples/`, one folder each: a bar snapping, a beam creaking, a single crack, a plank landing).
- The simulation still decides everything that happens: which events, when, how many (the same avalanche statistics, aftershock law, stick-slip and contact model as below), how loud (energy released), and at what pitch (a slice plays slower for a bigger section; half of the 1/size law, an octave at most).
- The recorded slice is also the force that rings the member's modes, so the member's length and section still colour it. The modes carry the joint loss of a member built into a frame (loss factor 0.012 for steel up to 0.04 for concrete), not the far smaller loss of the bare material, so they thud and die away instead of ringing.
- Recordings are cut into single events where the level jumps, so one recording of a splitting beam gives many different cracks.

Wind tones are not affected: they are continuous and fully computed.

## The body: modes from beam theory (`body.rs`)

| What | Formula | What it gives |
|---|---|---|
| Bending modes, both axes | `f_n = (β_n L)² / 2πL² · √(EI/ρA)`, `β_n L` from the end conditions (held in joints, cantilever stub, free piece), with Timoshenko's shear correction | the inharmonic clang of a bar. Twice the length is four times lower |
| Axial modes | `f_n = n c / 2L`, `c = √(E/ρ)` | a harmonic series: the pitched ring of a tie that snaps. 431 Hz for 6 m of steel |
| Wall modes of tubes | ovalling: `f_i = i(i²−1)/√(i²+1) · √(E t² / 12ρ(1−ν²)R⁴) / 2π` | the bright ring of a hollow section |
| Decay | amplitude `exp(−π f η t)`, loss factor η: steel 0.002, carbon 0.006, timber 0.016, concrete 0.028; times `1 + 3·damage` | steel rings for seconds, timber knocks, concrete thuds; a cracked member sounds dead |
| Radiation | a bar narrower than the wavelength in air is a dipole: amplitude falls as `f^1.5` below `ka = 1` | slender members are thin and bright, fat ones full |
| Mode shape at the drive point | `φ_n(x)` | hitting or breaking a member at a node of a mode leaves that mode out |

A long member has hundreds of audible modes. Each voice gets a budget (28 per piece for a break, 22 for an impact, 18 for a straining member); neighbours are merged, keeping their energy.

## The force (`material.rs`)

**Release.** A member that breaks was carrying force. That force steps to zero in the time the crack takes to cross the section, and each piece springs back from its static deflection. The modal engine handles a step exactly like any other force: mode n rings at `gain / ω_n`, which is the static deflection. Tension dumps the energy into axial modes, bending into bending modes; the split comes from the solver's axial force and end moments (`N²L/2EA` against `∫M²/2EI`).

**Micro-fracture.** Cracks advance in jumps. Measured acoustic emission from wood and other brittle solids has event energies on a power law, `P(E) ~ E^−β` with β about 1.4, and aftershocks that thin out by Omori's law, rate `~ t^−0.75` (Mäkinen et al. 2015 for wood in compression). The model draws events from those laws. Duration follows size (`E^⅓`), so large cracks are low and small ones tick. Damage in the solver never decreases, so driving the event rate from *new* damage reproduces the Kaiser effect: reloading to a load a member has already seen is quiet.

**Stick-slip.** Joints and crack faces under changing load stick until friction is overcome, then slip in small jumps. Each slip is an impulse into the member. At a steady rate of load change the slips are evenly spaced, and their rate is the pitch of the creak: it rises when the load changes faster. Steel has one large slip per event (a bolt going into bearing); timber has long regular trains; concrete only grinds once cracked.

**Contact.** A piece landing pushes on the floor for the Hertz contact time `τ = 2.87 (m² / (R E*² v))^⅕`. A long piece is not rigid, so the mass is limited to what a bending wave reaches during the contact. Steel on the floor: a fraction of a millisecond, exciting modes up to several kHz. Timber: milliseconds, a dull knock. A loose steel piece is the one case computed from end to end: it is a free bar, so its free modes under that one pulse are the whole sound, with no recording and no joint loss (`examples/render_bars.rs` renders it, and two free bars knocked together). The other materials land with a recorded knock.

**Each failure mode** is a short score of those events:

| Cause | Score |
|---|---|
| Steel rupture | release in 0.25 ms, then the stress wave rattles each joint every `2L/c` |
| Steel hinge | the tension side tears in 5 to 8 growing steps over 35 to 100 ms |
| Connection | 2 to 4 fasteners shear within milliseconds, then release; the member stays whole and the neighbour is jolted |
| Timber splinter | 6 to 10 fibre bundles let go over 10 to 28 ms, then about 100 aftershocks |
| Timber split | a crack runs the length at 300 m/s, then release |
| Crushing | no clean release: hundreds of grain or fibre events, thinning over half a second |
| Concrete crack, shear | release in 0.5 ms, then rubble |
| Rebar rupture | concrete cracks, then 2 to 4 bars snap and ring inside it |
| Fibre rupture | release in 0.08 ms, then 260 fibre snaps within 70 ms |
| Delamination | a crack runs the length at 1500 m/s, then release |

**Loudness** is `(E / 1 kJ)^0.3` of the energy released. Physics says square root; a game cannot play the 60 dB between a creak and a girder snapping, so it is compressed.

## Wind: members that sing (`wind.rs`)

Wind sheds vortices from alternate sides of a member at the Strouhal rate `f = St·U/d`, and the lift force that swings with them makes sound in two ways.

- The air: a fluctuating force on a body much smaller than the wavelength radiates as a dipole, `p = ωF / 4πrc` (Curle). This is a wire humming; the member need not move. Twice the wind is twice the pitch and eight times the pressure.
- The member: the same force drives the member's bending modes, and a moving member pushes the air it displaces plus the added mass around it. Per unit of lift a mode radiates `4 (ρ_air/ρ) (S/A) β² / √((1−β²)² + η²)` times what the air does, with `β = f/f_n`, `S` the outline of the section and `A` its material area. Off a mode that is nothing. On a mode it is `1/η` larger: about 2 for a typical steel tube (thin walls, little damping) and under 1 for a solid timber post. So a steel tube rings out above the wind's tone when a gust sweeps the shedding rate through one of its modes, and timber mostly hums.

The rate, flow and motion of every member come from the wind model in `breakit-core` (`Wind::member_aero`). The lift coefficient is the section's (0.3 for a tube, 0.15 past the drag crisis, up to 1.0 for a square). Along a still member the wake is in step over only about six depths (EN 1991-1-4 Table E.4), so the cells add as random phases, `F = f'√(L·l_c)`; when the member moves with the shedding the cells fall into step and the tone gets louder and steadier.

Level is `(p / 1 Pa)^0.6` at 10 m, the same compressive law as the break energy (`(p²)^0.3`).

## What was checked

Nobody can listen in the cloud container, so it is checked by measurement:

- End to end (`tests/simulated.rs`): a steel tie pulled apart in the real solver reports a rupture holding the energy σ²/2E · volume, nearly all of it axial, and is heard; a structure at rest under load is silent.
- Unit tests: free-free bar frequencies against the textbook values (22.4, 61.7, 121 · √(EI/ρA)/2πL²), axial modes at `c/2L`, decay rates, the 1/ω step response, the pitch of a snapped tie at `c/4L` of its pieces, contact times, the Kaiser effect, creak rate against loading rate, strain energy against hand calculation.
- Spectra against 52 CC0 recordings of wood, metal and rock breaking (the bank an earlier version used, still in git history). This caught two errors of intuition: slender members were first made far too bassy (fixed by the dipole radiation law), and concrete rubble too dull (stones are small and hard: short contacts, bright clicks).

## What is not physics

- Mode gains are relative; absolute level comes from the energy law above, not from a radiation calculation.
- Event counts, the size of a slip, and the share of energy each part of a score gets are estimates, chosen once per material, not measured.
- Radiation is a per-mode efficiency, not a sound field: no directivity, no near field.
- Only the broken member rings (plus the table). Its neighbours get the load in the solver, and creak for it, but the stress wave is not carried through the joints.
- Debris pieces are assumed to be lengths of a typical section of their material; the debris system does not say which member a piece came from.
- Wind tones: how far the shedding rate and strength wander (3% and 35% on a still member, a tenth of that locked in) and the pressure that plays at full level (1 Pa at 10 m) are estimates. The lift drives every bending mode alike; its shape along the span is not projected onto the mode shapes.
- The table and floor are ten damped modes, the same everywhere.
- The hall is a feedback delay network sized for the lab, not a simulation of it.

## Cost

Modes are two-pole filters stored eight to a lane so the loop vectorises. A busy collapse costs about 1.5% of a core; see `docs/audio.md` for the worst case.

## Sources

- K. van den Doel, D. Pai, "The sounds of physical shapes", Presence 1998; van den Doel, Kry, Pai, "FoleyAutomatic", SIGGRAPH 2001: modal synthesis driven by contact forces.
- J. O'Brien, P. Cook, G. Essl, "Synthesizing sounds from physically based motion", SIGGRAPH 2001; O'Brien, Shen, Gatchalian, "Synthesizing sounds from rigid-body simulations", SCA 2002.
- C. Zheng, D. James, "Rigid-body fracture sound with precomputed soundbanks", SIGGRAPH 2010: brittle fracture as time-varying rigid-body sound models driven by crack impulses, with cheap proxies for debris.
- J. Chadwick, C. Zheng, D. James, "Precomputed acceleration noise for improved rigid-body sound", SIGGRAPH 2012: small objects are heard mostly through the contact pulse itself, not their modes (the "direct" part of every event here).
- P. Cook, "Physically informed sonic modeling (PhISM)", Computer Music Journal 1997, and *Real Sound Synthesis for Interactive Applications*, 2002: stochastic particle models for granular sound.
- F. Avanzini, D. Rocchesso, S. Serafin: nonlinear contact and friction models for sound; the Sound Design Toolkit's impact, friction and crumpling models.
- T. Mäkinen, A. Miksic, M. Ovaska, M. Alava, "Avalanches in wood compression", 2015 (arXiv:1506.04563): acoustic emission energy exponent 1.37 to 1.43, Omori aftershock exponent about 0.75.
- J. Sethna, K. Dahmen, C. Myers, "Crackling noise", Nature 2001.
- Beam and contact formulas: any vibration text (Blevins, *Formulas for Natural Frequency and Mode Shape*); Hertz contact time as used in impact-echo testing.

## Why not an audio middleware crate

`bevy_seedling` (Firewheel) is the likely future of Bevy audio and supports custom DSP nodes, but its README lists support up to Bevy 0.19 and the game is pinned to 0.20.1. `bevy_audio` (rodio) and `bevy_kira_audio` play clips and have no place for a synthesiser on the audio thread. The engine here is plain Rust on a `cpal` stream with no engine dependency; wrapping the `Mixer` in a Firewheel node later is a small job.
