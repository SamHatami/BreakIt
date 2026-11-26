# Sound

All sound in BreakIt is synthesised while the game runs, with no sample files.

The structure's sounds (members straining, damage, breaks, debris) are physically based: the game hands the audio engine what the solver knows about a member, and the sound is computed from that. The model, its sources and its limits are in `docs/sound-physics.md`. Ambience, rigs and UI are ordinary synthesis patches.

- `crates/breakit-audio` (no engine): member vibration modes (`body.rs`), the modal engine (`modal.rs`), the force events (`material.rs`), the listener that turns a `DynamicSim` into commands (`listen.rs`), the ambience, rig and UI patches, the mixer, the hall reverb, device output (cpal) and a WAV recorder.
- `crates/breakit-game/src/sound.rs` (Bevy): runs the listener on the game's simulation and turns the rest of the game state (debris, rigs, lab anchors, build edits, camera) into audio commands. It never touches samples.

## How it fits together

The game sends `breakit_audio::Command`s through an `AudioHandle` (the `Sound` resource in the game). They are cheap messages on a channel; the `Mixer` applies them between blocks on the audio thread.

| Command | Sent when | Sounds like |
|---|---|---|
| `Strain` | every frame in simulate mode, for the 8 members whose load or damage is changing most: the member's `Body` (material, section, length), utilisation and damage | joints sticking and slipping as the load moves (steel: a bolt bangs into bearing; timber: creaks whose pitch rises with loading rate), micro-cracks as damage grows (sparse ticks early, dense crackle with rare loud cracks near failure). A steady load is silent |
| `Wind` | every frame while a wind load blows, for the 6 members whose vortex shedding is loudest (`listen::wind_tones`): the member's `Body`, shedding rate, radiating lift force and lock-in | the member singing: the wind's own tone at the shedding rate, and the member's modes ringing when a gust carries that rate through one of them. Wanders on a still member, steady when locked in |
| `Damage` | a member's `DamageState` gets worse (yielded, cracked, crushed, delaminated), with the elastic energy it holds | a small release in that member: a crack and its aftershocks, a joint slip at first yield, grinding, plies peeling |
| `Break` | a new `BreakEvent`: the member's `Body`, cause, position along it, the elastic energy it held and how much of that was axial | the force the member carried steps to zero and both pieces ring from it, plus that failure mode's micro-fracture: steel tie bang and pitched ring, hinge tearing, bolts shearing in sequence, timber splinter run and aftershocks, split along the grain, concrete crack and rubble, bars snapping, carbon crack and fibre fizz |
| `Impact` | a debris piece hits the floor (`DebrisWorld::drain_impacts`): the piece's `Body`, speed and mass | a Hertz contact pulse into the piece's free modes and the table: steel clang, timber knock, concrete thud and grit |
| `Rig` | a `RigSound` component is on an entity | fan wall, shake table, wave paddle, cart, winch, hydraulic power unit |
| `Ambient` | at start, from `SoundAnchors` | room tone, HVAC, mains hum, distant machinery |
| `Ui` | build-mode edits, found by diffing the design and build tool each frame; test start and stop | pen clicks, ruler clacks, eraser, rotary switch, stamps, relay plus warning tone |
| `Listener` | every frame, from the orbit camera | |

Patches are modal synthesis with stochastic excitation: an exciter (click, noise burst, stick-slip pulse train, decaying crackle) drives a body (a bank of resonant modes or a band filter). Each patch file explains its own sounds: `material.rs`, `rigs.rs`, `ambience.rs`, `ui.rs`.

## The mixer

- Four buses (structure, rigs, ambience, UI) with their own volume, plus master.
- Positioning: inverse-distance attenuation past a reference distance, equal-power pan from the camera's right vector, air absorption, and a reverb send that grows with distance. `Place::metres_per_unit` lets the hall scenery, drawn 20x larger than the play table, be heard at its real lab distance.
- Hall reverb: an 8-line feedback delay network sized for the 60 x 30 x 12 m steel-clad lab (about 3 s mid-band RT60, shorter highs, late early reflections).
- At most 48 one-shots; the quietest is stolen. At most 6 impacts per 50 ms, so a collapse is not a wall of identical knocks. At most 8 straining members; one not updated for 1.5 s is dropped. Rigs not updated for 0.5 s fade out.
- A peak limiter with a soft knee: the output never exceeds 1.

Cost: a busy collapse with ambience and reverb renders about 70 times faster than real time on one core of the cloud container (1.5% of a core). The worst case the mixer allows, 48 one-shots all ringing long steel breaks plus 8 straining members, measured about 10% of a core (`cargo test --release -p breakit-audio -- --ignored --nocapture cost`). New voices get fewer modes when 12 or more one-shots already sound.

## Listening and checking

```text
cargo run --release -p breakit-audio --example render_demo -- <dir> [stress breaks impacts geometry ambience rigs ui collapse]
```

writes one WAV per scene (add `steel timber concrete composite` for one failure story per material; `geometry` plays the same events on members of different size), each a scripted sequence of the same commands the game sends. ```text
cargo run --release -p breakit-audio --example render_sim -- <dir>
```

runs the real solver on a steel and a timber truss loaded to failure and renders what the listener makes of it: nothing is scripted but the load. `tests/simulated.rs` checks the same path end to end (a steel tie pulled apart reports a rupture with the energy σ²/2E · volume and is heard; a structure at rest is silent).

`BREAKIT_AUDIO_WAV=<file>` makes the game record to a WAV instead of the sound card, and `BREAKIT_CAPTURE=<dir>` records `<dir>/audio.wav` alongside the screenshots. `BREAKIT_MUTE=1` starts muted. Without a sound device (the cloud container) the game runs silent.

On Linux, building needs the ALSA headers (`libasound2-dev`).

## Hooks for other systems

- Test rigs: add `RigSound { kind, drive, speed }` next to a `Transform`. `drive` is 0..1, `speed` the physical rate (wind m/s, actuator, paddle or cart velocity m/s, rope speed). `breakit_core::RigState` from the loads work maps onto it directly.
- Lab scene: `SoundAnchors` is rebuilt from `breakit_lab::LabAnchors` and `LabSettings::model_scale` whenever the anchors change (HVAC in the roof and at the control room, hum at the cabinets and nearest lamps, machinery at the power units and pumps).
- Anything else: `Res<Sound>` and `sound.send(Command::Ui(UiSound::Pass))`.

## Open points

- Nobody has listened to it in the cloud container: it is checked by measurement (mode frequencies and decay against beam theory, spectra against reference recordings) and needs Sam's ear.
- See `docs/sound-physics.md` for what the model leaves out.
