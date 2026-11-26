# Sound

BreakIt's sound is made while the game runs. Ambience, rigs, UI and member stress are pure synthesis. Damage, breaks and debris impacts default to a hybrid: a small bank of CC0 recordings for the transient and texture, picked, pitched, filtered and layered by code, with synthesised groans, rings and thumps mixed under them. Pure synthesis for those is still there (`Command::Style(SoundStyle::Synth)`).

Why the hybrid: the first, fully synthesised version did not read as a structure failing when Sam listened to it. Real fracture is thousands of micro-events (grit, fibres, crack fronts) and the recordings carry that texture cheaply.

The recordings are 52 clips (2.4 MB, mono 32 kHz) in `crates/breakit-audio/samples/`, all CC0 from OpenGameArt, listed with their sources in `samples/SOURCES.md`. `tools/cc0_samples.py` downloads, trims, normalises and converts them and regenerates `src/sample_index.rs`; it needs network access to opengameart.org.

- `crates/breakit-audio` (no engine): DSP blocks, the patches, the mixer, the hall reverb, device output (cpal) and a WAV recorder.
- `crates/breakit-game/src/sound.rs` (Bevy): turns game and solver state into audio commands. It never touches samples.

## How it fits together

The game sends `breakit_audio::Command`s through an `AudioHandle` (the `Sound` resource in the game). They are cheap messages on a channel; the `Mixer` applies them between blocks on the audio thread.

| Command | Sent by `sound.rs` when | Sounds like |
|---|---|---|
| `Stress` | every frame in simulate mode, one per material: utilisation of the hardest-working member, rise rate, members above half capacity | steel groans and pings, timber creaks and fibre crackle, concrete strain rumble and micro-crack ticks, composite ticks. Quiet below about 0.3 utilisation, busy near capacity |
| `Damage` | a member's `DamageState` gets worse (yielded, cracked, crushed, delaminated) | deep settling groan, crack tick and crackle, gritty crunch, dry rip |
| `Break` | a new `BreakEvent`, at the failure point along the member, scaled by member mass | steel snap and ring (rupture), bend-groan into a clank (hinge), timber splinter cascade or tearing split, concrete crack and crumble, rebar ping, composite crack with fibre fizz |
| `Impact` | a debris piece hits the floor (`DebrisWorld::drain_impacts`) | steel clang, timber knock, concrete thud and gravel, composite clack |
| `Rig` | a `RigSound` component is on an entity | fan wall, shake table, wave paddle, cart, winch, hydraulic power unit |
| `Ambient` | at start, from `SoundAnchors` | room tone, HVAC, mains hum, distant machinery |
| `Ui` | build-mode edits, found by diffing the design and build tool each frame; test start and stop | pen clicks, ruler clacks, eraser, rotary switch, stamps, relay plus warning tone |
| `Listener` | every frame, from the orbit camera | |

`recorded.rs` builds the hybrid one-shots: a random clip per layer, pitch varied ±6% and set by member mass (heavier is lower and longer), gain varied about ±2 dB, a random low-pass, layers spread in time, and debris showers that thin out. Impact speed sets loudness and brightness.

Patches are modal synthesis with stochastic excitation: an exciter (click, noise burst, stick-slip pulse train, decaying crackle) drives a body (a bank of resonant modes or a band filter). Each patch file explains its own sounds: `material.rs`, `rigs.rs`, `ambience.rs`, `ui.rs`.

## The mixer

- Four buses (structure, rigs, ambience, UI) with their own volume, plus master.
- Positioning: inverse-distance attenuation past a reference distance, equal-power pan from the camera's right vector, air absorption, and a reverb send that grows with distance. `Place::metres_per_unit` lets the hall scenery, drawn 20x larger than the play table, be heard at its real lab distance.
- Hall reverb: an 8-line feedback delay network sized for the 60 x 30 x 12 m steel-clad lab (about 3 s mid-band RT60, shorter highs, late early reflections).
- At most 48 one-shots; the quietest is stolen. At most 6 impacts per 50 ms, so a collapse is not a wall of identical knocks. Rigs not updated for 0.5 s fade out.
- A peak limiter with a soft knee: the output never exceeds 1.

Cost: the full mix (beds, reverb and a busy collapse) renders 100 to 180 times faster than real time on one core of the cloud container, so under 1% of a core in the game.

## Listening and checking

```text
cargo run --release -p breakit-audio --example render_demo -- <dir> [stress breaks impacts ambience rigs ui collapse]
```

writes one WAV per scene (add `steel timber concrete composite` for one failure story per material; `BREAKIT_STYLE=synth` renders the pure-synthesis path), each a scripted sequence of the same commands the game sends. `BREAKIT_AUDIO_WAV=<file>` makes the game record to a WAV instead of the sound card, and `BREAKIT_CAPTURE=<dir>` records `<dir>/audio.wav` alongside the screenshots. `BREAKIT_MUTE=1` starts muted. Without a sound device (the cloud container) the game runs silent.

On Linux, building needs the ALSA headers (`libasound2-dev`).

## Hooks for other systems

- Test rigs: add `RigSound { kind, drive, speed }` next to a `Transform`. `drive` is 0..1, `speed` the physical rate (wind m/s, actuator, paddle or cart velocity m/s, rope speed). `breakit_core::RigState` from the loads work maps onto it directly.
- Lab scene: `SoundAnchors` is rebuilt from `breakit_lab::LabAnchors` and `LabSettings::model_scale` whenever the anchors change (HVAC in the roof and at the control room, hum at the cabinets and nearest lamps, machinery at the power units and pumps).
- Anything else: `Res<Sound>` and `sound.send(Command::Ui(UiSound::Pass))`.

## Open points

- Tuned by spectrogram and level measurements against the recordings, not by ear. The render demo is the quickest way to judge.
- There is no recording of steel actually snapping or buckling in the bank; steel uses metal hits and falls over synthesised groan and ring. A better CC0 steel source would help most.
- Stress beds first become audible at about 0.45 utilisation; whether that should be earlier is a game-feel call.
- Damage and break sounds do not yet depend on the profile (a thin tube and a heavy I-beam of the same mass sound alike).
