//! Game side of the sound: turns what the game and the solver are doing into
//! `breakit-audio` commands. All synthesis happens in `breakit-audio` on its
//! own thread; nothing here touches samples.
//!
//! * Build mode: clicks for placing, deleting, selecting, supports, tools,
//!   found by diffing the design and the build tool each frame, so build
//!   code needs no sound calls.
//! * Simulate mode: `breakit_audio::StructureListener` follows the solver
//!   (members whose load or damage is changing, damage onsets, breaks with
//!   the elastic energy the member let go of), and debris ground hits come
//!   from the visual debris world.
//! * Always: the hall ambience at the lab's anchor positions (from
//!   `breakit_lab::LabAnchors`), and the camera as the listener.
//! * Test rigs: give a rig entity a [`RigSound`] and a `Transform`.
//!
//! `BREAKIT_AUDIO_WAV=<file>` records everything to a WAV instead of the
//! sound card (and `BREAKIT_CAPTURE=<dir>` records `<dir>/audio.wav`), for
//! headless checks. `BREAKIT_MUTE=1` starts silent.

use bevy::app::AppExit;
use bevy::prelude::*;
use breakit_audio::listen::wind_tones;
use breakit_audio::{
    AmbientKind, AudioHandle, AudioOutput, Body, Command, MaterialKind, Place, RigKind,
    StructureListener, UiSound,
};
use breakit_core::loads::LoadKind;
use breakit_core::{Material, Section};
use breakit_lab::{Anchor, LabAnchors, LabSettings};
use breakit_render::breakit_mesh::debris::DebrisImpact;
use breakit_render::breakit_mesh::MaterialKind as MeshMaterial;
use breakit_render::StructureVisual;

use crate::analysis::{BreakitSet, Simulation};
use crate::build::BuildTool;
use crate::camera::OrbitCamera;
use crate::design::Design;
use crate::Mode;

pub struct SoundPlugin;

impl Plugin for SoundPlugin {
    fn build(&self, app: &mut App) {
        let (output, handle) = open_output();
        info!("sound: {}", output.description);
        if std::env::var_os("BREAKIT_MUTE").is_some() {
            handle.send(Command::Master(0.0));
        }
        app.insert_non_send(SoundOutput(Some(output)))
            .insert_resource(Sound(handle))
            .init_resource::<SoundAnchors>()
            .init_resource::<StructureSound>()
            .add_systems(
                Update,
                (
                    update_listener,
                    build_sounds.run_if(in_state(Mode::Build)),
                    structure_sounds
                        .run_if(in_state(Mode::Simulate))
                        .after(BreakitSet::Physics),
                    debris_sounds,
                    rig_sounds,
                    anchors_from_lab.run_if(resource_exists_and_changed::<LabAnchors>),
                    update_ambience,
                ),
            )
            .add_systems(OnEnter(Mode::Simulate), enter_test)
            .add_systems(OnExit(Mode::Simulate), leave_test)
            .add_systems(Last, finish_on_exit);
    }
}

/// Sends commands to the audio thread. Use it from any system:
/// `sound.send(Command::Ui(UiSound::Pass))`.
#[derive(Resource, Clone, Debug, Deref)]
pub struct Sound(pub AudioHandle);

/// Keeps the audio stream (or recorder) alive. Not `Send` on every
/// platform, hence a non-send resource.
struct SoundOutput(Option<AudioOutput>);

fn open_output() -> (AudioOutput, AudioHandle) {
    let wav = std::env::var_os("BREAKIT_AUDIO_WAV")
        .map(std::path::PathBuf::from)
        .or_else(|| {
            std::env::var_os("BREAKIT_CAPTURE").map(|d| std::path::Path::new(&d).join("audio.wav"))
        });
    if let Some(path) = wav {
        if let Some(dir) = path.parent() {
            let _ = std::fs::create_dir_all(dir);
        }
        return AudioOutput::record_to(path, 48_000);
    }
    match AudioOutput::open_device() {
        Ok(out) => out,
        Err(e) => {
            warn!("sound: no output ({e}); running silent");
            AudioOutput::silent()
        }
    }
}

/// Drops the output on exit so a WAV recorder writes its file.
fn finish_on_exit(mut exit: MessageReader<AppExit>, mut commands: Commands) {
    if exit.read().next().is_some() {
        commands.queue(|world: &mut World| {
            if let Some(mut out) = world.remove_non_send::<SoundOutput>() {
                if let Some(o) = out.0.take() {
                    o.finish();
                }
            }
        });
    }
}

fn v(p: Vec3) -> [f32; 3] {
    p.to_array()
}

fn update_listener(sound: Res<Sound>, camera: Option<Single<&GlobalTransform, With<OrbitCamera>>>) {
    let Some(cam) = camera else { return };
    let t = cam.compute_transform();
    sound.send(Command::Listener {
        position: v(t.translation),
        right: v(*t.right()),
        up: v(*t.up()),
    });
}

// ---------------------------------------------------------------------------
// Ambience

/// One fixed hall sound source.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AmbientSource {
    pub kind: AmbientKind,
    /// World position, or `None` for a wide, non-positional bed.
    pub position: Option<Vec3>,
    /// Metres of sound travel per world unit: 1 / `model_scale` for the
    /// hall, which is drawn larger than the play table.
    pub metres_per_unit: f32,
    pub level: f32,
}

/// The hall's sound sources. Built from the lab's [`LabAnchors`] once the
/// lab exists; until then a room-tone bed plays on its own.
#[derive(Resource, Clone, Debug)]
pub struct SoundAnchors {
    pub sources: Vec<AmbientSource>,
}

impl Default for SoundAnchors {
    fn default() -> Self {
        Self {
            sources: vec![AmbientSource {
                kind: AmbientKind::Room,
                position: None,
                metres_per_unit: 1.0,
                level: 1.0,
            }],
        }
    }
}

impl SoundAnchors {
    /// Room tone everywhere, air handling high in the roof, mains hum at the
    /// electrical cabinets and the lamps nearest the table, and machinery
    /// (motors and pumps idling) at the power units and pumps.
    pub fn from_lab(settings: &LabSettings, anchors: &LabAnchors) -> Self {
        let mpu = 1.0 / settings.model_scale.max(1e-3);
        let mut sources = SoundAnchors::default().sources;
        let mut add = |kind, position, level| {
            sources.push(AmbientSource {
                kind,
                position: Some(position),
                metres_per_unit: mpu,
                level,
            });
        };
        // Air handling units on the roof: the lab has no anchor for them, so
        // place two along the ridge, plus one at the control room's AC.
        let ridge = settings.hall.ridge;
        add(
            AmbientKind::Hvac,
            settings.lab_to_world(Vec3::new(-15.0, ridge - 1.5, 0.0)),
            0.9,
        );
        add(
            AmbientKind::Hvac,
            settings.lab_to_world(Vec3::new(15.0, ridge - 1.5, 0.0)),
            0.7,
        );
        for p in anchors.of(Anchor::ControlRoom) {
            add(AmbientKind::Hvac, p, 0.4);
        }
        for p in anchors.of(Anchor::ElectricalCabinets) {
            add(AmbientKind::ElectricalHum, p, 0.6);
        }
        let mut lamps: Vec<Vec3> = anchors.of(Anchor::HighBayLamp).collect();
        lamps.sort_by(|a, b| a.length_squared().total_cmp(&b.length_squared()));
        for p in lamps.into_iter().take(3) {
            add(AmbientKind::ElectricalHum, p, 0.25);
        }
        for p in anchors
            .of(Anchor::PowerUnit)
            .chain(anchors.of(Anchor::Pump))
        {
            add(AmbientKind::Machinery, p, 0.45);
        }
        Self { sources }
    }
}

fn anchors_from_lab(
    settings: Res<LabSettings>,
    lab: Res<LabAnchors>,
    mut anchors: ResMut<SoundAnchors>,
) {
    if lab.0.is_empty() {
        return;
    }
    *anchors = SoundAnchors::from_lab(&settings, &lab);
}

fn update_ambience(sound: Res<Sound>, anchors: Res<SoundAnchors>, mut sent: Local<usize>) {
    if !anchors.is_changed() {
        return;
    }
    for (i, a) in anchors.sources.iter().enumerate() {
        sound.send(Command::Ambient {
            id: i as u32,
            kind: a.kind,
            level: a.level,
            place: a.position.map(|p| Place::scaled(v(p), a.metres_per_unit)),
        });
    }
    // Silence sources a previous list had and this one does not.
    for i in anchors.sources.len()..*sent {
        for kind in [
            AmbientKind::Room,
            AmbientKind::Hvac,
            AmbientKind::ElectricalHum,
            AmbientKind::Machinery,
        ] {
            sound.send(Command::Ambient {
                id: i as u32,
                kind,
                level: 0.0,
                place: None,
            });
        }
    }
    *sent = anchors.sources.len();
}

// ---------------------------------------------------------------------------
// Build mode

#[derive(Default, PartialEq)]
struct BuildSnapshot {
    revision: u64,
    nodes: usize,
    beams: usize,
    supports: Vec<bool>,
    loads: usize,
    kind: Option<crate::design::BeamKind>,
    plane: Option<crate::build::BuildPlane>,
    level: f32,
    selected: Option<usize>,
    hovered: Option<usize>,
}

/// What build-mode change, if any, deserves a click.
fn classify(before: &BuildSnapshot, after: &BuildSnapshot) -> Option<UiSound> {
    if after.revision != before.revision {
        return Some(
            if after.nodes == 0 && after.beams == 0 && before.nodes > 0 {
                UiSound::Delete
            } else if after.beams > before.beams {
                UiSound::PlaceBeam
            } else if after.nodes > before.nodes {
                UiSound::PlaceNode
            } else if after.nodes < before.nodes || after.beams < before.beams {
                UiSound::Delete
            } else if after.supports != before.supports {
                UiSound::Support
            } else {
                UiSound::Select
            },
        );
    }
    if after.kind != before.kind || after.plane != before.plane || after.level != before.level {
        return Some(UiSound::Select);
    }
    if after.selected != before.selected && after.selected.is_some() {
        return Some(UiSound::Select);
    }
    if after.hovered != before.hovered && after.hovered.is_some() {
        return Some(UiSound::Hover);
    }
    None
}

fn build_sounds(
    sound: Res<Sound>,
    design: Res<Design>,
    tool: Res<BuildTool>,
    mut last: Local<Option<BuildSnapshot>>,
) {
    let now = BuildSnapshot {
        revision: design.revision,
        nodes: design.nodes.len(),
        beams: design.beams.len(),
        supports: design.supports.iter().map(|s| s.is_some()).collect(),
        loads: design
            .loads
            .iter()
            .filter(|l| l.length_squared() > 0.0)
            .count(),
        kind: Some(tool.kind),
        plane: Some(tool.plane),
        level: tool.level,
        selected: tool.selected,
        hovered: tool.hovered,
    };
    if let Some(before) = last.as_ref() {
        if let Some(s) = classify(before, &now) {
            sound.send(Command::Ui(s));
        }
    }
    *last = Some(now);
}

fn enter_test(sound: Res<Sound>, mut st: ResMut<StructureSound>) {
    *st = StructureSound::default();
    sound.send(Command::Ui(UiSound::TestStart));
}

fn leave_test(sound: Res<Sound>) {
    sound.send(Command::ResetStructure);
    sound.send(Command::Ui(UiSound::TestStop));
}

// ---------------------------------------------------------------------------
// Simulate mode

/// Per-run bookkeeping for structure sounds: the structure the run was
/// built from and the listener that follows its simulation.
#[derive(Resource, Default)]
pub struct StructureSound {
    structure: Option<breakit_core::Structure>,
    listener: StructureListener,
}

fn structure_sounds(
    time: Res<Time>,
    sound: Res<Sound>,
    design: Res<Design>,
    sim: Option<Res<Simulation>>,
    mut st: ResMut<StructureSound>,
) {
    let Some(sim) = sim else { return };
    let st = &mut *st;
    // The design cannot change during a run.
    let structure = st.structure.get_or_insert_with(|| design.to_structure());
    st.listener.update(
        sim.sim(),
        structure,
        time.delta_secs(),
        [0.0; 3],
        |command| sound.send(command),
    );
    // Members singing in the wind, while a wind load is blowing.
    let run = sim.run();
    let now = run.sim().time();
    for case in &run.loads().cases {
        if let LoadKind::Wind(wind) = &case.kind {
            if case.window.factor(now) > 0.0 {
                wind_tones(run.sim(), structure, wind, [0.0; 3], |command| {
                    sound.send(command)
                });
            }
        }
    }
}

fn core_material(k: MeshMaterial) -> MaterialKind {
    match k {
        MeshMaterial::Steel => MaterialKind::Steel,
        MeshMaterial::Timber => MaterialKind::Timber,
        MeshMaterial::Concrete | MeshMaterial::ReinforcedConcrete => MaterialKind::Concrete,
        MeshMaterial::Composite => MaterialKind::Composite,
    }
}

/// The body of a debris piece: a length of a typical member of its
/// material, as long as the piece is across. A reinforcing bar is a thin
/// steel rod.
fn debris_body(hit: &DebrisImpact) -> Body {
    match hit.part {
        breakit_render::breakit_mesh::fracture::PiecePart::Rebar => {
            let mut bar = Body::member(
                &Material::steel(),
                &Section::rect(0.016, 0.016),
                (2.0 * hit.radius).max(0.05) as f64,
            );
            bar.ends = breakit_audio::Ends::Free;
            bar
        }
        _ => Body::fragment(core_material(hit.kind), hit.radius),
    }
}

fn debris_sounds(sound: Res<Sound>, visual: Option<ResMut<StructureVisual>>) {
    let Some(mut visual) = visual else { return };
    for hit in visual.scene.debris.drain_impacts() {
        let body = debris_body(&hit);
        sound.send(Command::Impact {
            body,
            speed: hit.speed,
            mass: body.mass(),
            place: Place::at(v(hit.position)),
        });
    }
}

// ---------------------------------------------------------------------------
// Test rigs

/// Put this next to a `Transform` on a rig entity and it is heard every
/// frame. `drive` is 0..1 (how hard the rig works), `speed` its physical
/// rate: wind speed m/s for the fan wall, actuator, paddle or cart velocity
/// m/s, rope speed for the winch. Removing it fades the rig out.
#[derive(Component, Clone, Copy, Debug)]
pub struct RigSound {
    pub kind: RigKind,
    pub drive: f32,
    pub speed: f32,
}

fn rig_sounds(sound: Res<Sound>, rigs: Query<(Entity, &RigSound, &GlobalTransform)>) {
    for (e, rig, t) in &rigs {
        sound.send(Command::Rig {
            id: e.index_u32(),
            kind: rig.kind,
            drive: rig.drive,
            speed: rig.speed,
            place: Place::at(v(t.translation())),
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn snap(nodes: usize, beams: usize, revision: u64) -> BuildSnapshot {
        BuildSnapshot {
            revision,
            nodes,
            beams,
            ..Default::default()
        }
    }

    #[test]
    fn edits_map_to_clicks() {
        assert_eq!(
            classify(&snap(2, 0, 1), &snap(3, 0, 2)),
            Some(UiSound::PlaceNode)
        );
        assert_eq!(
            classify(&snap(2, 0, 1), &snap(3, 1, 2)),
            Some(UiSound::PlaceBeam)
        );
        assert_eq!(
            classify(&snap(3, 2, 1), &snap(2, 1, 2)),
            Some(UiSound::Delete)
        );
        assert_eq!(
            classify(&snap(3, 2, 1), &snap(0, 0, 2)),
            Some(UiSound::Delete)
        );
        assert_eq!(classify(&snap(3, 2, 1), &snap(3, 2, 1)), None);
        let mut a = snap(3, 2, 1);
        let mut b = snap(3, 2, 2);
        a.supports = vec![false; 3];
        b.supports = vec![true, false, false];
        assert_eq!(classify(&a, &b), Some(UiSound::Support));
        let mut h = snap(3, 2, 2);
        h.hovered = Some(1);
        assert_eq!(classify(&b, &h), Some(UiSound::Hover));
    }

    #[test]
    fn debris_bodies_are_physical() {
        let hit = DebrisImpact {
            position: Vec3::ZERO,
            speed: 3.0,
            radius: 0.5,
            kind: MeshMaterial::Steel,
            part: breakit_render::breakit_mesh::fracture::PiecePart::Body,
        };
        let body = debris_body(&hit);
        // A metre of 100 x 5 mm steel tube is about 12 kg.
        assert!(body.mass() > 5.0 && body.mass() < 25.0, "{}", body.mass());
        assert_eq!(body.ends, breakit_audio::Ends::Free);
    }

    #[test]
    fn lab_anchors_become_scaled_sources() {
        let settings = LabSettings::default();
        let lab = LabAnchors(vec![
            (Anchor::ElectricalCabinets, Vec3::new(500.0, -15.0, 100.0)),
            (Anchor::PowerUnit, Vec3::new(-300.0, -15.0, 40.0)),
            (Anchor::HighBayLamp, Vec3::new(0.0, 200.0, 0.0)),
        ]);
        let a = SoundAnchors::from_lab(&settings, &lab);
        assert_eq!(a.sources[0].kind, AmbientKind::Room);
        assert!(a.sources.iter().any(|s| s.kind == AmbientKind::Machinery));
        assert!(a
            .sources
            .iter()
            .filter(|s| s.position.is_some())
            .all(|s| (s.metres_per_unit - 0.05).abs() < 1e-6));
        // Roof units are above the table.
        assert!(a
            .sources
            .iter()
            .filter(|s| s.kind == AmbientKind::Hvac)
            .all(|s| s.position.unwrap().y > 100.0));
    }
}
