//! Procedural audio for BreakIt.
//!
//! Every sound is synthesised while the game runs: there are no sample files
//! to author, license or ship, and a sound can be regenerated or retuned by
//! editing a patch. The game describes *what is happening* with
//! [`Command`]s (members under load, a member yielding, a member breaking, a
//! chunk hitting the floor, a fan spinning) and the [`Mixer`] turns that into
//! sound on the audio thread.
//!
//! * [`dsp`]: noise, filters, resonators, envelopes.
//! * [`material`]: stress creaks and groans, damage onsets, break sounds and
//!   debris impacts for steel, timber, concrete and composites.
//! * [`ambience`]: the test hall (mains hum, HVAC, distant machinery).
//! * [`rigs`]: wind fan wall, shake table, wave paddle, carts, hydraulics.
//! * [`ui`]: build-mode clicks and mode changes.
//! * [`mixer`]: voices, buses, positioning, the hall reverb, the limiter.
//! * [`output`]: the real-time device stream (feature `device`) and a
//!   WAV recorder for headless runs.
//!
//! Like `breakit-core`, this crate has no engine dependency.

pub mod ambience;
pub mod dsp;
pub mod material;
pub mod mixer;
pub mod output;
pub mod recorded;
pub mod reverb;
pub mod rigs;
mod sample_index;
pub mod ui;
pub mod wav;

pub use breakit_core::{BreakCause, DamageState, MaterialKind};
pub use mixer::Mixer;
pub use output::{AudioHandle, AudioOutput};

/// A world position. The game is Y-up, SI units.
pub type Pos = [f32; 3];

/// Where a sound comes from.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Place {
    /// World position.
    pub position: Pos,
    /// Metres of sound travel per world unit. 1 for the structure on the
    /// play table; the hall scenery is drawn 20x larger, so its anchors use
    /// 1/20 to be heard at their real lab distance.
    pub metres_per_unit: f32,
}

impl Place {
    pub fn at(position: Pos) -> Self {
        Self {
            position,
            metres_per_unit: 1.0,
        }
    }

    pub fn scaled(position: Pos, metres_per_unit: f32) -> Self {
        Self {
            position,
            metres_per_unit,
        }
    }
}

/// Mix bus. Each has its own volume.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Bus {
    /// Structure: stress, damage, breaks, debris.
    Structure,
    /// Test rigs.
    Rigs,
    /// Hall ambience.
    Ambience,
    /// Interface sounds.
    Ui,
}

impl Bus {
    pub const ALL: [Bus; 4] = [Bus::Structure, Bus::Rigs, Bus::Ambience, Bus::Ui];

    pub fn index(self) -> usize {
        self as usize
    }
}

/// Test rigs that make continuous sound.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum RigKind {
    /// Wall of axial fans for wind loads.
    WindFan,
    /// Hydraulic shake table (servo actuators, pump).
    ShakeTable,
    /// Hydraulic wave paddle in the flume, with the water.
    WavePaddle,
    /// A loaded cart rolling along a track.
    Cart,
    /// Crane winch lifting or lowering a drop weight.
    Winch,
    /// Hydraulic power unit: electric motor and pump, idles when rigs are
    /// armed.
    PowerUnit,
}

/// Fixed hall noise sources.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum AmbientKind {
    /// Room tone: the hall's air, cladding ticks, far-off traffic.
    Room,
    /// Air handling: ducts, vents, a fan tone.
    Hvac,
    /// Mains hum from high-bay lamps and electrical cabinets.
    ElectricalHum,
    /// Distant workshop machinery: compressors cycling, a far clank.
    Machinery,
}

/// Interface sounds.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum UiSound {
    /// Cursor moved onto a new grid point or node.
    Hover,
    /// A node was placed.
    PlaceNode,
    /// A beam was placed.
    PlaceBeam,
    /// Something was deleted.
    Delete,
    /// Tool, material or plane changed.
    Select,
    /// A support was set or changed.
    Support,
    /// The action is not allowed.
    Invalid,
    /// Undo or redo.
    Undo,
    /// Test run starts (build -> simulate).
    TestStart,
    /// Test run stopped, back to the drawing board.
    TestStop,
    /// Structure passed / failed a scenario.
    Pass,
    Fail,
}

/// Everything the game can tell the audio engine. Commands are cheap; send
/// continuous ones (stress, rigs, ambience, listener) every frame or when
/// they change.
#[derive(Clone, Debug, PartialEq)]
pub enum Command {
    /// The camera: its position and unit right and up vectors.
    Listener { position: Pos, right: Pos, up: Pos },
    /// How hard the members of one material are working right now,
    /// aggregated by the game. `load` is the utilisation of the hardest
    /// working members (1 = at capacity, above 1 = past it; the bed is quiet
    /// below about 0.3), `rate` how fast it is rising per second, `members`
    /// how many members are above half capacity.
    /// The engine keeps one bed per material and smooths it.
    Stress {
        material: MaterialKind,
        load: f32,
        rate: f32,
        members: f32,
        place: Place,
    },
    /// A member reached a worse damage state (yielded, cracked, crushed,
    /// delaminated). `size` is the member's mass in kg.
    Damage {
        material: MaterialKind,
        state: DamageState,
        size: f32,
        place: Place,
    },
    /// A member broke.
    Break {
        material: MaterialKind,
        cause: BreakCause,
        size: f32,
        place: Place,
    },
    /// A loose piece hit the floor or the structure. `speed` m/s normal to
    /// the contact, `size` kg.
    Impact {
        material: MaterialKind,
        speed: f32,
        size: f32,
        place: Place,
    },
    /// A test rig's state. `drive` 0..1 is how hard it works; `speed` is its
    /// physical rate (fan wind speed in m/s, actuator / paddle / cart
    /// velocity in m/s, winch rope speed). Rigs not updated for half a
    /// second fade out.
    Rig {
        id: u32,
        kind: RigKind,
        drive: f32,
        speed: f32,
        place: Place,
    },
    /// Set the level (0..1) of an ambient source. `place: None` plays it as
    /// a wide, non-positional bed.
    Ambient {
        id: u32,
        kind: AmbientKind,
        level: f32,
        place: Option<Place>,
    },
    /// A one-shot interface sound.
    Ui(UiSound),
    /// Bus volume, linear 0..1.
    Volume { bus: Bus, gain: f32 },
    /// Master volume, linear 0..1.
    Master(f32),
    /// How damage, breaks and impacts are made: pure synthesis, or CC0
    /// recordings layered with synthesis.
    Style(SoundStyle),
    /// Stop all structure sounds (stress beds to zero, one-shots cut), e.g.
    /// when a test run is reset.
    ResetStructure,
}

/// How structure one-shots (damage, breaks, impacts) are made.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub enum SoundStyle {
    /// Everything synthesised.
    Synth,
    /// CC0 recordings for the transients, varied and layered with
    /// synthesised bodies and tails. See `recorded.rs` and `samples/`.
    #[default]
    Recorded,
}

/// Anything that renders sound. Voices are mono; the mixer positions them.
pub trait Voice: Send {
    /// Adds this voice's next `out.len()` samples into `out` (mono).
    /// Returns false once it has finished and can be dropped.
    fn render(&mut self, out: &mut [f32]) -> bool;
}

/// A boxed one-shot.
pub type BoxVoice = Box<dyn Voice>;
