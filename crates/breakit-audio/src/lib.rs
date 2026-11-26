//! Procedural audio for BreakIt.
//!
//! Every sound is synthesised while the game runs. The structure's sounds
//! are physically based: the game hands over what the solver knows (a
//! member's material, section and length, how hard it is working, the energy
//! it released when it broke, how fast a piece hit the floor) and the sound
//! is computed from that, as force events ringing the member's own
//! vibration modes. The game describes *what is happening* with
//! [`Command`]s and the [`Mixer`] turns that into sound on the audio thread.
//!
//! * [`dsp`]: noise, filters, resonators, envelopes.
//! * [`body`]: a member's vibration modes from its geometry and material.
//! * [`modal`]: the modal bank and the voice that plays force events into
//!   it.
//! * [`material`]: the force events: stress release, micro-fracture,
//!   stick-slip and contact, for members straining, damage, breaks and
//!   debris impacts.
//! * [`ambience`]: the test hall (mains hum, HVAC, distant machinery).
//! * [`rigs`]: wind fan wall, shake table, wave paddle, carts, hydraulics.
//! * [`ui`]: build-mode clicks and mode changes.
//! * [`mixer`]: voices, buses, positioning, the hall reverb, the limiter.
//! * [`output`]: the real-time device stream (feature `device`) and a
//!   WAV recorder for headless runs.
//!
//! Like `breakit-core`, this crate has no engine dependency.

pub mod ambience;
pub mod body;
pub mod dsp;
pub mod listen;
pub mod material;
pub mod mixer;
pub mod modal;
pub mod output;
pub mod reverb;
pub mod rigs;
pub mod ui;
pub mod wav;

pub use body::{Body, Ends};
pub use breakit_core::{BreakCause, DamageState, MaterialKind};
pub use listen::StructureListener;
pub use material::BreakInfo;
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
    /// One member whose load or damage is changing. `id` is the member's
    /// index, `utilization` its load over capacity (1 = at capacity) and
    /// `damage` its fraction of the way to failure (0..1, never
    /// decreasing). Send it every frame for the members that are changing
    /// most; the engine keeps a voice per member, listens to the changes
    /// (joints slipping as the load moves, micro-cracks as damage grows) and
    /// drops voices that are no longer updated. A steady load is silent.
    Strain {
        id: u32,
        body: Body,
        utilization: f32,
        damage: f32,
        place: Place,
    },
    /// A member reached a worse damage state (yielded, cracked, crushed,
    /// delaminated). `energy` is the elastic energy it holds, J.
    Damage {
        body: Body,
        state: DamageState,
        energy: f32,
        place: Place,
    },
    /// A member broke.
    Break { info: BreakInfo, place: Place },
    /// A loose piece hit the floor or the structure. `speed` m/s normal to
    /// the contact, `mass` kg.
    Impact {
        body: Body,
        speed: f32,
        mass: f32,
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
    /// Stop all structure sounds (strain voices dropped, one-shots cut), e.g.
    /// when a test run is reset.
    ResetStructure,
}

/// Anything that renders sound. Voices are mono; the mixer positions them.
pub trait Voice: Send {
    /// Adds this voice's next `out.len()` samples into `out` (mono).
    /// Returns false once it has finished and can be dropped.
    fn render(&mut self, out: &mut [f32]) -> bool;
}

/// A boxed one-shot.
pub type BoxVoice = Box<dyn Voice>;
