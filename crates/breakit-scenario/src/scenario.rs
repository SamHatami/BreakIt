//! The scenario definition: what the player must build and what the test
//! does to it. Scenarios are data, loaded from RON files (see
//! `scenarios/*.ron` and [`crate::builtin`]).
//!
//! Positions are world coordinates in metres, Y up. Loads are in newtons or
//! kilograms, times in seconds from the start of the test.

use serde::{Deserialize, Serialize};

use breakit_core::MaterialKind;

/// One test brief.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Scenario {
    /// Stable id, used for save files and unlocks.
    pub id: String,
    /// Test sheet number shown on the register, e.g. "B-02".
    pub sheet: String,
    /// Register tab, e.g. "Bridges".
    pub group: String,
    pub title: String,
    /// The brief the player reads before building.
    pub brief: String,
    pub limits: Limits,
    /// The only places the structure may be supported.
    pub anchors: Vec<Anchor>,
    /// The lane vehicles and deck loads run along, for bridges.
    #[serde(default)]
    pub deck: Option<Deck>,
    /// Named points the structure must carry something at.
    #[serde(default)]
    pub payloads: Vec<Payload>,
    /// What happens during the test, in time order or not.
    pub events: Vec<Event>,
    /// Seconds the structure must survive.
    pub duration: f64,
    /// Seconds over which self weight is ramped in before the clock starts.
    /// Without it gravity hits all at once and overshoots to twice its
    /// static effect.
    #[serde(default = "default_settle")]
    pub settle: f64,
    #[serde(default)]
    pub collapse: CollapseRule,
    /// Proof test to destruction after a pass, for the reserve and
    /// graceful-failure read-outs.
    #[serde(default)]
    pub overload: Option<Overload>,
    #[serde(default)]
    pub objectives: Vec<Objective>,
    pub scoring: Scoring,
}

fn default_settle() -> f64 {
    1.0
}

/// Hard limits. Breaking any of these blocks the test (except in sandbox).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Limits {
    /// Every node must lie inside this box.
    pub volume: Aabb,
    /// Most the structure may cost. The test cannot run above it.
    #[serde(default)]
    pub budget: Option<f64>,
    /// Most the structure may weigh, kg.
    #[serde(default)]
    pub max_mass: Option<f64>,
    /// Materials the player may use. `None` allows all.
    #[serde(default)]
    pub materials: Option<Vec<MaterialClass>>,
}

/// Axis-aligned box, metres.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Aabb {
    pub min: [f64; 3],
    pub max: [f64; 3],
}

impl Aabb {
    pub fn contains(&self, p: [f64; 3], tol: f64) -> bool {
        (0..3).all(|i| p[i] >= self.min[i] - tol && p[i] <= self.max[i] + tol)
    }
}

/// Serialisable mirror of the core [`MaterialKind`].
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum MaterialClass {
    Steel,
    Timber,
    Concrete,
    Composite,
}

impl From<MaterialKind> for MaterialClass {
    fn from(k: MaterialKind) -> Self {
        match k {
            MaterialKind::Steel => MaterialClass::Steel,
            MaterialKind::Timber => MaterialClass::Timber,
            MaterialKind::Concrete => MaterialClass::Concrete,
            MaterialKind::Composite => MaterialClass::Composite,
        }
    }
}

impl MaterialClass {
    pub fn label(self) -> &'static str {
        match self {
            MaterialClass::Steel => "steel",
            MaterialClass::Timber => "timber",
            MaterialClass::Concrete => "concrete",
            MaterialClass::Composite => "composite",
        }
    }
}

/// A foundation point the structure may be supported at.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Anchor {
    pub name: String,
    pub position: [f64; 3],
    /// The most restraint this foundation can give.
    pub kind: AnchorKind,
    /// How far a support may sit from the anchor, metres.
    #[serde(default = "default_anchor_tol")]
    pub tolerance: f64,
}

fn default_anchor_tol() -> f64 {
    0.05
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum AnchorKind {
    /// Holds position only: pinned supports.
    Pinned,
    /// Holds position and rotation: pinned or fixed supports.
    Fixed,
}

/// A straight, level traffic lane. Nodes within `width` sideways and
/// `height` up or down of the line from `from` to `to` are deck nodes;
/// vehicles and deck loads go onto them.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Deck {
    pub from: [f64; 3],
    pub to: [f64; 3],
    /// Largest sideways distance from the lane centre line for a deck
    /// node, metres.
    pub width: f64,
    /// Largest vertical distance from the lane for a deck node, metres.
    #[serde(default = "default_deck_height")]
    pub height: f64,
    /// Largest gap between deck nodes along the lane (and at either end)
    /// before a wheel falls through, metres.
    pub max_gap: f64,
}

/// Equipment the structure must carry: a tank, a pump skid, a rooftop unit.
///
/// It sits on the player's nodes at its feet. Its mass is shared between
/// the feet as point masses, so it has weight (ramped in with self weight)
/// and inertia when shaken, and its centre of mass `height` above the feet
/// turns sideways inertia into an overturning couple on them. A resting or
/// bolted item can tip, slide or tear its bolts and come off.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Payload {
    pub name: String,
    /// Centre of the footprint, where it sits on the structure.
    pub at: [f64; 3],
    /// How far each carrying node may be from its foot, metres.
    #[serde(default = "default_payload_tol")]
    pub tolerance: f64,
    /// kg.
    pub mass: f64,
    /// Feet as offsets from `at`. Empty: one foot at `at`.
    #[serde(default)]
    pub feet: Vec<[f64; 3]>,
    /// Centre of mass above the feet, m.
    #[serde(default)]
    pub height: f64,
    /// Drawn size (x, y, z), m.
    #[serde(default)]
    pub size: Option<[f64; 3]>,
    #[serde(default)]
    pub mount: Mount,
    #[serde(default)]
    pub limits: PayloadLimits,
    /// Losing it fails the test.
    #[serde(default)]
    pub critical: bool,
    /// Drag coefficient on its `size` box facing the wind (1.8 for a
    /// sign board, about 1.2 for a cabinet). No `size`, no wind on it.
    #[serde(default = "default_drag")]
    pub drag: f64,
}

fn default_drag() -> f64 {
    1.2
}

impl Payload {
    /// Area of its `size` box facing a horizontal wind along `dir`, m².
    pub fn wind_area(&self, dir: [f64; 3]) -> f64 {
        let Some([sx, sy, sz]) = self.size else {
            return 0.0;
        };
        let (dx, dz) = (dir[0], dir[2]);
        let n = (dx * dx + dz * dz).sqrt();
        if n < 1e-12 {
            return 0.0;
        }
        sy * ((dx / n).abs() * sz + (dz / n).abs() * sx)
    }

    /// Rough contact radius, m: half the largest drawn size, else half the
    /// footprint span, else 0.3 m.
    pub fn radius(&self) -> f64 {
        if let Some(sz) = self.size {
            return 0.5 * sz.iter().copied().fold(0.0, f64::max);
        }
        let span = self
            .feet
            .iter()
            .map(|f| (f[0] * f[0] + f[2] * f[2]).sqrt())
            .fold(0.0, f64::max);
        if span > 0.0 {
            span
        } else {
            0.3
        }
    }

    /// Foot positions in the world, as built.
    pub fn foot_positions(&self) -> Vec<[f64; 3]> {
        if self.feet.is_empty() {
            return vec![self.at];
        }
        self.feet
            .iter()
            .map(|f| [self.at[0] + f[0], self.at[1] + f[1], self.at[2] + f[2]])
            .collect()
    }
}

/// How the equipment is held on.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub enum Mount {
    /// Welded or cast in: never comes off.
    #[default]
    Fixed,
    /// Bolted down; each foot's bolts hold `capacity` N in pull-out or
    /// shear, after which it rests on friction 0.3.
    Bolted { capacity: f64 },
    /// Just standing there. Tips when a foot lifts, slides when sideways
    /// inertia beats `friction` times the weight on its feet.
    Resting { friction: f64 },
}

/// Tolerances for the equipment. Each one set becomes a report row; with
/// `strict` a breach fails the test, otherwise it costs score.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct PayloadLimits {
    /// Largest tilt of its footprint, degrees.
    #[serde(default)]
    pub tilt_deg: Option<f64>,
    /// Largest downward deflection under its feet, m.
    #[serde(default)]
    pub sag: Option<f64>,
    /// Largest movement of its footprint centre, m.
    #[serde(default)]
    pub drift: Option<f64>,
    #[serde(default)]
    pub strict: bool,
}

fn default_deck_height() -> f64 {
    0.25
}

fn default_payload_tol() -> f64 {
    0.3
}

fn default_ramp() -> f64 {
    1.0
}

/// Things the test does over time.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum Event {
    /// Vehicles drive along the deck one after another.
    Convoy {
        name: String,
        start: f64,
        /// m/s.
        speed: f64,
        count: usize,
        /// Front-to-front distance between vehicles, metres.
        spacing: f64,
        axles: Vec<Axle>,
    },
    /// Uniform load along the whole deck: crowds, snow, ballast. N/m.
    DeckLoad {
        name: String,
        start: f64,
        ramp: f64,
        per_metre: f64,
    },
    /// Horizontal ground acceleration a(t) = peak * sin(2 pi f t), with
    /// linear fade in and out. Every free node feels -m a.
    GroundShake {
        name: String,
        start: f64,
        duration: f64,
        /// Peak ground acceleration, m/s^2.
        peak: f64,
        /// Hz.
        frequency: f64,
        /// Shake direction (normalised on use).
        direction: [f64; 3],
        /// Fade in / out time, s.
        #[serde(default = "default_ramp")]
        fade: f64,
    },
    /// The fan wall blows: drag on every member (core `Wind`, with gusts,
    /// shielding and vortex shedding) and on payloads with a `size`.
    Wind {
        name: String,
        start: f64,
        /// s; `None` blows to the end of the test.
        #[serde(default)]
        duration: Option<f64>,
        /// Spin-up and spin-down time, s.
        #[serde(default = "default_ramp")]
        ramp: f64,
        /// Mean speed in the jet core, m/s.
        speed: f64,
        /// Direction the wind blows towards (horizontal, normalised on
        /// use). The fan wall stands upstream of the table.
        direction: [f64; 3],
        /// Along-wind turbulence intensity σ/U; 0 is a steady wind.
        #[serde(default)]
        gusts: f64,
        /// The fan wall's jet: centre and half sizes across and up, m.
        /// `None` is a uniform wind over the whole table.
        #[serde(default)]
        jet: Option<Jet>,
    },
    /// Water in the flume (core `Water`): the level, a current and a wave.
    /// The flume bed is the lowest anchor's level.
    Water {
        name: String,
        start: f64,
        #[serde(default)]
        duration: Option<f64>,
        /// Fill / drain ramp, s.
        #[serde(default = "default_ramp")]
        ramp: f64,
        /// Flume bed, absolute y, m.
        bed: f64,
        /// Still water level, absolute y, m.
        level: f64,
        /// Current, m/s (horizontal).
        #[serde(default)]
        current: [f64; 3],
        #[serde(default)]
        wave: Option<Wave>,
        /// Rest position of the paddle along the wave direction, m.
        #[serde(default = "default_paddle")]
        paddle: f64,
    },
}

fn default_paddle() -> f64 {
    -13.6
}

/// The fan wall's jet.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Jet {
    pub center: [f64; 3],
    pub half_width: f64,
    pub half_height: f64,
}

/// A wave made by the flume's paddle.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum Wave {
    /// One solitary wave (a tsunami-like surge) of crest `height` above
    /// still water, m, travelling along `direction` from the paddle.
    Surge { height: f64, direction: [f64; 3] },
    /// Regular waves, crest to trough `height` m, `period` s.
    Regular {
        height: f64,
        period: f64,
        direction: [f64; 3],
    },
}

impl Event {
    pub fn name(&self) -> &str {
        match self {
            Event::Convoy { name, .. }
            | Event::DeckLoad { name, .. }
            | Event::GroundShake { name, .. }
            | Event::Wind { name, .. }
            | Event::Water { name, .. } => name,
        }
    }
}

/// One axle of a vehicle, `offset` metres behind the front.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Axle {
    pub offset: f64,
    /// N.
    pub load: f64,
}

/// When the structure counts as collapsed: a watched node (deck nodes and
/// payload nodes, or all free nodes if there are none) has moved further
/// than `displacement` from where it was built, or more than
/// `max_broken_fraction` of its members broke.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct CollapseRule {
    pub displacement: f64,
    pub max_broken_fraction: f64,
}

impl Default for CollapseRule {
    fn default() -> Self {
        Self {
            displacement: 0.5,
            max_broken_fraction: 0.25,
        }
    }
}

/// Proof test after a pass: every scenario load (not self weight) scales
/// up by `rate` of its design value per second until collapse or
/// `max_factor`.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Overload {
    pub rate: f64,
    pub max_factor: f64,
    /// Traffic stand-in during the proof test: a uniform deck load of this
    /// many N/m at factor 1, since vehicles have driven off by then.
    #[serde(default)]
    pub deck_per_metre: Option<f64>,
}

/// Bonus objectives. Each one met adds to the score.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum Objective {
    /// Deck sag stays under `limit` metres (span / 300 is a usual code
    /// limit).
    DeckDeflection { limit: f64 },
    /// The named payload moves less than `limit` metres.
    PayloadDrift { payload: String, limit: f64 },
    /// No member breaks during the test.
    NoBreaks,
    /// Under the proof test, at least `warning` seconds pass between first
    /// visible damage (yield, cracking) and collapse.
    GracefulFailure { warning: f64 },
    /// The proof test reaches at least this load factor before collapse.
    Reserve { factor: f64 },
}

/// How a pass is graded.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Scoring {
    /// Cost at or below this scores full marks.
    pub target_cost: f64,
    /// Mass at or below this scores full marks, kg.
    pub target_mass: f64,
    /// Safety margin (1 / peak utilisation) in this band scores full
    /// marks. Below it is risky, above it is overbuilt.
    pub margin_band: [f64; 2],
    pub weights: Weights,
    /// Score fraction (0..1) needed for two and three stars. One star is a
    /// pass.
    pub stars: [f64; 2],
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Weights {
    pub cost: f64,
    pub mass: f64,
    pub margin: f64,
    pub objectives: f64,
}

/// Parse errors from [`Scenario::from_ron`].
pub type ParseError = ron::error::SpannedError;

impl Scenario {
    pub fn from_ron(text: &str) -> Result<Self, ParseError> {
        ron::from_str(text)
    }

    pub fn to_ron(&self) -> String {
        ron::ser::to_string_pretty(self, ron::ser::PrettyConfig::default())
            .expect("scenario always serialises")
    }

    pub fn anchor(&self, name: &str) -> Option<&Anchor> {
        self.anchors.iter().find(|a| a.name == name)
    }
}
