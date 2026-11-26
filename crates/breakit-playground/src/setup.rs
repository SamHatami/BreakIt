//! What the player has dialled in on the load bench: one block of settings
//! per load model, gravity, and point loads. Pure data, saved with a
//! playground file; [`crate::Live`] turns it into `breakit_core::loads`
//! cases while the simulation runs.
//!
//! Numbers change in steps ([`Param::step`]) so a mouse click or a key
//! press always lands on a round value.

use serde::{Deserialize, Serialize};

/// Everything on the load bench.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct Setup {
    pub wind: WindSetup,
    pub quake: QuakeSetup,
    pub water: WaterSetup,
    pub traffic: TrafficSetup,
    pub drop: DropSetup,
    /// Gravity as a multiple of standard gravity (1 = Earth).
    pub gravity: f64,
    /// Size of the next point load placed by clicking a node, N (downward).
    pub point_load: f64,
    /// Point loads placed by clicking, world frame.
    pub point_loads: Vec<PointLoad>,
}

impl Default for Setup {
    fn default() -> Self {
        Self {
            wind: WindSetup::default(),
            quake: QuakeSetup::default(),
            water: WaterSetup::default(),
            traffic: TrafficSetup::default(),
            drop: DropSetup::default(),
            gravity: 1.0,
            point_load: 50_000.0,
            point_loads: Vec::new(),
        }
    }
}

/// Wind from a fan wall.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct WindSetup {
    pub on: bool,
    /// Mean speed, m/s.
    pub speed: f64,
    /// Direction the wind blows towards, degrees from +x towards +z.
    pub heading: f64,
    pub gusts: bool,
    /// Turbulence intensity σ_u / U (0.1 to 0.2 outdoors).
    pub turbulence: f64,
}

impl Default for WindSetup {
    fn default() -> Self {
        Self {
            on: false,
            speed: 25.0,
            heading: 90.0,
            gusts: true,
            turbulence: 0.15,
        }
    }
}

/// Which ground motion the shake table plays.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum QuakeMotion {
    /// Steady sine at the set frequency.
    #[default]
    Harmonic,
    /// Kanai-Tajimi synthetic earthquake (20 s strong motion), soil
    /// frequency = the set frequency.
    Synthetic,
    /// Sine sweep from 0.5 Hz up to the set frequency over 30 s, to find
    /// the structure's natural frequencies.
    Sweep,
}

impl QuakeMotion {
    pub fn next(self) -> Self {
        match self {
            QuakeMotion::Harmonic => QuakeMotion::Synthetic,
            QuakeMotion::Synthetic => QuakeMotion::Sweep,
            QuakeMotion::Sweep => QuakeMotion::Harmonic,
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            QuakeMotion::Harmonic => "sine",
            QuakeMotion::Synthetic => "earthquake",
            QuakeMotion::Sweep => "sweep",
        }
    }
}

/// Shake table.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct QuakeSetup {
    pub on: bool,
    pub motion: QuakeMotion,
    /// Peak table acceleration, g.
    pub amplitude: f64,
    /// Sine frequency, soil frequency or top of the sweep, Hz.
    pub frequency: f64,
    /// Shaking direction, degrees from +x towards +z.
    pub heading: f64,
    /// Seed of the synthetic record; bumped by "new record".
    pub seed: u64,
}

impl Default for QuakeSetup {
    fn default() -> Self {
        Self {
            on: false,
            motion: QuakeMotion::Harmonic,
            amplitude: 0.3,
            frequency: 2.0,
            heading: 0.0,
            seed: 1,
        }
    }
}

/// Waves on the water.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum WaveKind {
    #[default]
    Calm,
    /// Regular Airy waves.
    Regular,
    /// One solitary wave from the paddle, sent when switched on or changed.
    Surge,
}

impl WaveKind {
    pub fn next(self) -> Self {
        match self {
            WaveKind::Calm => WaveKind::Regular,
            WaveKind::Regular => WaveKind::Surge,
            WaveKind::Surge => WaveKind::Calm,
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            WaveKind::Calm => "calm",
            WaveKind::Regular => "regular",
            WaveKind::Surge => "surge",
        }
    }
}

/// Water in the basin or flume.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct WaterSetup {
    pub on: bool,
    /// Still water level, absolute y, m.
    pub level: f64,
    /// Current speed along the heading, m/s.
    pub current: f64,
    pub waves: WaveKind,
    /// Wave height crest to trough (surge: crest above still water), m.
    pub height: f64,
    /// Wave period, s.
    pub period: f64,
}

impl Default for WaterSetup {
    fn default() -> Self {
        Self {
            on: false,
            level: -1.0,
            current: 0.5,
            waves: WaveKind::Calm,
            height: 0.8,
            period: 3.0,
        }
    }
}

/// The next truck sent across.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct TrafficSetup {
    /// Gross mass, t.
    pub mass: f64,
    /// Speed, m/s.
    pub speed: f64,
    /// Number of axles, 2 to 5, 1.4 m apart.
    pub axles: u8,
}

impl Default for TrafficSetup {
    fn default() -> Self {
        Self {
            mass: 20.0,
            speed: 8.0,
            axles: 3,
        }
    }
}

/// The next weight dropped from the crane.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct DropSetup {
    /// kg.
    pub mass: f64,
    /// Release height above the target, m.
    pub height: f64,
}

impl Default for DropSetup {
    fn default() -> Self {
        Self {
            mass: 2_000.0,
            height: 4.0,
        }
    }
}

/// A force placed on a node by clicking it.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct PointLoad {
    pub node: usize,
    /// N, world frame.
    pub force: [f64; 3],
}

/// A number on the bench that steps up and down.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Param {
    WindSpeed,
    WindHeading,
    Turbulence,
    QuakeAmplitude,
    QuakeFrequency,
    QuakeHeading,
    WaterLevel,
    Current,
    WaveHeight,
    WavePeriod,
    TruckMass,
    TruckSpeed,
    TruckAxles,
    DropMass,
    DropHeight,
    Gravity,
    PointLoad,
}

impl Param {
    /// The values it steps through, in its unit (see [`Self::unit`]).
    pub fn steps(self) -> &'static [f64] {
        match self {
            Param::WindSpeed => &[
                0.0, 5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0, 50.0, 60.0, 70.0, 80.0,
            ],
            Param::WindHeading | Param::QuakeHeading => {
                &[-135.0, -90.0, -45.0, 0.0, 45.0, 90.0, 135.0, 180.0]
            }
            Param::Turbulence => &[0.05, 0.1, 0.15, 0.2, 0.25, 0.3],
            Param::QuakeAmplitude => &[
                0.05, 0.1, 0.15, 0.2, 0.3, 0.4, 0.5, 0.6, 0.8, 1.0, 1.2, 1.5, 2.0,
            ],
            Param::QuakeFrequency => {
                &[0.5, 0.75, 1.0, 1.5, 2.0, 2.5, 3.0, 4.0, 5.0, 6.0, 8.0, 10.0]
            }
            Param::WaterLevel => &[
                -3.5, -3.0, -2.5, -2.0, -1.5, -1.0, -0.5, 0.0, 0.5, 1.0, 1.5, 2.0, 3.0,
            ],
            Param::Current => &[0.0, 0.25, 0.5, 1.0, 1.5, 2.0, 3.0, 4.0],
            Param::WaveHeight => &[0.2, 0.4, 0.6, 0.8, 1.0, 1.2, 1.5, 1.8, 2.0, 2.5],
            Param::WavePeriod => &[1.5, 2.0, 2.5, 3.0, 4.0, 5.0, 6.0, 8.0],
            Param::TruckMass => &[2.0, 5.0, 10.0, 15.0, 20.0, 30.0, 40.0, 60.0, 80.0],
            Param::TruckSpeed => &[1.0, 2.0, 4.0, 6.0, 8.0, 12.0, 16.0, 20.0, 25.0],
            Param::TruckAxles => &[2.0, 3.0, 4.0, 5.0],
            Param::DropMass => &[
                100.0, 250.0, 500.0, 1_000.0, 2_000.0, 5_000.0, 10_000.0, 20_000.0,
            ],
            Param::DropHeight => &[0.5, 1.0, 2.0, 3.0, 4.0, 6.0, 8.0, 10.0],
            Param::Gravity => &[0.0, 0.25, 0.5, 1.0, 1.5, 2.0, 3.0, 5.0],
            Param::PointLoad => &[
                1_000.0, 5_000.0, 10_000.0, 20_000.0, 50_000.0, 100_000.0, 200_000.0, 500_000.0,
            ],
        }
    }

    pub fn get(self, s: &Setup) -> f64 {
        match self {
            Param::WindSpeed => s.wind.speed,
            Param::WindHeading => s.wind.heading,
            Param::Turbulence => s.wind.turbulence,
            Param::QuakeAmplitude => s.quake.amplitude,
            Param::QuakeFrequency => s.quake.frequency,
            Param::QuakeHeading => s.quake.heading,
            Param::WaterLevel => s.water.level,
            Param::Current => s.water.current,
            Param::WaveHeight => s.water.height,
            Param::WavePeriod => s.water.period,
            Param::TruckMass => s.traffic.mass,
            Param::TruckSpeed => s.traffic.speed,
            Param::TruckAxles => s.traffic.axles as f64,
            Param::DropMass => s.drop.mass,
            Param::DropHeight => s.drop.height,
            Param::Gravity => s.gravity,
            Param::PointLoad => s.point_load,
        }
    }

    pub fn set(self, s: &mut Setup, v: f64) {
        match self {
            Param::WindSpeed => s.wind.speed = v,
            Param::WindHeading => s.wind.heading = v,
            Param::Turbulence => s.wind.turbulence = v,
            Param::QuakeAmplitude => s.quake.amplitude = v,
            Param::QuakeFrequency => s.quake.frequency = v,
            Param::QuakeHeading => s.quake.heading = v,
            Param::WaterLevel => s.water.level = v,
            Param::Current => s.water.current = v,
            Param::WaveHeight => s.water.height = v,
            Param::WavePeriod => s.water.period = v,
            Param::TruckMass => s.traffic.mass = v,
            Param::TruckSpeed => s.traffic.speed = v,
            Param::TruckAxles => s.traffic.axles = v.round().clamp(2.0, 5.0) as u8,
            Param::DropMass => s.drop.mass = v,
            Param::DropHeight => s.drop.height = v,
            Param::Gravity => s.gravity = v,
            Param::PointLoad => s.point_load = v,
        }
    }

    /// Moves to the next step up (`dir > 0`) or down. Headings wrap round;
    /// everything else stops at the ends. Returns whether the value changed.
    pub fn step(self, s: &mut Setup, dir: i8) -> bool {
        let steps = self.steps();
        let v = self.get(s);
        if matches!(self, Param::WindHeading | Param::QuakeHeading) {
            // Nearest heading round the circle, then the next one.
            let gap = |x: f64| ((x - v).rem_euclid(360.0)).min((v - x).rem_euclid(360.0));
            let i = (0..steps.len())
                .min_by(|&a, &b| gap(steps[a]).total_cmp(&gap(steps[b])))
                .unwrap_or(0);
            let n = steps.len();
            let j = if dir > 0 {
                (i + 1) % n
            } else {
                (i + n - 1) % n
            };
            self.set(s, steps[j]);
            return true;
        }
        let eps = 1e-9 * (1.0 + v.abs());
        let next = if dir > 0 {
            steps.iter().copied().find(|&x| x > v + eps)
        } else {
            steps.iter().copied().rev().find(|&x| x < v - eps)
        };
        match next {
            Some(x) => {
                self.set(s, x);
                true
            }
            None => false,
        }
    }

    pub fn unit(self) -> &'static str {
        match self {
            Param::WindSpeed | Param::Current | Param::TruckSpeed => "m/s",
            Param::WindHeading | Param::QuakeHeading => "deg",
            Param::Turbulence | Param::TruckAxles => "",
            Param::QuakeAmplitude | Param::Gravity => "g",
            Param::QuakeFrequency => "Hz",
            Param::WaterLevel | Param::WaveHeight | Param::DropHeight => "m",
            Param::WavePeriod => "s",
            Param::TruckMass => "t",
            Param::DropMass => "kg",
            Param::PointLoad => "kN",
        }
    }

    /// The value with its unit, for the panel.
    pub fn display(self, s: &Setup) -> String {
        let v = self.get(s);
        match self {
            Param::Turbulence => format!("{:.0} %", v * 100.0),
            Param::TruckAxles => format!("{v:.0}"),
            Param::PointLoad => format!("{:.0} kN", v / 1000.0),
            Param::DropMass if v >= 1000.0 => format!("{:.0} t", v / 1000.0),
            Param::QuakeAmplitude | Param::Gravity | Param::Current | Param::WaterLevel => {
                format!("{v:.2} {}", self.unit())
            }
            Param::QuakeFrequency | Param::WaveHeight | Param::WavePeriod | Param::DropHeight => {
                format!("{v:.1} {}", self.unit())
            }
            _ => format!("{v:.0} {}", self.unit()),
        }
    }
}

/// Unit horizontal vector for a heading in degrees from +x towards +z.
pub fn heading_vector(deg: f64) -> breakit_core::Vec3 {
    let r = deg.to_radians();
    breakit_core::Vec3::new(r.cos(), 0.0, r.sin())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn params_step_to_round_values_and_stop_at_the_ends() {
        let mut s = Setup::default();
        assert!(Param::WindSpeed.step(&mut s, 1));
        assert_eq!(s.wind.speed, 30.0);
        s.wind.speed = 27.3;
        Param::WindSpeed.step(&mut s, -1);
        assert_eq!(s.wind.speed, 25.0);
        s.wind.speed = 80.0;
        assert!(!Param::WindSpeed.step(&mut s, 1));
        assert_eq!(s.wind.speed, 80.0);
        s.gravity = 0.0;
        assert!(!Param::Gravity.step(&mut s, -1));
    }

    #[test]
    fn headings_wrap_round() {
        let mut s = Setup::default();
        s.wind.heading = 135.0;
        Param::WindHeading.step(&mut s, 1);
        assert_eq!(s.wind.heading, 180.0);
        Param::WindHeading.step(&mut s, 1);
        assert_eq!(s.wind.heading, -135.0);
        Param::WindHeading.step(&mut s, -1);
        assert_eq!(s.wind.heading, 180.0);
        s.wind.heading = -180.0;
        Param::WindHeading.step(&mut s, -1);
        assert_eq!(s.wind.heading, 135.0);
        let v = heading_vector(90.0);
        assert!((v.z - 1.0).abs() < 1e-12 && v.x.abs() < 1e-12);
    }

    #[test]
    fn axles_stay_whole() {
        let mut s = Setup::default();
        Param::TruckAxles.set(&mut s, 7.4);
        assert_eq!(s.traffic.axles, 5);
        assert_eq!(Param::TruckAxles.display(&s), "5");
        assert_eq!(Param::PointLoad.display(&s), "50 kN");
    }
}
