//! The lab's materials. Each scenery material is one mesh, so the list here
//! is also the list of draw calls the hall costs.

use bevy::prelude::*;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum Mat {
    /// Floor slab.
    Concrete,
    /// Painted floor markings.
    FloorPaint,
    /// Steel strong-floor plates.
    FloorSteel,
    /// Primary steel: portal frames, rafters, crane runway.
    FrameSteel,
    /// Secondary steel: purlins, girts, bracing, brackets.
    LightSteel,
    /// Inside face of the wall and roof sheeting.
    Cladding,
    /// Daylight through roof lights and high windows.
    Skylight,
    /// Crane bridge and other safety-yellow plant.
    Yellow,
    /// Black of hazard stripes, rubber, motors.
    Dark,
    /// High-bay lamp housings.
    LampHousing,
    /// The glowing face of a high-bay lamp.
    LampGlow,
    /// Machinery enamel (pumps, power units, cabinets).
    MachineGreen,
    MachineGrey,
    MachineBlue,
    MachineOrange,
    /// Pipes by service colour.
    PipeGreen,
    PipeRed,
    PipeGrey,
    PipeBlue,
    /// Stainless tanks, chrome actuator rods.
    Stainless,
    /// Plywood, pallets, timber stock.
    Timber,
    /// Cardboard, crates.
    Cardboard,
    /// Indicator lamps on machinery (small, very bright: good bokeh).
    IndicatorGreen,
    IndicatorRed,
    IndicatorAmber,
    /// Warm-lit control room windows.
    WindowWarm,
    /// Flume and screen glass (blended).
    Glass,
    /// Water (blended).
    Water,
    // Play table.
    TableTop,
    TableFrame,
    GridMinor,
    GridMajor,
    /// Modelling foam for scenario terrain.
    Foam,
    /// Dark rubber and fasteners on the play table.
    TableDark,
}

/// True for materials drawn with alpha blending (sorted, no shadows).
pub fn blended(m: Mat) -> bool {
    matches!(m, Mat::Glass | Mat::Water)
}

fn srgb(hex: u32) -> Color {
    Color::srgb_u8((hex >> 16) as u8, (hex >> 8) as u8, hex as u8)
}

fn emissive(hex: u32, strength: f32) -> LinearRgba {
    srgb(hex).to_linear() * strength
}

pub fn material(m: Mat) -> StandardMaterial {
    let paint = |hex: u32, rough: f32| StandardMaterial {
        base_color: srgb(hex),
        perceptual_roughness: rough,
        reflectance: 0.4,
        ..default()
    };
    let metal = |hex: u32, rough: f32, metallic: f32| StandardMaterial {
        base_color: srgb(hex),
        perceptual_roughness: rough,
        metallic,
        ..default()
    };
    let glow = |hex: u32, strength: f32| StandardMaterial {
        base_color: srgb(hex),
        emissive: emissive(hex, strength),
        unlit: false,
        ..default()
    };
    match m {
        Mat::Concrete => paint(0x7b7d79, 0.85),
        Mat::FloorPaint => paint(0xd9ab30, 0.7),
        Mat::FloorSteel => metal(0x50575c, 0.55, 0.7),
        Mat::FrameSteel => paint(0x4f6475, 0.55),
        Mat::LightSteel => metal(0x8a949c, 0.5, 0.5),
        Mat::Cladding => paint(0xb9c1c6, 0.7),
        Mat::Skylight => glow(0xdfe9f2, 3.0),
        Mat::Yellow => paint(0xe0a526, 0.45),
        Mat::Dark => paint(0x1d2024, 0.6),
        Mat::LampHousing => metal(0x3a3f44, 0.4, 0.8),
        Mat::LampGlow => glow(0xfff1dc, 40.0),
        Mat::MachineGreen => paint(0x4c6b58, 0.4),
        Mat::MachineGrey => paint(0x9ca3a8, 0.45),
        Mat::MachineBlue => paint(0x2f5d8a, 0.4),
        Mat::MachineOrange => paint(0xd8682a, 0.45),
        Mat::PipeGreen => paint(0x3f7d4f, 0.5),
        Mat::PipeRed => paint(0xa8312a, 0.5),
        Mat::PipeGrey => metal(0x9aa0a4, 0.45, 0.5),
        Mat::PipeBlue => paint(0x3b6ea5, 0.5),
        Mat::Stainless => metal(0xc8ccd0, 0.25, 1.0),
        Mat::Timber => paint(0xb88c5a, 0.8),
        Mat::Cardboard => paint(0xa4825a, 0.9),
        Mat::IndicatorGreen => glow(0x52ff7a, 30.0),
        Mat::IndicatorRed => glow(0xff3b2e, 30.0),
        Mat::IndicatorAmber => glow(0xffb02e, 30.0),
        Mat::WindowWarm => glow(0xffd9a0, 4.0),
        Mat::Glass => StandardMaterial {
            base_color: Color::srgba(0.75, 0.85, 0.9, 0.18),
            perceptual_roughness: 0.05,
            reflectance: 0.6,
            alpha_mode: AlphaMode::Blend,
            ..default()
        },
        Mat::Water => StandardMaterial {
            base_color: Color::srgba(0.16, 0.36, 0.46, 0.7),
            perceptual_roughness: 0.08,
            reflectance: 0.5,
            alpha_mode: AlphaMode::Blend,
            ..default()
        },
        Mat::TableTop => paint(0x22282e, 0.7),
        Mat::TableFrame => metal(0x3b434b, 0.45, 0.6),
        Mat::GridMinor => StandardMaterial {
            base_color: srgb(0x4d6f8a),
            emissive: emissive(0x4d6f8a, 0.15),
            perceptual_roughness: 0.8,
            ..default()
        },
        Mat::GridMajor => StandardMaterial {
            base_color: srgb(0x86b4d8),
            emissive: emissive(0x86b4d8, 0.35),
            perceptual_roughness: 0.8,
            ..default()
        },
        Mat::Foam => paint(0xd6cfbf, 0.95),
        Mat::TableDark => paint(0x15181b, 0.7),
    }
}
