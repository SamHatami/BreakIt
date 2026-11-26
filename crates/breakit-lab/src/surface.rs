//! Procedural surfaces for the lab: `StandardMaterial` lighting with the
//! look of each material worked out in `surface.wesl`.
//!
//! Each [`Mat`] that has a look maps to a [`SurfaceKind`]. The shader lays
//! its patterns out in metres from the fragment's position, so the generated
//! meshes need no UVs and a wall can be any size without stretching.
//! Materials without a kind (glowing things, rubber) stay plain.

use bevy::asset::embedded_asset;
use bevy::pbr::{ExtendedMaterial, MaterialExtension, MaterialPlugin};
use bevy::prelude::*;
use bevy::render::render_resource::AsBindGroup;
use bevy::shader::ShaderRef;

use crate::palette::Mat;
use crate::LabSettings;

const SHADER_PATH: &str = "embedded://breakit_lab/surface.wesl";

pub type LabMaterial = ExtendedMaterial<StandardMaterial, LabSurface>;

/// Must match the `KIND_` constants in `surface.wesl`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SurfaceKind {
    /// Power-floated slab, precast panels, plinths.
    Concrete = 1,
    /// Enamel on steel and machinery: grime runs, dust, a few chips.
    Paint = 2,
    /// Zinc spangle.
    Galvanised = 3,
    /// Brushed stainless.
    Stainless = 4,
    /// Wall and roof sheets.
    Sheeting = 5,
    /// Modelling foam, laid out at model scale.
    Foam = 6,
    /// The dark mat of the play table, laid out at model scale.
    TableTop = 7,
    /// Plywood and sawn timber.
    Timber = 8,
    Glass = 9,
    Water = 10,
    /// Hot-rolled steel plate of the strong floor.
    Plate = 11,
}

/// The kind a scenery material is drawn with, if it has one.
pub fn kind_of(mat: Mat) -> Option<SurfaceKind> {
    use SurfaceKind as K;
    Some(match mat {
        Mat::Concrete => K::Concrete,
        Mat::FloorSteel => K::Plate,
        Mat::FloorPaint
        | Mat::FrameSteel
        | Mat::Yellow
        | Mat::MachineGreen
        | Mat::MachineGrey
        | Mat::MachineBlue
        | Mat::MachineOrange
        | Mat::PipeGreen
        | Mat::PipeRed
        | Mat::PipeBlue
        | Mat::TableFrame => K::Paint,
        Mat::LightSteel | Mat::PipeGrey => K::Galvanised,
        Mat::Stainless => K::Stainless,
        Mat::Cladding => K::Sheeting,
        Mat::Timber => K::Timber,
        Mat::Glass => K::Glass,
        Mat::Water => K::Water,
        Mat::TableTop => K::TableTop,
        Mat::Foam => K::Foam,
        Mat::Dark
        | Mat::LampHousing
        | Mat::Cardboard
        | Mat::TableDark
        | Mat::Skylight
        | Mat::LampGlow
        | Mat::IndicatorGreen
        | Mat::IndicatorRed
        | Mat::IndicatorAmber
        | Mat::WindowWarm
        | Mat::GridMinor
        | Mat::GridMajor => return None,
    })
}

/// Uniforms of `surface.wesl`, in the order of its `LabSurface` struct.
#[derive(Asset, AsBindGroup, Reflect, Debug, Clone, Default)]
pub struct LabSurface {
    /// xyz: world position of the pattern's origin; w: pattern metres per
    /// world unit.
    #[uniform(100)]
    pub frame: Vec4,
    /// x: kind, y: detail on (1) or off (0), z: seed.
    #[uniform(100)]
    pub params: Vec4,
    /// Water only: amplitude (m), wavelength (m), speed (m/s), heading
    /// (rad, 0 along +x).
    #[uniform(100)]
    pub water: Vec4,
}

impl MaterialExtension for LabSurface {
    fn fragment_shader() -> ShaderRef {
        SHADER_PATH.into()
    }
}

/// Waves on the play table's water, in world units (the structure's
/// metres). A wave or flood rig drives this; the default is a basin at rest
/// with a light ripple.
#[derive(Resource, Clone, Copy, Debug, PartialEq)]
pub struct LabWater {
    /// Wave amplitude, m.
    pub amplitude: f32,
    /// Length of the main wave train, m.
    pub wavelength: f32,
    /// Phase speed, m/s.
    pub speed: f32,
    /// Direction the waves travel, radians from +x towards +z.
    pub heading: f32,
}

impl Default for LabWater {
    fn default() -> Self {
        Self {
            amplitude: 0.012,
            wavelength: 1.4,
            speed: 0.9,
            heading: 0.0,
        }
    }
}

impl LabWater {
    fn uniform(&self) -> Vec4 {
        Vec4::new(self.amplitude, self.wavelength, self.speed, self.heading)
    }
}

/// Builds the material for `mat`. `on_table` is true for parts of the play
/// table, which sit at the world origin in world units; the hall is in lab
/// metres under a scaled root.
pub fn lab_material(
    mat: Mat,
    kind: SurfaceKind,
    on_table: bool,
    settings: &LabSettings,
    water: &LabWater,
) -> LabMaterial {
    let k = settings.model_scale;
    // Foam, the table top and the water in the basin are part of the model
    // and patterned in its metres. Everything else is real-size, 20 world
    // units to the metre, wherever it stands.
    let model = on_table
        && matches!(
            kind,
            SurfaceKind::Foam | SurfaceKind::TableTop | SurfaceKind::Water | SurfaceKind::Glass
        );
    let frame = if model {
        Vec4::new(0.0, 0.0, 0.0, 1.0)
    } else {
        (-settings.lab_origin() * k).extend(1.0 / k)
    };
    let detail = if settings.look.surface_detail {
        1.0
    } else {
        0.0
    };
    ExtendedMaterial {
        base: crate::palette::material(mat),
        extension: LabSurface {
            frame,
            params: Vec4::new(kind as u32 as f32, detail, mat as u32 as f32 * 0.37, 0.0),
            water: if on_table {
                water.uniform()
            } else {
                // The hall's flume: a real basin, barely moving.
                Vec4::new(0.006, 0.5, 0.4, 0.3)
            },
        },
    }
}

pub(crate) fn build(app: &mut App) {
    embedded_asset!(app, "surface.wesl");
    app.add_plugins(MaterialPlugin::<LabMaterial>::default())
        .init_resource::<LabWater>()
        .add_systems(Update, sync_surfaces);
}

/// Pushes the detail switch and the table's waves into the materials when
/// either changes.
fn sync_surfaces(
    settings: Res<LabSettings>,
    water: Res<LabWater>,
    mut materials: ResMut<Assets<LabMaterial>>,
    mut last: Local<Option<(bool, LabWater)>>,
) {
    let want = (settings.look.surface_detail, *water);
    if *last == Some(want) {
        return;
    }
    let first = last.is_none();
    *last = Some(want);
    if first {
        return;
    }
    let ids: Vec<_> = materials.ids().collect();
    for id in ids {
        let Some(mut m) = materials.get_mut(id) else {
            continue;
        };
        m.extension.params.y = if want.0 { 1.0 } else { 0.0 };
        let table_water =
            m.extension.params.x == SurfaceKind::Water as u32 as f32 && m.extension.frame.w == 1.0;
        if table_water {
            m.extension.water = water.uniform();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn model_materials_are_patterned_in_model_metres() {
        let s = LabSettings::default();
        let w = LabWater::default();
        let foam = lab_material(Mat::Foam, SurfaceKind::Foam, true, &s, &w);
        assert_eq!(foam.extension.frame, Vec4::new(0.0, 0.0, 0.0, 1.0));
        // The table's frame is furniture: real metres, like the hall.
        let frame = lab_material(Mat::TableFrame, SurfaceKind::Paint, true, &s, &w);
        assert!((frame.extension.frame.w - 1.0 / s.model_scale).abs() < 1e-6);
        // The pattern origin is the hall's own origin, so patterns line up
        // with the saw cuts in the floor.
        let floor = lab_material(Mat::Concrete, SurfaceKind::Concrete, false, &s, &w);
        let o = floor.extension.frame.truncate();
        assert!((o - s.lab_to_world(Vec3::ZERO)).length() < 1e-3);
    }

    #[test]
    fn blended_materials_keep_their_blend_mode() {
        let s = LabSettings::default();
        let w = LabWater::default();
        for mat in [Mat::Glass, Mat::Water] {
            let m = lab_material(mat, kind_of(mat).unwrap(), true, &s, &w);
            assert!(matches!(m.base.alpha_mode, AlphaMode::Blend));
            assert!(crate::palette::blended(mat));
        }
    }
}
