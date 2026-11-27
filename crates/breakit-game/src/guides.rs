//! The sheet's build area, drawn in the world while building. The
//! foundations, the road and the equipment are drawn as things by
//! `site.rs`; the trucks by the load rigs (`rigs.rs`).

use bevy::prelude::*;
use breakit_scenario::PlayMode;

use crate::flow::Screen;
use crate::scenario::Sheets;

pub struct GuidesPlugin;

impl Plugin for GuidesPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(
            Update,
            draw_guides.run_if(in_state(Screen::Build).or_else(in_state(Screen::Test))),
        );
    }
}

fn v3(p: [f64; 3]) -> Vec3 {
    Vec3::new(p[0] as f32, p[1] as f32, p[2] as f32)
}

const GUIDE: Color = Color::srgba(0.88, 0.65, 0.15, 0.6);

fn draw_guides(mut gizmos: Gizmos, sheets: Res<Sheets>, screen: Res<State<Screen>>) {
    let sheet = sheets.current();
    let s = &sheet.def;
    let building = *screen.get() == Screen::Build;
    if sheet.mode == PlayMode::Sandbox {
        return;
    }
    if building {
        let (lo, hi) = (v3(s.limits.volume.min), v3(s.limits.volume.max));
        gizmos.cube(
            Transform::from_translation((lo + hi) * 0.5).with_scale(hi - lo),
            GUIDE,
        );
    }
}
