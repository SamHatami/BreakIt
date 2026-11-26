//! The sheet's guides drawn in the world: build area, foundations, the
//! traffic lane, payloads and, during a test, the vehicles on the deck.
//! Gizmos for now, until the play table and props draw them.

use bevy::prelude::*;
use breakit_scenario::PlayMode;

use crate::analysis::Simulation;
use crate::design::from_core;
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
const ANCHOR: Color = Color::srgb(0.88, 0.65, 0.15);
const LANE: Color = Color::srgba(0.9, 0.9, 0.9, 0.5);
const VEHICLE: Color = Color::srgb(0.85, 0.3, 0.2);

fn draw_guides(
    mut gizmos: Gizmos,
    sheets: Res<Sheets>,
    screen: Res<State<Screen>>,
    sim: Option<Res<Simulation>>,
) {
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
    for a in &s.anchors {
        let p = v3(a.position);
        gizmos.sphere(Isometry3d::from_translation(p), 0.12, ANCHOR);
        gizmos.line(p, p - Vec3::Y * 0.3, ANCHOR);
    }
    if let Some(d) = &s.deck {
        gizmos.line(v3(d.from), v3(d.to), LANE);
    }
    for p in &s.payloads {
        let at = v3(p.at);
        gizmos.cube(
            Transform::from_translation(at + Vec3::Y * 0.4).with_scale(Vec3::splat(0.8)),
            if building { GUIDE } else { VEHICLE },
        );
    }
    // Vehicles ride on the deflected deck: the rig's wheel contacts already
    // follow it.
    let Some(sim) = sim.filter(|_| !building) else {
        return;
    };
    let Some(run) = sim.run() else { return };
    for v in run.vehicles() {
        let length = match (v.wheels.first(), v.wheels.last()) {
            (Some(a), Some(b)) => (a.contact - b.contact).norm() as f32,
            _ => 0.0,
        };
        let dir = from_core(v.forward);
        let centre = from_core(v.position) + Vec3::Y * 0.6;
        gizmos.cube(
            Transform::from_translation(centre)
                .looking_to(dir, Vec3::Y)
                .with_scale(Vec3::new(1.6, 1.2, length + 1.0)),
            VEHICLE,
        );
    }
}
