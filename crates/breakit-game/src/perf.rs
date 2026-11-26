//! Frame-time read-out, top edge between the sheet and the read-outs. Hidden by default; F3 shows and hides it.
//!
//! Shows the average and worst frame over the last two seconds, the CPU
//! time of physics and of the member visuals, and what the lab costs. Set
//! `BREAKIT_PERF_LOG=1` to also print it once a second (for runs without a
//! screen, and for pasting into an issue).

use std::collections::VecDeque;
use std::time::Instant;

use bevy::prelude::*;
use breakit_lab::LabStats;
use breakit_render::RenderStats;

use crate::analysis::BreakitSet;

pub struct PerfPlugin;

impl Plugin for PerfPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(Perf {
            visible: false,
            log: std::env::var_os("BREAKIT_PERF_LOG").is_some(),
            ..default()
        })
        .add_systems(Startup, spawn_readout)
        .add_systems(Update, physics_start.before(BreakitSet::Physics))
        .add_systems(Update, physics_end.after(BreakitSet::Physics))
        .add_systems(Update, update_readout);
    }
}

/// Frame times kept for the read-out, seconds of history.
const WINDOW_S: f32 = 2.0;

#[derive(Resource, Default)]
pub struct Perf {
    pub visible: bool,
    log: bool,
    /// (frame ms, physics ms) per frame, newest last.
    samples: VecDeque<(f32, f32)>,
    physics_started: Option<Instant>,
    physics_ms: f32,
    since_log: f32,
}

impl Perf {
    /// Average frame ms, worst frame ms, average physics ms.
    pub fn summary(&self) -> Option<(f32, f32, f32)> {
        if self.samples.is_empty() {
            return None;
        }
        let n = self.samples.len() as f32;
        let avg = self.samples.iter().map(|s| s.0).sum::<f32>() / n;
        let worst = self.samples.iter().map(|s| s.0).fold(0.0, f32::max);
        let phys = self.samples.iter().map(|s| s.1).sum::<f32>() / n;
        Some((avg, worst, phys))
    }
}

#[derive(Component)]
struct PerfText;

fn spawn_readout(mut commands: Commands) {
    commands.spawn((
        Text::new(""),
        TextFont::from_font_size(12.0),
        TextColor(Color::srgb(0.85, 0.9, 0.95)),
        TextLayout::justify(Justify::Right),
        BackgroundColor(Color::srgba(0.05, 0.06, 0.07, 0.55)),
        Node {
            position_type: PositionType::Absolute,
            top: px(2),
            left: px(370),
            padding: UiRect::axes(px(8), px(5)),
            ..default()
        },
        PerfText,
    ));
}

fn physics_start(mut perf: ResMut<Perf>) {
    perf.physics_started = Some(Instant::now());
}

fn physics_end(mut perf: ResMut<Perf>) {
    if let Some(t) = perf.physics_started.take() {
        perf.physics_ms = t.elapsed().as_secs_f32() * 1000.0;
    }
}

fn update_readout(
    keys: Res<ButtonInput<KeyCode>>,
    // Real time: `Time` clamps frames to 250 ms, which hides slow frames.
    time: Res<Time<Real>>,
    mut perf: ResMut<Perf>,
    render: Option<Res<RenderStats>>,
    lab: Res<LabStats>,
    mut text: Single<(&mut Text, &mut Visibility), With<PerfText>>,
) {
    if keys.just_pressed(KeyCode::F3) {
        perf.visible = !perf.visible;
    }
    let dt = time.delta_secs();
    let phys = perf.physics_ms;
    perf.samples.push_back((dt * 1000.0, phys));
    let mut total: f32 = perf.samples.iter().map(|s| s.0).sum::<f32>() / 1000.0;
    while total > WINDOW_S && perf.samples.len() > 1 {
        total -= perf
            .samples
            .pop_front()
            .map(|s| s.0 / 1000.0)
            .unwrap_or(0.0);
    }
    let Some((avg, worst, phys_avg)) = perf.summary() else {
        return;
    };
    let visuals = render.map(|r| r.scene_update_ms).unwrap_or(0.0);
    let s = format!(
        "{:.1} ms  {:.0} fps  worst {:.1} ms\nphysics {:.2} ms  visuals {:.2} ms\nlab {}k tris  {} meshes  {} lamps  table: {}",
        avg,
        1000.0 / avg.max(1e-3),
        worst,
        phys_avg,
        visuals,
        (lab.hall_triangles + lab.table_triangles + 500) / 1000,
        lab.hall_meshes + lab.table_meshes,
        lab.point_lights,
        lab.table_name,
    );
    perf.since_log += dt;
    if perf.log && perf.since_log >= 1.0 {
        perf.since_log = 0.0;
        println!("perf: {}", s.replace('\n', "  |  "));
    }
    let (t, vis) = &mut *text;
    let want = if perf.visible {
        Visibility::Inherited
    } else {
        Visibility::Hidden
    };
    if **vis != want {
        **vis = want;
    }
    if perf.visible && t.0 != s {
        t.0 = s;
    }
}
