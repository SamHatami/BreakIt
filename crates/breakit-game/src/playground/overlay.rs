//! The playground's debug overlays, drawn with gizmos over the members:
//! applied forces, support reactions, an exaggerated deformed shape, the
//! picked member or node, and stand-ins for the rigs (wind, water, trucks,
//! weights) until the lab draws the real ones.

use bevy::prelude::*;
use breakit_playground::Live;

use super::{Bench, Pick};
use crate::analysis::{FrameState, Simulation};
use crate::design::{from_core, Design};

const FORCE: Color = Color::srgb(1.0, 0.54, 0.24);
const REACTION: Color = Color::srgb(0.48, 0.78, 0.49);
const PICKED: Color = Color::srgb(1.0, 0.54, 0.24);
const WIND: Color = Color::srgba(0.75, 0.85, 1.0, 0.55);
const WATER: Color = Color::srgba(0.35, 0.6, 0.85, 0.7);
const TRUCK: Color = Color::srgb(0.85, 0.3, 0.2);
const WEIGHT: Color = Color::srgb(0.8, 0.8, 0.82);

/// Longest arrow drawn, m.
const ARROW: f32 = 1.6;

/// The utilisation ramp (blue, amber, orange, red), as in the HUD.
pub fn ramp(u: f32) -> Color {
    match u {
        u if u < 0.5 => Color::srgb_u8(0x2F, 0x6F, 0xB0),
        u if u < 0.8 => Color::srgb_u8(0xE8, 0xC5, 0x47),
        u if u < 1.0 => Color::srgb_u8(0xF0, 0x8A, 0x24),
        _ => Color::srgb_u8(0xC4, 0x2B, 0x23),
    }
}

/// Arrow length for a force, scaled so the largest drawn is [`ARROW`] m.
pub fn arrow_scale(largest: f32) -> f32 {
    if largest > 0.0 {
        ARROW / largest
    } else {
        0.0
    }
}

pub fn draw(
    mut gizmos: Gizmos,
    bench: Res<Bench>,
    sim: Option<Res<Simulation>>,
    design: Res<Design>,
    state: Res<FrameState>,
) {
    let Some(sim) = sim else { return };
    let Some(live) = sim.live() else { return };
    let pos = |n: usize| from_core(live.sim().position(n));

    if bench.forces {
        let f: Vec<Vec3> = live
            .applied_forces()
            .iter()
            .map(|f| from_core(*f))
            .collect();
        let largest = f.iter().map(|v| v.length()).fold(0.0, f32::max);
        let k = arrow_scale(largest);
        for (n, v) in f.iter().enumerate() {
            if v.length() > 0.02 * largest {
                let p = pos(n);
                gizmos.arrow(p - *v * k, p, FORCE);
            }
        }
    }

    if bench.reactions {
        let r = live.reactions();
        let largest = r.iter().map(|(_, f)| f.norm() as f32).fold(0.0, f32::max);
        let k = arrow_scale(largest);
        for (n, f) in r {
            let v = from_core(f);
            if v.length() > 0.02 * largest {
                let p = pos(n);
                gizmos.arrow(p - v * k, p, REACTION);
            }
        }
    }

    let scale = bench.deform_scale();
    if scale > 1.0 && state.positions.len() == design.nodes.len() {
        let ghost = |n: usize| design.nodes[n] + (state.positions[n] - design.nodes[n]) * scale;
        for (i, b) in design.beams.iter().enumerate() {
            let m = state.members.get(i).copied().unwrap_or_default();
            if m.broken {
                continue;
            }
            gizmos.line(ghost(b.a), ghost(b.b), ramp(m.utilization).with_alpha(0.8));
        }
    }

    match bench.pick {
        Some(Pick::Node(n)) if n < design.nodes.len() => {
            gizmos.sphere(Isometry3d::from_translation(pos(n)), 0.14, PICKED);
        }
        Some(Pick::Member(m)) if m < design.beams.len() => {
            let b = design.beams[m];
            let (a, c) = (pos(b.a), pos(b.b));
            gizmos.line(a, c, PICKED);
            gizmos.sphere(Isometry3d::from_translation(a), 0.08, PICKED);
            gizmos.sphere(Isometry3d::from_translation(c), 0.08, PICKED);
        }
        _ => {}
    }

    rigs(&mut gizmos, live, &design);
}

/// Stand-ins for the rigs.
fn rigs(gizmos: &mut Gizmos, live: &Live, design: &Design) {
    let rig = live.rig_state();
    let (lo, hi) = design.nodes.iter().fold(
        (Vec3::splat(f32::MAX), Vec3::splat(f32::MIN)),
        |(lo, hi), p| (lo.min(*p), hi.max(*p)),
    );
    let fades = live.fades();
    let t = live.time() as f32;
    if fades[0] > 0.0 {
        for w in &rig.wind {
            let dir = from_core(w.direction).normalize_or_zero();
            let speed = (w.speed * fades[0]) as f32;
            if dir == Vec3::ZERO || speed < 0.5 {
                continue;
            }
            // Streaks across the structure, moving downwind.
            let across = Vec3::Y.cross(dir).normalize_or_zero();
            let mid = (lo + hi) * 0.5;
            let half = (hi - lo).length() * 0.5 + 1.0;
            let len = (speed / 12.0).clamp(0.3, 3.0);
            for i in -3..=3 {
                for j in 0..3 {
                    let lane = mid + across * (i as f32 * half / 3.0) + Vec3::Y * (j as f32 * 1.0);
                    let phase =
                        ((t * speed * 0.25 + (i * 7 + j * 3) as f32 * 0.37) % 1.0) * 2.0 - 1.0;
                    let p = lane + dir * (phase * (half + 2.0));
                    gizmos.arrow(p, p + dir * len, WIND);
                }
            }
        }
    }
    // Water surface: lines along the flow, sampled along x.
    if live.water_surface(0.0, 0.0).is_some() {
        for zi in -3..=3 {
            let z = zi as f64;
            let pts: Vec<Vec3> = (-28..=28)
                .filter_map(|xi| {
                    let x = xi as f64 * 0.5;
                    let y = live.water_surface(x, z)?;
                    Some(Vec3::new(x as f32, y as f32, z as f32))
                })
                .collect();
            gizmos.linestrip(pts, WATER);
        }
    }
    for v in &rig.vehicles {
        let fwd = from_core(v.forward).normalize_or(Vec3::X);
        let p = from_core(v.position);
        let len = v
            .wheels
            .first()
            .zip(v.wheels.last())
            .map(|(a, b)| from_core(a.contact - b.contact).length())
            .unwrap_or(2.0)
            + 1.6;
        let rot = Quat::from_rotation_arc(Vec3::X, fwd);
        gizmos.cube(
            Transform::from_translation(p + Vec3::Y * (0.75 + v.bounce as f32))
                .with_rotation(rot)
                .with_scale(Vec3::new(len, 1.1, 1.2)),
            TRUCK,
        );
        for w in &v.wheels {
            let c = from_core(w.contact) + Vec3::Y * 0.25;
            gizmos.sphere(Isometry3d::from_translation(c), 0.25, TRUCK);
        }
    }
    for i in &rig.impactors {
        gizmos.sphere(
            Isometry3d::from_translation(from_core(i.position)),
            i.radius as f32,
            WEIGHT,
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn arrows_scale_to_the_largest_force() {
        assert_eq!(arrow_scale(0.0), 0.0);
        assert!((arrow_scale(8000.0) * 8000.0 - ARROW).abs() < 1e-6);
        assert_ne!(ramp(0.2), ramp(1.2));
    }
}
