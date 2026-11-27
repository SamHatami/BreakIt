//! Orbit camera: right-drag to orbit, middle-drag (or shift + right-drag) to
//! pan, scroll to zoom. Left mouse is left free for building.

use bevy::input::mouse::{AccumulatedMouseMotion, AccumulatedMouseScroll, MouseScrollUnit};
use bevy::prelude::*;
use breakit_lab::{LabCamera, LabSettings};

pub struct OrbitCameraPlugin;

impl Plugin for OrbitCameraPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, spawn_camera)
            .add_systems(Update, orbit_camera);
    }
}

#[derive(Component, Debug, Clone)]
pub struct OrbitCamera {
    pub focus: Vec3,
    pub distance: f32,
    /// Rotation about world Y, radians.
    pub yaw: f32,
    /// Elevation above the horizon, radians.
    pub pitch: f32,
}

impl Default for OrbitCamera {
    fn default() -> Self {
        Self {
            focus: Vec3::new(0.0, 0.5, 0.0),
            distance: 9.0,
            yaw: 0.6,
            pitch: 0.45,
        }
    }
}

impl OrbitCamera {
    pub fn transform(&self) -> Transform {
        let rot = Quat::from_euler(EulerRot::YXZ, self.yaw, -self.pitch, 0.0);
        let pos = self.focus + rot * Vec3::new(0.0, 0.0, self.distance);
        Transform::from_translation(pos).looking_at(self.focus, Vec3::Y)
    }
}

fn spawn_camera(mut commands: Commands) {
    let orbit = OrbitCamera::default();
    commands.spawn((
        Camera3d::default(),
        orbit.transform(),
        LabCamera {
            focus_distance: orbit.distance,
        },
        orbit,
    ));
}

fn orbit_camera(
    buttons: Res<ButtonInput<MouseButton>>,
    keys: Res<ButtonInput<KeyCode>>,
    motion: Res<AccumulatedMouseMotion>,
    scroll: Res<AccumulatedMouseScroll>,
    lab: Res<LabSettings>,
    mut cam: Single<(&mut OrbitCamera, &mut Transform, &mut LabCamera)>,
) {
    let (orbit, transform, lens) = &mut *cam;
    let shift = keys.any_pressed([KeyCode::ShiftLeft, KeyCode::ShiftRight]);
    let d = motion.delta;

    let panning =
        buttons.pressed(MouseButton::Middle) || (shift && buttons.pressed(MouseButton::Right));
    if panning {
        let right = transform.right();
        let up = transform.up();
        let scale = orbit.distance * 0.0015;
        orbit.focus += (-d.x * *right + d.y * *up) * scale;
    } else if buttons.pressed(MouseButton::Right) {
        orbit.yaw -= d.x * 0.005;
        orbit.pitch = (orbit.pitch + d.y * 0.005).clamp(-1.5, 1.5);
    }

    let lines = match scroll.unit {
        MouseScrollUnit::Line => scroll.delta.y,
        MouseScrollUnit::Pixel => scroll.delta.y / 40.0,
    };
    if lines != 0.0 {
        orbit.distance = (orbit.distance * 0.9_f32.powf(lines)).clamp(1.0, 250.0);
    }

    // Stay above the hall floor.
    let floor = lab.floor_y() + 1.0;
    let min_pitch = ((floor - orbit.focus.y) / orbit.distance)
        .clamp(-1.0, 1.0)
        .asin();
    orbit.pitch = orbit.pitch.max(min_pitch);

    **transform = orbit.transform();
    lens.focus_distance = orbit.distance;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn camera_sits_at_distance_and_looks_at_focus() {
        let o = OrbitCamera::default();
        let t = o.transform();
        assert!((t.translation.distance(o.focus) - o.distance).abs() < 1e-4);
        let to_focus = (o.focus - t.translation).normalize();
        assert!(t.forward().dot(to_focus) > 0.9999);
        // Positive pitch puts the camera above the focus.
        assert!(t.translation.y > o.focus.y);
    }
}
