//! Where a click in build mode lands in 3D: the work plane, how it turns
//! with the camera, and the axis lock. Plain maths on `Vec3`, so it is
//! tested without a window.

use bevy::prelude::*;

use super::GRID;

/// The work plane's orientation, named by the plane it lies in.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default, Hash)]
pub enum BuildPlane {
    /// Horizontal, `y = level`: decks, floors, tower footprints.
    Ground,
    /// Vertical, facing the front, `z = level`: a bridge elevation.
    #[default]
    Wall,
    /// Vertical, facing the side, `x = level`: cross frames, bracing
    /// between two walls.
    Side,
}

impl BuildPlane {
    pub const ALL: [BuildPlane; 3] = [BuildPlane::Ground, BuildPlane::Wall, BuildPlane::Side];

    pub fn normal(self) -> Vec3 {
        match self {
            BuildPlane::Ground => Vec3::Y,
            BuildPlane::Wall => Vec3::Z,
            BuildPlane::Side => Vec3::X,
        }
    }

    /// The two in-plane axes, for drawing its grid.
    pub fn axes(self) -> (Vec3, Vec3) {
        match self {
            BuildPlane::Ground => (Vec3::X, Vec3::Z),
            BuildPlane::Wall => (Vec3::X, Vec3::Y),
            BuildPlane::Side => (Vec3::Z, Vec3::Y),
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            BuildPlane::Ground => "XZ",
            BuildPlane::Wall => "XY",
            BuildPlane::Side => "YZ",
        }
    }

    /// Plain name for hints.
    pub fn name(self) -> &'static str {
        match self {
            BuildPlane::Ground => "floor",
            BuildPlane::Wall => "front",
            BuildPlane::Side => "side",
        }
    }

    pub fn axis(self) -> &'static str {
        match self {
            BuildPlane::Ground => "y",
            BuildPlane::Wall => "z",
            BuildPlane::Side => "x",
        }
    }

    /// The plane's coordinate of a point (its height above a floor plane,
    /// its depth for a wall).
    pub fn level_of(self, p: Vec3) -> f32 {
        p.dot(self.normal())
    }
}

/// How the work plane is chosen.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default, Hash)]
pub enum PlaneMode {
    /// Turns with the camera: the plane that faces it best.
    #[default]
    Auto,
    Locked(BuildPlane),
}

impl PlaneMode {
    /// Tab: auto, floor, front, side, auto.
    pub fn next(self) -> PlaneMode {
        match self {
            PlaneMode::Auto => PlaneMode::Locked(BuildPlane::Ground),
            PlaneMode::Locked(BuildPlane::Ground) => PlaneMode::Locked(BuildPlane::Wall),
            PlaneMode::Locked(BuildPlane::Wall) => PlaneMode::Locked(BuildPlane::Side),
            PlaneMode::Locked(BuildPlane::Side) => PlaneMode::Auto,
        }
    }

    pub fn resolve(self, forward: Vec3, current: BuildPlane) -> BuildPlane {
        match self {
            PlaneMode::Auto => auto_plane(forward, current),
            PlaneMode::Locked(p) => p,
        }
    }
}

/// The plane that faces a camera looking along `forward` best. Looking
/// down steeply gives the floor plane; otherwise the wall that faces the
/// camera more. `current` wins near the switch angles so the plane does
/// not flicker while orbiting.
pub fn auto_plane(forward: Vec3, current: BuildPlane) -> BuildPlane {
    const MARGIN: f32 = 0.08;
    let f = forward.normalize_or_zero();
    let score = |p: BuildPlane| {
        let s = f.dot(p.normal()).abs();
        // The floor plane needs a steep view: at a shallow angle points on
        // it run off to the horizon.
        let s = if p == BuildPlane::Ground { s - 0.25 } else { s };
        if p == current {
            s + MARGIN
        } else {
            s
        }
    };
    BuildPlane::ALL
        .into_iter()
        .max_by(|a, b| score(*a).total_cmp(&score(*b)))
        .unwrap_or(current)
}

/// Rounds to the snap grid.
pub fn snap(p: Vec3) -> Vec3 {
    (p / GRID).round() * GRID
}

/// Where a ray meets the plane, snapped to the grid in the plane and kept
/// exactly at its level.
pub fn on_plane(origin: Vec3, dir: Vec3, plane: BuildPlane, level: f32) -> Option<Vec3> {
    let n = plane.normal();
    let denom = dir.dot(n);
    if denom.abs() < 1e-4 {
        return None;
    }
    let t = (level - origin.dot(n)) / denom;
    if t <= 0.0 {
        return None;
    }
    let p = snap(origin + dir * t);
    Some(p - n * p.dot(n) + n * level)
}

/// World axis a beam can be locked to.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Axis {
    X,
    Y,
    Z,
}

impl Axis {
    pub const ALL: [Axis; 3] = [Axis::X, Axis::Y, Axis::Z];

    pub fn dir(self) -> Vec3 {
        match self {
            Axis::X => Vec3::X,
            Axis::Y => Vec3::Y,
            Axis::Z => Vec3::Z,
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            Axis::X => "along x",
            Axis::Y => "vertical",
            Axis::Z => "along z",
        }
    }

    /// The usual red, green, blue of x, y, z.
    pub fn color(self) -> Color {
        match self {
            Axis::X => Color::srgb(0.95, 0.35, 0.3),
            Axis::Y => Color::srgb(0.45, 0.85, 0.35),
            Axis::Z => Color::srgb(0.35, 0.6, 1.0),
        }
    }
}

/// Parameter along the line `p0 + t * d` (unit `d`) of the point closest to
/// the ray `o + s * r` (unit `r`). `None` when they are parallel.
pub fn closest_on_line(p0: Vec3, d: Vec3, o: Vec3, r: Vec3) -> Option<f32> {
    let w = p0 - o;
    let b = d.dot(r);
    let denom = 1.0 - b * b;
    if denom < 1e-5 {
        return None;
    }
    Some((b * r.dot(w) - d.dot(w)) / denom)
}

/// The axis lock: of the three world axes through `start`, the one whose
/// grid point nearest the ray lands closest to the cursor on screen.
/// `to_screen` projects a world point to viewport pixels.
pub fn axis_lock(
    start: Vec3,
    origin: Vec3,
    dir: Vec3,
    cursor: Vec2,
    to_screen: impl Fn(Vec3) -> Option<Vec2>,
) -> Option<(Axis, Vec3)> {
    Axis::ALL
        .into_iter()
        .filter_map(|a| {
            let t = closest_on_line(start, a.dir(), origin, dir)?;
            let t = (t / GRID).round() * GRID;
            let p = start + a.dir() * t;
            let d = to_screen(p)?.distance(cursor);
            Some((a, p, d, t))
        })
        // A zero-length lock is no beam; prefer an axis that moves.
        .min_by(|a, b| {
            let pa = a.2 + if a.3 == 0.0 { 1e4 } else { 0.0 };
            let pb = b.2 + if b.3 == 0.0 { 1e4 } else { 0.0 };
            pa.total_cmp(&pb)
        })
        .map(|(a, p, _, _)| (a, p))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn snapping_rounds_to_grid() {
        assert_eq!(snap(Vec3::new(0.26, -0.24, 1.74)), Vec3::new(0.5, 0.0, 1.5));
    }

    #[test]
    fn auto_plane_faces_the_camera() {
        // Looking at a bridge from the front and a little above.
        let front = Vec3::new(-0.3, -0.4, -0.85);
        assert_eq!(auto_plane(front, BuildPlane::Side), BuildPlane::Wall);
        // From the side.
        let side = Vec3::new(-0.9, -0.3, 0.2);
        assert_eq!(auto_plane(side, BuildPlane::Wall), BuildPlane::Side);
        // From above.
        let top = Vec3::new(0.1, -0.95, -0.2);
        assert_eq!(auto_plane(top, BuildPlane::Wall), BuildPlane::Ground);
        // At 45 degrees between front and side, the current plane stays.
        let diag = Vec3::new(-0.7, -0.2, -0.7);
        assert_eq!(auto_plane(diag, BuildPlane::Wall), BuildPlane::Wall);
        assert_eq!(auto_plane(diag, BuildPlane::Side), BuildPlane::Side);
    }

    #[test]
    fn tab_cycles_every_plane_and_back_to_auto() {
        let mut m = PlaneMode::Auto;
        let mut seen = Vec::new();
        for _ in 0..4 {
            m = m.next();
            seen.push(m);
        }
        assert_eq!(seen.last(), Some(&PlaneMode::Auto));
        for p in BuildPlane::ALL {
            assert!(seen.contains(&PlaneMode::Locked(p)));
        }
    }

    #[test]
    fn ray_lands_on_the_plane_at_its_level() {
        let o = Vec3::new(0.3, 5.0, 10.0);
        let d = Vec3::new(0.0, -0.3, -1.0).normalize();
        let p = on_plane(o, d, BuildPlane::Wall, 1.5).unwrap();
        assert_eq!(p.z, 1.5);
        assert_eq!(p, snap(p));
        let side = on_plane(Vec3::new(10.0, 1.0, 0.2), -Vec3::X, BuildPlane::Side, -2.0).unwrap();
        assert_eq!(side, Vec3::new(-2.0, 1.0, 0.0));
        // A plane behind the camera, or edge-on, gives nothing.
        assert!(on_plane(o, -d, BuildPlane::Wall, 1.5).is_none());
        assert!(on_plane(o, Vec3::X, BuildPlane::Wall, 1.5).is_none());
    }

    #[test]
    fn closest_point_on_a_line_to_a_ray() {
        // Vertical line through the origin, ray along -z at height 2 and x 0.
        let t = closest_on_line(Vec3::ZERO, Vec3::Y, Vec3::new(0.0, 2.0, 5.0), -Vec3::Z).unwrap();
        assert!((t - 2.0).abs() < 1e-5);
        assert!(closest_on_line(Vec3::ZERO, Vec3::Z, Vec3::Y, -Vec3::Z).is_none());
    }

    #[test]
    fn axis_lock_picks_the_axis_under_the_cursor() {
        // Orthographic-ish projection looking down -z: screen = (x, -y).
        let to_screen = |p: Vec3| Some(Vec2::new(p.x * 100.0, -p.y * 100.0));
        let start = Vec3::new(1.0, 0.0, 0.0);
        // Cursor straight above the start: vertical lock, snapped to 2 m.
        let cursor = Vec2::new(105.0, -210.0);
        let ray_o = Vec3::new(1.05, 2.1, 10.0);
        let (axis, p) = axis_lock(start, ray_o, -Vec3::Z, cursor, to_screen).unwrap();
        assert_eq!(axis, Axis::Y);
        assert_eq!(p, Vec3::new(1.0, 2.0, 0.0));
        // Cursor off to the right: along x.
        let cursor = Vec2::new(300.0, -10.0);
        let ray_o = Vec3::new(3.0, 0.1, 10.0);
        let (axis, p) = axis_lock(start, ray_o, -Vec3::Z, cursor, to_screen).unwrap();
        assert_eq!(axis, Axis::X);
        assert_eq!(p, Vec3::new(3.0, 0.0, 0.0));
    }
}
