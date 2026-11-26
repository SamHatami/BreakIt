//! Member local coordinate frames.

use crate::Vec3;

/// World up axis (Y-up, as in Bevy).
pub const UP: Vec3 = Vec3::new(0.0, 1.0, 0.0);

/// Local axes (x, y, z) of a member running from `a` to `b`.
///
/// x points from a to b. y lies in the plane of x and world up (pointing
/// "upward"), z = x × y. For vertical members world X stands in for up.
pub fn member_axes(a: &Vec3, b: &Vec3) -> (Vec3, Vec3, Vec3) {
    let x = (b - a).normalize();
    let up = if x.dot(&UP).abs() > 0.999 {
        Vec3::x()
    } else {
        UP
    };
    let z = x.cross(&up).normalize();
    let y = z.cross(&x);
    (x, y, z)
}
