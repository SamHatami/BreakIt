//! Cross-section properties.
//!
//! Local axes follow [`crate::frame`]: x along the member, y "up" in the
//! member's plane with world up, z sideways. `iz` resists bending about z
//! (deflection in local y), `iy` resists bending about y (deflection in z).

use std::f64::consts::PI;

/// The shape a section was built from. Material models that need the real
/// geometry (concrete bar layout, shear stress) read it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Shape {
    /// Solid rectangle, `b` along local z, `h` along local y.
    Rect { b: f64, h: f64 },
    /// Circular hollow section.
    Tube { ro: f64, t: f64 },
}

#[derive(Clone, Debug, PartialEq)]
pub struct Section {
    pub shape: Shape,
    /// Area, m^2.
    pub area: f64,
    /// Second moment of area about local y, m^4.
    pub iy: f64,
    /// Second moment of area about local z, m^4.
    pub iz: f64,
    /// Torsion constant, m^4.
    pub j: f64,
    /// Plastic section modulus about local y, m^3.
    pub zy: f64,
    /// Plastic section modulus about local z, m^3.
    pub zz: f64,
    /// Elastic section modulus about local y, m^3.
    pub wy: f64,
    /// Elastic section modulus about local z, m^3.
    pub wz: f64,
}

impl Section {
    /// Solid rectangle, `b` wide along local z, `h` tall along local y.
    pub fn rect(b: f64, h: f64) -> Self {
        let (a, t) = if b > h { (b, h) } else { (h, b) };
        let j =
            a * t.powi(3) * (1.0 / 3.0 - 0.21 * (t / a) * (1.0 - t.powi(4) / (12.0 * a.powi(4))));
        Self {
            shape: Shape::Rect { b, h },
            area: b * h,
            iz: b * h.powi(3) / 12.0,
            iy: h * b.powi(3) / 12.0,
            j,
            zz: b * h * h / 4.0,
            zy: h * b * b / 4.0,
            wz: b * h * h / 6.0,
            wy: h * b * b / 6.0,
        }
    }

    /// Circular hollow section with outer radius `ro` and wall thickness `t`.
    pub fn tube(ro: f64, t: f64) -> Self {
        let ri = (ro - t).max(0.0);
        let i = PI / 4.0 * (ro.powi(4) - ri.powi(4));
        let z = 4.0 / 3.0 * (ro.powi(3) - ri.powi(3));
        Self {
            shape: Shape::Tube { ro, t },
            area: PI * (ro * ro - ri * ri),
            iy: i,
            iz: i,
            j: 2.0 * i,
            zy: z,
            zz: z,
            wy: i / ro,
            wz: i / ro,
        }
    }

    /// Peak shear stress over mean shear stress (τ_max = factor · V / A):
    /// 1.5 for a rectangle, 2 for a thin tube.
    pub fn shear_factor(&self) -> f64 {
        match self.shape {
            Shape::Rect { .. } => 1.5,
            Shape::Tube { .. } => 2.0,
        }
    }

    /// Width (perpendicular to bending) and depth (in the bending direction)
    /// for bending about local y (`axis` 0) or local z (`axis` 1). Tubes are
    /// replaced by the square of equal area.
    pub fn bending_dims(&self, axis: usize) -> (f64, f64) {
        match self.shape {
            Shape::Rect { b, h } => {
                if axis == 0 {
                    (h, b)
                } else {
                    (b, h)
                }
            }
            Shape::Tube { .. } => {
                let s = self.area.sqrt();
                (s, s)
            }
        }
    }
}
