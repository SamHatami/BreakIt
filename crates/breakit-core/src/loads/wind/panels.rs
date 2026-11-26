//! Flat panels that catch the wind: signboards, cladding sheets, a deck plate.
//!
//! A panel is a flat plate given by 3 or 4 corners. It has no stiffness or
//! mass of its own: each corner is tied to the nearest structure node when
//! the case is prepared ([`PanelSpec::resolve`]), and every frame the
//! aerodynamic force on the plate is shared between those nodes
//! ([`apply_panel`]).
//!
//! The model, per frame:
//!
//! * Geometry from the current corner positions: area `A` and unit normal
//!   `n` from the diagonals' cross product (exact for a planar quad), the
//!   area centroid from the two triangles.
//! * Relative wind `w` = wind velocity at the centroid − mean corner
//!   velocity (aerodynamic damping of a swinging sign for free).
//! * Angle of attack `α` between `w` and the plate plane:
//!   `sin α = |w·n| / |w|`.
//! * Normal force `F_n = q A K_p C_N(α)` with `q = ½ ρ |w|²`:
//!   * `C_N(α)` is Hoerner's flat-plate normal-force law (see
//!     [`normal_force_coefficient`]), scaled so that `C_N(90°)` equals the
//!     panel's [`PanelSpec::force_coefficient`] (1.8 for a signboard,
//!     EN 1991-1-4 §7.4.3).
//!   * `K_p = 1 − (1 − δ)²` is the porosity factor of AS/NZS 1170.2
//!     (Appendix D, hoardings and free-standing walls), `δ = 1 − φ` the
//!     solidity (see [`porosity_factor`]).
//! * Centre of pressure: at 90° the centroid, shifted by the EN 1991-1-4
//!   §7.4.3 eccentricity `e = ±0.25 b` along the horizontal in-plane axis when
//!   [`PanelSpec::eccentricity`] asks for the torsion case. For oblique flow
//!   it also moves toward the leading edge, from 0.5 of the chord at 90° to
//!   0.25 at grazing flow ([`center_of_pressure_fraction`]).
//! * The force goes to the corners with the bilinear (quad) or barycentric
//!   (triangle) weights of the centre of pressure, so the corner forces sum to
//!   the panel force and their moment about the centroid is that of the force
//!   acting at the centre of pressure.
//! * Skin friction along the in-plane wind component, `c_fr q A` on each of
//!   the two faces (EN 1991-1-4 §7.5, `c_fr = 0.01` for smooth surfaces),
//!   applied at the centroid. It is what remains when the panel is edge-on.
//!
//! A panel stops catching wind when every member at its corners is broken
//! (it is then a loose sheet; the debris is not modelled). While at least one
//! member holds it, the whole force still goes to the corner nodes.

use crate::loads::View;
use crate::{Structure, Vec3};

/// A flat panel (scenario data): where it is and how it catches the wind.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct PanelSpec {
    /// Display name, for the brief and the visuals.
    pub name: String,
    /// Rest positions of the 3 or 4 corners, in order around the edge, m.
    /// The normal follows the right-hand rule on this order.
    pub corners: Vec<Vec3>,
    /// A corner ties to the nearest structure node within this distance, m.
    pub capture_radius: f64,
    /// Net force coefficient normal to the plate with the wind square on
    /// (α = 90°), on the gross area. 1.8 for signboards, EN 1991-1-4 §7.4.3.
    pub force_coefficient: f64,
    /// Open-area ratio φ, 0 (solid) to 1 (all holes); see [`porosity_factor`].
    pub porosity: f64,
    /// Skin friction coefficient per face, EN 1991-1-4 Table 7.10: 0.01
    /// smooth (steel, smooth concrete), 0.02 rough, 0.04 very rough (ribs,
    /// folds).
    pub friction_coefficient: f64,
    /// Horizontal eccentricity of the centre of pressure as a fraction of
    /// the panel's horizontal width `b`, signed along the horizontal
    /// in-plane axis. 0 normally; ±0.25 is the torsion case of
    /// EN 1991-1-4 §7.4.3.
    pub eccentricity: f64,
}

impl PanelSpec {
    /// A solid signboard: `c_f = 1.8`, no porosity, smooth faces, centred
    /// load, corners captured within 0.25 m.
    pub fn sign(name: impl Into<String>, corners: Vec<Vec3>) -> Self {
        Self {
            name: name.into(),
            corners,
            capture_radius: 0.25,
            force_coefficient: 1.8,
            porosity: 0.0,
            friction_coefficient: 0.01,
            eccentricity: 0.0,
        }
    }

    /// Tie each corner to its nearest node within the capture radius and
    /// record the members that hold the panel. Call when the build is final.
    pub fn resolve(&self, s: &Structure) -> Result<PanelState, PanelError> {
        let n = self.corners.len();
        if !(3..=4).contains(&n) {
            return Err(PanelError::CornerCount {
                panel: self.name.clone(),
                count: n,
            });
        }
        if !(0.0..=1.0).contains(&self.porosity) {
            return Err(PanelError::Porosity {
                panel: self.name.clone(),
                porosity: self.porosity,
            });
        }
        let r = self.capture_radius.max(0.0);
        let mut nodes = Vec::with_capacity(n);
        for (corner, c) in self.corners.iter().enumerate() {
            let best = s
                .nodes
                .iter()
                .enumerate()
                .map(|(i, p)| ((p - c).norm(), i))
                .filter(|&(d, _)| d <= r)
                .min_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
            let Some((_, node)) = best else {
                return Err(PanelError::NoNode {
                    panel: self.name.clone(),
                    corner,
                });
            };
            if nodes.contains(&node) {
                return Err(PanelError::SharedNode {
                    panel: self.name.clone(),
                    node,
                });
            }
            nodes.push(node);
        }
        let rest: Vec<Vec3> = nodes.iter().map(|&i| s.nodes[i]).collect();
        let (area, normal, center) = plate_geometry(&rest);
        if area < 1e-9 {
            return Err(PanelError::Degenerate {
                panel: self.name.clone(),
            });
        }
        let members = s
            .members
            .iter()
            .enumerate()
            .filter(|(_, m)| nodes.contains(&m.a) || nodes.contains(&m.b))
            .map(|(i, _)| i)
            .collect();
        Ok(PanelState {
            nodes,
            members,
            center,
            normal,
            area,
            center_of_pressure: center,
            ..Default::default()
        })
    }
}

/// Why a panel could not be tied to the structure.
#[derive(Clone, Debug, PartialEq)]
pub enum PanelError {
    /// A panel needs 3 or 4 corners.
    CornerCount {
        /// Panel name.
        panel: String,
        /// Number of corners given.
        count: usize,
    },
    /// No structure node within the capture radius of a corner.
    NoNode {
        /// Panel name.
        panel: String,
        /// Index of the corner in [`PanelSpec::corners`].
        corner: usize,
    },
    /// Two corners resolved to the same node.
    SharedNode {
        /// Panel name.
        panel: String,
        /// The node both corners found.
        node: usize,
    },
    /// The resolved corners enclose no area.
    Degenerate {
        /// Panel name.
        panel: String,
    },
    /// Porosity outside 0..1.
    Porosity {
        /// Panel name.
        panel: String,
        /// The value given.
        porosity: f64,
    },
}

impl std::fmt::Display for PanelError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::CornerCount { panel, count } => {
                write!(f, "panel '{panel}' has {count} corners, needs 3 or 4")
            }
            Self::NoNode { panel, corner } => write!(
                f,
                "panel '{panel}': no node within the capture radius of corner {corner}"
            ),
            Self::SharedNode { panel, node } => {
                write!(f, "panel '{panel}': two corners resolve to node {node}")
            }
            Self::Degenerate { panel } => write!(f, "panel '{panel}' has no area"),
            Self::Porosity { panel, porosity } => {
                write!(f, "panel '{panel}': porosity {porosity} is outside 0..1")
            }
        }
    }
}

impl std::error::Error for PanelError {}

/// Runtime state of a panel: its nodes and the load of the last frame.
#[derive(Clone, Debug, Default)]
pub struct PanelState {
    /// Structure node of each corner, in corner order.
    pub nodes: Vec<usize>,
    /// Members touching any corner node; the panel lets go when all of them
    /// are broken.
    pub members: Vec<usize>,
    /// Area centroid of the plate in the last frame, m.
    pub center: Vec3,
    /// Unit normal of the plate in the last frame (right-hand rule on the
    /// corner order).
    pub normal: Vec3,
    /// Plate area in the last frame, m².
    pub area: f64,
    /// Where the normal force acted in the last frame, m.
    pub center_of_pressure: Vec3,
    /// Net pressure in the last frame, Pa: the normal force over the area,
    /// positive when it pushes along [`Self::normal`].
    pub net_pressure: f64,
    /// Total force on the panel in the last frame (normal + friction), N.
    pub force: Vec3,
    /// False once every member holding the panel is broken.
    pub attached: bool,
}

impl PanelState {
    /// State for the visuals (pressure colouring, flapping sign).
    pub fn rig(&self, name: &str) -> PanelRig {
        PanelRig {
            name: name.to_string(),
            center: self.center,
            normal: self.normal,
            net_pressure: self.net_pressure,
            force: self.force,
        }
    }
}

/// What the renderer needs of a panel at one time.
#[derive(Clone, Debug, Default)]
pub struct PanelRig {
    /// Panel name.
    pub name: String,
    /// Area centroid, m.
    pub center: Vec3,
    /// Unit normal.
    pub normal: Vec3,
    /// Net pressure along the normal, Pa.
    pub net_pressure: f64,
    /// Total force, N.
    pub force: Vec3,
}

/// Porosity reduction on the net force of a plate with open-area ratio
/// `porosity` (φ): `K_p = 1 − (1 − δ)²` with solidity `δ = 1 − φ`, i.e.
/// `1 − φ²`. AS/NZS 1170.2 Appendix D (net porosity factor for hoardings and
/// free-standing walls). Gentler than `(1 − φ)²`: a 30 % open sign still
/// carries about 90 % of the solid load, which matches the hoarding tests
/// that standard is based on (Letchford, JWEIA 2001).
pub fn porosity_factor(porosity: f64) -> f64 {
    let delta = 1.0 - porosity.clamp(0.0, 1.0);
    1.0 - (1.0 - delta) * (1.0 - delta)
}

/// Normal-force coefficient of a flat plate at angle of attack `alpha`
/// (rad, between the wind and the plate plane), scaled to `c90` at 90°.
///
/// Hoerner's fit for sharp-edged flat plates past the stall,
/// `C_N = 1 / (0.222 + 0.283 / sin α)` (Hoerner, *Fluid-Dynamic Drag*, 1965,
/// ch. 3; as used for post-stall blades by Lindenburg, ECN-C--03-025, 2003),
/// normalised by its 90° value 1/0.505:
///
/// `C_N(α) = c90 · 0.505 sin α / (0.222 sin α + 0.283)`.
///
/// It is 0 edge-on, `0.812 c90` at 45°. It ignores the attached-flow lift
/// slope below about 10°, where a sign carries little load anyway.
pub fn normal_force_coefficient(alpha: f64, c90: f64) -> f64 {
    let s = alpha.sin().abs();
    c90 * 0.505 * s / (0.222 * s + 0.283)
}

/// Centre of pressure of a flat plate from its leading edge, as a fraction
/// of the chord, at angle of attack `alpha` (rad): `0.25 + 0.25 α/(π/2)`,
/// linear from the quarter chord (thin-aerofoil value at small α) to mid
/// chord at 90°. A straight-line fit of the flat-plate measurements in
/// Hoerner, *Fluid-Dynamic Lift* (1985), ch. 4 (Flachsbart's data).
pub fn center_of_pressure_fraction(alpha: f64) -> f64 {
    let a = alpha.abs().min(std::f64::consts::FRAC_PI_2);
    0.25 + 0.25 * a / std::f64::consts::FRAC_PI_2
}

/// Add the wind force on one panel to `out` (per node, N).
///
/// `velocity(p)` is the wind velocity at `p` (m/s) including any ramp, e.g.
/// `&|p| wind.velocity(p, t, ramp)`; `rho` the air density (kg/m³); `scale`
/// the load factor on the force. Updates `state` with the geometry and load
/// of the frame.
pub fn apply_panel(
    spec: &PanelSpec,
    state: &mut PanelState,
    view: &View,
    rho: f64,
    scale: f64,
    velocity: &dyn Fn(Vec3) -> Vec3,
    out: &mut [Vec3],
) {
    state.net_pressure = 0.0;
    state.force = Vec3::zeros();
    state.attached = state.members.iter().any(|&m| !view.sim.is_broken(m));
    let n = state.nodes.len();
    if !state.attached || !(3..=4).contains(&n) {
        return;
    }
    let pts: Vec<Vec3> = state.nodes.iter().map(|&i| view.sim.position(i)).collect();
    let v_plate = state
        .nodes
        .iter()
        .map(|&i| view.sim.velocity(i))
        .sum::<Vec3>()
        / n as f64;
    let (area, normal, center) = plate_geometry(&pts);
    state.area = area;
    state.center = center;
    state.center_of_pressure = center;
    if area < 1e-12 {
        return;
    }
    state.normal = normal;
    let w = velocity(center) - v_plate;
    let speed = w.norm();
    if speed < 1e-9 {
        return;
    }
    let q = 0.5 * rho * speed * speed;
    let wn = w.dot(&normal);
    let w_t = w - normal * wn;
    let sin_a = (wn.abs() / speed).min(1.0);
    let alpha = sin_a.asin();
    let c_n =
        normal_force_coefficient(alpha, spec.force_coefficient) * porosity_factor(spec.porosity);
    let f_n = q * area * c_n * wn.signum() * scale;

    // In-plane axes: horizontal across the plate (EN eccentricity axis), and
    // the in-plane wind direction (chord direction for oblique flow).
    let mut h = Vec3::y().cross(&normal);
    if h.norm() < 1e-6 {
        h = pts[1] - pts[0];
        h -= normal * normal.dot(&h);
    }
    let h = h.normalize();
    let extent = |dir: &Vec3| {
        let (lo, hi) = pts
            .iter()
            .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), p| {
                let x = p.dot(dir);
                (lo.min(x), hi.max(x))
            });
        hi - lo
    };
    let mut cop = center + h * (spec.eccentricity * extent(&h));
    if w_t.norm() > 1e-9 * speed {
        let t = w_t.normalize();
        // Leading edge is upstream (−t); mid chord sits at the centroid.
        let shift = 0.5 - center_of_pressure_fraction(alpha);
        cop -= t * (shift * extent(&t));
    }
    state.center_of_pressure = cop;
    let f_normal = normal * f_n;
    let f_fric = w_t * (0.5 * rho * speed * spec.friction_coefficient * 2.0 * area * scale);

    let basis = (h, normal.cross(&h));
    for (i, wgt) in corner_weights(&pts, cop, basis).iter().enumerate() {
        out[state.nodes[i]] += f_normal * *wgt;
    }
    if f_fric.norm() > 0.0 {
        for (i, wgt) in corner_weights(&pts, center, basis).iter().enumerate() {
            out[state.nodes[i]] += f_fric * *wgt;
        }
    }
    state.net_pressure = f_n / area;
    state.force = f_normal + f_fric;
}

/// Area (m²), unit normal and area centroid of a triangle or planar quad.
fn plate_geometry(p: &[Vec3]) -> (f64, Vec3, Vec3) {
    let (cross, center) = if p.len() == 3 {
        let c = (p[1] - p[0]).cross(&(p[2] - p[0]));
        (c, (p[0] + p[1] + p[2]) / 3.0)
    } else {
        let c1 = (p[1] - p[0]).cross(&(p[2] - p[0]));
        let c2 = (p[2] - p[0]).cross(&(p[3] - p[0]));
        let (a1, a2) = (0.5 * c1.norm(), 0.5 * c2.norm());
        let g1 = (p[0] + p[1] + p[2]) / 3.0;
        let g2 = (p[0] + p[2] + p[3]) / 3.0;
        let center = if a1 + a2 > 0.0 {
            (g1 * a1 + g2 * a2) / (a1 + a2)
        } else {
            (p[0] + p[1] + p[2] + p[3]) / 4.0
        };
        ((p[2] - p[0]).cross(&(p[3] - p[1])), center)
    };
    let len = cross.norm();
    let normal = if len > 0.0 {
        cross / len
    } else {
        Vec3::zeros()
    };
    (0.5 * len, normal, center)
}

/// Weights of point `x` on the corners `p` (barycentric for a triangle,
/// inverse bilinear for a quad), computed in the plate plane spanned by
/// `basis`. They sum to 1 and `Σ wᵢ pᵢ = x` for a point in the plane.
fn corner_weights(p: &[Vec3], x: Vec3, basis: (Vec3, Vec3)) -> Vec<f64> {
    let to2 = |v: Vec3| (v.dot(&basis.0), v.dot(&basis.1));
    let q: Vec<(f64, f64)> = p.iter().map(|&v| to2(v)).collect();
    let (xx, xy) = to2(x);
    if q.len() == 3 {
        let (ax, ay) = (q[1].0 - q[0].0, q[1].1 - q[0].1);
        let (bx, by) = (q[2].0 - q[0].0, q[2].1 - q[0].1);
        let (rx, ry) = (xx - q[0].0, xy - q[0].1);
        let det = ax * by - ay * bx;
        if det.abs() < 1e-18 {
            return vec![1.0 / 3.0; 3];
        }
        let u = (rx * by - ry * bx) / det;
        let v = (ax * ry - ay * rx) / det;
        return vec![1.0 - u - v, u, v];
    }
    // P(u, v) = p0 + u e + v f + u v k, solved for (u, v) by Newton.
    let e = (q[1].0 - q[0].0, q[1].1 - q[0].1);
    let f = (q[3].0 - q[0].0, q[3].1 - q[0].1);
    let k = (
        q[0].0 - q[1].0 + q[2].0 - q[3].0,
        q[0].1 - q[1].1 + q[2].1 - q[3].1,
    );
    let (mut u, mut v) = (0.5, 0.5);
    for _ in 0..20 {
        let rx = q[0].0 + u * e.0 + v * f.0 + u * v * k.0 - xx;
        let ry = q[0].1 + u * e.1 + v * f.1 + u * v * k.1 - xy;
        let (jux, juy) = (e.0 + v * k.0, e.1 + v * k.1);
        let (jvx, jvy) = (f.0 + u * k.0, f.1 + u * k.1);
        let det = jux * jvy - jvx * juy;
        if det.abs() < 1e-18 {
            break;
        }
        let du = (rx * jvy - ry * jvx) / det;
        let dv = (jux * ry - juy * rx) / det;
        u -= du;
        v -= dv;
        if du.abs() + dv.abs() < 1e-14 {
            break;
        }
    }
    vec![(1.0 - u) * (1.0 - v), u * (1.0 - v), u * v, (1.0 - u) * v]
}
