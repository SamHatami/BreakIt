//! Sectional analysis of rectangular concrete members.
//!
//! Done once per member when a simulation starts; the damage model then only
//! looks numbers up. Follows EN 1992-1-1 with characteristic strengths:
//!
//! * cracking moment from the gross section, M_cr = f_ctm · b h² / 6
//! * cracked stiffness from the transformed section (n = Es / Ec)
//! * capacity from the rectangular stress block (depth 0.8 x, stress f_ck,
//!   ε_cu = 0.0035) with elastic, perfectly plastic bars, swept over the
//!   neutral axis depth to get the axial force - moment interaction curve
//! * plastic hinge rotation capacity (φ_u − φ_y) · l_p with l_p = d / 2
//!   (Park and Paulay)
//! * shear resistance without links, V_Rc (EN 1992-1-1 6.2.2)
//!
//! Bars sit in the four corners (see [`crate::material::Rebar`]), so each
//! bending axis sees two layers of As / 2. Concrete displaced by bars is
//! ignored.

use crate::material::ConcreteProps;
use crate::Section;

/// One bending direction of a concrete section.
#[derive(Clone, Debug)]
pub struct RcAxis {
    /// Section width perpendicular to bending, m.
    pub width: f64,
    /// Section depth in the bending direction, m.
    pub depth: f64,
    /// Effective depth to the tension bars, m.
    pub effective_depth: f64,
    /// Moment at the first flexural crack, N·m.
    pub cracking_moment: f64,
    /// Gross second moment of area, m^4.
    pub gross_inertia: f64,
    /// Cracked transformed second moment of area, m^4 (0 for plain concrete).
    pub cracked_inertia: f64,
    /// Neutral axis depth of the cracked elastic section, m.
    pub cracked_neutral_axis: f64,
    /// Moment when the tension bars first yield, N·m.
    pub yield_moment: f64,
    /// Ultimate moment in pure bending, N·m.
    pub ultimate_moment: f64,
    /// Neutral axis depth at the ultimate moment, m.
    pub ultimate_neutral_axis: f64,
    pub yield_curvature: f64,
    pub ultimate_curvature: f64,
    /// Plastic hinge rotation before the compression zone crushes, rad.
    pub hinge_capacity: f64,
    interaction: Interaction,
    /// Shear resistance per unit b·d before the axial term, Pa.
    v_base: f64,
}

#[derive(Clone, Debug)]
pub struct RcSection {
    pub area: f64,
    /// Total bar area, m^2.
    pub steel_area: f64,
    /// Squash load N0 = f_ck (A − As) + σ_s As with σ_s = min(f_y, Es ε_c2), N.
    pub squash_load: f64,
    /// Axial tension at which all bars yield, As f_y, N.
    pub tension_yield: f64,
    /// Axial tension at the first through-crack, f_ctm A, N.
    pub tension_cracking: f64,
    /// Compressive strength, Pa (for the shear axial term).
    pub fck: f64,
    pub axes: [RcAxis; 2],
}

impl RcSection {
    pub fn new(sec: &Section, ec: f64, c: &ConcreteProps) -> Self {
        let area = sec.area;
        let (as_tot, fy, es, cover) = match &c.rebar {
            Some(r) => (r.ratio * area, r.yield_strength, r.youngs_modulus, r.cover),
            None => (0.0, 0.0, 200e9, 0.0),
        };
        let squash_load = c.compression * (area - as_tot) + as_tot * fy.min(es * c.peak_strain);
        let axes = [0, 1].map(|axis| {
            let (b, h) = sec.bending_dims(axis);
            axis_analysis(
                b,
                h,
                cover.min(0.25 * h),
                as_tot,
                fy,
                es,
                ec,
                c,
                squash_load,
            )
        });
        Self {
            area,
            steel_area: as_tot,
            squash_load,
            tension_yield: as_tot * fy,
            tension_cracking: c.tension * area,
            fck: c.compression,
            axes,
        }
    }
}

impl RcAxis {
    /// Moment capacity under axial compression `n` (N, compression positive;
    /// negative for tension). Zero outside the interaction curve.
    pub fn moment_capacity(&self, n: f64) -> f64 {
        self.interaction.at(n)
    }

    /// Shear resistance without links under axial compression `n` (N), N.
    pub fn shear_capacity(&self, n: f64, area: f64, fck: f64) -> f64 {
        let sigma_cp = (n / area).clamp(0.0, 0.2 * fck);
        (self.v_base + 0.15 * sigma_cp) * self.width * self.effective_depth
    }
}

#[allow(clippy::too_many_arguments)]
fn axis_analysis(
    b: f64,
    h: f64,
    cover: f64,
    as_tot: f64,
    fy: f64,
    es: f64,
    ec: f64,
    c: &ConcreteProps,
    squash_load: f64,
) -> RcAxis {
    let fck = c.compression;
    let gross_inertia = b * h.powi(3) / 12.0;
    let cracking_moment = c.tension * b * h * h / 6.0;
    let reinforced = as_tot > 0.0;
    let d = if reinforced { h - cover } else { 0.9 * h };

    // Shear, EN 1992-1-1 (6.2.a) and (6.2.b) with C_Rk = 0.18, in MPa units.
    let d_mm = d * 1000.0;
    let k = (1.0 + (200.0 / d_mm).sqrt()).min(2.0);
    let rho = (0.5 * as_tot / (b * d)).min(0.02);
    let fck_mpa = fck / 1e6;
    let v_rc = 0.18 * k * (100.0 * rho * fck_mpa).cbrt();
    let v_min = 0.035 * k.powf(1.5) * fck_mpa.sqrt();
    let v_base = v_rc.max(v_min) * 1e6;

    if !reinforced {
        return RcAxis {
            width: b,
            depth: h,
            effective_depth: d,
            cracking_moment,
            gross_inertia,
            cracked_inertia: 0.0,
            cracked_neutral_axis: 0.0,
            yield_moment: cracking_moment,
            ultimate_moment: cracking_moment,
            ultimate_neutral_axis: 0.0,
            yield_curvature: 0.0,
            ultimate_curvature: 0.0,
            hinge_capacity: 0.0,
            interaction: Interaction::default(),
            v_base,
        };
    }

    // Cracked transformed section, bars As/2 at d' = cover and at d:
    // b x²/2 + (n−1) As' (x − d') = n As (d − x).
    let n = es / ec;
    let (a1, a2) = (0.5 * as_tot, 0.5 * as_tot);
    let qa = 0.5 * b;
    let qb = (n - 1.0) * a1 + n * a2;
    let qc = -((n - 1.0) * a1 * cover + n * a2 * d);
    let x_cr = (-qb + (qb * qb - 4.0 * qa * qc).sqrt()) / (2.0 * qa);
    let cracked_inertia = b * x_cr.powi(3) / 3.0
        + (n - 1.0) * a1 * (x_cr - cover).powi(2)
        + n * a2 * (d - x_cr).powi(2);
    let yield_curvature = fy / es / (d - x_cr);
    let yield_moment = ec * cracked_inertia * yield_curvature;

    // Stress block resultant (N compression positive, M about mid-depth) for
    // neutral axis depth x.
    let layers = [(cover, a1), (d, a2)];
    let resultant = |x: f64| {
        let a = (0.8 * x).min(h);
        let cc = fck * b * a;
        let mut nn = cc;
        let mut mm = cc * (0.5 * h - 0.5 * a);
        for &(y, asl) in &layers {
            let eps = c.ultimate_strain * (x - y) / x;
            let f = asl * (es * eps).clamp(-fy, fy);
            nn += f;
            mm += f * (0.5 * h - y);
        }
        (nn, mm)
    };

    let mut interaction = vec![(-as_tot * fy, 0.0)];
    let steps = 240;
    for i in 0..=steps {
        let x = h * 10f64.powf(-3.0 + 4.3 * i as f64 / steps as f64);
        let (nn, mm) = resultant(x);
        if nn >= squash_load {
            break;
        }
        if nn > interaction.last().unwrap().0 {
            interaction.push((nn, mm.max(0.0)));
        }
    }
    interaction.push((squash_load, 0.0));
    let interaction = Interaction::resample(&interaction);

    // Pure bending: bisect for N(x) = 0.
    let (mut lo, mut hi) = (1e-6 * h, h);
    for _ in 0..100 {
        let mid = 0.5 * (lo + hi);
        if resultant(mid).0 > 0.0 {
            hi = mid;
        } else {
            lo = mid;
        }
    }
    let x_u = 0.5 * (lo + hi);
    let ultimate_moment = resultant(x_u).1;
    let ultimate_curvature = c.ultimate_strain / x_u;
    let hinge_capacity = (ultimate_curvature - yield_curvature).max(0.0) * 0.5 * d;

    RcAxis {
        width: b,
        depth: h,
        effective_depth: d,
        cracking_moment,
        gross_inertia,
        cracked_inertia,
        cracked_neutral_axis: x_cr,
        yield_moment,
        ultimate_moment,
        ultimate_neutral_axis: x_u,
        yield_curvature,
        ultimate_curvature,
        hinge_capacity,
        interaction,
        v_base,
    }
}

/// Moment capacity against axial compression, sampled at evenly spaced N so
/// a lookup is O(1) and touches one cache line.
#[derive(Clone, Debug, Default)]
struct Interaction {
    n_min: f64,
    dn: f64,
    m: Vec<f64>,
}

impl Interaction {
    const SAMPLES: usize = 64;

    /// From (N, M) points ascending in N.
    fn resample(pts: &[(f64, f64)]) -> Self {
        let (n_min, n_max) = (pts[0].0, pts[pts.len() - 1].0);
        let dn = (n_max - n_min) / (Self::SAMPLES - 1) as f64;
        let m = (0..Self::SAMPLES)
            .map(|i| {
                let n = n_min + dn * i as f64;
                let j = pts.partition_point(|p| p.0 <= n).clamp(1, pts.len() - 1);
                let (a, b) = (pts[j - 1], pts[j]);
                a.1 + (b.1 - a.1) * ((n - a.0) / (b.0 - a.0)).clamp(0.0, 1.0)
            })
            .collect();
        Self { n_min, dn, m }
    }

    fn at(&self, n: f64) -> f64 {
        if self.m.is_empty() {
            return 0.0;
        }
        let t = (n - self.n_min) / self.dn;
        if t <= 0.0 || t >= (self.m.len() - 1) as f64 {
            return 0.0;
        }
        let i = t as usize;
        let f = t - i as f64;
        self.m[i] + (self.m[i + 1] - self.m[i]) * f
    }
}
