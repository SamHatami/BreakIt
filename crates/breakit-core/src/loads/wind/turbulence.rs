//! Spatially correlated turbulence: a sum of random Fourier modes.
//!
//! Atmospheric turbulence is a field of eddies of all sizes, carried
//! downwind by the mean wind. Two points close together see nearly the same
//! gust; points far apart (compared with the eddy size) see unrelated gusts.
//! That coherence is what decides how much of a gust a whole structure feels
//! at once, so a single gust time history applied everywhere overestimates
//! the load on a big structure and misses the twisting of a wide one.
//!
//! The field here is the random-Fourier-mode method (Kraichnan 1970; Smirnov,
//! Shi and Celik 2001): `u'(x, t) = Σ aₙ σₙ cos(kₙ·ξ + ωₙ t + φₙ)`, with
//! `ξ = x − Ū t ê` the position in a frame moving with the mean wind
//! (Taylor's frozen turbulence). Wavenumbers `kₙ` are spread over the
//! von Kármán energy spectrum `E(k) ∝ k⁴ / (1 + (k/kₑ)²)^{17/6}`,
//! `kₑ = 0.747 / L` with `L` the longitudinal integral length scale, in
//! random directions; each `σₙ` is a random unit vector normal to `kₙ`, so
//! the field is divergence free like a real incompressible flow; `ωₙ` gives
//! the eddies a finite lifetime `L / σ`. Spatial coherence and the spectrum
//! at a point then follow from the field itself instead of being imposed.
//!
//! The isotropic field is scaled per component to the atmospheric ratios
//! σ_v = 0.75 σ_u (across) and σ_w = 0.5 σ_u (vertical) (ESDU 85020,
//! EN 1991-1-4 Annex B), with the variance of each component normalised
//! exactly over the modes. Wavenumbers stop where the frequency seen at a
//! fixed point, `k Ū / 2π`, passes [`MAX_FREQUENCY`]: the solver samples
//! forces at the frame rate, and finer eddies average out over a member
//! anyway.
//!
//! The mode data is kept as separate arrays (structure of arrays) so the
//! per-point sum vectorises, and points are independent, so a frame's field
//! is evaluated in parallel.

use super::super::rng::Rng;
use crate::Vec3;

/// Highest frequency (Hz) the field puts energy at, seen from a fixed point.
pub const MAX_FREQUENCY: f64 = 6.0;

/// A frozen, convected, divergence-free random turbulence field with unit
/// variance per component (in the wind frame: along, across, up).
#[derive(Clone, Debug, Default)]
pub struct TurbulenceField {
    kx: Vec<f64>,
    ky: Vec<f64>,
    kz: Vec<f64>,
    /// Amplitude times direction, per component (wind frame).
    ax: Vec<f64>,
    ay: Vec<f64>,
    az: Vec<f64>,
    omega: Vec<f64>,
    phase: Vec<f64>,
    /// Integral length scale, m.
    pub length_scale: f64,
}

impl TurbulenceField {
    /// Build `modes` random modes for integral length scale `length_scale`
    /// (m), mean convection speed `mean_speed` (m/s) and relative intensity
    /// `intensity` (σ_u / Ū, only used for the eddy lifetime).
    pub fn new(
        length_scale: f64,
        mean_speed: f64,
        intensity: f64,
        modes: usize,
        seed: u64,
    ) -> Self {
        let l = length_scale.max(1e-3);
        let u = mean_speed.max(0.5);
        let ke = 0.747 / l;
        let k_lo = ke / 30.0;
        let k_hi = (std::f64::consts::TAU * MAX_FREQUENCY / u).max(4.0 * k_lo);
        let n = modes.max(8);
        let r = (k_hi / k_lo).powf(1.0 / n as f64);
        let mut rng = Rng::new(seed ^ 0x7075_7262);
        let mut f = Self {
            length_scale: l,
            ..Default::default()
        };
        // Eddy turnover rate: an eddy of the integral size lives L / σ_u.
        let turnover = (intensity.max(0.01) * u) / l;
        for i in 0..n {
            let (ka, kb) = (k_lo * r.powi(i as i32), k_lo * r.powi(i as i32 + 1));
            let k = (ka * kb).sqrt();
            let x = k / ke;
            let e = x.powi(4) / (1.0 + x * x).powf(17.0 / 6.0);
            let amp = (e * (kb - ka) / ke).sqrt();
            // Random direction on the sphere.
            let cz = 2.0 * rng.uniform() - 1.0;
            let th = std::f64::consts::TAU * rng.uniform();
            let s = (1.0 - cz * cz).max(0.0).sqrt();
            let kd = Vec3::new(s * th.cos(), s * th.sin(), cz);
            // Random unit vector normal to k (divergence free).
            let tmp = if kd.x.abs() < 0.9 {
                Vec3::x()
            } else {
                Vec3::y()
            };
            let e1 = kd.cross(&tmp).normalize();
            let e2 = kd.cross(&e1);
            let ps = std::f64::consts::TAU * rng.uniform();
            let sig = e1 * ps.cos() + e2 * ps.sin();
            f.kx.push(k * kd.x);
            f.ky.push(k * kd.y);
            f.kz.push(k * kd.z);
            f.ax.push(amp * sig.x);
            f.ay.push(amp * sig.y);
            f.az.push(amp * sig.z);
            // Smaller eddies turn over faster (Kolmogorov: ∝ k^{2/3}).
            f.omega
                .push(turnover * (k / ke).powf(2.0 / 3.0) * rng.normal());
            f.phase.push(std::f64::consts::TAU * rng.uniform());
        }
        // Exact unit variance per component: Σ a² / 2 = 1.
        for a in [&mut f.ax, &mut f.ay, &mut f.az] {
            let var: f64 = a.iter().map(|v| 0.5 * v * v).sum();
            if var > 0.0 {
                let k = 1.0 / var.sqrt();
                a.iter_mut().for_each(|v| *v *= k);
            }
        }
        f
    }

    /// Number of modes.
    pub fn modes(&self) -> usize {
        self.kx.len()
    }

    /// Field at moving-frame position `xi` (wind frame: along, across, up,
    /// m) and time `t` (s): unit-variance components (along, across, up).
    pub fn sample(&self, xi: Vec3, t: f64) -> Vec3 {
        let (mut u, mut v, mut w) = (0.0, 0.0, 0.0);
        for n in 0..self.kx.len() {
            let arg = self.kx[n] * xi.x
                + self.ky[n] * xi.y
                + self.kz[n] * xi.z
                + self.omega[n] * t
                + self.phase[n];
            let c = arg.cos();
            u += self.ax[n] * c;
            v += self.ay[n] * c;
            w += self.az[n] * c;
        }
        Vec3::new(u, v, w)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn components_have_unit_variance_over_space() {
        let f = TurbulenceField::new(20.0, 15.0, 0.15, 96, 3);
        let mut rng = Rng::new(9);
        let n = 40_000;
        let mut s2 = Vec3::zeros();
        let mut s1 = Vec3::zeros();
        for _ in 0..n {
            let p = Vec3::new(rng.uniform(), rng.uniform(), rng.uniform()) * 2000.0;
            let u = f.sample(p, 0.0);
            s1 += u;
            s2 += u.component_mul(&u);
        }
        for i in 0..3 {
            let var = s2[i] / n as f64 - (s1[i] / n as f64).powi(2);
            assert!((var - 1.0).abs() < 0.06, "component {i} variance {var}");
        }
    }
}
