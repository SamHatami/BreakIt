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
//! The mode data is kept as separate arrays (structure of arrays), padded
//! with silent modes to a whole number of vector registers, and the sum over
//! modes is a SIMD kernel ([`crate::simd`]) with its own vector cosine
//! (`cos_kernel`): the sum is most of the cost of gusty wind. Points are
//! independent, so a frame's field is evaluated in parallel with the same
//! answer on any thread count.

use super::super::rng::Rng;
use crate::{SimdLevel, Vec3};
use pulp::Simd;

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
    /// Modes before padding.
    count: usize,
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
        // Silent modes up to a multiple of the widest register (8 lanes),
        // so the kernel has no scalar tail.
        f.count = n;
        let padded = n.next_multiple_of(8);
        for v in [
            &mut f.kx,
            &mut f.ky,
            &mut f.kz,
            &mut f.ax,
            &mut f.ay,
            &mut f.az,
            &mut f.omega,
            &mut f.phase,
        ] {
            v.resize(padded, 0.0);
        }
        f
    }

    /// Number of modes.
    pub fn modes(&self) -> usize {
        self.count
    }

    /// Field at moving-frame position `xi` (wind frame: along, across, up,
    /// m) and time `t` (s): unit-variance components (along, across, up).
    /// On the process SIMD level ([`SimdLevel::from_env`]).
    pub fn sample(&self, xi: Vec3, t: f64) -> Vec3 {
        self.sample_with(SimdLevel::from_env(), xi, t)
    }

    /// [`Self::sample`] on instruction set `level`. The scalar level is the
    /// reference; the native one agrees with it to about 1e-13 of the
    /// field's standard deviation.
    pub fn sample_with(&self, level: SimdLevel, xi: Vec3, t: f64) -> Vec3 {
        struct Op<'a>(&'a TurbulenceField, Vec3, f64);
        impl pulp::WithSimd for Op<'_> {
            type Output = Vec3;
            #[inline(always)]
            fn with_simd<S: Simd>(self, simd: S) -> Vec3 {
                let Op(f, xi, t) = self;
                fn s<S: Simd>(v: &[f64]) -> &[S::f64s] {
                    S::as_simd_f64s(v).0
                }
                let s = s::<S>;
                let (kx, ky, kz) = (s(&f.kx), s(&f.ky), s(&f.kz));
                let (ax, ay, az) = (s(&f.ax), s(&f.ay), s(&f.az));
                let (om, ph) = (s(&f.omega), s(&f.phase));
                let (x, y, z, tt) = (
                    simd.splat_f64s(xi.x),
                    simd.splat_f64s(xi.y),
                    simd.splat_f64s(xi.z),
                    simd.splat_f64s(t),
                );
                let zero = simd.splat_f64s(0.0);
                let (mut u, mut v, mut w) = (zero, zero, zero);
                for i in 0..kx.len() {
                    let arg = simd.mul_add_e_f64s(om[i], tt, ph[i]);
                    let arg = simd.mul_add_e_f64s(kx[i], x, arg);
                    let arg = simd.mul_add_e_f64s(ky[i], y, arg);
                    let arg = simd.mul_add_e_f64s(kz[i], z, arg);
                    let c = cos_kernel(simd, arg);
                    u = simd.mul_add_e_f64s(ax[i], c, u);
                    v = simd.mul_add_e_f64s(ay[i], c, v);
                    w = simd.mul_add_e_f64s(az[i], c, w);
                }
                Vec3::new(
                    simd.reduce_sum_f64s(u),
                    simd.reduce_sum_f64s(v),
                    simd.reduce_sum_f64s(w),
                )
            }
        }
        level.arch().dispatch(Op(self, xi, t))
    }
}

/// 2π split for Cody-Waite reduction: `HI` keeps the top 32 bits of the
/// mantissa, so `n HI` and `n MID` are exact for |n| < 2²¹ (arguments up to
/// about 10⁷ rad), and `LO` is what the f64 nearest 2π misses.
const TAU_HI: f64 = f64::from_bits(std::f64::consts::TAU.to_bits() & !((1u64 << 21) - 1));
const TAU_MID: f64 = std::f64::consts::TAU - TAU_HI;
const TAU_LO: f64 = 2.449_293_598_294_706_4e-16;
/// Adding and subtracting 1.5·2⁵² rounds a double to the nearest integer.
const ROUND: f64 = 6_755_399_441_055_744.0;
/// cos(r) = Σ C[k] r^(2k), k = 0..12: the Taylor series to r²⁴, which is
/// within 2e-12 of cos on [−π, π] (the next term, π²⁶/26!, bounds it).
const COS_TAYLOR: [f64; 13] = {
    let mut c = [0.0; 13];
    let mut k = 0;
    let mut fact = 1.0;
    while k < 13 {
        c[k] = if k % 2 == 0 { 1.0 / fact } else { -1.0 / fact };
        fact *= ((2 * k + 1) * (2 * k + 2)) as f64;
        k += 1;
    }
    c
};

/// Vector cosine of every lane of `x`: reduced to [−π, π] by the nearest
/// multiple of 2π, then the Taylor polynomial in r². Agrees with
/// `f64::cos` to about 1e-12 for |x| up to 10⁷ rad. Only adds and
/// multiplies, so it runs on every instruction set `pulp` dispatches to.
#[inline(always)]
pub(crate) fn cos_kernel<S: Simd>(simd: S, x: S::f64s) -> S::f64s {
    let round = simd.splat_f64s(ROUND);
    let n = simd.sub_f64s(
        simd.mul_add_e_f64s(x, simd.splat_f64s(1.0 / std::f64::consts::TAU), round),
        round,
    );
    let nn = simd.neg_f64s(n);
    let r = simd.mul_add_e_f64s(nn, simd.splat_f64s(TAU_HI), x);
    let r = simd.mul_add_e_f64s(nn, simd.splat_f64s(TAU_MID), r);
    let r = simd.mul_add_e_f64s(nn, simd.splat_f64s(TAU_LO), r);
    let r2 = simd.mul_f64s(r, r);
    let mut p = simd.splat_f64s(COS_TAYLOR[12]);
    for c in COS_TAYLOR[..12].iter().rev() {
        p = simd.mul_add_e_f64s(p, r2, simd.splat_f64s(*c));
    }
    p
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn vector_cosine_matches_the_library_cosine() {
        struct Op(Vec<f64>);
        impl pulp::WithSimd for Op {
            type Output = Vec<f64>;
            fn with_simd<S: Simd>(self, simd: S) -> Vec<f64> {
                let mut x = self.0;
                let (v, _) = S::as_mut_simd_f64s(&mut x);
                for v in v.iter_mut() {
                    *v = cos_kernel(simd, *v);
                }
                x
            }
        }
        let mut rng = Rng::new(5);
        let xs: Vec<f64> = (0..4096)
            .map(|i| {
                let scale = [1.0, 10.0, 1e3, 1e5, 1e7][i % 5];
                (2.0 * rng.uniform() - 1.0) * scale
            })
            .collect();
        for level in [SimdLevel::Scalar, SimdLevel::Native] {
            let c = level.arch().dispatch(Op(xs.clone()));
            for (x, c) in xs.iter().zip(&c) {
                // Relative to the argument: what reduction can promise.
                let tol = 2e-12 + 4e-16 * x.abs();
                assert!((c - x.cos()).abs() < tol, "{} cos({x}) = {c}", level.name());
            }
        }
    }

    #[test]
    fn native_sum_matches_the_scalar_reference() {
        let f = TurbulenceField::new(20.0, 15.0, 0.15, 90, 4);
        assert_eq!(f.modes(), 90);
        let mut rng = Rng::new(2);
        for _ in 0..500 {
            let p = Vec3::new(rng.uniform(), rng.uniform(), rng.uniform()) * 500.0;
            let t = 100.0 * rng.uniform();
            let a = f.sample_with(SimdLevel::Scalar, p, t);
            let b = f.sample_with(SimdLevel::Native, p, t);
            // Against the plain library-cosine sum too.
            let mut c = Vec3::zeros();
            for n in 0..f.kx.len() {
                let arg =
                    f.kx[n] * p.x + f.ky[n] * p.y + f.kz[n] * p.z + f.omega[n] * t + f.phase[n];
                c += Vec3::new(f.ax[n], f.ay[n], f.az[n]) * arg.cos();
            }
            assert!((a - b).norm() < 1e-11, "{a} {b}");
            assert!((a - c).norm() < 1e-10, "{a} {c}");
        }
    }

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
