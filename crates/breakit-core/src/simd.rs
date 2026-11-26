//! SIMD policy for the solver's hot loops.
//!
//! Every hot kernel in this crate is written once, generic over
//! [`pulp::Simd`], and run through [`SimdLevel::arch`]: the widest
//! instruction set the CPU reports at start-up (AVX2 with FMA on x86-64,
//! NEON on AArch64), or plain scalar code. No kernel needs `unsafe`: `pulp`
//! does the CPU check and the `#[target_feature]` call.
//!
//! [`SimdLevel::Scalar`] is the reference. It runs the same source without
//! vector instructions or fused multiply-add, so results differ from the
//! native path in the last bits only; tests compare the two within a
//! tolerance and the benchmark gate (`examples/bench.rs --gate`) fails when
//! the native path stops being faster.
//!
//! Rules for a new kernel are in `CONTRIBUTING.md` ("Hot paths").

use pulp::Simd;
use std::sync::OnceLock;

/// Which instruction set the hot kernels run on.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum SimdLevel {
    /// The widest set the CPU supports, detected once at start-up.
    #[default]
    Native,
    /// Portable scalar code without fused multiply-add: the reference path.
    Scalar,
}

impl SimdLevel {
    /// The process default: [`Native`](Self::Native), or
    /// [`Scalar`](Self::Scalar) when the environment variable `BREAKIT_SIMD`
    /// is `scalar`. Read once.
    pub fn from_env() -> Self {
        static LEVEL: OnceLock<SimdLevel> = OnceLock::new();
        *LEVEL.get_or_init(|| match std::env::var("BREAKIT_SIMD") {
            Ok(v) if v.eq_ignore_ascii_case("scalar") => Self::Scalar,
            _ => Self::Native,
        })
    }

    /// The dispatcher kernels run through.
    pub(crate) fn arch(self) -> pulp::Arch {
        match self {
            Self::Native => pulp::Arch::new(),
            Self::Scalar => pulp::Arch::Scalar,
        }
    }

    /// Name of the instruction set this level runs on here, for logs and
    /// benchmark output: `"avx2+fma"`, `"neon"` or `"scalar"`.
    pub fn name(self) -> &'static str {
        struct Name;
        impl pulp::WithSimd for Name {
            type Output = &'static str;
            fn with_simd<S: Simd>(self, _: S) -> &'static str {
                if S::IS_SCALAR {
                    "scalar"
                } else if cfg!(target_arch = "x86_64") {
                    "avx2+fma"
                } else if cfg!(target_arch = "aarch64") {
                    "neon"
                } else {
                    "simd"
                }
            }
        }
        self.arch().dispatch(Name)
    }
}

/// `acc + a b`, fused wherever the instruction set has a fused multiply-add
/// (every non-scalar set `pulp` dispatches to), separate otherwise so the
/// scalar reference never calls a software `fma`.
#[inline(always)]
pub(crate) fn madd<S: Simd>(acc: f64, a: f64, b: f64) -> f64 {
    if S::IS_SCALAR {
        acc + a * b
    } else {
        a.mul_add(b, acc)
    }
}

/// a · b in double precision, with four vector accumulators.
pub(crate) fn dot(arch: pulp::Arch, a: &[f64], b: &[f64]) -> f64 {
    struct Op<'a>(&'a [f64], &'a [f64]);
    impl pulp::WithSimd for Op<'_> {
        type Output = f64;
        #[inline(always)]
        fn with_simd<S: Simd>(self, simd: S) -> f64 {
            let (av, at) = S::as_simd_f64s(self.0);
            let (bv, bt) = S::as_simd_f64s(self.1);
            let mut acc = [simd.splat_f64s(0.0); 4];
            let (a4, a1) = av.as_chunks::<4>();
            let (b4, b1) = bv.as_chunks::<4>();
            for (a, b) in a4.iter().zip(b4) {
                for k in 0..4 {
                    acc[k] = simd.mul_add_e_f64s(a[k], b[k], acc[k]);
                }
            }
            for (a, b) in a1.iter().zip(b1) {
                acc[0] = simd.mul_add_e_f64s(*a, *b, acc[0]);
            }
            let sum = simd.add_f64s(simd.add_f64s(acc[0], acc[1]), simd.add_f64s(acc[2], acc[3]));
            let tail: f64 = at.iter().zip(bt).map(|(a, b)| a * b).sum();
            simd.reduce_sum_f64s(sum) + tail
        }
    }
    arch.dispatch(Op(a, b))
}

/// y += a x
pub(crate) fn axpy(arch: pulp::Arch, y: &mut [f64], a: f64, x: &[f64]) {
    struct Op<'a>(&'a mut [f64], f64, &'a [f64]);
    impl pulp::WithSimd for Op<'_> {
        type Output = ();
        #[inline(always)]
        fn with_simd<S: Simd>(self, simd: S) {
            let (yv, yt) = S::as_mut_simd_f64s(self.0);
            let (xv, xt) = S::as_simd_f64s(self.2);
            let a = simd.splat_f64s(self.1);
            for (y, x) in yv.iter_mut().zip(xv) {
                *y = simd.mul_add_e_f64s(*x, a, *y);
            }
            for (y, x) in yt.iter_mut().zip(xt) {
                *y += self.1 * x;
            }
        }
    }
    arch.dispatch(Op(y, a, x));
}

/// p = z + beta p
pub(crate) fn xpby(arch: pulp::Arch, p: &mut [f64], beta: f64, z: &[f64]) {
    struct Op<'a>(&'a mut [f64], f64, &'a [f64]);
    impl pulp::WithSimd for Op<'_> {
        type Output = ();
        #[inline(always)]
        fn with_simd<S: Simd>(self, simd: S) {
            let (pv, pt) = S::as_mut_simd_f64s(self.0);
            let (zv, zt) = S::as_simd_f64s(self.2);
            let beta = simd.splat_f64s(self.1);
            for (p, z) in pv.iter_mut().zip(zv) {
                *p = simd.mul_add_e_f64s(*p, beta, *z);
            }
            for (p, z) in pt.iter_mut().zip(zt) {
                *p = z + self.1 * *p;
            }
        }
    }
    arch.dispatch(Op(p, beta, z));
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The slice kernels on both levels against a plain loop, on lengths
    /// that leave every possible remainder.
    #[test]
    fn slice_kernels_match_a_plain_loop() {
        for level in [SimdLevel::Native, SimdLevel::Scalar] {
            let arch = level.arch();
            for n in 0..40 {
                let a: Vec<f64> = (0..n).map(|i| (i as f64 * 0.37).sin()).collect();
                let b: Vec<f64> = (0..n).map(|i| (i as f64 * 0.11).cos()).collect();
                let plain: f64 = a.iter().zip(&b).map(|(a, b)| a * b).sum();
                assert!((dot(arch, &a, &b) - plain).abs() <= 1e-13 * (1.0 + plain.abs()));
                let mut y = b.clone();
                axpy(arch, &mut y, 0.75, &a);
                let mut p = b.clone();
                xpby(arch, &mut p, -1.25, &a);
                for i in 0..n {
                    assert!((y[i] - (b[i] + 0.75 * a[i])).abs() <= 1e-14);
                    assert!((p[i] - (a[i] - 1.25 * b[i])).abs() <= 1e-14);
                }
            }
        }
    }
}
