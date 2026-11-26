//! The hall as an environment map, for reflections and ambient light.
//!
//! Nothing is rendered to make it. The hall is a box with a pitched roof, so
//! what the table "sees" in any direction can be worked out from
//! [`HallSpec`]: floor, wall panels, the window band, roof lights, lamps, the
//! crane bridge. That radiance is painted into a small HDR cube map and
//! filtered on the CPU once at startup (a few milliseconds): a mip chain for
//! glossy reflections (rougher surfaces read blurrier mips) and a tiny cosine
//! convolved cube for diffuse light.
//!
//! This is what gives steel and water something to reflect, and it replaces
//! the flat ambient term with light that is cool from the roof lights above
//! and grey-warm from the floor below.

use std::f32::consts::PI;

use bevy::asset::RenderAssetUsages;
use bevy::image::Image;
use bevy::math::{Vec2, Vec3};
use bevy::render::render_resource::{
    Extent3d, TextureDimension, TextureFormat, TextureViewDescriptor, TextureViewDimension,
};

use crate::hall::HallSpec;

/// Radiance of the things in the hall, in units where a sunlit-overcast
/// roof light is about 8. The camera's environment light intensity scales
/// these to cd/m².
mod lum {
    use bevy::math::Vec3;
    pub const SKY: Vec3 = Vec3::new(7.2, 7.9, 8.6);
    pub const WINDOW: Vec3 = Vec3::new(5.0, 5.6, 6.2);
    pub const LAMP: Vec3 = Vec3::new(260.0, 232.0, 190.0);
    pub const FLOOR: Vec3 = Vec3::new(0.50, 0.49, 0.46);
    pub const PRECAST: Vec3 = Vec3::new(0.34, 0.34, 0.33);
    pub const CLADDING: Vec3 = Vec3::new(0.40, 0.43, 0.46);
    pub const ROOF: Vec3 = Vec3::new(0.21, 0.23, 0.26);
    pub const STEEL: Vec3 = Vec3::new(0.085, 0.11, 0.14);
    pub const CRANE: Vec3 = Vec3::new(0.50, 0.34, 0.07);
    pub const DOOR: Vec3 = Vec3::new(0.25, 0.26, 0.27);
}

/// What you see from `eye` (lab metres) inside the hall.
#[derive(Clone, Debug)]
pub struct HallRadiance {
    spec: HallSpec,
    eye: Vec3,
    lamps: Vec<Vec3>,
}

impl HallRadiance {
    pub fn new(spec: &HallSpec, eye: Vec3) -> Self {
        Self {
            spec: *spec,
            eye,
            lamps: spec.lamp_positions(),
        }
    }

    /// Radiance arriving at the eye from direction `d` (unit, lab axes).
    pub fn at(&self, d: Vec3) -> Vec3 {
        let s = &self.spec;
        let (hx, hz) = s.half();
        let e = self.eye;

        // Lamps first: small and very bright, they are the highlights.
        for l in &self.lamps {
            let v = *l - e;
            let dist = v.length();
            let cos = d.dot(v / dist);
            let r = 0.30 / dist;
            if cos > 1.0 - 0.5 * r * r {
                return lum::LAMP;
            }
            // Housing: a dark ring round the glowing face.
            let r2 = 0.42 / dist;
            if cos > 1.0 - 0.5 * r2 * r2 {
                return lum::STEEL * 0.6;
            }
        }

        // Crane bridge girders, crossing the hall above the eye.
        if d.y > 1e-4 {
            let t = (s.crane_rail + 0.5 - e.y) / d.y;
            let p = e + d * t;
            if p.z.abs() < hz - 0.6 && ((p.x + 6.0).abs() - 1.1).abs() < 0.3 {
                return lum::CRANE;
            }
        }

        let mut best = f32::MAX;
        let mut hit = Hit::Floor;
        let mut take = |t: f32, h: Hit| {
            if t > 1e-4 && t < best {
                best = t;
                hit = h;
            }
        };
        if d.y < -1e-5 {
            take(-e.y / d.y, Hit::Floor);
        }
        if d.x.abs() > 1e-5 {
            let sx = d.x.signum();
            take((sx * hx - e.x) / d.x, Hit::Gable(sx));
        }
        if d.z.abs() > 1e-5 {
            let sz = d.z.signum();
            take((sz * hz - e.z) / d.z, Hit::Wall);
        }
        let m = (s.ridge - s.eave) / hz;
        for side in [-1.0f32, 1.0] {
            let denom = d.y + m * side * d.z;
            if denom.abs() > 1e-6 {
                let t = (s.ridge - m * side * e.z - e.y) / denom;
                if t > 1e-4 && side * (e.z + d.z * t) >= 0.0 {
                    take(t, Hit::Roof);
                }
            }
        }
        let p = e + d * best;
        // Distance to the nearest portal frame line along the hall.
        let frame = {
            let k = ((p.x + hx) / s.bay).round();
            (p.x - (k * s.bay - hx)).abs()
        };
        match hit {
            Hit::Floor => {
                // Pools of lamp light on the floor.
                let pool: f32 = self
                    .lamps
                    .iter()
                    .map(|l| {
                        let r2 = Vec2::new(p.x - l.x, p.z - l.z).length_squared();
                        1.0 / (1.0 + r2 / 14.0)
                    })
                    .sum();
                lum::FLOOR * (0.55 + 0.4 * pool.min(1.6))
            }
            Hit::Wall => {
                if frame < 0.3 && p.y < s.eave {
                    lum::STEEL
                } else if p.y < 2.4 {
                    lum::PRECAST
                } else if p.y > 7.8 && p.y < 9.6 {
                    // Mullions break the band up.
                    if (p.x * 0.5).fract().abs() < 0.04 {
                        lum::STEEL
                    } else {
                        lum::WINDOW
                    }
                } else {
                    lum::CLADDING * (0.75 + 0.03 * p.y)
                }
            }
            Hit::Gable(sx) => {
                if sx < 0.0 && p.z.abs() < 6.0 && p.y > 7.5 && p.y < 10.5 {
                    lum::WINDOW
                } else if sx > 0.0 && p.z > 1.0 && p.z < 7.0 && p.y < 6.0 {
                    lum::DOOR
                } else if p.y < 2.4 {
                    lum::PRECAST
                } else {
                    lum::CLADDING * (0.75 + 0.03 * p.y)
                }
            }
            Hit::Roof => {
                let bay = ((p.x + hx) / s.bay).floor();
                let in_bay = p.x + hx - bay * s.bay;
                let light = bay as i32 % 2 == 0
                    && in_bay > 1.0
                    && in_bay < s.bay - 1.0
                    && p.z.abs() > 4.0
                    && p.z.abs() < 9.0;
                if light {
                    lum::SKY
                } else if frame < 0.3 || (p.z.abs() % 1.8) < 0.12 {
                    // Rafters and purlins.
                    lum::STEEL
                } else {
                    lum::ROOF
                }
            }
        }
    }
}

#[derive(Clone, Copy)]
enum Hit {
    Floor,
    Wall,
    Gable(f32),
    Roof,
}

/// A cube of square faces, `size` texels across, linear RGB.
#[derive(Clone)]
struct Cube {
    size: usize,
    px: Vec<Vec3>,
}

/// Direction of the texel centre at (`u`, `v`) in -1..1 on `face`, in
/// cube-map texture space (+X, -X, +Y, -Y, +Z, -Z; v down).
fn face_dir(face: usize, u: f32, v: f32) -> Vec3 {
    match face {
        0 => Vec3::new(1.0, -v, -u),
        1 => Vec3::new(-1.0, -v, u),
        2 => Vec3::new(u, 1.0, v),
        3 => Vec3::new(u, -1.0, -v),
        4 => Vec3::new(u, -v, 1.0),
        _ => Vec3::new(-u, -v, -1.0),
    }
    .normalize()
}

impl Cube {
    fn texel(&self, face: usize, i: usize, j: usize) -> usize {
        (face * self.size + j) * self.size + i
    }

    fn uv(&self, i: usize) -> f32 {
        (i as f32 + 0.5) / self.size as f32 * 2.0 - 1.0
    }

    /// Nearest texel in direction `d` (texture space).
    fn sample(&self, d: Vec3) -> Vec3 {
        let a = d.abs();
        let (face, u, v) = if a.x >= a.y && a.x >= a.z {
            if d.x > 0.0 {
                (0, -d.z / a.x, -d.y / a.x)
            } else {
                (1, d.z / a.x, -d.y / a.x)
            }
        } else if a.y >= a.z {
            if d.y > 0.0 {
                (2, d.x / a.y, d.z / a.y)
            } else {
                (3, d.x / a.y, -d.z / a.y)
            }
        } else if d.z > 0.0 {
            (4, d.x / a.z, -d.y / a.z)
        } else {
            (5, -d.x / a.z, -d.y / a.z)
        };
        let n = self.size as f32;
        let i = (((u + 1.0) * 0.5 * n) as usize).min(self.size - 1);
        let j = (((v + 1.0) * 0.5 * n) as usize).min(self.size - 1);
        self.px[self.texel(face, i, j)]
    }

    /// Half the resolution, each texel the mean of four.
    fn halved(&self) -> Cube {
        let size = self.size / 2;
        let mut px = Vec::with_capacity(6 * size * size);
        for face in 0..6 {
            for j in 0..size {
                for i in 0..size {
                    let mut c = Vec3::ZERO;
                    for (di, dj) in [(0, 0), (1, 0), (0, 1), (1, 1)] {
                        c += self.px[self.texel(face, 2 * i + di, 2 * j + dj)];
                    }
                    px.push(c * 0.25);
                }
            }
        }
        Cube { size, px }
    }

    fn for_each_dir(size: usize, mut f: impl FnMut(Vec3) -> Vec3) -> Cube {
        let mut c = Cube {
            size,
            px: Vec::with_capacity(6 * size * size),
        };
        for face in 0..6 {
            for j in 0..size {
                for i in 0..size {
                    c.px.push(f(face_dir(face, c.uv(i), c.uv(j))));
                }
            }
        }
        c
    }
}

/// Texture space to lab space and back: Bevy samples environment maps with
/// z negated (cube maps are left-handed).
fn flip_z(d: Vec3) -> Vec3 {
    Vec3::new(d.x, d.y, -d.z)
}

fn basis(n: Vec3) -> (Vec3, Vec3) {
    let up = if n.y.abs() < 0.9 { Vec3::Y } else { Vec3::X };
    let t = up.cross(n).normalize();
    (t, n.cross(t))
}

/// The same low-discrepancy points for every texel, so what little sampling
/// error there is shows as a smooth bias, not as speckle.
fn hammersley(i: u32, n: u32) -> (f32, f32) {
    (
        (i as f32 + 0.5) / n as f32,
        i.reverse_bits() as f32 / 4_294_967_296.0,
    )
}

/// The filtered maps, ready to become images.
pub struct EnvMaps {
    /// Mip chain, sharpest first.
    specular: Vec<Cube>,
    diffuse: Cube,
}

/// Paints and filters the hall's environment. `size` is the edge of the
/// sharpest specular face, a power of two (128 is plenty).
pub fn bake(radiance: &HallRadiance, size: usize) -> EnvMaps {
    let size = size.next_power_of_two().clamp(16, 512);
    // Sharpest level, supersampled so the lamps keep their energy.
    let ss = 3;
    let base = {
        let mut c = Cube {
            size,
            px: Vec::with_capacity(6 * size * size),
        };
        let step = 2.0 / (size * ss) as f32;
        for face in 0..6 {
            for j in 0..size {
                for i in 0..size {
                    let mut sum = Vec3::ZERO;
                    for sj in 0..ss {
                        for si in 0..ss {
                            let u = -1.0 + ((i * ss + si) as f32 + 0.5) * step;
                            let v = -1.0 + ((j * ss + sj) as f32 + 0.5) * step;
                            sum += radiance.at(flip_z(face_dir(face, u, v)));
                        }
                    }
                    c.px.push(sum / (ss * ss) as f32);
                }
            }
        }
        c
    };

    // Box-filtered pyramid: each rougher level samples a source whose
    // texels are already about the size of its lobe.
    let mut pyramid = vec![base];
    while pyramid.last().is_some_and(|c| c.size > 4) {
        let next = pyramid.last().map(Cube::halved);
        pyramid.extend(next);
    }
    let levels = pyramid.len();
    const SAMPLES: u32 = 48;
    let mut specular = vec![pyramid[0].clone()];
    for level in 1..levels {
        let rough = level as f32 / (levels - 1) as f32;
        let alpha = (rough * rough).max(1e-3);
        let src = &pyramid[level.min(levels - 1)];
        specular.push(Cube::for_each_dir(pyramid[level].size, |n| {
            let (t, b) = basis(n);
            let mut sum = Vec3::ZERO;
            let mut wsum = 0.0;
            for k in 0..SAMPLES {
                // GGX half vector about n, with view = normal = n.
                let (u1, u2) = hammersley(k, SAMPLES);
                let cos_h = ((1.0 - u1) / (1.0 + (alpha * alpha - 1.0) * u1)).sqrt();
                let sin_h = (1.0 - cos_h * cos_h).max(0.0).sqrt();
                let phi = 2.0 * PI * u2;
                let h = t * (sin_h * phi.cos()) + b * (sin_h * phi.sin()) + n * cos_h;
                let l = h * (2.0 * n.dot(h)) - n;
                let w = n.dot(l);
                if w > 0.0 {
                    sum += src.sample(l) * w;
                    wsum += w;
                }
            }
            sum / wsum.max(1e-6)
        }));
    }

    // Diffuse: the cosine-weighted integral over a coarse level, exactly.
    let src = pyramid
        .iter()
        .find(|c| c.size <= 16)
        .unwrap_or(&pyramid[levels - 1]);
    let mut dirs = Vec::with_capacity(src.px.len());
    for face in 0..6 {
        for j in 0..src.size {
            for i in 0..src.size {
                let (u, v) = (src.uv(i), src.uv(j));
                // Solid angle of a cube texel falls off towards the corners.
                let w = 1.0 / (1.0 + u * u + v * v).powf(1.5);
                dirs.push((face_dir(face, u, v), w));
            }
        }
    }
    let diffuse = Cube::for_each_dir(8, |n| {
        let mut sum = Vec3::ZERO;
        let mut wsum = 0.0;
        for ((l, w), c) in dirs.iter().zip(&src.px) {
            let k = n.dot(*l).max(0.0) * w;
            sum += *c * k;
            wsum += k;
        }
        sum / wsum.max(1e-6)
    });

    EnvMaps { specular, diffuse }
}

fn half_float(x: f32) -> u16 {
    let x = x.clamp(0.0, 65_000.0);
    let bits = x.to_bits();
    let exp = ((bits >> 23) & 0xff) as i32 - 127 + 15;
    let man = bits & 0x007f_ffff;
    if exp <= 0 {
        if exp < -10 {
            0
        } else {
            ((man | 0x0080_0000) >> (14 - exp)) as u16
        }
    } else {
        ((exp as u32) << 10 | man >> 13) as u16
    }
}

fn cube_image(levels: &[Cube]) -> Image {
    // Layer-major: each face carries its whole mip chain.
    let mut data = Vec::new();
    for face in 0..6 {
        for c in levels {
            let n = c.size * c.size;
            for p in &c.px[face * n..(face + 1) * n] {
                for x in [p.x, p.y, p.z, 1.0] {
                    data.extend_from_slice(&half_float(x).to_le_bytes());
                }
            }
        }
    }
    let size = levels[0].size as u32;
    let mut image = Image::new(
        Extent3d {
            width: size,
            height: size,
            depth_or_array_layers: 6,
        },
        TextureDimension::D2,
        data,
        TextureFormat::Rgba16Float,
        RenderAssetUsages::RENDER_WORLD,
    );
    image.texture_descriptor.mip_level_count = levels.len() as u32;
    image.texture_view_descriptor = Some(TextureViewDescriptor {
        dimension: Some(TextureViewDimension::Cube),
        ..Default::default()
    });
    image
}

impl EnvMaps {
    pub fn specular_image(&self) -> Image {
        cube_image(&self.specular)
    }

    pub fn diffuse_image(&self) -> Image {
        cube_image(std::slice::from_ref(&self.diffuse))
    }

    /// Mean diffuse radiance arriving on a surface facing `n` (lab axes).
    pub fn irradiance(&self, n: Vec3) -> Vec3 {
        self.diffuse.sample(flip_z(n.normalize()))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn maps() -> EnvMaps {
        let spec = HallSpec::default();
        bake(&HallRadiance::new(&spec, Vec3::new(4.0, 0.86, 5.0)), 64)
    }

    #[test]
    fn half_floats_round_trip() {
        for (x, bits) in [(0.0, 0u16), (1.0, 0x3c00), (2.0, 0x4000), (0.5, 0x3800)] {
            assert_eq!(half_float(x), bits, "{x}");
        }
        // Small values keep their order.
        assert!(half_float(1e-4) < half_float(2e-4));
        assert!(half_float(1e9) < 0x7c00, "never infinity");
    }

    #[test]
    fn cube_lookup_inverts_face_directions() {
        let c = Cube::for_each_dir(8, |d| d);
        for face in 0..6 {
            for &(u, v) in &[(-0.6f32, 0.3f32), (0.1, -0.8), (0.7, 0.7)] {
                let d = face_dir(face, u, v);
                assert!(c.sample(d).dot(d) > 0.97, "face {face} at {u},{v}");
            }
        }
    }

    #[test]
    fn light_comes_from_above_and_the_lamps_are_the_brightest_thing() {
        let m = maps();
        let up = m.irradiance(Vec3::Y);
        let down = m.irradiance(Vec3::NEG_Y);
        assert!(up.length() > 1.2 * down.length(), "{up} vs {down}");
        // Cool from the roof lights above, warmer off the floor.
        assert!(up.z / up.x > down.z / down.x);
        let peak = m.specular[0]
            .px
            .iter()
            .fold(0.0f32, |a, p| a.max(p.max_element()));
        assert!(peak > 20.0, "lamps survive supersampling: {peak}");
    }

    #[test]
    fn filtering_keeps_the_energy_and_blurs() {
        let m = maps();
        let mean = |c: &Cube| c.px.iter().copied().sum::<Vec3>().length() / c.px.len() as f32;
        let sharp = mean(&m.specular[0]);
        let levels = m.specular.len();
        assert_eq!(m.specular[levels - 1].size, 4);
        for (i, c) in m.specular.iter().enumerate() {
            let e = mean(c);
            assert!(
                (e / sharp - 1.0).abs() < 0.35,
                "level {i} mean {e} vs {sharp}"
            );
        }
        let spread = |c: &Cube| {
            let hi = c.px.iter().fold(0.0f32, |a, p| a.max(p.max_element()));
            let lo = c.px.iter().fold(f32::MAX, |a, p| a.min(p.max_element()));
            hi / lo
        };
        assert!(spread(&m.specular[levels - 1]) < 0.2 * spread(&m.specular[0]));
    }
}
