//! Part thumbnails for the parts browser: a short length of each member,
//! seen three-quarters from above with its end cut showing, so an I-beam,
//! a box, a tube and a timber joist look like what they are.
//!
//! They are rendered once on the CPU from the same [`Profile`] the member
//! mesh and its section properties come from, into small textures, so the
//! browser needs no extra cameras or render passes.

use std::collections::HashMap;

use bevy::asset::RenderAssetUsages;
use bevy::image::ImageSampler;
use bevy::prelude::*;
use bevy::render::render_resource::{Extent3d, TextureDimension, TextureFormat};
use breakit_core::MaterialKind;
use breakit_render::breakit_mesh::Profile;

use crate::build::BuildTool;
use crate::design::{catalog, BeamKind};

/// Icon size, physical px.
pub const ICON_PX: u32 = 112;
/// Supersampling per axis.
const SS: u32 = 3;

/// Thumbnails by part, filled for every catalog part and every part the
/// sheet allows.
#[derive(Resource, Default)]
pub struct PartIcons(HashMap<String, Handle<Image>>);

impl PartIcons {
    pub fn get(&self, kind: BeamKind) -> Option<Handle<Image>> {
        self.0.get(&key(kind)).cloned()
    }
}

fn key(kind: BeamKind) -> String {
    format!("{:?} {:?}", kind.material_kind(), kind.profile())
}

/// Renders the icons that are missing.
pub fn ensure_icons(
    mut icons: ResMut<PartIcons>,
    mut images: ResMut<Assets<Image>>,
    tool: Res<BuildTool>,
    mut seeded: Local<bool>,
) {
    let mut want: Vec<BeamKind> = Vec::new();
    if !*seeded {
        *seeded = true;
        want.extend(catalog().parts.iter().map(|p| BeamKind(p.choice)));
    }
    if tool.is_changed() {
        want.extend(tool.allowed.iter().copied());
        want.push(tool.kind);
    }
    for k in want {
        let id = key(k);
        if icons.0.contains_key(&id) {
            continue;
        }
        let rgba = render_icon(k.profile(), k.material_kind(), ICON_PX, fill(k));
        let mut image = Image::new(
            Extent3d {
                width: ICON_PX,
                height: ICON_PX,
                depth_or_array_layers: 1,
            },
            TextureDimension::D2,
            rgba,
            TextureFormat::Rgba8UnormSrgb,
            RenderAssetUsages::RENDER_WORLD,
        );
        image.sampler = ImageSampler::linear();
        icons.0.insert(id, images.add(image));
    }
}

/// How much of the icon a part fills, 0.65 to 1: its size on a log scale
/// between the smallest and largest catalog part of the same material,
/// so a tie rod reads thinner than a pontoon without vanishing.
fn fill(k: BeamKind) -> f32 {
    let r = k.profile().bounding_radius();
    let (lo, hi) = catalog()
        .parts
        .iter()
        .map(|p| BeamKind(p.choice))
        .filter(|p| p.material_kind() == k.material_kind())
        .map(|p| p.profile().bounding_radius())
        .fold((r, r), |(lo, hi), x| (lo.min(x), hi.max(x)));
    if hi <= lo * 1.01 {
        return 1.0;
    }
    0.65 + 0.35 * ((r / lo).ln() / (hi / lo).ln()).clamp(0.0, 1.0)
}

/// Base colour per material, linear RGB.
fn base_colour(kind: MaterialKind) -> Vec3 {
    let c = match kind {
        MaterialKind::Steel => [0.50, 0.56, 0.62],
        MaterialKind::Timber => [0.80, 0.60, 0.36],
        MaterialKind::Concrete => [0.70, 0.69, 0.66],
        MaterialKind::Composite => [0.22, 0.23, 0.26],
    };
    Vec3::from(c).powf(2.2)
}

struct Tri {
    p: [Vec3; 3],
    n: Vec3,
    /// End cuts are drawn a touch lighter, like fresh-cut stock.
    cut: bool,
}

/// The member's surface as triangles: a length along x of `3 r`
/// (bounding radius `r`), outline faces plus both end caps.
fn triangles(profile: Profile) -> (Vec<Tri>, f32) {
    let r = profile.bounding_radius().max(1e-4);
    let half = 1.5 * r;
    let at = |x: f32, p: Vec2| Vec3::new(x, p.x, p.y);
    let mut tris = Vec::new();
    for lp in profile.outline() {
        let n = lp.len();
        for i in 0..n {
            let (a, b) = (lp[i], lp[(i + 1) % n]);
            if a.p.distance(b.p) < 1e-7 {
                continue;
            }
            let nn = (a.n + b.n).normalize_or_zero();
            let normal = Vec3::new(0.0, nn.x, nn.y);
            let (p0, p1, p2, p3) = (at(-half, a.p), at(half, a.p), at(half, b.p), at(-half, b.p));
            tris.push(Tri {
                p: [p0, p1, p2],
                n: normal,
                cut: false,
            });
            tris.push(Tri {
                p: [p0, p2, p3],
                n: normal,
                cut: false,
            });
        }
    }
    let cap = profile.cap();
    for t in &cap.triangles {
        let [a, b, c] = t.map(|i| cap.points[i as usize]);
        for (x, nx) in [(half, 1.0), (-half, -1.0)] {
            tris.push(Tri {
                p: [at(x, a), at(x, b), at(x, c)],
                n: Vec3::new(nx, 0.0, 0.0),
                cut: true,
            });
        }
    }
    (tris, r)
}

/// RGBA8 (sRGB) pixels of a `size` x `size` icon on a clear ground; the
/// member fills `fill` (0..1) of it.
pub fn render_icon(profile: Profile, kind: MaterialKind, size: u32, fill: f32) -> Vec<u8> {
    let (tris, r) = triangles(profile);
    // Three-quarter view: turned about Y, then tipped towards the viewer.
    let view = Quat::from_rotation_x(0.38) * Quat::from_rotation_y(-0.95);
    let light = (view * Vec3::new(0.35, 0.85, 0.55)).normalize();
    let back = (view * Vec3::new(-0.6, 0.1, 0.4)).normalize();
    let base = base_colour(kind);
    let n = size * SS;
    let scale = fill * 0.5 * n as f32 / (1.6 * r);
    let centre = Vec2::splat(n as f32 * 0.5);
    let mut depth = vec![f32::NEG_INFINITY; (n * n) as usize];
    let mut colour = vec![Vec4::ZERO; (n * n) as usize];
    for t in &tris {
        let normal = view * t.n;
        if normal.z <= 1e-4 {
            continue; // facing away; the camera looks down -z
        }
        let q: Vec<Vec3> =
            t.p.iter()
                .map(|p| {
                    let v = view * *p;
                    Vec3::new(centre.x + v.x * scale, centre.y - v.y * scale, v.z)
                })
                .collect();
        let lit = 0.30 + 0.75 * normal.dot(light).max(0.0) + 0.18 * normal.dot(back).max(0.0);
        let rim = (1.0 - normal.z).powi(3) * 0.25;
        let mut c = base * lit + Vec3::splat(rim);
        if t.cut {
            c = c * 1.12 + Vec3::splat(0.02);
        }
        raster(&q, n, &mut depth, &mut colour, c.extend(1.0));
    }
    // Box filter down to the icon size, linear to sRGB.
    let mut out = Vec::with_capacity((size * size * 4) as usize);
    for y in 0..size {
        for x in 0..size {
            let mut acc = Vec4::ZERO;
            for sy in 0..SS {
                for sx in 0..SS {
                    let i = ((y * SS + sy) * n + x * SS + sx) as usize;
                    let c = colour[i];
                    acc += Vec4::new(c.x * c.w, c.y * c.w, c.z * c.w, c.w);
                }
            }
            acc /= (SS * SS) as f32;
            let a = acc.w;
            let rgb = if a > 0.0 {
                acc.truncate() / a
            } else {
                Vec3::ZERO
            };
            let s = |v: f32| (v.clamp(0.0, 1.0).powf(1.0 / 2.2) * 255.0).round() as u8;
            out.extend([s(rgb.x), s(rgb.y), s(rgb.z), (a * 255.0).round() as u8]);
        }
    }
    out
}

/// Fills one screen-space triangle with a depth test (larger z is nearer).
fn raster(q: &[Vec3], n: u32, depth: &mut [f32], colour: &mut [Vec4], c: Vec4) {
    let (a, b, cc) = (q[0], q[1], q[2]);
    let area = (b.x - a.x) * (cc.y - a.y) - (b.y - a.y) * (cc.x - a.x);
    if area.abs() < 1e-9 {
        return;
    }
    let min_x = a.x.min(b.x).min(cc.x).floor().max(0.0) as u32;
    let max_x = (a.x.max(b.x).max(cc.x).ceil() as u32).min(n - 1);
    let min_y = a.y.min(b.y).min(cc.y).floor().max(0.0) as u32;
    let max_y = (a.y.max(b.y).max(cc.y).ceil() as u32).min(n - 1);
    for y in min_y..=max_y {
        for x in min_x..=max_x {
            let (px, py) = (x as f32 + 0.5, y as f32 + 0.5);
            let w0 = ((b.x - px) * (cc.y - py) - (b.y - py) * (cc.x - px)) / area;
            let w1 = ((cc.x - px) * (a.y - py) - (cc.y - py) * (a.x - px)) / area;
            let w2 = 1.0 - w0 - w1;
            if w0 < -1e-6 || w1 < -1e-6 || w2 < -1e-6 {
                continue;
            }
            let z = w0 * a.z + w1 * b.z + w2 * cc.z;
            let i = (y * n + x) as usize;
            if z > depth[i] {
                depth[i] = z;
                colour[i] = c;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn coverage(px: &[u8]) -> f32 {
        px.chunks(4).filter(|p| p[3] > 128).count() as f32 / (px.len() / 4) as f32
    }

    #[test]
    fn every_catalog_part_draws_a_visible_icon() {
        for p in &catalog().parts {
            let k = BeamKind(p.choice);
            let px = render_icon(k.profile(), k.material_kind(), 48, fill(k));
            let c = coverage(&px);
            assert!(c > 0.03 && c < 0.9, "{}: coverage {c}", p.id);
        }
    }

    /// Writes every catalog icon as raw RGBA to `BREAKIT_ICON_DUMP`, to
    /// look at them outside the game: `cargo test -p breakit-game
    /// dump_icons -- --ignored`.
    #[test]
    #[ignore = "writes files; run by hand"]
    fn dump_icons() {
        let dir = std::env::var("BREAKIT_ICON_DUMP").unwrap_or_else(|_| "target/icons".into());
        std::fs::create_dir_all(&dir).expect("icon dump folder");
        for p in &catalog().parts {
            let k = BeamKind(p.choice);
            let px = render_icon(k.profile(), k.material_kind(), ICON_PX, fill(k));
            std::fs::write(format!("{dir}/{}.rgba", p.id), px).expect("write icon");
        }
    }
}
