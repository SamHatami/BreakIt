//! Turns the solver's cells into the vertices of the water surface. Pure
//! data, no engine types beyond arrays, so it is tested without a window.
//!
//! The mesh is a regular lattice over the whole grid, `upsample` vertices
//! per cell, whether a cell is wet or not. A dry vertex takes the level of
//! the water next to it, so the surface carries on flat into a bank or a
//! wall and ends where it meets the bed; the shader discards what lies
//! below the bed (negative depth). That gives a clean shoreline that moves
//! smoothly as water runs up a slope, with no re-meshing.

use breakit_water::Field;

/// Depth below which a cell counts as dry for the surface, m.
const WET: f64 = 4e-3;

/// How far below the bed a surface with no water near it is parked, m.
const PARKED: f32 = 0.06;

/// Vertex data of the surface, ready for the mesh.
#[derive(Clone, Debug, Default)]
pub struct SurfaceBuffers {
    /// World positions.
    pub positions: Vec<[f32; 3]>,
    /// Unit normals.
    pub normals: Vec<[f32; 3]>,
    /// x: foam cover 0..1.5, y: water depth, m (negative under the bed).
    pub uvs: Vec<[f32; 2]>,
    /// xy: flow velocity (x, z), m/s; z: breaking strength; w: curvature
    /// of the surface (its Laplacian), 1/m.
    pub colors: Vec<[f32; 4]>,
    /// Vertices along x and along z.
    pub size: (usize, usize),
    level: Vec<f32>,
    wet: Vec<bool>,
    scratch: Vec<f32>,
}

/// Triangle indices of a lattice of `nx` by `nz` vertices.
pub fn lattice_indices(nx: usize, nz: usize) -> Vec<u32> {
    let mut out = Vec::with_capacity((nx - 1) * (nz - 1) * 6);
    for j in 0..nz - 1 {
        for i in 0..nx - 1 {
            let a = (j * nx + i) as u32;
            let (b, c, d) = (a + 1, a + nx as u32, a + nx as u32 + 1);
            out.extend_from_slice(&[a, c, b, b, c, d]);
        }
    }
    out
}

impl SurfaceBuffers {
    /// Fills the buffers from the water `field` and its `foam`, with
    /// `upsample` vertices per cell. The vertices on the low-x edge are
    /// moved by `paddle_shift` (m), to stay on the face of the paddle.
    pub fn fill(&mut self, field: &Field, foam: &[f64], upsample: usize, paddle_shift: f32) {
        let g = field.grid;
        let (nx, nz) = (g.nx, g.nz);
        let up = upsample.max(1);
        let n = nx * nz;

        // The level every cell would have if the water next to it carried
        // on: wet cells keep theirs, dry cells take the mean of their wet
        // neighbours, two rings deep.
        self.level.clear();
        self.wet.clear();
        for c in 0..n {
            let wet = field.depth(c) > WET;
            self.wet.push(wet);
            self.level.push(if wet {
                field.surface[c] as f32
            } else {
                field.bed[c] as f32 - PARKED
            });
        }
        for _ in 0..2 {
            self.scratch.clone_from(&self.level);
            let mut grown = self.wet.clone();
            for j in 0..nz {
                for i in 0..nx {
                    let c = j * nx + i;
                    if self.wet[c] {
                        continue;
                    }
                    let (mut sum, mut count) = (0.0, 0);
                    for (di, dj) in [
                        (-1i32, 0i32),
                        (1, 0),
                        (0, -1),
                        (0, 1),
                        (-1, -1),
                        (1, 1),
                        (-1, 1),
                        (1, -1),
                    ] {
                        let (a, b) = (i as i32 + di, j as i32 + dj);
                        if a < 0 || b < 0 || a >= nx as i32 || b >= nz as i32 {
                            continue;
                        }
                        let k = b as usize * nx + a as usize;
                        if self.wet[k] {
                            sum += self.level[k];
                            count += 1;
                        }
                    }
                    if count > 0 {
                        self.scratch[c] = sum / count as f32;
                        grown[c] = true;
                    }
                }
            }
            std::mem::swap(&mut self.level, &mut self.scratch);
            self.wet = grown;
        }

        let (vx, vz) = (nx * up + 1, nz * up + 1);
        self.size = (vx, vz);
        let count = vx * vz;
        self.positions.resize(count, [0.0; 3]);
        self.normals.resize(count, [0.0, 1.0, 0.0]);
        self.uvs.resize(count, [0.0; 2]);
        self.colors.resize(count, [0.0; 4]);
        let dx = g.dx as f32;
        let level = &self.level;
        let at = |i: i32, j: i32| -> usize {
            j.clamp(0, nz as i32 - 1) as usize * nx + i.clamp(0, nx as i32 - 1) as usize
        };
        for b in 0..vz {
            for a in 0..vx {
                // Position in cell-centre coordinates.
                let fx = a as f32 / up as f32 - 0.5;
                let fz = b as f32 / up as f32 - 0.5;
                let (i0, j0) = (fx.floor() as i32, fz.floor() as i32);
                let (tx, tz) = (fx - i0 as f32, fz - j0 as f32);
                let corners = [
                    (at(i0, j0), (1.0 - tx) * (1.0 - tz)),
                    (at(i0 + 1, j0), tx * (1.0 - tz)),
                    (at(i0, j0 + 1), (1.0 - tx) * tz),
                    (at(i0 + 1, j0 + 1), tx * tz),
                ];
                let mut y = 0.0;
                let mut bed = 0.0;
                let mut slope = [0.0f32; 2];
                let mut curve = 0.0;
                let mut flow = [0.0f32; 3];
                let mut cover = 0.0;
                for (c, w) in corners {
                    let (i, j) = ((c % nx) as i32, (c / nx) as i32);
                    let l = level[c];
                    let (xl, xr) = (level[at(i - 1, j)], level[at(i + 1, j)]);
                    let (zl, zr) = (level[at(i, j - 1)], level[at(i, j + 1)]);
                    y += w * l;
                    bed += w * field.bed[c] as f32;
                    slope[0] += w * (xr - xl) / (2.0 * dx);
                    slope[1] += w * (zr - zl) / (2.0 * dx);
                    curve += w * (xl + xr + zl + zr - 4.0 * l) / (dx * dx);
                    flow[0] += w * field.u[c] as f32;
                    flow[1] += w * field.v[c] as f32;
                    cover += w * foam[c] as f32;
                }
                // Breaking shows as a steep, fast-rising face.
                let rise = corners
                    .iter()
                    .map(|(c, w)| w * field.w[*c] as f32)
                    .sum::<f32>();
                flow[2] = (rise.max(0.0) * 2.0 / (9.81 * (y - bed).max(0.05)).sqrt()).min(2.0);
                let k = b * vx + a;
                let mut x = g.x0 as f32 + a as f32 * dx / up as f32;
                if a == 0 {
                    x += paddle_shift;
                }
                self.positions[k] = [x, y, g.z0 as f32 + b as f32 * dx / up as f32];
                let inv = 1.0 / (1.0 + slope[0] * slope[0] + slope[1] * slope[1]).sqrt();
                self.normals[k] = [-slope[0] * inv, inv, -slope[1] * inv];
                self.uvs[k] = [cover, y - bed];
                self.colors[k] = [flow[0], flow[1], flow[2], curve];
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use breakit_water::{Config, Grid, Solver};

    fn pool() -> Solver {
        let mut s = Solver::new(Grid::new(20, 6, 0.2, -2.0, -0.6), Config::default());
        // A beach rising along x from −1 m, dry beyond x = 1.
        s.set_bed(|x, _| -1.0 + (x + 0.5).max(0.0) * 0.8);
        s.fill_to(0.0, |_, _| true);
        s
    }

    #[test]
    fn lattice_covers_the_grid_edge_to_edge() {
        let s = pool();
        let field = Field::capture(&s);
        let mut b = SurfaceBuffers::default();
        b.fill(&field, &vec![0.0; 120], 2, 0.0);
        assert_eq!(b.size, (41, 13));
        assert_eq!(b.positions.len(), 41 * 13);
        let (first, last) = (b.positions[0], b.positions[41 * 13 - 1]);
        assert!((first[0] + 2.0).abs() < 1e-6 && (first[2] + 0.6).abs() < 1e-6);
        assert!((last[0] - 2.0).abs() < 1e-5 && (last[2] - 0.6).abs() < 1e-5);
        assert_eq!(lattice_indices(41, 13).len(), 40 * 12 * 6);
        assert!(lattice_indices(41, 13)
            .iter()
            .all(|i| (*i as usize) < 41 * 13));
    }

    #[test]
    fn still_water_is_flat_and_ends_at_the_shore() {
        let s = pool();
        let field = Field::capture(&s);
        let mut b = SurfaceBuffers::default();
        b.fill(&field, &vec![0.0; 120], 2, 0.0);
        let mut shore = f32::MIN;
        for (p, uv) in b.positions.iter().zip(&b.uvs) {
            if uv[1] > 0.0 {
                // Wet: on the still level, and the depth is level − bed.
                assert!(p[1].abs() < 1e-5, "surface at {}", p[1]);
                shore = shore.max(p[0]);
            }
        }
        // The bed crosses the still level at x = 0.75.
        assert!((shore - 0.75).abs() < 0.11, "shore at {shore}");
        for n in &b.normals {
            assert!((n[0] * n[0] + n[1] * n[1] + n[2] * n[2] - 1.0).abs() < 1e-5);
        }
        // Near the shore the dry vertices carry the water level on, so the
        // triangles that cross the shoreline are flat too.
        let just_dry = b
            .positions
            .iter()
            .zip(&b.uvs)
            .filter(|(p, uv)| uv[1] <= 0.0 && p[0] < 0.75 + 0.3)
            .all(|(p, _)| p[1].abs() < 1e-5);
        assert!(just_dry);
    }

    #[test]
    fn low_x_edge_follows_the_paddle() {
        let s = pool();
        let field = Field::capture(&s);
        let mut b = SurfaceBuffers::default();
        b.fill(&field, &vec![0.0; 120], 1, 0.25);
        assert!((b.positions[0][0] + 1.75).abs() < 1e-6);
        assert!((b.positions[1][0] + 1.8).abs() < 1e-6);
    }
}
