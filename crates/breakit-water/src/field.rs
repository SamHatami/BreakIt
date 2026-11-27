//! A frozen copy of the water at one instant, for everything that reads it:
//! the structure's loads (it implements the core's
//! [`WaterField`]), floating bodies, the renderer and the tests.

use breakit_core::loads::water::WaterField;
use breakit_core::Vec3;

use crate::grid::Grid;
use crate::solver::Solver;

/// Depth below which a cell counts as dry for readers, m.
const WET: f64 = 5e-3;

/// The water at one instant, per cell.
#[derive(Clone, Debug)]
pub struct Field {
    /// The grid.
    pub grid: Grid,
    /// Simulated time, s.
    pub time: f64,
    /// Bed level, absolute y, m.
    pub bed: Vec<f64>,
    /// Free surface, absolute y, m (the bed where dry).
    pub surface: Vec<f64>,
    /// Depth-averaged velocity along x at the cell centre, m/s.
    pub u: Vec<f64>,
    /// Depth-averaged velocity along z at the cell centre, m/s.
    pub v: Vec<f64>,
    /// Depth-averaged vertical velocity, m/s.
    pub w: Vec<f64>,
    /// Local acceleration `∂u/∂t` along x, averaged over the last frame, m/s².
    pub au: Vec<f64>,
    /// Local acceleration along z, m/s².
    pub av: Vec<f64>,
    /// Local vertical acceleration, m/s².
    pub aw: Vec<f64>,
    lowest_bed: f64,
    highest_surface: f64,
}

impl Field {
    /// The solver's state now, with no accelerations.
    pub fn capture(s: &Solver) -> Self {
        let g = s.grid;
        let n = g.cells();
        let mut f = Self {
            grid: g,
            time: s.time(),
            bed: s.bed().to_vec(),
            surface: s.surface().to_vec(),
            u: vec![0.0; n],
            v: vec![0.0; n],
            w: s.vertical().to_vec(),
            au: vec![0.0; n],
            av: vec![0.0; n],
            aw: vec![0.0; n],
            lowest_bed: f64::INFINITY,
            highest_surface: f64::NEG_INFINITY,
        };
        for j in 0..g.nz {
            for i in 0..g.nx {
                let c = g.c(i, j);
                (f.u[c], f.v[c]) = s.cell_velocity(i, j);
                if f.surface[c] - f.bed[c] > WET {
                    f.lowest_bed = f.lowest_bed.min(f.bed[c]);
                    f.highest_surface = f.highest_surface.max(f.surface[c]);
                }
            }
        }
        f
    }

    /// The solver's state now, with accelerations from the change since
    /// `before` (the field one frame earlier).
    pub fn capture_after(s: &Solver, before: &Field) -> Self {
        let mut f = Self::capture(s);
        let dt = f.time - before.time;
        if dt > 1e-9 && before.u.len() == f.u.len() {
            for c in 0..f.u.len() {
                f.au[c] = (f.u[c] - before.u[c]) / dt;
                f.av[c] = (f.v[c] - before.v[c]) / dt;
                f.aw[c] = (f.w[c] - before.w[c]) / dt;
            }
        }
        f
    }

    /// Water depth of cell `c`, m.
    pub fn depth(&self, c: usize) -> f64 {
        self.surface[c] - self.bed[c]
    }

    /// The wet cells around `(x, z)` with their bilinear weights (summing
    /// to 1), or `None` if all are dry or the point is off the grid.
    fn stencil(&self, x: f64, z: f64) -> Option<[(usize, f64); 4]> {
        let g = &self.grid;
        let fx = (x - g.x0) / g.dx;
        let fz = (z - g.z0) / g.dx;
        if fx < 0.0 || fz < 0.0 || fx > g.nx as f64 || fz > g.nz as f64 {
            return None;
        }
        let (ax, az) = (fx - 0.5, fz - 0.5);
        let i0 = ax.floor().clamp(0.0, (g.nx - 1) as f64);
        let j0 = az.floor().clamp(0.0, (g.nz - 1) as f64);
        let tx = (ax - i0).clamp(0.0, 1.0);
        let tz = (az - j0).clamp(0.0, 1.0);
        let (i0, j0) = (i0 as usize, j0 as usize);
        let (i1, j1) = ((i0 + 1).min(g.nx - 1), (j0 + 1).min(g.nz - 1));
        let mut out = [
            (g.c(i0, j0), (1.0 - tx) * (1.0 - tz)),
            (g.c(i1, j0), tx * (1.0 - tz)),
            (g.c(i0, j1), (1.0 - tx) * tz),
            (g.c(i1, j1), tx * tz),
        ];
        let mut sum = 0.0;
        for (c, wgt) in &mut out {
            if self.depth(*c) <= WET {
                *wgt = 0.0;
            }
            sum += *wgt;
        }
        if sum <= 1e-9 {
            return None;
        }
        for (_, wgt) in &mut out {
            *wgt /= sum;
        }
        Some(out)
    }

    /// Surface (absolute y) and bed (absolute y) at `(x, z)`, m, or `None`
    /// where dry.
    pub fn column(&self, x: f64, z: f64) -> Option<(f64, f64)> {
        let st = self.stencil(x, z)?;
        let mut s = 0.0;
        let mut b = 0.0;
        for (c, wgt) in st {
            s += wgt * self.surface[c];
            b += wgt * self.bed[c];
        }
        Some((s, b))
    }

    /// Depth-averaged horizontal velocity `(u, v)` at `(x, z)`, m/s (zero
    /// where dry).
    pub fn flow(&self, x: f64, z: f64) -> (f64, f64) {
        self.stencil(x, z).map_or((0.0, 0.0), |st| {
            st.iter().fold((0.0, 0.0), |a, (c, wgt)| {
                (a.0 + wgt * self.u[*c], a.1 + wgt * self.v[*c])
            })
        })
    }
}

impl WaterField for Field {
    fn bed(&self) -> f64 {
        self.lowest_bed
    }

    fn max_surface(&self) -> f64 {
        self.highest_surface
    }

    fn surface(&self, x: f64, z: f64) -> Option<f64> {
        self.column(x, z).map(|c| c.0)
    }

    /// Horizontal velocity and acceleration are uniform over the depth (the
    /// model's depth averages); the vertical ones grow linearly from zero at
    /// the bed to twice the depth average at the surface.
    fn kinematics(&self, p: Vec3) -> Option<(Vec3, Vec3)> {
        let st = self.stencil(p.x, p.z)?;
        let (mut s, mut b) = (0.0, 0.0);
        let mut vel = Vec3::zeros();
        let mut acc = Vec3::zeros();
        for (c, wgt) in st {
            s += wgt * self.surface[c];
            b += wgt * self.bed[c];
            vel += Vec3::new(self.u[c], self.w[c], self.v[c]) * wgt;
            acc += Vec3::new(self.au[c], self.aw[c], self.av[c]) * wgt;
        }
        let h = s - b;
        if p.y > s || p.y < b || h <= WET {
            return None;
        }
        let rise = 2.0 * (p.y - b) / h;
        vel.y *= rise;
        acc.y *= rise;
        Some((vel, acc))
    }
}
