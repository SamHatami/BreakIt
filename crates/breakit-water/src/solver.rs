//! The reference water solver: depth-averaged, non-hydrostatic shallow
//! water on a staggered grid, on the CPU in `f64`.
//!
//! # Model
//!
//! Unknowns per cell column: the free surface `ζ` (absolute y, m), the
//! depth-averaged horizontal velocity `(u, v)` on the cell faces (m/s), the
//! depth-averaged vertical velocity `w` (m/s) and the depth-averaged
//! non-hydrostatic pressure `p` (divided by the density, m²/s²). With the
//! depth `h = ζ − b` over the bed `b`:
//!
//! ```text
//! ∂ζ/∂t + ∇·(h u) = s                                   (volume)
//! ∂u/∂t + u·∇u + g ∇ζ + (1/h)(∇(h p) + 2 p ∇b) = −c_f u|u|/h   (momentum)
//! ∂w/∂t + u·∇w = 2 p / h                                (vertical momentum)
//! h ∇·u + 2 (w − u·∇b) = 0                              (incompressibility)
//! ```
//!
//! This is the one-layer non-hydrostatic system (Stelling & Zijlema 2003;
//! Bristeau et al. 2015). Without `p` it is the shallow-water equations.
//! With it, small waves on depth `d` travel at `c² = g d / (1 + (kd)²/4)`,
//! which follows the exact `c² = g tanh(kd)/k` to 2.5 % in speed up to
//! `kd = 2` and 4 % at `kd = 3`: waves disperse, a paddle makes a wave
//! train instead of a bore, and waves shoal as they should.
//!
//! # Scheme
//!
//! * Hydrostatic part: the scheme of Stelling & Duinmeijer (2003). Upwind
//!   water depths at the faces keep depths positive, so cells flood and dry
//!   without special cases; the momentum-conserving advection gives bores
//!   the right speed and energy loss, which is what a broken wave is.
//! * Non-hydrostatic part: a pressure projection. The predicted velocities
//!   are corrected by the gradient of `p` so that the incompressibility
//!   constraint holds. The system for `p` is symmetric positive definite and
//!   is solved by conjugate gradients with an incomplete Cholesky
//!   preconditioner, warm-started from the last step.
//! * Breaking: where the surface rises faster than `α √(g h)` the pressure
//!   is switched off (the hydrostatic front approximation of SWASH, Smit et
//!   al. 2013), and the front turns into a bore that dissipates energy. The
//!   flag persists while the rise stays above `β √(g h)`.
//!
//! The paddle is a wall whose face velocity is prescribed (linear wavemaker
//! theory: the wall itself does not move through the grid).

use crate::grid::Grid;

/// Bed friction is only worked out in water shallower than this, m: in
/// deeper water it changes the velocity by less than 0.01 % a second.
const FRICTION_DEPTH: f64 = 0.75;

/// Dot product with four running sums, so the additions do not form one
/// long dependency chain (a plain `.sum()` must add in order).
#[inline]
fn dot(a: &[f64], b: &[f64]) -> f64 {
    let mut acc = [0.0; 4];
    let (ca, cb) = (a.chunks_exact(4), b.chunks_exact(4));
    let tail: f64 = ca
        .remainder()
        .iter()
        .zip(cb.remainder())
        .map(|(x, y)| x * y)
        .sum();
    for (x, y) in ca.zip(cb) {
        for k in 0..4 {
            acc[k] += x[k] * y[k];
        }
    }
    (acc[0] + acc[1]) + (acc[2] + acc[3]) + tail
}

/// One side of the grid.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Side {
    /// Low x.
    XMin = 0,
    /// High x.
    XMax = 1,
    /// Low z.
    ZMin = 2,
    /// High z.
    ZMax = 3,
}

/// What happens at a side of the grid.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Boundary {
    /// A fixed wall: no flow through it.
    Wall,
    /// A piston wave paddle: the water at the wall moves with
    /// [`Solver::paddle_velocity`] along the side's axis.
    Piston,
    /// Open water at a still `level` (absolute y, m): waves leave without
    /// reflecting (Sommerfeld condition), on top of a steady `current`.
    Radiating {
        /// Still level outside, absolute y, m.
        level: f64,
        /// Steady current through the side along its axis, m/s.
        current: f64,
    },
}

/// Solver constants.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Config {
    /// Gravity, m/s².
    pub gravity: f64,
    /// Manning roughness of the bed, s/m^(1/3). 0.012 is smooth concrete
    /// or a painted flume.
    pub manning: f64,
    /// Depth below which a face is dry, m.
    pub dry: f64,
    /// Depth below which a cell is hydrostatic, m.
    pub min_dispersive_depth: f64,
    /// False gives the plain shallow-water equations (no dispersion).
    pub dispersive: bool,
    /// Courant number for the substep.
    pub cfl: f64,
    /// Most substeps per call to [`Solver::advance`]; beyond it the water
    /// runs slow instead of unstable.
    pub max_substeps: usize,
    /// Breaking starts where `∂ζ/∂t > alpha √(g h)`.
    pub breaking_alpha: f64,
    /// Breaking persists next to breaking cells while `∂ζ/∂t > beta √(g h)`.
    pub breaking_beta: f64,
    /// Relative residual at which the pressure solve stops.
    pub pressure_tolerance: f64,
    /// Most conjugate-gradient iterations per substep.
    pub pressure_iterations: usize,
    /// Bed slopes steeper than this (rise over run) are treated as steps:
    /// no non-hydrostatic bed term across them.
    pub max_bed_slope: f64,
    /// Speed limit, m/s, as a guard against a blow-up in a thin film.
    pub max_speed: f64,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            gravity: 9.81,
            manning: 0.012,
            dry: 1e-3,
            min_dispersive_depth: 0.02,
            dispersive: true,
            cfl: 0.45,
            max_substeps: 16,
            breaking_alpha: 0.6,
            breaking_beta: 0.3,
            pressure_tolerance: 1e-3,
            pressure_iterations: 60,
            max_bed_slope: 1.0,
            max_speed: 25.0,
        }
    }
}

/// What the last [`Solver::advance`] cost.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct StepStats {
    /// Substeps taken.
    pub substeps: usize,
    /// Conjugate-gradient iterations over all substeps.
    pub pressure_iterations: usize,
    /// Simulated time covered, s (less than asked if the substep limit bit).
    pub advanced: f64,
    /// Cells flagged as breaking at the end.
    pub breaking_cells: usize,
}

/// The reference solver. See the module docs for the model.
#[derive(Clone, Debug)]
pub struct Solver {
    /// The grid.
    pub grid: Grid,
    /// Constants.
    pub config: Config,
    /// Boundary of each [`Side`], indexed by `Side as usize`.
    pub boundaries: [Boundary; 4],
    /// Velocity of [`Boundary::Piston`] sides along their axis, m/s.
    pub paddle_velocity: f64,
    /// The x-velocity a sponge (see [`Self::set_sponge`]) relaxes the
    /// water towards, m/s: the steady current, so the sponge absorbs waves
    /// and lets the current through.
    pub sponge_velocity: f64,
    bed: Vec<f64>,
    zeta: Vec<f64>,
    h: Vec<f64>,
    u: Vec<f64>,
    v: Vec<f64>,
    w: Vec<f64>,
    p: Vec<f64>,
    hu: Vec<f64>,
    hv: Vec<f64>,
    un: Vec<f64>,
    vn: Vec<f64>,
    wn: Vec<f64>,
    zn: Vec<f64>,
    sx: Vec<f64>,
    sz: Vec<f64>,
    minv_x: Vec<f64>,
    minv_z: Vec<f64>,
    gu: Vec<f64>,
    gv: Vec<f64>,
    active: Vec<bool>,
    breaking: Vec<bool>,
    break_strength: Vec<f64>,
    blockage: Vec<f64>,
    block_u: Vec<f64>,
    block_v: Vec<f64>,
    source: Vec<f64>,
    sponge: Vec<f64>,
    cg_r: Vec<f64>,
    cg_z: Vec<f64>,
    cg_d: Vec<f64>,
    cg_q: Vec<f64>,
    cg_diag: Vec<f64>,
    ax: Vec<f64>,
    az: Vec<f64>,
    lz: Vec<f64>,
    fx: Vec<f64>,
    bx: Vec<f64>,
    bz: Vec<f64>,
    precon: Vec<f64>,
    time: f64,
    stats: StepStats,
}

impl Solver {
    /// A dry, flat grid at bed level 0 with walls all round.
    pub fn new(grid: Grid, config: Config) -> Self {
        let n = grid.cells();
        let (nu, nv) = (grid.u_faces(), grid.v_faces());
        let c = || vec![0.0; n];
        // Padded by a row at both ends for the pressure stencil.
        let pad = || vec![0.0; n + 2 * grid.nx];
        Self {
            grid,
            config,
            boundaries: [Boundary::Wall; 4],
            paddle_velocity: 0.0,
            sponge_velocity: 0.0,
            bed: c(),
            zeta: c(),
            h: c(),
            u: vec![0.0; nu],
            v: vec![0.0; nv],
            w: c(),
            p: c(),
            hu: vec![0.0; nu],
            hv: vec![0.0; nv],
            un: vec![0.0; nu],
            vn: vec![0.0; nv],
            wn: c(),
            zn: c(),
            sx: vec![0.0; nu],
            sz: vec![0.0; nv],
            minv_x: vec![0.0; nu],
            minv_z: vec![0.0; nv],
            gu: vec![0.0; nu],
            gv: vec![0.0; nv],
            active: vec![false; n],
            breaking: vec![false; n],
            break_strength: c(),
            blockage: c(),
            block_u: c(),
            block_v: c(),
            source: c(),
            sponge: c(),
            cg_r: c(),
            cg_z: pad(),
            cg_d: pad(),
            cg_q: c(),
            cg_diag: c(),
            ax: pad(),
            az: pad(),
            lz: pad(),
            fx: pad(),
            bx: pad(),
            bz: pad(),
            precon: pad(),
            time: 0.0,
            stats: StepStats::default(),
        }
    }

    /// Sets the boundary of one side.
    pub fn set_boundary(&mut self, side: Side, b: Boundary) {
        self.boundaries[side as usize] = b;
    }

    /// Sets the bed level (absolute y, m) of every cell from its centre
    /// position `(x, z)`. Water already there keeps its depth where the bed
    /// rises above it.
    pub fn set_bed(&mut self, mut f: impl FnMut(f64, f64) -> f64) {
        let g = self.grid;
        for j in 0..g.nz {
            for i in 0..g.nx {
                let (x, z) = g.centre(i, j);
                let c = g.c(i, j);
                self.bed[c] = f(x, z);
                self.zeta[c] = self.zeta[c].max(self.bed[c]);
            }
        }
        self.refresh_slopes();
    }

    fn refresh_slopes(&mut self) {
        let g = self.grid;
        let smax = self.config.max_bed_slope;
        self.sx.fill(0.0);
        self.sz.fill(0.0);
        for j in 0..g.nz {
            for i in 1..g.nx {
                let s = (self.bed[g.c(i, j)] - self.bed[g.c(i - 1, j)]) / g.dx;
                self.sx[g.u(i, j)] = if s.abs() <= smax { s } else { 0.0 };
            }
        }
        for j in 1..g.nz {
            for i in 0..g.nx {
                let s = (self.bed[g.c(i, j)] - self.bed[g.c(i, j - 1)]) / g.dx;
                self.sz[g.v(i, j)] = if s.abs() <= smax { s } else { 0.0 };
            }
        }
    }

    /// Fills every cell whose bed is below `level` up to `level` (absolute
    /// y, m) and stops the water. Cells for which `inside(x, z)` is false
    /// are left alone.
    pub fn fill_to(&mut self, level: f64, mut inside: impl FnMut(f64, f64) -> bool) {
        let g = self.grid;
        for j in 0..g.nz {
            for i in 0..g.nx {
                let (x, z) = g.centre(i, j);
                if inside(x, z) {
                    let c = g.c(i, j);
                    self.zeta[c] = level.max(self.bed[c]);
                }
            }
        }
        self.u.fill(0.0);
        self.v.fill(0.0);
        self.w.fill(0.0);
        self.p.fill(0.0);
        self.refresh_depth();
    }

    /// Sets the surface of every cell to `f(x, z)` (absolute y, m), clamped
    /// to the bed. Velocities are kept.
    pub fn set_surface(&mut self, mut f: impl FnMut(f64, f64) -> f64) {
        let g = self.grid;
        for j in 0..g.nz {
            for i in 0..g.nx {
                let (x, z) = g.centre(i, j);
                let c = g.c(i, j);
                self.zeta[c] = f(x, z).max(self.bed[c]);
            }
        }
        self.refresh_depth();
    }

    /// Sets the x-velocity of every x-face from its position `(x, z)`, m/s.
    pub fn set_velocity_x(&mut self, mut f: impl FnMut(f64, f64) -> f64) {
        let g = self.grid;
        for j in 0..g.nz {
            for i in 1..g.nx {
                let x = g.x0 + i as f64 * g.dx;
                let z = g.z0 + (j as f64 + 0.5) * g.dx;
                self.u[g.u(i, j)] = f(x, z);
            }
        }
    }

    fn refresh_depth(&mut self) {
        for c in 0..self.h.len() {
            self.h[c] = (self.zeta[c] - self.bed[c]).max(0.0);
        }
    }

    /// Volume source of every cell, m of depth per second (a pump inlet or
    /// rain; negative drains).
    pub fn set_source(&mut self, mut f: impl FnMut(f64, f64) -> f64) {
        let g = self.grid;
        for j in 0..g.nz {
            for i in 0..g.nx {
                let (x, z) = g.centre(i, j);
                self.source[g.c(i, j)] = f(x, z);
            }
        }
    }

    /// Damping rate of every cell, 1/s: a sponge that absorbs waves where
    /// it is non-zero (velocities relax to [`Self::sponge_velocity`] along x
    /// and to zero along z at this rate).
    pub fn set_sponge(&mut self, mut f: impl FnMut(f64, f64) -> f64) {
        let g = self.grid;
        for j in 0..g.nz {
            for i in 0..g.nx {
                let (x, z) = g.centre(i, j);
                self.sponge[g.c(i, j)] = f(x, z).max(0.0);
            }
        }
    }

    /// Removes all blockage (see [`Self::add_blockage`]).
    pub fn clear_blockage(&mut self) {
        self.blockage.fill(0.0);
        self.block_u.fill(0.0);
        self.block_v.fill(0.0);
    }

    /// Adds a slender obstacle in the cell at `(x, z)`: `cd_area` is its
    /// drag area `½ C_d D L_wet` (m²) and `(vx, vz)` its own horizontal
    /// velocity (m/s). The water in that cell is slowed towards the
    /// obstacle's velocity by the same quadratic drag the obstacle feels,
    /// which is what makes a wake behind a pile.
    pub fn add_blockage(&mut self, x: f64, z: f64, cd_area: f64, vx: f64, vz: f64) {
        let Some((i, j)) = self.grid.cell_at(x, z) else {
            return;
        };
        let c = self.grid.c(i, j);
        let k = cd_area / (self.grid.dx * self.grid.dx);
        let total = self.blockage[c] + k;
        if total > 0.0 {
            self.block_u[c] = (self.block_u[c] * self.blockage[c] + vx * k) / total;
            self.block_v[c] = (self.block_v[c] * self.blockage[c] + vz * k) / total;
        }
        self.blockage[c] = total;
    }

    /// Simulated time, s.
    pub fn time(&self) -> f64 {
        self.time
    }

    /// Cost of the last [`Self::advance`].
    pub fn stats(&self) -> StepStats {
        self.stats
    }

    /// Bed level per cell, absolute y, m.
    pub fn bed(&self) -> &[f64] {
        &self.bed
    }

    /// Free surface per cell, absolute y, m (equal to the bed where dry).
    pub fn surface(&self) -> &[f64] {
        &self.zeta
    }

    /// Water depth per cell, m.
    pub fn depth(&self) -> &[f64] {
        &self.h
    }

    /// Velocity on the x-faces, m/s (see [`Grid::u`]).
    pub fn face_u(&self) -> &[f64] {
        &self.u
    }

    /// Velocity on the z-faces, m/s (see [`Grid::v`]).
    pub fn face_v(&self) -> &[f64] {
        &self.v
    }

    /// Depth-averaged vertical velocity per cell, m/s.
    pub fn vertical(&self) -> &[f64] {
        &self.w
    }

    /// Non-hydrostatic pressure over density per cell, m²/s².
    pub fn pressure(&self) -> &[f64] {
        &self.p
    }

    /// Which cells are breaking.
    pub fn breaking(&self) -> &[bool] {
        &self.breaking
    }

    /// How hard each cell is breaking: surface rise rate over `√(g h)`,
    /// 0 where not breaking.
    pub fn break_strength(&self) -> &[f64] {
        &self.break_strength
    }

    /// Blockage per cell (drag area over cell area, dimensionless).
    pub fn blockage(&self) -> &[f64] {
        &self.blockage
    }

    /// Depth-averaged velocity at the centre of cell `(i, j)`, m/s.
    pub fn cell_velocity(&self, i: usize, j: usize) -> (f64, f64) {
        let g = &self.grid;
        (
            0.5 * (self.u[g.u(i, j)] + self.u[g.u(i + 1, j)]),
            0.5 * (self.v[g.v(i, j)] + self.v[g.v(i, j + 1)]),
        )
    }

    /// Water volume on the grid, m³.
    pub fn volume(&self) -> f64 {
        self.h.iter().sum::<f64>() * self.grid.dx * self.grid.dx
    }

    /// Mechanical energy of the water over the density, m⁵/s²: potential
    /// energy about y = 0 plus kinetic energy.
    pub fn energy(&self) -> f64 {
        let g = &self.grid;
        let mut e = 0.0;
        for j in 0..g.nz {
            for i in 0..g.nx {
                let c = g.c(i, j);
                let h = self.h[c];
                if h <= 0.0 {
                    continue;
                }
                let (uc, vc) = self.cell_velocity(i, j);
                e += 0.5 * self.config.gravity * (self.zeta[c].powi(2) - self.bed[c].powi(2))
                    + 0.5 * h * (uc * uc + vc * vc + self.w[c] * self.w[c]);
            }
        }
        e * g.dx * g.dx
    }

    /// The largest stable substep for the current state, s.
    pub fn stable_dt(&self) -> f64 {
        let g = &self.grid;
        let grav = self.config.gravity;
        let mut smax: f64 = 1e-6;
        for j in 0..g.nz {
            for i in 0..g.nx {
                let h = self.h[g.c(i, j)];
                if h <= self.config.dry {
                    continue;
                }
                let ux = self.u[g.u(i, j)].abs().max(self.u[g.u(i + 1, j)].abs());
                let vz = self.v[g.v(i, j)].abs().max(self.v[g.v(i, j + 1)].abs());
                smax = smax.max(ux.max(vz) + (grav * h).sqrt());
            }
        }
        // Two dimensions share the Courant number.
        let dims = if g.nx > 1 && g.nz > 1 {
            2.0_f64.sqrt()
        } else {
            1.0
        };
        self.config.cfl * g.dx / (smax * dims)
    }

    /// Advances the water by `dt` seconds in as many substeps as stability
    /// needs.
    pub fn advance(&mut self, dt: f64) -> StepStats {
        let mut stats = StepStats::default();
        let mut left = dt;
        while left > 1e-12 && stats.substeps < self.config.max_substeps {
            let stable = self.stable_dt();
            let remaining = self.config.max_substeps - stats.substeps;
            let n = (left / stable).ceil().max(1.0);
            let h = if n as usize <= remaining {
                left / n
            } else {
                stable
            };
            stats.pressure_iterations += self.substep(h);
            stats.substeps += 1;
            stats.advanced += h;
            left -= h;
        }
        stats.breaking_cells = self.breaking.iter().filter(|b| **b).count();
        self.stats = stats;
        stats
    }

    /// Upwind depths at the faces for the velocities `u`, `v`.
    fn face_depths(&mut self, new: bool) {
        let g = self.grid;
        let (u, v) = if new {
            (&self.un, &self.vn)
        } else {
            (&self.u, &self.v)
        };
        for j in 0..g.nz {
            for i in 1..g.nx {
                let (l, r) = (g.c(i - 1, j), g.c(i, j));
                let f = g.u(i, j);
                let bf = self.bed[l].max(self.bed[r]);
                let zf = if u[f] > 0.0 {
                    self.zeta[l]
                } else if u[f] < 0.0 {
                    self.zeta[r]
                } else {
                    self.zeta[l].max(self.zeta[r])
                };
                self.hu[f] = (zf - bf).max(0.0);
            }
            for (i, c, side) in [
                (0, g.c(0, j), Side::XMin),
                (g.nx, g.c(g.nx - 1, j), Side::XMax),
            ] {
                self.hu[g.u(i, j)] = match self.boundaries[side as usize] {
                    Boundary::Wall => 0.0,
                    _ => self.h[c],
                };
            }
        }
        for i in 0..g.nx {
            for j in 1..g.nz {
                let (l, r) = (g.c(i, j - 1), g.c(i, j));
                let f = g.v(i, j);
                let bf = self.bed[l].max(self.bed[r]);
                let zf = if v[f] > 0.0 {
                    self.zeta[l]
                } else if v[f] < 0.0 {
                    self.zeta[r]
                } else {
                    self.zeta[l].max(self.zeta[r])
                };
                self.hv[f] = (zf - bf).max(0.0);
            }
            for (j, c, side) in [
                (0, g.c(i, 0), Side::ZMin),
                (g.nz, g.c(i, g.nz - 1), Side::ZMax),
            ] {
                self.hv[g.v(i, j)] = match self.boundaries[side as usize] {
                    Boundary::Wall => 0.0,
                    _ => self.h[c],
                };
            }
        }
    }

    /// Velocity through a boundary face next to cell `c`. `outward` is +1
    /// on the high sides and -1 on the low sides.
    fn boundary_velocity(&self, side: Side, c: usize, outward: f64) -> f64 {
        let h = self.h[c];
        match self.boundaries[side as usize] {
            Boundary::Wall => 0.0,
            _ if h <= self.config.dry => 0.0,
            Boundary::Piston => self.paddle_velocity,
            Boundary::Radiating { level, current } => {
                current + outward * (self.config.gravity / h).sqrt() * (self.zeta[c] - level)
            }
        }
    }

    /// One substep. Returns the pressure iterations used.
    fn substep(&mut self, dt: f64) -> usize {
        let g = self.grid;
        let cfg = self.config;
        let (nx, nz, dx) = (g.nx, g.nz, g.dx);
        let grav = cfg.gravity;
        self.refresh_depth();
        self.face_depths(false);

        // Predictor: gravity, advection, friction, blockage, sponge.
        let fric = grav * cfg.manning * cfg.manning;
        for j in 0..nz {
            for i in 1..nx {
                let f = g.u(i, j);
                if self.hu[f] <= cfg.dry {
                    self.un[f] = 0.0;
                    continue;
                }
                let (l, r) = (g.c(i - 1, j), g.c(i, j));
                let uf = self.u[f];
                let hbar = (0.5 * (self.h[l] + self.h[r])).max(self.hu[f]);
                let q = |k: usize| self.hu[k] * self.u[k];
                let ql = 0.5 * (q(f - 1) + q(f));
                let qr = 0.5 * (q(f) + q(f + 1));
                let mut adv = 0.0;
                if ql > 0.0 {
                    adv += ql * (uf - self.u[f - 1]);
                }
                if qr < 0.0 {
                    adv += qr * (self.u[f + 1] - uf);
                }
                let qv = |k: usize| self.hv[k] * self.v[k];
                let qb = 0.5 * (qv(g.v(i - 1, j)) + qv(g.v(i, j)));
                let qt = 0.5 * (qv(g.v(i - 1, j + 1)) + qv(g.v(i, j + 1)));
                if qb > 0.0 && j > 0 {
                    adv += qb * (uf - self.u[g.u(i, j - 1)]);
                }
                if qt < 0.0 && j + 1 < nz {
                    adv += qt * (self.u[g.u(i, j + 1)] - uf);
                }
                let mut us =
                    uf - dt * (grav * (self.zeta[r] - self.zeta[l]) / dx + adv / (hbar * dx));
                let vbar = 0.25
                    * (self.v[g.v(i - 1, j)]
                        + self.v[g.v(i, j)]
                        + self.v[g.v(i - 1, j + 1)]
                        + self.v[g.v(i, j + 1)]);
                let speed = (uf * uf + vbar * vbar).sqrt();
                let hf = self.hu[f];
                if hf < FRICTION_DEPTH {
                    us /= 1.0 + dt * fric * speed / (hf * hf.cbrt());
                }
                let kap = 0.5 * (self.blockage[l] + self.blockage[r]) / hbar;
                if kap > 0.0 {
                    let bu = 0.5 * (self.block_u[l] + self.block_u[r]);
                    let bv = 0.5 * (self.block_v[l] + self.block_v[r]);
                    let rel = ((uf - bu).powi(2) + (vbar - bv).powi(2)).sqrt();
                    us = (us + dt * kap * rel * bu) / (1.0 + dt * kap * rel);
                }
                let sp = dt * 0.5 * (self.sponge[l] + self.sponge[r]);
                us = (us + sp * self.sponge_velocity) / (1.0 + sp);
                self.un[f] = us.clamp(-cfg.max_speed, cfg.max_speed);
            }
            self.un[g.u(0, j)] = self.boundary_velocity(Side::XMin, g.c(0, j), -1.0);
            self.un[g.u(nx, j)] = self.boundary_velocity(Side::XMax, g.c(nx - 1, j), 1.0);
        }
        for i in 0..nx {
            for j in 1..nz {
                let f = g.v(i, j);
                if self.hv[f] <= cfg.dry {
                    self.vn[f] = 0.0;
                    continue;
                }
                let (l, r) = (g.c(i, j - 1), g.c(i, j));
                let vf = self.v[f];
                let hbar = (0.5 * (self.h[l] + self.h[r])).max(self.hv[f]);
                let q = |k: usize| self.hv[k] * self.v[k];
                let ql = 0.5 * (q(g.v(i, j - 1)) + q(f));
                let qr = 0.5 * (q(f) + q(g.v(i, j + 1)));
                let mut adv = 0.0;
                if ql > 0.0 {
                    adv += ql * (vf - self.v[g.v(i, j - 1)]);
                }
                if qr < 0.0 {
                    adv += qr * (self.v[g.v(i, j + 1)] - vf);
                }
                let qu = |k: usize| self.hu[k] * self.u[k];
                let qb = 0.5 * (qu(g.u(i, j - 1)) + qu(g.u(i, j)));
                let qt = 0.5 * (qu(g.u(i + 1, j - 1)) + qu(g.u(i + 1, j)));
                if qb > 0.0 && i > 0 {
                    adv += qb * (vf - self.v[g.v(i - 1, j)]);
                }
                if qt < 0.0 && i + 1 < nx {
                    adv += qt * (self.v[g.v(i + 1, j)] - vf);
                }
                let mut vs =
                    vf - dt * (grav * (self.zeta[r] - self.zeta[l]) / dx + adv / (hbar * dx));
                let ubar = 0.25
                    * (self.u[g.u(i, j - 1)]
                        + self.u[g.u(i + 1, j - 1)]
                        + self.u[g.u(i, j)]
                        + self.u[g.u(i + 1, j)]);
                let speed = (vf * vf + ubar * ubar).sqrt();
                let hf = self.hv[f];
                if hf < FRICTION_DEPTH {
                    vs /= 1.0 + dt * fric * speed / (hf * hf.cbrt());
                }
                let kap = 0.5 * (self.blockage[l] + self.blockage[r]) / hbar;
                if kap > 0.0 {
                    let bu = 0.5 * (self.block_u[l] + self.block_u[r]);
                    let bv = 0.5 * (self.block_v[l] + self.block_v[r]);
                    let rel = ((ubar - bu).powi(2) + (vf - bv).powi(2)).sqrt();
                    vs = (vs + dt * kap * rel * bv) / (1.0 + dt * kap * rel);
                }
                vs /= 1.0 + dt * 0.5 * (self.sponge[l] + self.sponge[r]);
                self.vn[f] = vs.clamp(-cfg.max_speed, cfg.max_speed);
            }
            self.vn[g.v(i, 0)] = self.boundary_velocity(Side::ZMin, g.c(i, 0), -1.0);
            self.vn[g.v(i, nz)] = self.boundary_velocity(Side::ZMax, g.c(i, nz - 1), 1.0);
        }

        // Vertical velocity: upwind advection.
        let mut iterations = 0;
        if cfg.dispersive {
            for j in 0..nz {
                for i in 0..nx {
                    let c = g.c(i, j);
                    if self.h[c] <= cfg.dry {
                        self.wn[c] = 0.0;
                        continue;
                    }
                    let (uc, vc) = self.cell_velocity(i, j);
                    let wc = self.w[c];
                    let mut d = 0.0;
                    if uc > 0.0 && i > 0 {
                        d += uc * (wc - self.w[c - 1]);
                    } else if uc < 0.0 && i + 1 < nx {
                        d += uc * (self.w[c + 1] - wc);
                    }
                    if vc > 0.0 && j > 0 {
                        d += vc * (wc - self.w[c - nx]);
                    } else if vc < 0.0 && j + 1 < nz {
                        d += vc * (self.w[c + nx] - wc);
                    }
                    self.wn[c] = wc - dt * d / dx;
                }
            }
            iterations = self.project(dt);
        }

        // Volume: upwind depths for the corrected velocities, with outflow
        // limited to what the cell holds.
        self.face_depths(true);
        for j in 0..nz {
            for i in 0..nx {
                let c = g.c(i, j);
                let (fl, fr, fb, ft) = (g.u(i, j), g.u(i + 1, j), g.v(i, j), g.v(i, j + 1));
                let out = (self.hu[fr] * self.un[fr]).max(0.0)
                    + (-self.hu[fl] * self.un[fl]).max(0.0)
                    + (self.hv[ft] * self.vn[ft]).max(0.0)
                    + (-self.hv[fb] * self.vn[fb]).max(0.0);
                let has = self.h[c] * dx / dt;
                if out > has && out > 0.0 {
                    let s = has / out;
                    if self.un[fr] > 0.0 {
                        self.un[fr] *= s;
                    }
                    if self.un[fl] < 0.0 {
                        self.un[fl] *= s;
                    }
                    if self.vn[ft] > 0.0 {
                        self.vn[ft] *= s;
                    }
                    if self.vn[fb] < 0.0 {
                        self.vn[fb] *= s;
                    }
                }
            }
        }
        for j in 0..nz {
            for i in 0..nx {
                let c = g.c(i, j);
                let div = self.hu[g.u(i + 1, j)] * self.un[g.u(i + 1, j)]
                    - self.hu[g.u(i, j)] * self.un[g.u(i, j)]
                    + self.hv[g.v(i, j + 1)] * self.vn[g.v(i, j + 1)]
                    - self.hv[g.v(i, j)] * self.vn[g.v(i, j)];
                let z = self.zeta[c] - dt * div / dx + dt * self.source[c];
                self.zn[c] = z.max(self.bed[c]);
            }
        }

        // Breaking flags from the surface rise rate.
        if cfg.dispersive {
            for j in 0..nz {
                for i in 0..nx {
                    let c = g.c(i, j);
                    let h = self.zn[c] - self.bed[c];
                    let mut on = false;
                    let mut strength = 0.0;
                    if h > cfg.min_dispersive_depth {
                        let rate = (self.zn[c] - self.zeta[c]) / dt - self.source[c];
                        let crit = (grav * h).sqrt();
                        let near = self.breaking[c]
                            || (i > 0 && self.breaking[c - 1])
                            || (i + 1 < nx && self.breaking[c + 1])
                            || (j > 0 && self.breaking[c - nx])
                            || (j + 1 < nz && self.breaking[c + nx]);
                        on = rate > cfg.breaking_alpha * crit
                            || (near && rate > cfg.breaking_beta * crit);
                        if on {
                            strength = rate / crit;
                        }
                    }
                    self.active[c] = on;
                    self.break_strength[c] = strength;
                }
            }
            self.breaking.copy_from_slice(&self.active);
        }

        std::mem::swap(&mut self.zeta, &mut self.zn);
        std::mem::swap(&mut self.u, &mut self.un);
        std::mem::swap(&mut self.v, &mut self.vn);
        if cfg.dispersive {
            std::mem::swap(&mut self.w, &mut self.wn);
        }
        self.refresh_depth();
        self.time += dt;
        iterations
    }

    /// `C(u, v, w)` at cell `(i, j)`: the incompressibility residual
    /// `h ∇·u − 2 u·∇b + 2 w`, with `w2 = 2 w`.
    #[inline]
    fn constraint(&self, u: &[f64], v: &[f64], w2: f64, i: usize, j: usize) -> f64 {
        let g = &self.grid;
        let c = g.c(i, j);
        let (fl, fr, fb, ft) = (g.u(i, j), g.u(i + 1, j), g.v(i, j), g.v(i, j + 1));
        self.h[c] * (u[fr] - u[fl] + v[ft] - v[fb]) / g.dx
            - (u[fr] * self.sx[fr] + u[fl] * self.sx[fl])
            - (v[ft] * self.sz[ft] + v[fb] * self.sz[fb])
            + w2
    }

    /// `gu, gv = M⁻¹ Cᵀ p`: the velocity correction per unit time for the
    /// pressure `p` (0 on inactive cells).
    fn pressure_gradient(&mut self) {
        let g = self.grid;
        let (nx, nz, dx) = (g.nx, g.nz, g.dx);
        let p = &self.p;
        for j in 0..nz {
            for i in 1..nx {
                let f = g.u(i, j);
                let (l, r) = (g.c(i - 1, j), g.c(i, j));
                self.gu[f] = self.minv_x[f]
                    * ((self.h[l] * p[l] - self.h[r] * p[r]) / dx - self.sx[f] * (p[l] + p[r]));
            }
        }
        for j in 1..nz {
            for i in 0..nx {
                let f = g.v(i, j);
                let (l, r) = (g.c(i, j - 1), g.c(i, j));
                self.gv[f] = self.minv_z[f]
                    * ((self.h[l] * p[l] - self.h[r] * p[r]) / dx - self.sz[f] * (p[l] + p[r]));
            }
        }
    }

    /// `cg_q = A cg_d` for the assembled five-point matrix `A = C M⁻¹ Cᵀ`.
    /// `cg_d`, `ax` and `az` are padded by `nx` entries at both ends so the
    /// stencil needs no edge cases; inactive rows are identity rows.
    fn apply_matrix(&mut self) {
        let nx = self.grid.nx;
        let n = self.h.len();
        let (x, ax, az) = (
            &self.cg_d[..n + 2 * nx],
            &self.ax[..n + 2 * nx],
            &self.az[..n + 2 * nx],
        );
        let (diag, q) = (&self.cg_diag[..n], &mut self.cg_q[..n]);
        for c in 0..n {
            let k = c + nx;
            q[c] = diag[c] * x[k]
                + ax[k] * x[k + 1]
                + ax[k - 1] * x[k - 1]
                + az[k] * x[k + nx]
                + az[k - nx] * x[k - nx];
        }
    }

    /// `cg_z = P⁻¹ cg_r` with the modified incomplete Cholesky factor
    /// (`precon` its inverse diagonal, `lx`, `lz` its off-diagonals times
    /// the inverse diagonals they meet in the two sweeps; all padded like
    /// `cg_d`). The sweeps are a chain through every cell, so the chain is
    /// kept to one multiply and one subtract per cell.
    fn precondition(&mut self) {
        let nx = self.grid.nx;
        let n = self.h.len();
        let m = n + 2 * nx;
        let (fx, fz, bx, bz) = (&self.fx[..m], &self.lz[..m], &self.bx[..m], &self.bz[..m]);
        let pre = &self.precon[..m];
        let (r, z) = (&self.cg_r[..n], &mut self.cg_z[..m]);
        for c in 0..n {
            let k = c + nx;
            z[k] = (r[c] - fz[k - nx] * z[k - nx]) * pre[k] - fx[k] * z[k - 1];
        }
        for c in (0..n).rev() {
            let k = c + nx;
            z[k] = (z[k] * pre[k] - bz[k] * z[k + nx]) - bx[k] * z[k + 1];
        }
    }

    /// Pressure projection on the predicted `un`, `vn`, `wn`.
    fn project(&mut self, dt: f64) -> usize {
        let g = self.grid;
        let cfg = self.config;
        let (nx, nz, dx) = (g.nx, g.nz, g.dx);
        let mut any = false;
        for c in 0..self.h.len() {
            let a = self.h[c] > cfg.min_dispersive_depth && !self.breaking[c];
            self.active[c] = a;
            any |= a;
            if !a {
                self.p[c] = 0.0;
            }
        }
        // Inverse face masses; boundary and dry faces are not corrected.
        self.minv_x.fill(0.0);
        self.minv_z.fill(0.0);
        self.gu.fill(0.0);
        self.gv.fill(0.0);
        for j in 0..nz {
            for i in 1..nx {
                let f = g.u(i, j);
                if self.hu[f] > cfg.dry {
                    let hb = 0.5 * (self.h[g.c(i - 1, j)] + self.h[g.c(i, j)]);
                    self.minv_x[f] = 1.0 / hb.max(cfg.dry);
                }
            }
        }
        for j in 1..nz {
            for i in 0..nx {
                let f = g.v(i, j);
                if self.hv[f] > cfg.dry {
                    let hb = 0.5 * (self.h[g.c(i, j - 1)] + self.h[g.c(i, j)]);
                    self.minv_z[f] = 1.0 / hb.max(cfg.dry);
                }
            }
        }
        let mut iterations = 0;
        if any {
            // Assemble A = C M⁻¹ Cᵀ (five-point, symmetric positive
            // definite) and the right-hand side −C(u*, w*)/dt.
            let o = nx;
            let n = self.h.len();
            let mut rhs_norm = 0.0;
            for j in 0..nz {
                for i in 0..nx {
                    let c = g.c(i, j);
                    self.ax[o + c] = 0.0;
                    self.az[o + c] = 0.0;
                    if !self.active[c] {
                        self.cg_r[c] = 0.0;
                        self.cg_diag[c] = 1.0;
                        continue;
                    }
                    let b = -self.constraint(&self.un, &self.vn, 2.0 * self.wn[c], i, j) / dt;
                    self.cg_r[c] = b;
                    rhs_norm += b * b;
                    let h = self.h[c];
                    let (fl, fr, fb, ft) = (g.u(i, j), g.u(i + 1, j), g.v(i, j), g.v(i, j + 1));
                    self.cg_diag[c] = (h / dx - self.sx[fr]).powi(2) * self.minv_x[fr]
                        + (h / dx + self.sx[fl]).powi(2) * self.minv_x[fl]
                        + (h / dx - self.sz[ft]).powi(2) * self.minv_z[ft]
                        + (h / dx + self.sz[fb]).powi(2) * self.minv_z[fb]
                        + 4.0 / h;
                    if i + 1 < nx && self.active[c + 1] {
                        self.ax[o + c] = (h / dx - self.sx[fr])
                            * (-self.h[c + 1] / dx - self.sx[fr])
                            * self.minv_x[fr];
                    }
                    if j + 1 < nz && self.active[c + nx] {
                        self.az[o + c] = (h / dx - self.sz[ft])
                            * (-self.h[c + nx] / dx - self.sz[ft])
                            * self.minv_z[ft];
                    }
                }
            }
            if rhs_norm > 0.0 {
                // Modified incomplete Cholesky, MIC(0).
                for c in 0..n {
                    let k = o + c;
                    if !self.active[c] {
                        self.precon[k] = 0.0;
                        self.lz[k] = 0.0;
                        self.fx[k] = 0.0;
                        self.bx[k] = 0.0;
                        self.bz[k] = 0.0;
                        continue;
                    }
                    let mut e = self.cg_diag[c];
                    let (a, pl) = (self.ax[k - 1], self.precon[k - 1]);
                    e -= (a * pl).powi(2) + 0.97 * a * self.az[k - 1] * pl * pl;
                    let (a, pl) = (self.az[k - nx], self.precon[k - nx]);
                    e -= (a * pl).powi(2) + 0.97 * a * self.ax[k - nx] * pl * pl;
                    if e < 0.25 * self.cg_diag[c] {
                        e = self.cg_diag[c];
                    }
                    let pre = 1.0 / e.sqrt();
                    self.precon[k] = pre;
                    // Forward: z = (r − Lz z↓) pre − (Lx← pre) z←.
                    self.lz[k] = self.az[k] * pre;
                    self.fx[k] = self.ax[k - 1] * self.precon[k - 1] * pre;
                    // Backward: z = z pre − (Lz pre) z↑ − (Lx pre) z→.
                    self.bx[k] = self.ax[k] * pre * pre;
                    self.bz[k] = self.az[k] * pre * pre;
                }
                // r = b − A p0 (warm start).
                self.cg_d[o..o + n].copy_from_slice(&self.p);
                self.apply_matrix();
                for c in 0..n {
                    self.cg_r[c] -= self.cg_q[c];
                }
                self.precondition();
                let mut rz = 0.0;
                for c in 0..n {
                    self.cg_d[o + c] = self.cg_z[o + c];
                    rz += self.cg_r[c] * self.cg_z[o + c];
                }
                let stop = cfg.pressure_tolerance.powi(2) * rhs_norm;
                let mut rr = dot(&self.cg_r, &self.cg_r);
                while iterations < cfg.pressure_iterations && rr > stop && rz > 0.0 {
                    self.apply_matrix();
                    let d = &self.cg_d[o..o + n];
                    let dq = dot(d, &self.cg_q);
                    if dq <= 0.0 {
                        break;
                    }
                    let alpha = rz / dq;
                    for c in 0..n {
                        self.p[c] += alpha * d[c];
                        self.cg_r[c] -= alpha * self.cg_q[c];
                    }
                    rr = dot(&self.cg_r, &self.cg_r);
                    self.precondition();
                    let z = &self.cg_z[o..o + n];
                    let rz_new = dot(&self.cg_r, z);
                    let beta = rz_new / rz;
                    rz = rz_new;
                    for (d, z) in self.cg_d[o..o + n].iter_mut().zip(z) {
                        *d = *z + beta * *d;
                    }
                    iterations += 1;
                }
            }
            // Correct the velocities with M⁻¹ Cᵀ p.
            self.pressure_gradient();
            for f in 0..self.un.len() {
                self.un[f] += dt * self.gu[f];
            }
            for f in 0..self.vn.len() {
                self.vn[f] += dt * self.gv[f];
            }
        }
        // Vertical velocity: from the pressure where active, from the
        // constraint elsewhere so it is consistent when the cell rejoins.
        for j in 0..nz {
            for i in 0..nx {
                let c = g.c(i, j);
                let h = self.h[c];
                if self.active[c] {
                    self.wn[c] += dt * 2.0 * self.p[c] / h;
                } else if h > cfg.dry {
                    self.wn[c] = -0.5 * self.constraint(&self.un, &self.vn, 0.0, i, j);
                } else {
                    self.wn[c] = 0.0;
                }
            }
        }
        iterations
    }
}
