//! The grid the water lives on: square cells in the horizontal plane, with
//! depths at cell centres and velocities on cell faces (a staggered C grid).

/// A uniform grid of square cells in the x-z plane.
///
/// Cell `(i, j)` covers `x0 + i dx .. x0 + (i+1) dx` and
/// `z0 + j dx .. z0 + (j+1) dx`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Grid {
    /// Cells along x.
    pub nx: usize,
    /// Cells along z.
    pub nz: usize,
    /// Cell size, m.
    pub dx: f64,
    /// World x of the low-x edge, m.
    pub x0: f64,
    /// World z of the low-z edge, m.
    pub z0: f64,
}

impl Grid {
    /// A grid of `nx` by `nz` cells of size `dx` with its low corner at
    /// `(x0, z0)`.
    pub fn new(nx: usize, nz: usize, dx: f64, x0: f64, z0: f64) -> Self {
        assert!(
            nx >= 1 && nz >= 1 && dx > 0.0,
            "grid needs cells and a positive cell size"
        );
        Self { nx, nz, dx, x0, z0 }
    }

    /// The grid covering `x_min..x_max` by `z_min..z_max` with cells of
    /// about `dx` (the extent is kept, the cell size is used as given and
    /// the cell counts round up).
    pub fn covering(x: (f64, f64), z: (f64, f64), dx: f64) -> Self {
        let nx = ((x.1 - x.0) / dx - 1e-9).ceil().max(1.0) as usize;
        let nz = ((z.1 - z.0) / dx - 1e-9).ceil().max(1.0) as usize;
        Self::new(nx, nz, dx, x.0, z.0)
    }

    /// Number of cells.
    pub fn cells(&self) -> usize {
        self.nx * self.nz
    }

    /// Index of cell `(i, j)`.
    #[inline]
    pub fn c(&self, i: usize, j: usize) -> usize {
        j * self.nx + i
    }

    /// Index of the x-face on the low-x side of cell `(i, j)`; `i` runs to
    /// `nx` inclusive.
    #[inline]
    pub fn u(&self, i: usize, j: usize) -> usize {
        j * (self.nx + 1) + i
    }

    /// Index of the z-face on the low-z side of cell `(i, j)`; `j` runs to
    /// `nz` inclusive.
    #[inline]
    pub fn v(&self, i: usize, j: usize) -> usize {
        j * self.nx + i
    }

    /// Number of x-faces.
    pub fn u_faces(&self) -> usize {
        (self.nx + 1) * self.nz
    }

    /// Number of z-faces.
    pub fn v_faces(&self) -> usize {
        self.nx * (self.nz + 1)
    }

    /// World position of the centre of cell `(i, j)`.
    pub fn centre(&self, i: usize, j: usize) -> (f64, f64) {
        (
            self.x0 + (i as f64 + 0.5) * self.dx,
            self.z0 + (j as f64 + 0.5) * self.dx,
        )
    }

    /// High-x edge, m.
    pub fn x1(&self) -> f64 {
        self.x0 + self.nx as f64 * self.dx
    }

    /// High-z edge, m.
    pub fn z1(&self) -> f64 {
        self.z0 + self.nz as f64 * self.dx
    }

    /// The cell containing `(x, z)`, or `None` outside the grid.
    pub fn cell_at(&self, x: f64, z: f64) -> Option<(usize, usize)> {
        let fi = (x - self.x0) / self.dx;
        let fj = (z - self.z0) / self.dx;
        if fi < 0.0 || fj < 0.0 {
            return None;
        }
        let (i, j) = (fi as usize, fj as usize);
        (i < self.nx && j < self.nz).then_some((i, j))
    }
}
