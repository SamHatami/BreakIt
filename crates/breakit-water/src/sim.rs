//! The water of one play table: the solver plus what rides on it (foam,
//! spray, the wake of the structure) and the snapshot the loads read.

use std::sync::Arc;

use breakit_core::loads::rng::Rng;
use breakit_core::loads::water::{piston_height_to_stroke, wave_number, Water, Waves};
use breakit_core::loads::View;
use breakit_core::section::Shape;
use breakit_core::Vec3;

use crate::field::Field;
use crate::grid::Grid;
use crate::paddle::{model_height_to_stroke, model_wave_number};
use crate::solver::{Boundary, Config, Side, Solver, StepStats};

/// What drives the water this frame.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Drive {
    /// Paddle velocity along +x, m/s.
    pub paddle_velocity: f64,
    /// Paddle displacement from rest along +x, m (for the renderer).
    pub paddle_position: f64,
    /// Steady current along +x, m/s.
    pub current: f64,
    /// Still water level to hold, absolute y, m.
    pub still_level: f64,
}

impl Drive {
    /// The drive for a core water case at time `t` since the case started,
    /// with window factor `ramp`. Waves must travel along +x.
    ///
    /// The core sizes the stroke of regular waves with Biésel's transfer
    /// function; the stroke is rescaled here by the solver model's own
    /// function so the simulated waves have the height the case asks for.
    pub fn from_case(water: &Water, t: f64, ramp: f64) -> Self {
        let rig = water.rig(t, ramp);
        let level = water.still_level(t);
        let mut gain = 1.0;
        if let Some(Waves::Airy(a)) = &water.waves {
            let depth = level - water.bed;
            if a.period > 0.0 && depth > 0.0 {
                let omega = std::f64::consts::TAU / a.period;
                let exact = piston_height_to_stroke(wave_number(omega, depth) * depth);
                if let Some(k) = model_wave_number(omega, depth, 9.81) {
                    gain = exact / model_height_to_stroke(k, depth);
                }
            }
        }
        Self {
            paddle_velocity: rig.paddle_velocity * gain,
            paddle_position: rig.paddle_position * gain,
            current: water.current.x * ramp.clamp(0.0, 1.0),
            still_level: level,
        }
    }
}

/// A flume along x for [`WaterSim::flume`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FlumeSetup {
    /// Paddle rest face (low x) and far end (high x), m.
    pub x: (f64, f64),
    /// Channel walls, m.
    pub z: (f64, f64),
    /// Cell size, m.
    pub dx: f64,
    /// Bed level, absolute y, m.
    pub bed: f64,
    /// Still water level at the start, absolute y, m.
    pub still_level: f64,
    /// An absorber beach at the far end: the bed rises from `toe` (x, m) at
    /// `slope` (rise over run).
    pub beach: Option<(f64, f64)>,
    /// True: the far end is an outlet that lets waves and the current out
    /// (long waves and surges almost completely; it reflects about 5 % of
    /// 4 s waves and 10 % of 3 s waves on 2.6 m). False: a wall.
    pub outlet: bool,
}

/// One spray droplet.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Droplet {
    /// World position, m.
    pub position: [f32; 3],
    /// Velocity, m/s.
    pub velocity: [f32; 3],
    /// Age, s.
    pub age: f32,
    /// Age at which it is gone, s.
    pub life: f32,
    /// Radius, m.
    pub size: f32,
}

/// Tuning of the foam and spray: looks, not loads.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Whitewater {
    /// Foam added per second by a cell breaking at unit strength.
    pub foam_from_breaking: f64,
    /// Foam added per second per unit blockage at 1 m/s relative flow.
    pub foam_from_wakes: f64,
    /// Time for foam to fade to 1/e, s.
    pub foam_life: f64,
    /// Droplets per m² of breaking surface per second at unit strength.
    pub spray_rate: f64,
    /// Droplets per second per m² of drag area at 1 m/s relative flow.
    pub splash_rate: f64,
    /// Most droplets alive.
    pub max_droplets: usize,
}

impl Default for Whitewater {
    fn default() -> Self {
        Self {
            foam_from_breaking: 4.0,
            foam_from_wakes: 0.5,
            foam_life: 3.5,
            spray_rate: 1500.0,
            splash_rate: 200.0,
            max_droplets: 4000,
        }
    }
}

/// The water of one table.
#[derive(Clone, Debug)]
pub struct WaterSim {
    /// The solver.
    pub solver: Solver,
    /// Foam and spray tuning.
    pub whitewater: Whitewater,
    foam: Vec<f64>,
    foam_scratch: Vec<f64>,
    droplets: Vec<Droplet>,
    rng: Rng,
    field: Arc<Field>,
    drive: Drive,
    hold_level: bool,
    /// Per structure node: was it under water at the last [`Self::couple`]?
    wet_nodes: Vec<bool>,
}

impl WaterSim {
    /// Wraps a solver that is already set up (bed, boundaries, water).
    /// With `hold_level` the water is topped up or drained through the bed
    /// to follow [`Drive::still_level`], which is how a flood rises.
    pub fn new(solver: Solver, hold_level: bool) -> Self {
        let n = solver.grid.cells();
        let field = Arc::new(Field::capture(&solver));
        Self {
            solver,
            whitewater: Whitewater::default(),
            foam: vec![0.0; n],
            foam_scratch: vec![0.0; n],
            droplets: Vec::new(),
            rng: Rng::new(0x57A7E5),
            field,
            drive: Drive::default(),
            hold_level,
            wet_nodes: Vec::new(),
        }
    }

    /// A flume along x with a piston paddle at the low-x end.
    pub fn flume(setup: FlumeSetup, config: Config) -> Self {
        let grid = Grid::covering(setup.x, setup.z, setup.dx);
        let mut solver = Solver::new(grid, config);
        solver.set_bed(|x, _| match setup.beach {
            Some((toe, slope)) if x > toe => setup.bed + (x - toe) * slope,
            _ => setup.bed,
        });
        solver.fill_to(setup.still_level, |_, _| true);
        solver.set_boundary(Side::XMin, Boundary::Piston);
        if setup.outlet {
            solver.set_boundary(
                Side::XMax,
                Boundary::Radiating {
                    level: setup.still_level,
                    current: 0.0,
                },
            );
        }
        let mut sim = Self::new(solver, !setup.outlet);
        sim.drive.still_level = setup.still_level;
        sim
    }

    /// The water after the last [`Self::advance`], shared with the loads.
    pub fn field(&self) -> Arc<Field> {
        self.field.clone()
    }

    /// Foam cover per cell, 0 (none) to about 1 (white).
    pub fn foam(&self) -> &[f64] {
        &self.foam
    }

    /// Spray droplets in the air.
    pub fn droplets(&self) -> &[Droplet] {
        &self.droplets
    }

    /// The drive of the last [`Self::advance`].
    pub fn drive(&self) -> Drive {
        self.drive
    }

    /// Advances the water by `dt` seconds.
    pub fn advance(&mut self, dt: f64, drive: Drive) -> StepStats {
        self.advance_with(dt, drive, Solver::advance)
    }

    /// [`Self::advance`] with the solver step taken by `step` (another
    /// backend: it gets the solver with this frame's drive, blockage and
    /// sources set, and leaves the new state in it).
    pub fn advance_with(
        &mut self,
        dt: f64,
        drive: Drive,
        step: impl FnOnce(&mut Solver, f64) -> StepStats,
    ) -> StepStats {
        self.drive = drive;
        self.solver.paddle_velocity = drive.paddle_velocity + drive.current;
        self.solver.sponge_velocity = drive.current;
        for b in &mut self.solver.boundaries {
            if let Boundary::Radiating { level, current } = b {
                *level = drive.still_level;
                *current = drive.current;
            }
        }
        if self.hold_level {
            self.follow_level(drive.still_level);
        }
        let stats = step(&mut self.solver, dt);
        self.update_foam(stats.advanced);
        self.update_spray(stats.advanced);
        self.field = Arc::new(Field::capture_after(&self.solver, &self.field));
        stats
    }

    /// Sets the source that moves the mean level of the basin below
    /// `target` towards it.
    fn follow_level(&mut self, target: f64) {
        let s = &self.solver;
        let (mut sum, mut n) = (0.0, 0usize);
        for (b, z) in s.bed().iter().zip(s.surface()) {
            if *b < target {
                sum += z.max(*b);
                n += 1;
            }
        }
        let diff = if n > 0 { target - sum / n as f64 } else { 0.0 };
        let rate = if diff.abs() > 5e-3 {
            (diff / 0.5).clamp(-2.0, 2.0)
        } else {
            0.0
        };
        let bed = self.solver.bed().to_vec();
        let g = self.solver.grid;
        self.solver.set_source(|x, z| {
            let below = g
                .cell_at(x, z)
                .is_some_and(|(i, j)| bed[g.c(i, j)] < target);
            if below {
                rate
            } else {
                0.0
            }
        });
    }

    /// Lets the structure stand in the water: every wet part of every
    /// unbroken member slows the water in its cell with the drag it feels
    /// (`drag_tube`, `drag_rect`: the drag coefficients the load case
    /// uses). Call once per frame before [`Self::advance`].
    pub fn couple(&mut self, view: &View, drag_tube: f64, drag_rect: f64) {
        self.solver.clear_blockage();
        self.splash_entries(view);
        let field = self.field.clone();
        const SAMPLES: usize = 8;
        for m in 0..view.structure.members.len() {
            let Some(mn) = view.member(m) else { continue };
            let geo = &view.geometry.members[m];
            let (cd, width) = match geo.shape {
                Shape::Tube { .. } => (drag_tube, geo.depth_y),
                Shape::Rect { .. } => (drag_rect, 0.5 * (geo.depth_y + geo.depth_z)),
            };
            let ds = mn.length / SAMPLES as f64;
            for k in 0..SAMPLES {
                let (p, vel) = mn.at((k as f64 + 0.5) / SAMPLES as f64);
                let Some((surface, bed)) = field.column(p.x, p.z) else {
                    continue;
                };
                if p.y > surface || p.y < bed {
                    continue;
                }
                // Only the part of the member across the flow blocks it.
                let across = (1.0 - mn.x.y * mn.x.y).max(0.0).sqrt().max(mn.x.y.abs());
                self.solver
                    .add_blockage(p.x, p.z, 0.5 * cd * width * ds * across, vel.x, vel.z);
            }
        }
    }

    /// Something hits the water at `p` at `speed` (m/s); `size` scales the
    /// splash (1 for a node of a typical member). Throws spray, leaves a
    /// patch of foam and starts a ring of waves.
    pub fn splash(&mut self, p: Vec3, speed: f64, size: f64) {
        let g = self.solver.grid;
        let Some((i, j)) = g.cell_at(p.x, p.z) else {
            return;
        };
        let c = g.c(i, j);
        let depth = self.solver.depth()[c];
        if depth <= 0.02 {
            return;
        }
        let surface = self.solver.surface()[c];
        self.foam[c] = (self.foam[c] + 0.6 * size).min(1.5);
        self.solver
            .displace(p.x, p.z, (0.03 * speed * size).min(0.3 * depth));
        let count = (10.0 * speed * speed * size).min(80.0) as usize;
        for _ in 0..count {
            if self.droplets.len() >= self.whitewater.max_droplets {
                break;
            }
            let angle = std::f64::consts::TAU * self.rng.uniform();
            let out = 0.35 * speed * self.rng.uniform();
            let up = 0.6 * speed * (0.3 + self.rng.uniform());
            self.droplets.push(Droplet {
                position: [
                    (p.x + 0.1 * angle.cos() * self.rng.uniform()) as f32,
                    (surface + 0.02) as f32,
                    (p.z + 0.1 * angle.sin() * self.rng.uniform()) as f32,
                ],
                velocity: [
                    (out * angle.cos()) as f32,
                    up as f32,
                    (out * angle.sin()) as f32,
                ],
                age: 0.0,
                life: (0.5 + 0.9 * self.rng.uniform()) as f32,
                size: (0.008 + 0.014 * self.rng.uniform().powi(2)) as f32,
            });
        }
    }

    /// Splashes where nodes of the structure have just dropped into the
    /// water.
    fn splash_entries(&mut self, view: &View) {
        let n = view.structure.nodes.len();
        let field = self.field.clone();
        let under = |node: usize| {
            let p = view.sim.position(node);
            field
                .column(p.x, p.z)
                .is_some_and(|(surface, bed)| p.y < surface && p.y > bed)
        };
        if self.wet_nodes.len() != n {
            self.wet_nodes = (0..n).map(under).collect();
            return;
        }
        for node in 0..n {
            let wet = under(node);
            if wet && !self.wet_nodes[node] {
                let v = view.sim.velocity(node);
                if v.y < -1.0 {
                    let size = (view.sim.node_mass(node) / 20.0).clamp(0.3, 3.0);
                    self.splash(view.sim.position(node), v.norm(), size);
                }
            }
            self.wet_nodes[node] = wet;
        }
    }

    fn update_foam(&mut self, dt: f64) {
        if dt <= 0.0 {
            return;
        }
        let s = &self.solver;
        let g = s.grid;
        let ww = self.whitewater;
        let decay = (-dt / ww.foam_life).exp();
        let (h, strength, block) = (s.depth(), s.break_strength(), s.blockage());
        for j in 0..g.nz {
            for i in 0..g.nx {
                let c = g.c(i, j);
                if h[c] <= s.config.dry {
                    // Foam left on a wet bed fades fast.
                    self.foam_scratch[c] = self.foam[c] * (-dt / 0.6_f64).exp();
                    continue;
                }
                let (u, v) = s.cell_velocity(i, j);
                // Semi-Lagrangian: where did this water come from?
                let fx = (i as f64 - u * dt / g.dx).clamp(0.0, (g.nx - 1) as f64);
                let fz = (j as f64 - v * dt / g.dx).clamp(0.0, (g.nz - 1) as f64);
                let (i0, j0) = (fx.floor() as usize, fz.floor() as usize);
                let (i1, j1) = ((i0 + 1).min(g.nx - 1), (j0 + 1).min(g.nz - 1));
                let (tx, tz) = (fx - i0 as f64, fz - j0 as f64);
                let mut f = (self.foam[g.c(i0, j0)] * (1.0 - tx) + self.foam[g.c(i1, j0)] * tx)
                    * (1.0 - tz)
                    + (self.foam[g.c(i0, j1)] * (1.0 - tx) + self.foam[g.c(i1, j1)] * tx) * tz;
                f *= decay;
                let speed2 = u * u + v * v;
                f += dt * ww.foam_from_breaking * strength[c].min(2.0);
                f += dt
                    * ww.foam_from_wakes
                    * (block[c] / h[c].max(0.05)).min(4.0)
                    * (speed2 - 0.1).max(0.0);
                // The thin fast edge of a run-up or a bore over a dry bed.
                if h[c] < 0.08 && speed2 > 0.25 {
                    f += dt * 2.0;
                }
                self.foam_scratch[c] = f.clamp(0.0, 1.5);
            }
        }
        std::mem::swap(&mut self.foam, &mut self.foam_scratch);
    }

    fn update_spray(&mut self, dt: f64) {
        if dt <= 0.0 {
            return;
        }
        let dtf = dt as f32;
        let field = &self.field;
        self.droplets.retain_mut(|d| {
            d.age += dtf;
            d.velocity[1] -= 9.81 * dtf;
            let drag = 1.0 / (1.0 + 0.8 * dtf);
            for k in 0..3 {
                d.velocity[k] *= drag;
                d.position[k] += d.velocity[k] * dtf;
            }
            let landed = field
                .column(d.position[0] as f64, d.position[2] as f64)
                .is_none_or(|(s, _)| (d.position[1] as f64) < s - 0.02);
            d.age < d.life && !(landed && d.velocity[1] < 0.0)
        });
        let s = &self.solver;
        let g = s.grid;
        let ww = self.whitewater;
        let area = g.dx * g.dx;
        let (h, strength, block, surface) =
            (s.depth(), s.break_strength(), s.blockage(), s.surface());
        for j in 0..g.nz {
            for i in 0..g.nx {
                let c = g.c(i, j);
                if h[c] <= 0.02 || (strength[c] <= 0.0 && block[c] <= 0.0) {
                    continue;
                }
                let (u, v) = s.cell_velocity(i, j);
                let speed = (u * u + v * v).sqrt();
                let lift = (9.81 * h[c]).sqrt();
                let breaking = ww.spray_rate * area * strength[c].min(2.0) * dt;
                let splash = if speed > 0.8 {
                    ww.splash_rate * block[c] * area * speed * speed * dt
                } else {
                    0.0
                };
                for (expect, up, kick) in
                    [(breaking, 0.35 * lift, 0.25), (splash, 0.9 * speed, 0.6)]
                {
                    let mut n = expect.floor() as usize;
                    if self.rng.uniform() < expect - n as f64 {
                        n += 1;
                    }
                    for _ in 0..n {
                        if self.droplets.len() >= ww.max_droplets {
                            return;
                        }
                        let (x, z) = g.centre(i, j);
                        let r = |rng: &mut Rng| rng.uniform() - 0.5;
                        let jitter = [r(&mut self.rng), r(&mut self.rng), r(&mut self.rng)];
                        let rise = up * (0.4 + self.rng.uniform());
                        self.droplets.push(Droplet {
                            position: [
                                (x + jitter[0] * g.dx) as f32,
                                (surface[c] + 0.02) as f32,
                                (z + jitter[2] * g.dx) as f32,
                            ],
                            velocity: [
                                (u * (1.0 - kick) + kick * speed.max(lift) * 2.0 * jitter[0])
                                    as f32,
                                rise as f32,
                                (v * (1.0 - kick) + kick * speed.max(lift) * 2.0 * jitter[1])
                                    as f32,
                            ],
                            age: 0.0,
                            life: (0.5 + 0.9 * self.rng.uniform()) as f32,
                            size: (0.008 + 0.014 * self.rng.uniform().powi(2)) as f32,
                        });
                    }
                }
            }
        }
    }
}
