//! The water solver of `breakit-water` on the GPU.
//!
//! [`GpuSolver`] runs the substeps of [`Solver`] as compute passes
//! (`solver.wgsl`, the same scheme with the red-black pressure solver) and
//! reads the result back into the CPU [`Solver`], which stays the mirror
//! everything else reads: the foam, the spray, the loads on the structure
//! and the renderer do not know where the water was computed.
//!
//! The read-back does not wait for the GPU: a step hands in the state it
//! will produce and takes whatever earlier state has arrived, so the mirror
//! is usually one step behind. [`GpuSolver::sync`] waits, for tests and
//! captures.
//!
//! The GPU works in `f32` and different GPUs round differently, so results
//! match the CPU solver to a tolerance, never bit for bit. See
//! `docs/water.md`.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

use breakit_water::{Boundary, FlowState, Solver, StepStats};

const SHADER: &str = include_str!("solver.wgsl");

/// The mirror the substep is sized from is a step old: keep this much of
/// the stable substep.
const SAFETY: f64 = 0.85;

/// Most read-backs in flight; beyond it a step is not read back and the
/// mirror falls further behind instead of the queue growing.
const MAX_READBACKS: usize = 6;

/// Bytes per array entry (a `vec4<f32>`).
const ENTRY: u64 = 16;

#[derive(Clone, Copy)]
enum Pass {
    Depths,
    Predict,
    Assemble,
    RelaxRed,
    RelaxBlack,
    CorrectFaces,
    CorrectVertical,
    DepthsNew,
    Limit,
    Continuity,
    Commit,
}

const ENTRY_POINTS: [&str; 11] = [
    "depths",
    "predict",
    "assemble",
    "relax_red",
    "relax_black",
    "correct_faces",
    "correct_vertical",
    "depths_new",
    "limit",
    "continuity",
    "commit",
];

/// The pressure relaxation of a substep: see
/// [`breakit_water::PressureSolver::RedBlack`], which is the same on the
/// CPU.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Relaxation {
    /// Sweeps per substep (each is a red and a black pass).
    pub sweeps: usize,
    /// Over-relaxation factor, 1 to 2.
    pub omega: f64,
}

impl Default for Relaxation {
    fn default() -> Self {
        Self {
            sweeps: 16,
            omega: 1.6,
        }
    }
}

struct Readback {
    buffer: wgpu::Buffer,
    ready: Arc<AtomicBool>,
    failed: Arc<AtomicBool>,
    pending: bool,
    epoch: u64,
    serial: u64,
    time: f64,
}

/// The water of one [`Solver`] on the GPU. Make it for a solver that is
/// set up, then call [`Self::step`] where `Solver::advance` would be.
pub struct GpuSolver {
    nx: usize,
    nz: usize,
    /// Pressure relaxation per substep.
    pub relaxation: Relaxation,
    params: wgpu::Buffer,
    state: wgpu::Buffer,
    ground: wgpu::Buffer,
    depth: wgpu::Buffer,
    cell: wgpu::Buffer,
    pressure: wgpu::Buffer,
    drag: wgpu::Buffer,
    bind: wgpu::BindGroup,
    pipelines: Vec<wgpu::ComputePipeline>,
    readbacks: Vec<Readback>,
    /// The solver revision the GPU state was taken from.
    revision: Option<u64>,
    /// Counts the uploads of the state: a read-back from before the last
    /// one is dropped.
    epoch: u64,
    serial: u64,
    applied: u64,
    time: f64,
    upload: Vec<[f32; 4]>,
    surface: Vec<f64>,
    u: Vec<f64>,
    v: Vec<f64>,
    w: Vec<f64>,
    strength: Vec<f64>,
}

impl GpuSolver {
    /// Buffers and pipelines for the grid of `solver`. The state is taken
    /// from the solver at the first [`Self::step`].
    pub fn new(device: &wgpu::Device, solver: &Solver, relaxation: Relaxation) -> Self {
        let (nx, nz) = (solver.grid.nx, solver.grid.nz);
        let entries = (nx + 1) * (nz + 1);
        let storage = |label: &str, stride: u64| {
            device.create_buffer(&wgpu::BufferDescriptor {
                label: Some(label),
                size: entries as u64 * stride,
                usage: wgpu::BufferUsages::STORAGE
                    | wgpu::BufferUsages::COPY_DST
                    | wgpu::BufferUsages::COPY_SRC,
                mapped_at_creation: false,
            })
        };
        let params = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("water params"),
            size: PARAM_WORDS as u64 * 4,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let state = storage("water state", ENTRY);
        let ground = storage("water ground", ENTRY);
        let depth = storage("water face depths", ENTRY);
        let predicted = storage("water predicted", ENTRY);
        let matrix = storage("water pressure matrix", ENTRY);
        let cell = storage("water cells", ENTRY);
        let pressure = storage("water pressure", 4);
        let drag = storage("water drag", ENTRY);

        let buffers = [
            &params, &state, &ground, &depth, &predicted, &matrix, &cell, &pressure, &drag,
        ];
        let layout_entries: Vec<_> = (0..buffers.len() as u32)
            .map(|binding| wgpu::BindGroupLayoutEntry {
                binding,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: if binding == 0 {
                        wgpu::BufferBindingType::Uniform
                    } else {
                        wgpu::BufferBindingType::Storage { read_only: false }
                    },
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            })
            .collect();
        let bind_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("water"),
            entries: &layout_entries,
        });
        let bind_entries: Vec<_> = buffers
            .iter()
            .enumerate()
            .map(|(binding, buffer)| wgpu::BindGroupEntry {
                binding: binding as u32,
                resource: buffer.as_entire_binding(),
            })
            .collect();
        let bind = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("water"),
            layout: &bind_layout,
            entries: &bind_entries,
        });
        let module = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("water solver"),
            source: wgpu::ShaderSource::Wgsl(SHADER.into()),
        });
        let layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("water"),
            bind_group_layouts: &[Some(&bind_layout)],
            immediate_size: 0,
        });
        let pipelines = ENTRY_POINTS
            .iter()
            .map(|entry| {
                device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
                    label: Some(entry),
                    layout: Some(&layout),
                    module: &module,
                    entry_point: Some(entry),
                    compilation_options: wgpu::PipelineCompilationOptions::default(),
                    cache: None,
                })
            })
            .collect();
        let cells = nx * nz;
        Self {
            nx,
            nz,
            relaxation,
            params,
            state,
            ground,
            depth,
            cell,
            pressure,
            drag,
            bind,
            pipelines,
            readbacks: Vec::new(),
            revision: None,
            epoch: 0,
            serial: 0,
            applied: 0,
            time: solver.time(),
            upload: vec![[0.0; 4]; entries],
            surface: vec![0.0; cells],
            u: vec![0.0; (nx + 1) * nz],
            v: vec![0.0; nx * (nz + 1)],
            w: vec![0.0; cells],
            strength: vec![0.0; cells],
        }
    }

    /// Simulated time of the state on the GPU, s (the mirror is behind it
    /// until its read-back arrives).
    pub fn time(&self) -> f64 {
        self.time
    }

    /// Read-backs still on their way.
    pub fn in_flight(&self) -> usize {
        self.readbacks.iter().filter(|r| r.pending).count()
    }

    /// Advances the water by `dt` seconds on the GPU, where
    /// `Solver::advance` would. `solver` supplies what drives the water
    /// (paddle, boundaries, sources, sponge, blockage, splashes) and gets
    /// the newest state that has come back from the GPU.
    pub fn step(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        solver: &mut Solver,
        dt: f64,
    ) -> StepStats {
        assert!(
            solver.grid.nx == self.nx && solver.grid.nz == self.nz,
            "the GPU water was made for another grid"
        );
        // A failed poll shows as a read-back that does not arrive.
        let _ = device.poll(wgpu::PollType::Poll);
        self.collect(solver);
        let mut splashes = solver.take_surface_edits();
        if self.revision != Some(solver.revision()) {
            self.upload_state(queue, solver);
            // The uploaded surface already holds them.
            splashes.clear();
        }
        let mut stats = StepStats::default();
        if dt <= 1e-12 {
            return stats;
        }
        let stable = SAFETY * solver.stable_dt();
        let most = solver.config.max_substeps.max(1);
        let wanted = (dt / stable).ceil().max(1.0);
        let (substeps, h) = if wanted as usize <= most {
            (wanted as usize, dt / wanted)
        } else {
            (most, stable)
        };
        let advanced = substeps as f64 * h;
        self.upload_inputs(queue, solver, &splashes, advanced);
        queue.write_buffer(
            &self.params,
            0,
            bytemuck::cast_slice(&self.param_words(solver, h)),
        );

        let mut encoder =
            device.create_command_encoder(&wgpu::CommandEncoderDescriptor { label: None });
        let groups = (
            (self.nx as u32 + 1).div_ceil(8),
            (self.nz as u32 + 1).div_ceil(8),
        );
        // One pass per dispatch: each reads what the one before wrote.
        let mut run = |pass: Pass| {
            let mut compute = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor::default());
            compute.set_pipeline(&self.pipelines[pass as usize]);
            compute.set_bind_group(0, &self.bind, &[]);
            compute.dispatch_workgroups(groups.0, groups.1, 1);
        };
        for _ in 0..substeps {
            run(Pass::Depths);
            run(Pass::Predict);
            if solver.config.dispersive {
                run(Pass::Assemble);
                for _ in 0..self.relaxation.sweeps {
                    run(Pass::RelaxRed);
                    run(Pass::RelaxBlack);
                }
                run(Pass::CorrectFaces);
                run(Pass::CorrectVertical);
            }
            run(Pass::DepthsNew);
            run(Pass::Limit);
            run(Pass::Continuity);
            run(Pass::Commit);
        }
        self.time += advanced;
        self.serial += 1;
        let slot = self.free_readback(device);
        if let Some(slot) = slot {
            let bytes = self.entries() as u64 * ENTRY;
            let target = &self.readbacks[slot].buffer;
            encoder.copy_buffer_to_buffer(&self.state, 0, target, 0, bytes);
            encoder.copy_buffer_to_buffer(&self.cell, 0, target, bytes, bytes);
        }
        queue.submit([encoder.finish()]);
        if let Some(slot) = slot {
            let r = &mut self.readbacks[slot];
            r.pending = true;
            r.epoch = self.epoch;
            r.serial = self.serial;
            r.time = self.time;
            r.ready.store(false, Ordering::Release);
            r.failed.store(false, Ordering::Release);
            let (ready, failed) = (r.ready.clone(), r.failed.clone());
            r.buffer.map_async(wgpu::MapMode::Read, .., move |result| {
                if result.is_err() {
                    failed.store(true, Ordering::Release);
                }
                ready.store(true, Ordering::Release);
            });
        }
        stats.substeps = substeps;
        stats.pressure_iterations = if solver.config.dispersive {
            substeps * self.relaxation.sweeps
        } else {
            0
        };
        stats.advanced = advanced;
        stats.breaking_cells = solver.breaking().iter().filter(|b| **b).count();
        solver.set_stats(stats);
        stats
    }

    /// Waits for the GPU and brings `solver` up to the state of the last
    /// [`Self::step`]. Returns false if the device was lost or timed out.
    pub fn sync(&mut self, device: &wgpu::Device, solver: &mut Solver) -> bool {
        let done = device.poll(wgpu::PollType::wait_indefinitely()).is_ok();
        self.collect(solver);
        done && self.applied == self.serial
    }

    fn entries(&self) -> usize {
        (self.nx + 1) * (self.nz + 1)
    }

    /// A read-back buffer that is not in flight, made if there is room.
    fn free_readback(&mut self, device: &wgpu::Device) -> Option<usize> {
        if let Some(slot) = self.readbacks.iter().position(|r| !r.pending) {
            return Some(slot);
        }
        if self.readbacks.len() >= MAX_READBACKS {
            return None;
        }
        self.readbacks.push(Readback {
            buffer: device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("water read-back"),
                size: 2 * self.entries() as u64 * ENTRY,
                usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            }),
            ready: Arc::new(AtomicBool::new(false)),
            failed: Arc::new(AtomicBool::new(false)),
            pending: false,
            epoch: 0,
            serial: 0,
            time: 0.0,
        });
        Some(self.readbacks.len() - 1)
    }

    /// Takes the read-backs that have arrived; the newest goes into
    /// `solver`.
    fn collect(&mut self, solver: &mut Solver) {
        let arrived = |r: &Readback| r.pending && r.ready.load(Ordering::Acquire);
        let newest = self
            .readbacks
            .iter()
            .enumerate()
            .filter(|(_, r)| {
                arrived(r)
                    && !r.failed.load(Ordering::Acquire)
                    && r.epoch == self.epoch
                    && r.serial > self.applied
            })
            .max_by_key(|(_, r)| r.serial)
            .map(|(slot, _)| slot);
        if let Some(slot) = newest {
            let (serial, time) = (self.readbacks[slot].serial, self.readbacks[slot].time);
            if let Ok(view) = self.readbacks[slot].buffer.get_mapped_range(..) {
                // The mapping is aligned for f32 (wgpu::MAP_ALIGNMENT is 8).
                let data: &[[f32; 4]] = bytemuck::cast_slice(&view);
                let (state, cell) = data.split_at(self.entries());
                let (nx, nz) = (self.nx, self.nz);
                for j in 0..=nz {
                    for i in 0..=nx {
                        let s = state[j * (nx + 1) + i];
                        if j < nz {
                            self.u[j * (nx + 1) + i] = s[1] as f64;
                        }
                        if i < nx {
                            self.v[j * nx + i] = s[2] as f64;
                        }
                        if i < nx && j < nz {
                            let c = j * nx + i;
                            self.surface[c] = s[0] as f64;
                            self.w[c] = s[3] as f64;
                            self.strength[c] = cell[j * (nx + 1) + i][3] as f64;
                        }
                    }
                }
                drop(view);
                solver.load_state(FlowState {
                    surface: &self.surface,
                    u: &self.u,
                    v: &self.v,
                    w: &self.w,
                    break_strength: &self.strength,
                    time,
                });
                self.applied = serial;
            }
        }
        for r in &mut self.readbacks {
            if arrived(r) {
                if !r.failed.load(Ordering::Acquire) {
                    r.buffer.unmap();
                }
                r.pending = false;
            }
        }
    }

    /// Fills the upload scratch in the padded layout: `f(cell, x-face,
    /// z-face)` with `None` where entry `(i, j)` has none.
    fn fill(&mut self, f: impl Fn(Option<usize>, Option<usize>, Option<usize>) -> [f32; 4]) {
        let (nx, nz) = (self.nx, self.nz);
        for j in 0..=nz {
            for i in 0..=nx {
                self.upload[j * (nx + 1) + i] = f(
                    (i < nx && j < nz).then_some(j * nx + i),
                    (j < nz).then_some(j * (nx + 1) + i),
                    (i < nx).then_some(j * nx + i),
                );
            }
        }
    }

    /// The whole flow state from `solver` to the GPU.
    fn upload_state(&mut self, queue: &wgpu::Queue, solver: &Solver) {
        let at = |a: &[f64], k: Option<usize>| k.map_or(0.0, |k| a[k] as f32);
        self.fill(|c, fu, fv| {
            [
                at(solver.surface(), c),
                at(solver.face_u(), fu),
                at(solver.face_v(), fv),
                at(solver.vertical(), c),
            ]
        });
        queue.write_buffer(&self.state, 0, bytemuck::cast_slice(&self.upload));
        let breaking = solver.breaking();
        self.fill(|c, _, _| {
            let on = c.is_some_and(|c| breaking[c]);
            [0.0, 0.0, if on { 1.0 } else { 0.0 }, 0.0]
        });
        queue.write_buffer(&self.depth, 0, bytemuck::cast_slice(&self.upload));
        let zeros = vec![0.0_f32; self.entries()];
        queue.write_buffer(&self.pressure, 0, bytemuck::cast_slice(&zeros));
        self.revision = Some(solver.revision());
        self.epoch += 1;
        self.applied = self.serial;
        self.time = solver.time();
    }

    /// What drives this step: the bed, the sources (with the splashes
    /// spread over the step as extra source), the sponge and the blockage.
    fn upload_inputs(
        &mut self,
        queue: &wgpu::Queue,
        solver: &Solver,
        splashes: &[(usize, f64)],
        advanced: f64,
    ) {
        let at = |a: &[f64], k: Option<usize>| k.map_or(0.0, |k| a[k] as f32);
        let (sx, sz) = solver.bed_slopes();
        self.fill(|c, fu, fv| {
            [
                at(solver.bed(), c),
                at(sx, fu),
                at(sz, fv),
                at(solver.source(), c),
            ]
        });
        for &(c, metres) in splashes {
            let k = (c / self.nx) * (self.nx + 1) + c % self.nx;
            self.upload[k][3] += (metres / advanced) as f32;
        }
        queue.write_buffer(&self.ground, 0, bytemuck::cast_slice(&self.upload));
        let (bu, bv) = solver.blockage_velocity();
        self.fill(|c, _, _| {
            [
                at(solver.blockage(), c),
                at(bu, c),
                at(bv, c),
                at(solver.sponge(), c),
            ]
        });
        queue.write_buffer(&self.drag, 0, bytemuck::cast_slice(&self.upload));
    }

    /// `Params` of `solver.wgsl` for a substep of `dt`.
    fn param_words(&self, solver: &Solver, dt: f64) -> [u32; PARAM_WORDS] {
        let cfg = &solver.config;
        let f = |x: f64| (x as f32).to_bits();
        let mut words = [0; PARAM_WORDS];
        words[0] = self.nx as u32;
        words[1] = self.nz as u32;
        words[2] = cfg.dispersive as u32;
        let scalars = [
            solver.grid.dx,
            dt,
            cfg.gravity,
            cfg.dry,
            cfg.manning,
            cfg.min_dispersive_depth,
            cfg.breaking_alpha,
            cfg.breaking_beta,
            cfg.max_speed,
            self.relaxation.omega,
            solver.paddle_velocity,
            solver.sponge_velocity,
        ];
        for (word, x) in words[4..16].iter_mut().zip(scalars) {
            *word = f(x);
        }
        for (side, boundary) in solver.boundaries.iter().enumerate() {
            let (kind, level, current) = match *boundary {
                Boundary::Wall => (0, 0.0, 0.0),
                Boundary::Piston => (1, 0.0, 0.0),
                Boundary::Radiating { level, current } => (2, level, current),
            };
            words[16 + side] = kind;
            words[20 + side] = f(level);
            words[24 + side] = f(current);
        }
        words
    }
}

/// Size of `Params` in `solver.wgsl`, in 32-bit words.
const PARAM_WORDS: usize = 28;
