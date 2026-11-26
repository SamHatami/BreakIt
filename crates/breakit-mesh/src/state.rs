//! What the visuals need to know about a structure and its members, and the
//! adapter from the solver.
//!
//! The solver's damage API is growing (concrete, composites). Everything the
//! renderer reads goes through [`MemberVisualState`], so a richer solver only
//! changes [`VisualStructure::capture`], never the shaders or fracture code.

use glam::{Quat, Vec3};

use breakit_core::{BreakCause, DynamicSim, Structure};

use crate::pose::{rest_frame, GpuMember, MemberPose};
use crate::profile::Profile;

/// Material family, which picks the surface look and how a member breaks.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum MaterialKind {
    Steel,
    Timber,
    /// Plain concrete.
    Concrete,
    /// Reinforced concrete: exposed, bent rebar where it breaks.
    ReinforcedConcrete,
    /// Fibre composite (carbon, glass): delaminates and brooms.
    Composite,
}

impl MaterialKind {
    /// From the solver's material: its family, plus whether concrete has bars.
    pub fn from_material(m: &breakit_core::Material) -> Self {
        use breakit_core::MaterialKind as K;
        match m.kind {
            K::Steel => MaterialKind::Steel,
            K::Timber => MaterialKind::Timber,
            K::Concrete if m.reinforced() => MaterialKind::ReinforcedConcrete,
            K::Concrete => MaterialKind::Concrete,
            K::Composite => MaterialKind::Composite,
        }
    }

    pub fn index(self) -> u32 {
        self as u32
    }

    pub const ALL: [MaterialKind; 5] = [
        MaterialKind::Steel,
        MaterialKind::Timber,
        MaterialKind::Concrete,
        MaterialKind::ReinforcedConcrete,
        MaterialKind::Composite,
    ];
}

/// How a member looks: its cross-section and material family.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MemberStyle {
    pub profile: Profile,
    pub kind: MaterialKind,
}

/// Damage state of one member, all values 0..1 unless noted.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct MemberVisualState {
    /// Current utilisation (1 = capacity). Drives the stress heatmap.
    pub utilization: f32,
    /// Plastic hinge rotations `[axis y, axis z][end a, end b]`, rad.
    pub hinge: [[f32; 2]; 2],
    /// Hinge rotation as a fraction of the member's capacity, per end.
    pub hinge_damage: [f32; 2],
    /// Flexural cracking per end, and through-cracking along the span.
    pub crack: [f32; 2],
    pub crack_span: f32,
    /// Crushing per end.
    pub crush: [f32; 2],
    pub delamination: f32,
    /// Has yielded at some point (permanent set).
    pub yielded: bool,
    pub broken: bool,
}

/// A member breaking this frame.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct VisualBreak {
    pub member: usize,
    /// Where along the member it failed, 0 = end a, 1 = end b.
    pub position: f32,
    pub cause: BreakCause,
}

/// Static description of a structure's visuals: one entry per solver member.
#[derive(Clone, Debug)]
pub struct VisualStructure {
    pub members: Vec<VisualMember>,
    /// Number of solver break events already turned into debris.
    pub consumed_breaks: usize,
}

#[derive(Clone, Debug)]
pub struct VisualMember {
    pub a: usize,
    pub b: usize,
    pub style: MemberStyle,
    pub rest_frame: Quat,
    pub rest_length: f32,
    /// Stable per-member random seed (crack patterns, fracture shapes).
    pub seed: f32,
}

/// One frame of solver output in the form the visuals use.
#[derive(Clone, Debug, Default)]
pub struct VisualFrame {
    pub node_positions: Vec<Vec3>,
    /// Node rotations since rest. Empty means "no rotations" (build mode).
    pub node_rotations: Vec<Quat>,
    /// Node velocities, for debris launch. Empty means at rest.
    pub node_velocities: Vec<Vec3>,
    pub members: Vec<MemberVisualState>,
    /// Breaks that happened since the previous frame.
    pub breaks: Vec<VisualBreak>,
}

pub fn v3(v: breakit_core::Vec3) -> Vec3 {
    Vec3::new(v.x as f32, v.y as f32, v.z as f32)
}

impl VisualStructure {
    /// Build from the solver model plus one style per member.
    pub fn new(s: &Structure, styles: &[MemberStyle]) -> Self {
        assert_eq!(styles.len(), s.members.len(), "one style per member");
        let members = s
            .members
            .iter()
            .zip(styles)
            .enumerate()
            .map(|(i, (m, style))| {
                let (a, b) = (v3(s.nodes[m.a]), v3(s.nodes[m.b]));
                VisualMember {
                    a: m.a,
                    b: m.b,
                    style: *style,
                    rest_frame: rest_frame(a, b),
                    rest_length: (b - a).length(),
                    seed: hash01(i as u32 ^ 0x9e37_79b9),
                }
            })
            .collect();
        Self {
            members,
            consumed_breaks: 0,
        }
    }

    /// Styles from the solver materials, all with one profile. Handy for
    /// tests and examples; real designs pick a profile per member.
    pub fn uniform(s: &Structure, profile: Profile) -> Self {
        let styles: Vec<MemberStyle> = s
            .members
            .iter()
            .map(|m| MemberStyle {
                profile,
                kind: MaterialKind::from_material(&m.material),
            })
            .collect();
        Self::new(s, &styles)
    }

    /// Read this frame's state from the dynamic solver.
    pub fn capture(&mut self, sim: &DynamicSim, structure: &Structure) -> VisualFrame {
        let n = structure.nodes.len();
        let mut f = VisualFrame {
            node_positions: (0..n).map(|i| v3(sim.position(i))).collect(),
            node_rotations: (0..n)
                .map(|i| {
                    let q = sim.orientation(i);
                    Quat::from_xyzw(q.i as f32, q.j as f32, q.k as f32, q.w as f32)
                })
                .collect(),
            node_velocities: Vec::new(),
            members: Vec::with_capacity(self.members.len()),
            breaks: Vec::new(),
        };
        for (i, d) in sim.damage_all().enumerate() {
            let hinge = [
                [d.hinge[0][0] as f32, d.hinge[0][1] as f32],
                [d.hinge[1][0] as f32, d.hinge[1][1] as f32],
            ];
            let end_mag = |e: usize| (hinge[0][e].powi(2) + hinge[1][e].powi(2)).sqrt();
            let (ma, mb) = (end_mag(0), end_mag(1));
            // Ductile members report damage 0.5..1 for the plastic rotation
            // they have used up; share it between the ends by hinge size.
            let plastic = ((d.damage as f32 - 0.5) * 2.0).clamp(0.0, 1.0);
            let top = ma.max(mb).max(1e-9);
            let hinge_damage = if ma + mb > 0.0 {
                [plastic * ma / top, plastic * mb / top]
            } else {
                [0.0; 2]
            };
            f.members.push(MemberVisualState {
                utilization: d.utilization as f32,
                hinge,
                hinge_damage,
                crack: [d.crack[0] as f32, d.crack[1] as f32],
                crack_span: d.tension_crack as f32,
                crush: [d.crush[0] as f32, d.crush[1] as f32],
                delamination: d.delamination as f32,
                yielded: d.state >= breakit_core::DamageState::Yielded
                    || ma + mb > 0.0
                    || d.plastic_strain.abs() > 0.0,
                broken: sim.is_broken(i),
            });
        }
        let events = sim.break_events();
        for e in &events[self.consumed_breaks.min(events.len())..] {
            // The solver reports 0, 1 or 0.5. Keep end failures a section
            // depth or so inside the member and spread mid-span ones out, so
            // neighbouring breaks do not all look alike.
            let vm = &self.members[e.member];
            let inset = (1.5 * vm.style.profile.half_depth() / vm.rest_length.max(1e-3)).min(0.3);
            let p = e.position as f32;
            let position = if p < 0.25 {
                inset
            } else if p > 0.75 {
                1.0 - inset
            } else {
                0.35 + 0.3 * vm.seed
            };
            f.breaks.push(VisualBreak {
                member: e.member,
                position,
                cause: e.cause,
            });
        }
        self.consumed_breaks = events.len();
        f
    }

    /// Frame for build mode: straight members at their node positions.
    pub fn frame_from_positions(&self, positions: &[Vec3], utilization: &[f32]) -> VisualFrame {
        VisualFrame {
            node_positions: positions.to_vec(),
            node_rotations: Vec::new(),
            node_velocities: Vec::new(),
            members: (0..self.members.len())
                .map(|i| MemberVisualState {
                    utilization: utilization.get(i).copied().unwrap_or(0.0),
                    ..Default::default()
                })
                .collect(),
            breaks: Vec::new(),
        }
    }

    /// Pose of member `i` in `frame`.
    pub fn pose(&self, i: usize, frame: &VisualFrame) -> MemberPose {
        let vm = &self.members[i];
        let (pa, pb) = (frame.node_positions[vm.a], frame.node_positions[vm.b]);
        if frame.node_rotations.is_empty() {
            let mut p = MemberPose::straight(pa, pb);
            p.rest_length = vm.rest_length;
            return p;
        }
        let hinge = frame.members.get(i).map(|m| m.hinge).unwrap_or_default();
        MemberPose::from_nodes(
            pa,
            pb,
            vm.rest_frame,
            frame.node_rotations[vm.a],
            frame.node_rotations[vm.b],
            hinge,
            vm.rest_length,
            2.0 * vm.style.profile.half_depth(),
        )
    }

    /// Pack member `i` for the GPU.
    pub fn gpu_member(&self, i: usize, frame: &VisualFrame, member_flags: u32) -> GpuMember {
        let pose = self.pose(i, frame);
        let st = frame.members.get(i).copied().unwrap_or_default();
        let vm = &self.members[i];
        let mut fl = member_flags;
        if st.broken {
            fl |= crate::pose::flags::HIDDEN;
        }
        GpuMember {
            pa: pose.pa.extend(pose.stub_a),
            pb: pose.pb.extend(pose.stub_b),
            qa: Vec3::new(pose.qa.x, pose.qa.y, pose.qa.z).extend(pose.qa.w),
            qb: Vec3::new(pose.qb.x, pose.qb.y, pose.qb.z).extend(pose.qb.w),
            ta: pose.ta.extend(st.hinge_damage[0]),
            tb: pose.tb.extend(st.hinge_damage[1]),
            damage0: glam::Vec4::new(st.utilization, st.crack[0], st.crack[1], st.crack_span),
            damage1: glam::Vec4::new(
                st.crush[0],
                st.crush[1],
                st.delamination,
                st.yielded as u32 as f32,
            ),
            info: glam::Vec4::new(
                vm.rest_length,
                vm.seed,
                vm.style.kind.index() as f32,
                fl as f32,
            ),
            clip: glam::Vec4::new(0.0, 1.0, 0.0, 0.0),
            size: glam::Vec4::new(
                2.0 * vm.style.profile.half_depth(),
                2.0 * vm.style.profile.half_width(),
                0.0,
                0.0,
            ),
        }
    }
}

/// Integer hash to 0..1.
pub fn hash01(mut x: u32) -> f32 {
    x ^= x >> 16;
    x = x.wrapping_mul(0x7feb_352d);
    x ^= x >> 15;
    x = x.wrapping_mul(0x846c_a68b);
    x ^= x >> 16;
    (x >> 8) as f32 / (1u32 << 24) as f32
}
