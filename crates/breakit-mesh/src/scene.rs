//! The whole visual state of one structure, engine-free.
//!
//! [`VisualScene`] turns solver frames into exactly what a renderer needs:
//!
//! * `gpu`: one [`GpuMember`] record per *slot*. Slot `i` below the member
//!   count is member `i`. A break never re-meshes anything: the member's slot
//!   becomes its a-side half and new slots are appended for the b-side half
//!   and any exposed rebar ([`Remnant`]s). Halves stay attached to their
//!   nodes, spring back and ring for a moment, then follow the node rigidly.
//! * `slots`: which template (profile, lean or detailed) each slot draws.
//! * `debris` and `particles`: loose chunks, sparks, dust, splinters, fibres.
//!
//! The per-frame cost is one 176-byte record per slot (see
//! `examples/visual_bench.rs`); the meshes themselves are never touched.

use glam::{Quat, Vec3, Vec4};

use crate::cut::cut_reach;
use crate::debris::{DebrisConfig, DebrisWorld};
use crate::fracture::{
    fracture, BreakInput, BurstKind, DebrisPiece, ParticleBurst, PiecePart, Rng,
};
use crate::joint::{member_part_pose, node_part_pose, JointLook, JointPose};
use crate::particles::ParticlePool;
use crate::pose::{flags, GpuMember, MemberPose};
use crate::profile::Profile;
use crate::state::{MaterialKind, MemberVisualState, VisualBreak, VisualFrame, VisualStructure};

/// Which template a slot draws with.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Slot {
    pub profile: Profile,
    /// Use the detailed template (fracture faces need the vertices).
    pub detailed: bool,
}

/// A piece of a broken member that is still attached to a node.
#[derive(Clone, Debug)]
pub struct Remnant {
    pub member: usize,
    pub slot: usize,
    /// Node this piece hangs on to.
    pub anchor: usize,
    /// Pose in the anchor node's frame at the moment of the break.
    pub bent: MemberPose,
    /// Pose in the anchor node's frame once the elastic bend has sprung back.
    pub relaxed: MemberPose,
    pub clip: Vec4,
    pub profile: Profile,
    pub kind: MaterialKind,
    pub flags: u32,
    pub seed: f32,
    /// Scene time of the break.
    pub born: f32,
    /// Damage frozen at the break.
    pub state: MemberVisualState,
    /// Ringing after the break: direction and size of the free end's swing
    /// (anchor frame), and its angular frequency.
    ring: Vec3,
    omega: f32,
    /// True when the free end is end b of the pose.
    free_end_b: bool,
    /// Member ends (a, b) whose joint hardware rides on this piece.
    rides: [bool; 2],
}

/// Connection hardware at one member end. The list is fixed for the life of
/// the scene; [`VisualScene::joint_states`] has this frame's poses.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct JointInstance {
    pub member: usize,
    /// 0 = end a, 1 = end b.
    pub end: usize,
    pub node: usize,
    pub look: JointLook,
    pub profile: Profile,
    pub kind: MaterialKind,
    /// Distance from the node's centre to where the member is cut, m.
    pub back: f32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct JointState {
    pub pose: JointPose,
    /// The connection has failed: its fuse parts (pin, bolts, weld) are
    /// gone and the member has come away from the node.
    pub failed: bool,
}

/// The body of a node where joints meet.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Hub {
    pub node: usize,
    pub radius: f32,
    /// A cast concrete block instead of a steel ball.
    pub concrete: bool,
}

impl Remnant {
    /// Pose in the anchor's frame `age` seconds after the break.
    pub fn local_pose(&self, age: f32) -> MemberPose {
        if age > 3.0 {
            return self.relaxed;
        }
        // Under-damped spring from the bent shape to the relaxed one.
        let w = 1.0 - (-7.0 * age).exp() * (self.omega * age).cos();
        let mut p = self.bent.blend(&self.relaxed, w);
        let swing = self.ring * ((-4.0 * age).exp() * (self.omega * age).sin());
        let len = (p.pb - p.pa).length().max(1e-4);
        if self.free_end_b {
            p.pb += swing;
            p.tb = (p.tb + 1.5 * swing / len).normalize();
        } else {
            p.pa += swing;
            p.ta = (p.ta - 1.5 * swing / len).normalize();
        }
        p
    }
}

#[derive(Clone, Copy, Debug)]
pub struct SceneConfig {
    pub debris: DebrisConfig,
    pub max_particles: usize,
}

impl Default for SceneConfig {
    fn default() -> Self {
        Self {
            debris: DebrisConfig::default(),
            max_particles: 8192,
        }
    }
}

pub struct VisualScene {
    pub structure: VisualStructure,
    pub slots: Vec<Slot>,
    /// Bumped whenever `slots` changes (a break), so a renderer knows to
    /// re-sync its instances.
    pub slots_revision: u64,
    pub gpu: Vec<GpuMember>,
    pub remnants: Vec<Remnant>,
    pub debris: DebrisWorld,
    pub particles: ParticlePool,
    /// Debris pieces created since the renderer last drained this, with
    /// their ids in `debris`.
    pub spawned: Vec<(u64, DebrisPiece)>,
    pub time: f32,
    /// Connection hardware, in member order. Empty unless the structure has
    /// joints.
    pub joints: Vec<JointInstance>,
    /// This frame's pose of each of `joints`.
    pub joint_states: Vec<JointState>,
    pub hubs: Vec<Hub>,
    /// This frame's position and rotation of each of `hubs`.
    pub hub_poses: Vec<(Vec3, Quat)>,
    /// World pose carrying each member's end a and end b hardware.
    end_poses: Vec<[MemberPose; 2]>,
    joint_failed: Vec<[bool; 2]>,
    broken: Vec<bool>,
    prev_positions: Vec<Vec3>,
}

impl VisualScene {
    pub fn new(structure: VisualStructure, config: SceneConfig) -> Self {
        let slots: Vec<Slot> = structure
            .members
            .iter()
            .map(|m| Slot {
                profile: m.style.profile,
                detailed: false,
            })
            .collect();
        let mut particles = ParticlePool::new(config.max_particles);
        particles.ground_y = config.debris.ground_y;
        let mut joints = Vec::new();
        for (i, m) in structure.members.iter().enumerate() {
            for (end, node) in [m.a, m.b].into_iter().enumerate() {
                let look = structure.joint(i, end);
                if look != JointLook::None {
                    joints.push(JointInstance {
                        member: i,
                        end,
                        node,
                        look,
                        profile: m.style.profile,
                        kind: m.style.kind,
                        back: structure.trim(i, end),
                    });
                }
            }
        }
        let hubs: Vec<Hub> = (0..structure.node_radius.len())
            .filter(|&n| structure.node_radius[n] > 0.0)
            .map(|n| Hub {
                node: n,
                radius: structure.node_radius[n],
                concrete: joints
                    .iter()
                    .filter(|j| j.node == n)
                    .all(|j| j.look == JointLook::Monolithic),
            })
            .collect();
        let straight = MemberPose::straight(Vec3::ZERO, Vec3::X);
        Self {
            joint_states: Vec::with_capacity(joints.len()),
            hub_poses: Vec::with_capacity(hubs.len()),
            end_poses: vec![[straight; 2]; if joints.is_empty() { 0 } else { slots.len() }],
            joint_failed: vec![[false; 2]; slots.len()],
            joints,
            hubs,
            broken: vec![false; slots.len()],
            gpu: vec![GpuMember::default(); slots.len()],
            slots,
            slots_revision: 0,
            structure,
            remnants: Vec::new(),
            debris: DebrisWorld::new(config.debris),
            particles,
            spawned: Vec::new(),
            time: 0.0,
            prev_positions: Vec::new(),
        }
    }

    pub fn member_count(&self) -> usize {
        self.structure.members.len()
    }

    /// Advance to a new solver frame: handle its breaks, pack every slot for
    /// the GPU and step debris and particles. `member_flags` is OR-ed into
    /// every member's flags (e.g. [`flags::STRESS_VIEW`]).
    pub fn update(&mut self, frame: &VisualFrame, dt: f32, member_flags: u32) {
        self.time += dt;
        let node_rot = |n: usize| {
            frame
                .node_rotations
                .get(n)
                .copied()
                .unwrap_or(Quat::IDENTITY)
        };

        for br in &frame.breaks {
            if br.member < self.broken.len() && !self.broken[br.member] {
                self.break_member(br, frame, dt);
            }
        }

        self.gpu.resize(self.slots.len(), GpuMember::default());
        let track = !self.end_poses.is_empty();
        for i in 0..self.structure.members.len() {
            if !self.broken[i] {
                let pose = self.structure.pose(i, frame);
                self.gpu[i] = self
                    .structure
                    .gpu_member_posed(i, &pose, frame, member_flags);
                if track {
                    self.end_poses[i] = [pose; 2];
                }
            }
        }
        for r in &self.remnants {
            let age = self.time - r.born;
            let pose = r
                .local_pose(age)
                .transformed(node_rot(r.anchor), frame.node_positions[r.anchor]);
            if track {
                for end in 0..2 {
                    if r.rides[end] {
                        self.end_poses[r.member][end] = pose;
                    }
                }
            }
            let st = &r.state;
            self.gpu[r.slot] = GpuMember {
                pa: pose.pa.extend(pose.stub_a),
                pb: pose.pb.extend(pose.stub_b),
                qa: Vec4::from(pose.qa),
                qb: Vec4::from(pose.qb),
                ta: pose.ta.extend(st.hinge_damage[0]),
                tb: pose.tb.extend(st.hinge_damage[1]),
                damage0: Vec4::new(st.utilization, st.crack[0], st.crack[1], st.crack_span),
                damage1: Vec4::new(
                    st.crush[0],
                    st.crush[1],
                    st.delamination,
                    st.yielded as u32 as f32,
                ),
                info: Vec4::new(
                    pose.rest_length,
                    r.seed,
                    r.kind.index() as f32,
                    ((member_flags & !flags::HIDDEN) | r.flags) as f32,
                ),
                clip: r.clip,
                size: Vec4::new(
                    2.0 * r.profile.half_depth(),
                    2.0 * r.profile.half_width(),
                    age,
                    0.0,
                ),
            };
        }

        self.joint_states.clear();
        for j in &self.joints {
            let vm = &self.structure.members[j.member];
            let q = node_rot(j.node) * vm.rest_frame;
            self.joint_states.push(JointState {
                pose: JointPose {
                    node: node_part_pose(frame.node_positions[j.node], q, j.end, j.back),
                    member: member_part_pose(&self.end_poses[j.member][j.end], j.end, j.back),
                },
                failed: self.joint_failed[j.member][j.end],
            });
        }
        self.hub_poses.clear();
        self.hub_poses.extend(
            self.hubs
                .iter()
                .map(|h| (frame.node_positions[h.node], node_rot(h.node))),
        );

        self.debris.step(dt);
        self.particles.step(dt);
        self.prev_positions.clear();
        self.prev_positions.extend_from_slice(&frame.node_positions);
    }

    fn add_remnant(&mut self, mut r: Remnant, reuse_slot: Option<usize>, detailed: bool) {
        let slot = Slot {
            profile: r.profile,
            detailed,
        };
        r.slot = match reuse_slot {
            Some(s) => {
                self.slots[s] = slot;
                s
            }
            None => {
                self.slots.push(slot);
                self.slots.len() - 1
            }
        };
        self.remnants.push(r);
    }

    fn break_member(&mut self, br: &VisualBreak, frame: &VisualFrame, dt: f32) {
        let i = br.member;
        self.broken[i] = true;
        // A connection that lets go where hardware is drawn frees the whole
        // member from that node; anywhere else the member itself tears.
        let released = (br.cause == breakit_core::BreakCause::Connection)
            .then_some(if br.position < 0.5 { 0 } else { 1 })
            .filter(|&end| self.structure.joint(i, end) != JointLook::None);
        if released.is_none() {
            self.slots_revision += 1;
        }
        let (lo, hi) = self.structure.window(i);
        let vm = self.structure.members[i].clone();
        let (profile, kind) = (vm.style.profile, vm.style.kind);
        let pose = self.structure.pose(i, frame);
        let l = pose.rest_length.max(1e-3);
        let depth = 2.0 * profile.half_depth();
        let size = depth.max(2.0 * profile.half_width());
        let mut rng = Rng::new((vm.seed * 16_777_216.0) as u32 ^ 0x51ed_270b);
        let mut state = frame.members.get(i).copied().unwrap_or_default();
        state.broken = true;

        // Keep the whole fracture face inside the member.
        let margin = ((cut_reach(kind, depth) + 0.1 * depth) / l).min(0.45);
        let sb = br
            .position
            .clamp(margin.max(lo + 0.02), (1.0 - margin).min(hi - 0.02));
        // Concrete loses a crushed zone between its two faces.
        let gap = match kind {
            MaterialKind::Concrete | MaterialKind::ReinforcedConcrete => {
                (0.3 * depth / l).min(0.5 * margin)
            }
            _ => 0.0,
        };

        let node_q = |n: usize| {
            frame
                .node_rotations
                .get(n)
                .copied()
                .unwrap_or(Quat::IDENTITY)
        };
        let to_local = |p: &MemberPose, n: usize| {
            let inv = node_q(n).inverse();
            p.transformed(inv, -(inv * frame.node_positions[n]))
        };
        let (ta_node, tb_node) = (pose.qa * Vec3::X, pose.qb * Vec3::X);
        let p0 = pose.pa + ta_node * pose.stub_a;
        let p1 = pose.pb - tb_node * pose.stub_b;
        let lc = (p1 - p0).length();
        let turn = |q: Quat, to: Vec3| (Quat::from_rotation_arc(q * Vec3::X, to) * q).normalize();

        // Half a: keeps its hinge crease, loses the elastic bend.
        let relaxed_a = MemberPose {
            pb: p0 + pose.ta * (lc + pose.stub_b),
            qb: turn(pose.qa, pose.ta),
            tb: pose.ta,
            stub_b: 0.0,
            ..pose
        };
        let relaxed_b = MemberPose {
            pa: p1 - pose.tb * (lc + pose.stub_a),
            qa: turn(pose.qb, pose.tb),
            ta: pose.tb,
            stub_a: 0.0,
            ..pose
        };
        let ring_of = |rng: &mut Rng, free_len: f32, axis: Vec3| {
            let side = axis
                .cross(rng.unit_vec())
                .try_normalize()
                .unwrap_or(Vec3::Y);
            // The swing is that of the whole pose's far end; scale it so the
            // visible free end moves a few percent of its length.
            let amp =
                (0.05 * free_len).min(0.6 * depth) / (free_len / l).max(0.05).powi(2).max(0.2);
            (
                side * amp.min(0.25 * l),
                (70.0 / (1.0 + 5.0 * free_len)).clamp(9.0, 45.0),
            )
        };
        let (ring_a, omega_a) = ring_of(&mut rng, sb * l, pose.ta);
        let (ring_b, omega_b) = ring_of(&mut rng, (1.0 - sb) * l, pose.tb);
        let inv_a = node_q(vm.a).inverse();
        let inv_b = node_q(vm.b).inverse();
        if let Some(end) = released {
            // One whole piece, hanging on by its other end.
            self.joint_failed[i][end] = true;
            let (anchor, relaxed, ring, omega, inv) = if end == 0 {
                (vm.b, &relaxed_b, ring_b, omega_b, inv_b)
            } else {
                (vm.a, &relaxed_a, ring_a, omega_a, inv_a)
            };
            self.add_remnant(
                Remnant {
                    member: i,
                    slot: 0,
                    anchor,
                    bent: to_local(&pose, anchor),
                    relaxed: to_local(relaxed, anchor),
                    clip: Vec4::new(lo, hi, 0.0, 0.0),
                    profile,
                    kind,
                    flags: 0,
                    seed: vm.seed,
                    born: self.time,
                    state,
                    ring: inv * ring,
                    omega: omega * 0.5,
                    free_end_b: end == 1,
                    rides: [true; 2],
                },
                Some(i),
                false,
            );
            let (origin, axis) = if end == 0 {
                (pose.pa, pose.ta)
            } else {
                (pose.pb, -pose.tb)
            };
            let kind = match kind {
                MaterialKind::Timber => BurstKind::Splinters,
                MaterialKind::Concrete | MaterialKind::ReinforcedConcrete => BurstKind::Dust,
                _ => BurstKind::Sparks,
            };
            self.particles.emit(&ParticleBurst {
                kind,
                origin,
                axis,
                count: 24,
                scale: size,
            });
            return;
        }
        self.add_remnant(
            Remnant {
                member: i,
                slot: 0,
                anchor: vm.a,
                bent: to_local(&pose, vm.a),
                relaxed: to_local(&relaxed_a, vm.a),
                clip: Vec4::new(lo, sb - gap, 0.0, 1.0),
                profile,
                kind,
                flags: 0,
                seed: vm.seed,
                born: self.time,
                state,
                ring: inv_a * ring_a,
                omega: omega_a,
                free_end_b: true,
                rides: [true, false],
            },
            Some(i),
            true,
        );
        self.add_remnant(
            Remnant {
                member: i,
                slot: 0,
                anchor: vm.b,
                bent: to_local(&pose, vm.b),
                relaxed: to_local(&relaxed_b, vm.b),
                clip: Vec4::new(sb + gap, hi, 1.0, 0.0),
                profile,
                kind,
                flags: 0,
                seed: vm.seed,
                born: self.time,
                state,
                ring: inv_b * ring_b,
                omega: omega_b,
                free_end_b: false,
                rides: [false, true],
            },
            None,
            true,
        );

        // Exposed reinforcement: four corner bars torn and bent out of each face.
        if kind == MaterialKind::ReinforcedConcrete {
            let (hy, hz) = (profile.half_depth() * 0.76, profile.half_width() * 0.76);
            let bar = Profile::Round {
                r: (0.045 * size * 200.0).round().max(1.0) / 200.0,
            };
            for (cy, cz) in [(-hy, -hz), (-hy, hz), (hy, hz), (hy, -hz)] {
                for (anchor, s_root, dir) in [
                    (vm.a, sb - gap - 0.4 * depth / l, 1.0f32),
                    (vm.b, sb + gap + 0.4 * depth / l, -1.0f32),
                ] {
                    let s_root = s_root.clamp(0.0, 1.0);
                    let (c, t) = pose.centreline(s_root);
                    let q = pose.frame(s_root, t);
                    let start = c + q * Vec3::new(0.0, cy, cz);
                    let len = depth * rng.range(1.0, 2.0);
                    let out = q * Vec3::new(0.0, cy.signum(), cz.signum()) * 0.7;
                    let bend =
                        (out * rng.range(0.05, 0.3) + Vec3::NEG_Y * rng.range(0.05, 0.35)) * depth;
                    let t = t * dir;
                    let end = start + t * len + bend;
                    let tip_dir = (t * len * 0.35 + bend * 2.0).normalize();
                    let bar_pose = MemberPose {
                        pa: start,
                        pb: end,
                        qa: turn(q, t),
                        qb: turn(q, tip_dir),
                        ta: t,
                        tb: tip_dir,
                        stub_a: 0.0,
                        stub_b: 0.0,
                        rest_length: len,
                    };
                    let local = to_local(&bar_pose, anchor);
                    self.add_remnant(
                        Remnant {
                            member: i,
                            slot: 0,
                            anchor,
                            bent: local,
                            relaxed: local,
                            clip: Vec4::new(0.0, 1.0, 0.0, 0.0),
                            profile: bar,
                            kind: MaterialKind::Steel,
                            flags: flags::REBAR,
                            seed: rng.next_f32(),
                            born: self.time,
                            state: MemberVisualState::default(),
                            ring: Vec3::ZERO,
                            omega: 0.0,
                            free_end_b: true,
                            rides: [false; 2],
                        },
                        None,
                        false,
                    );
                }
            }
        }

        // Loose pieces and particle bursts.
        let vel = |n: usize| match self.prev_positions.get(n) {
            Some(p) if dt > 0.0 => (frame.node_positions[n] - *p) / dt,
            _ => frame.node_velocities.get(n).copied().unwrap_or(Vec3::ZERO),
        };
        let f = fracture(&BreakInput {
            profile: &profile,
            kind,
            pose: &pose,
            position: sb,
            velocity_a: vel(vm.a),
            velocity_b: vel(vm.b),
            seed: (vm.seed * 16_777_216.0) as u32,
        });
        for piece in f.pieces {
            if piece.part == PiecePart::Chunk {
                let id = self.debris.add(&piece);
                self.spawned.push((id, piece));
            }
        }
        for b in &f.bursts {
            self.particles.emit(b);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::state::{MemberStyle, VisualMember};
    use breakit_core::BreakCause;

    fn one_member(kind: MaterialKind) -> (VisualScene, VisualFrame) {
        let (a, b) = (Vec3::new(0.0, 3.0, 0.0), Vec3::new(4.0, 3.0, 0.0));
        let structure = VisualStructure::from_members(
            vec![VisualMember {
                a: 0,
                b: 1,
                style: MemberStyle {
                    profile: Profile::Rect { b: 0.2, h: 0.3 },
                    kind,
                },
                rest_frame: crate::pose::rest_frame(a, b),
                rest_length: 4.0,
                seed: 0.42,
            }],
            2,
        );
        let frame = VisualFrame {
            node_positions: vec![a, b],
            node_rotations: vec![Quat::IDENTITY; 2],
            node_velocities: Vec::new(),
            members: vec![MemberVisualState::default()],
            breaks: Vec::new(),
        };
        (VisualScene::new(structure, SceneConfig::default()), frame)
    }

    fn break_it(scene: &mut VisualScene, frame: &mut VisualFrame) {
        frame.breaks.push(VisualBreak {
            member: 0,
            position: 0.5,
            cause: BreakCause::Brittle,
        });
        frame.members[0].broken = true;
        scene.update(frame, 1.0 / 60.0, 0);
        frame.breaks.clear();
    }

    #[test]
    fn intact_member_packs_one_full_window() {
        let (mut scene, frame) = one_member(MaterialKind::Steel);
        scene.update(&frame, 1.0 / 60.0, 0);
        assert_eq!(scene.gpu.len(), 1);
        assert_eq!(scene.gpu[0].clip, Vec4::new(0.0, 1.0, 0.0, 0.0));
        assert_eq!(scene.slots_revision, 0);
    }

    #[test]
    fn break_splits_into_two_attached_halves() {
        let (mut scene, mut frame) = one_member(MaterialKind::Timber);
        scene.update(&frame, 1.0 / 60.0, 0);
        break_it(&mut scene, &mut frame);
        assert_eq!(scene.slots.len(), 2);
        assert!(scene.slots.iter().all(|s| s.detailed));
        assert_eq!(scene.slots_revision, 1);
        let (a, b) = (scene.gpu[0], scene.gpu[1]);
        // Windows meet at the break; the facing ends are fracture faces.
        assert_eq!(a.clip.x, 0.0);
        assert_eq!(b.clip.y, 1.0);
        assert!((a.clip.y - b.clip.x).abs() < 1e-6);
        assert_eq!((a.clip.w, b.clip.z), (1.0, 1.0));
        assert_eq!(a.info.w as u32 & flags::HIDDEN, 0);
        assert!(scene.particles.alive_count() > 0);

        // Move and turn node b: its half follows rigidly, half a stays put.
        for _ in 0..240 {
            scene.update(&frame, 1.0 / 60.0, 0);
        }
        let settled_a = scene.gpu[0];
        let settled_b = scene.gpu[1];
        let r = Quat::from_rotation_z(0.5);
        frame.node_positions[1] += Vec3::new(0.3, -1.0, 0.2);
        frame.node_rotations[1] = r;
        scene.update(&frame, 1.0 / 60.0, 0);
        assert!((scene.gpu[0].pb - settled_a.pb).length() < 1e-4);
        let moved = scene.gpu[1];
        assert!((moved.pb.truncate() - frame.node_positions[1]).length() < 1e-4);
        let expect_ta = r * settled_b.ta.truncate();
        assert!((moved.ta.truncate() - expect_ta).length() < 1e-3);
    }

    #[test]
    fn halves_spring_back_then_settle() {
        let (mut scene, mut frame) = one_member(MaterialKind::Steel);
        // Bent like a fixed-fixed beam sagging: both ends rotated inwards.
        frame.node_rotations[0] = Quat::from_rotation_z(-0.15);
        frame.node_rotations[1] = Quat::from_rotation_z(0.15);
        scene.update(&frame, 1.0 / 60.0, 0);
        break_it(&mut scene, &mut frame);
        let first = scene.gpu[0];
        let mut moved = 0.0f32;
        let mut last = first;
        for _ in 0..300 {
            scene.update(&frame, 1.0 / 60.0, 0);
            moved = moved.max((scene.gpu[0].pb - first.pb).length());
            last = scene.gpu[0];
        }
        assert!(moved > 0.01, "free end should swing, moved {moved}");
        scene.update(&frame, 1.0 / 60.0, 0);
        assert!(
            (scene.gpu[0].pb - last.pb).length() < 1e-5,
            "should have settled"
        );
        // Relaxed half is straight: end tangents agree.
        assert!((last.ta.truncate() - last.tb.truncate()).length() < 1e-3);
    }

    #[test]
    fn reinforced_concrete_exposes_bars_and_sheds_chunks() {
        let (mut plain, mut f1) = one_member(MaterialKind::Concrete);
        let (mut rc, mut f2) = one_member(MaterialKind::ReinforcedConcrete);
        plain.update(&f1, 1.0 / 60.0, 0);
        rc.update(&f2, 1.0 / 60.0, 0);
        break_it(&mut plain, &mut f1);
        break_it(&mut rc, &mut f2);
        assert_eq!(plain.slots.len(), 2);
        assert_eq!(rc.slots.len(), 2 + 8);
        let bars = rc
            .gpu
            .iter()
            .filter(|g| g.info.w as u32 & flags::REBAR != 0)
            .count();
        assert_eq!(bars, 8);
        assert!(!rc.spawned.is_empty(), "concrete should shed loose chunks");
        // The crushed zone leaves a gap between the halves.
        assert!(rc.gpu[1].clip.x - rc.gpu[0].clip.y > 0.01);
        for g in &rc.gpu {
            assert!(g.pa.is_finite() && g.pb.is_finite() && g.ta.is_finite());
        }
    }

    fn jointed(looks: [JointLook; 2]) -> (VisualScene, VisualFrame) {
        let (scene, frame) = one_member(MaterialKind::Steel);
        let structure = scene.structure.with_joints(vec![looks]);
        (VisualScene::new(structure, SceneConfig::default()), frame)
    }

    #[test]
    fn joints_trim_the_member_and_sit_on_its_ends() {
        let (mut scene, frame) = jointed([JointLook::Bolted, JointLook::None]);
        scene.update(&frame, 1.0 / 60.0, 0);
        assert_eq!((scene.joints.len(), scene.hubs.len()), (1, 1));
        let back = scene.joints[0].back;
        assert!(back > 0.15 && back < 0.3, "{back}");
        // Only the jointed end is cut back.
        let clip = scene.gpu[0].clip;
        assert!((clip.x - back / 4.0).abs() < 1e-6 && clip.y == 1.0);
        let j = scene.joint_states[0];
        assert!(!j.failed);
        let cut = Vec3::new(back, 3.0, 0.0);
        assert!((j.pose.node.0 - cut).length() < 1e-5);
        assert!((j.pose.member.0 - cut).length() < 1e-5);
        assert_eq!(scene.hub_poses[0].0, Vec3::new(0.0, 3.0, 0.0));
    }

    #[test]
    fn a_pin_lets_the_member_turn_in_the_node() {
        let (mut scene, mut frame) = jointed([JointLook::Pinned, JointLook::Pinned]);
        // Both nodes turn; a pinned-pinned member stays straight.
        frame.node_rotations[0] = Quat::from_rotation_z(-0.2);
        frame.node_rotations[1] = Quat::from_rotation_z(0.2);
        scene.update(&frame, 1.0 / 60.0, 0);
        let g = scene.gpu[0];
        assert!((g.ta.truncate() - Vec3::X).length() < 1e-5);
        assert!((g.tb.truncate() - Vec3::X).length() < 1e-5);
        let j = scene.joint_states[0].pose;
        assert!((j.node.1.angle_between(j.member.1) - 0.2).abs() < 1e-3);
    }

    #[test]
    fn a_failed_connection_frees_the_member_whole() {
        let (mut scene, mut frame) = jointed([JointLook::Bolted, JointLook::Welded]);
        scene.update(&frame, 1.0 / 60.0, 0);
        frame.breaks.push(VisualBreak {
            member: 0,
            position: 0.05,
            cause: BreakCause::Connection,
        });
        frame.members[0].broken = true;
        scene.update(&frame, 1.0 / 60.0, 0);
        frame.breaks.clear();
        // Still one slot, no fracture faces, nothing hidden.
        assert_eq!((scene.slots.len(), scene.slots_revision), (1, 0));
        let g = scene.gpu[0];
        assert_eq!((g.clip.z, g.clip.w), (0.0, 0.0));
        assert_eq!(g.info.w as u32 & flags::HIDDEN, 0);
        assert!(scene.joint_states[0].failed && !scene.joint_states[1].failed);
        // Node a moves away: its side of the joint goes with it, the member
        // and its end plate stay with node b.
        for _ in 0..240 {
            scene.update(&frame, 1.0 / 60.0, 0);
        }
        let before = scene.joint_states[0].pose;
        frame.node_positions[0] += Vec3::new(0.0, -1.0, 0.0);
        scene.update(&frame, 1.0 / 60.0, 0);
        let after = scene.joint_states[0].pose;
        assert!((after.node.0 - before.node.0 - Vec3::new(0.0, -1.0, 0.0)).length() < 1e-5);
        assert!((after.member.0 - before.member.0).length() < 1e-5);
    }

    #[test]
    fn mid_span_breaks_keep_the_hardware_on_each_half() {
        let (mut scene, mut frame) = jointed([JointLook::Bolted, JointLook::Bolted]);
        scene.update(&frame, 1.0 / 60.0, 0);
        break_it(&mut scene, &mut frame);
        assert_eq!(scene.slots.len(), 2);
        let (a, b) = (scene.gpu[0], scene.gpu[1]);
        assert!(a.clip.x > 0.0 && b.clip.y < 1.0);
        assert!(scene.joint_states.iter().all(|j| !j.failed));
        for _ in 0..240 {
            scene.update(&frame, 1.0 / 60.0, 0);
        }
        frame.node_positions[1] += Vec3::new(0.0, -0.5, 0.0);
        let before = [scene.joint_states[0].pose, scene.joint_states[1].pose];
        scene.update(&frame, 1.0 / 60.0, 0);
        assert!((scene.joint_states[0].pose.member.0 - before[0].member.0).length() < 1e-5);
        let moved = scene.joint_states[1].pose.member.0 - before[1].member.0;
        assert!((moved - Vec3::new(0.0, -0.5, 0.0)).length() < 1e-4);
    }
}
