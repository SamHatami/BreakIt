//! What does deforming N members cost the CPU per frame?
//!
//! Compares the two ways to draw a bending structure:
//!
//! * GPU path (what BreakIt does): pack one 176-byte record per member; the
//!   vertex shader bends the shared template.
//! * CPU path (the "pure polygon mesh" approach): bend every vertex of every
//!   member on the CPU and re-upload the vertex buffers.
//!
//! Run: `cargo run --release -p breakit-mesh --example visual_bench [members]`

use std::time::Instant;

use breakit_mesh::mesh::{member_template, TemplateOptions};
use breakit_mesh::pose::rest_frame;
use breakit_mesh::state::{MemberStyle, VisualMember};
use breakit_mesh::{
    MaterialKind, MemberVisualState, Profile, SceneConfig, VisualBreak, VisualFrame, VisualScene,
    VisualStructure,
};
use glam::{Quat, Vec3};

fn main() {
    let counts: Vec<usize> = match std::env::args().nth(1) {
        Some(a) => vec![a.parse().expect("member count")],
        None => vec![1_000, 5_000, 20_000],
    };
    let profile = Profile::Tube { r: 0.05, t: 0.005 };
    let lean = member_template(&profile, TemplateOptions::default());
    let detailed = member_template(&profile, TemplateOptions::detailed());
    println!(
        "template (steel tube): {} vertices / {} triangles lean, {} / {} detailed",
        lean.vertex_count(),
        lean.triangle_count(),
        detailed.vertex_count(),
        detailed.triangle_count()
    );
    println!(
        "{:>8} {:>14} {:>14} {:>12} {:>14} {:>12}",
        "members", "GPU path ms", "upload KB", "CPU path ms", "upload KB", "break µs"
    );
    for n in counts {
        // A chain of members with every node moved and turned each frame.
        let members: Vec<VisualMember> = (0..n)
            .map(|i| {
                let (a, b) = (
                    Vec3::new(i as f32 * 2.0, 3.0, 0.0),
                    Vec3::new(i as f32 * 2.0 + 2.0, 3.0, 0.0),
                );
                VisualMember {
                    a: i,
                    b: i + 1,
                    style: MemberStyle {
                        profile,
                        kind: MaterialKind::ALL[i % 5],
                    },
                    rest_frame: rest_frame(a, b),
                    rest_length: 2.0,
                    seed: (i as f32 * 0.618).fract(),
                }
            })
            .collect();
        let structure = VisualStructure::from_members(members, n + 1);
        let frame_at = |t: f32| VisualFrame {
            node_positions: (0..=n)
                .map(|i| Vec3::new(i as f32 * 2.0, 3.0 + 0.05 * (t + i as f32).sin(), 0.0))
                .collect(),
            node_rotations: (0..=n)
                .map(|i| Quat::from_rotation_z(0.05 * (t + i as f32).cos()))
                .collect(),
            node_velocities: Vec::new(),
            members: vec![
                MemberVisualState {
                    utilization: 0.5,
                    hinge: [[0.0, 0.0], [0.02, -0.01]],
                    ..Default::default()
                };
                n
            ],
            breaks: Vec::new(),
        };
        let mut scene = VisualScene::new(structure, SceneConfig::default());
        let frames = 30;

        // GPU path: everything the CPU does per frame.
        let f = frame_at(0.0);
        scene.update(&f, 1.0 / 60.0, 0);
        let t = Instant::now();
        for k in 0..frames {
            let f = frame_at(k as f32 * 0.1);
            let t0 = Instant::now();
            scene.update(&f, 1.0 / 60.0, 0);
            std::hint::black_box(&scene.gpu);
            let _ = t0;
        }
        // Subtract building the synthetic frames.
        let t_total = t.elapsed().as_secs_f64();
        let t = Instant::now();
        for k in 0..frames {
            std::hint::black_box(frame_at(k as f32 * 0.1));
        }
        let gpu_ms = (t_total - t.elapsed().as_secs_f64()).max(0.0) * 1e3 / frames as f64;
        let gpu_kb = scene.gpu.len() * std::mem::size_of_val(&scene.gpu[0]) / 1024;

        // CPU path: bend every vertex (positions and normals) of every member.
        let f = frame_at(0.3);
        let mut out_p = vec![[0.0f32; 3]; lean.vertex_count()];
        let mut out_n = out_p.clone();
        let cpu_frames = if n > 5_000 { 3 } else { 10 };
        let t = Instant::now();
        for _ in 0..cpu_frames {
            for i in 0..n {
                let pose = scene.structure.pose(i, &f);
                for (v, (p, nrm)) in lean.positions.iter().zip(&lean.normals).enumerate() {
                    let (wp, wn) = pose.deform(Vec3::from_array(*p), Vec3::from_array(*nrm));
                    out_p[v] = wp.to_array();
                    out_n[v] = wn.to_array();
                }
                std::hint::black_box((&out_p, &out_n));
            }
        }
        let cpu_ms = t.elapsed().as_secs_f64() * 1e3 / cpu_frames as f64;
        let cpu_kb = n * lean.vertex_count() * 24 / 1024;

        // One break: split into halves, shed chunks, emit particles.
        let breaks = n.min(200);
        let mut f = frame_at(0.3);
        f.breaks = (0..breaks)
            .map(|i| VisualBreak {
                member: i * (n / breaks),
                position: 0.5,
                cause: breakit_core::BreakCause::Brittle,
            })
            .collect();
        let before = Instant::now();
        scene.update(&f, 1.0 / 60.0, 0);
        let with = before.elapsed().as_secs_f64();
        f.breaks.clear();
        let before = Instant::now();
        scene.update(&f, 1.0 / 60.0, 0);
        let without = before.elapsed().as_secs_f64();
        let break_us = (with - without).max(0.0) * 1e6 / breaks as f64;

        println!(
            "{:>8} {:>14.3} {:>14} {:>12.2} {:>14} {:>12.1}",
            n, gpu_ms, gpu_kb, cpu_ms, cpu_kb, break_us
        );
    }
}
