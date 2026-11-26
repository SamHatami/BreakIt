//! Sample designs shared by the tests and the probe example.
#![allow(dead_code)]

use breakit_core::{Material, Section, Structure, Support, Vec3};

/// Planar Warren truss like the game's starter bridge: 6 bays, 1 m deep.
pub fn warren(r: f64, t: f64, depth: f64) -> Structure {
    let (m, sec) = (Material::steel(), Section::tube(r, t));
    let mut s = Structure::new();
    let bottom: Vec<usize> = (0..=6)
        .map(|i| s.add_node(Vec3::new(i as f64 - 3.0, 0.0, 0.0)))
        .collect();
    let top: Vec<usize> = (0..6)
        .map(|i| s.add_node(Vec3::new(i as f64 - 2.5, depth, 0.0)))
        .collect();
    for w in bottom.windows(2) {
        s.add_member(w[0], w[1], m.clone(), sec.clone());
    }
    for w in top.windows(2) {
        s.add_member(w[0], w[1], m.clone(), sec.clone());
    }
    for (i, &tn) in top.iter().enumerate() {
        s.add_member(bottom[i], tn, m.clone(), sec.clone());
        s.add_member(tn, bottom[i + 1], m.clone(), sec.clone());
    }
    s.add_support(Support::pinned(bottom[0]));
    s.add_support(Support::pinned(bottom[6]));
    s
}

/// Four-legged mast tapering to the head, with ring braces.
pub fn mast(r: f64, t: f64, levels: usize) -> Structure {
    let (m, sec) = (Material::steel(), Section::tube(r, t));
    let mut s = Structure::new();
    let corners = [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)];
    let mut rings: Vec<Vec<usize>> = Vec::new();
    for k in 0..levels {
        let y = 6.0 * k as f64 / levels as f64;
        let f = 1.0 - 0.8 * k as f64 / levels as f64;
        rings.push(
            corners
                .iter()
                .map(|&(x, z)| s.add_node(Vec3::new(x * f, y, z * f)))
                .collect(),
        );
    }
    let head = s.add_node(Vec3::new(0.0, 6.0, 0.0));
    for k in 0..levels {
        for c in 0..4 {
            let a = rings[k][c];
            if k > 0 {
                s.add_member(rings[k][c], rings[k][(c + 1) % 4], m.clone(), sec.clone());
            }
            let up = if k + 1 < levels {
                rings[k + 1][c]
            } else {
                head
            };
            s.add_member(a, up, m.clone(), sec.clone());
            if k + 1 < levels {
                s.add_member(a, rings[k + 1][(c + 1) % 4], m.clone(), sec.clone());
            }
        }
    }
    for &n in &rings[0] {
        s.add_support(Support::fixed(n));
    }
    s
}
