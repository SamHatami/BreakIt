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

/// Platform 3 m up on four columns at the corners of a 3 m square, with a
/// deck grid that has nodes under the P-01 pump and cabinet feet. `braced`
/// adds X bracing on all four sides.
pub fn platform(r: f64, t: f64, braced: bool) -> Structure {
    deck_on_legs(r, t, braced, 0.0, 3.0)
}

/// F-01 jetty: the P-01 deck at y = 1 on legs from the flume bed.
pub fn jetty(r: f64, t: f64, braced: bool) -> Structure {
    deck_on_legs(r, t, braced, -3.2, 1.0)
}

/// Four-legged tower for the W-01 sign: base at the corners of a 2 m
/// square, top ring at x = ±0.5, z = ±1 m and y = 8 m (the sign's feet),
/// `levels` braced storeys.
pub fn sign_tower(r: f64, t: f64, levels: usize) -> Structure {
    let (m, sec) = (Material::steel(), Section::tube(r, t));
    let mut s = Structure::new();
    let corners = [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)];
    let rings: Vec<Vec<usize>> = (0..=levels)
        .map(|k| {
            let f = k as f64 / levels as f64;
            corners
                .iter()
                .map(|&(x, z)| s.add_node(Vec3::new(x * (1.0 - 0.5 * f), 8.0 * f, z)))
                .collect()
        })
        .collect();
    for k in 0..=levels {
        for c in 0..4 {
            if k > 0 {
                s.add_member(rings[k][c], rings[k][(c + 1) % 4], m.clone(), sec.clone());
            }
            if k < levels {
                s.add_member(rings[k][c], rings[k + 1][c], m.clone(), sec.clone());
                s.add_member(
                    rings[k][c],
                    rings[k + 1][(c + 1) % 4],
                    m.clone(),
                    sec.clone(),
                );
            }
        }
    }
    // Plan diagonal at the top so the sign's frame is a rigid table.
    s.add_member(rings[levels][0], rings[levels][2], m.clone(), sec.clone());
    for &n in &rings[0] {
        s.add_support(Support::fixed(n));
    }
    s
}

fn deck_on_legs(r: f64, t: f64, braced: bool, y0: f64, y1: f64) -> Structure {
    let (m, sec) = (Material::steel(), Section::tube(r, t));
    let mut s = Structure::new();
    let xs = [-1.5, -1.0, -0.5, 0.5, 1.0, 1.5];
    let zs = [-1.5, -0.5, 0.5, 1.5];
    let mut grid = vec![vec![0; zs.len()]; xs.len()];
    for (i, &x) in xs.iter().enumerate() {
        for (k, &z) in zs.iter().enumerate() {
            grid[i][k] = s.add_node(Vec3::new(x, y1, z));
        }
    }
    for i in 0..xs.len() {
        for k in 0..zs.len() {
            if i + 1 < xs.len() {
                s.add_member(grid[i][k], grid[i + 1][k], m.clone(), sec.clone());
            }
            if k + 1 < zs.len() {
                s.add_member(grid[i][k], grid[i][k + 1], m.clone(), sec.clone());
            }
        }
    }
    let (last_x, last_z) = (xs.len() - 1, zs.len() - 1);
    let corners = [(0, 0), (last_x, 0), (last_x, last_z), (0, last_z)];
    let mut feet = Vec::new();
    for &(i, k) in &corners {
        let f = s.add_node(Vec3::new(xs[i], y0, zs[k]));
        s.add_member(f, grid[i][k], m.clone(), sec.clone());
        s.add_support(Support::fixed(f));
        feet.push(f);
    }
    if braced {
        for c in 0..4 {
            let (a, b) = (corners[c], corners[(c + 1) % 4]);
            s.add_member(feet[c], grid[b.0][b.1], m.clone(), sec.clone());
            s.add_member(feet[(c + 1) % 4], grid[a.0][a.1], m.clone(), sec.clone());
        }
    }
    s
}

/// F-02 floating footbridge: two stringers at x = ±0.75 from the near-bank
/// pins (z = -3.5) to z = 2.8, cross beams at every station, and a pontoon
/// along x (from -2.4 to 2.4 m, centred `draft` below the path) under each of
/// `pontoons` stations, each tied up to the path by posts and braces.
/// Frame members are `r` x `t` tubes, pontoons `pr` x `pt`.
pub fn pontoon_bridge(
    r: f64,
    t: f64,
    pr: f64,
    pt: f64,
    pontoons: &[usize],
    draft: f64,
) -> Structure {
    let (m, sec) = (Material::steel(), Section::tube(r, t));
    let float = Section::tube(pr, pt);
    let mut s = Structure::new();
    let zs: Vec<f64> = (0..7).map(|k| -3.5 + 1.05 * k as f64).collect();
    let rows: Vec<[usize; 2]> = zs
        .iter()
        .map(|&z| {
            [
                s.add_node(Vec3::new(-0.75, 0.0, z)),
                s.add_node(Vec3::new(0.75, 0.0, z)),
            ]
        })
        .collect();
    for k in 0..zs.len() {
        s.add_member(rows[k][0], rows[k][1], m.clone(), sec.clone());
        if k + 1 < zs.len() {
            for side in 0..2 {
                s.add_member(rows[k][side], rows[k + 1][side], m.clone(), sec.clone());
            }
            // Plan bracing so the path is a stiff plate.
            s.add_member(rows[k][0], rows[k + 1][1], m.clone(), sec.clone());
        }
    }
    for &k in pontoons {
        let z = zs[k];
        let xs = [-2.4, -0.75, 0.75, 2.4];
        let p: Vec<usize> = xs
            .iter()
            .map(|&x| s.add_node(Vec3::new(x, -draft, z)))
            .collect();
        for w in p.windows(2) {
            s.add_member(w[0], w[1], m.clone(), float.clone());
        }
        // Posts and braces up to the path.
        s.add_member(p[1], rows[k][0], m.clone(), sec.clone());
        s.add_member(p[2], rows[k][1], m.clone(), sec.clone());
        s.add_member(p[0], rows[k][0], m.clone(), sec.clone());
        s.add_member(p[3], rows[k][1], m.clone(), sec.clone());
        s.add_member(p[1], rows[k][1], m.clone(), sec.clone());
    }
    s.add_support(Support::pinned(rows[0][0]));
    s.add_support(Support::pinned(rows[0][1]));
    s
}
