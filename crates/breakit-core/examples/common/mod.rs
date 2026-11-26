//! Structures shared by the benchmark examples (`bench`, `gate`): a braced
//! tower frame and a 3D truss bridge at several sizes.
#![allow(
    dead_code,
    reason = "each example uses a different part of this module"
)]

use breakit_core::*;

pub struct Case {
    pub name: String,
    pub s: Structure,
    /// Node whose displacement is reported.
    pub probe: usize,
    /// Nodes that receive the sideways push, and its direction.
    pub pushed: Vec<usize>,
    pub push_dir: Vec3,
}

pub fn link(s: &mut Structure, a: usize, b: usize, sub: usize, mat: &Material, sec: &Section) {
    let (pa, pb) = (s.nodes[a], s.nodes[b]);
    let mut prev = a;
    for i in 1..sub {
        let n = s.add_node(pa + (pb - pa) * (i as f64 / sub as f64));
        s.add_member(prev, n, mat.clone(), sec.clone());
        prev = n;
    }
    s.add_member(prev, b, mat.clone(), sec.clone());
}

/// Multi-storey frame: columns on a grid, beams both ways on every floor,
/// diagonal bracing on the outer faces. Clamped at the base.
pub fn tower(nx: usize, nz: usize, floors: usize, sub: usize) -> Case {
    let mat = Material::steel();
    let col = Section::tube(0.10, 0.008);
    let beam = Section::rect(0.10, 0.20);
    let brace = Section::tube(0.05, 0.005);
    let (bay, h) = (4.0, 3.0);
    let mut s = Structure::new();
    let id = |i: usize, j: usize, k: usize| (k * (nz + 1) + j) * (nx + 1) + i;
    for k in 0..=floors {
        for j in 0..=nz {
            for i in 0..=nx {
                s.add_node(Vec3::new(i as f64 * bay, k as f64 * h, j as f64 * bay));
            }
        }
    }
    for k in 0..floors {
        for j in 0..=nz {
            for i in 0..=nx {
                link(&mut s, id(i, j, k), id(i, j, k + 1), sub, &mat, &col);
                if i < nx {
                    link(
                        &mut s,
                        id(i, j, k + 1),
                        id(i + 1, j, k + 1),
                        sub,
                        &mat,
                        &beam,
                    );
                    if (j == 0 || j == nz) && (i + k) % 2 == 0 {
                        link(&mut s, id(i, j, k), id(i + 1, j, k + 1), sub, &mat, &brace);
                    }
                }
                if j < nz {
                    link(
                        &mut s,
                        id(i, j, k + 1),
                        id(i, j + 1, k + 1),
                        sub,
                        &mat,
                        &beam,
                    );
                    if (i == 0 || i == nx) && (j + k) % 2 == 0 {
                        link(&mut s, id(i, j, k), id(i, j + 1, k + 1), sub, &mat, &brace);
                    }
                }
            }
        }
    }
    for j in 0..=nz {
        for i in 0..=nx {
            s.add_support(Support::fixed(id(i, j, 0)));
        }
    }
    let pushed = (0..=nz).map(|j| id(0, j, floors)).collect();
    Case {
        name: format!("tower {}x{}x{}", nx, nz, floors),
        probe: id(nx, nz, floors),
        s,
        pushed,
        push_dir: Vec3::new(1.0, 0.0, 0.3).normalize(),
    }
}

/// Through-truss bridge along X: two Warren trusses with verticals, floor
/// beams, deck stringer and top/bottom lateral bracing. Pinned at four corners.
pub fn bridge(panels: usize, sub: usize) -> Case {
    let mat = Material::steel();
    let chord = Section::tube(0.12, 0.010);
    let web = Section::tube(0.07, 0.006);
    let floor = Section::rect(0.10, 0.25);
    let lat = Section::tube(0.04, 0.004);
    let (p, w, h) = (4.0, 6.0, 5.0);
    let mut s = Structure::new();
    // node(i, side, top)
    let id = |i: usize, side: usize, top: usize| (i * 2 + side) * 2 + top;
    for i in 0..=panels {
        for side in 0..2 {
            for top in 0..2 {
                s.add_node(Vec3::new(i as f64 * p, top as f64 * h, side as f64 * w));
            }
        }
    }
    for i in 0..=panels {
        for side in 0..2 {
            link(&mut s, id(i, side, 0), id(i, side, 1), sub, &mat, &web);
            if i < panels {
                link(
                    &mut s,
                    id(i, side, 0),
                    id(i + 1, side, 0),
                    sub,
                    &mat,
                    &chord,
                );
                link(
                    &mut s,
                    id(i, side, 1),
                    id(i + 1, side, 1),
                    sub,
                    &mat,
                    &chord,
                );
                let (lo, hi) = if i % 2 == 0 { (i, i + 1) } else { (i + 1, i) };
                link(&mut s, id(lo, side, 0), id(hi, side, 1), sub, &mat, &web);
            }
        }
        link(&mut s, id(i, 0, 0), id(i, 1, 0), sub, &mat, &floor);
        link(&mut s, id(i, 0, 1), id(i, 1, 1), sub, &mat, &lat);
        if i < panels {
            link(&mut s, id(i, 0, 0), id(i + 1, 1, 0), sub, &mat, &lat);
            link(
                &mut s,
                id(i, i % 2, 1),
                id(i + 1, 1 - i % 2, 1),
                sub,
                &mat,
                &lat,
            );
        }
    }
    for i in [0, panels] {
        for side in 0..2 {
            s.add_support(Support::pinned(id(i, side, 0)));
        }
    }
    let mid = panels / 2;
    Case {
        name: format!("bridge {} panels", panels),
        probe: id(mid, 0, 0),
        s,
        pushed: vec![id(mid, 0, 0), id(mid, 1, 0)],
        push_dir: Vec3::new(0.0, -1.0, 0.1).normalize(),
    }
}

pub fn cases(size: &str) -> Vec<Case> {
    match size {
        "s" => vec![tower(2, 2, 4, 1), bridge(8, 1)],
        "m" => vec![tower(3, 3, 8, 1), bridge(20, 2)],
        "l" => vec![tower(5, 5, 12, 1), bridge(60, 3)],
        "xl" => vec![tower(8, 8, 16, 1), bridge(120, 4)],
        _ => panic!("unknown size {size}"),
    }
}

pub fn weight(s: &Structure) -> f64 {
    s.self_weight_loads(9.81).iter().map(|l| -l.force.y).sum()
}

/// The DOF numbering of the dynamic solver for `s`: the number of unknowns
/// and, per member, the first DOF of each of its four 3-DOF slots.
pub fn dof_blocks(s: &Structure) -> (usize, Vec<[Option<usize>; 4]>) {
    let mut fixed = vec![(false, false); s.nodes.len()];
    for sup in &s.supports {
        fixed[sup.node].0 |= sup.fixes_translation();
        fixed[sup.node].1 |= sup.fixes_rotation();
    }
    let mut n = 0;
    let mut next = |free: bool| {
        free.then(|| {
            n += 3;
            n - 3
        })
    };
    let slots: Vec<_> = fixed.iter().map(|f| (next(!f.0), next(!f.1))).collect();
    let blocks = s
        .members
        .iter()
        .map(|m| [slots[m.a].0, slots[m.a].1, slots[m.b].0, slots[m.b].1])
        .collect();
    (n, blocks)
}
