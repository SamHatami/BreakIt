//! Times one 60 Hz frame on a 3D truss bridge of roughly `bays` x 16 members.
use breakit_core::*;
use std::time::Instant;

fn main() {
    let bays: usize = std::env::args()
        .nth(1)
        .map(|a| a.parse().unwrap())
        .unwrap_or(120);
    let (mat, sec) = (Material::steel(), Section::tube(0.05, 0.005));
    let mut s = Structure::new();
    let (w, hgt, bay) = (3.0, 3.0, 2.0);
    // Four chords: bottom-left, bottom-right, top-left, top-right.
    let mut chords = vec![Vec::new(); 4];
    for i in 0..=bays {
        let x = i as f64 * bay;
        for (c, (y, z)) in [(0.0, 0.0), (0.0, w), (hgt, 0.0), (hgt, w)]
            .iter()
            .enumerate()
        {
            chords[c].push(s.add_node(Vec3::new(x, *y, *z)));
        }
    }
    let m = |s: &mut Structure, a: usize, b: usize| {
        s.add_member(a, b, mat.clone(), sec.clone());
    };
    for i in 0..=bays {
        let n: Vec<usize> = (0..4).map(|c| chords[c][i]).collect();
        m(&mut s, n[0], n[1]);
        m(&mut s, n[2], n[3]);
        m(&mut s, n[0], n[2]);
        m(&mut s, n[1], n[3]);
        m(&mut s, n[0], n[3]);
        if i < bays {
            for c in 0..4 {
                m(&mut s, chords[c][i], chords[c][i + 1]);
            }
            m(&mut s, chords[0][i], chords[2][i + 1]);
            m(&mut s, chords[1][i], chords[3][i + 1]);
            m(&mut s, chords[0][i], chords[1][i + 1]);
            m(&mut s, chords[2][i], chords[3][i + 1]);
        }
    }
    for c in [0, 1] {
        s.add_support(Support::pinned(chords[c][0]));
        s.add_support(Support::pinned(chords[c][bays]));
    }
    println!("{} nodes, {} members", s.nodes.len(), s.members.len());
    let t = Instant::now();
    let r = solve_static(&s, &s.self_weight_loads(9.81));
    println!(
        "static solve (dense): {:.1} ms, ok={}",
        t.elapsed().as_secs_f64() * 1e3,
        r.is_ok()
    );
    let mut sim = DynamicSim::new(&s, SimConfig::default());
    sim.step(1.0 / 60.0);
    let frames = 30;
    let t = Instant::now();
    for _ in 0..frames {
        sim.step(1.0 / 60.0);
    }
    println!(
        "dynamic frame (16 substeps x 1 iter): {:.2} ms",
        t.elapsed().as_secs_f64() * 1e3 / frames as f64
    );
}
