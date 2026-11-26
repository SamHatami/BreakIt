//! Wind panels and engineering checks against hand calculations and
//! published numbers.
mod common;
use std::f64::consts::{PI, TAU};

use breakit_core::loads::wind::checks::*;
use breakit_core::loads::wind::panels::*;
use breakit_core::loads::wind::{Wind, AIR_DENSITY};
use breakit_core::loads::{Geometry, View};
use breakit_core::{DynamicSim, Material, Section, SimConfig, Structure, Support, Vec3};
use common::*;

// ---------------------------------------------------------------- Rayleigh

#[test]
fn rayleigh_cantilever_matches_euler_bernoulli() {
    // Steel tube 100 x 5 mm, 4 m, 20 elements, vibrating across (y).
    // f1 = 3.516 / (2π L²) √(EI / ρA).
    let l = 4.0;
    let (s, _) = cantilever(l, 20, Material::steel());
    let sec = tube();
    let m = Material::steel();
    let exact = 3.516 / (TAU * l * l) * (m.youngs_modulus * sec.iz / (m.density * sec.area)).sqrt();
    let f = fundamental_frequency(&s, Vec3::y()).unwrap();
    println!("cantilever: Rayleigh {f:.5} Hz, exact {exact:.5} Hz");
    assert!(rel_err(f, exact) < 0.01, "{f} vs {exact}");
}

#[test]
fn rayleigh_simply_supported_beam() {
    // f1 = π / (2 L²) √(EI / ρA).
    let l = 6.0;
    let mut s = Structure::new();
    let ids = s.add_line(
        Vec3::zeros(),
        Vec3::new(l, 0.0, 0.0),
        20,
        &Material::steel(),
        &tube(),
    );
    // Pin both ends; one also holds the twist about the beam axis.
    s.add_support(Support {
        node: ids[0],
        fixed: [true, true, true, true, false, false],
    });
    s.add_support(Support::pinned(ids[20]));
    let sec = tube();
    let m = Material::steel();
    let exact = PI / (2.0 * l * l) * (m.youngs_modulus * sec.iz / (m.density * sec.area)).sqrt();
    let f = fundamental_frequency(&s, Vec3::y()).unwrap();
    println!("simply supported: Rayleigh {f:.5} Hz, exact {exact:.5} Hz");
    assert!(rel_err(f, exact) < 0.01, "{f} vs {exact}");
}

#[test]
fn rayleigh_tip_mass() {
    // Massless-ish cantilever with a big tip mass: f = (1/2π) √(3EI / (M L³)),
    // with a third of a percent from the beam's own mass (0.23 m_beam added).
    let l = 2.0;
    let (s, ids) = cantilever(l, 10, Material::steel());
    let sec = tube();
    let m = Material::steel();
    let m_beam = m.density * sec.area * l;
    let tip = 1000.0;
    let k = 3.0 * m.youngs_modulus * sec.iz / l.powi(3);
    let exact = (k / (tip + 0.2357 * m_beam)).sqrt() / TAU;
    let f = fundamental_frequency_with_masses(&s, Vec3::y(), &[(ids[10], tip)]).unwrap();
    println!("tip mass: Rayleigh {f:.5} Hz, expected {exact:.5} Hz");
    assert!(rel_err(f, exact) < 0.01);
    assert_eq!(
        fundamental_frequency_with_masses(&s, Vec3::y(), &[(99, 1.0)]),
        Err(CheckError::Node(99))
    );
    assert_eq!(
        fundamental_frequency(&s, Vec3::zeros()),
        Err(CheckError::Direction)
    );
}

// ---------------------------------------------------------- Vortex shedding

#[test]
fn strouhal_table_points() {
    let rect = Section::rect(1.0, 1.0).shape;
    for (r, st) in [
        (1.0, 0.12),
        (2.0, 0.06),
        (3.0, 0.06),
        (3.5, 0.15),
        (5.0, 0.11),
        (10.0, 0.09),
    ] {
        assert!(
            (strouhal_number(&rect, r, 1.0) - st).abs() < 1e-12,
            "d/b {r}"
        );
    }
    assert!((strouhal_number(&rect, 1.5, 1.0) - 0.09).abs() < 1e-12);
    assert_eq!(strouhal_number(&tube().shape, 0.1, 0.1), 0.18);
}

#[test]
fn chimney_critical_speed_and_scruton() {
    // Welded steel chimney: b = 1.0 m, f1 = 1.0 Hz, St = 0.18 (circle),
    // m_e = 300 kg/m, δ_s = 0.012 (EN 1991-1-4 Table F.2), ρ = 1.25.
    // v_crit = f b / St = 5.556 m/s (E.2); Sc = 2 δ m / (ρ b²) = 5.76 (E.4).
    let st = strouhal_number(&tube().shape, 1.0, 1.0);
    let v = critical_vortex_speed(1.0, 1.0, st);
    assert!((v - 1.0 / 0.18).abs() < 1e-12, "{v}");
    let sc = scruton_number(300.0, 0.012, 1.25, 1.0);
    assert!((sc - 5.76).abs() < 1e-12, "{sc}");
    // A 0.3 m square sign post at 2.5 Hz: St 0.12, v_crit = 6.25 m/s.
    let sq = Section::rect(0.3, 0.3).shape;
    let v = critical_vortex_speed(2.5, 0.3, strouhal_number(&sq, 0.3, 0.3));
    assert!((v - 6.25).abs() < 1e-12, "{v}");
}

// ---------------------------------------------------------------- Galloping

#[test]
fn galloping_square_section() {
    // Square b = 0.5 m, f1 = 2 Hz, Sc = 10, a_G = 1.2 (EN Table E.7):
    // v_CG = 2 Sc f b / a_G = 16.67 m/s (E.18).
    let sq = Section::rect(0.5, 0.5).shape;
    let ag = galloping_factor(&sq, 0.5, 0.5).unwrap();
    assert!((ag - 1.2).abs() < 1e-12);
    let v = galloping_onset_speed(2.0, 0.5, 10.0, ag);
    assert!((v - 2.0 * 10.0 * 2.0 * 0.5 / 1.2).abs() < 1e-12, "{v}");
    assert!((v - 16.6667).abs() < 1e-3);
    // Table points and its limits.
    assert!((galloping_factor(&sq, 0.5, 1.0).unwrap() - 0.7).abs() < 1e-12);
    assert!((galloping_factor(&sq, 1.0, 1.5).unwrap() - 1.0).abs() < 1e-12);
    assert!((galloping_factor(&sq, 1.5, 1.0).unwrap() - 1.7).abs() < 1e-12);
    assert_eq!(galloping_factor(&sq, 3.0, 1.0), None);
    assert_eq!(galloping_factor(&tube().shape, 0.1, 0.1), None);
    assert_eq!(galloping_onset_speed(2.0, 0.5, 10.0, 0.0), f64::INFINITY);
    // Den Hartog: a square (dC_L/dα ≈ −2.7, C_D ≈ 2.0, Parkinson & Smith
    // 1964) gallops; a circle (dC_L/dα = 0) does not.
    assert!(den_hartog_unstable(-2.7, 2.0));
    assert!(!den_hartog_unstable(0.0, 1.2));
}

// ------------------------------------------------------------------ Flutter

#[test]
fn selberg_great_belt_east_bridge() {
    // Great Belt East Bridge (Larsen, JWEIA 48, 1993; the usual flutter
    // benchmark): B = 31 m, m = 22.74 t/m, I = 2.47e6 kg m²/m,
    // f_b = 0.099 Hz, f_t = 0.272 Hz. Flat-plate flutter about 74 m/s
    // (section-model tests found flutter above 70 m/s).
    let (b, m, i, fb, ft) = (31.0, 22_740.0, 2.47e6, 0.099, 0.272);
    let u = selberg_flutter_speed(b, m, i, fb, ft, AIR_DENSITY).unwrap();
    let r = rocard_flutter_speed(b, m, i, fb, ft, AIR_DENSITY).unwrap();
    println!("Great Belt: Selberg {u:.1} m/s, Rocard {r:.1} m/s");
    assert!(rel_err(u, 74.0) < 0.05, "{u}");
    assert!(rel_err(u, r) < 0.03, "Selberg {u} vs Rocard {r}");
    assert_eq!(
        selberg_flutter_speed(b, m, i, 0.3, 0.272, AIR_DENSITY),
        None
    );
}

#[test]
fn coherence() {
    assert!((davenport_coherence(0.1, 10.0, 20.0, 10.0) - (-0.5f64).exp()).abs() < 1e-15);
    assert_eq!(davenport_coherence(1.0, 0.0, 20.0, 10.0), 1.0);
}

// ------------------------------------------------------------------- Panels

/// Two posts at z = 0 and z = 6 with rails at y = 2 and y = 5: the frame of
/// a 6 x 3 m sign in the plane x = 0. Returns the structure and the corner
/// nodes (bottom-left, bottom-right, top-right, top-left seen from −x).
fn sign_frame() -> (Structure, [usize; 4]) {
    let mut s = Structure::new();
    let mat = Material::steel();
    let sec = Section::tube(0.1, 0.008);
    let p0 = s.add_line(Vec3::zeros(), Vec3::new(0.0, 5.0, 0.0), 5, &mat, &sec);
    let p1 = s.add_line(
        Vec3::new(0.0, 0.0, 6.0),
        Vec3::new(0.0, 5.0, 6.0),
        5,
        &mat,
        &sec,
    );
    s.add_member(p0[2], p1[2], mat.clone(), sec.clone());
    s.add_member(p0[5], p1[5], mat, sec);
    s.add_support(Support::fixed(p0[0]));
    s.add_support(Support::fixed(p1[0]));
    (s, [p0[2], p1[2], p1[5], p0[5]])
}

fn corners() -> Vec<Vec3> {
    vec![
        Vec3::new(0.0, 2.0, 0.0),
        Vec3::new(0.0, 2.0, 6.0),
        Vec3::new(0.0, 5.0, 6.0),
        Vec3::new(0.0, 5.0, 0.0),
    ]
}

/// Panel forces per node for wind at `speed` blowing along `dir`.
fn panel_forces(spec: &PanelSpec, speed: f64, dir: Vec3) -> (Vec<Vec3>, PanelState, Structure) {
    let (s, _) = sign_frame();
    let mut state = spec.resolve(&s).unwrap();
    let mut wind = Wind::new(speed, dir);
    wind.prepare(&s);
    let sim = DynamicSim::new(
        &s,
        SimConfig {
            gravity: Vec3::zeros(),
            ..Default::default()
        },
    );
    let geometry = Geometry::new(&s);
    let view = View {
        structure: &s,
        sim: &sim,
        geometry: &geometry,
        time: 0.0,
    };
    let mut out = vec![Vec3::zeros(); s.nodes.len()];
    apply_panel(
        spec,
        &mut state,
        &view,
        AIR_DENSITY,
        1.0,
        &|p| wind.velocity(p, 0.0, 1.0),
        &mut out,
    );
    (out, state, s)
}

/// Sum of forces and moment about `about`.
fn resultant(out: &[Vec3], s: &Structure, about: Vec3) -> (Vec3, Vec3) {
    let f = out.iter().sum();
    let m = out
        .iter()
        .zip(&s.nodes)
        .map(|(f, p)| (p - about).cross(f))
        .sum();
    (f, m)
}

#[test]
fn panel_resolves_corners() {
    let (s, nodes) = sign_frame();
    let state = PanelSpec::sign("sign", corners()).resolve(&s).unwrap();
    assert_eq!(state.nodes, nodes.to_vec());
    assert!((state.area - 18.0).abs() < 1e-12);
    let mut far = corners();
    far[2].x = 1.0;
    let err = PanelSpec::sign("sign", far).resolve(&s).unwrap_err();
    assert_eq!(
        err,
        PanelError::NoNode {
            panel: "sign".into(),
            corner: 2
        }
    );
    assert!(err.to_string().contains("corner 2"));
    let two = PanelSpec::sign("sign", corners()[..2].to_vec()).resolve(&s);
    assert!(matches!(two, Err(PanelError::CornerCount { count: 2, .. })));
}

#[test]
fn panel_square_on_matches_signboard_force() {
    // 6 x 3 m sign, wind 40 m/s square on, c_f = 1.8 (EN 1991-1-4 §7.4.3):
    // F = ½ ρ U² c_f A = 0.5 · 1.225 · 1600 · 1.8 · 18 = 31 752 N.
    let spec = PanelSpec::sign("sign", corners());
    let (out, state, s) = panel_forces(&spec, 40.0, Vec3::x());
    let expected = 0.5 * AIR_DENSITY * 40.0 * 40.0 * 1.8 * 18.0;
    let (f, m) = resultant(&out, &s, state.center);
    println!("square on: F = {:.3} N, expected {expected:.3} N", f.x);
    assert!(rel_err(f.x, expected) < 1e-9, "{f}");
    assert!(f.y.abs() < 1e-9 * expected && f.z.abs() < 1e-9 * expected);
    assert!(m.norm() < 1e-9 * expected, "centred load, moment {m}");
    assert!(rel_err(state.force.x, expected) < 1e-9);
    assert!(rel_err(state.net_pressure.abs(), expected / 18.0) < 1e-9);
    // The plate normal points along −x for this corner order, so the wind
    // pushes against it: a negative net pressure.
    assert!(state.net_pressure < 0.0);
    assert_eq!(state.rig("sign").net_pressure, state.net_pressure);
}

#[test]
fn panel_eccentric_torsion_case() {
    // EN 1991-1-4 §7.4.3: the force acts with e = 0.25 b sideways, giving a
    // torsion moment F · 0.25 b about the vertical axis (b = 6 m).
    let mut spec = PanelSpec::sign("sign", corners());
    spec.eccentricity = 0.25;
    let (out, state, s) = panel_forces(&spec, 40.0, Vec3::x());
    let expected = 0.5 * AIR_DENSITY * 40.0 * 40.0 * 1.8 * 18.0;
    let (f, m) = resultant(&out, &s, state.center);
    let torsion = expected * 0.25 * 6.0;
    println!("eccentric: M_y = {:.3} N m, expected ±{torsion:.3}", m.y);
    assert!(rel_err(f.x, expected) < 1e-9);
    assert!(rel_err(m.y.abs(), torsion) < 1e-9, "{m}");
    assert!(m.x.abs() < 1e-6 * torsion && m.z.abs() < 1e-6 * torsion);
}

#[test]
fn panel_oblique_45_degrees() {
    // Wind at 45° to the plate: C_N = 1.8 · 0.505 sin α / (0.222 sin α +
    // 0.283) = 0.812 · 1.8 (Hoerner). The centre of pressure moves toward
    // the leading edge: 0.375 of the 6 m chord, 0.75 m upstream of the
    // middle. Friction off to isolate the normal force.
    let mut spec = PanelSpec::sign("sign", corners());
    spec.friction_coefficient = 0.0;
    let dir = Vec3::new(1.0, 0.0, 1.0).normalize();
    let (out, state, s) = panel_forces(&spec, 40.0, dir);
    let sa = 0.5f64.sqrt();
    let cn = 1.8 * 0.505 * sa / (0.222 * sa + 0.283);
    assert!((cn / 1.8 - 0.8116).abs() < 1e-3);
    assert!((normal_force_coefficient(PI / 4.0, 1.8) - cn).abs() < 1e-12);
    let expected = 0.5 * AIR_DENSITY * 1600.0 * cn * 18.0;
    let (f, m) = resultant(&out, &s, state.center);
    println!("45°: F = {f}, expected {expected:.3} along x");
    assert!(rel_err(f.x, expected) < 1e-9, "{f}");
    assert!(f.z.abs() < 1e-9 * expected, "no friction: normal only");
    // Wind runs toward +z along the plate, so the leading edge is at z = 0:
    // CoP at z = 3 − 0.75 = 2.25. Moment about y of a +x force at Δz = −0.75.
    assert!((state.center_of_pressure.z - 2.25).abs() < 1e-9);
    assert!(rel_err(m.y, -0.75 * expected) < 1e-9, "{m}");
    // With friction: the in-plane component is 2 c_fr ½ρ|w| w_t A.
    spec.friction_coefficient = 0.01;
    let (out, _, _) = panel_forces(&spec, 40.0, dir);
    let f: Vec3 = out.iter().sum();
    let fric = 0.01 * 2.0 * 18.0 * 0.5 * AIR_DENSITY * 40.0 * 40.0 * sa;
    assert!(rel_err(f.z, fric) < 1e-9, "{} vs {fric}", f.z);
}

#[test]
fn panel_edge_on_carries_only_friction() {
    let spec = PanelSpec::sign("sign", corners());
    let (out, state, _) = panel_forces(&spec, 40.0, Vec3::z());
    let f: Vec3 = out.iter().sum();
    let q = 0.5 * AIR_DENSITY * 1600.0;
    assert!(f.x.abs() < 1e-9 * q * 18.0, "normal {f}");
    assert!(rel_err(f.z, 0.01 * q * 2.0 * 18.0) < 1e-9, "{f}");
    assert!(state.net_pressure.abs() < 1e-9);
}

#[test]
fn panel_porosity_reduces_force() {
    // K_p = 1 − (1 − δ)² with δ = 0.7 (30 % open): 0.91 (AS/NZS 1170.2 D).
    assert!((porosity_factor(0.3) - 0.91).abs() < 1e-12);
    assert_eq!(porosity_factor(0.0), 1.0);
    assert_eq!(porosity_factor(1.0), 0.0);
    let mut spec = PanelSpec::sign("mesh", corners());
    spec.porosity = 0.3;
    let (out, _, _) = panel_forces(&spec, 40.0, Vec3::x());
    let f: Vec3 = out.iter().sum();
    let expected = 0.91 * 0.5 * AIR_DENSITY * 1600.0 * 1.8 * 18.0;
    assert!(rel_err(f.x, expected) < 1e-9);
}

#[test]
fn panel_triangle() {
    // Right triangle of 9 m² square on: full force, CoP at the centroid.
    let c = corners();
    let spec = PanelSpec::sign("tri", vec![c[0], c[1], c[2]]);
    let (out, state, s) = panel_forces(&spec, 40.0, Vec3::x());
    let expected = 0.5 * AIR_DENSITY * 1600.0 * 1.8 * 9.0;
    let (f, m) = resultant(&out, &s, state.center);
    assert!(rel_err(f.x, expected) < 1e-9);
    assert!(m.norm() < 1e-9 * expected);
}
