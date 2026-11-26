use breakit_parts::node::*;
use breakit_parts::*;

fn end(c: &Catalog, id: &str, deg: f64) -> NodeEnd {
    let choice = c.part(id).unwrap().choice;
    let r = deg.to_radians();
    NodeEnd {
        dir: [r.cos(), r.sin(), 0.0],
        choice,
        fixed: None,
        allowed: c.joints_for(&choice),
    }
}

#[test]
fn truss_node_shares_a_gusset_in_the_members_plane() {
    let c = Catalog::standard();
    let d = detail_node(&[
        end(&c, "steel-tube-m", 0.0),
        end(&c, "steel-tube-m", 180.0),
        end(&c, "steel-tube-s", 60.0),
        end(&c, "steel-tube-s", 120.0),
    ]);
    let g = d.gusset.expect("four steel members share a gusset");
    assert!((g.normal[2].abs() - 1.0).abs() < 1e-9);
    assert!(g.reach > 0.1 && g.reach < 1.0);
    for e in &d.ends {
        assert_eq!(e.detail, Detail::Gusset);
        assert_eq!(e.joint, JointSpec::Bolted);
        assert_eq!(e.capacity, 1.0);
    }
}

#[test]
fn shallow_angles_lose_capacity_and_grow_the_gusset() {
    let c = Catalog::standard();
    let steep = detail_node(&[
        end(&c, "steel-tube-m", 0.0),
        end(&c, "steel-tube-m", 180.0),
        end(&c, "steel-tube-s", 60.0),
    ]);
    let shallow = detail_node(&[
        end(&c, "steel-tube-m", 0.0),
        end(&c, "steel-tube-m", 180.0),
        end(&c, "steel-tube-s", 12.0),
    ]);
    assert_eq!(steep.ends[2].capacity, 1.0);
    let f = shallow.ends[2].capacity;
    assert!((f - eccentricity_factor(12f64.to_radians())).abs() < 1e-12);
    assert!(f < 0.75 && f >= 0.5);
    assert!(shallow.gusset.unwrap().reach > steep.gusset.unwrap().reach);
    assert!(eccentricity_factor(0.0) == 0.5);
}

#[test]
fn square_framing_into_a_run_is_a_pinned_fin_plate() {
    let c = Catalog::standard();
    let d = detail_node(&[
        end(&c, "steel-column", 90.0),
        end(&c, "steel-column", 270.0),
        end(&c, "steel-ibeam-m", 0.0),
    ]);
    assert_eq!(d.ends[2].joint, JointSpec::Pinned);
    assert_eq!(d.ends[2].detail, Detail::FinPlate);
    assert!(d.ends[2].plate.is_some());
}

#[test]
fn two_members_splice_in_line_and_end_plate_at_a_knee() {
    let c = Catalog::standard();
    let line = detail_node(&[
        end(&c, "steel-ibeam-m", 0.0),
        end(&c, "steel-ibeam-m", 180.0),
    ]);
    assert_eq!(line.ends[0].detail, Detail::Splice);
    assert!(line.ends[0].stiffness > 1.0);
    let knee = detail_node(&[end(&c, "steel-ibeam-m", 0.0), end(&c, "steel-column", 90.0)]);
    assert_eq!(knee.ends[0].detail, Detail::EndPlate);
    assert!(knee.gusset.is_none());
    let base = detail_node(&[end(&c, "steel-column", 90.0)]);
    assert_eq!(base.ends[0].detail, Detail::BasePlate);
    // Plates come from standard thicknesses and scale with the section.
    let big = detail_node(&[
        end(&c, "steel-ibeam-l", 0.0),
        end(&c, "steel-ibeam-l", 90.0),
    ]);
    let (p, q) = (knee.ends[0].plate.unwrap(), big.ends[0].plate.unwrap());
    assert!(q.length > p.length && q.thickness >= p.thickness);
}

#[test]
fn other_materials_get_their_own_details() {
    let c = Catalog::standard();
    let timber = detail_node(&[end(&c, "timber-joist", 0.0), end(&c, "timber-post", 90.0)]);
    assert_eq!(timber.ends[0].joint, JointSpec::Dowelled);
    assert_eq!(timber.ends[0].detail, Detail::SlottedPlate);
    let rc = detail_node(&[end(&c, "rc-beam", 0.0), end(&c, "rc-column", 90.0)]);
    assert_eq!(rc.ends[0].detail, Detail::Cast);
    let rod = detail_node(&[end(&c, "steel-rod", 30.0), end(&c, "steel-column", 90.0)]);
    assert_eq!(rod.ends[0].detail, Detail::Clevis);
    // A player's choice wins over the automatic one.
    let mut e = end(&c, "steel-tube-m", 0.0);
    e.fixed = Some(JointSpec::Welded);
    let w = detail_node(&[e, end(&c, "steel-tube-m", 90.0)]);
    assert_eq!(
        (w.ends[0].joint, w.ends[0].detail),
        (JointSpec::Welded, Detail::Weld)
    );
}
