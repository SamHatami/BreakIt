//! Node-aware connection detailing.
//!
//! A joint is not only a type (bolted, welded...) but a physical detail that
//! depends on what meets at the node: how many members, at what angles, and
//! how big they are. [`detail_node`] looks at every member end at one node
//! and picks, per end, the joint type (unless the player set it), the
//! detail (gusset plate, end plate, fin plate, splice, clevis, cast node,
//! dowelled slotted-in plate, bearing, weld), the plate sizes, and factors
//! that scale the joint's stiffness and capacity:
//!
//! * three or more steel members share a **gusset plate** in their plane,
//!   sized from the deepest member and the shallowest angle;
//! * a member framing square (75 degrees or more to every other member)
//!   into a continuous run gets a **fin plate**, nominally pinned;
//! * two steel members in line are **spliced** with end plates, which is
//!   stiffer than a knee; two at an angle meet with **end plates**;
//! * a member alone at a node (a support) sits on a **base plate**;
//! * shallow angles (under 30 degrees) put the bolt group off the member
//!   axis: eccentricity cuts capacity, down to half at 0 degrees;
//! * crowded nodes (five or more members) lose some capacity too.
//!
//! Everything here is engine-free; the game turns it into solver joints and
//! the renderer draws the plates.

use breakit_core::MaterialKind;
use serde::{Deserialize, Serialize};

use crate::{auto_joint, JointSpec, MemberChoice, ProfileSpec};

/// One member end at a node.
#[derive(Clone, Debug, PartialEq)]
pub struct NodeEnd {
    /// Unit vector from the node along the member.
    pub dir: [f64; 3],
    pub choice: MemberChoice,
    /// Joint the player set at this end; `None` adapts.
    pub fixed: Option<JointSpec>,
    /// Joint types this member's family allows (see `Catalog::joints_for`).
    pub allowed: Vec<JointSpec>,
}

/// The physical connection at a member end.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum Detail {
    /// Shared plate in the plane of three or more members, bolted or welded.
    Gusset,
    /// Plate welded across the member end, bolted to the next member.
    EndPlate,
    /// Single plate on the web, nominally pinned.
    FinPlate,
    /// Two members in line, end plates bolted face to face.
    Splice,
    /// Plate under a member that ends at a support.
    BasePlate,
    /// Member welded straight on, no plate.
    Weld,
    /// Forked end and pin, for ties and round bars.
    Clevis,
    /// Steel plate slotted into the timber, fixed with dowels.
    SlottedPlate,
    /// Cast in one with the next concrete member.
    Cast,
    /// Concrete sitting on a bearing pad.
    Bearing,
}

impl Detail {
    pub fn label(self) -> &'static str {
        match self {
            Detail::Gusset => "gusset",
            Detail::EndPlate => "end plate",
            Detail::FinPlate => "fin plate",
            Detail::Splice => "splice",
            Detail::BasePlate => "base plate",
            Detail::Weld => "weld",
            Detail::Clevis => "clevis",
            Detail::SlottedPlate => "slotted plate",
            Detail::Cast => "cast",
            Detail::Bearing => "bearing",
        }
    }
}

/// A rectangular steel plate, metres.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Plate {
    /// Along the member.
    pub length: f64,
    /// Across the member.
    pub width: f64,
    pub thickness: f64,
}

/// What one member end gets.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EndDetail {
    pub joint: JointSpec,
    pub detail: Detail,
    /// Smallest angle to another member at the node, radians (pi if none).
    pub angle: f64,
    /// Multiplies the joint's force and moment capacity (eccentricity,
    /// crowding), 0.5..=1.
    pub capacity: f64,
    /// Multiplies the joint's finite rotational stiffness.
    pub stiffness: f64,
    /// The end's own plate (end plate, fin plate, splice plate, base plate,
    /// slotted plate, clevis), if any. Gusset ends share [`NodeDetail::gusset`].
    pub plate: Option<Plate>,
}

/// A gusset plate shared by the members at a node.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Gusset {
    /// Normal of the plate's plane (the members' plane), unit.
    pub normal: [f64; 3],
    /// How far the plate reaches from the node along each member, metres.
    pub reach: f64,
    pub thickness: f64,
}

/// The connection at one node: one [`EndDetail`] per input end, in order,
/// and the gusset if the node has one.
#[derive(Clone, Debug, PartialEq)]
pub struct NodeDetail {
    pub ends: Vec<EndDetail>,
    pub gusset: Option<Gusset>,
}

/// Standard plate thicknesses, mm.
const PLATES: [f64; 7] = [8.0, 10.0, 12.0, 15.0, 20.0, 25.0, 30.0];

fn plate_for(t_mm: f64) -> f64 {
    PLATES
        .into_iter()
        .find(|p| *p >= t_mm)
        .unwrap_or(PLATES[PLATES.len() - 1])
        / 1000.0
}

/// Depth, width and thickest wall of a section, mm.
fn dims(p: &ProfileSpec) -> (f64, f64, f64) {
    match *p {
        ProfileSpec::Rect { b, h } => (h, b, 0.0),
        ProfileSpec::Round { d } => (d, d, d / 2.0),
        ProfileSpec::Chs { d, t } => (d, d, t),
        ProfileSpec::Rhs { b, h, t } => (h, b, t),
        ProfileSpec::I { h, b, tw, tf } => (h, b, tw.max(tf)),
    }
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn angle(a: [f64; 3], b: [f64; 3]) -> f64 {
    dot(a, b).clamp(-1.0, 1.0).acos()
}

/// Capacity left at an angle `theta` (radians) to the nearest member:
/// full from 30 degrees, falling linearly to half at 0.
pub fn eccentricity_factor(theta: f64) -> f64 {
    let deg = theta.to_degrees();
    (0.5 + 0.5 * deg / 30.0).min(1.0)
}

/// Details every member end at a node. See the module docs for the rules.
pub fn detail_node(ends: &[NodeEnd]) -> NodeDetail {
    let n = ends.len();
    let steel_like = |e: &NodeEnd| {
        matches!(
            e.choice.material.kind(),
            MaterialKind::Steel | MaterialKind::Composite
        )
    };
    let gusset_node = n >= 3 && ends.iter().filter(|e| steel_like(e)).count() >= 3;
    let crowding = if n >= 5 { 0.9 } else { 1.0 };

    let mut out = Vec::with_capacity(n);
    for (i, e) in ends.iter().enumerate() {
        let others: Vec<&NodeEnd> = ends
            .iter()
            .enumerate()
            .filter(|(j, _)| *j != i)
            .map(|(_, o)| o)
            .collect();
        let theta = others
            .iter()
            .map(|o| angle(e.dir, o.dir))
            .fold(std::f64::consts::PI, f64::min);
        let through = others
            .iter()
            .any(|o| angle(e.dir, o.dir) > 160f64.to_radians());
        // Square framing into a continuous run: every other member at 75
        // degrees or more, and two of them in line with each other.
        let others_run = others.iter().enumerate().any(|(a, oa)| {
            others[a + 1..]
                .iter()
                .any(|ob| angle(oa.dir, ob.dir) > 160f64.to_radians())
        });
        let square = !through && others_run && theta >= 75f64.to_radians();

        let kinds: Vec<MaterialKind> = others.iter().map(|o| o.choice.material.kind()).collect();
        let joint = e.fixed.unwrap_or_else(|| {
            if steel_like(e) && square && e.allowed.contains(&JointSpec::Pinned) {
                JointSpec::Pinned
            } else {
                auto_joint(&e.choice, &kinds, &e.allowed)
            }
        });

        let (h, b, t) = dims(&e.choice.profile);
        let plate_t = plate_for(t + 2.0);
        let mm = |x: f64| x / 1000.0;
        let round = matches!(e.choice.profile, ProfileSpec::Round { .. });
        let (detail, plate, stiffness) = match (e.choice.material.kind(), joint) {
            (MaterialKind::Concrete, JointSpec::Monolithic) => (Detail::Cast, None, 1.0),
            (MaterialKind::Concrete, _) => (
                Detail::Bearing,
                Some(Plate {
                    length: mm(b),
                    width: mm(b),
                    thickness: 0.02,
                }),
                1.0,
            ),
            (MaterialKind::Timber, JointSpec::Dowelled | JointSpec::Bolted) => (
                Detail::SlottedPlate,
                Some(Plate {
                    length: mm(1.2 * h),
                    width: mm(0.8 * h),
                    thickness: 0.01,
                }),
                1.0,
            ),
            (MaterialKind::Timber, _) => (
                Detail::FinPlate,
                Some(Plate {
                    length: mm(0.8 * h),
                    width: mm(0.6 * h),
                    thickness: 0.008,
                }),
                1.0,
            ),
            (_, JointSpec::Pinned) if round || n <= 2 => (
                Detail::Clevis,
                Some(Plate {
                    length: mm(4.0 * h),
                    width: mm(3.0 * h),
                    thickness: plate_for(h / 2.0),
                }),
                1.0,
            ),
            (_, JointSpec::Pinned) if !gusset_node || square => (
                Detail::FinPlate,
                Some(Plate {
                    length: mm(0.6 * h).max(0.1),
                    width: 0.1,
                    thickness: plate_t,
                }),
                1.0,
            ),
            _ if gusset_node => (Detail::Gusset, None, 1.0),
            (_, JointSpec::Welded | JointSpec::Rigid) => (Detail::Weld, None, 1.0),
            _ if n == 1 => (
                Detail::BasePlate,
                Some(Plate {
                    length: mm(b + 100.0),
                    width: mm(h + 100.0),
                    thickness: plate_for(t + 10.0),
                }),
                1.0,
            ),
            // Spliced in line: deep end plates bolted face to face, about
            // twice as stiff as a knee.
            _ if through => (
                Detail::Splice,
                Some(Plate {
                    length: mm(h + 60.0),
                    width: mm(b + 20.0),
                    thickness: plate_t,
                }),
                2.0,
            ),
            _ => (
                Detail::EndPlate,
                Some(Plate {
                    length: mm(h + 40.0),
                    width: mm(b + 20.0),
                    thickness: plate_t,
                }),
                1.0,
            ),
        };
        out.push(EndDetail {
            joint,
            detail,
            angle: theta,
            capacity: eccentricity_factor(theta) * crowding,
            stiffness,
            plate,
        });
    }

    let gusset = gusset_node.then(|| {
        // The members' plane: the first pair that is not in line.
        let mut normal = [0.0, 0.0, 1.0];
        'pairs: for a in 0..n {
            for b in a + 1..n {
                let c = cross(ends[a].dir, ends[b].dir);
                let len = dot(c, c).sqrt();
                if len > 0.2 {
                    normal = c.map(|x| x / len);
                    break 'pairs;
                }
            }
        }
        let depth = ends
            .iter()
            .map(|e| dims(&e.choice.profile).0)
            .fold(0.0, f64::max);
        let wall = ends
            .iter()
            .map(|e| dims(&e.choice.profile).2)
            .fold(0.0, f64::max);
        let theta = out
            .iter()
            .map(|d| d.angle)
            .fold(std::f64::consts::PI, f64::min);
        // Room for two bolt rows past the member's edge, longer the
        // shallower the members meet.
        let reach = (depth / 1000.0 * (0.75 + 0.5 / theta.sin().max(0.25)) + 0.08).min(1.0);
        Gusset {
            normal,
            reach,
            thickness: plate_for(wall + 2.0 + 2.0 * (n as f64 - 3.0).max(0.0)),
        }
    });

    NodeDetail { ends: out, gusset }
}
