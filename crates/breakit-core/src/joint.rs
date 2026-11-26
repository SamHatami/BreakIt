//! Member end connections.
//!
//! Every member end carries a [`Joint`]: a rotational spring in series with
//! the member's bending stiffness, a moment capacity past which the
//! connection yields, a plastic rotation capacity past which it fails, and a
//! force capacity for the resultant of axial and shear force through it.
//! Torsion always passes through a joint: a pinned end is a ball joint in
//! bending only, so a pinned-pinned member cannot spin freely about its axis.
//!
//! [`JointKind`] turns the connection types the player picks into numbers
//! relative to the member they connect. Stiffness classes follow EN 1993-1-8
//! 5.2.2: pinned below 0.5 EI/L, rigid above 25 EI/L for a braced frame,
//! semi-rigid in between. Capacities are game values chosen so each type has
//! a clear trade-off, not design-code resistances.

use crate::concrete::RcSection;
use crate::material::{MaterialKind, Model};
use crate::{Material, Section};

/// One member end's connection to its node.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Joint {
    /// Rotational stiffness about the member's local y and z, N·m/rad.
    /// Infinite = rigid, 0 = pinned.
    pub stiffness: [f64; 2],
    /// End moment about local y and z at which the connection yields, N·m.
    /// Infinite = at least as strong as the member.
    pub moment_capacity: [f64; 2],
    /// Plastic rotation at this end (both axes summed, joint and member
    /// hinge alike) at which the connection fails, rad.
    pub rotation_capacity: f64,
    /// Resultant of axial and shear force at which the connection fails, N.
    pub force_capacity: f64,
}

impl Joint {
    /// Full-strength, rigid: the member alone decides. The default.
    pub const RIGID: Joint = Joint {
        stiffness: [f64::INFINITY; 2],
        moment_capacity: [f64::INFINITY; 2],
        rotation_capacity: f64::INFINITY,
        force_capacity: f64::INFINITY,
    };

    /// Free to rotate in bending, never fails.
    pub const PINNED: Joint = Joint {
        stiffness: [0.0; 2],
        ..Joint::RIGID
    };

    pub fn is_rigid(&self) -> bool {
        *self == Joint::RIGID
    }

    /// True if the joint passes no moment about either axis.
    pub fn is_pinned(&self) -> bool {
        self.stiffness == [0.0; 2]
    }

    /// Utilisation of the connection under axial force `n`, shears `v` and
    /// end moments `m` (per axis), 1.0 = at capacity. Plastic rotation is
    /// checked separately.
    pub fn utilization(&self, n: f64, v: [f64; 2], m: [f64; 2]) -> f64 {
        let force = (n * n + v[0] * v[0] + v[1] * v[1]).sqrt() / self.force_capacity;
        let moment: f64 = (0..2)
            .filter(|&a| self.stiffness[a] > 0.0)
            .map(|a| m[a].abs() / self.moment_capacity[a])
            .sum();
        force.max(moment)
    }
}

impl Default for Joint {
    fn default() -> Self {
        Joint::RIGID
    }
}

/// The connection types the player chooses from.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum JointKind {
    /// Rigid and full strength, never the weak point. Sandbox and tests.
    Rigid,
    /// A single pin: no moment, cheap, fails when the pin shears.
    Pinned,
    /// Bolted end plate: semi-rigid, yields before the member does and
    /// rotates a good deal before the bolts let go.
    Bolted,
    /// Full-strength weld, rigid. The member yields first, but the weld
    /// cracks after little plastic rotation at its face.
    Welded,
    /// Reinforced concrete cast in one piece: rigid, the member decides.
    Monolithic,
    /// Timber: steel plate with dowels. Flexible, weak, ductile.
    Dowelled,
}

impl JointKind {
    pub const ALL: [JointKind; 6] = [
        JointKind::Rigid,
        JointKind::Pinned,
        JointKind::Bolted,
        JointKind::Welded,
        JointKind::Monolithic,
        JointKind::Dowelled,
    ];

    /// Whether this connection can be made in the given material.
    pub fn suits(self, kind: MaterialKind) -> bool {
        use MaterialKind::*;
        match self {
            JointKind::Rigid | JointKind::Pinned => true,
            JointKind::Bolted => matches!(kind, Steel | Timber | Composite),
            JointKind::Welded => kind == Steel,
            JointKind::Monolithic => kind == Concrete,
            JointKind::Dowelled => kind == Timber,
        }
    }

    /// The connection's numbers for one end of a member of `length` metres.
    pub fn joint(self, mat: &Material, sec: &Section, length: f64) -> Joint {
        let (n_member, m_member) = member_capacity(mat, sec);
        let ei_l = [
            mat.youngs_modulus * sec.iy / length,
            mat.youngs_modulus * sec.iz / length,
        ];
        let inf = f64::INFINITY;
        match self {
            JointKind::Rigid | JointKind::Monolithic => Joint::RIGID,
            JointKind::Pinned => Joint {
                stiffness: [0.0; 2],
                moment_capacity: [inf; 2],
                rotation_capacity: inf,
                force_capacity: 0.6 * n_member,
            },
            JointKind::Bolted => Joint {
                stiffness: ei_l.map(|k| 4.0 * k),
                moment_capacity: m_member.map(|m| 0.45 * m),
                rotation_capacity: 0.06,
                force_capacity: 0.8 * n_member,
            },
            JointKind::Welded => Joint {
                rotation_capacity: 0.025,
                ..Joint::RIGID
            },
            JointKind::Dowelled => Joint {
                stiffness: ei_l.map(|k| 1.5 * k),
                moment_capacity: m_member.map(|m| 0.3 * m),
                rotation_capacity: 0.04,
                force_capacity: 0.5 * n_member,
            },
        }
    }
}

/// Tension capacity (N) and moment capacity per axis (N·m) of a member,
/// the reference joint capacities are scaled from.
pub fn member_capacity(mat: &Material, sec: &Section) -> (f64, [f64; 2]) {
    match &mat.model {
        Model::Elastic => (f64::INFINITY, [f64::INFINITY; 2]),
        Model::Ductile(p) => (
            p.yield_strength * sec.area,
            [p.yield_strength * sec.zy, p.yield_strength * sec.zz],
        ),
        Model::Timber(t) => (
            t.tension * sec.area,
            [t.bending * sec.wy, t.bending * sec.wz],
        ),
        Model::Concrete(c) => match &c.rebar {
            Some(_) => {
                let rc = RcSection::new(sec, mat.youngs_modulus, c);
                (
                    rc.tension_yield,
                    [
                        rc.axes[0].moment_capacity(0.0),
                        rc.axes[1].moment_capacity(0.0),
                    ],
                )
            }
            None => (
                c.tension * sec.area,
                [c.tension * sec.wy, c.tension * sec.wz],
            ),
        },
        Model::Composite(c) => (
            c.tension * sec.area,
            [c.tension * sec.wy, c.tension * sec.wz],
        ),
    }
}

/// End-moment stiffness of one bending plane, M = K θ, for a member of
/// flexural stiffness `ei_l` = EI/L with rotational springs `k` (N·m/rad,
/// infinite = rigid, 0 = pinned) in series at its two ends. θ are the end
/// rotations of the nodes relative to the member chord.
///
/// The member's flexibility is L/(6EI) [[2, −1], [−1, 2]]; each spring adds
/// 1/k to its diagonal, and a pinned end is condensed out exactly, so a pinned
/// end has an exactly zero row and column. Rigid ends give the usual
/// EI/L [[4, 2], [2, 4]].
pub fn bending_stiffness(ei_l: f64, k: [f64; 2]) -> [[f64; 2]; 2] {
    if ei_l <= 0.0 || (k[0] == 0.0 && k[1] == 0.0) {
        return [[0.0; 2]; 2];
    }
    let s = k.map(|k| if k.is_infinite() { 0.0 } else { 1.0 / k });
    let f = 1.0 / (6.0 * ei_l);
    if k[0] == 0.0 {
        return [[0.0, 0.0], [0.0, 1.0 / (2.0 * f + s[1])]];
    }
    if k[1] == 0.0 {
        return [[1.0 / (2.0 * f + s[0]), 0.0], [0.0, 0.0]];
    }
    let (a, b, d) = (2.0 * f + s[0], -f, 2.0 * f + s[1]);
    let det = a * d - b * b;
    [[d / det, -b / det], [-b / det, a / det]]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rigid_ends_give_the_classic_beam() {
        let k = bending_stiffness(2.0, [f64::INFINITY; 2]);
        for (got, want) in k.iter().flatten().zip([8.0, 4.0, 4.0, 8.0]) {
            assert!((got - want).abs() < 1e-12, "{k:?}");
        }
    }

    #[test]
    fn one_pinned_end_gives_3ei_over_l() {
        let k = bending_stiffness(2.0, [0.0, f64::INFINITY]);
        assert_eq!(k[0], [0.0, 0.0]);
        assert_eq!(k[1][0], 0.0);
        assert!((k[1][1] - 6.0).abs() < 1e-12);
    }

    #[test]
    fn stiff_springs_approach_rigid_and_soft_ones_pinned() {
        let rigid = bending_stiffness(1.0, [f64::INFINITY; 2]);
        let stiff = bending_stiffness(1.0, [1e9; 2]);
        let soft = bending_stiffness(1.0, [1e-9; 2]);
        assert!((stiff[0][0] - rigid[0][0]).abs() < 1e-6);
        assert!(soft[0][0].abs() < 1e-8);
    }
}
