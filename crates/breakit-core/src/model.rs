//! The structure model shared by both solvers.

use crate::{Joint, Material, Section, Vec3};

/// A straight beam member between two nodes.
#[derive(Clone, Debug)]
pub struct Member {
    pub a: usize,
    pub b: usize,
    pub material: Material,
    pub section: Section,
    /// Connections at end a and end b. Rigid unless set.
    pub ends: [Joint; 2],
}

/// Restrained degrees of freedom at a node: [ux, uy, uz, rx, ry, rz].
#[derive(Clone, Debug)]
pub struct Support {
    pub node: usize,
    pub fixed: [bool; 6],
}

impl Support {
    /// Clamped: no translation, no rotation.
    pub fn fixed(node: usize) -> Self {
        Self {
            node,
            fixed: [true; 6],
        }
    }
    /// Pinned: no translation, free rotation.
    pub fn pinned(node: usize) -> Self {
        Self {
            node,
            fixed: [true, true, true, false, false, false],
        }
    }
    pub fn fixes_translation(&self) -> bool {
        self.fixed[0] && self.fixed[1] && self.fixed[2]
    }
    pub fn fixes_rotation(&self) -> bool {
        self.fixed[3] && self.fixed[4] && self.fixed[5]
    }
}

/// A point force (N) and moment (N·m) applied at a node, in world axes.
#[derive(Clone, Debug)]
pub struct NodeLoad {
    pub node: usize,
    pub force: Vec3,
    pub moment: Vec3,
}

impl NodeLoad {
    pub fn force(node: usize, force: Vec3) -> Self {
        Self {
            node,
            force,
            moment: Vec3::zeros(),
        }
    }
}

/// A 3D frame: nodes, members and supports.
#[derive(Clone, Debug, Default)]
pub struct Structure {
    pub nodes: Vec<Vec3>,
    pub members: Vec<Member>,
    pub supports: Vec<Support>,
}

impl Structure {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn add_node(&mut self, p: Vec3) -> usize {
        self.nodes.push(p);
        self.nodes.len() - 1
    }

    pub fn add_member(
        &mut self,
        a: usize,
        b: usize,
        material: Material,
        section: Section,
    ) -> usize {
        assert!(a != b, "member needs two different nodes");
        self.members.push(Member {
            a,
            b,
            material,
            section,
            ends: [Joint::RIGID; 2],
        });
        self.members.len() - 1
    }

    /// Set the connections at a member's two ends.
    pub fn set_ends(&mut self, m: usize, a: Joint, b: Joint) {
        self.members[m].ends = [a, b];
    }

    pub fn add_support(&mut self, s: Support) {
        self.supports.push(s);
    }

    pub fn member_length(&self, m: usize) -> f64 {
        let mem = &self.members[m];
        (self.nodes[mem.b] - self.nodes[mem.a]).norm()
    }

    /// Straight line of `segments` equal members from `start` to `end`.
    /// Returns the node indices in order.
    pub fn add_line(
        &mut self,
        start: Vec3,
        end: Vec3,
        segments: usize,
        material: &Material,
        section: &Section,
    ) -> Vec<usize> {
        assert!(segments > 0);
        let ids: Vec<usize> = (0..=segments)
            .map(|i| self.add_node(start + (end - start) * (i as f64 / segments as f64)))
            .collect();
        for w in ids.windows(2) {
            self.add_member(w[0], w[1], material.clone(), section.clone());
        }
        ids
    }

    /// Self-weight as lumped nodal loads (half of each member's weight per end).
    pub fn self_weight_loads(&self, g: f64) -> Vec<NodeLoad> {
        let mut f = vec![Vec3::zeros(); self.nodes.len()];
        for (i, m) in self.members.iter().enumerate() {
            let w = m.material.density * m.section.area * self.member_length(i) * g;
            f[m.a].y -= 0.5 * w;
            f[m.b].y -= 0.5 * w;
        }
        f.into_iter()
            .enumerate()
            .map(|(n, force)| NodeLoad::force(n, force))
            .collect()
    }
}
