//! The player's design: nodes, beams, supports and loads, in game terms.
//!
//! This module has no rendering or input code, so it can be unit tested
//! headless. [`Design::to_structure`] is the single place where the game
//! hands a structure to `breakit-core`.

use bevy::math::Vec3;
use bevy::prelude::Resource;
use breakit_core::{Material, NodeLoad, Section, Structure, Support};
use breakit_render::breakit_mesh::{MaterialKind, MemberStyle, Profile};

/// What the player is building with. Each kind maps to a core material and
/// cross-section.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum BeamKind {
    /// Steel tube, 50 mm outer radius, 5 mm wall.
    #[default]
    SteelTube,
    /// Timber 100 x 150 mm.
    Timber,
}

impl BeamKind {
    pub fn material(self) -> Material {
        match self {
            BeamKind::SteelTube => Material::steel(),
            BeamKind::Timber => Material::timber(),
        }
    }

    /// The cross-section, the one description both the solver's section
    /// properties and the drawn mesh come from.
    pub fn profile(self) -> Profile {
        match self {
            BeamKind::SteelTube => Profile::Tube { r: 0.05, t: 0.005 },
            BeamKind::Timber => Profile::Rect { b: 0.1, h: 0.15 },
        }
    }

    pub fn section(self) -> Section {
        self.profile().section()
    }

    /// How the renderer draws this kind of beam.
    pub fn style(self) -> MemberStyle {
        MemberStyle {
            profile: self.profile(),
            kind: MaterialKind::from_material(&self.material()),
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            BeamKind::SteelTube => "steel tube",
            BeamKind::Timber => "timber",
        }
    }

    /// Every kind, in the order the parts drawer lists them (keys 1, 2, ...).
    pub const ALL: [BeamKind; 2] = [BeamKind::SteelTube, BeamKind::Timber];

    /// Mass per metre of member, kg/m.
    pub fn mass_per_metre(self) -> f32 {
        (self.section().area * self.material().density) as f32
    }

    /// Price per metre of member under a scenario cost model (joints not
    /// included).
    pub fn cost_per_metre(self, costs: &breakit_scenario::CostModel) -> f32 {
        self.mass_per_metre() * costs.price_per_kg(self.material().kind.into()) as f32
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Beam {
    pub a: usize,
    pub b: usize,
    pub kind: BeamKind,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SupportKind {
    /// All six degrees of freedom fixed.
    Fixed,
    /// Translations fixed, rotations free.
    Pinned,
}

/// Point load on each of the starter bridge's three middle bottom nodes, N.
pub const STARTER_LOAD: f32 = 100_000.0;

/// Load steps the F key cycles through, N (downward).
pub const LOAD_STEPS: [f32; 5] = [0.0, 10_000.0, 50_000.0, 100_000.0, 200_000.0];

/// The structure the player is building. Node and beam indices map one to
/// one onto `Structure` nodes and members.
#[derive(Resource, Clone, Debug, Default)]
pub struct Design {
    pub nodes: Vec<Vec3>,
    pub beams: Vec<Beam>,
    /// Support per node, if any.
    pub supports: Vec<Option<SupportKind>>,
    /// Applied point load per node, in newtons (world frame).
    pub loads: Vec<Vec3>,
    /// Nodes the scenario placed (payload points). They can't be
    /// moved, deleted or have their support changed.
    pub locked: Vec<bool>,
    /// The sheet's foundations: a node placed on one gets its support.
    pub anchors: Vec<(Vec3, SupportKind)>,
    /// Bumped on every edit so systems can react to changes cheaply.
    pub revision: u64,
}

impl Design {
    /// A small starter truss so a new session has something to look at.
    pub fn starter_bridge() -> Self {
        let mut d = Design::default();
        let span = 6;
        let mut bottom = Vec::new();
        let mut top = Vec::new();
        for i in 0..=span {
            bottom.push(d.add_node(Vec3::new(i as f32 - span as f32 / 2.0, 0.0, 0.0)));
        }
        for i in 0..span {
            top.push(d.add_node(Vec3::new(i as f32 + 0.5 - span as f32 / 2.0, 1.0, 0.0)));
        }
        for w in bottom.windows(2) {
            d.add_beam(w[0], w[1], BeamKind::SteelTube);
        }
        for w in top.windows(2) {
            d.add_beam(w[0], w[1], BeamKind::SteelTube);
        }
        for (i, &t) in top.iter().enumerate() {
            d.add_beam(bottom[i], t, BeamKind::SteelTube);
            d.add_beam(t, bottom[i + 1], BeamKind::SteelTube);
        }
        d.set_support(bottom[0], Some(SupportKind::Pinned));
        d.set_support(bottom[span], Some(SupportKind::Pinned));
        for &n in &bottom[2..=4] {
            d.loads[n] = Vec3::new(0.0, -STARTER_LOAD, 0.0);
        }
        d
    }

    fn touch(&mut self) {
        self.revision += 1;
    }

    /// Adds a node, or returns the existing one at (nearly) the same spot.
    pub fn add_node(&mut self, p: Vec3) -> usize {
        if let Some(i) = self.node_at(p, 1e-3) {
            return i;
        }
        self.nodes.push(p);
        let anchor = self.anchors.iter().find(|(a, _)| a.distance(p) <= 0.05);
        self.supports.push(anchor.map(|(_, k)| *k));
        self.loads.push(Vec3::ZERO);
        self.locked.push(false);
        self.touch();
        self.nodes.len() - 1
    }

    /// Adds a node the player can't move or delete, with an optional
    /// support.
    pub fn add_locked_node(&mut self, p: Vec3, support: Option<SupportKind>) -> usize {
        let i = self.add_node(p);
        self.locked[i] = true;
        self.supports[i] = support;
        i
    }

    pub fn is_locked(&self, node: usize) -> bool {
        self.locked.get(node).copied().unwrap_or(false)
    }

    pub fn node_at(&self, p: Vec3, tol: f32) -> Option<usize> {
        self.nodes.iter().position(|n| n.distance(p) <= tol)
    }

    /// Adds a beam between two distinct nodes. Returns `None` for a
    /// zero-length or duplicate beam.
    pub fn add_beam(&mut self, a: usize, b: usize, kind: BeamKind) -> Option<usize> {
        if a == b || a >= self.nodes.len() || b >= self.nodes.len() {
            return None;
        }
        if self
            .beams
            .iter()
            .any(|m| (m.a == a && m.b == b) || (m.a == b && m.b == a))
        {
            return None;
        }
        self.beams.push(Beam { a, b, kind });
        self.touch();
        Some(self.beams.len() - 1)
    }

    pub fn set_support(&mut self, node: usize, s: Option<SupportKind>) {
        self.supports[node] = s;
        self.touch();
    }

    /// Cycles a node's support: none, pinned, fixed, none. Locked nodes keep
    /// theirs.
    pub fn cycle_support(&mut self, node: usize) {
        if self.is_locked(node) {
            return;
        }
        let next = match self.supports[node] {
            None => Some(SupportKind::Pinned),
            Some(SupportKind::Pinned) => Some(SupportKind::Fixed),
            Some(SupportKind::Fixed) => None,
        };
        self.set_support(node, next);
    }

    /// Cycles a node's downward load through [`LOAD_STEPS`].
    pub fn cycle_load(&mut self, node: usize) {
        const STEPS: [f32; 5] = LOAD_STEPS;
        let cur = -self.loads[node].y;
        let i = STEPS
            .iter()
            .position(|&s| (s - cur).abs() < 1.0)
            .unwrap_or(0);
        self.loads[node] = Vec3::new(0.0, -STEPS[(i + 1) % STEPS.len()], 0.0);
        self.touch();
    }

    /// Removes a node and every beam touching it, keeping indices dense.
    /// Returns false, and changes nothing, for a locked node.
    pub fn remove_node(&mut self, node: usize) -> bool {
        if self.is_locked(node) || node >= self.nodes.len() {
            return false;
        }
        self.nodes.remove(node);
        self.supports.remove(node);
        self.loads.remove(node);
        self.locked.remove(node);
        self.beams.retain(|m| m.a != node && m.b != node);
        for m in &mut self.beams {
            if m.a > node {
                m.a -= 1;
            }
            if m.b > node {
                m.b -= 1;
            }
        }
        self.touch();
        true
    }

    pub fn remove_beam(&mut self, beam: usize) {
        if beam < self.beams.len() {
            self.beams.remove(beam);
            self.touch();
        }
    }

    pub fn set_beam_kind(&mut self, beam: usize, kind: BeamKind) {
        if let Some(b) = self.beams.get_mut(beam) {
            if b.kind != kind {
                b.kind = kind;
                self.touch();
            }
        }
    }

    pub fn beam_length(&self, beam: usize) -> f32 {
        let m = &self.beams[beam];
        self.nodes[m.a].distance(self.nodes[m.b])
    }

    pub fn total_length(&self) -> f32 {
        self.beams
            .iter()
            .map(|m| self.nodes[m.a].distance(self.nodes[m.b]))
            .sum()
    }

    /// Converts the design to a core `Structure`. Node `i` and beam `j` map
    /// to node `i` and member `j`.
    pub fn to_structure(&self) -> Structure {
        let mut s = Structure::new();
        for &p in &self.nodes {
            s.add_node(to_core(p));
        }
        for m in &self.beams {
            s.add_member(m.a, m.b, m.kind.material(), m.kind.section());
        }
        for (i, sup) in self.supports.iter().enumerate() {
            match sup {
                Some(SupportKind::Fixed) => s.add_support(Support::fixed(i)),
                Some(SupportKind::Pinned) => s.add_support(Support::pinned(i)),
                None => {}
            }
        }
        s
    }

    /// Player-applied point loads as core loads.
    pub fn node_loads(&self) -> Vec<NodeLoad> {
        self.loads
            .iter()
            .enumerate()
            .filter(|(_, f)| **f != Vec3::ZERO)
            .map(|(i, f)| NodeLoad::force(i, to_core(*f)))
            .collect()
    }
}

pub fn to_core(v: Vec3) -> breakit_core::Vec3 {
    breakit_core::Vec3::new(v.x as f64, v.y as f64, v.z as f64)
}

pub fn from_core(v: breakit_core::Vec3) -> Vec3 {
    Vec3::new(v.x as f32, v.y as f32, v.z as f32)
}

#[cfg(test)]
mod tests {
    use super::*;
    use breakit_core::solve_static;

    #[test]
    fn starter_bridge_maps_one_to_one_and_solves() {
        let d = Design::starter_bridge();
        let s = d.to_structure();
        assert_eq!(s.nodes.len(), d.nodes.len());
        assert_eq!(s.members.len(), d.beams.len());
        assert_eq!(s.supports.len(), 2);
        let r = solve_static(&s, &d.node_loads()).expect("starter bridge is stable");
        assert_eq!(r.member_forces.len(), d.beams.len());
        // Midspan sags under the load.
        let mid = 3;
        assert!(r.displacements[mid].y < 0.0);
    }

    #[test]
    fn duplicate_nodes_and_beams_are_rejected() {
        let mut d = Design::default();
        let a = d.add_node(Vec3::ZERO);
        let b = d.add_node(Vec3::X);
        assert_eq!(d.add_node(Vec3::ZERO), a);
        assert!(d.add_beam(a, b, BeamKind::SteelTube).is_some());
        assert!(d.add_beam(b, a, BeamKind::SteelTube).is_none());
        assert!(d.add_beam(a, a, BeamKind::SteelTube).is_none());
    }

    #[test]
    fn locked_nodes_survive_delete_and_keep_their_support() {
        let mut d = Design::default();
        let a = d.add_locked_node(Vec3::ZERO, Some(SupportKind::Pinned));
        let b = d.add_node(Vec3::X);
        d.add_beam(a, b, BeamKind::SteelTube);
        assert!(!d.remove_node(a));
        d.cycle_support(a);
        assert_eq!(d.supports[a], Some(SupportKind::Pinned));
        assert!(d.remove_node(b));
        assert_eq!(d.nodes.len(), 1);
        assert!(d.beams.is_empty());
    }

    #[test]
    fn member_mass_and_price_per_metre() {
        let steel = BeamKind::SteelTube;
        // Tube 50 mm radius, 5 mm wall: about 11.7 kg/m.
        assert!((steel.mass_per_metre() - 11.7).abs() < 0.1);
        let costs = breakit_scenario::CostModel::default();
        assert!(BeamKind::Timber.cost_per_metre(&costs) < steel.cost_per_metre(&costs));
    }

    #[test]
    fn removing_a_node_reindexes_beams() {
        let mut d = Design::default();
        let a = d.add_node(Vec3::ZERO);
        let b = d.add_node(Vec3::X);
        let c = d.add_node(Vec3::Y);
        d.add_beam(a, b, BeamKind::SteelTube);
        d.add_beam(b, c, BeamKind::Timber);
        d.add_beam(a, c, BeamKind::SteelTube);
        d.remove_node(a);
        assert_eq!(d.nodes.len(), 2);
        assert_eq!(d.beams.len(), 1);
        assert_eq!((d.beams[0].a, d.beams[0].b), (0, 1));
        assert_eq!(d.beams[0].kind, BeamKind::Timber);
    }

    #[test]
    fn load_cycles_back_to_zero() {
        let mut d = Design::default();
        let a = d.add_node(Vec3::ZERO);
        for _ in 0..LOAD_STEPS.len() {
            d.cycle_load(a);
        }
        assert_eq!(d.loads[a], Vec3::ZERO);
    }
}
