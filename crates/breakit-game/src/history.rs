//! Undo and redo for build mode.
//!
//! [`History::observe`] runs after every frame's edits and snapshots the
//! design whenever its revision changed, so no edit has to remember to
//! record itself. Undo and redo swap whole designs; designs are small.

use bevy::prelude::*;

use crate::design::Design;

/// Most snapshots kept.
const LIMIT: usize = 200;

#[derive(Resource, Debug, Default)]
pub struct History {
    undo: Vec<Design>,
    redo: Vec<Design>,
    /// The design as last seen.
    last: Option<Design>,
}

impl History {
    /// Forget everything and start from `d` (a new sheet, a cleared bench).
    pub fn reset(&mut self, d: &Design) {
        self.undo.clear();
        self.redo.clear();
        self.last = Some(d.clone());
    }

    /// Records the design if it changed since the last call.
    pub fn observe(&mut self, d: &Design) {
        match &self.last {
            Some(last) if last.revision == d.revision => {}
            Some(last) => {
                self.undo.push(last.clone());
                if self.undo.len() > LIMIT {
                    self.undo.remove(0);
                }
                self.redo.clear();
                self.last = Some(d.clone());
            }
            None => self.last = Some(d.clone()),
        }
    }

    pub fn can_undo(&self) -> bool {
        !self.undo.is_empty()
    }

    pub fn can_redo(&self) -> bool {
        !self.redo.is_empty()
    }

    /// Steps back one edit. Returns false if there is nothing to undo.
    pub fn undo(&mut self, d: &mut Design) -> bool {
        let Some(prev) = self.undo.pop() else {
            return false;
        };
        self.redo.push(d.clone());
        self.restore(d, prev);
        true
    }

    pub fn redo(&mut self, d: &mut Design) -> bool {
        let Some(next) = self.redo.pop() else {
            return false;
        };
        self.undo.push(d.clone());
        self.restore(d, next);
        true
    }

    fn restore(&mut self, d: &mut Design, mut to: Design) {
        // Revisions only grow, so everything keyed on them recomputes.
        to.revision = d.revision + 1;
        *d = to;
        self.last = Some(d.clone());
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::design::BeamKind;

    #[test]
    fn undo_and_redo_walk_the_edits() {
        let mut d = Design::default();
        let mut h = History::default();
        h.reset(&d);
        let a = d.add_node(Vec3::ZERO);
        h.observe(&d);
        let b = d.add_node(Vec3::X);
        h.observe(&d);
        d.add_beam(a, b, BeamKind::SteelTube);
        h.observe(&d);
        assert_eq!(d.beams.len(), 1);

        assert!(h.undo(&mut d));
        h.observe(&d);
        assert_eq!((d.nodes.len(), d.beams.len()), (2, 0));
        assert!(h.undo(&mut d));
        assert_eq!(d.nodes.len(), 1);
        assert!(h.redo(&mut d));
        assert!(h.redo(&mut d));
        assert_eq!(d.beams.len(), 1);
        assert!(!h.redo(&mut d));

        // A new edit after undo drops the redo branch.
        h.undo(&mut d);
        d.add_node(Vec3::Y);
        h.observe(&d);
        assert!(!h.can_redo());
    }

    #[test]
    fn revisions_keep_increasing_through_undo() {
        let mut d = Design::default();
        let mut h = History::default();
        h.reset(&d);
        d.add_node(Vec3::ZERO);
        h.observe(&d);
        let before = d.revision;
        h.undo(&mut d);
        assert!(d.revision > before);
    }
}
