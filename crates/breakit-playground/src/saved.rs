//! Playground files: the structure, the table and the load bench in one
//! RON document, so a setup can be saved and loaded again.

use breakit_parts::{JointSpec, MemberChoice};
use serde::{Deserialize, Serialize};

use crate::Setup;

/// File format version written by this build.
pub const VERSION: u32 = 1;

/// A support as stored.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum SavedSupport {
    Fixed,
    Pinned,
}

/// A member as stored.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct SavedBeam {
    pub a: usize,
    pub b: usize,
    pub member: MemberChoice,
    /// Joint at each end; `None` adapts to the node.
    pub joints: [Option<JointSpec>; 2],
}

/// The player's structure, engine free: positions in m, loads in N.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct SavedDesign {
    pub nodes: Vec<[f32; 3]>,
    pub beams: Vec<SavedBeam>,
    pub supports: Vec<Option<SavedSupport>>,
    pub loads: Vec<[f32; 3]>,
}

/// One saved playground.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SavedPlayground {
    pub version: u32,
    /// Built-in play table index.
    pub table: usize,
    pub setup: Setup,
    pub design: SavedDesign,
}

/// A playground file that could not be read.
#[derive(Debug)]
pub enum FileError {
    Parse(String),
    /// Written by a newer build.
    Version(u32),
    /// A beam or load names a node that is not there.
    BadNode(usize),
}

impl std::fmt::Display for FileError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            FileError::Parse(e) => write!(f, "not a playground file: {e}"),
            FileError::Version(v) => write!(f, "saved by a newer version (format {v})"),
            FileError::BadNode(n) => write!(f, "refers to node {n}, which is not in the file"),
        }
    }
}

impl std::error::Error for FileError {}

impl SavedPlayground {
    pub fn to_ron(&self) -> String {
        ron::ser::to_string_pretty(self, ron::ser::PrettyConfig::default())
            .expect("playground data always serialises")
    }

    /// Parses and checks a file.
    pub fn from_ron(text: &str) -> Result<Self, FileError> {
        let p: Self = ron::from_str(text).map_err(|e| FileError::Parse(e.to_string()))?;
        if p.version > VERSION {
            return Err(FileError::Version(p.version));
        }
        let n = p.design.nodes.len();
        for b in &p.design.beams {
            for k in [b.a, b.b] {
                if k >= n {
                    return Err(FileError::BadNode(k));
                }
            }
        }
        if let Some(l) = p.setup.point_loads.iter().find(|l| l.node >= n) {
            return Err(FileError::BadNode(l.node));
        }
        Ok(p)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use breakit_parts::Catalog;

    fn sample() -> SavedPlayground {
        let cat = Catalog::standard();
        let member = cat.parts[0].choice;
        let mut setup = Setup::default();
        setup.wind.on = true;
        setup.point_loads.push(crate::PointLoad {
            node: 1,
            force: [0.0, -1000.0, 0.0],
        });
        SavedPlayground {
            version: VERSION,
            table: 2,
            setup,
            design: SavedDesign {
                nodes: vec![[0.0, 0.0, 0.0], [2.0, 0.0, 0.0]],
                beams: vec![SavedBeam {
                    a: 0,
                    b: 1,
                    member,
                    joints: [None, Some(JointSpec::Pinned)],
                }],
                supports: vec![Some(SavedSupport::Fixed), None],
                loads: vec![[0.0; 3]; 2],
            },
        }
    }

    #[test]
    fn round_trips_through_ron() {
        let p = sample();
        let text = p.to_ron();
        assert_eq!(SavedPlayground::from_ron(&text).unwrap(), p);
    }

    #[test]
    fn rejects_dangling_nodes_and_newer_files() {
        let mut p = sample();
        p.design.beams[0].b = 7;
        assert!(matches!(
            SavedPlayground::from_ron(&p.to_ron()),
            Err(FileError::BadNode(7))
        ));
        let mut p = sample();
        p.version = VERSION + 1;
        assert!(SavedPlayground::from_ron(&p.to_ron()).is_err());
        assert!(SavedPlayground::from_ron("nonsense").is_err());
    }
}
