//! Saving and loading playground setups: the design, the table and the
//! bench, as RON files in a `playground` folder next to the log folder
//! (see `release.rs`), or under `BREAKIT_PLAYGROUND_DIR`.

use std::path::PathBuf;

use bevy::math::Vec3;
use breakit_playground::{
    saved::VERSION, SavedBeam, SavedDesign, SavedPlayground, SavedSupport, Setup,
};

use crate::design::{Beam, BeamKind, Design, SupportKind};

/// Folder the slots live in.
pub fn folder() -> PathBuf {
    if let Some(d) = std::env::var_os("BREAKIT_PLAYGROUND_DIR") {
        return PathBuf::from(d);
    }
    crate::release::log_path()
        .and_then(|p| p.parent()?.parent().map(|d| d.join("playground")))
        .unwrap_or_else(|| std::env::temp_dir().join("BreakIt").join("playground"))
}

fn slot_path(slot: u8) -> PathBuf {
    folder().join(format!("slot{slot}.ron"))
}

pub fn save_design(d: &Design) -> SavedDesign {
    SavedDesign {
        nodes: d.nodes.iter().map(|p| p.to_array()).collect(),
        beams: d
            .beams
            .iter()
            .map(|b| SavedBeam {
                a: b.a,
                b: b.b,
                member: b.kind.0,
                joints: b.joints,
            })
            .collect(),
        supports: d
            .supports
            .iter()
            .map(|s| {
                s.map(|k| match k {
                    SupportKind::Fixed => SavedSupport::Fixed,
                    SupportKind::Pinned => SavedSupport::Pinned,
                })
            })
            .collect(),
        loads: d.loads.iter().map(|l| l.to_array()).collect(),
    }
}

/// The design a file describes (no anchors, nothing locked).
pub fn design_from(s: &SavedDesign) -> Design {
    let n = s.nodes.len();
    let mut supports: Vec<Option<SupportKind>> = s
        .supports
        .iter()
        .map(|k| {
            k.map(|k| match k {
                SavedSupport::Fixed => SupportKind::Fixed,
                SavedSupport::Pinned => SupportKind::Pinned,
            })
        })
        .collect();
    supports.resize(n, None);
    let mut loads: Vec<Vec3> = s.loads.iter().map(|l| Vec3::from_array(*l)).collect();
    loads.resize(n, Vec3::ZERO);
    Design {
        nodes: s.nodes.iter().map(|p| Vec3::from_array(*p)).collect(),
        beams: s
            .beams
            .iter()
            .map(|b| Beam {
                a: b.a,
                b: b.b,
                kind: BeamKind(b.member),
                joints: b.joints,
            })
            .collect(),
        supports,
        loads,
        locked: vec![false; n],
        ..Default::default()
    }
}

/// Writes a slot; returns where.
pub fn save(slot: u8, d: &Design, table: usize, setup: &Setup) -> std::io::Result<PathBuf> {
    let file = SavedPlayground {
        version: VERSION,
        table,
        setup: setup.clone(),
        design: save_design(d),
    };
    let path = slot_path(slot);
    if let Some(dir) = path.parent() {
        std::fs::create_dir_all(dir)?;
    }
    std::fs::write(&path, file.to_ron())?;
    Ok(path)
}

/// A slot could not be loaded.
#[derive(Debug)]
pub enum LoadError {
    Empty,
    Io(std::io::Error),
    File(breakit_playground::FileError),
}

impl std::fmt::Display for LoadError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            LoadError::Empty => write!(f, "nothing saved yet"),
            LoadError::Io(e) => write!(f, "{e}"),
            LoadError::File(e) => write!(f, "{e}"),
        }
    }
}

pub fn load(slot: u8) -> Result<SavedPlayground, LoadError> {
    let text = match std::fs::read_to_string(slot_path(slot)) {
        Ok(t) => t,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => return Err(LoadError::Empty),
        Err(e) => return Err(LoadError::Io(e)),
    };
    SavedPlayground::from_ron(&text).map_err(LoadError::File)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn designs_round_trip_through_a_file() {
        let mut d = Design::starter_bridge();
        d.beams[0].joints[1] = Some(breakit_parts::JointSpec::Pinned);
        let back = design_from(&save_design(&d));
        assert_eq!(back.nodes, d.nodes);
        assert_eq!(back.beams, d.beams);
        assert_eq!(back.supports, d.supports);
        assert_eq!(back.loads, d.loads);
        let dir = std::env::temp_dir().join(format!("breakit-pg-{}", std::process::id()));
        let file = SavedPlayground {
            version: VERSION,
            table: 2,
            setup: Setup::default(),
            design: save_design(&d),
        };
        std::fs::create_dir_all(&dir).unwrap();
        let p = dir.join("slot9.ron");
        std::fs::write(&p, file.to_ron()).unwrap();
        let read = SavedPlayground::from_ron(&std::fs::read_to_string(&p).unwrap()).unwrap();
        assert_eq!(design_from(&read.design).beams, d.beams);
        let _ = std::fs::remove_dir_all(&dir);
    }
}
