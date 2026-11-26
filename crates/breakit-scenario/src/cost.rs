//! What a structure costs and weighs, in kronor at real installed rates.
//!
//! Cost is mass times a price per kilogram for each material, plus a price
//! per member end for its joint, by joint type (pinned 1x, dowelled 1.5x,
//! bolted 2x, monolithic 3x, welded 4x, from the components design). A
//! core `Joint` does not remember its kind, so the game passes the joint
//! kinds per member end; without them every end is priced as bolted.

use serde::{Deserialize, Serialize};

use breakit_core::Structure;

use crate::scenario::MaterialClass;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CostModel {
    /// Price per kg of steel, timber, concrete and composite.
    pub steel: f64,
    pub timber: f64,
    pub concrete: f64,
    pub composite: f64,
    /// Price of one member end's joint, by kind.
    pub joints: JointPrices,
}

/// Joint kinds as the cost model sees them, mirroring the components
/// design's `JointKind`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum JointClass {
    Pinned,
    Dowelled,
    #[default]
    Bolted,
    Monolithic,
    Welded,
    /// Ideal rigid joint, sandbox only.
    Rigid,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct JointPrices {
    pub pinned: f64,
    pub dowelled: f64,
    pub bolted: f64,
    pub monolithic: f64,
    pub welded: f64,
    pub rigid: f64,
}

impl Default for JointPrices {
    fn default() -> Self {
        Self {
            pinned: 400.0,
            dowelled: 600.0,
            bolted: 800.0,
            monolithic: 1200.0,
            welded: 1600.0,
            rigid: 1600.0,
        }
    }
}

impl JointPrices {
    pub fn price(&self, j: JointClass) -> f64 {
        match j {
            JointClass::Pinned => self.pinned,
            JointClass::Dowelled => self.dowelled,
            JointClass::Bolted => self.bolted,
            JointClass::Monolithic => self.monolithic,
            JointClass::Welded => self.welded,
            JointClass::Rigid => self.rigid,
        }
    }
}

impl Default for CostModel {
    fn default() -> Self {
        Self {
            steel: 50.0,
            timber: 24.0,
            concrete: 5.0,
            composite: 600.0,
            joints: JointPrices::default(),
        }
    }
}

impl CostModel {
    pub fn price_per_kg(&self, m: MaterialClass) -> f64 {
        match m {
            MaterialClass::Steel => self.steel,
            MaterialClass::Timber => self.timber,
            MaterialClass::Concrete => self.concrete,
            MaterialClass::Composite => self.composite,
        }
    }

    /// Bill with every joint priced as bolted.
    pub fn estimate(&self, s: &Structure) -> Bill {
        self.estimate_with_joints(s, &[])
    }

    /// Bill with the joint kind of each member end, `[end a, end b]` per
    /// member; members past the end of `ends` are priced as bolted.
    pub fn estimate_with_joints(&self, s: &Structure, ends: &[[JointClass; 2]]) -> Bill {
        let mut bill = Bill::default();
        for (i, m) in s.members.iter().enumerate() {
            let mass = m.material.density * m.section.area * s.member_length(i);
            let class = MaterialClass::from(m.material.kind);
            bill.mass += mass;
            bill.materials += mass * self.price_per_kg(class);
            let [ja, jb] = ends.get(i).copied().unwrap_or_default();
            bill.joints += self.joints.price(ja) + self.joints.price(jb);
            match bill.by_material.iter_mut().find(|l| l.material == class) {
                Some(line) => {
                    line.mass += mass;
                    line.cost += mass * self.price_per_kg(class);
                    line.members += 1;
                }
                None => bill.by_material.push(BillLine {
                    material: class,
                    members: 1,
                    mass,
                    cost: mass * self.price_per_kg(class),
                }),
            }
        }
        bill
    }
}

/// Bill of materials.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Bill {
    /// kg.
    pub mass: f64,
    pub materials: f64,
    pub joints: f64,
    pub by_material: Vec<BillLine>,
}

impl Bill {
    pub fn total(&self) -> f64 {
        self.materials + self.joints
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct BillLine {
    pub material: MaterialClass,
    pub members: usize,
    pub mass: f64,
    pub cost: f64,
}
