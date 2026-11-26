//! Building parts for BreakIt, engine-free.
//!
//! The player picks members at one of three depths ([`Layer`]), and every
//! depth ends in the same [`MemberChoice`]: a material, a cross-section and
//! a joint at each end. [`MemberChoice::resolve`] turns that into the
//! `breakit-core` material, section and joints the solvers use, and the
//! `breakit-mesh` profile the renderer draws, so what the player picked,
//! what is drawn and what is computed are one description.
//!
//! * **Catalog**: ready-made [`Part`]s ("Steel I-beam, medium").
//! * **Engineer**: a [`Family`] of real standard sizes (IPE, HEA, CHS, sawn
//!   timber, glulam, RC), a grade from the family, and a joint per end.
//! * **Sandbox**: any dimensions, checked against [`Limits`] so a section
//!   stays inside what the beam model is valid for.
//!
//! The data lives in `data/parts.ron` ([`Catalog::standard`]); prices are
//! per kg of material plus a cost per member end by joint type.
//!
//! Dimensions in the data are millimetres; everything returned is SI.

pub mod node;

use std::collections::BTreeMap;

use breakit_core::{
    ConcreteClass, Joint, JointKind, Material, MaterialKind, Section, SteelGrade, TimberClass,
};
use breakit_mesh::Profile;
use serde::{Deserialize, Serialize};

/// How deep the player's control goes. Ordered: a scenario that allows
/// `Engineer` also allows `Catalog`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub enum Layer {
    Catalog,
    Engineer,
    Sandbox,
}

/// What a member is for. The solver treats all of them as beams; the role
/// sets sensible defaults and lets the palette group parts.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum Role {
    Beam,
    Column,
    Brace,
    /// Slender bar meant for tension. Still carries compression until real
    /// tension-only cables exist.
    Tie,
}

/// A material and grade, as stored in data and designs.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum MaterialSpec {
    Steel(SteelSpec),
    /// Timber class and grain angle to the member axis, degrees.
    Timber(TimberSpec, f64),
    Concrete(ConcreteSpec),
    /// Reinforced concrete: class, steel ratio As/Ac, cover to bar centres (mm).
    ReinforcedConcrete(ConcreteSpec, f64, f64),
    Cfrp,
    CfrpLaminate,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum SteelSpec {
    S235,
    S355,
    S460,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum TimberSpec {
    C16,
    C24,
    GL28h,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum ConcreteSpec {
    C25,
    C30,
    C40,
}

impl MaterialSpec {
    pub fn material(&self) -> Material {
        match *self {
            MaterialSpec::Steel(g) => Material::steel_grade(match g {
                SteelSpec::S235 => SteelGrade::S235,
                SteelSpec::S355 => SteelGrade::S355,
                SteelSpec::S460 => SteelGrade::S460,
            }),
            MaterialSpec::Timber(c, deg) => Material::timber_class(timber(c), deg.to_radians()),
            MaterialSpec::Concrete(c) => Material::concrete_class(concrete(c)),
            MaterialSpec::ReinforcedConcrete(c, ratio, cover) => {
                Material::reinforced_concrete_class(concrete(c), ratio, cover / 1000.0)
            }
            MaterialSpec::Cfrp => Material::cfrp(),
            MaterialSpec::CfrpLaminate => Material::cfrp_laminate(),
        }
    }

    pub fn kind(&self) -> MaterialKind {
        match self {
            MaterialSpec::Steel(_) => MaterialKind::Steel,
            MaterialSpec::Timber(..) => MaterialKind::Timber,
            MaterialSpec::Concrete(_) | MaterialSpec::ReinforcedConcrete(..) => {
                MaterialKind::Concrete
            }
            MaterialSpec::Cfrp | MaterialSpec::CfrpLaminate => MaterialKind::Composite,
        }
    }

    /// Grade or class as printed on a drawing: "S355", "C24", "GL28h",
    /// "C30/37", "CFRP".
    pub fn grade_label(&self) -> &'static str {
        let key = self.price_key();
        match self {
            MaterialSpec::Cfrp => "CFRP",
            MaterialSpec::CfrpLaminate => "CFRP lam.",
            _ => key.rsplit(' ').next().unwrap_or(key),
        }
    }

    /// The key prices are looked up by: material and grade, not grain
    /// angle, rebar ratio or cover.
    pub fn price_key(&self) -> &'static str {
        match self {
            MaterialSpec::Steel(SteelSpec::S235) => "steel S235",
            MaterialSpec::Steel(SteelSpec::S355) => "steel S355",
            MaterialSpec::Steel(SteelSpec::S460) => "steel S460",
            MaterialSpec::Timber(TimberSpec::C16, _) => "timber C16",
            MaterialSpec::Timber(TimberSpec::C24, _) => "timber C24",
            MaterialSpec::Timber(TimberSpec::GL28h, _) => "glulam GL28h",
            MaterialSpec::Concrete(ConcreteSpec::C25) => "concrete C25/30",
            MaterialSpec::Concrete(ConcreteSpec::C30) => "concrete C30/37",
            MaterialSpec::Concrete(ConcreteSpec::C40) => "concrete C40/50",
            MaterialSpec::ReinforcedConcrete(ConcreteSpec::C25, ..) => "RC C25/30",
            MaterialSpec::ReinforcedConcrete(ConcreteSpec::C30, ..) => "RC C30/37",
            MaterialSpec::ReinforcedConcrete(ConcreteSpec::C40, ..) => "RC C40/50",
            MaterialSpec::Cfrp => "CFRP",
            MaterialSpec::CfrpLaminate => "CFRP laminate",
        }
    }
}

fn timber(c: TimberSpec) -> TimberClass {
    match c {
        TimberSpec::C16 => TimberClass::C16,
        TimberSpec::C24 => TimberClass::C24,
        TimberSpec::GL28h => TimberClass::GL28h,
    }
}

fn concrete(c: ConcreteSpec) -> ConcreteClass {
    match c {
        ConcreteSpec::C25 => ConcreteClass::C25,
        ConcreteSpec::C30 => ConcreteClass::C30,
        ConcreteSpec::C40 => ConcreteClass::C40,
    }
}

/// A cross-section in millimetres, as stored in data and designs.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum ProfileSpec {
    /// Solid rectangle, `b` wide, `h` deep.
    Rect { b: f64, h: f64 },
    /// Solid round bar of diameter `d`.
    Round { d: f64 },
    /// Circular hollow section: outside diameter `d`, wall `t`.
    Chs { d: f64, t: f64 },
    /// Rectangular hollow section `b` x `h`, wall `t`.
    Rhs { b: f64, h: f64, t: f64 },
    /// I or H section: depth `h`, flange width `b`, web `tw`, flange `tf`.
    I { h: f64, b: f64, tw: f64, tf: f64 },
}

impl ProfileSpec {
    /// The renderer's profile, in metres.
    pub fn profile(&self) -> Profile {
        let m = |x: f64| (x / 1000.0) as f32;
        match *self {
            ProfileSpec::Rect { b, h } => Profile::Rect { b: m(b), h: m(h) },
            ProfileSpec::Round { d } => Profile::Round { r: m(d / 2.0) },
            ProfileSpec::Chs { d, t } => Profile::Tube {
                r: m(d / 2.0),
                t: m(t),
            },
            ProfileSpec::Rhs { b, h, t } => Profile::BoxSection {
                b: m(b),
                h: m(h),
                t: m(t),
            },
            ProfileSpec::I { h, b, tw, tf } => Profile::IBeam {
                h: m(h),
                b: m(b),
                tw: m(tw),
                tf: m(tf),
            },
        }
    }

    /// Section properties for the solver.
    pub fn section(&self) -> Section {
        self.profile().section()
    }

    /// Dimensions in millimetres, e.g. "CHS 114.3x5" or "I 200x100".
    pub fn designation(&self) -> String {
        let n = |x: f64| {
            let s = format!("{x:.1}");
            s.trim_end_matches(".0").to_string()
        };
        match *self {
            ProfileSpec::Rect { b, h } => format!("{}x{}", n(b), n(h)),
            ProfileSpec::Round { d } => format!("Ø{}", n(d)),
            ProfileSpec::Chs { d, t } => format!("CHS {}x{}", n(d), n(t)),
            ProfileSpec::Rhs { b, h, t } => format!("RHS {}x{}x{}", n(h), n(b), n(t)),
            ProfileSpec::I { h, b, .. } => format!("I {}x{}", n(h), n(b)),
        }
    }
}

/// The joint types as stored in data and designs; see [`JointKind`].
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub enum JointSpec {
    Rigid,
    Pinned,
    Bolted,
    Welded,
    Monolithic,
    Dowelled,
}

impl JointSpec {
    pub const ALL: [JointSpec; 6] = [
        JointSpec::Pinned,
        JointSpec::Dowelled,
        JointSpec::Bolted,
        JointSpec::Monolithic,
        JointSpec::Welded,
        JointSpec::Rigid,
    ];

    /// Lower-case name for the UI.
    pub fn label(self) -> &'static str {
        match self {
            JointSpec::Rigid => "rigid",
            JointSpec::Pinned => "pinned",
            JointSpec::Bolted => "bolted",
            JointSpec::Welded => "welded",
            JointSpec::Monolithic => "monolithic",
            JointSpec::Dowelled => "dowelled",
        }
    }

    pub fn kind(self) -> JointKind {
        match self {
            JointSpec::Rigid => JointKind::Rigid,
            JointSpec::Pinned => JointKind::Pinned,
            JointSpec::Bolted => JointKind::Bolted,
            JointSpec::Welded => JointKind::Welded,
            JointSpec::Monolithic => JointKind::Monolithic,
            JointSpec::Dowelled => JointKind::Dowelled,
        }
    }
}

/// Everything the game stores about one member, whichever layer it came from.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct MemberChoice {
    pub material: MaterialSpec,
    pub profile: ProfileSpec,
    /// Joints at ends a and b.
    pub joints: [JointSpec; 2],
}

/// A member ready for `breakit-core` and `breakit-mesh`.
#[derive(Clone, Debug)]
pub struct Resolved {
    pub material: Material,
    pub section: Section,
    pub profile: Profile,
    pub joints: [Joint; 2],
}

impl MemberChoice {
    /// Core material, section and joints for a member `length` metres long
    /// (joint stiffness is relative to the member's EI/L).
    pub fn resolve(&self, length: f64) -> Resolved {
        let material = self.material.material();
        let section = self.profile.section();
        let joints = self
            .joints
            .map(|j| j.kind().joint(&material, &section, length));
        Resolved {
            profile: self.profile.profile(),
            material,
            section,
            joints,
        }
    }

    /// Mass of a member `length` metres long, kg.
    pub fn mass(&self, length: f64) -> f64 {
        self.material.material().density * self.profile.section().area * length
    }

    /// True if both joints can be made in this material.
    pub fn joints_suit_material(&self) -> bool {
        let kind = self.material.kind();
        self.joints.iter().all(|j| j.kind().suits(kind))
    }
}

/// The joint a member end gets when the player leaves it on automatic:
/// what a builder would use for this material where it meets `others`, the
/// materials of the other members at the same node. Timber meeting timber
/// is dowelled, concrete meeting concrete is cast in one (monolithic),
/// steel and mixed materials are bolted (the usual site connection; welding
/// is stiffer but costs twice as much, so it is the player's call), concrete
/// sits on a pinned bearing where it meets anything else, and solid round
/// bars are pinned ties. The result is always one of `allowed` (the first one if the rule's
/// pick is not), so a family's joint list has the last word.
pub fn auto_joint(c: &MemberChoice, others: &[MaterialKind], allowed: &[JointSpec]) -> JointSpec {
    let kind = c.material.kind();
    let same = others.iter().all(|k| *k == kind);
    let pick = match (kind, c.profile) {
        (_, ProfileSpec::Round { .. }) => JointSpec::Pinned,
        (MaterialKind::Timber, _) if same => JointSpec::Dowelled,
        (MaterialKind::Concrete, _) if same => JointSpec::Monolithic,
        // Concrete can't be bolted to other materials in this model: a
        // pinned bearing.
        (MaterialKind::Concrete, _) => JointSpec::Pinned,
        _ => JointSpec::Bolted,
    };
    if allowed.is_empty() || allowed.contains(&pick) {
        pick
    } else {
        allowed[0]
    }
}

/// A ready-made part in the Catalog layer.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Part {
    /// Stable id used in saves and scenario files.
    pub id: String,
    pub name: String,
    pub role: Role,
    pub choice: MemberChoice,
}

/// One standard size in a [`Family`].
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Size {
    /// Designation, e.g. "IPE 200" or "45x220".
    pub name: String,
    pub profile: ProfileSpec,
}

/// A family of standard sizes for the Engineer layer.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Family {
    pub id: String,
    pub name: String,
    /// Grades the player can pick; the first is the default.
    pub grades: Vec<MaterialSpec>,
    /// Sizes from small to large.
    pub sizes: Vec<Size>,
    /// Joint types the player can pick; the first is the default.
    pub joints: Vec<JointSpec>,
}

impl Family {
    /// The member for size `size` and grade `grade` (indices into the lists),
    /// with the default joint at both ends.
    pub fn choice(&self, size: usize, grade: usize) -> MemberChoice {
        MemberChoice {
            material: self.grades[grade],
            profile: self.sizes[size].profile,
            joints: [self.joints[0]; 2],
        }
    }
}

/// What makes a custom (Sandbox) member fall outside the beam model, or
/// outside what can be built.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Limits {
    /// Thinnest wall, flange, web or bar, mm.
    pub min_thickness: f64,
    /// Largest outside dimension, mm.
    pub max_dimension: f64,
    /// Width or diameter over wall thickness of a hollow section. Thin walls
    /// buckle locally, which the beam model does not see.
    pub max_wall_ratio: f64,
    /// Flange outstand over thickness, b / (2 tf), of an I section.
    pub max_flange_ratio: f64,
    /// Web depth over thickness of an I section.
    pub max_web_ratio: f64,
    /// Member length over the section's least radius of gyration. Not
    /// applied to solid round bars, which are ties.
    pub max_slenderness: f64,
}

/// Why a member choice was refused.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum LimitError {
    TooThin {
        thickness: f64,
        min: f64,
    },
    TooLarge {
        dimension: f64,
        max: f64,
    },
    WallTooThin {
        ratio: f64,
        max: f64,
    },
    FlangeTooThin {
        ratio: f64,
        max: f64,
    },
    WebTooThin {
        ratio: f64,
        max: f64,
    },
    TooSlender {
        slenderness: f64,
        max: f64,
    },
    /// Hollow wall at least half the size, or another impossible shape.
    BadShape,
    /// A joint that cannot be made in this material (welded timber).
    JointUnsuitable(JointSpec),
}

impl std::fmt::Display for LimitError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            LimitError::TooThin { thickness, min } => {
                write!(
                    f,
                    "{thickness:.1} mm is thinner than the {min:.1} mm minimum"
                )
            }
            LimitError::TooLarge { dimension, max } => {
                write!(
                    f,
                    "{dimension:.0} mm is larger than the {max:.0} mm maximum"
                )
            }
            LimitError::WallTooThin { ratio, max } => {
                write!(f, "wall too thin for its size ({ratio:.0} > {max:.0})")
            }
            LimitError::FlangeTooThin { ratio, max } => {
                write!(f, "flange too thin for its width ({ratio:.0} > {max:.0})")
            }
            LimitError::WebTooThin { ratio, max } => {
                write!(f, "web too thin for its depth ({ratio:.0} > {max:.0})")
            }
            LimitError::TooSlender { slenderness, max } => {
                write!(
                    f,
                    "too slender for its length ({slenderness:.0} > {max:.0})"
                )
            }
            LimitError::BadShape => write!(f, "impossible section shape"),
            LimitError::JointUnsuitable(j) => write!(f, "{j:?} joints do not suit this material"),
        }
    }
}

impl std::error::Error for LimitError {}

impl Limits {
    /// Check a member `length` metres long.
    pub fn check(&self, c: &MemberChoice, length: f64) -> Result<(), LimitError> {
        if let Some(j) = c.joints.iter().find(|j| !j.kind().suits(c.material.kind())) {
            return Err(LimitError::JointUnsuitable(*j));
        }
        // The concrete model places bars in the corners of a solid rectangle.
        if c.material.kind() == MaterialKind::Concrete
            && !matches!(c.profile, ProfileSpec::Rect { .. })
        {
            return Err(LimitError::BadShape);
        }
        let (dims, thick): (Vec<f64>, Vec<f64>) = match c.profile {
            ProfileSpec::Rect { b, h } => (vec![b, h], vec![b, h]),
            ProfileSpec::Round { d } => (vec![d], vec![d]),
            ProfileSpec::Chs { d, t } => (vec![d], vec![t]),
            ProfileSpec::Rhs { b, h, t } => (vec![b, h], vec![t]),
            ProfileSpec::I { h, b, tw, tf } => (vec![h, b], vec![tw, tf]),
        };
        if dims
            .iter()
            .chain(&thick)
            .any(|v| !(v.is_finite() && *v > 0.0))
        {
            return Err(LimitError::BadShape);
        }
        if let Some(&t) = thick.iter().find(|&&t| t < self.min_thickness) {
            return Err(LimitError::TooThin {
                thickness: t,
                min: self.min_thickness,
            });
        }
        if let Some(&d) = dims.iter().find(|&&d| d > self.max_dimension) {
            return Err(LimitError::TooLarge {
                dimension: d,
                max: self.max_dimension,
            });
        }
        let ratio_check = |ratio: f64, max: f64, e: fn(f64, f64) -> LimitError| {
            if ratio > max {
                Err(e(ratio, max))
            } else {
                Ok(())
            }
        };
        let wall = |ratio, max| LimitError::WallTooThin { ratio, max };
        match c.profile {
            ProfileSpec::Chs { d, t } => {
                if 2.0 * t >= d {
                    return Err(LimitError::BadShape);
                }
                ratio_check(d / t, self.max_wall_ratio, wall)?;
            }
            ProfileSpec::Rhs { b, h, t } => {
                if 2.0 * t >= b.min(h) {
                    return Err(LimitError::BadShape);
                }
                ratio_check(b.max(h) / t, self.max_wall_ratio, wall)?;
            }
            ProfileSpec::I { h, b, tw, tf } => {
                if 2.0 * tf >= h || tw >= b {
                    return Err(LimitError::BadShape);
                }
                ratio_check(b / (2.0 * tf), self.max_flange_ratio, |ratio, max| {
                    LimitError::FlangeTooThin { ratio, max }
                })?;
                ratio_check((h - 2.0 * tf) / tw, self.max_web_ratio, |ratio, max| {
                    LimitError::WebTooThin { ratio, max }
                })?;
            }
            _ => {}
        }
        if let ProfileSpec::Round { .. } = c.profile {
            // Round bars are ties and rods; slender is what they are for.
            return Ok(());
        }
        let s = c.profile.section();
        let r = (s.iy.min(s.iz) / s.area).sqrt();
        ratio_check(length / r, self.max_slenderness, |slenderness, max| {
            LimitError::TooSlender { slenderness, max }
        })
    }
}

/// Prices in game currency.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Prices {
    /// Per kg of material, by [`MaterialSpec::price_key`].
    pub per_kg: BTreeMap<String, f64>,
    /// Per member end, by joint type.
    pub joint: BTreeMap<JointSpec, f64>,
}

impl Prices {
    /// Cost of a member `length` metres long, joints included. Unpriced
    /// materials or joints cost nothing; [`Catalog::validate`] catches them.
    pub fn member(&self, c: &MemberChoice, length: f64) -> f64 {
        let per_kg = self
            .per_kg
            .get(c.material.price_key())
            .copied()
            .unwrap_or(0.0);
        let joints: f64 = c
            .joints
            .iter()
            .map(|j| self.joint.get(j).copied().unwrap_or(0.0))
            .sum();
        per_kg * c.mass(length) + joints
    }
}

/// All parts, families, prices and limits.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Catalog {
    pub parts: Vec<Part>,
    pub families: Vec<Family>,
    pub prices: Prices,
    pub limits: Limits,
}

/// What is wrong with a catalog file.
#[derive(Debug)]
pub enum CatalogError {
    Parse(ron::error::SpannedError),
    Invalid(String),
}

impl std::fmt::Display for CatalogError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            CatalogError::Parse(e) => write!(f, "catalog does not parse: {e}"),
            CatalogError::Invalid(s) => write!(f, "invalid catalog: {s}"),
        }
    }
}

impl std::error::Error for CatalogError {}

impl Catalog {
    /// The catalog shipped with the game (`data/parts.ron`).
    pub fn standard() -> Catalog {
        Catalog::from_ron(include_str!("../data/parts.ron")).expect("data/parts.ron is valid")
    }

    /// Parse and validate a catalog.
    pub fn from_ron(text: &str) -> Result<Catalog, CatalogError> {
        let c: Catalog = ron::from_str(text).map_err(CatalogError::Parse)?;
        c.validate().map_err(CatalogError::Invalid)?;
        Ok(c)
    }

    pub fn part(&self, id: &str) -> Option<&Part> {
        self.parts.iter().find(|p| p.id == id)
    }

    pub fn family(&self, id: &str) -> Option<&Family> {
        self.families.iter().find(|f| f.id == id)
    }

    /// The family, size index and grade index a member's section and
    /// material come from, if they are standard. Joints are not compared.
    pub fn family_of(&self, c: &MemberChoice) -> Option<(&Family, usize, usize)> {
        self.families.iter().find_map(|f| {
            let grade = f.grades.iter().position(|g| *g == c.material)?;
            let size = f.sizes.iter().position(|s| s.profile == c.profile)?;
            Some((f, size, grade))
        })
    }

    /// The catalog part with this material and section, if any. Joints are
    /// not compared.
    pub fn part_of(&self, c: &MemberChoice) -> Option<&Part> {
        self.parts
            .iter()
            .find(|p| p.choice.material == c.material && p.choice.profile == c.profile)
    }

    /// Short designation: the standard size and grade ("IPE 200 S355"), else
    /// the section in millimetres and the grade.
    pub fn designation(&self, c: &MemberChoice) -> String {
        let size = match self.family_of(c) {
            Some((f, s, _)) => f.sizes[s].name.clone(),
            None => c.profile.designation(),
        };
        format!("{size} {}", c.material.grade_label())
    }

    /// Joint types a member of this material can have: the family's when the
    /// member is standard, else every type that suits the material.
    pub fn joints_for(&self, c: &MemberChoice) -> Vec<JointSpec> {
        match self.family_of(c) {
            Some((f, ..)) => f.joints.clone(),
            None => JointSpec::ALL
                .into_iter()
                .filter(|j| *j != JointSpec::Rigid && j.kind().suits(c.material.kind()))
                .collect(),
        }
    }

    /// Check what loading cannot: unique ids, prices for every material and
    /// joint used, joints that suit their material, non-empty families, and
    /// every standard section inside the sandbox limits at 1 m.
    pub fn validate(&self) -> Result<(), String> {
        let mut ids = std::collections::BTreeSet::new();
        for id in self
            .parts
            .iter()
            .map(|p| &p.id)
            .chain(self.families.iter().map(|f| &f.id))
        {
            if !ids.insert(id) {
                return Err(format!("duplicate id {id}"));
            }
        }
        let mut choices: Vec<(String, MemberChoice)> = self
            .parts
            .iter()
            .map(|p| (p.id.clone(), p.choice))
            .collect();
        for f in &self.families {
            if f.grades.is_empty() || f.sizes.is_empty() || f.joints.is_empty() {
                return Err(format!("family {} needs grades, sizes and joints", f.id));
            }
            for (gi, g) in f.grades.iter().enumerate() {
                for (si, s) in f.sizes.iter().enumerate() {
                    for j in &f.joints {
                        let mut c = f.choice(si, gi);
                        c.joints = [*j; 2];
                        choices.push((format!("{} {} {:?}", f.id, s.name, g), c));
                    }
                }
            }
        }
        for (id, c) in &choices {
            if !self.prices.per_kg.contains_key(c.material.price_key()) {
                return Err(format!("{id}: no price for {}", c.material.price_key()));
            }
            for j in &c.joints {
                if !self.prices.joint.contains_key(j) {
                    return Err(format!("{id}: no price for {j:?} joints"));
                }
            }
            self.limits
                .check(c, 1.0)
                .map_err(|e| format!("{id}: {e}"))?;
        }
        Ok(())
    }
}
