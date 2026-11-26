//! Material properties and failure models.
//!
//! A [`Material`] carries the elastic constants every solver needs plus a
//! [`Model`] that decides how a member made of it yields, cracks and breaks.
//! The rules themselves live in [`crate::damage`].
//!
//! Strengths are characteristic (5th percentile) values from the usual design
//! codes, without partial safety factors: EN 1993 steel, EN 338 timber,
//! EN 1992 concrete and reinforcement, and typical manufacturer data for
//! fibre composites.

/// Broad material family. Renderers match on this to pick textures, debris
/// and failure effects.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum MaterialKind {
    Steel,
    Timber,
    Concrete,
    Composite,
}

/// Elastic, perfectly plastic metal with axial-bending interaction.
#[derive(Clone, Debug, PartialEq)]
pub struct DuctileProps {
    /// Yield strength f_y, Pa.
    pub yield_strength: f64,
    /// Axial plastic strain at which a member ruptures.
    pub rupture_strain: f64,
    /// Plastic hinge rotation (rad) at which a member snaps.
    pub hinge_rotation_capacity: f64,
}

/// Timber strengths along the member axis.
///
/// Built by [`Material::timber_at_grain_angle`], which applies Hankinson's
/// formula, so these are already the values for the member's grain angle.
#[derive(Clone, Debug, PartialEq)]
pub struct TimberProps {
    /// Tension strength, Pa. Brittle: the member splinters.
    pub tension: f64,
    /// Compression strength, Pa. Ductile: fibres crush and the member shortens.
    pub compression: f64,
    /// Bending strength, Pa, applied to the elastic section modulus.
    pub bending: f64,
    /// Shear strength, Pa. Brittle: the member splits along the grain.
    pub shear: f64,
    /// Compressive plastic strain at which crushed timber gives way.
    pub crush_strain_capacity: f64,
    /// Grain angle to the member axis, rad (0 = grain along the member).
    pub grain_angle: f64,
}

/// Steel bars cast into a concrete member.
///
/// The bars sit in the four corners of the (rectangular) section, so each
/// face holds half of the total area and the member is equally strong in
/// both bending directions and both senses, like a typical column.
#[derive(Clone, Debug, PartialEq)]
pub struct Rebar {
    /// Total steel area over gross concrete area, As / Ac (0.01 = 1 %).
    pub ratio: f64,
    /// Distance from each face to the bar centres, m.
    pub cover: f64,
    /// Yield strength, Pa.
    pub yield_strength: f64,
    /// Young's modulus, Pa.
    pub youngs_modulus: f64,
    /// Average plastic bar strain over the member at which the bars snap.
    /// Lower than the bar's own uniform elongation because strain
    /// concentrates at cracks.
    pub rupture_strain: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ConcreteProps {
    /// Cylinder compressive strength f_ck, Pa.
    pub compression: f64,
    /// Mean tensile strength f_ctm, Pa. Concrete cracks here.
    pub tension: f64,
    /// Ultimate compressive strain ε_cu (0.0035 in EN 1992).
    pub ultimate_strain: f64,
    /// Strain at peak stress under pure compression ε_c2 (0.002).
    pub peak_strain: f64,
    /// None for plain (unreinforced) concrete, which breaks when it cracks.
    pub rebar: Option<Rebar>,
}

/// Fibre-reinforced polymer: linear elastic until it fails suddenly.
#[derive(Clone, Debug, PartialEq)]
pub struct CompositeProps {
    /// Tensile strength along the member, Pa (fibre rupture).
    pub tension: f64,
    /// Compressive strength along the member, Pa (fibre kinking).
    pub compression: f64,
    /// Interlaminar shear strength, Pa. Above it the laminate delaminates.
    pub interlaminar_shear: f64,
    /// Matrix cracking (first-ply failure) starts at this fraction of the
    /// ultimate stress. 1.0 means none, as in unidirectional profiles.
    pub first_ply_failure: f64,
    /// Fraction of stiffness lost to matrix cracking by the time the member
    /// reaches its ultimate stress.
    pub matrix_crack_stiffness_loss: f64,
}

/// How a material yields, cracks and breaks.
#[derive(Clone, Debug, PartialEq)]
pub enum Model {
    /// Never yields or breaks.
    Elastic,
    Ductile(DuctileProps),
    Timber(TimberProps),
    Concrete(ConcreteProps),
    Composite(CompositeProps),
}

#[derive(Clone, Debug, PartialEq)]
pub struct Material {
    pub name: &'static str,
    pub kind: MaterialKind,
    /// Young's modulus E along the member, Pa.
    pub youngs_modulus: f64,
    /// Shear modulus G, Pa.
    pub shear_modulus: f64,
    /// Density, kg/m^3.
    pub density: f64,
    pub model: Model,
}

impl Material {
    /// Structural steel, roughly S355.
    pub fn steel() -> Self {
        Self {
            name: "steel",
            kind: MaterialKind::Steel,
            youngs_modulus: 210e9,
            shear_modulus: 81e9,
            density: 7850.0,
            model: Model::Ductile(DuctileProps {
                yield_strength: 355e6,
                rupture_strain: 0.15,
                hinge_rotation_capacity: 0.08,
            }),
        }
    }

    /// Softwood timber, strength class C24 (EN 338), grain along the member.
    pub fn timber() -> Self {
        Self::timber_at_grain_angle(0.0)
    }

    /// C24 timber whose grain runs at `angle` (rad) to the member axis.
    ///
    /// Timber is about 25 times stiffer and 35 times stronger in tension
    /// along the grain than across it. Off-axis properties follow Hankinson's
    /// formula f(θ) = f0 f90 / (f0 sin²θ + f90 cos²θ), so a member cut across
    /// the grain is far weaker and splits easily.
    pub fn timber_at_grain_angle(angle: f64) -> Self {
        // EN 338:2016 C24: f_m 24, f_t0 14.5, f_t90 0.4, f_c0 21, f_c90 2.5,
        // f_v 4.0 MPa; E0,mean 11.0, E90,mean 0.37, G_mean 0.69 GPa; ρ_mean 420.
        let h = |f0: f64, f90: f64| hankinson(f0, f90, angle);
        Self {
            name: "timber",
            kind: MaterialKind::Timber,
            youngs_modulus: h(11.0e9, 0.37e9),
            shear_modulus: 0.69e9,
            density: 420.0,
            model: Model::Timber(TimberProps {
                tension: h(14.5e6, 0.4e6),
                compression: h(21e6, 2.5e6),
                bending: h(24e6, 0.4e6),
                shear: 4.0e6,
                crush_strain_capacity: 0.02,
                grain_angle: angle,
            }),
        }
    }

    /// Plain (unreinforced) concrete C30/37. Cracks and breaks at the first
    /// tensile crack, so it only works in compression.
    pub fn concrete() -> Self {
        // EN 1992-1-1 table 3.1, C30/37: f_ck 30, f_ctm 2.9 MPa, E_cm 33 GPa;
        // ν = 0.2 so G = E / 2.4.
        Self {
            name: "concrete",
            kind: MaterialKind::Concrete,
            youngs_modulus: 33e9,
            shear_modulus: 33e9 / 2.4,
            density: 2400.0,
            model: Model::Concrete(ConcreteProps {
                compression: 30e6,
                tension: 2.9e6,
                ultimate_strain: 0.0035,
                peak_strain: 0.002,
                rebar: None,
            }),
        }
    }

    /// Reinforced concrete C30/37 with B500B bars: 1 % steel, 40 mm to the
    /// bar centres.
    pub fn reinforced_concrete() -> Self {
        Self::reinforced_concrete_with(0.01, 0.04)
    }

    /// Reinforced concrete with steel ratio `ratio` (As / Ac) and bar cover
    /// `cover` (m, face to bar centre).
    pub fn reinforced_concrete_with(ratio: f64, cover: f64) -> Self {
        let mut m = Self::concrete();
        m.name = "reinforced concrete";
        m.density = 2500.0;
        if let Model::Concrete(c) = &mut m.model {
            c.rebar = Some(Rebar {
                ratio,
                cover,
                yield_strength: 500e6,
                youngs_modulus: 200e9,
                rupture_strain: 0.025,
            });
        }
        m
    }

    /// Pultruded unidirectional carbon fibre (CFRP) profile, like a
    /// strengthening plate: very stiff and strong along the fibres, no
    /// warning before it fails.
    pub fn cfrp() -> Self {
        Self {
            name: "cfrp",
            kind: MaterialKind::Composite,
            youngs_modulus: 165e9,
            shear_modulus: 5e9,
            density: 1600.0,
            model: Model::Composite(CompositeProps {
                tension: 2800e6,
                compression: 1400e6,
                interlaminar_shear: 70e6,
                first_ply_failure: 1.0,
                matrix_crack_stiffness_loss: 0.0,
            }),
        }
    }

    /// Quasi-isotropic carbon fibre laminate, like a woven tube. Softer and
    /// weaker than a unidirectional profile, and matrix cracks lower its
    /// stiffness before the fibres break.
    pub fn cfrp_laminate() -> Self {
        Self {
            name: "cfrp laminate",
            kind: MaterialKind::Composite,
            youngs_modulus: 55e9,
            shear_modulus: 21e9,
            density: 1550.0,
            model: Model::Composite(CompositeProps {
                tension: 600e6,
                compression: 500e6,
                interlaminar_shear: 60e6,
                first_ply_failure: 0.5,
                matrix_crack_stiffness_loss: 0.2,
            }),
        }
    }

    /// Same material that never yields or breaks. Useful for elastic tests.
    pub fn elastic_only(mut self) -> Self {
        self.model = Model::Elastic;
        self
    }

    /// Concrete with steel bars.
    pub fn reinforced(&self) -> bool {
        matches!(&self.model, Model::Concrete(c) if c.rebar.is_some())
    }
}

/// Structural steel grades (EN 10025). Only the yield strength differs.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum SteelGrade {
    S235,
    S355,
    S460,
}

/// Timber strength classes: sawn softwood (EN 338) and glulam (EN 14080).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum TimberClass {
    C16,
    C24,
    GL28h,
}

/// Concrete strength classes (EN 1992-1-1 table 3.1).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum ConcreteClass {
    C25,
    C30,
    C40,
}

impl Material {
    /// Structural steel of the given grade.
    pub fn steel_grade(grade: SteelGrade) -> Self {
        let mut m = Self::steel();
        let (name, fy) = match grade {
            SteelGrade::S235 => ("steel S235", 235e6),
            SteelGrade::S355 => ("steel S355", 355e6),
            SteelGrade::S460 => ("steel S460", 460e6),
        };
        m.name = name;
        if let Model::Ductile(p) = &mut m.model {
            p.yield_strength = fy;
        }
        m
    }

    /// Timber of the given class with its grain at `angle` (rad) to the
    /// member axis (Hankinson's formula, as [`Material::timber_at_grain_angle`]).
    pub fn timber_class(class: TimberClass, angle: f64) -> Self {
        // [f_m, f_t0, f_t90, f_c0, f_c90, f_v] MPa, [E0, E90, G] GPa, ρ_mean.
        let (name, f, e, rho) = match class {
            TimberClass::C16 => (
                "timber C16",
                [16.0, 10.0, 0.4, 17.0, 2.2, 3.2],
                [8.0, 0.27, 0.5],
                370.0,
            ),
            TimberClass::C24 => (
                "timber C24",
                [24.0, 14.5, 0.4, 21.0, 2.5, 4.0],
                [11.0, 0.37, 0.69],
                420.0,
            ),
            TimberClass::GL28h => (
                "glulam GL28h",
                [28.0, 22.3, 0.5, 28.0, 2.5, 3.5],
                [12.6, 0.3, 0.65],
                460.0,
            ),
        };
        let h = |f0: f64, f90: f64| hankinson(f0, f90, angle);
        Self {
            name,
            kind: MaterialKind::Timber,
            youngs_modulus: h(e[0] * 1e9, e[1] * 1e9),
            shear_modulus: e[2] * 1e9,
            density: rho,
            model: Model::Timber(TimberProps {
                tension: h(f[1] * 1e6, f[2] * 1e6),
                compression: h(f[3] * 1e6, f[4] * 1e6),
                bending: h(f[0] * 1e6, f[2] * 1e6),
                shear: f[5] * 1e6,
                crush_strain_capacity: 0.02,
                grain_angle: angle,
            }),
        }
    }

    /// Plain concrete of the given class.
    pub fn concrete_class(class: ConcreteClass) -> Self {
        let (name, fck, fctm, ecm) = match class {
            ConcreteClass::C25 => ("concrete C25/30", 25e6, 2.6e6, 31e9),
            ConcreteClass::C30 => ("concrete C30/37", 30e6, 2.9e6, 33e9),
            ConcreteClass::C40 => ("concrete C40/50", 40e6, 3.5e6, 35e9),
        };
        let mut m = Self::concrete();
        m.name = name;
        m.youngs_modulus = ecm;
        m.shear_modulus = ecm / 2.4;
        if let Model::Concrete(c) = &mut m.model {
            c.compression = fck;
            c.tension = fctm;
        }
        m
    }

    /// Reinforced concrete of the given class with B500B bars at steel
    /// ratio `ratio` and cover `cover` (m, face to bar centre).
    pub fn reinforced_concrete_class(class: ConcreteClass, ratio: f64, cover: f64) -> Self {
        let rc = Self::reinforced_concrete_with(ratio, cover);
        let mut m = Self::concrete_class(class);
        m.name = match class {
            ConcreteClass::C25 => "reinforced concrete C25/30",
            ConcreteClass::C30 => "reinforced concrete C30/37",
            ConcreteClass::C40 => "reinforced concrete C40/50",
        };
        m.density = rc.density;
        if let (Model::Concrete(c), Model::Concrete(r)) = (&mut m.model, rc.model) {
            c.rebar = r.rebar;
        }
        m
    }
}

/// Hankinson's formula for a property at `angle` to the grain.
pub fn hankinson(along: f64, across: f64, angle: f64) -> f64 {
    let (s, c) = angle.sin_cos();
    along * across / (along * s * s + across * c * c)
}
