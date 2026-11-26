//! Material properties.

/// A linear-elastic, perfectly-plastic material with a failure limit.
#[derive(Clone, Debug, PartialEq)]
pub struct Material {
    pub name: &'static str,
    /// Young's modulus E, Pa.
    pub youngs_modulus: f64,
    /// Shear modulus G, Pa.
    pub shear_modulus: f64,
    /// Density, kg/m^3.
    pub density: f64,
    /// Yield strength f_y, Pa. Members yield when an internal force reaches
    /// the section capacity computed from this value.
    pub yield_strength: f64,
    /// Axial plastic strain at which a member ruptures.
    pub rupture_strain: f64,
    /// Plastic hinge rotation (rad) at which a member snaps.
    pub hinge_rotation_capacity: f64,
    /// Brittle materials break at first yield instead of deforming plastically.
    pub brittle: bool,
}

impl Material {
    /// Structural steel, roughly S355.
    pub fn steel() -> Self {
        Self {
            name: "steel",
            youngs_modulus: 210e9,
            shear_modulus: 81e9,
            density: 7850.0,
            yield_strength: 355e6,
            rupture_strain: 0.15,
            hinge_rotation_capacity: 0.08,
            brittle: false,
        }
    }

    /// Softwood timber, roughly C24, treated as brittle.
    pub fn timber() -> Self {
        Self {
            name: "timber",
            youngs_modulus: 11e9,
            shear_modulus: 0.69e9,
            density: 420.0,
            yield_strength: 24e6,
            rupture_strain: 0.0,
            hinge_rotation_capacity: 0.0,
            brittle: true,
        }
    }

    /// Same material that never yields. Useful for purely elastic tests.
    pub fn elastic_only(mut self) -> Self {
        self.yield_strength = f64::INFINITY;
        self
    }
}
