//! Per-member damage and failure, by material.
//!
//! The dynamic solver does the plastic return of every member itself (inside
//! the substep solve, with the tangent stiffness) against yield caps the
//! material model sets: an axial yield force per sense and a plastic moment
//! per bending axis. After each substep the model reads the committed forces
//! and plastic increments, books damage, checks for failure and sets the
//! stiffness and caps for the next substep. Brittle limits (cracking,
//! splintering, fibre rupture, shear) are checks, not caps.
//!
//! What each material does:
//!
//! * **Steel**: elastic, perfectly plastic. Axial yield with M-N interaction
//!   Mp,N = Mp (1 − (N/Ny)²), plastic hinges at the member ends, rupture past
//!   the plastic strain capacity, hinge snap past the rotation capacity.
//! * **Timber**: brittle in tension, bending and shear, ductile in
//!   compression. EN 1995 interaction: σt/ft + σm/fm ≤ 1 and
//!   (σc/fc)² + σm/fm ≤ 1. Tension and bending failures splinter, shear
//!   (τ = 1.5 V/A over f_v) splits the member along the grain, compression
//!   crushes it gradually.
//! * **Concrete**: see [`crate::concrete`] for the section analysis. Tension
//!   cracks the section; plain concrete breaks there, reinforced concrete
//!   carries on with only its bars in tension. Flexural cracks lower the
//!   bending stiffness by Branson's effective inertia,
//!   Ie = (Mcr/Ma)³ Ig + (1 − (Mcr/Ma)³) Icr. Bars yield into plastic hinges
//!   capped by the M-N interaction curve, and the hinge crushes the
//!   compression zone at its rotation capacity. Axial compression crushes at
//!   the squash load. Shear past V_Rc breaks the member.
//! * **Composite**: linear elastic until fibres break in tension or kink in
//!   compression. Interlaminar shear past the ILSS delaminates the section
//!   into two halves (a quarter of the bending stiffness for a rectangle,
//!   half the section modulus), which usually fails soon after. Laminates
//!   lose some stiffness to matrix cracking above first-ply failure.

// Index loops over [end] and [axis] read closer to the formulas here.
#![allow(clippy::needless_range_loop)]

use crate::concrete::RcSection;
use crate::material::{MaterialKind, Model};
use crate::section::Shape;
use crate::{Material, Section};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum BreakCause {
    /// Steel: axial plastic strain passed the rupture strain.
    Rupture,
    /// Steel: a plastic hinge rotated past its capacity.
    Hinge,
    /// A brittle member reached its strength: plain concrete cracking.
    Brittle,
    /// Timber: tension or bending failure, the member splinters.
    Splinter,
    /// Timber: shear failure, the member splits along the grain.
    TimberSplit,
    /// Concrete crushed (axial squash or at a plastic hinge), or timber
    /// crushed past its compressive strain capacity.
    Crushing,
    /// Concrete: diagonal shear failure.
    ConcreteShear,
    /// Reinforced concrete: the bars snapped in tension.
    RebarRupture,
    /// Composite: fibres broke in tension or kinked in compression.
    FibreRupture,
    /// Composite: interlaminar shear failure.
    Delamination,
    /// A joint failed: the pin or bolts sheared, or the weld or connection
    /// ran out of rotation. `BreakEvent.position` says which end (0 or 1).
    Connection,
}

/// Worst condition a member has reached. Ordered, and never goes back.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum DamageState {
    #[default]
    Intact,
    /// Concrete cracks, timber fibre cracking near failure, composite matrix
    /// cracking.
    Cracked,
    /// Permanent deformation: steel yield or plastic hinges, yielded rebar.
    Yielded,
    /// A composite member split into layers.
    Delaminated,
    /// Concrete crushing or spalling, timber crushed along the grain.
    Crushed,
    Broken,
}

/// What the renderer needs to draw one member.
///
/// End-indexed arrays are `[end a, end b]`. Axis-indexed arrays are
/// `[bending about local y, bending about local z]`, with local axes from
/// [`crate::frame::member_axes`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MemberDamage {
    pub kind: MaterialKind,
    pub state: DamageState,
    /// Fraction of the way to failure, 0..1, never decreases. Brittle
    /// members (timber, plain concrete, composites) report their peak
    /// utilisation. Ductile members (steel, reinforced concrete) use 0..0.5
    /// up to first yield and 0.5..1 for the plastic deformation they have
    /// used up.
    pub damage: f64,
    /// Current load over capacity, 1.0 = at capacity.
    pub utilization: f64,
    /// Signed plastic hinge rotation, rad, `[axis][end]`.
    pub hinge: [[f64; 2]; 2],
    /// Axial plastic strain (steel yield, timber crushing, rebar yield).
    pub plastic_strain: f64,
    /// Flexural crack damage per end, 0..1 (concrete cracks up to bar yield;
    /// timber fibre cracking; composite matrix cracking).
    pub crack: [f64; 2],
    /// Through-cracking along the member from axial tension, 0..1.
    pub tension_crack: f64,
    /// Compression crushing per end, 0..1 (1 = crushed through).
    pub crush: [f64; 2],
    /// Composites: 0.5 per delaminated bending plane.
    pub delamination: f64,
    /// Current end moments, N·m, `[axis][end]`, solver sign convention:
    /// uniform bending gives opposite signs at the two ends.
    pub moment: [[f64; 2]; 2],
    /// Current axial force, N, tension positive.
    pub axial: f64,
    /// Remaining bending stiffness over the intact value, 0..1 (lowest axis).
    pub stiffness: f64,
    /// Peak utilisation of the connections at ends a and b, 0..1 (force,
    /// moment or plastic rotation over capacity). 0 for rigid joints.
    pub joint: [f64; 2],
}

impl MemberDamage {
    pub fn new(kind: MaterialKind) -> Self {
        Self {
            kind,
            state: DamageState::Intact,
            damage: 0.0,
            utilization: 0.0,
            hinge: [[0.0; 2]; 2],
            plastic_strain: 0.0,
            crack: [0.0; 2],
            tension_crack: 0.0,
            crush: [0.0; 2],
            delamination: 0.0,
            moment: [[0.0; 2]; 2],
            axial: 0.0,
            stiffness: 1.0,
            joint: [0.0; 2],
        }
    }
}

/// Capacities precomputed from a member's material and section.
#[derive(Clone, Debug)]
pub(crate) enum Law {
    Elastic,
    Ductile {
        ny: f64,
        mp: [f64; 2],
        rupture_strain: f64,
        hinge_capacity: f64,
    },
    Timber {
        area: f64,
        nt: f64,
        nc: f64,
        w: [f64; 2],
        fm: f64,
        fv: f64,
        shear_factor: f64,
        crush_capacity: f64,
    },
    /// Shared between members with the same material and section.
    Concrete(std::sync::Arc<ConcreteLaw>),
    Composite {
        area: f64,
        w: [f64; 2],
        xt: f64,
        xc: f64,
        ilss: f64,
        shear_factor: f64,
        first_ply: f64,
        loss: f64,
        /// Bending stiffness left after delamination.
        delam_stiffness: f64,
    },
}

#[derive(Clone, Debug)]
pub(crate) struct ConcreteLaw {
    rc: RcSection,
    plain: bool,
    /// E_s A_s, N per unit strain.
    es_as: f64,
    crush_capacity: f64,
    rebar_rupture: f64,
    ft: f64,
    w: [f64; 2],
}

impl Law {
    pub fn new(mat: &Material, sec: &Section) -> Self {
        match &mat.model {
            Model::Elastic => Law::Elastic,
            Model::Ductile(p) => Law::Ductile {
                ny: p.yield_strength * sec.area,
                mp: [p.yield_strength * sec.zy, p.yield_strength * sec.zz],
                rupture_strain: p.rupture_strain,
                hinge_capacity: p.hinge_rotation_capacity,
            },
            Model::Timber(t) => Law::Timber {
                area: sec.area,
                nt: t.tension * sec.area,
                nc: t.compression * sec.area,
                w: [sec.wy, sec.wz],
                fm: t.bending,
                fv: t.shear,
                shear_factor: sec.shear_factor(),
                crush_capacity: t.crush_strain_capacity,
            },
            Model::Concrete(c) => {
                let rc = RcSection::new(sec, mat.youngs_modulus, c);
                let es = c.rebar.as_ref().map_or(200e9, |r| r.youngs_modulus);
                Law::Concrete(std::sync::Arc::new(ConcreteLaw {
                    plain: c.rebar.is_none(),
                    es_as: es * rc.steel_area,
                    crush_capacity: c.ultimate_strain - c.peak_strain,
                    rebar_rupture: c.rebar.as_ref().map_or(0.0, |r| r.rupture_strain),
                    ft: c.tension,
                    w: [sec.wy, sec.wz],
                    rc,
                }))
            }
            Model::Composite(c) => Law::Composite {
                area: sec.area,
                w: [sec.wy, sec.wz],
                xt: c.tension,
                xc: c.compression,
                ilss: c.interlaminar_shear,
                shear_factor: sec.shear_factor(),
                first_ply: c.first_ply_failure,
                loss: c.matrix_crack_stiffness_loss,
                delam_stiffness: match sec.shape {
                    Shape::Rect { .. } => 0.25,
                    Shape::Tube { .. } => 0.5,
                },
            },
        }
    }

    /// Utilisation of an intact member under axial force `n` (tension
    /// positive), end moments `m[axis][end]` and shears `v[axis]`. Used by the
    /// static solve for the build-mode heatmap.
    pub fn intact_utilization(&self, n: f64, m: [[f64; 2]; 2], v: [f64; 2]) -> f64 {
        self.utilization(&MatState::default(), n, m, v)
    }

    fn utilization(&self, st: &MatState, n: f64, m: [[f64; 2]; 2], v: [f64; 2]) -> f64 {
        let mmax = |a: usize| m[a][0].abs().max(m[a][1].abs());
        match self {
            Law::Elastic => 0.0,
            Law::Ductile { ny, mp, .. } => {
                let r = (n.abs() / ny).min(1.0);
                r * r + mmax(0) / mp[0] + mmax(1) / mp[1]
            }
            Law::Timber {
                area,
                nt,
                nc,
                w,
                fm,
                fv,
                shear_factor,
                ..
            } => {
                let mut u: f64 = 0.0;
                for e in 0..2 {
                    let sm = m[0][e].abs() / w[0] + m[1][e].abs() / w[1];
                    u = u.max(timber_interaction(n, sm, *nt, *nc, *fm));
                }
                let tau = shear_factor * v[0].hypot(v[1]) / area;
                u.max(tau / fv)
            }
            Law::Concrete(c) => {
                let rc = &c.rc;
                let nc = -n;
                let shear = (0..2)
                    .map(|a| (v[a] / rc.axes[a].shear_capacity(nc, rc.area, rc.fck)).powi(2))
                    .sum::<f64>()
                    .sqrt();
                let flex = if c.plain {
                    let sm = mmax(0) / c.w[0] + mmax(1) / c.w[1];
                    ((n / rc.area + sm) / c.ft).max((nc / rc.area + sm) / rc.fck)
                } else {
                    let axial = if n > 0.0 {
                        n / rc.tension_yield
                    } else {
                        nc / rc.squash_load
                    };
                    let bend: f64 = (0..2)
                        .map(|a| mmax(a) / concrete_cap(&rc.axes[a], nc))
                        .sum();
                    axial.max(bend)
                };
                flex.max(shear)
            }
            Law::Composite {
                area,
                w,
                xt,
                xc,
                ilss,
                shear_factor,
                ..
            } => {
                let mut u: f64 = 0.0;
                for e in 0..2 {
                    let sb = m[0][e].abs() / (w[0] * st.w_scale[0])
                        + m[1][e].abs() / (w[1] * st.w_scale[1]);
                    u = u.max(((n / area + sb) / xt).max((-n / area + sb) / xc));
                }
                for a in 0..2 {
                    if !st.delaminated[a] {
                        u = u.max(shear_factor * v[a].abs() / area / ilss);
                    }
                }
                u
            }
        }
    }
}

/// Builds laws once per distinct (material, section) pair, so members of the
/// same kind share their concrete interaction tables.
#[derive(Default)]
pub(crate) struct LawCache(Vec<(Material, Section, Law)>);

impl LawCache {
    pub fn get(&mut self, mat: &Material, sec: &Section) -> Law {
        if let Some((_, _, law)) = self.0.iter().find(|(m, s, _)| m == mat && s == sec) {
            return law.clone();
        }
        let law = Law::new(mat, sec);
        self.0.push((mat.clone(), sec.clone(), law.clone()));
        law
    }
}

fn timber_interaction(n: f64, sm: f64, nt: f64, nc: f64, fm: f64) -> f64 {
    if n >= 0.0 {
        n / nt + sm / fm
    } else {
        // Compression above f_c is crushing (a plastic plateau, booked
        // separately), not an interaction failure.
        (n / nc).abs().min(1.0).powi(2) + sm / fm
    }
}

fn concrete_cap(ax: &crate::concrete::RcAxis, nc: f64) -> f64 {
    ax.moment_capacity(nc).max(0.05 * ax.ultimate_moment)
}

/// History variables of a member's material.
#[derive(Clone, Debug)]
pub(crate) struct MatState {
    k_axial0: f64,
    ei0: [f64; 2],
    /// Total |plastic hinge rotation| over both ends and axes.
    pub hinge_rotation: f64,
    hinge_end: [[f64; 2]; 2],
    peak_util: f64,
    peak_moment: [[f64; 2]; 2],
    yielded: bool,
    // Concrete.
    axial_cracked: bool,
    rebar_strain: f64,
    crush_strain: f64,
    // Composite.
    delaminated: [bool; 2],
    w_scale: [f64; 2],
    ei_factor: [f64; 2],
    mc_scale: f64,
}

impl Default for MatState {
    fn default() -> Self {
        Self {
            k_axial0: 0.0,
            ei0: [0.0; 2],
            hinge_rotation: 0.0,
            hinge_end: [[0.0; 2]; 2],
            peak_util: 0.0,
            peak_moment: [[0.0; 2]; 2],
            yielded: false,
            axial_cracked: false,
            rebar_strain: 0.0,
            crush_strain: 0.0,
            delaminated: [false; 2],
            w_scale: [1.0; 2],
            ei_factor: [1.0; 2],
            mc_scale: 1.0,
        }
    }
}

impl MatState {
    pub fn new(k_axial: f64, ei_l: [f64; 2]) -> Self {
        Self {
            k_axial0: k_axial,
            ei0: ei_l,
            ..Default::default()
        }
    }
}

/// What the solver needs from the material for the next substep. The solver
/// does the plastic return against these caps inside its solve (or once
/// after it), and the material model resets them after every substep.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Mech {
    /// Axial stiffness EA/L in compression (and in tension while uncracked).
    pub k_axial: f64,
    /// Axial stiffness in tension: lower once concrete has cracked through.
    pub k_tension: f64,
    /// EI/L about local y and z.
    pub ei_l: [f64; 2],
    /// Axial yield force [tension, compression], N; infinite for none.
    pub n_cap: [f64; 2],
    /// Plastic end moment per bending axis, N·m; infinite for none.
    pub mp: [f64; 2],
}

impl Mech {
    /// Stiffness and yield caps of an undamaged member.
    pub fn new(law: &Law, k_axial: f64, ei_l: [f64; 2], plastic: bool) -> Self {
        let inf = f64::INFINITY;
        let (n_cap, mp) = if !plastic {
            ([inf; 2], [inf; 2])
        } else {
            match law {
                Law::Ductile { ny, mp, .. } => ([*ny; 2], *mp),
                Law::Timber { nc, .. } => ([inf, *nc], [inf; 2]),
                Law::Concrete(c) if !c.plain => (
                    [inf, c.rc.squash_load],
                    [0, 1].map(|a| concrete_cap(&c.rc.axes[a], 0.0)),
                ),
                Law::Concrete(c) => ([inf, c.rc.squash_load], [inf; 2]),
                _ => ([inf; 2], [inf; 2]),
            }
        };
        Self {
            k_axial,
            k_tension: k_axial,
            ei_l,
            n_cap,
            mp,
        }
    }
}

/// A substep's internal forces after the solver's plastic return, which the
/// solver has already added to the member's rest shape.
pub(crate) struct Committed {
    /// Axial force, N, tension positive.
    pub n: f64,
    /// End moments `[axis][end]`, N·m.
    pub m: [[f64; 2]; 2],
    /// Plastic elongation added this substep, m.
    pub dp_axial: f64,
    /// Plastic hinge rotation added this substep, rad, `[axis][end]`.
    pub dp: [[f64; 2]; 2],
    /// Total plastic hinge rotation, rad, `[axis][end]`.
    pub hinge: [[f64; 2]; 2],
    pub l0: f64,
    pub len: f64,
}

fn shears(m: &[[f64; 2]; 2], len: f64) -> [f64; 2] {
    [(m[0][0] + m[0][1]) / len, (m[1][0] + m[1][1]) / len]
}

type Failure = Option<(BreakCause, f64)>;

fn first(f: &mut Failure, cause: BreakCause, pos: f64) {
    if f.is_none() {
        *f = Some((cause, pos));
    }
}

/// Book the hinge rotation added this substep. Returns the |increment| per
/// `[axis][end]`.
fn book_hinges(st: &mut MatState, c: &Committed) -> [[f64; 2]; 2] {
    let inc: [[f64; 2]; 2] = std::array::from_fn(|a| [c.dp[a][0].abs(), c.dp[a][1].abs()]);
    for a in 0..2 {
        for e in 0..2 {
            if inc[a][e] > 0.0 {
                st.yielded = true;
                st.hinge_rotation += inc[a][e];
                st.hinge_end[a][e] += inc[a][e];
            }
        }
    }
    inc
}

/// Advance a member's material by one substep from the forces the solver
/// committed, and set the stiffness and yield caps for the next one. Returns
/// the failure mode and position along the member (0 = end a, 1 = end b) if
/// it broke.
pub(crate) fn update(
    law: &Law,
    st: &mut MatState,
    dmg: &mut MemberDamage,
    mech: &mut Mech,
    c: &Committed,
    plastic: bool,
) -> Failure {
    let mut fail = None;
    let (n, m) = (c.n, c.m);
    let inf = f64::INFINITY;

    match law {
        Law::Elastic => {}
        Law::Ductile {
            ny,
            mp,
            rupture_strain,
            hinge_capacity,
        } => {
            if c.dp_axial != 0.0 {
                dmg.plastic_strain += c.dp_axial.abs() / c.l0;
                st.yielded = true;
                if dmg.plastic_strain > *rupture_strain {
                    first(&mut fail, BreakCause::Rupture, 0.5);
                }
            }
            book_hinges(st, c);
            if st.hinge_rotation > *hinge_capacity {
                // Snaps at the end that has rotated most.
                let h = |e: usize| st.hinge_end[0][e] + st.hinge_end[1][e];
                first(
                    &mut fail,
                    BreakCause::Hinge,
                    if h(1) > h(0) { 1.0 } else { 0.0 },
                );
            }
            if plastic {
                // Axial force reduces the plastic moment: Mp,N = Mp (1 − n²),
                // exact for a rectangle and close for tubes.
                let r = (n.abs() / ny).min(1.0);
                mech.mp = mp.map(|m| m * (1.0 - r * r).max(0.05));
            }
            let progress =
                (dmg.plastic_strain / rupture_strain).max(st.hinge_rotation / hinge_capacity);
            if st.yielded {
                dmg.state = dmg.state.max(DamageState::Yielded);
                dmg.damage = dmg.damage.max(0.5 + 0.5 * progress.min(1.0));
            }
        }
        Law::Timber {
            area,
            nt,
            nc,
            w,
            fm,
            fv,
            shear_factor,
            crush_capacity,
        } => {
            if c.dp_axial < 0.0 {
                st.crush_strain += -c.dp_axial / c.l0;
                dmg.plastic_strain = st.crush_strain;
                if st.crush_strain > *crush_capacity {
                    first(&mut fail, BreakCause::Crushing, 0.5);
                }
            }
            if plastic && n > *nt {
                first(&mut fail, BreakCause::Splinter, 0.5);
            }
            let mut worst = (0.0, 0);
            for e in 0..2 {
                let sm = m[0][e].abs() / w[0] + m[1][e].abs() / w[1];
                let u = timber_interaction(n, sm, *nt, *nc, *fm);
                dmg.crack[e] = dmg.crack[e].max(((u - 0.8) / 0.2).clamp(0.0, 1.0));
                if u > worst.0 {
                    worst = (u, e);
                }
            }
            // A little slack so a crushing post with numerical noise in its
            // moments does not count as a bending failure.
            if plastic && worst.0 > 1.0 + 1e-3 {
                first(&mut fail, BreakCause::Splinter, worst.1 as f64);
            }
            let v = shears(&m, c.len);
            if plastic && shear_factor * v[0].hypot(v[1]) / area > *fv {
                first(&mut fail, BreakCause::TimberSplit, 0.5);
            }
            let crush = (st.crush_strain / crush_capacity).min(1.0);
            dmg.crush = [crush; 2];
            if crush > 0.0 {
                dmg.state = dmg.state.max(DamageState::Crushed);
            } else if dmg.crack[0].max(dmg.crack[1]) > 0.0 {
                dmg.state = dmg.state.max(DamageState::Cracked);
            }
        }
        Law::Concrete(cl) => {
            let rc = &cl.rc;
            let ncomp = -n;
            // Axial: crack through, then bars only in tension; yield and
            // crushing come back from the solver's return as plastic strain.
            if plastic && !st.axial_cracked && n > rc.tension_cracking {
                if cl.plain {
                    first(&mut fail, BreakCause::Brittle, 0.5);
                } else {
                    st.axial_cracked = true;
                    mech.k_tension = cl.es_as / c.l0;
                    mech.n_cap[0] = rc.tension_yield;
                }
            }
            if c.dp_axial > 0.0 {
                st.rebar_strain += c.dp_axial / c.l0;
                st.yielded = true;
                if st.rebar_strain > cl.rebar_rupture {
                    first(&mut fail, BreakCause::RebarRupture, 0.5);
                }
            } else if c.dp_axial < 0.0 {
                st.crush_strain += -c.dp_axial / c.l0;
                if st.crush_strain > cl.crush_capacity {
                    first(&mut fail, BreakCause::Crushing, 0.5);
                }
            }
            let inc = book_hinges(st, c);
            let mut shear_sq = 0.0;
            let mut shear_end = 0;
            for axis in 0..2 {
                let ax = &rc.axes[axis];
                for e in 0..2 {
                    st.peak_moment[axis][e] = st.peak_moment[axis][e].max(m[axis][e].abs());
                }
                if cl.plain {
                    let mcr = ((cl.ft + ncomp / rc.area) * cl.w[axis]).max(0.0);
                    for e in 0..2 {
                        if plastic && m[axis][e].abs() > mcr {
                            first(&mut fail, BreakCause::Brittle, e as f64);
                        }
                    }
                } else {
                    for e in 0..2 {
                        if inc[axis][e] > 0.0 && st.hinge_end[axis][e] > ax.hinge_capacity {
                            first(&mut fail, BreakCause::Crushing, e as f64);
                        }
                    }
                    let pk = st.peak_moment[axis][0].max(st.peak_moment[axis][1]);
                    if plastic && pk > ax.cracking_moment {
                        // Branson's effective inertia for the peak moment.
                        let r = (ax.cracking_moment / pk).powi(3);
                        let scale = r + (1.0 - r) * ax.cracked_inertia / ax.gross_inertia;
                        mech.ei_l[axis] = mech.ei_l[axis].min(st.ei0[axis] * scale);
                    }
                    if plastic {
                        mech.mp[axis] = concrete_cap(ax, ncomp);
                    }
                    for e in 0..2 {
                        let pk = st.peak_moment[axis][e];
                        let span = (ax.yield_moment - ax.cracking_moment).max(1e-9);
                        let cr = ((pk - ax.cracking_moment) / span).clamp(0.0, 1.0);
                        dmg.crack[e] = dmg.crack[e].max(cr);
                        let cu = (st.hinge_end[axis][e] / ax.hinge_capacity.max(1e-9)).min(1.0);
                        dmg.crush[e] = dmg.crush[e].max(cu);
                    }
                }
                let v = (m[axis][0] + m[axis][1]) / c.len;
                shear_sq += (v / ax.shear_capacity(ncomp, rc.area, rc.fck)).powi(2);
                if m[axis][1].abs() > m[axis][0].abs() {
                    shear_end = 1;
                }
            }
            if plastic && shear_sq.sqrt() > 1.0 {
                first(&mut fail, BreakCause::ConcreteShear, shear_end as f64);
            }
            let axial_crush = (st.crush_strain / cl.crush_capacity).min(1.0);
            for e in 0..2 {
                dmg.crush[e] = dmg.crush[e].max(axial_crush);
            }
            if st.axial_cracked {
                let t = (n / rc.tension_yield).clamp(0.1, 1.0);
                dmg.tension_crack = dmg.tension_crack.max(t);
            }
            dmg.plastic_strain = st.rebar_strain + st.crush_strain;
            if dmg.crack[0].max(dmg.crack[1]) > 0.0 || st.axial_cracked {
                dmg.state = dmg.state.max(DamageState::Cracked);
            }
            if st.yielded {
                dmg.state = dmg.state.max(DamageState::Yielded);
            }
            if st.crush_strain > 0.0 || dmg.crush[0].max(dmg.crush[1]) > 0.5 {
                dmg.state = dmg.state.max(DamageState::Crushed);
            }
            if st.yielded || st.crush_strain > 0.0 {
                let progress = dmg.crush[0]
                    .max(dmg.crush[1])
                    .max(st.rebar_strain / cl.rebar_rupture.max(1e-9));
                dmg.damage = dmg.damage.max(0.5 + 0.5 * progress.min(1.0));
            }
        }
        Law::Composite {
            area,
            w,
            xt,
            xc,
            ilss,
            shear_factor,
            first_ply,
            loss,
            delam_stiffness,
        } => {
            let mut worst = (0.0, 0);
            for e in 0..2 {
                let sb =
                    m[0][e].abs() / (w[0] * st.w_scale[0]) + m[1][e].abs() / (w[1] * st.w_scale[1]);
                let u = ((n / area + sb) / xt).max((-n / area + sb) / xc);
                if *first_ply < 1.0 {
                    let cr = ((u - first_ply) / (1.0 - first_ply)).clamp(0.0, 1.0);
                    dmg.crack[e] = dmg.crack[e].max(cr);
                }
                if u > worst.0 {
                    worst = (u, e);
                }
            }
            if plastic && worst.0 >= 1.0 {
                first(&mut fail, BreakCause::FibreRupture, worst.1 as f64);
            }
            let v = shears(&m, c.len);
            if plastic {
                for a in 0..2 {
                    if !st.delaminated[a] && shear_factor * v[a].abs() / area > *ilss {
                        st.delaminated[a] = true;
                        st.w_scale[a] = 0.5;
                        st.ei_factor[a] *= delam_stiffness;
                        dmg.delamination += 0.5;
                    }
                }
                let s = 1.0 - loss * dmg.crack[0].max(dmg.crack[1]);
                if s < st.mc_scale {
                    st.mc_scale = s;
                    mech.k_axial = st.k_axial0 * s;
                    mech.k_tension = mech.k_axial;
                }
                for a in 0..2 {
                    let ei = st.ei0[a] * st.mc_scale * st.ei_factor[a];
                    mech.ei_l[a] = mech.ei_l[a].min(ei);
                }
            }
            if st.delaminated[0] || st.delaminated[1] {
                dmg.state = dmg.state.max(DamageState::Delaminated);
            } else if dmg.crack[0].max(dmg.crack[1]) > 0.0 {
                dmg.state = dmg.state.max(DamageState::Cracked);
            }
        }
    }
    if !plastic {
        mech.n_cap = [inf; 2];
        mech.mp = [inf; 2];
    }

    let util = law.utilization(st, n, m, shears(&m, c.len));
    st.peak_util = st.peak_util.max(util);
    dmg.utilization = util;
    dmg.moment = m;
    dmg.axial = n;
    dmg.hinge = c.hinge;
    dmg.stiffness = (0..2)
        .map(|a| {
            if st.ei0[a] > 0.0 {
                mech.ei_l[a] / st.ei0[a]
            } else {
                1.0
            }
        })
        .fold(1.0, f64::min);
    let pre = match law {
        Law::Elastic => 0.0,
        Law::Ductile { .. } => 0.5 * st.peak_util.min(1.0),
        Law::Concrete(cl) if !cl.plain => 0.5 * st.peak_util.min(1.0),
        _ => st.peak_util.min(1.0),
    };
    dmg.damage = dmg.damage.max(pre);
    if fail.is_some() {
        dmg.state = DamageState::Broken;
        dmg.damage = 1.0;
    }
    fail
}
