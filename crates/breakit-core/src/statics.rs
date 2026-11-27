//! Linear static analysis of a 3D frame with the direct stiffness method.
//!
//! Each member is a 12-DOF Euler-Bernoulli beam element. The global system is
//! assembled as a sparse matrix and solved with faer's sparse Cholesky.

use nalgebra::{SMatrix, SVector};

use crate::frame::member_axes;
use crate::joint::bending_stiffness;
use crate::{NodeLoad, NodeSpring, Structure, Vec3};

type Mat12 = SMatrix<f64, 12, 12>;
type Vec12 = SVector<f64, 12>;

/// Internal forces at the two ends of a member, in member local axes.
/// Sign convention: forces the member exerts on its nodes are reversed;
/// `axial` is positive in tension.
#[derive(Clone, Debug, Default)]
pub struct MemberForces {
    pub axial: f64,
    pub torsion: f64,
    /// Bending moments about local y at ends a and b, N·m.
    pub my: [f64; 2],
    /// Bending moments about local z at ends a and b, N·m.
    pub mz: [f64; 2],
    /// Shear forces along local y and z, N.
    pub vy: f64,
    pub vz: f64,
    /// Utilisation of the intact member, 1.0 = at capacity, using the same
    /// material rules as the dynamic solver (for steel
    /// (N/Ny)² + |My|/Mpy + |Mz|/Mpz).
    pub utilization: f64,
    /// Utilisation of the connections at ends a and b (force and moment
    /// against [`crate::Joint`] capacities); 0 for rigid joints.
    pub joint_utilization: [f64; 2],
}

#[derive(Clone, Debug)]
pub struct StaticResult {
    /// Node translations, m.
    pub displacements: Vec<Vec3>,
    /// Node rotations (small-angle vectors), rad.
    pub rotations: Vec<Vec3>,
    pub member_forces: Vec<MemberForces>,
}

#[derive(Debug, PartialEq)]
pub enum SolveError {
    /// The structure can move without resisting (a mechanism) or is unsupported.
    Unstable,
}

impl std::fmt::Display for SolveError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "structure is unstable (mechanism or missing supports)")
    }
}
impl std::error::Error for SolveError {}

/// Local 12x12 stiffness, DOF order per end: ux uy uz rx ry rz.
pub fn local_stiffness(s: &Structure, m: usize) -> Mat12 {
    let mem = &s.members[m];
    let l = s.member_length(m);
    let (e, g) = (mem.material.youngs_modulus, mem.material.shear_modulus);
    let sec = &mem.section;
    let mut k = Mat12::zeros();
    let ea = e * sec.area / l;
    k[(0, 0)] = ea;
    k[(6, 6)] = ea;
    k[(0, 6)] = -ea;
    k[(6, 0)] = -ea;
    let gj = g * sec.j / l;
    k[(3, 3)] = gj;
    k[(9, 9)] = gj;
    k[(3, 9)] = -gj;
    k[(9, 3)] = -gj;
    // Bending: end moments M = K φ, with φ the node rotations relative to the
    // chord and K from the member's bending stiffness and its end joints.
    // About z: DOFs uy (1, 7) and rz (5, 11), chord rotation (uy_b − uy_a)/L.
    // About y: DOFs uz (2, 8) and ry (4, 10), chord rotation −(uz_b − uz_a)/L.
    for (axis, idx, sign, inertia) in [
        (0, [2, 4, 8, 10], -1.0, sec.iy),
        (1, [1, 5, 7, 11], 1.0, sec.iz),
    ] {
        let kb = bending_stiffness(
            e * inertia / l,
            [mem.ends[0].stiffness[axis], mem.ends[1].stiffness[axis]],
        );
        // φ = B [v_a, θ_a, v_b, θ_b].
        let c = sign / l;
        let bm = [[c, 1.0, -c, 0.0], [c, 0.0, -c, 1.0]];
        for r in 0..4 {
            for col in 0..4 {
                let mut v = 0.0;
                for i in 0..2 {
                    for j in 0..2 {
                        v += bm[i][r] * kb[i][j] * bm[j][col];
                    }
                }
                k[(idx[r], idx[col])] = v;
            }
        }
    }
    k
}

/// 12x12 rotation from global to member local axes.
pub fn transformation(s: &Structure, m: usize) -> Mat12 {
    let mem = &s.members[m];
    let (x, y, z) = member_axes(&s.nodes[mem.a], &s.nodes[mem.b]);
    let mut t = Mat12::zeros();
    for blk in 0..4 {
        for (row, axis) in [x, y, z].iter().enumerate() {
            for col in 0..3 {
                t[(blk * 3 + row, blk * 3 + col)] = axis[col];
            }
        }
    }
    t
}

/// Solve K·u = f for the given nodal loads (sparse Cholesky).
pub fn solve_static(s: &Structure, loads: &[NodeLoad]) -> Result<StaticResult, SolveError> {
    solve_static_with_springs(s, loads, &[])
}

/// [`solve_static`] with elastic supports added to the stiffness (for
/// buoyancy in the build-mode check: a structure on floats has no fixed
/// support it needs).
pub fn solve_static_with_springs(
    s: &Structure,
    loads: &[NodeLoad],
    springs: &[NodeSpring],
) -> Result<StaticResult, SolveError> {
    use faer::linalg::solvers::Solve;
    use faer::sparse::{SparseColMat, Triplet};
    use faer::{Mat, Side};

    let n = s.nodes.len() * 6;
    let mut f = vec![0.0; n];
    let mut diag = vec![0.0; n];
    let mut ke_global = Vec::with_capacity(s.members.len());
    let mut entries: Vec<(usize, usize, f64)> = Vec::with_capacity(s.members.len() * 78);

    for m in 0..s.members.len() {
        let t = transformation(s, m);
        let kl = local_stiffness(s, m);
        let k = t.transpose() * kl * t;
        let mem = &s.members[m];
        let dofs = element_dofs(mem.a, mem.b);
        for r in 0..12 {
            diag[dofs[r]] += k[(r, r)];
            for c in 0..12 {
                if dofs[r] >= dofs[c] {
                    entries.push((dofs[r], dofs[c], k[(r, c)]));
                }
            }
        }
        ke_global.push((kl, t));
    }
    for sp in springs {
        for i in 0..3 {
            let k = sp.stiffness[i];
            if k > 0.0 {
                let d = sp.node * 6 + i;
                diag[d] += k;
                entries.push((d, d, k));
            }
        }
    }
    for l in loads {
        for i in 0..3 {
            f[l.node * 6 + i] += l.force[i];
            f[l.node * 6 + 3 + i] += l.moment[i];
        }
    }

    // A node where members meet through pins can be free to rotate about an
    // axis no member resists. That rotation carries no load, so pin it with a
    // tiny spring instead of calling the structure unstable.
    if s.members
        .iter()
        .any(|m| m.ends.iter().any(|j| !j.is_rigid()))
    {
        let scale = (0..n)
            .filter(|d| d % 6 >= 3)
            .map(|d| diag[d])
            .fold(0.0, f64::max);
        for d in (0..n).filter(|d| d % 6 >= 3 && diag[*d] != 0.0) {
            entries.push((d, d, 1e-10 * scale));
        }
    }

    // Free DOFs: not restrained and attached to at least one member.
    let mut restrained = vec![false; n];
    for sup in &s.supports {
        for i in 0..6 {
            restrained[sup.node * 6 + i] |= sup.fixed[i];
        }
    }
    let mut map = vec![usize::MAX; n];
    let free: Vec<usize> = (0..n)
        .filter(|&d| !restrained[d] && diag[d] != 0.0)
        .collect();
    for (i, &d) in free.iter().enumerate() {
        map[d] = i;
    }
    let nf = free.len();
    let mut u = vec![0.0; n];
    if nf > 0 {
        let trip: Vec<_> = entries
            .iter()
            .filter(|(r, c, _)| map[*r] != usize::MAX && map[*c] != usize::MAX)
            .map(|&(r, c, v)| Triplet::new(map[r], map[c], v))
            .collect();
        let kff = SparseColMat::<usize, f64>::try_new_from_triplets(nf, nf, &trip)
            .map_err(|_| SolveError::Unstable)?;
        let llt = kff
            .sp_cholesky(Side::Lower)
            .map_err(|_| SolveError::Unstable)?;
        let mut x = Mat::<f64>::from_fn(nf, 1, |i, _| f[free[i]]);
        llt.solve_in_place(x.as_mut());
        for (i, &d) in free.iter().enumerate() {
            u[d] = x[(i, 0)];
        }
        // Reject near-singular systems that Cholesky still accepted.
        if !u.iter().all(|v| v.is_finite()) || u.iter().fold(0.0f64, |a, v| a.max(v.abs())) > 1e6 {
            return Err(SolveError::Unstable);
        }
    }

    let displacements = (0..s.nodes.len())
        .map(|i| Vec3::new(u[i * 6], u[i * 6 + 1], u[i * 6 + 2]))
        .collect();
    let rotations = (0..s.nodes.len())
        .map(|i| Vec3::new(u[i * 6 + 3], u[i * 6 + 4], u[i * 6 + 5]))
        .collect();
    let mut laws = crate::damage::LawCache::default();
    let member_forces = s
        .members
        .iter()
        .zip(ke_global.iter())
        .map(|(mem, (kl, t))| {
            let dofs = element_dofs(mem.a, mem.b);
            let ue = Vec12::from_fn(|r, _| u[dofs[r]]);
            let fl = kl * (t * ue);
            let axial = fl[6];
            let mut mf = MemberForces {
                axial,
                torsion: fl[9],
                my: [-fl[4], fl[10]],
                mz: [-fl[5], fl[11]],
                vy: fl[7],
                vz: fl[8],
                utilization: 0.0,
                joint_utilization: [0.0; 2],
            };
            for end in 0..2 {
                mf.joint_utilization[end] =
                    mem.ends[end].utilization(axial, [mf.vz, mf.vy], [mf.my[end], mf.mz[end]]);
            }
            let law = laws.get(&mem.material, &mem.section);
            mf.utilization = law.intact_utilization(mf.axial, [mf.my, mf.mz], [mf.vz, mf.vy]);
            mf
        })
        .collect();

    Ok(StaticResult {
        displacements,
        rotations,
        member_forces,
    })
}

fn element_dofs(a: usize, b: usize) -> [usize; 12] {
    let mut d = [0; 12];
    for i in 0..6 {
        d[i] = a * 6 + i;
        d[6 + i] = b * 6 + i;
    }
    d
}
