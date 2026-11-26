//! Return mapping for axial yield and plastic hinges, and the per-substep
//! material update (damage, hinges, breaks) of each member.

use super::*;

/// Viscoplastic return of one force: minimises over the plastic increment dp
/// ½ k (c − dp)² + fy |dp| + ½ k r dp², with r = τ/h. Returns
/// (force, tangent stiffness, energy, dp).
pub(super) fn return_1d(k: f64, c: f64, fy: f64, r: f64) -> (f64, f64, f64, f64) {
    let trial = k * c;
    let excess = trial.abs() - fy;
    if excess.partial_cmp(&0.0) != Some(std::cmp::Ordering::Greater) {
        return (trial, k, 0.5 * k * c * c, 0.0);
    }
    let dp = excess / (k * (1.0 + r)) * trial.signum();
    let e = c - dp;
    (
        k * e,
        k * r / (1.0 + r),
        0.5 * k * e * e + fy * dp.abs() + 0.5 * k * r * dp * dp,
        dp,
    )
}

/// The same for the two end moments of one bending plane, which are coupled:
/// M = K (θ − dp), K symmetric (rigid ends: EI/L [[4, 2], [2, 4]]; joints
/// change it, and a pinned end has a zero row). Each end has its own plastic
/// moment `mp` (the member's, or a weaker joint's) and viscous term
/// η = K_ii r. Returns (M, tangent, energy, dp).
#[allow(clippy::type_complexity)]
pub(super) fn return_bend(
    k: [[f64; 2]; 2],
    th: [f64; 2],
    mp: [f64; 2],
    r: f64,
) -> ([f64; 2], [[f64; 2]; 2], f64, [f64; 2]) {
    let trial = [
        k[0][0] * th[0] + k[0][1] * th[1],
        k[1][0] * th[0] + k[1][1] * th[1],
    ];
    let mut act = [trial[0].abs() > mp[0], trial[1].abs() > mp[1]];
    let mut sg = [trial[0].signum(), trial[1].signum()];
    let kd = [k[0][0] * (1.0 + r), k[1][1] * (1.0 + r)];
    let mut dp = [0.0; 2];
    let mut m = trial;
    if act[0] || act[1] {
        for _ in 0..4 {
            let f = [trial[0] - mp[0] * sg[0], trial[1] - mp[1] * sg[1]];
            dp = match act {
                [true, true] => {
                    let det = kd[0] * kd[1] - k[0][1] * k[0][1];
                    [
                        (kd[1] * f[0] - k[0][1] * f[1]) / det,
                        (kd[0] * f[1] - k[0][1] * f[0]) / det,
                    ]
                }
                [true, false] => [f[0] / kd[0], 0.0],
                [false, true] => [0.0, f[1] / kd[1]],
                [false, false] => [0.0; 2],
            };
            m = [
                trial[0] - k[0][0] * dp[0] - k[0][1] * dp[1],
                trial[1] - k[1][0] * dp[0] - k[1][1] * dp[1],
            ];
            let mut changed = false;
            for i in 0..2 {
                if act[i] && dp[i] * sg[i] < 0.0 {
                    act[i] = false;
                    changed = true;
                } else if !act[i] && m[i].abs() > mp[i] * (1.0 + 1e-12) {
                    act[i] = true;
                    sg[i] = m[i].signum();
                    changed = true;
                }
            }
            if !changed {
                break;
            }
        }
    }
    // Tangent K − K_A (K_AA + η_A)⁻¹ K_Aᵀ over the active ends A.
    let kt = match act {
        [true, true] => {
            let det = kd[0] * kd[1] - k[0][1] * k[0][1];
            let inv = [[kd[1] / det, -k[0][1] / det], [-k[0][1] / det, kd[0] / det]];
            let mut t = k;
            for a in 0..2 {
                for b in 0..2 {
                    for i in 0..2 {
                        for j in 0..2 {
                            t[a][b] -= k[a][i] * inv[i][j] * k[j][b];
                        }
                    }
                }
            }
            t
        }
        [true, false] | [false, true] => {
            let i = if act[0] { 0 } else { 1 };
            let mut t = k;
            for a in 0..2 {
                for b in 0..2 {
                    t[a][b] -= k[a][i] * k[i][b] / kd[i];
                }
            }
            t
        }
        [false, false] => k,
    };
    let e = [th[0] - dp[0], th[1] - dp[1]];
    let mut energy = 0.5 * (k[0][0] * e[0] * e[0] + k[1][1] * e[1] * e[1]) + k[0][1] * e[0] * e[1];
    for i in 0..2 {
        // Only yielding ends: mp may be infinite elsewhere.
        if dp[i] != 0.0 {
            energy += mp[i] * dp[i].abs() + 0.5 * k[i][i] * r * dp[i] * dp[i];
        }
    }
    (m, kt, energy, dp)
}
impl DynamicSim {
    /// τ/h of the plastic return: (inside the solve, at the material update).
    /// `None` inside the solve means the solve is elastic.
    pub(super) fn plastic_ratio(&self, h: f64) -> (Option<f64>, f64) {
        let c = &self.config;
        if c.solver == Solver::Direct && c.implicit_plasticity {
            let r = c.plastic_relaxation.max(0.0) / h;
            (c.plasticity.then_some(r), r)
        } else {
            (None, 0.0)
        }
    }

    pub(super) fn update_material(&mut self, i: usize, h: f64) {
        let gm = self.geom(i);
        let st = self.strains(i, &gm);
        let plastic = self.config.plasticity;
        // The same plastic return the solve used, now committed.
        let r = self.plastic_ratio(h).1;
        let resp = self.response(i, &st, plastic.then_some(r));
        self.commit_material(i, gm.len, &resp);
    }

    /// Commit the response `resp` of member `i` at its current length `len`:
    /// plastic strains, damage, joint utilisation and failure.
    #[allow(clippy::needless_range_loop)]
    pub(super) fn commit_material(&mut self, i: usize, len: f64, resp: &Response) {
        let plastic = self.config.plasticity;
        let bm = &mut self.beams[i];
        bm.plastic_elongation += resp.dp_axial;
        // Rotation at an end where the joint is weaker than the member is the
        // joint yielding; the material model only sees the member's own.
        let mut member_dp = resp.dp;
        for axis in 0..2 {
            for end in 0..2 {
                let dp = resp.dp[axis][end];
                bm.hinge[axis][end] += dp;
                bm.joint_rotation[end] += dp.abs();
                if bm.joints[end].moment_capacity[axis] < bm.mp_now[axis] {
                    bm.joint_hinge[axis][end] += dp;
                    member_dp[axis][end] = 0.0;
                }
            }
        }
        let member_hinge: [[f64; 2]; 2] =
            std::array::from_fn(|a| [0, 1].map(|e| bm.hinge[a][e] - bm.joint_hinge[a][e]));
        bm.yield_mask = resp.yield_mask;
        let mut mech = Mech {
            k_axial: bm.k_axial,
            k_tension: bm.k_tension,
            ei_l: bm.ei_l,
            n_cap: bm.n_cap,
            mp: bm.mp_now,
        };
        let committed = Committed {
            n: resp.n,
            m: resp.m,
            dp_axial: resp.dp_axial,
            dp: member_dp,
            hinge: member_hinge,
            l0: bm.l0,
            len,
        };
        let failure = damage::update(
            &bm.law,
            &mut bm.mat,
            &mut bm.damage,
            &mut mech,
            &committed,
            plastic,
        );
        bm.k_axial = mech.k_axial;
        bm.k_tension = mech.k_tension;
        bm.ei_l = mech.ei_l;
        bm.n_cap = mech.n_cap;
        bm.mp_now = mech.mp;

        // Connections: force and moment against their capacities, and the
        // plastic rotation at each end against the joint's rotation capacity.
        let mut failure = failure;
        let v = [
            (resp.m[0][0] + resp.m[0][1]) / len,
            (resp.m[1][0] + resp.m[1][1]) / len,
        ];
        for end in 0..2 {
            let j = &bm.joints[end];
            if j.is_rigid() {
                continue;
            }
            let u = j
                .utilization(resp.n, v, [resp.m[0][end], resp.m[1][end]])
                .max(bm.joint_rotation[end] / j.rotation_capacity);
            bm.damage.joint[end] = bm.damage.joint[end].max(u.min(1.0));
            let force = (resp.n * resp.n + v[0] * v[0] + v[1] * v[1]).sqrt();
            if failure.is_none()
                && (force > j.force_capacity || bm.joint_rotation[end] > j.rotation_capacity)
            {
                failure = Some((BreakCause::Connection, end as f64));
                bm.damage.state = damage::DamageState::Broken;
                bm.damage.damage = 1.0;
            }
        }

        if let Some((cause, position)) = failure {
            bm.broken = true;
            self.events.push(BreakEvent {
                member: i,
                time: self.time,
                cause,
                position,
            });
        }
    }
}
