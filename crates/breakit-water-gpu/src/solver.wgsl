// The water solver of `breakit-water` as compute passes.
//
// One substep is the passes below in order; each cell or face is written
// by exactly one invocation of a pass and never read by another invocation
// of the same pass, so the result does not depend on scheduling. The
// scheme, and every formula here, is `Solver::substep` in
// `breakit-water/src/solver.rs` with the red-black pressure solver; keep
// the two in step.
//
// Every array has the same layout: `(nx + 1) x (nz + 1)` entries, entry
// `(i, j)` holding cell `(i, j)`, the x-face on its low-x side and the
// z-face on its low-z side. The last column has no cell and no z-face, the
// last row no cell and no x-face.

struct Params {
    nx: u32,
    nz: u32,
    dispersive: u32,
    pad0: u32,
    dx: f32,
    dt: f32,
    gravity: f32,
    dry: f32,
    manning: f32,
    min_depth: f32,
    alpha: f32,
    beta: f32,
    max_speed: f32,
    omega: f32,
    paddle: f32,
    sponge_velocity: f32,
    // Per side (x low, x high, z low, z high): 0 wall, 1 piston, 2 radiating.
    kind: vec4<u32>,
    level: vec4<f32>,
    current: vec4<f32>,
}

@group(0) @binding(0) var<uniform> P: Params;
// (surface, u, v, w)
@group(0) @binding(1) var<storage, read_write> state: array<vec4<f32>>;
// (bed, bed slope at the x-face, at the z-face, volume source)
@group(0) @binding(2) var<storage, read_write> ground: array<vec4<f32>>;
// (depth at the x-face, at the z-face, breaking 0 or 1, unused)
@group(0) @binding(3) var<storage, read_write> depth: array<vec4<f32>>;
// (predicted u, predicted v, unused, unused)
@group(0) @binding(4) var<storage, read_write> pred: array<vec4<f32>>;
// (diagonal, coupling to the cell at +x, at +z, right-hand side); after the
// pressure solve, x is the outflow limiter of the cell
@group(0) @binding(5) var<storage, read_write> mat: array<vec4<f32>>;
// (predicted w, new surface, unused, breaking strength)
@group(0) @binding(6) var<storage, read_write> cell: array<vec4<f32>>;
@group(0) @binding(7) var<storage, read_write> pres: array<f32>;
// (blockage, its x-velocity, its z-velocity, sponge rate)
@group(0) @binding(8) var<storage, read_write> drag: array<vec4<f32>>;

const FRICTION_DEPTH: f32 = 0.75;

fn ix(i: u32, j: u32) -> u32 {
    return j * (P.nx + 1u) + i;
}

// Depth of cell (i, j).
fn hc(i: u32, j: u32) -> f32 {
    let k = ix(i, j);
    return max(state[k].x - ground[k].x, 0.0);
}

fn is_active(i: u32, j: u32) -> bool {
    return P.dispersive != 0u && hc(i, j) > P.min_depth && depth[ix(i, j)].z < 0.5;
}

// Velocity through a boundary face next to cell (i, j). `outward` is +1
// on the high sides and -1 on the low sides.
fn boundary_velocity(side: u32, i: u32, j: u32, outward: f32) -> f32 {
    let kind = P.kind[side];
    let h = hc(i, j);
    if kind == 0u || h <= P.dry {
        return 0.0;
    }
    if kind == 1u {
        return P.paddle;
    }
    return P.current[side] + outward * sqrt(P.gravity / h) * (state[ix(i, j)].x - P.level[side]);
}

// Upwind depth at x-face (i, j) for the face velocity `vel`.
fn face_depth_x(i: u32, j: u32, vel: f32) -> f32 {
    if i == 0u {
        return select(hc(0u, j), 0.0, P.kind.x == 0u);
    }
    if i == P.nx {
        return select(hc(P.nx - 1u, j), 0.0, P.kind.y == 0u);
    }
    let l = ix(i - 1u, j);
    let r = ix(i, j);
    let bf = max(ground[l].x, ground[r].x);
    var zf = max(state[l].x, state[r].x);
    if vel > 0.0 {
        zf = state[l].x;
    } else if vel < 0.0 {
        zf = state[r].x;
    }
    return max(zf - bf, 0.0);
}

fn face_depth_z(i: u32, j: u32, vel: f32) -> f32 {
    if j == 0u {
        return select(hc(i, 0u), 0.0, P.kind.z == 0u);
    }
    if j == P.nz {
        return select(hc(i, P.nz - 1u), 0.0, P.kind.w == 0u);
    }
    let l = ix(i, j - 1u);
    let r = ix(i, j);
    let bf = max(ground[l].x, ground[r].x);
    var zf = max(state[l].x, state[r].x);
    if vel > 0.0 {
        zf = state[l].x;
    } else if vel < 0.0 {
        zf = state[r].x;
    }
    return max(zf - bf, 0.0);
}

// Inverse mass of x-face (i, j): 0 for boundary and dry faces.
fn minv_x(i: u32, j: u32) -> f32 {
    if i == 0u || i >= P.nx || depth[ix(i, j)].x <= P.dry {
        return 0.0;
    }
    return 1.0 / max(0.5 * (hc(i - 1u, j) + hc(i, j)), P.dry);
}

fn minv_z(i: u32, j: u32) -> f32 {
    if j == 0u || j >= P.nz || depth[ix(i, j)].y <= P.dry {
        return 0.0;
    }
    return 1.0 / max(0.5 * (hc(i, j - 1u) + hc(i, j)), P.dry);
}

// The incompressibility residual of cell (i, j) for the predicted
// velocities, with `w2` twice the vertical velocity.
fn constraint(i: u32, j: u32, w2: f32) -> f32 {
    let fl = ix(i, j);
    let fr = ix(i + 1u, j);
    let ft = ix(i, j + 1u);
    return hc(i, j) * (pred[fr].x - pred[fl].x + pred[ft].y - pred[fl].y) / P.dx
        - (pred[fr].x * ground[fr].y + pred[fl].x * ground[fl].y)
        - (pred[ft].y * ground[ft].z + pred[fl].y * ground[fl].z)
        + w2;
}

// 1: upwind face depths for the current velocities.
@compute @workgroup_size(8, 8)
fn depths(@builtin(global_invocation_id) id: vec3<u32>) {
    let i = id.x;
    let j = id.y;
    if i > P.nx || j > P.nz {
        return;
    }
    let k = ix(i, j);
    var d = depth[k];
    d.x = 0.0;
    d.y = 0.0;
    if j < P.nz {
        d.x = face_depth_x(i, j, state[k].y);
    }
    if i < P.nx {
        d.y = face_depth_z(i, j, state[k].z);
    }
    depth[k] = d;
}

// 2: predictor: gravity, advection, friction, blockage, sponge; the
// boundary velocities; upwind advection of the vertical velocity.
@compute @workgroup_size(8, 8)
fn predict(@builtin(global_invocation_id) id: vec3<u32>) {
    let i = id.x;
    let j = id.y;
    let nx = P.nx;
    let nz = P.nz;
    if i > nx || j > nz {
        return;
    }
    let k = ix(i, j);
    let dt = P.dt;
    let fric = P.gravity * P.manning * P.manning;
    var un = 0.0;
    var vn = 0.0;
    if j < nz {
        if i == 0u {
            un = boundary_velocity(0u, 0u, j, -1.0);
        } else if i == nx {
            un = boundary_velocity(1u, nx - 1u, j, 1.0);
        } else if depth[k].x > P.dry {
            let l = ix(i - 1u, j);
            let uf = state[k].y;
            let hf = depth[k].x;
            let hbar = max(0.5 * (hc(i - 1u, j) + hc(i, j)), hf);
            let q0 = depth[k - 1u].x * state[k - 1u].y;
            let q1 = hf * uf;
            let q2 = depth[k + 1u].x * state[k + 1u].y;
            let ql = 0.5 * (q0 + q1);
            let qr = 0.5 * (q1 + q2);
            var adv = 0.0;
            if ql > 0.0 {
                adv += ql * (uf - state[k - 1u].y);
            }
            if qr < 0.0 {
                adv += qr * (state[k + 1u].y - uf);
            }
            let up = ix(i, j + 1u);
            let upl = ix(i - 1u, j + 1u);
            let qb = 0.5 * (depth[l].y * state[l].z + depth[k].y * state[k].z);
            let qt = 0.5 * (depth[upl].y * state[upl].z + depth[up].y * state[up].z);
            if qb > 0.0 && j > 0u {
                adv += qb * (uf - state[ix(i, j - 1u)].y);
            }
            if qt < 0.0 && j + 1u < nz {
                adv += qt * (state[up].y - uf);
            }
            var us = uf - dt * (P.gravity * (state[k].x - state[l].x) / P.dx + adv / (hbar * P.dx));
            let vbar = 0.25 * (state[l].z + state[k].z + state[upl].z + state[up].z);
            let speed = sqrt(uf * uf + vbar * vbar);
            if hf < FRICTION_DEPTH {
                us /= 1.0 + dt * fric * speed / (hf * pow(hf, 1.0 / 3.0));
            }
            let kap = 0.5 * (drag[l].x + drag[k].x) / hbar;
            if kap > 0.0 {
                let bu = 0.5 * (drag[l].y + drag[k].y);
                let bv = 0.5 * (drag[l].z + drag[k].z);
                let rel = sqrt((uf - bu) * (uf - bu) + (vbar - bv) * (vbar - bv));
                us = (us + dt * kap * rel * bu) / (1.0 + dt * kap * rel);
            }
            let sp = dt * 0.5 * (drag[l].w + drag[k].w);
            us = (us + sp * P.sponge_velocity) / (1.0 + sp);
            un = clamp(us, -P.max_speed, P.max_speed);
        }
    }
    if i < nx {
        if j == 0u {
            vn = boundary_velocity(2u, i, 0u, -1.0);
        } else if j == nz {
            vn = boundary_velocity(3u, i, nz - 1u, 1.0);
        } else if depth[k].y > P.dry {
            let l = ix(i, j - 1u);
            let vf = state[k].z;
            let hf = depth[k].y;
            let hbar = max(0.5 * (hc(i, j - 1u) + hc(i, j)), hf);
            let up = ix(i, j + 1u);
            let q0 = depth[l].y * state[l].z;
            let q1 = hf * vf;
            let q2 = depth[up].y * state[up].z;
            let ql = 0.5 * (q0 + q1);
            let qr = 0.5 * (q1 + q2);
            var adv = 0.0;
            if ql > 0.0 {
                adv += ql * (vf - state[l].z);
            }
            if qr < 0.0 {
                adv += qr * (state[up].z - vf);
            }
            let r = ix(i + 1u, j);
            let rl = ix(i + 1u, j - 1u);
            let qb = 0.5 * (depth[l].x * state[l].y + depth[k].x * state[k].y);
            let qt = 0.5 * (depth[rl].x * state[rl].y + depth[r].x * state[r].y);
            if qb > 0.0 && i > 0u {
                adv += qb * (vf - state[ix(i - 1u, j)].z);
            }
            if qt < 0.0 && i + 1u < nx {
                adv += qt * (state[r].z - vf);
            }
            var vs = vf - dt * (P.gravity * (state[k].x - state[l].x) / P.dx + adv / (hbar * P.dx));
            let ubar = 0.25 * (state[l].y + state[rl].y + state[k].y + state[r].y);
            let speed = sqrt(vf * vf + ubar * ubar);
            if hf < FRICTION_DEPTH {
                vs /= 1.0 + dt * fric * speed / (hf * pow(hf, 1.0 / 3.0));
            }
            let kap = 0.5 * (drag[l].x + drag[k].x) / hbar;
            if kap > 0.0 {
                let bu = 0.5 * (drag[l].y + drag[k].y);
                let bv = 0.5 * (drag[l].z + drag[k].z);
                let rel = sqrt((ubar - bu) * (ubar - bu) + (vf - bv) * (vf - bv));
                vs = (vs + dt * kap * rel * bv) / (1.0 + dt * kap * rel);
            }
            vs /= 1.0 + dt * 0.5 * (drag[l].w + drag[k].w);
            vn = clamp(vs, -P.max_speed, P.max_speed);
        }
    }
    pred[k] = vec4<f32>(un, vn, 0.0, 0.0);

    var c = cell[k];
    if i < nx && j < nz {
        let wc = state[k].w;
        c.x = wc;
        if P.dispersive != 0u {
            if hc(i, j) <= P.dry {
                c.x = 0.0;
            } else {
                let uc = 0.5 * (state[k].y + state[k + 1u].y);
                let vc = 0.5 * (state[k].z + state[ix(i, j + 1u)].z);
                var d = 0.0;
                if uc > 0.0 && i > 0u {
                    d += uc * (wc - state[k - 1u].w);
                } else if uc < 0.0 && i + 1u < nx {
                    d += uc * (state[k + 1u].w - wc);
                }
                if vc > 0.0 && j > 0u {
                    d += vc * (wc - state[ix(i, j - 1u)].w);
                } else if vc < 0.0 && j + 1u < nz {
                    d += vc * (state[ix(i, j + 1u)].w - wc);
                }
                c.x = wc - dt * d / P.dx;
            }
        }
    } else {
        c.x = 0.0;
    }
    cell[k] = c;
}

// 3: the pressure system of the active cells; identity rows elsewhere.
@compute @workgroup_size(8, 8)
fn assemble(@builtin(global_invocation_id) id: vec3<u32>) {
    let i = id.x;
    let j = id.y;
    if i > P.nx || j > P.nz {
        return;
    }
    let k = ix(i, j);
    if i >= P.nx || j >= P.nz || !is_active(i, j) {
        mat[k] = vec4<f32>(1.0, 0.0, 0.0, 0.0);
        pres[k] = 0.0;
        return;
    }
    let h = hc(i, j);
    let fr = ix(i + 1u, j);
    let ft = ix(i, j + 1u);
    let hd = h / P.dx;
    let mxl = minv_x(i, j);
    let mxr = minv_x(i + 1u, j);
    let mzb = minv_z(i, j);
    let mzt = minv_z(i, j + 1u);
    let sxl = ground[k].y;
    let sxr = ground[fr].y;
    let szb = ground[k].z;
    let szt = ground[ft].z;
    let diag = (hd - sxr) * (hd - sxr) * mxr + (hd + sxl) * (hd + sxl) * mxl
        + (hd - szt) * (hd - szt) * mzt + (hd + szb) * (hd + szb) * mzb
        + 4.0 / h;
    var ax = 0.0;
    var az = 0.0;
    if i + 1u < P.nx && is_active(i + 1u, j) {
        ax = (hd - sxr) * (-hc(i + 1u, j) / P.dx - sxr) * mxr;
    }
    if j + 1u < P.nz && is_active(i, j + 1u) {
        az = (hd - szt) * (-hc(i, j + 1u) / P.dx - szt) * mzt;
    }
    let rhs = -constraint(i, j, 2.0 * cell[k].x) / P.dt;
    mat[k] = vec4<f32>(diag, ax, az, rhs);
}

// 4: one colour of a red-black over-relaxation sweep.
fn relax(i: u32, j: u32) {
    let k = ix(i, j);
    let m = mat[k];
    var sum = 0.0;
    if i + 1u < P.nx {
        sum += m.y * pres[k + 1u];
    }
    if i > 0u {
        sum += mat[k - 1u].y * pres[k - 1u];
    }
    if j + 1u < P.nz {
        sum += m.z * pres[ix(i, j + 1u)];
    }
    if j > 0u {
        let b = ix(i, j - 1u);
        sum += mat[b].z * pres[b];
    }
    let p = pres[k];
    pres[k] = p + P.omega * ((m.w - sum) / m.x - p);
}

@compute @workgroup_size(8, 8)
fn relax_red(@builtin(global_invocation_id) id: vec3<u32>) {
    if id.x < P.nx && id.y < P.nz && (id.x + id.y) % 2u == 0u {
        relax(id.x, id.y);
    }
}

@compute @workgroup_size(8, 8)
fn relax_black(@builtin(global_invocation_id) id: vec3<u32>) {
    if id.x < P.nx && id.y < P.nz && (id.x + id.y) % 2u == 1u {
        relax(id.x, id.y);
    }
}

// 5: the pressure corrects the face velocities.
@compute @workgroup_size(8, 8)
fn correct_faces(@builtin(global_invocation_id) id: vec3<u32>) {
    let i = id.x;
    let j = id.y;
    if i > P.nx || j > P.nz || P.dispersive == 0u {
        return;
    }
    let k = ix(i, j);
    var v = pred[k];
    if j < P.nz && i > 0u && i < P.nx {
        let l = k - 1u;
        let g = minv_x(i, j)
            * ((hc(i - 1u, j) * pres[l] - hc(i, j) * pres[k]) / P.dx - ground[k].y * (pres[l] + pres[k]));
        v.x += P.dt * g;
    }
    if i < P.nx && j > 0u && j < P.nz {
        let l = ix(i, j - 1u);
        let g = minv_z(i, j)
            * ((hc(i, j - 1u) * pres[l] - hc(i, j) * pres[k]) / P.dx - ground[k].z * (pres[l] + pres[k]));
        v.y += P.dt * g;
    }
    pred[k] = v;
}

// 6: the vertical velocity: from the pressure where it acts, from the
// constraint elsewhere.
@compute @workgroup_size(8, 8)
fn correct_vertical(@builtin(global_invocation_id) id: vec3<u32>) {
    let i = id.x;
    let j = id.y;
    if i >= P.nx || j >= P.nz || P.dispersive == 0u {
        return;
    }
    let k = ix(i, j);
    let h = hc(i, j);
    var c = cell[k];
    if is_active(i, j) {
        c.x += P.dt * 2.0 * pres[k] / h;
    } else if h > P.dry {
        c.x = -0.5 * constraint(i, j, 0.0);
    } else {
        c.x = 0.0;
    }
    cell[k] = c;
}

// 7: upwind face depths for the corrected velocities.
@compute @workgroup_size(8, 8)
fn depths_new(@builtin(global_invocation_id) id: vec3<u32>) {
    let i = id.x;
    let j = id.y;
    if i > P.nx || j > P.nz {
        return;
    }
    let k = ix(i, j);
    var d = depth[k];
    d.x = 0.0;
    d.y = 0.0;
    if j < P.nz {
        d.x = face_depth_x(i, j, pred[k].x);
    }
    if i < P.nx {
        d.y = face_depth_z(i, j, pred[k].y);
    }
    depth[k] = d;
}

// 8: a cell cannot give more water than it holds: the factor on its
// outgoing faces (kept in `mat`, which the pressure solve is done with).
@compute @workgroup_size(8, 8)
fn limit(@builtin(global_invocation_id) id: vec3<u32>) {
    let i = id.x;
    let j = id.y;
    if i > P.nx || j > P.nz {
        return;
    }
    let k = ix(i, j);
    var scale = 1.0;
    if i < P.nx && j < P.nz {
        let fr = ix(i + 1u, j);
        let ft = ix(i, j + 1u);
        let out = max(depth[fr].x * pred[fr].x, 0.0) + max(-depth[k].x * pred[k].x, 0.0)
            + max(depth[ft].y * pred[ft].y, 0.0) + max(-depth[k].y * pred[k].y, 0.0);
        let has = hc(i, j) * P.dx / P.dt;
        if out > has && out > 0.0 {
            scale = has / out;
        }
    }
    mat[k] = vec4<f32>(scale, 0.0, 0.0, 0.0);
}

// The limited velocity of x-face (i, j): scaled by the cell it leaves.
fn limited_u(i: u32, j: u32) -> f32 {
    let u = pred[ix(i, j)].x;
    if u > 0.0 && i > 0u {
        return u * mat[ix(i - 1u, j)].x;
    }
    if u < 0.0 && i < P.nx {
        return u * mat[ix(i, j)].x;
    }
    return u;
}

fn limited_v(i: u32, j: u32) -> f32 {
    let v = pred[ix(i, j)].y;
    if v > 0.0 && j > 0u {
        return v * mat[ix(i, j - 1u)].x;
    }
    if v < 0.0 && j < P.nz {
        return v * mat[ix(i, j)].x;
    }
    return v;
}

fn was_breaking(i: u32, j: u32) -> bool {
    return depth[ix(i, j)].z > 0.5;
}

// 9: volume, and the breaking flags from how fast the surface rises.
@compute @workgroup_size(8, 8)
fn continuity(@builtin(global_invocation_id) id: vec3<u32>) {
    let i = id.x;
    let j = id.y;
    if i >= P.nx || j >= P.nz {
        return;
    }
    let k = ix(i, j);
    let fr = ix(i + 1u, j);
    let ft = ix(i, j + 1u);
    let div = depth[fr].x * limited_u(i + 1u, j) - depth[k].x * limited_u(i, j)
        + depth[ft].y * limited_v(i, j + 1u) - depth[k].y * limited_v(i, j);
    let bed = ground[k].x;
    let source = ground[k].w;
    let zn = max(state[k].x - P.dt * div / P.dx + P.dt * source, bed);
    var strength = 0.0;
    if P.dispersive != 0u {
        let h = zn - bed;
        if h > P.min_depth {
            let rate = (zn - state[k].x) / P.dt - source;
            let crit = sqrt(P.gravity * h);
            let near = was_breaking(i, j)
                || (i > 0u && was_breaking(i - 1u, j))
                || (i + 1u < P.nx && was_breaking(i + 1u, j))
                || (j > 0u && was_breaking(i, j - 1u))
                || (j + 1u < P.nz && was_breaking(i, j + 1u));
            if rate > P.alpha * crit || (near && rate > P.beta * crit) {
                strength = rate / crit;
            }
        }
    }
    var c = cell[k];
    c.y = zn;
    c.w = strength;
    cell[k] = c;
}

// 10: the new state.
@compute @workgroup_size(8, 8)
fn commit(@builtin(global_invocation_id) id: vec3<u32>) {
    let i = id.x;
    let j = id.y;
    if i > P.nx || j > P.nz {
        return;
    }
    let k = ix(i, j);
    var s = vec4<f32>(0.0);
    var d = depth[k];
    if j < P.nz {
        s.y = limited_u(i, j);
    }
    if i < P.nx {
        s.z = limited_v(i, j);
    }
    if i < P.nx && j < P.nz {
        s.x = cell[k].y;
        s.w = cell[k].x;
        if P.dispersive != 0u {
            d.z = select(0.0, 1.0, cell[k].w > 0.0);
        }
    }
    state[k] = s;
    depth[k] = d;
}
