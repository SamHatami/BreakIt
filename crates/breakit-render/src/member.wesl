// BreakIt member shader.
//
// Vertex stage: bends the shared member template (position = (s, y, z), s in
// 0..1 along the member) along the member's cubic Hermite curve, creases it
// at plastic hinges and clamps it to its side of the fracture surface. The
// CPU reference is `breakit_mesh::pose::MemberPose::deform` and
// `breakit_mesh::cut::cut_offset`; keep them in step.
//
// Fragment stage: the material look (steel, timber, concrete, composite) and
// its damage (flaking paint, splits, cracks, spalling, whitening), all
// procedural, so there are no textures to author or keep in sync.

import bevy_pbr::render::{
    mesh_functions,
    view_transformations::position_world_to_clip,
};

@if(PREPASS_PIPELINE)
import bevy_pbr::prepass::io::{Vertex, VertexOutput, decompress_vertex};
@else
import bevy_pbr::render::forward_io::{Vertex, VertexOutput, FragmentOutput, decompress_vertex};

@if(!PREPASS_PIPELINE)
import bevy_pbr::render::{
    pbr_fragment::pbr_input_from_standard_material,
    pbr_functions::{apply_pbr_lighting, main_pass_post_lighting_processing},
};

struct Member {
    pa: vec4<f32>,      // xyz end a, w hinge stub length at a
    pb: vec4<f32>,      // xyz end b, w hinge stub length at b
    qa: vec4<f32>,      // member frame at a (xyzw)
    qb: vec4<f32>,
    ta: vec4<f32>,      // xyz elastic tangent at a, w hinge damage at a
    tb: vec4<f32>,
    damage0: vec4<f32>, // utilisation, crack a, crack b, crack along span
    damage1: vec4<f32>, // crush a, crush b, delamination, yielded
    info: vec4<f32>,    // rest length, seed, material kind, flags
    clip: vec4<f32>,    // s window lo, hi; fracture face at lo, at hi
    size: vec4<f32>,    // section depth, width, seconds since break
}

@group(constants::MATERIAL_BIND_GROUP) @binding(100)
var<storage, read> members: array<Member>;

const KIND_STEEL: u32 = 0u;
const KIND_TIMBER: u32 = 1u;
const KIND_CONCRETE: u32 = 2u;
const KIND_RC: u32 = 3u;
const KIND_COMPOSITE: u32 = 4u;

const FLAG_HIDDEN: u32 = 1u;
const FLAG_STRESS_VIEW: u32 = 2u;
const FLAG_HIGHLIGHT: u32 = 4u;
const FLAG_REBAR: u32 = 8u;

// ---------------------------------------------------------------- noise

fn hash2(p: vec2<f32>) -> f32 {
    var p3 = fract(vec3<f32>(p.x, p.y, p.x) * 0.1031);
    p3 += dot(p3, p3.yzx + 33.33);
    return fract((p3.x + p3.y) * p3.z);
}

fn vnoise(p: vec2<f32>) -> f32 {
    let i = floor(p);
    let f = p - i;
    let u = f * f * (3.0 - 2.0 * f);
    let a = hash2(i);
    let b = hash2(i + vec2<f32>(1.0, 0.0));
    let c = hash2(i + vec2<f32>(0.0, 1.0));
    let d = hash2(i + vec2<f32>(1.0, 1.0));
    return a + (b - a) * u.x + (c - a) * u.y + (a - b - c + d) * u.x * u.y;
}

fn fbm(p: vec2<f32>) -> f32 {
    return 0.5 * vnoise(p) + 0.3 * vnoise(p * 2.13 + 7.1) + 0.2 * vnoise(p * 4.31 + 3.7);
}

// Distance to the nearest cell border of a jittered grid: the crack network.
// Returns (border distance in cell units, cell hash).
fn cell_edges(p: vec2<f32>) -> vec2<f32> {
    let i = floor(p);
    let f = p - i;
    var f1 = 8.0;
    var f2 = 8.0;
    var id = 0.0;
    for (var y = -1; y <= 1; y++) {
        for (var x = -1; x <= 1; x++) {
            let g = vec2<f32>(f32(x), f32(y));
            let c = i + g;
            let o = vec2<f32>(hash2(c), hash2(c + 17.3));
            let d = length(g + o - f);
            if d < f1 {
                f2 = f1;
                f1 = d;
                id = hash2(c + 5.1);
            } else if d < f2 {
                f2 = d;
            }
        }
    }
    return vec2<f32>(f2 - f1, id);
}

// ------------------------------------------------------- fracture surface

// Mirrors breakit_mesh::cut::cut_offset. Metres along the member.
fn cut_offset(kind: u32, yz: vec2<f32>, depth: f32, seed: f32) -> f32 {
    let d = max(depth, 1e-4);
    let q = yz / d + vec2<f32>(seed * 37.0);
    if kind == KIND_STEEL {
        let tilt = vec2<f32>(hash2(vec2<f32>(seed * 91.0)), hash2(vec2<f32>(seed * 53.0))) - 0.5;
        return d * (0.5 * dot(tilt, yz / d) + 0.12 * (vnoise(q * 9.0) - 0.5));
    } else if kind == KIND_TIMBER {
        let cell = floor(q * vec2<f32>(5.0, 9.0));
        let h = hash2(cell);
        return d * (1.7 * (h - 0.5) * (0.4 + 0.6 * h) + 0.25 * (vnoise(q * 3.0) - 0.5));
    } else if kind == KIND_COMPOSITE {
        let ply = floor(q.x * 7.0);
        return d * (1.3 * (hash2(vec2<f32>(ply, 7.0)) - 0.5) + 0.2 * (vnoise(q * 14.0) - 0.5));
    }
    return d * (0.45 * (vnoise(q * 3.5) - 0.5) + 0.18 * (vnoise(q * 11.0) - 0.5));
}

// ------------------------------------------------------------ deformation

fn qrot(q: vec4<f32>, v: vec3<f32>) -> vec3<f32> {
    let t = 2.0 * cross(q.xyz, v);
    return v + q.w * t + cross(q.xyz, t);
}

// Rotate `v` by the shortest rotation taking unit `a` to unit `b`.
fn arc_rot(a: vec3<f32>, b: vec3<f32>, v: vec3<f32>) -> vec3<f32> {
    let k = cross(a, b);
    let c = dot(a, b);
    return v * c + cross(k, v) + k * (dot(k, v) / max(1.0 + c, 1e-4));
}

struct Bent {
    position: vec3<f32>,
    normal: vec3<f32>,
    s: f32,
    // Section point after necking / splaying, metres.
    yz: vec2<f32>,
}

fn bend(m: Member, local: vec3<f32>, normal_in: vec3<f32>) -> Bent {
    let l0 = max(m.info.x, 1e-6);
    let depth = m.size.x;
    let kind = u32(m.info.z);
    let flags = u32(m.info.w);
    var yz = local.yz;
    var s = local.x;

    // Fracture window. Both halves of a broken member share one surface.
    var d_frac = 1e9;
    if m.clip.z + m.clip.w > 0.0 {
        let cut = cut_offset(kind, yz, depth, m.info.y) / l0;
        let lo = m.clip.x + m.clip.z * cut;
        let hi = m.clip.y + m.clip.w * cut;
        s = clamp(s, lo, hi);
        if m.clip.z > 0.5 { d_frac = min(d_frac, (s - lo) * l0); }
        if m.clip.w > 0.5 { d_frac = min(d_frac, (hi - s) * l0); }
    } else {
        s = clamp(s, m.clip.x, m.clip.y);
    }
    if d_frac < 1e8 && (flags & FLAG_REBAR) == 0u {
        if kind == KIND_STEEL {
            // Necking: the section draws in towards a ductile tear.
            let t = 1.0 - min(d_frac / (0.7 * depth), 1.0);
            yz *= 1.0 - 0.32 * t * t;
        } else if kind == KIND_TIMBER {
            // Splinters lift away from the section.
            let t = 1.0 - min(d_frac / (0.9 * depth), 1.0);
            let h = hash2(floor((yz / depth + m.info.y * 37.0) * vec2<f32>(5.0, 9.0)));
            yz *= 1.0 + 0.22 * t * t * h;
        } else if kind == KIND_COMPOSITE {
            // Brooming: the plies splay apart.
            let t = 1.0 - min(d_frac / (1.4 * depth), 1.0);
            let ply = floor((yz.x / depth + m.info.y * 37.0) * 7.0);
            yz.x = yz.x * (1.0 + 0.55 * t * t) + depth * 0.22 * t * t * (hash2(vec2<f32>(ply, 3.0)) - 0.5);
            yz.y *= 1.0 + 0.12 * t * t;
        }
    }

    // Centreline: straight hinge stubs that follow the nodes, cubic Hermite
    // between them.
    let ta_node = qrot(m.qa, vec3<f32>(1.0, 0.0, 0.0));
    let tb_node = qrot(m.qb, vec3<f32>(1.0, 0.0, 0.0));
    let sa = m.pa.w / l0;
    let sb = m.pb.w / l0;
    var c: vec3<f32>;
    var t: vec3<f32>;
    if s < sa {
        c = m.pa.xyz + ta_node * (s * l0);
        t = ta_node;
    } else if s > 1.0 - sb {
        c = m.pb.xyz - tb_node * ((1.0 - s) * l0);
        t = tb_node;
    } else {
        let u = clamp((s - sa) / max(1.0 - sa - sb, 1e-6), 0.0, 1.0);
        let p0 = m.pa.xyz + ta_node * m.pa.w;
        let p1 = m.pb.xyz - tb_node * m.pb.w;
        let lc = length(p1 - p0);
        let m0 = m.ta.xyz * lc;
        let m1 = m.tb.xyz * lc;
        let u2 = u * u;
        let u3 = u2 * u;
        c = (2.0 * u3 - 3.0 * u2 + 1.0) * p0 + (u3 - 2.0 * u2 + u) * m0
            + (-2.0 * u3 + 3.0 * u2) * p1 + (u3 - u2) * m1;
        let d = (6.0 * u2 - 6.0 * u) * p0 + (3.0 * u2 - 4.0 * u + 1.0) * m0
            + (-6.0 * u2 + 6.0 * u) * p1 + (3.0 * u2 - 2.0 * u) * m1;
        let dl = length(d);
        if dl > 1e-6 { t = d / dl; } else { t = ta_node; }
    }

    // Section frame: end frames blended, turned onto the tangent.
    var qb = m.qb;
    if dot(m.qa, qb) < 0.0 { qb = -qb; }
    let q = normalize(mix(m.qa, qb, s));
    let x = qrot(q, vec3<f32>(1.0, 0.0, 0.0));

    var out: Bent;
    out.position = c + arc_rot(x, t, qrot(q, vec3<f32>(0.0, yz.x, yz.y)));
    out.normal = arc_rot(x, t, qrot(q, normal_in));
    out.s = s;
    out.yz = yz;
    return out;
}

@vertex
fn vertex(vertex_in: Vertex) -> VertexOutput {
    var out: VertexOutput;
    let vertex = decompress_vertex(vertex_in, vertex_in.instance_index);
    let m = members[mesh_functions::get_tag(vertex_in.instance_index)];

@if(PREPASS_PIPELINE && NORMAL_PREPASS_OR_DEFERRED_PREPASS && VERTEX_NORMALS)
    let normal_in = vertex.normal;
@if(PREPASS_PIPELINE && !(NORMAL_PREPASS_OR_DEFERRED_PREPASS && VERTEX_NORMALS))
    let normal_in = vec3<f32>(0.0, 1.0, 0.0);
@if(!PREPASS_PIPELINE)
    let normal_in = vertex.normal;

    let bent = bend(m, vertex.position, normal_in);
    out.world_position = vec4<f32>(bent.position, 1.0);
    out.position = position_world_to_clip(bent.position);
    if (u32(m.info.w) & FLAG_HIDDEN) != 0u {
        // Outside the clip volume: every triangle of a hidden member is culled.
        out.position = vec4<f32>(2.0, 2.0, 2.0, 1.0);
    }

@if(PREPASS_PIPELINE) {
@if(UNCLIPPED_DEPTH_ORTHO_EMULATION) {
    out.unclipped_depth = out.position.z;
    out.position.z = min(out.position.z, 1.0);
}
@if(NORMAL_PREPASS_OR_DEFERRED_PREPASS)
    out.world_normal = bent.normal;
@if(MOTION_VECTOR_PREPASS)
    out.previous_world_position = out.world_position;
} @else {
    out.world_normal = bent.normal;
}

    // uv.x carries the clamped s; uv.y the perimeter coordinate (negative on
    // end caps). uv_b carries the section point in metres.
@if(VERTEX_UVS_A)
    out.uv = vec2<f32>(bent.s, vertex.uv.y);
@if(VERTEX_UVS_B)
    out.uv_b = vertex.position.yz;
@if(VERTEX_OUTPUT_INSTANCE_INDEX)
    out.instance_index = vertex_in.instance_index;
@if(VISIBILITY_RANGE_DITHER)
    out.visibility_range_dither = 0;

    return out;
}

// --------------------------------------------------------------- surface

struct Surface {
    color: vec3<f32>,
    roughness: f32,
    metallic: f32,
    // Relative surface height in metres, for bump shading.
    height: f32,
    emissive: vec3<f32>,
}

fn heat(u_in: f32) -> vec3<f32> {
    let u = max(u_in, 0.0);
    if u > 1.0 { return vec3<f32>(1.0, 0.05, 0.75); }
    let c0 = vec3<f32>(0.25, 0.42, 0.72);
    let c1 = vec3<f32>(0.15, 0.75, 0.3);
    let c2 = vec3<f32>(1.0, 0.82, 0.1);
    let c3 = vec3<f32>(1.0, 0.1, 0.05);
    if u < 0.3 { return mix(c0, c1, u / 0.3); }
    if u < 0.7 { return mix(c1, c2, (u - 0.3) / 0.4); }
    return mix(c2, c3, (u - 0.7) / 0.3);
}

@if(!PREPASS_PIPELINE)
fn surface(m: Member, s: f32, perim: f32, yz: vec2<f32>) -> Surface {
    let l0 = max(m.info.x, 1e-6);
    let depth = max(m.size.x, 1e-4);
    let width = max(m.size.y, 1e-4);
    let kind = u32(m.info.z);
    let flags = u32(m.info.w);
    let seed = m.info.y;
    let x = s * l0;
    let is_cap = perim < -0.5;
    // Surface coordinates in metres: along the member and around the
    // section on the sides, the section plane on caps.
    var w = vec2<f32>(x, perim * 2.0 * (depth + width));
    if is_cap { w = yz; }
    w += vec2<f32>(seed * 13.0, seed * 29.0);

    // Local damage: end values fade into the span over a few depths.
    let xa = x;
    let xb = l0 - x;
    let fa = exp(-xa / (1.6 * depth));
    let fb = exp(-xb / (1.6 * depth));
    let hinge = max(m.ta.w * exp(-xa / (1.2 * depth)), m.tb.w * exp(-xb / (1.2 * depth)));
    let crack = max(m.damage0.w, max(m.damage0.y * fa, m.damage0.z * fb));
    let crush = max(m.damage1.x * fa, m.damage1.y * fb);
    let delam = m.damage1.z;
    let util = m.damage0.x;

    // Distance to a fracture face along the member, and whether this
    // fragment is on the face itself.
    var d_frac = 1e9;
    if m.clip.z + m.clip.w > 0.0 {
        let cut = cut_offset(kind, yz, depth, seed) / l0;
        if m.clip.z > 0.5 { d_frac = min(d_frac, (s - (m.clip.x + cut)) * l0); }
        if m.clip.w > 0.5 { d_frac = min(d_frac, ((m.clip.y + cut) - s) * l0); }
    }
    let on_face = d_frac < 0.004 + 0.02 * depth;
    let age = m.size.z;

    var o: Surface;
    o.height = 0.0;
    o.emissive = vec3<f32>(0.0);

    if (flags & FLAG_REBAR) != 0u {
        // Ribbed reinforcing bar, a little rusty.
        let rib = smoothstep(0.55, 0.9, sin(x * 6.2832 / (1.4 * depth) + perim * 6.2832) * 0.5 + 0.5);
        let rust = smoothstep(0.45, 0.75, fbm(w * 90.0));
        o.color = mix(vec3<f32>(0.12, 0.11, 0.11), vec3<f32>(0.22, 0.1, 0.05), rust * 0.7);
        o.roughness = 0.62;
        o.metallic = 0.85 - 0.5 * rust;
        o.height = rib * 0.12 * depth;
    } else if kind == KIND_STEEL {
        // Painted structural steel.
        let tone = fbm(w * 5.0);
        var paint = vec3<f32>(0.09, 0.16, 0.23) * (0.85 + 0.3 * tone);
        let rust = smoothstep(0.66, 0.8, fbm(w * 11.0 + 3.0)) * 0.6;
        paint = mix(paint, vec3<f32>(0.33, 0.16, 0.07), rust);
        o.color = paint;
        o.roughness = 0.5 + 0.3 * rust;
        o.metallic = 0.1;
        // Yielding: paint and mill scale flake off along slip bands.
        let yielded = max(hinge, m.damage1.w * smoothstep(0.85, 1.05, util) * 0.22);
        if yielded > 0.001 {
            let band = sin((w.x + w.y) * 55.0 / max(depth * 6.0, 0.3) + fbm(w * 14.0) * 5.0) * 0.5 + 0.5;
            let flake = cell_edges(w * 26.0 / max(depth * 5.0, 0.25));
            let bare = smoothstep(1.0 - yielded * 1.25, 1.0 - yielded * 0.75, 0.55 * flake.y + 0.45 * band);
            o.color = mix(o.color, vec3<f32>(0.62, 0.63, 0.66), bare);
            o.metallic = mix(o.metallic, 1.0, bare);
            o.roughness = mix(o.roughness, 0.38, bare);
            o.height -= bare * 0.0015;
        }
        if on_face {
            // Fresh tear: bright, granular, glowing for a moment.
            let grain = hash2(floor(yz * 900.0));
            o.color = vec3<f32>(0.55 + 0.25 * grain);
            o.metallic = 1.0;
            o.roughness = 0.45 + 0.3 * grain;
            o.height = grain * 0.002;
            o.emissive = vec3<f32>(1.0, 0.32, 0.05) * 4.0 * exp(-age * 1.8);
        } else if d_frac < 0.8 * depth {
            // Necked zone: stretched bare metal with heat tint.
            let t = 1.0 - d_frac / (0.8 * depth);
            o.color = mix(o.color, vec3<f32>(0.5, 0.48, 0.55), t * t);
            o.metallic = mix(o.metallic, 1.0, t);
            o.emissive = vec3<f32>(1.0, 0.25, 0.03) * 1.5 * t * t * exp(-age * 1.8);
        }
    } else if kind == KIND_TIMBER {
        var v = w.y;
        var along = w.x;
        if is_cap {
            // End grain: growth rings around an off-centre pith.
            let pith = vec2<f32>(-1.7 * depth, 0.3 * width) + seed * depth;
            let r = length(yz - pith);
            let ring = sin(r * 260.0 + fbm(yz * 30.0) * 3.0) * 0.5 + 0.5;
            o.color = mix(vec3<f32>(0.66, 0.5, 0.31), vec3<f32>(0.47, 0.32, 0.18), ring * 0.8);
            o.height = -ring * 0.0006;
            v = yz.y;
            along = yz.x * 0.2;
        } else {
            let warp = fbm(vec2<f32>(along * 1.3, v * 9.0)) * 0.035;
            let ring = sin((v + warp) * 230.0) * 0.5 + 0.5;
            let streak = fbm(vec2<f32>(along * 3.0, v * 160.0));
            o.color = mix(vec3<f32>(0.68, 0.5, 0.3), vec3<f32>(0.46, 0.3, 0.16), 0.65 * ring + 0.25 * streak);
            o.color *= 0.9 + 0.2 * fbm(w * 2.0);
            o.height = -ring * 0.0008;
        }
        o.roughness = 0.82;
        o.metallic = 0.0;
        // Splits run along the grain and open up with damage.
        let dmg = max(crack, crush);
        if dmg > 0.001 {
            let lane = v * 22.0 / max(depth * 5.0, 0.3) + 0.35 * vnoise(vec2<f32>(along * 1.5, v * 8.0));
            let lane_id = floor(lane);
            let lh = hash2(vec2<f32>(lane_id, 4.0 + seed * 50.0));
            let reach = vnoise(vec2<f32>(along * 0.9 / max(depth * 5.0, 0.3), lane_id * 3.1));
            let open = smoothstep(1.0 - dmg * 0.8, 1.0 - dmg * 0.5, 0.45 * lh + 0.55 * reach);
            let line = 1.0 - smoothstep(0.015, 0.03 + 0.1 * open * dmg, abs(fract(lane) - 0.5));
            let split = line * open;
            o.color = mix(o.color, vec3<f32>(0.1, 0.06, 0.03), split);
            o.height -= split * 0.006;
        }
        if on_face || d_frac < 0.0 {
            // Fresh, pale, fibrous wood.
            let fibre = hash2(floor(yz * vec2<f32>(260.0, 700.0)));
            o.color = mix(vec3<f32>(0.88, 0.74, 0.52), vec3<f32>(0.7, 0.54, 0.34), fibre);
            o.roughness = 0.95;
            o.height = fibre * 0.003;
        } else if d_frac < depth {
            // Splinters: freshly exposed sides.
            let t = 1.0 - d_frac / depth;
            let fibre = fbm(vec2<f32>(along * 6.0, v * 300.0));
            o.color = mix(o.color, mix(vec3<f32>(0.86, 0.72, 0.5), vec3<f32>(0.66, 0.5, 0.3), fibre), smoothstep(0.0, 0.5, t));
        }
    } else if kind == KIND_COMPOSITE {
        // Carbon twill under clear coat.
        let tow = 0.012;
        let t = w / tow;
        let ti = floor(t);
        let tf = fract(t);
        let over = (i32(ti.x) + i32(ti.y) * 3 + 4000) % 4 < 2;
        var strand = 0.0;
        if over { strand = sin(tf.y * 3.1416) * (0.75 + 0.25 * sin(tf.x * 40.0)); }
        else { strand = sin(tf.x * 3.1416) * (0.75 + 0.25 * sin(tf.y * 40.0)); }
        var shade = 0.035;
        if over { shade = 0.07; }
        o.color = vec3<f32>(shade + 0.05 * strand) * vec3<f32>(0.9, 0.95, 1.05);
        o.roughness = 0.22;
        o.metallic = 0.15;
        o.height = strand * 0.0004;
        // Matrix cracking and delamination: stress whitening, then split plies.
        let dmg = max(crack, delam);
        if dmg > 0.001 {
            let patchy = fbm(w * 9.0 / max(depth * 5.0, 0.3));
            let white = smoothstep(1.0 - dmg * 0.75, 1.1 - dmg * 0.5, patchy);
            o.color = mix(o.color, vec3<f32>(0.42, 0.43, 0.46), white * 0.5);
            o.roughness = mix(o.roughness, 0.7, white);
        }
        if delam > 0.001 && !is_cap {
            let ply = (yz.x / depth + 0.5) * 7.0;
            let split = (1.0 - smoothstep(0.02, 0.06, abs(fract(ply) - 0.5))) *
                smoothstep(1.0 - delam * 1.4, 1.0 - delam * 0.5, hash2(vec2<f32>(floor(ply), floor(x * 2.0 / depth) + seed * 9.0)));
            o.color = mix(o.color, vec3<f32>(0.75, 0.75, 0.78), split * 0.7);
            o.height -= split * 0.004;
        }
        if on_face || d_frac < 1.2 * depth {
            // Broomed end: dull, layered, fibrous.
            let tt = 1.0 - clamp(d_frac / (1.2 * depth), 0.0, 1.0);
            let ply = floor((yz.x / depth + seed * 37.0) * 7.0);
            let fibre = hash2(vec2<f32>(floor(yz.y * 1500.0), ply));
            let raw = vec3<f32>(0.04 + 0.1 * fibre) + 0.03 * hash2(vec2<f32>(ply, 1.0));
            o.color = mix(o.color, raw, tt);
            o.roughness = mix(o.roughness, 0.9, tt);
            o.height += fibre * 0.002 * tt;
        }
    } else {
        // Concrete.
        let tone = fbm(w * 4.0);
        let speck = hash2(floor(w * 260.0));
        o.color = vec3<f32>(0.5, 0.49, 0.46) * (0.84 + 0.26 * tone) * (0.94 + 0.12 * speck);
        let pore = smoothstep(0.78, 0.86, vnoise(w * 70.0));
        o.color *= 1.0 - 0.45 * pore;
        o.height = -pore * 0.002 + tone * 0.0006;
        // Formwork board lines.
        if !is_cap {
            let board = abs(fract(w.x / 0.6) - 0.5);
            o.color *= 1.0 - 0.12 * (1.0 - smoothstep(0.0, 0.012, board));
        }
        o.roughness = 0.93;
        o.metallic = 0.0;
        // Cracks: a network that opens up patch by patch, mostly across the
        // member like flexural cracks.
        if crack > 0.001 {
            let k = 5.0 / max(depth, 0.08);
            let wob = vec2<f32>(vnoise(w * k * 1.3), vnoise(w * k * 1.3 + 9.0)) * 0.35;
            let e = cell_edges(vec2<f32>(w.x * 0.75, w.y * 0.3) * k + wob);
            let grow = vnoise(w * k * 0.35 + 11.0);
            let open = smoothstep(1.0 - crack * 0.95, 1.0 - crack * 0.5, grow);
            let width_c = (0.008 + 0.04 * crack) * open;
            let line = (1.0 - smoothstep(width_c * 0.4, width_c, e.x)) * open;
            o.color = mix(o.color, vec3<f32>(0.07, 0.065, 0.06), line);
            o.height -= line * 0.008;
        }
        // Crushing: the cover spalls off in patches and shows the aggregate.
        if crush > 0.001 {
            let k = 9.0 / max(depth, 0.08);
            let spall = smoothstep(1.0 - crush * 1.2, 1.05 - crush * 0.7, fbm(w * k * 0.4 + 5.0));
            let stone = cell_edges(w * 60.0);
            let agg = mix(vec3<f32>(0.42, 0.4, 0.36), vec3<f32>(0.55, 0.47, 0.38), stone.y);
            o.color = mix(o.color, agg * (0.6 + 0.5 * smoothstep(0.0, 0.2, stone.x)), spall);
            o.height -= spall * (0.006 + 0.004 * stone.y);
        }
        if on_face || d_frac < 0.0 {
            // Broken face: stones in a pale matrix.
            let stone = cell_edges(yz * 55.0 + seed * 9.0);
            let matrix = vec3<f32>(0.68, 0.67, 0.63);
            let agg = mix(vec3<f32>(0.38, 0.36, 0.33), vec3<f32>(0.6, 0.5, 0.38), stone.y);
            o.color = mix(matrix, agg, smoothstep(0.08, 0.2, stone.x) * step(0.35, stone.y));
            o.height = stone.y * 0.004;
            o.roughness = 0.97;
        }
    }

    if (flags & FLAG_STRESS_VIEW) != 0u && (flags & FLAG_REBAR) == 0u {
        let lum = dot(o.color, vec3<f32>(0.3, 0.6, 0.1));
        o.color = heat(util) * (0.55 + 0.9 * lum);
        o.metallic *= 0.3;
    }
    if (flags & FLAG_HIGHLIGHT) != 0u {
        o.emissive += vec3<f32>(0.25, 0.3, 0.1);
    }
    return o;
}

@if(!PREPASS_PIPELINE)
@fragment
fn fragment(in: VertexOutput, @builtin(front_facing) is_front: bool) -> FragmentOutput {
    var pbr_input = pbr_input_from_standard_material(in, is_front);
    let m = members[mesh_functions::get_tag(in.instance_index)];

    let o = surface(m, in.uv.x, in.uv.y, in.uv_b);
    pbr_input.material.base_color = vec4<f32>(o.color, 1.0);
    pbr_input.material.perceptual_roughness = o.roughness;
    pbr_input.material.metallic = o.metallic;
    pbr_input.material.emissive = vec4<f32>(o.emissive, 1.0);

    // Bump shading from the procedural height (Mikkelsen's surface gradient).
    let p = in.world_position.xyz;
    let n = pbr_input.N;
    let dpx = dpdx(p);
    let dpy = dpdy(p);
    let dhx = dpdx(o.height);
    let dhy = dpdy(o.height);
    let r1 = cross(dpy, n);
    let r2 = cross(n, dpx);
    let det = dot(dpx, r1);
    let grad = sign(det) * (dhx * r1 + dhy * r2);
    pbr_input.N = normalize(abs(det) * n - 1.5 * grad);

    var out: FragmentOutput;
    out.color = apply_pbr_lighting(pbr_input);
    out.color = main_pass_post_lighting_processing(pbr_input, out.color);
    return out;
}
