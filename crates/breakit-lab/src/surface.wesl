// BreakIt lab surfaces.
//
// One fragment shader for everything in the hall and on the play table that
// is not a structural member: poured concrete, painted and galvanised steel,
// stainless, sheeting, plywood, modelling foam, the table top, glass and
// water. It is `StandardMaterial` lighting with a procedural surface in
// front: each kind varies its colour, roughness and normal from noise laid
// out in metres, so there are no textures and no UVs to keep in step with
// the generated meshes.
//
// Positions are projected along the dominant axis of the face normal. The
// scenery is boxes, bars and cylinders, so that is seamless where it matters.

import bevy_pbr::render::{
    forward_io::{VertexOutput, FragmentOutput},
    mesh_view_bindings::globals,
    pbr_fragment::pbr_input_from_standard_material,
    pbr_functions::{apply_pbr_lighting, main_pass_post_lighting_processing},
};

struct LabSurface {
    // xyz: world position of the surface origin; w: surface metres per
    // world unit (1/20 in the hall, 1 on the table).
    frame: vec4<f32>,
    // x: kind, y: detail (0 leaves the plain material), z: seed.
    params: vec4<f32>,
    // Water: x amplitude (m), y wavelength (m), z speed (m/s), w heading (rad).
    water: vec4<f32>,
}

@group(constants::MATERIAL_BIND_GROUP) @binding(100)
var<uniform> surf: LabSurface;

const KIND_CONCRETE: u32 = 1u;
const KIND_PAINT: u32 = 2u;
const KIND_GALVANISED: u32 = 3u;
const KIND_STAINLESS: u32 = 4u;
const KIND_SHEETING: u32 = 5u;
const KIND_FOAM: u32 = 6u;
const KIND_TABLE_TOP: u32 = 7u;
const KIND_TIMBER: u32 = 8u;
const KIND_GLASS: u32 = 9u;
const KIND_WATER: u32 = 10u;
const KIND_PLATE: u32 = 11u;

// ---------------------------------------------------------------- noise

fn hash2(p: vec2<f32>) -> f32 {
    var p3 = fract(vec3<f32>(p.x, p.y, p.x) * 0.1031);
    p3 += dot(p3, p3.yzx + 33.33);
    return fract((p3.x + p3.y) * p3.z);
}

fn hash22(p: vec2<f32>) -> vec2<f32> {
    var p3 = fract(vec3<f32>(p.x, p.y, p.x) * vec3<f32>(0.1031, 0.1030, 0.0973));
    p3 += dot(p3, p3.yzx + 33.33);
    return fract((p3.xx + p3.yz) * p3.zy);
}

fn vnoise(p: vec2<f32>) -> f32 {
    let i = floor(p);
    let f = fract(p);
    let u = f * f * (3.0 - 2.0 * f);
    return mix(
        mix(hash2(i), hash2(i + vec2<f32>(1.0, 0.0)), u.x),
        mix(hash2(i + vec2<f32>(0.0, 1.0)), hash2(i + vec2<f32>(1.0, 1.0)), u.x),
        u.y,
    );
}

fn fbm(p_in: vec2<f32>) -> f32 {
    var p = p_in;
    var a = 0.5;
    var s = 0.0;
    for (var i = 0; i < 4; i++) {
        s += a * vnoise(p);
        p = p * 2.03 + vec2<f32>(17.1, 9.2);
        a *= 0.5;
    }
    return s / 0.9375;
}

// x: distance to the nearest cell border, y: a random value per cell.
fn cells(p: vec2<f32>) -> vec2<f32> {
    let i = floor(p);
    let f = fract(p);
    var d1 = 8.0;
    var d2 = 8.0;
    var id = 0.0;
    for (var y = -1; y <= 1; y++) {
        for (var x = -1; x <= 1; x++) {
            let g = vec2<f32>(f32(x), f32(y));
            let o = hash22(i + g);
            let d = length(g + o - f);
            if d < d1 {
                d2 = d1;
                d1 = d;
                id = hash2(i + g + 7.0);
            } else if d < d2 {
                d2 = d;
            }
        }
    }
    return vec2<f32>(d2 - d1, id);
}

// 1 while a pattern of `freq` cycles per metre is resolved by the pixels,
// falling to 0 as it shrinks under them: fades detail out before it can
// shimmer.
fn resolved(uv: vec2<f32>, freq: f32) -> f32 {
    let w = max(length(dpdx(uv)), length(dpdy(uv)));
    return 1.0 - smoothstep(0.2, 0.6, w * freq);
}

struct Look {
    color: vec3<f32>,
    roughness: f32,
    metallic: f32,
    // Relief in surface metres, for bump shading.
    height: f32,
    alpha: f32,
}

// ------------------------------------------------------------- surfaces

fn concrete(base: vec3<f32>, uv: vec2<f32>, up: f32) -> Look {
    var o: Look;
    // Pour to pour and within a pour: slow clouds, then mottling.
    let cloud = fbm(uv * 0.22);
    let mottle = fbm(uv * 1.9 + 4.0);
    let fine = resolved(uv, 60.0);
    let speck = mix(0.5, hash2(floor(uv * 240.0)), fine);
    o.color = base * (0.84 + 0.2 * cloud + 0.14 * mottle) * (0.96 + 0.08 * speck);
    // Old spills and where things stood.
    let stain = smoothstep(0.58, 0.74, fbm(uv * 0.55 + 21.0)) * smoothstep(0.3, 0.6, mottle);
    o.color *= 1.0 - 0.2 * stain;
    // Fine dark pores.
    let pore = smoothstep(0.8, 0.9, vnoise(uv * 110.0)) * resolved(uv, 110.0);
    o.color *= 1.0 - 0.16 * pore;
    o.height = -0.0015 * pore + 0.0006 * mottle;
    // A power-floated floor is burnished in patches and takes a dull
    // reflection; walls and upstands are as struck.
    let burnish = smoothstep(0.35, 0.7, fbm(uv * 0.5 + 9.0)) * (1.0 - 0.7 * stain);
    o.roughness = mix(0.9, mix(0.56, 0.27, burnish), up);
    o.metallic = 0.0;
    o.alpha = 1.0;
    return o;
}

fn paint(base: vec3<f32>, uv: vec2<f32>, up: f32, side: f32, rough: f32, seed: f32) -> Look {
    var o: Look;
    let tone = fbm(uv * 1.3 + seed);
    o.color = base * (0.92 + 0.16 * tone);
    o.roughness = rough + 0.12 * (tone - 0.5);
    o.metallic = 0.0;
    // Sprayed enamel is never quite flat.
    o.height = 0.00012 * vnoise(uv * 170.0) * resolved(uv, 170.0);
    // Grime runs down vertical faces.
    let run = fbm(vec2<f32>(uv.x * 5.0 + seed, uv.y * 0.45));
    let grime = smoothstep(0.52, 0.82, run) * side;
    o.color *= 1.0 - 0.22 * grime;
    o.roughness += 0.15 * grime;
    // Dust settles on anything that faces up.
    let dust = up * (0.3 + 0.5 * fbm(uv * 2.6 + 11.0));
    o.color = mix(o.color, vec3<f32>(0.46, 0.44, 0.41), 0.4 * dust);
    o.roughness = mix(o.roughness, 0.85, 0.6 * dust);
    // Chips down to primer and a little rust, in scattered patches.
    let zone = smoothstep(0.66, 0.8, fbm(uv * 0.9 + 31.0 + seed));
    let c = cells(uv * 160.0);
    let chip = step(0.8, c.y) * smoothstep(0.02, 0.1, c.x) * zone * resolved(uv, 160.0);
    let rust = mix(vec3<f32>(0.42, 0.39, 0.36), vec3<f32>(0.33, 0.17, 0.08), step(0.9, c.y));
    o.color = mix(o.color, rust, chip);
    o.roughness = mix(o.roughness, 0.8, chip);
    o.height -= 0.0003 * chip;
    o.alpha = 1.0;
    return o;
}

fn galvanised(base: vec3<f32>, uv: vec2<f32>, metallic: f32, rough: f32) -> Look {
    var o: Look;
    // Zinc spangle: crystals that each catch the light differently.
    let fine = resolved(uv, 55.0);
    let c = cells(uv * 55.0);
    let spangle = mix(0.5, c.y, fine);
    let bloom = fbm(uv * 1.1 + 3.0);
    o.color = base * (0.86 + 0.24 * spangle) * (0.92 + 0.14 * bloom);
    o.roughness = rough + 0.22 * (spangle - 0.5) + 0.15 * smoothstep(0.55, 0.8, bloom);
    o.metallic = metallic;
    o.height = 0.0;
    o.alpha = 1.0;
    return o;
}

fn stainless(base: vec3<f32>, uv: vec2<f32>, rough: f32) -> Look {
    var o: Look;
    // Brushed round the vessel: long fine streaks.
    let fine = resolved(uv, 300.0);
    let streak = mix(0.5, fbm(vec2<f32>(uv.x * 2.5, uv.y * 300.0)), fine);
    let patchy = fbm(uv * 0.8 + 13.0);
    o.color = base * (0.9 + 0.12 * streak) * (0.94 + 0.1 * patchy);
    o.roughness = rough + 0.16 * streak + 0.1 * smoothstep(0.5, 0.8, patchy);
    o.metallic = 1.0;
    o.height = 0.00006 * streak;
    o.alpha = 1.0;
    return o;
}

fn sheeting(base: vec3<f32>, uv: vec2<f32>, side: f32) -> Look {
    var o: Look;
    // Sheets a metre wide, each a shade off its neighbour.
    let sheet = hash2(vec2<f32>(floor(uv.x), 3.0));
    let run = fbm(vec2<f32>(uv.x * 2.2, uv.y * 0.18));
    o.color = base * (0.94 + 0.08 * sheet) * (1.0 - 0.12 * smoothstep(0.5, 0.85, run) * side);
    o.color *= 0.95 + 0.1 * fbm(uv * 0.4);
    o.roughness = 0.62 + 0.15 * run;
    o.metallic = 0.0;
    o.height = 0.0;
    o.alpha = 1.0;
    return o;
}

fn foam(base: vec3<f32>, uv: vec2<f32>, side: f32) -> Look {
    var o: Look;
    // Closed-cell modelling foam: a fine sparkle of cells, and on cut
    // faces the faint drag lines of the hot wire.
    let fine = resolved(uv, 28.0);
    let c = cells(uv * 28.0);
    let cell = mix(0.5, c.y, fine);
    let wire = fbm(vec2<f32>(uv.x * 0.6, uv.y * 42.0)) * side * resolved(uv, 42.0);
    o.color = base * (0.97 + 0.04 * cell) * (0.97 + 0.05 * fbm(uv * 0.8)) * (1.0 - 0.06 * wire);
    o.roughness = 0.94;
    o.metallic = 0.0;
    o.height = (0.0015 * smoothstep(0.0, 0.25, c.x) * fine) - 0.002 * wire;
    o.alpha = 1.0;
    return o;
}

fn table_top(base: vec3<f32>, uv: vec2<f32>, up: f32) -> Look {
    var o: Look;
    // A dark cutting-mat surface: matte, a little worn where models have
    // been dragged across it.
    let wear = fbm(uv * 0.35 + 5.0);
    let fine = resolved(uv, 40.0);
    let grain = mix(0.5, vnoise(uv * 40.0), fine);
    o.color = base * (0.9 + 0.2 * wear) * (0.94 + 0.12 * grain);
    // Fine scratches in a few directions.
    var scratch = 0.0;
    for (var i = 0; i < 3; i++) {
        let a = 0.6 + 1.1 * f32(i);
        let d = vec2<f32>(cos(a), sin(a));
        let t = dot(uv, vec2<f32>(-d.y, d.x)) * 7.0 + fbm(uv * 0.5 + f32(i) * 9.0) * 3.0;
        let on = step(0.72, hash2(vec2<f32>(floor(t), f32(i))));
        scratch = max(scratch, on * (1.0 - smoothstep(0.0, 0.03, abs(fract(t) - 0.5))));
    }
    scratch *= resolved(uv, 120.0) * up * smoothstep(0.4, 0.7, wear);
    o.color += vec3<f32>(0.05) * scratch;
    o.roughness = mix(0.72, 0.5, smoothstep(0.45, 0.75, wear)) + 0.2 * scratch;
    o.metallic = 0.0;
    o.height = -0.0006 * scratch;
    o.alpha = 1.0;
    return o;
}

fn timber(base: vec3<f32>, uv: vec2<f32>) -> Look {
    var o: Look;
    // Rotary-cut face veneer: wide wandering figure along the board.
    let warp = fbm(vec2<f32>(uv.x * 0.7, uv.y * 3.0));
    let fine = resolved(uv, 70.0);
    let ring = mix(0.5, sin((uv.y + 0.12 * warp) * 150.0) * 0.5 + 0.5, fine);
    let streak = fbm(vec2<f32>(uv.x * 1.5, uv.y * 90.0));
    o.color = base * (0.86 + 0.18 * ring) * (0.9 + 0.2 * streak) * (0.92 + 0.14 * warp);
    o.roughness = 0.78 - 0.1 * ring;
    o.metallic = 0.0;
    o.height = -0.0004 * ring;
    o.alpha = 1.0;
    return o;
}

fn plate(base: vec3<f32>, uv: vec2<f32>) -> Look {
    var o: Look;
    // Hot-rolled plate: blue-grey mill scale, scuffed bright where loads
    // have been dragged over it.
    let scale = fbm(uv * 2.4);
    let scuff = smoothstep(0.55, 0.8, fbm(vec2<f32>(uv.x * 0.6, uv.y * 3.5) + 7.0));
    o.color = base * (0.8 + 0.3 * scale);
    o.color = mix(o.color, vec3<f32>(0.42, 0.43, 0.45), 0.5 * scuff);
    o.roughness = mix(0.62, 0.36, scuff) + 0.1 * (scale - 0.5);
    o.metallic = 0.85;
    o.height = 0.0;
    o.alpha = 1.0;
    return o;
}

// Height of the water surface at `p` (metres, horizontal), and its slope.
// A few trains of waves a little off the main heading, so the surface never
// looks like a single sine; the amplitude can be driven by a wave rig.
fn water_slope(p: vec2<f32>, t: f32) -> vec2<f32> {
    let amp = surf.water.x;
    let len = max(surf.water.y, 0.02);
    let speed = surf.water.z;
    var slope = vec2<f32>(0.0);
    for (var i = 0; i < 5; i++) {
        let fi = f32(i);
        let a = surf.water.w + (fi - 2.0) * 0.37 + 0.6 * sin(fi * 2.4);
        let d = vec2<f32>(cos(a), sin(a));
        let k = 6.2832 / (len * (1.0 - 0.17 * fi));
        let w = k * speed * (0.8 + 0.1 * fi);
        let h = amp * (0.5 / (1.0 + fi));
        slope += d * (h * k * cos(dot(d, p) * k - w * t + fi * 1.9));
    }
    // Wind ripple on top.
    let r = p * 9.0 + vec2<f32>(t * 0.35, t * 0.21);
    let e = 0.05;
    let n0 = fbm(r);
    slope += 0.012 * vec2<f32>(fbm(r + vec2<f32>(e, 0.0)) - n0, fbm(r + vec2<f32>(0.0, e)) - n0) / e
        * resolved(p, 9.0);
    return slope;
}

// ------------------------------------------------------------- fragment

@fragment
fn fragment(in: VertexOutput, @builtin(front_facing) is_front: bool) -> FragmentOutput {
    var pbr_input = pbr_input_from_standard_material(in, is_front);
    let kind = u32(surf.params.x);
    let seed = surf.params.z;

    if kind != 0u && surf.params.y > 0.5 {
        let p = (in.world_position.xyz - surf.frame.xyz) * surf.frame.w;
        let n = pbr_input.world_normal;
        let an = abs(n);
        var uv = p.xy;
        if an.y > 0.72 {
            uv = p.xz;
        } else if an.x > an.z {
            uv = p.zy;
        }
        uv += vec2<f32>(seed * 7.3, seed * 3.1);
        let up = smoothstep(0.6, 0.95, n.y);
        let side = 1.0 - smoothstep(0.3, 0.7, an.y);
        let base = pbr_input.material.base_color.rgb;
        let rough = pbr_input.material.perceptual_roughness;

        var o: Look;
        var bump = true;
        if kind == KIND_CONCRETE {
            o = concrete(base, uv, up);
        } else if kind == KIND_PAINT {
            o = paint(base, uv, up, side, rough, seed);
        } else if kind == KIND_GALVANISED {
            o = galvanised(base, uv, pbr_input.material.metallic, rough);
        } else if kind == KIND_STAINLESS {
            o = stainless(base, uv, rough);
        } else if kind == KIND_SHEETING {
            o = sheeting(base, uv, side);
        } else if kind == KIND_FOAM {
            o = foam(base, uv, side);
        } else if kind == KIND_TABLE_TOP {
            o = table_top(base, uv, up);
        } else if kind == KIND_TIMBER {
            o = timber(base, uv);
        } else if kind == KIND_PLATE {
            o = plate(base, uv);
        } else {
            bump = false;
            o.color = base;
            o.roughness = rough;
            o.metallic = pbr_input.material.metallic;
            o.alpha = pbr_input.material.base_color.a;
            // What a transparent surface shows is mostly its reflection:
            // faint head-on, strong at a glancing angle.
            var nn = pbr_input.N;
            if kind == KIND_WATER {
                let s = water_slope(p.xz, globals.time);
                nn = normalize(vec3<f32>(-s.x, 1.0, -s.y));
                if dot(nn, pbr_input.V) < 0.0 { nn = -nn; }
                // Deeper blue-green where the slope turns away.
                o.color = base * (0.85 + 0.6 * clamp(length(s), 0.0, 0.5));
            } else {
                // Glass: a film of dust and wipe marks dulls it in places.
                let film = fbm(vec2<f32>(uv.x * 1.2, uv.y * 0.5) + 2.0);
                o.roughness = rough + 0.25 * smoothstep(0.5, 0.8, film);
                o.alpha += 0.1 * smoothstep(0.5, 0.8, film);
            }
            let nv = clamp(dot(nn, pbr_input.V), 0.0, 1.0);
            let fresnel = 0.03 + 0.97 * pow(1.0 - nv, 5.0);
            o.alpha = mix(o.alpha, 0.96, fresnel);
            pbr_input.N = nn;
        }

        pbr_input.material.base_color = vec4<f32>(o.color, o.alpha);
        pbr_input.material.perceptual_roughness = clamp(o.roughness, 0.045, 1.0);
        pbr_input.material.metallic = o.metallic;

        if bump {
            // Bump shading from the procedural height (Mikkelsen's surface
            // gradient), with the height brought into world units.
            let h = o.height / surf.frame.w;
            let wp = in.world_position.xyz;
            let nn = pbr_input.N;
            let dpx = dpdx(wp);
            let dpy = dpdy(wp);
            let r1 = cross(dpy, nn);
            let r2 = cross(nn, dpx);
            let det = dot(dpx, r1);
            let grad = sign(det) * (dpdx(h) * r1 + dpdy(h) * r2);
            pbr_input.N = normalize(abs(det) * nn - grad);
        }
    }

    var out: FragmentOutput;
    out.color = apply_pbr_lighting(pbr_input);
    out.color = main_pass_post_lighting_processing(pbr_input, out.color);
    return out;
}
