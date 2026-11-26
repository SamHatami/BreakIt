// BreakIt water surface.
//
// The mesh is the simulated surface (see `surface.rs`); each vertex carries
// the water depth, the flow velocity, the foam cover, how hard the water is
// breaking and the curvature of the surface. This shader turns that into
// what water looks like, on top of `StandardMaterial` lighting:
//
// * refraction and depth colour: the material is specularly transmissive,
//   and the thickness the light crosses is the depth along the refracted
//   ray, so the bed shifts and tints with depth and viewing angle;
// * reflection: the hall's environment map and lamps, by Fresnel;
// * ripples: small moving slopes added to the simulated normal, carried
//   with the flow, stronger where the water is fast or breaking;
// * caustics: light focused on the bed by the surface above it, from the
//   surface curvature (large) and from the ripples (fine);
// * foam: a broken, drifting cover where the simulation has put foam, and
//   a lace of it along a moving shoreline.

import bevy_pbr::render::{
    forward_io::{VertexOutput, FragmentOutput},
    mesh_view_bindings::view,
    pbr_fragment::pbr_input_from_standard_material,
    pbr_functions::{apply_pbr_lighting, main_pass_post_lighting_processing},
};

struct WaterLook {
    // x: ripples and foam detail (0 or 1), y: caustics (0 or 1),
    // z: refraction (1) or plain alpha blending (0), w: water time, s.
    params: vec4<f32>,
    // rgb: colour of deep water. a: its opacity when blending, or how
    // brightly the body of the water glows with scattered light when
    // refracting.
    tint: vec4<f32>,
}

@group(constants::MATERIAL_BIND_GROUP) @binding(100)
var<uniform> look: WaterLook;

fn hash2(p: vec2<f32>) -> f32 {
    var p3 = fract(vec3<f32>(p.x, p.y, p.x) * 0.1031);
    p3 += dot(p3, p3.yzx + 33.33);
    return fract((p3.x + p3.y) * p3.z);
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
    for (var i = 0; i < 3; i++) {
        s += a * vnoise(p);
        p = p * 2.07 + vec2<f32>(13.7, 5.3);
        a *= 0.5;
    }
    return s / 0.875;
}

// Slope of a rippled height field at p: a handful of short wave trains
// crossing each other, each a smooth sinusoid, so the reflection stays
// clean at any angle. `rough` (0..1) brings in the shorter ones.
fn ripple_slope(p: vec2<f32>, t: f32, rough: f32) -> vec2<f32> {
    var slope = vec2<f32>(0.0);
    // (direction angle, wavelength m, relative height)
    let trains = array<vec3<f32>, 7>(
        vec3<f32>(0.3, 1.9, 1.0),
        vec3<f32>(1.9, 1.3, 0.8),
        vec3<f32>(3.7, 0.83, 0.6),
        vec3<f32>(5.1, 0.57, 0.45),
        vec3<f32>(0.9, 0.39, 0.3),
        vec3<f32>(2.8, 0.27, 0.22),
        vec3<f32>(4.4, 0.19, 0.16),
    );
    for (var i = 0; i < 7; i++) {
        let w = trains[i];
        let dir = vec2<f32>(cos(w.x), sin(w.x));
        let k = 6.2832 / w.y;
        // Short gravity-capillary ripples: speed grows slowly with length.
        let omega = sqrt(9.81 * k) * 0.35;
        let weight = select(1.0, rough, i >= 3);
        slope += dir * (w.z * weight * cos(k * dot(dir, p) - omega * t + f32(i) * 1.7));
    }
    return slope;
}

// Bright focused lines on the bed: ridges where two drifting noise fields
// cancel.
fn caustic_lines(q: vec2<f32>, t: f32) -> f32 {
    let a = vnoise(q * 1.9 + vec2<f32>(t * 0.27, t * 0.19));
    let b = vnoise(q * 2.7 + vec2<f32>(-t * 0.21, t * 0.33) + 4.7);
    let r = 1.0 - abs(a + b - 1.0);
    return r * r * r * r;
}

@fragment
fn fragment(
    in: VertexOutput,
    @builtin(front_facing) is_front: bool,
) -> FragmentOutput {
    let cover_in = in.uv.x;
    let depth = in.uv.y;
    // Under the bed: this part of the lattice is not water.
    if depth <= 0.0 {
        discard;
    }
    var pbr_input = pbr_input_from_standard_material(in, is_front);
    let t = look.params.w;
    let p = in.world_position.xz;
    let flow = in.color.xy;
    let breaking = in.color.z;
    let curvature = in.color.w;
    let speed = length(flow);
    let detail = look.params.x;

    // Things carried by the flow are drawn twice, half a cycle apart, and
    // cross-faded, so the pattern travels without stretching.
    let cycle = fract(vec2<f32>(t * 0.4, t * 0.4 + 0.5));
    let blend = 1.0 - abs(2.0 * cycle.x - 1.0);
    let carry0 = p - flow * (cycle.x - 0.5) * 1.5;
    let carry1 = p - flow * (cycle.y - 0.5) * 1.5 + vec2<f32>(3.1, 7.7);

    // Detail fades before it gets smaller than a pixel.
    let dist = length(view.world_position - in.world_position.xyz);
    let resolved = 1.0 / (1.0 + (dist / 45.0) * (dist / 45.0));

    var n = pbr_input.N;
    var slope = vec2<f32>(0.0);
    if detail > 0.5 {
        let rough = clamp(0.25 + 0.5 * speed + 2.0 * breaking, 0.0, 1.0);
        let s = mix(ripple_slope(carry1, t, rough), ripple_slope(carry0, t, rough), blend);
        let strength = 0.010 + 0.016 * min(speed, 3.0) + 0.08 * min(breaking, 1.5);
        slope = s * strength * resolved;
        n = normalize(n + vec3<f32>(-slope.x, 0.0, -slope.y));
    }
    if dot(n, pbr_input.V) < 0.0 {
        n = reflect(n, pbr_input.V);
    }

    // Path of the refracted ray down to the bed.
    let cos_v = clamp(dot(n, pbr_input.V), 0.05, 1.0);
    let sin_t = sqrt(1.0 - cos_v * cos_v) / 1.333;
    let cos_t = sqrt(1.0 - sin_t * sin_t);
    let path = depth / cos_t;

    // Foam: the simulated cover breaks up into a drifting pattern.
    var foam = 0.0;
    {
        var pattern = 0.5;
        if detail > 0.5 {
            let f0 = fbm(carry0 * 3.3);
            let f1 = fbm(carry1 * 3.3);
            let cells = mix(f1, f0, blend);
            let fine = mix(vnoise(carry1 * 8.0), vnoise(carry0 * 8.0), blend);
            pattern = clamp(0.8 * (1.0 - abs(2.0 * cells - 1.0)) + 0.2 * fine, 0.0, 1.0);
        }
        let cover = clamp(cover_in, 0.0, 1.3);
        foam = smoothstep(1.05 - cover, 1.45 - cover, pattern + 0.25 * cover);
        // A lace of foam where thin water moves fast over the bed.
        let shore = (1.0 - smoothstep(0.02, 0.12, depth)) * smoothstep(0.4, 1.4, speed);
        foam = max(foam, 0.8 * shore * smoothstep(0.3, 0.75, pattern));
        // A film of water a few millimetres deep is hardly there.
        foam = clamp(foam, 0.0, 1.0) * smoothstep(0.0, 0.02, depth);
        // Even a full cover is bubbles: never a flat white sheet.
        foam *= 0.72 + 0.28 * pattern;
    }

    // Caustics tint what is seen through the surface.
    var lens = 1.0;
    if look.params.y > 0.5 {
        let refracted = refract(-pbr_input.V, n, 1.0 / 1.333);
        let bed_point = p + refracted.xz * path;
        // Sharp at about a metre of water, soft and weak in the deep.
        let focus = smoothstep(0.03, 0.7, depth) * (1.0 - 0.6 * smoothstep(1.5, 4.5, depth));
        let lively = 0.35 + 0.5 * min(speed, 1.5) + 2.0 * min(abs(curvature) * depth, 0.4);
        let lines = caustic_lines(bed_point, t) * resolved;
        // A crest is a converging lens, a trough a diverging one.
        let swell = clamp(1.0 - 0.6 * curvature * min(depth, 2.5), 0.7, 1.6);
        lens = swell * (0.92 + 1.1 * lines * focus * lively);
    }

    let foam_color = vec3<f32>(0.93, 0.95, 0.95);
    if look.params.z > 0.5 {
        pbr_input.material.base_color = vec4<f32>(mix(vec3<f32>(lens), foam_color, foam), 1.0);
        pbr_input.material.specular_transmission = 1.0 - foam;
        pbr_input.material.thickness = path;
        // Light scattered back out of the body of the water: what gives
        // deep water its own colour where it reflects something dark.
        let body = (1.0 - exp(-path / 3.0)) * (1.0 - foam);
        pbr_input.material.emissive = vec4<f32>(look.tint.rgb * look.tint.a * body, 1.0);
    } else {
        // No refraction: see-through by depth, mirror-like at a glance.
        let fresnel = 0.02 + 0.98 * pow(1.0 - cos_v, 5.0);
        let murk = 1.0 - exp(-path / 1.4);
        let alpha = mix(mix(0.12, look.tint.a, murk), 0.97, fresnel);
        pbr_input.material.base_color = vec4<f32>(
            mix(look.tint.rgb, foam_color, foam),
            mix(alpha, 1.0, foam),
        );
    }
    pbr_input.material.perceptual_roughness = mix(pbr_input.material.perceptual_roughness, 0.75, foam);
    pbr_input.N = n;

    var out: FragmentOutput;
    out.color = apply_pbr_lighting(pbr_input);
    out.color = main_pass_post_lighting_processing(pbr_input, out.color);
    return out;
}
