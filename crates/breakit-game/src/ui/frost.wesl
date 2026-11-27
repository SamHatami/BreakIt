// A frosted UI panel (`frost.rs`): the blurred scene at this pixel,
// darkened by the theme's tint, cut to the node's rounded corners, with a
// hairline along the edge.

import bevy_render::view::View;
import bevy_ui_render::ui_vertex_output::UiVertexOutput;

@group(0) @binding(0) var<uniform> view: View;

struct Frost {
    tint: vec4<f32>,
    edge: vec4<f32>,
}

@group(1) @binding(0) var<uniform> frost: Frost;
@group(1) @binding(1) var blur: texture_2d<f32>;
@group(1) @binding(2) var blur_sampler: sampler;

// Signed distance to the node's rounded box, in px; `p` from its centre,
// y down. Radii: top left, top right, bottom right, bottom left.
fn rounded_box(p: vec2<f32>, half: vec2<f32>, rx: vec4<f32>, ry: vec4<f32>) -> f32 {
    var r = vec2<f32>(rx.x, ry.x);
    if p.x >= 0.0 && p.y < 0.0 {
        r = vec2<f32>(rx.y, ry.y);
    } else if p.x >= 0.0 && p.y >= 0.0 {
        r = vec2<f32>(rx.z, ry.z);
    } else if p.x < 0.0 && p.y >= 0.0 {
        r = vec2<f32>(rx.w, ry.w);
    }
    let rad = min(min(r.x, r.y), min(half.x, half.y));
    let q = abs(p) - half + vec2<f32>(rad);
    return length(max(q, vec2<f32>(0.0))) + min(max(q.x, q.y), 0.0) - rad;
}

@fragment
fn fragment(in: UiVertexOutput) -> @location(0) vec4<f32> {
    let screen = (in.position.xy - view.viewport.xy) / view.viewport.zw;
    let scene = textureSample(blur, blur_sampler, screen).rgb;
    // A faint sheen towards the top keeps large panels from looking flat.
    let sheen = 1.0 + 0.06 * (1.0 - in.uv.y);
    var c = mix(scene, frost.tint.rgb, frost.tint.a) * sheen;
    let p = (in.uv - vec2<f32>(0.5)) * in.size;
    let d = rounded_box(p, in.size * 0.5, in.border_radius_x, in.border_radius_y);
    let inside = clamp(0.5 - d, 0.0, 1.0);
    let edge = clamp(1.0 - abs(d + 0.75), 0.0, 1.0) * frost.edge.a;
    c = mix(c, frost.edge.rgb, edge);
    return vec4<f32>(c, inside);
}
