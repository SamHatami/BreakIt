// Blur for the frosted UI panels (`frost.rs`): a 4x box downsample of the
// finished frame, then a separable Gaussian at quarter size.

import bevy_core_pipeline::fullscreen_vertex_shader::FullscreenVertexOutput;

@group(0) @binding(0) var src: texture_2d<f32>;
@group(0) @binding(1) var samp: sampler;

// Bright lamps would otherwise bloom through every panel.
fn tame(c: vec3<f32>) -> vec3<f32> {
    return min(c, vec3<f32>(2.0));
}

// Four bilinear taps one source texel off the centre of each 4x4 block
// average all sixteen pixels.
@fragment
fn down(in: FullscreenVertexOutput) -> @location(0) vec4<f32> {
    let t = 1.0 / vec2<f32>(textureDimensions(src));
    var c = tame(textureSample(src, samp, in.uv + t * vec2<f32>(-1.0, -1.0)).rgb);
    c += tame(textureSample(src, samp, in.uv + t * vec2<f32>(1.0, -1.0)).rgb);
    c += tame(textureSample(src, samp, in.uv + t * vec2<f32>(-1.0, 1.0)).rgb);
    c += tame(textureSample(src, samp, in.uv + t * vec2<f32>(1.0, 1.0)).rgb);
    return vec4<f32>(c * 0.25, 1.0);
}

// Nine-tap Gaussian folded into five bilinear taps, spread over twice the
// texel distance: about a 36 px radius at full size.
const SPREAD: f32 = 2.0;

fn gauss(uv: vec2<f32>, dir: vec2<f32>) -> vec4<f32> {
    let step = dir * SPREAD / vec2<f32>(textureDimensions(src));
    var c = textureSample(src, samp, uv).rgb * 0.2270270;
    c += textureSample(src, samp, uv + step * 1.3846154).rgb * 0.3162162;
    c += textureSample(src, samp, uv - step * 1.3846154).rgb * 0.3162162;
    c += textureSample(src, samp, uv + step * 3.2307692).rgb * 0.0702703;
    c += textureSample(src, samp, uv - step * 3.2307692).rgb * 0.0702703;
    return vec4<f32>(c, 1.0);
}

@fragment
fn blur_h(in: FullscreenVertexOutput) -> @location(0) vec4<f32> {
    return gauss(in.uv, vec2<f32>(1.0, 0.0));
}

@fragment
fn blur_v(in: FullscreenVertexOutput) -> @location(0) vec4<f32> {
    return gauss(in.uv, vec2<f32>(0.0, 1.0));
}
