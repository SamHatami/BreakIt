#!/usr/bin/env python3
"""Generates the source recordings for the structure sounds with Stable Audio
Open and writes them into crates/breakit-audio/samples/<timbre>/.

    python3 tools/generate_samples.py [--model small|1.0] [--takes 3] [timbre ...]

Needs the `stable-audio-tools` package, access to huggingface.co, and a
Hugging Face token in HF_TOKEN from an account that accepted the model's
licence (Stability AI Community License). Runs on a CPU: the Small model
takes seconds per clip, the 1.0 model many minutes.

Each clip is mixed to mono, cut at the first and last sound, limited to the
length wanted, peak-normalised, faded out and written as 16-bit WAV. Clips
that came out silent are skipped. The simulation decides when and how the
clips play (see docs/sound-physics.md); they only supply timbre, so each
prompt asks for one clean, dry event.
"""

import argparse
import os
import sys
import wave

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "crates", "breakit-audio", "samples")

DRY = "isolated sound effect, dry, close microphone, no reverb, no music, no voices"

# folder -> (prompt, seconds kept)
PROMPTS = {
    "steel_snap": ("thick steel bar snapping in tension, one sharp heavy metallic bang with a short low ring", 2.0),
    "steel_groan": ("large steel beam bending and buckling slowly, deep metal groan and creak under strain", 6.0),
    "steel_knock": ("steel bolt slipping in a joint under load, single dull heavy metallic knock", 1.0),
    "steel_tear": ("welded steel joint tearing apart, harsh ripping metal crack", 2.0),
    "steel_drop_heavy": ("heavy steel beam dropped on a concrete floor, one deep clang and rattle", 3.0),
    "steel_drop_light": ("short steel rod dropped on a concrete floor, bounce and settle", 2.0),
    "timber_creak": ("thick wooden beam creaking slowly under heavy load, low dry wood creak", 6.0),
    "timber_crack": ("wooden beam starting to split, a few separate dry wood cracks", 3.0),
    "timber_break": ("large timber beam snapping and splintering, loud wood crack with tearing fibres", 3.0),
    "timber_crush": ("wooden beam being crushed, dull crunching of wood fibres", 3.0),
    "timber_drop": ("heavy wooden plank dropped on a concrete floor, one dull knock and rattle", 2.0),
    "concrete_crack": ("concrete beam cracking under load, a few separate sharp dry stone cracks", 3.0),
    "concrete_break": ("reinforced concrete beam breaking, deep crack, crumbling and falling grit", 4.0),
    "concrete_crush": ("concrete being crushed slowly, grinding and crumbling stone", 5.0),
    "rebar_snap": ("steel reinforcement bar snapping inside concrete, sharp metallic ping with crumbling", 2.0),
    "rubble": ("concrete rubble and chunks falling on a concrete floor", 3.0),
    "carbon_snap": ("carbon fibre tube snapping, very sharp dry crack", 1.0),
    "carbon_crackle": ("carbon fibre laminate delaminating, fast dry crackling and ticking", 3.0),
    "carbon_drop": ("carbon fibre tube dropped on a concrete floor, light hollow clatter", 2.0),
}

MODELS = {
    "small": ("stabilityai/stable-audio-open-small", dict(steps=8, cfg_scale=1.0, sampler_type="pingpong")),
    "1.0": ("stabilityai/stable-audio-open-1.0", dict(steps=60, cfg_scale=7.0, sampler_type="dpmpp-3m-sde", sigma_min=0.3, sigma_max=500)),
}


def tidy(audio, rate, seconds):
    """Mono float array -> trimmed, normalised, faded clip, or None if silent."""
    peak = np.abs(audio).max()
    if peak < 1e-3:
        return None
    loud = np.nonzero(np.abs(audio) > 0.02 * peak)[0]
    start = max(int(loud[0]) - int(0.002 * rate), 0)
    audio = audio[start : min(int(loud[-1]) + int(0.05 * rate), start + int(seconds * rate))]
    if len(audio) < 0.05 * rate:
        return None
    fade = min(int(0.03 * rate), len(audio) // 4)
    audio = audio.copy()
    audio[-fade:] *= np.linspace(1.0, 0.0, fade)
    return audio / np.abs(audio).max() * 0.95


def write(path, audio, rate):
    with wave.open(path, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(rate)
        w.writeframes((audio * 32767.0).astype("<i2").tobytes())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", choices=MODELS, default="small")
    ap.add_argument("--takes", type=int, default=3)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--out", default=OUT)
    ap.add_argument("timbres", nargs="*")
    args = ap.parse_args()

    import torch
    from stable_audio_tools import get_pretrained_model
    from stable_audio_tools.inference.generation import generate_diffusion_cond

    name, settings = MODELS[args.model]
    model, config = get_pretrained_model(name)
    model = model.to("cpu").eval()
    rate, size = config["sample_rate"], config["sample_size"]
    window = size / rate

    for folder in args.timbres or PROMPTS:
        prompt, seconds = PROMPTS[folder]
        os.makedirs(os.path.join(args.out, folder), exist_ok=True)
        kept = 0
        for take in range(args.takes):
            conditioning = [{"prompt": f"{prompt}, {DRY}", "seconds_start": 0, "seconds_total": min(seconds + 1.0, window)}]
            with torch.no_grad():
                out = generate_diffusion_cond(
                    model,
                    conditioning=conditioning,
                    sample_size=size,
                    seed=args.seed + 1000 * take + (hash(folder) % 997),
                    device="cpu",
                    **settings,
                )
            audio = tidy(out[0].mean(dim=0).float().cpu().numpy(), rate, seconds)
            if audio is None:
                print(f"{folder} take {take}: silent, skipped", file=sys.stderr)
                continue
            path = os.path.join(args.out, folder, f"{kept:02}.wav")
            write(path, audio, rate)
            kept += 1
            print(f"{folder}/{kept - 1:02}.wav  {len(audio) / rate:.2f} s")


if __name__ == "__main__":
    main()
