#!/usr/bin/env python3
"""Fetches the CC0 recordings BreakIt layers into its structure sounds and
converts them into crates/breakit-audio/samples/ (mono, 32 kHz, 16-bit WAV,
leading silence trimmed, peak-normalised, faded out) plus
crates/breakit-audio/src/sample_index.rs and samples/SOURCES.md.

    python3 tools/cc0_samples.py

Needs curl, ffmpeg and the py7zr package (pip install py7zr), and network
access to opengameart.org. Every source here is CC0 (public domain); the
licence of each page was checked when it was added. Re-running regenerates
the same files.
"""

import os
import shutil
import subprocess
import sys
import tempfile
import zipfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "crates", "breakit-audio", "samples")
INDEX = os.path.join(ROOT, "crates", "breakit-audio", "src", "sample_index.rs")
RATE = 32000

# (page, author, archive or file URL)
SOURCES = {
    "wood": ("https://opengameart.org/content/35-wooden-crackshitsdestructions", "qubodup",
             "https://opengameart.org/sites/default/files/independent_nu_ljudbank-wood_crack_hit_destruction.7z"),
    "crunch": ("https://opengameart.org/content/5-break-crunch-impacts", "qubodup",
               "https://opengameart.org/sites/default/files/independent_nu_ljudbank-break_crunch_impact.7z"),
    "bfh": ("https://opengameart.org/content/75-cc0-breaking-falling-hit-sfx", "rubberduck",
            "https://opengameart.org/sites/default/files/sfx_breaking_and_falling.zip"),
    "bmac_bong": ("https://opengameart.org/content/metal-impact-sounds", "BMacZero",
                  "https://opengameart.org/sites/default/files/bong1.wav"),
    "bmac_thud2": ("https://opengameart.org/content/metal-impact-sounds", "BMacZero",
                   "https://opengameart.org/sites/default/files/thud2.wav"),
    "bmac_thud3": ("https://opengameart.org/content/metal-impact-sounds", "BMacZero",
                   "https://opengameart.org/sites/default/files/thud3.wav"),
    "bmac_clink1": ("https://opengameart.org/content/metal-impact-sounds", "BMacZero",
                    "https://opengameart.org/sites/default/files/clink1_0.wav"),
    "bmac_clink2": ("https://opengameart.org/content/metal-impact-sounds", "BMacZero",
                    "https://opengameart.org/sites/default/files/clink2.wav"),
    "bmac_clink3": ("https://opengameart.org/content/metal-impact-sounds", "BMacZero",
                    "https://opengameart.org/sites/default/files/clink3.wav"),
}

W = "wood_impact/"
C = "impcrunch/"
B = ""

# group -> [(source, path inside archive or "", max seconds)]
CLIPS = {
    "wood_crack": [("wood", W + f"crack{n:02}.mp3.flac", 1.6) for n in (1, 2, 4, 5, 7, 9)],
    "wood_break": [("bfh", B + f"bfh1_wood_breaking_{n:02}.ogg", 0.6) for n in (1, 2, 3, 4)],
    "wood_hit": [("bfh", B + f"bfh1_wood_hit_{n:02}.ogg", 0.5) for n in (1, 2, 3)]
    + [("wood", W + f"impactwood{n:02}.mp3.flac", 0.9) for n in (7, 11, 14, 23)],
    "metal_hit": [("bfh", B + f"bfh1_metal_hit_{n:02}.ogg", 0.6) for n in (1, 2, 3, 4, 5, 6)],
    "metal_small": [(f"bmac_clink{n}", "", 0.4) for n in (1, 2, 3)],
    "metal_heavy": [("bmac_bong", "", 1.8), ("bmac_thud2", "", 0.6), ("bmac_thud3", "", 0.5)],
    "metal_fall": [("bfh", B + f"bfh1_metal_falling_{n:02}.ogg", 1.0) for n in (1, 2, 3, 4, 5)],
    "rock_break": [("bfh", B + f"bfh1_rock_breaking_{n:02}.ogg", 1.0) for n in (1, 2, 3)]
    + [("bfh", B + f"bfh1_breaking_{n:02}.ogg", 1.0) for n in (1, 2, 3)],
    "crunch": [("crunch", C + f"impactcrunch{n:02}.mp3.flac", 1.0) for n in (1, 2, 3, 4, 5)],
    "rock_fall": [("bfh", B + f"bfh1_rock_falling_{n:02}.ogg", 0.8) for n in (1, 2, 3, 5, 6, 9)]
    + [("bfh", B + "bfh1_rock_hit_01.ogg", 0.3)],
}


def fetch(tmp, key):
    page, _, url = SOURCES[key]
    dest = os.path.join(tmp, key)
    if os.path.exists(dest):
        return dest
    os.makedirs(dest)
    file = os.path.join(dest, url.rsplit("/", 1)[1])
    subprocess.run(["curl", "-sSfL", "-m", "300", "-o", file, url], check=True)
    if file.endswith(".zip"):
        zipfile.ZipFile(file).extractall(dest)
    elif file.endswith(".7z"):
        import py7zr
        py7zr.SevenZipFile(file).extractall(dest)
    return dest


def peak_db(src):
    out = subprocess.run(["ffmpeg", "-hide_banner", "-i", src, "-af", "volumedetect", "-f", "null", "-"],
                         capture_output=True, text=True).stderr
    return float(out.split("max_volume:")[1].split("dB")[0])


def convert(src, dst, max_s):
    fade = min(0.15, max_s / 4)
    tmp = dst + ".tmp.wav"
    af = (
        "aformat=channel_layouts=mono,"
        "silenceremove=start_periods=1:start_threshold=-50dB,"
        f"atrim=0:{max_s},afade=t=out:st={max_s - fade:.3f}:d={fade:.3f}"
    )
    subprocess.run(["ffmpeg", "-loglevel", "error", "-y", "-i", src, "-af", af,
                    "-ar", str(RATE), "-ac", "1", "-c:a", "pcm_f32le", tmp], check=True)
    # Peak-normalise to -1 dBFS.
    gain = -1.0 - peak_db(tmp)
    subprocess.run(["ffmpeg", "-loglevel", "error", "-y", "-i", tmp, "-af", f"volume={gain:.2f}dB",
                    "-c:a", "pcm_s16le", dst], check=True)
    os.remove(tmp)


def main():
    if os.path.isdir(OUT):
        shutil.rmtree(OUT)
    os.makedirs(OUT)
    index = []
    used = {}
    with tempfile.TemporaryDirectory() as tmp:
        for group, clips in CLIPS.items():
            os.makedirs(os.path.join(OUT, group))
            for i, (key, inner, max_s) in enumerate(clips):
                d = fetch(tmp, key)
                src = os.path.join(d, inner) if inner else os.path.join(d, SOURCES[key][2].rsplit("/", 1)[1])
                name = f"{group}/{i:02}.wav"
                convert(src, os.path.join(OUT, name), max_s)
                index.append((group, name))
                used.setdefault(SOURCES[key][0], (SOURCES[key][1], []))[1].append(
                    f"{name} <- {inner or SOURCES[key][2].rsplit('/', 1)[1]}")

    with open(os.path.join(OUT, "SOURCES.md"), "w") as f:
        f.write("# Recorded samples\n\nAll files here are derived from CC0 (public domain) recordings on "
                "OpenGameArt: converted to mono 32 kHz, trimmed and normalised by `tools/cc0_samples.py`. "
                "No attribution is required; it is given anyway.\n\n")
        for page, (author, files) in used.items():
            f.write(f"## {page}\n\nBy {author}, CC0.\n\n")
            for line in files:
                f.write(f"- `{line}`\n")
            f.write("\n")

    groups = list(CLIPS)
    with open(INDEX, "w") as f:
        f.write("// Generated by tools/cc0_samples.py. Do not edit.\n\n")
        f.write("/// Sample groups, in the order of [`GROUPS`].\n")
        f.write("#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]\npub enum Group {\n")
        for g in groups:
            f.write(f"    {''.join(p.title() for p in g.split('_'))},\n")
        f.write("}\n\n")
        f.write(f"pub const GROUPS: [Group; {len(groups)}] = [\n")
        for g in groups:
            f.write(f"    Group::{''.join(p.title() for p in g.split('_'))},\n")
        f.write("];\n\n")
        f.write("/// 16-bit mono WAV files per group.\npub fn files(group: Group) -> &'static [&'static [u8]] {\n    match group {\n")
        for g in groups:
            names = [n for (gg, n) in index if gg == g]
            f.write(f"        Group::{''.join(p.title() for p in g.split('_'))} => &[\n")
            for n in names:
                f.write(f"            include_bytes!(\"../samples/{n}\"),\n")
            f.write("        ],\n")
        f.write("    }\n}\n")
    total = sum(os.path.getsize(os.path.join(dp, fn)) for dp, _, fns in os.walk(OUT) for fn in fns)
    print(f"{len(index)} clips, {total / 1e6:.2f} MB in {OUT}")


if __name__ == "__main__":
    sys.exit(main())
