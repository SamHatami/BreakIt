# Playtesting BreakIt

Thanks for testing. BreakIt is early: expect rough edges, missing art and
things that change week to week. What we need most is what confused you,
what felt wrong, and anything that crashed or ran slowly.

## Get a build

Builds are on the [Releases page](https://github.com/SamHatami/BreakIt/releases).

- **Tagged releases** (`v0.1.0`, `v0.2.0`, ...) are the ones to test against.
- **nightly** is rebuilt from `main` after every merge. Newer, less tested.

Download the zip for your system, unpack it anywhere you can write to
(Desktop or Downloads, not Program Files), and run:

- Windows: `BreakIt.exe`. The build is not signed, so Windows SmartScreen may
  say "Windows protected your PC": click **More info**, then **Run anyway**.
- Linux: `./BreakIt` (needs Vulkan drivers; under X11 also `libxkbcommon-x11`).

You need a GPU with Vulkan, DirectX 12 or Metal support. Integrated graphics
from the last few years should work; tell us if yours does not.

## Controls

The controls card in the game is always the current list. At the time of
writing:

| Input | Does |
| --- | --- |
| Right mouse drag | orbit |
| Middle mouse drag, or Shift + right drag | pan |
| Wheel | zoom |
| F | frame the selection, or everything |
| Left mouse | use the tool: place a node, chain beams, select |
| B, S, X, L | draw, support, delete, load (sandbox) |
| 1, 2 | steel, timber (also changes the selected beam) |
| Del | delete the selected beam or hovered node |
| Ctrl+Z, Ctrl+Y | undo, redo |
| Tab, Q / E | switch drawing plane, move it down / up |
| D | exaggerate deflection |
| C | clear back to the sheet's start |
| T | next play table |
| Space | run the test; during a test, pause |
| `,` `.` | test slower / faster |
| H | utilisation heatmap |
| F1, Esc | brief, pause or back |
| F3 | hide or show the frame-time read-out |

The read-out in the top right shows the build's version and frame times.
Leave it on while you test; it makes screenshots much more useful.

## What to test

Play however you like, but these are the questions we have right now:

1. **First minutes.** Without reading anything, could you work out how to
   build something and run a test? Where did you get stuck?
2. **Does the physics feel right?** Do structures bend, yield and break the
   way you expect? Anything that holds when it should obviously fail, or
   fails for no visible reason?
3. **Is a failure readable?** After a collapse, can you tell which member
   went first and why, and what to change?
4. **Performance.** Note the fps in the read-out when building, during a
   test, and during a big collapse. Anything under 30 fps, or stutters,
   please report with your GPU.
5. **Crashes and freezes.** Always worth a report, with the log file.

## Report feedback

Open an issue with the **Playtest report** template:
<https://github.com/SamHatami/BreakIt/issues/new?template=playtest.yml>

One issue per problem is easiest to act on. Screenshots and short clips
help a lot (Win+Shift+S for a screenshot, Win+Alt+R records with the Xbox
Game Bar).

### Log file

Every run writes `logs/breakit.log` next to `BreakIt.exe`; the run before
it is kept as `logs/breakit.prev.log`. After a crash, start the game again
only after you copied the log, or attach `breakit.prev.log`. If the game
folder is not writable the log goes to `%TEMP%\BreakIt\logs` on Windows
(`/tmp/BreakIt/logs` on Linux). The log starts with the version and lists
your GPU, so please attach it to crash and performance reports.

For a performance report, start the game with `BREAKIT_PERF_LOG=1` set and
it writes the read-out to the log once a second. On Windows, in PowerShell:

```
$env:BREAKIT_PERF_LOG=1; .\BreakIt.exe
```

## Making a release (maintainers)

`.github/workflows/release.yml` does it. Every push to `main` refreshes the
`nightly` pre-release. For a numbered release, tag `main`:

```
git tag v0.1.0 && git push origin v0.1.0
```

Or, without pushing a tag: Actions tab, **Release**, **Run workflow** on
`main` with `release_tag` set to `v0.1.0`. The workflow then creates the tag.

The workflow builds Windows and Linux zips with `cargo build --profile dist`
(about 20 to 30 minutes on a cold cache) and publishes the release.
