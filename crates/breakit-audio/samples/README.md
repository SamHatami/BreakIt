# Recordings

Source timbre for the structure sounds (see `src/timbre.rs` and `docs/sound-physics.md`). The simulation decides which event plays, when, how loud, at what pitch and for how long; these files only say what a crack or a clang sounds like.

One folder per kind of event. Put 16-bit WAV files (mono, any sample rate, one clean event each, dry, no silence in front) into the folder and rebuild: `build.rs` picks them up. While a folder is empty the closest free recording from `cc0/` stands in (see `Timbre::stand_in`).

| Folder | What is in it |
|---|---|
| `steel_snap` | thick steel bar snapping in tension |
| `steel_groan` | steel beam bending and buckling slowly (sustained) |
| `steel_knock` | bolt slipping in a joint, single knock |
| `steel_tear` | welded joint or bolt group tearing |
| `steel_drop_heavy` | heavy steel member landing on concrete |
| `steel_drop_light` | short steel piece landing on concrete |
| `timber_creak` | timber beam creaking under load (sustained) |
| `timber_crack` | single dry cracks of a beam starting to split |
| `timber_break` | beam snapping and splintering |
| `timber_crush` | wood fibres crushing |
| `timber_drop` | plank landing on concrete |
| `concrete_crack` | single cracks in concrete |
| `concrete_break` | concrete beam breaking and crumbling |
| `concrete_crush` | concrete grinding and crushing (sustained) |
| `rebar_snap` | reinforcement bar snapping inside concrete |
| `rubble` | rubble landing on concrete |
| `carbon_snap` | carbon fibre tube snapping |
| `carbon_crackle` | laminate delaminating, fast dry crackle |
| `carbon_drop` | carbon tube landing |

A first set generated with Stable Audio Open Small (`tools/generate_samples.py`) was tried on 2026-11-27 and sounded worse than the free recordings, so the folders are empty again and the stand-ins play. The clips are in git history (commit before this one) if they are wanted for comparison.

`cc0/` holds public-domain recordings from OpenGameArt; sources in `cc0/SOURCES.md`.
