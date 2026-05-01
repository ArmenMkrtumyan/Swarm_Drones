# Setup & usage

## Setup

```bash
cd ~/Desktop/Swarm_Drones/optimization

# First time only:
python3 -m venv .optim_env
source .optim_env/bin/activate
pip install -r requirements.txt

# Every session:
source .optim_env/bin/activate
```

Direct deps: `numpy`, `matplotlib`. The native `MacOSX` backend is preferred on macOS to avoid a known TkAgg / system-Tcl version crash.

### System dependencies for `--gui` on Linux / WSL

Matplotlib's `TkAgg` backend depends on two pieces that ship as system packages on Debian/Ubuntu/WSL, not as pip packages — so they can't go in `requirements.txt`. Without them, `--gui` silently falls back to headless. Install both:

```bash
sudo apt update
sudo apt install -y python3-tk python3-pil.imagetk
```

- `python3-tk` provides `tkinter` itself. Missing → `GUI backend 'TkAgg' unavailable (No module named 'tkinter')`.
- `python3-pil.imagetk` provides `PIL.ImageTk`, which matplotlib's `TkAgg` uses to draw figures into the Tk canvas. Missing → `cannot import name 'ImageTk' from 'PIL'`. (Pillow installed via pip does NOT include this — Tk integration needs the Tk dev headers at build time, which only the apt package provides.)

WSL2 on Windows 11 renders the resulting matplotlib window through WSLg automatically; no X server setup needed. If `$DISPLAY` is empty after install, run `wsl --shutdown` from PowerShell and reopen the WSL terminal.

If you're using a venv that wasn't created with `--system-site-packages`, the apt packages won't be visible — either recreate the venv with that flag or run from system python.

---

## Run modes

The demo runner (`tools/demo.py`) is the main entrypoint. Two run modes, two persistence modes, freely combined.

### Headless (default)

Renders frames to PNGs without opening a window. Output files **overwrite** on every run — they are scratch artifacts.

```bash
python tools/demo.py                                  # default
python tools/demo.py --headless                       # explicit (same thing)
python tools/demo.py --map maze --drones 4
```

Writes to `outputs/images/`:

- `demo_initial.png` — first frame
- `demo_final.png` — last frame
- `coverage_curve.png` — coverage % over time

### GUI (live animation)

Opens an interactive window. Drones move in real time; the coverage trail fills in with per-drone colors.

```bash
python tools/demo.py --gui
python tools/demo.py --gui --map maze --drones 4
```

### Persisting outputs with `--save`

By default, output files are scratch (overwritten next run). Add `--save` to keep them: outputs are renamed with a `SAVED_<tag>_` prefix so multiple runs accumulate side-by-side.

```bash
python tools/demo.py --save                           # SAVED_<timestamp>_*.png
python tools/demo.py --save baseline_random           # SAVED_baseline_random_*.png
python tools/demo.py --gui --save run3                # also writes SAVED_run3_animation.gif
```

Tag rules:

- `--save` (no value) → `SAVED_20260426_180012_demo_initial.png`
- `--save NAME` → `SAVED_NAME_demo_initial.png`

`--gui` and `--headless` are mutually exclusive; argparse rejects the combination.

### All flags

| Flag | Default | Notes |
|---|---|---|
| `--headless` / `--gui` | `--headless` | mutually exclusive |
| `--save [TAG]` | off (overwrite) | omit TAG for timestamp |
| `--map {random,maze}` | `random` | auto-generators |
| `--map-file PATH` | none | load `.npy` / `.txt` map |
| `--size N` | 55 | grid side length (in cells) |
| `--drones N` | 3 | swarm size |
| `--seed N` | 17 | reproducibility |

**There is intentionally no `--steps` flag and no step-count cap.** The simulation runs until one of two natural terminal conditions is reached:

- `env.is_done()` — 100 % coverage (mission success)
- `env.all_depleted()` — every drone hit its voltage cutoff (mission failure)

The all-depleted condition is **physics-bounded**: maximum sim time is `(initial_energy − cutoff_energy) / P_hover ≈ 121 kJ / 165 W ≈ 734 s ≈ 7340 steps`. Past that, every drone has hit cutoff and `is_terminal()` fires automatically — no artificial step cap needed.

**To compress wall-clock time on a long run, use the speed slider in the GUI window** (1× / 2× / 5× / 10× / 20×, bottom of the figure). It controls how many `env.step()` calls happen between renders — each step still uses the same `step_seconds = 0.1` and the same physics, so the simulation is bit-identical regardless of slider value. At 20× a 12-minute hover plays in ~36 s real time. The slider does **not** scale `step_seconds` itself; doing that would let drones travel further than `drone_radius` per step and break wall-collision detection.

(Changing the swarm config — more drones, larger sensor radius, smaller map — also shortens runs but gives you a different mission, not just a faster wall-clock. Pick the slider when you want the same mission but less waiting; pick a config change when you want a different scenario.)

For verification scripts (separate from demos), see [verification.md](verification.md).

---

## Creating maps

Two ways to feed a map into `tools/demo.py`.

### A — auto-generate (built into `tools/demo.py`)

- `--map random` — boundary walls + scattered interior obstacles, BFS-validated to keep the free space connected, retried on failure
- `--map maze` — recursive-backtracker perfect maze with 1-cell-wide corridors

No file is written; the grid is regenerated each run from `--seed`.

### B — hand-draw with the editor

```bash
python tools/editor.py --size 21
```

Controls:

- **left-drag** — paint walls
- **right-drag** — erase to free
- **s** — save (default path: `outputs/maps/custom_map.npy`)
- **c** — clear interior (keep boundary)
- **r** — reset to fresh boundary-walled grid
- **f** — fill all interior with walls
- **v** — toggle validation overlay
- **g** — toggle grid lines
- **q** — quit

Live validation overlays:

- **red** — free cells in a disconnected pocket (unreachable from main area)
- **orange** — free cells touching the outer border (boundary wall missing)

Edit an existing map and save to a new file:

```bash
python tools/editor.py --load outputs/maps/custom_map.npy --out outputs/maps/v2.npy
```

Then run the demo on it:

```bash
python tools/demo.py --map-file outputs/maps/custom_map.npy
```

---

## Outputs

Everything generated lives under `outputs/` (gitignored):

```
outputs/
├── images/   demo PNGs and animation GIFs
└── maps/     editor-saved .npy / .txt grids
```

Both directories are created on demand. Don't write generated files anywhere else — the project root stays for source only.
