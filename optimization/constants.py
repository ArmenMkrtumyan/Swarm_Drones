"""
Shared constants for the swarm coverage testbed.

Lives at the project root so any module (library or tool) can import from it
without circular dependencies. Things that belong here:
    - cross-file paths (so we don't compute the same Path() twice)
    - visual styling shared by visualize.py and editor.py
    - figure layout that should look the same in saved PNGs and the live window
    - CLI defaults that are NOT physical/algorithmic parameters

Things that DO NOT belong here (intentionally):
    - SimConfig / DroneConfig / BatteryConfig defaults — already centralized
      in dataclasses, meant to be overridden per-experiment
    - FREE / WALL cell IDs — canonical in maze.py
    - dimensional constants like 3.6 J per mAh·V — derived, not tunable
"""

from __future__ import annotations

from pathlib import Path

# ---------------------------------------------------------------------------
# Project paths
# ---------------------------------------------------------------------------
PROJECT_ROOT = Path(__file__).resolve().parent
OUTPUTS_DIR = PROJECT_ROOT / "outputs"
IMAGES_DIR = OUTPUTS_DIR / "images"
MAPS_DIR = OUTPUTS_DIR / "maps"

# ---------------------------------------------------------------------------
# Visual styling (used by visualize.py and editor.py)
# ---------------------------------------------------------------------------
WALL_COLOR = "#444"
FREE_COLOR = "white"

DISCONNECTED_OVERLAY_RGBA = (1.0, 0.0, 0.0, 0.55)
BREACH_OVERLAY_RGBA = (1.0, 0.55, 0.0, 0.65)

SENSOR_CIRCLE_ALPHA = 0.4

# Per-drone palette generation (visualize.init_drone_palette / drone_color).
# Hues are placed at 1/n spacing on the HSV wheel and the whole wheel is
# rotated by a random offset, so every drone is maximally far from every other
# in hue, and different runs draw different specific colors.
DRONE_PALETTE_SATURATION = 0.72   # vivid without being neon
DRONE_PALETTE_VALUE = 0.88        # bright enough to read as a paint overlay
# Mid-gray for depleted drones. Saturation 0 keeps it away from every palette
# color (which all have S = 0.72), so a dead drone is unambiguous regardless
# of which random rotation the run drew.
DEPLETED_DRONE_RGB = (0.5, 0.5, 0.5)

# Translucent paint for "first-visitor wins" cell coloring.
FIRST_VISITOR_ALPHA = 0.45

# Dark translucent overlay stacked on top of cells visited by ≥ 2 distinct
# drones. Combines with the first-visitor color to produce a darker tint that
# reads as "shared territory" without obscuring whose color it originally was.
OVERLAP_OVERLAY_RGBA = (0.0, 0.0, 0.0, 0.30)

DRONE_LABEL_FONTSIZE = 8
BATTERY_PANEL_FONTSIZE = 8

# ---------------------------------------------------------------------------
# Figure layout — most of this is now auto-computed from content (n_drones and
# font size) inside visualize.py. Only the maze size is a free knob.
# ---------------------------------------------------------------------------
MAZE_INCHES = 6.0                   # square edge length of the maze axes
FIGURE_SIZE_EDITOR = (8, 8)         # editor stays static; no panel attached

# Heuristic conversion factors used when sizing the side panel from text content.
# At 8pt monospace: ≈ 0.072"/char wide and ≈ 0.16"/line tall (with default leading).
# These are approximations; matplotlib does final layout via tight_layout().
PANEL_CHAR_WIDTH_INCHES = 0.072
PANEL_LINE_HEIGHT_INCHES = 0.16
PANEL_PADDING_INCHES = 0.4          # extra inches around the text block

# When the live GUI window would exceed this fraction of the user's screen,
# scale the whole figure down to fit. Headless PNGs ignore this.
SCREEN_FILL_FRACTION = 0.75

# matplotlib FuncAnimation calls update() every `interval` milliseconds.
# Frames-per-second: fps = 1000 ms/s ÷ ANIMATION_INTERVAL_MS.
# 50 ms → 20 fps (smooth-ish without burning CPU on a 21×21 grid).
ANIMATION_INTERVAL_MS = 50

# ---------------------------------------------------------------------------
# Demo defaults (CLI flags override these)
# ---------------------------------------------------------------------------
DEFAULT_GRID_SIZE = 15              # odd — recursive_backtracker requires odd to fit corridors
DEFAULT_N_DRONES = 3
DEFAULT_SEED = 17
DEFAULT_MAP_KIND = "random"
DEFAULT_OBSTACLE_DENSITY = 0.20     # 20% of the (size-2)² interior cells become walls
DEFAULT_OUTPUT_MAP_NAME = "custom_map.npy"
