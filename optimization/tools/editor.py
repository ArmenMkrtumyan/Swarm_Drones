"""
Interactive map editor for the swarm coverage testbed.

Click + drag to paint walls. Live BFS validation highlights:
    red    = free cells in a disconnected pocket (unreachable from the main area)
    orange = free cells touching the outer boundary (boundary wall missing)

Keys:
    left mouse drag    -> paint walls
    right mouse drag   -> erase to free
    c                  -> clear interior (keep boundary walls)
    r                  -> reset to a fresh boundary-walled grid
    f                  -> fill all interior cells with walls
    v                  -> toggle validation overlay
    g                  -> toggle grid lines
    s                  -> save to --out path (default: outputs/maps/custom_map.npy)
    q                  -> quit

Usage:
    python tools/editor.py --size 21
    python tools/editor.py --load outputs/maps/custom_map.npy --out outputs/maps/v2.npy
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

# Make the project root importable when running `python tools/editor.py` directly.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import matplotlib

# Prefer the native Cocoa backend on macOS (matches matplotlib's own default and
# avoids the system-Tcl/Tk version mismatch that crashes TkAgg on some macs).
# Honor MPLBACKEND if the user has explicitly set one.
if not os.environ.get("MPLBACKEND"):
    matplotlib.use("MacOSX" if sys.platform == "darwin" else "TkAgg", force=True)

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import ListedColormap

from constants import (
    BREACH_OVERLAY_RGBA,
    DEFAULT_GRID_SIZE,
    DEFAULT_OUTPUT_MAP_NAME,
    DISCONNECTED_OVERLAY_RGBA,
    FIGURE_SIZE_EDITOR,
    FREE_COLOR,
    MAPS_DIR,
    WALL_COLOR,
)
from maze import (
    FREE,
    WALL,
    boundary_breaches,
    disconnected_cells,
    load_map,
    open_arena,
    save_map,
)


class MapEditor:
    def __init__(self, grid: np.ndarray, out_path: str) -> None:
        self.grid = grid.astype(np.int8, copy=True)
        self.out_path = out_path
        self.show_validation = True
        self.show_grid = True
        self.painting: int | None = None  # 1=wall, 0=free, None=not painting

        self.fig, self.ax = plt.subplots(figsize=FIGURE_SIZE_EDITOR)
        self.fig.canvas.mpl_connect("button_press_event", self._on_press)
        self.fig.canvas.mpl_connect("button_release_event", self._on_release)
        self.fig.canvas.mpl_connect("motion_notify_event", self._on_motion)
        self.fig.canvas.mpl_connect("key_press_event", self._on_key)

        self._redraw()

    # -------- rendering --------

    def _redraw(self) -> None:
        self.ax.clear()
        h, w = self.grid.shape

        base = np.where(self.grid == WALL, 1.0, 0.0)
        self.ax.imshow(
            base,
            cmap=ListedColormap([FREE_COLOR, WALL_COLOR]),
            origin="upper",
            extent=(0, w, h, 0),
            vmin=0,
            vmax=1,
        )

        if self.show_validation:
            overlay = np.zeros((h, w, 4))
            for x, y in disconnected_cells(self.grid):
                overlay[y, x] = DISCONNECTED_OVERLAY_RGBA
            for x, y in boundary_breaches(self.grid):
                overlay[y, x] = BREACH_OVERLAY_RGBA
            self.ax.imshow(overlay, origin="upper", extent=(0, w, h, 0))

        if self.show_grid:
            for x in range(w + 1):
                self.ax.axvline(x, color="#bbb", lw=0.3)
            for y in range(h + 1):
                self.ax.axhline(y, color="#bbb", lw=0.3)

        self.ax.set_xlim(0, w)
        self.ax.set_ylim(h, 0)
        self.ax.set_aspect("equal")
        self.ax.set_xticks([])
        self.ax.set_yticks([])

        n_disc = len(disconnected_cells(self.grid))
        n_breach = len(boundary_breaches(self.grid))
        status = "OK" if n_disc == 0 and n_breach == 0 else f"disconnected={n_disc}  breaches={n_breach}"
        self.ax.set_title(
            f"editor  size={w}x{h}  out={self.out_path}  [{status}]\n"
            f"L-drag=wall  R-drag=erase  s=save  c=clear  r=reset  v=validate  q=quit"
        )
        self.fig.canvas.draw_idle()

    # -------- events --------

    def _cell_at(self, event) -> tuple[int, int] | None:
        if event.inaxes is not self.ax or event.xdata is None or event.ydata is None:
            return None
        x = int(np.floor(event.xdata))
        y = int(np.floor(event.ydata))
        h, w = self.grid.shape
        if 0 <= x < w and 0 <= y < h:
            return x, y
        return None

    def _paint(self, event) -> None:
        if self.painting is None:
            return
        cell = self._cell_at(event)
        if cell is None:
            return
        x, y = cell
        if self.grid[y, x] != self.painting:
            self.grid[y, x] = self.painting
            self._redraw()

    def _on_press(self, event) -> None:
        if event.button == 1:
            self.painting = WALL
        elif event.button == 3:
            self.painting = FREE
        else:
            return
        self._paint(event)

    def _on_release(self, _event) -> None:
        self.painting = None

    def _on_motion(self, event) -> None:
        self._paint(event)

    def _on_key(self, event) -> None:
        if event.key == "s":
            save_map(self.grid, self.out_path)
            print(f"saved {self.grid.shape} map to {self.out_path}")
        elif event.key == "c":
            self.grid[1:-1, 1:-1] = FREE
            self._redraw()
        elif event.key == "r":
            self.grid = open_arena(self.grid.shape[0])
            self._redraw()
        elif event.key == "f":
            self.grid[1:-1, 1:-1] = WALL
            self._redraw()
        elif event.key == "v":
            self.show_validation = not self.show_validation
            self._redraw()
        elif event.key == "g":
            self.show_grid = not self.show_grid
            self._redraw()
        elif event.key == "q":
            plt.close(self.fig)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--size", type=int, default=DEFAULT_GRID_SIZE)
    parser.add_argument("--load", type=str, default=None, help="load existing .npy/.txt map")
    parser.add_argument("--out", type=str, default=str(MAPS_DIR / DEFAULT_OUTPUT_MAP_NAME))
    args = parser.parse_args()

    if args.load:
        grid = load_map(args.load)
        print(f"loaded map shape={grid.shape} from {args.load}")
    else:
        grid = open_arena(args.size)

    editor = MapEditor(grid, args.out)
    plt.show()


if __name__ == "__main__":
    main()
