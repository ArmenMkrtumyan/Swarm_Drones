"""
Matplotlib renderer for CoverageEnv.

Headless mode:  render_frame(env, save_path='frame.png')
Interactive:    animate(env, policy_fn, n_steps=200)
"""

from __future__ import annotations

import colorsys
import os
import sys
from typing import Callable, Optional

import matplotlib

# Default to a non-interactive backend ONLY if no caller has already imported
# pyplot (and therefore chosen a backend). matplotlib 3.x changed `force=False`
# semantics: it now suppresses *failure* but still performs the switch. So an
# unconditional `matplotlib.use("Agg", force=False)` here would silently clobber
# a caller's explicit `matplotlib.use("TkAgg", force=True)` whenever they import
# us afterward — turning every GUI run into a non-interactive Agg figure that
# can't be shown. The sys.modules guard preserves caller intent.
if "matplotlib.pyplot" not in sys.modules:
    matplotlib.use("Agg", force=False)

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from matplotlib.colors import ListedColormap

from constants import (
    BATTERY_PANEL_FONTSIZE,
    DEPLETED_DRONE_RGB,
    DRONE_LABEL_FONTSIZE,
    DRONE_PALETTE_SATURATION,
    DRONE_PALETTE_VALUE,
    FIRST_VISITOR_ALPHA,
    FREE_COLOR,
    MAZE_INCHES,
    OVERLAP_OVERLAY_RGBA,
    PANEL_CHAR_WIDTH_INCHES,
    PANEL_LINE_HEIGHT_INCHES,
    PANEL_PADDING_INCHES,
    SCREEN_FILL_FRACTION,
    SENSOR_CIRCLE_ALPHA,
    WALL_COLOR,
)
from environment import CoverageEnv
from maze import FREE, WALL


# Module-level palette state. Built lazily by drone_color() if the caller
# never invokes init_drone_palette() — useful so test helpers and one-off
# scripts don't have to plumb a seed if they don't care about reproducibility.
_PALETTE: list[tuple[float, float, float]] = []


def init_drone_palette(n_drones: int, seed: Optional[int] = None) -> None:
    """
    Build the per-drone color palette for this run.

    Hues are placed at 1/n_drones spacing on the HSV circle, then the whole
    circle is rotated by a random offset drawn from `seed` (or OS entropy if
    seed is None). Every drone is therefore exactly 360°/n_drones from its
    nearest palette neighbor — maximally distinct in hue regardless of `n`.

    Call this once after constructing the env, before any render_frame() or
    animate() call. For reproducible visuals (e.g., test fixtures), pass a
    fixed seed; for the demo, leave it None so each run looks different.
    """
    global _PALETTE
    rng = np.random.default_rng(seed)
    offset = float(rng.random())  # rotation in [0, 1)
    s = DRONE_PALETTE_SATURATION
    v = DRONE_PALETTE_VALUE
    _PALETTE = [
        colorsys.hsv_to_rgb((offset + i / n_drones) % 1.0, s, v)
        for i in range(n_drones)
    ]


def drone_color(i: int) -> tuple[float, float, float]:
    """
    RGB triple for drone `i`. If the palette has not yet been initialized
    (e.g., a renderer was called before init_drone_palette), fall back to a
    deterministic 9-color palette so output is at least defined.
    """
    if not _PALETTE:
        init_drone_palette(9, seed=0)
    return _PALETTE[i % len(_PALETTE)]


def format_duration(total_seconds: float) -> str:
    """
    Render a duration so long sims read naturally:
        45.2  → "45.2 seconds"
        65.0  → "1 minute 5 seconds"
        119.7 → "2 minutes 0 seconds"
        734.6 → "12 minutes 15 seconds"

    Under 60 s we keep one decimal so per-frame updates feel live; once we cross
    a minute we round to whole seconds because the decimal stops being useful
    relative to the larger unit. Singular/plural labels are correct on both
    sides ("1 minute 1 second", "2 minutes 5 seconds").
    """
    if total_seconds < 60.0:
        return f"{total_seconds:.1f} seconds"
    total_int = int(round(total_seconds))
    minutes, seconds = divmod(total_int, 60)
    min_label = "minute" if minutes == 1 else "minutes"
    sec_label = "second" if seconds == 1 else "seconds"
    return f"{minutes} {min_label} {seconds} {sec_label}"


def _drone_render_color(i: int, depleted: bool) -> tuple:
    """Color used for the drone's marker / sensor circle / arrow. Gray if dead."""
    return DEPLETED_DRONE_RGB if depleted else drone_color(i)


def _render_map(env: CoverageEnv, ax) -> None:
    ax.clear()
    h, w = env.grid.shape

    # Layer 1: walls (gray) vs free (white)
    base = np.where(env.grid == WALL, 1.0, 0.0)
    ax.imshow(base, cmap=ListedColormap([FREE_COLOR, WALL_COLOR]), origin="upper",
              extent=(0, w, h, 0), vmin=0, vmax=1)

    # Layer 2a: per-drone first-visitor paint. Each free cell wears the color
    # of whichever drone first reached it; later visitors don't repaint, so the
    # map stays readable as territory ownership rather than a chaotic mix.
    free_mask = env.grid == FREE
    overlay = np.zeros((h, w, 4))
    for i in range(env.n_drones):
        mask = (env.first_visitor == i) & free_mask
        if not np.any(mask):
            continue
        r_, g_, b_ = drone_color(i)
        overlay[mask] = (r_, g_, b_, FIRST_VISITOR_ALPHA)
    ax.imshow(overlay, origin="upper", extent=(0, w, h, 0))

    # Layer 2b: shared-territory overlay. Cells touched by ≥ 2 distinct drones
    # get a translucent black on top, darkening the first-visitor color into a
    # visually distinct "overlapped" tone without hiding whose territory it is.
    multi_visited = (env.drone_covered.sum(axis=0) >= 2) & free_mask
    if np.any(multi_visited):
        overlap_overlay = np.zeros((h, w, 4))
        overlap_overlay[multi_visited] = OVERLAP_OVERLAY_RGBA
        ax.imshow(overlap_overlay, origin="upper", extent=(0, w, h, 0))

    # Layer 3: drones with index labels (depleted drones rendered in gray)
    pos = env.positions()
    vel = env.velocities()
    depleted = [env.battery_state(i)["depleted"] for i in range(len(env.drones))]
    colors = [_drone_render_color(i, depleted[i]) for i in range(env.n_drones)]
    ax.scatter(pos[:, 0], pos[:, 1], c=colors, s=60, edgecolors="black", zorder=3)

    # Velocity arrows — drawn in cell-units. With angles='xy' and scale=1 in
    # 'xy' units, an arrow of length |v| cells/s shows where the drone would
    # reach in 1 sim second of straight-line motion. So a 1.5 cells/s drone
    # gets a 1.5-cell-long arrow, instantly readable for direction + speed.
    speed_cells = np.linalg.norm(vel, axis=1)
    moving = speed_cells > 1e-3
    mpc = env.sim_cfg.meters_per_cell
    if np.any(moving):
        arrow_colors = [_drone_render_color(i, depleted[i]) for i in range(env.n_drones)]
        ax.quiver(
            pos[moving, 0], pos[moving, 1],
            vel[moving, 0], vel[moving, 1],
            angles="xy", scale_units="xy", scale=1,
            color=[c for c, m in zip(arrow_colors, moving) if m],
            width=0.005, headwidth=4, headlength=5, zorder=4,
        )
        # Speed label at each arrow tip in real-world m/s.
        for i in range(len(env.drones)):
            if not moving[i]:
                continue
            tip = pos[i] + vel[i]
            ax.annotate(
                f"{speed_cells[i] * mpc:.1f} m/s",
                xy=(tip[0], tip[1]),
                xytext=(4, 4), textcoords="offset points",
                fontsize=7, color="black",
                bbox=dict(boxstyle="round,pad=0.15", facecolor="white",
                          edgecolor="none", alpha=0.7),
                zorder=5,
            )

    for i, p in enumerate(pos):
        circle_color = _drone_render_color(i, depleted[i])
        circ = plt.Circle(
            (p[0], p[1]),
            env.drone_cfg.sensor_radius,
            fill=False, color=circle_color, alpha=SENSOR_CIRCLE_ALPHA, linestyle="--",
        )
        ax.add_patch(circ)
        label = f"D{i + 1}" + (" ☠" if depleted[i] else "")
        ax.annotate(
            label, (p[0], p[1]),
            xytext=(6, -6), textcoords="offset points",
            fontsize=DRONE_LABEL_FONTSIZE, color="black", fontweight="bold", zorder=4,
        )

    ax.set_xlim(0, w)
    ax.set_ylim(h, 0)
    ax.set_aspect("equal")
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_title(
        f"time={format_duration(env.time_seconds)}   "
        f"coverage={env.coverage_fraction():.1%}"
    )


def _render_battery_panel(env: CoverageEnv, ax) -> None:
    """Draw a per-drone battery readout. Uses a text bbox so the visible box hugs content."""
    ax.clear()
    ax.set_xticks([])
    ax.set_yticks([])
    # Hide the axes spines — the visible border comes from the text's bbox below,
    # which auto-sizes to the actual content (no giant empty rectangle).
    for spine in ax.spines.values():
        spine.set_visible(False)
    if not env.drones:
        return

    cfg = env.battery_cfg
    per_cell_min = cfg.min_voltage_v / cfg.n_cells
    v_full = cfg.cell_full_voltage_v * cfg.n_cells
    v_empty = cfg.cell_dead_voltage_v * cfg.n_cells
    mpc = env.sim_cfg.meters_per_cell
    h, w = env.grid.shape
    header = [
        "Battery",
        "Hawk's Work F450",
        f"{cfg.voltage_v:.1f}V nominal  ({v_full:.1f}V full → {v_empty:.1f}V empty)",
        f"{cfg.capacity_mah:.0f}mAh  mass {env.drone_cfg.mass_kg:.1f}kg",
        f"cutoff: {cfg.min_voltage_v:.1f}V ({per_cell_min:.2f}V × {cfg.n_cells} cells)",
        f"world: {w * mpc:.0f}m × {h * mpc:.0f}m  ({mpc:.1f}m/cell)",
        "",
    ]
    rows = []
    for i in range(len(env.drones)):
        b = env.battery_state(i)
        tag = "  [DEAD Battery, landing now!]" if b["depleted"] else ""
        cov_m2 = env.drone_coverage_m2(i)
        cov_pct = 100.0 * env.drone_coverage_fraction(i)
        tv = env.total_visits(i)
        sr = env.self_revisits(i)
        co = env.cross_overlap_visits(i)
        rows.append(
            f"D{i + 1}{tag} | V {b['voltage_v_current']:5.2f} | "
            f"used ({100 - b['percent_remaining']:5.2f}%) | "
            f"cov {cov_m2:6.0f} m² ({cov_pct:5.2f}%) | "
            f"visits {tv:5d} (self↺{sr:4d}, on-others {co:5d})"
        )
    rows.append("")
    rows.append(f"global cov:     {100 * env.coverage_fraction():7.2f}%")
    rows.append(f"overlap area:   {env.overlap_cells_m2():9.0f} m²")
    rows.append(f"wasted visits:  {env.wasted_visits_total():9d}")

    # transform=ax.transAxes uses axes-fraction coordinates (0,0)=bottom-left,
    # (1,1)=top-right. The bbox auto-shrinks to the text rectangle, so the
    # visible border hugs the content instead of stretching to the axes edge.
    ax.text(
        0.5, 0.98, "\n".join(header + rows),
        transform=ax.transAxes,
        ha="center", va="top",
        fontsize=BATTERY_PANEL_FONTSIZE, family="monospace",
        bbox=dict(boxstyle="round,pad=0.5", facecolor="white", edgecolor="#888", linewidth=0.8),
    )


def _detect_screen_inches() -> Optional[tuple[float, float]]:
    """
    Best-effort screen-size detection. Tries env vars first (user override),
    then macOS system_profiler, then falls back to None. Deliberately avoids
    Tkinter — Tk crashes hard on some macOS / Tcl combinations and there is no
    safe way to catch that abort.

    Override path:  set SWARM_SCREEN_INCHES="WIDTH,HEIGHT"  e.g.  "15.6,10.0"
    """
    override = os.environ.get("SWARM_SCREEN_INCHES")
    if override:
        try:
            w, h = (float(x) for x in override.split(","))
            return (w, h)
        except ValueError:
            pass

    if sys.platform == "darwin":
        try:
            import subprocess
            out = subprocess.run(
                ["system_profiler", "SPDisplaysDataType"],
                capture_output=True, text=True, timeout=2,
            ).stdout
            # Parse a line like "Resolution: 1920 x 1080"
            for line in out.splitlines():
                line = line.strip()
                if line.startswith("Resolution:"):
                    parts = line.split()
                    w_px = int(parts[1])
                    h_px = int(parts[3])
                    # Assume ~110 ppi (typical for laptop retina-scaled or external).
                    return (w_px / 110.0, h_px / 110.0)
        except Exception:
            pass

    return None


def _compute_figure_dims(env: CoverageEnv) -> tuple[float, float, list[float]]:
    """
    Derive figsize and width_ratios from content and (when available) screen size:
      - Panel size = max-line-chars × char-width  by  n-lines × line-height
      - Maze axes  = MAZE_INCHES square (auto-grown if the panel is taller)
      - If the result is bigger than SCREEN_FILL_FRACTION of the detected
        display, scale the whole figure down uniformly.
    """
    # Worst-case panel content: every drone row could carry the full DEAD label.
    # Header(7) + drone rows + 1 blank + 3 footer (global / overlap-cells / wasted-visits).
    n_lines = 7 + env.n_drones + 1 + 3
    # Worst-case row width — the per-drone row is the widest line on the panel.
    max_chars = max(
        len("D9  [DEAD Battery, landing now!] | V 12.59 | used (99.99%) | "
            "cov 999999 m² (99.99%) | visits 99999 (self↺9999, on-others 99999)"),
        len("cutoff: 99.9V (4.20V × 6 cells)"),
        len("world: 9999m × 9999m  (99.9m/cell)"),
        len("overlap area:   99999999 m²"),
    )

    panel_w = max_chars * PANEL_CHAR_WIDTH_INCHES + PANEL_PADDING_INCHES
    panel_h = n_lines * PANEL_LINE_HEIGHT_INCHES + PANEL_PADDING_INCHES

    maze_size = max(MAZE_INCHES, panel_h)
    fig_w = maze_size + panel_w
    fig_h = maze_size + 0.4   # ~0.4" headroom for the title above the maze axes

    # Optional: cap to a fraction of the live display so a giant panel doesn't
    # spawn a window taller than the user's monitor.
    screen = _detect_screen_inches()
    if screen is not None:
        max_w = screen[0] * SCREEN_FILL_FRACTION
        max_h = screen[1] * SCREEN_FILL_FRACTION
        if fig_w > max_w or fig_h > max_h:
            scale = min(max_w / fig_w, max_h / fig_h)
            fig_w *= scale
            fig_h *= scale
            maze_size *= scale
            panel_w *= scale

    return fig_w, fig_h, [maze_size, panel_w]


def _make_figure(env: CoverageEnv) -> tuple:
    """Two-column layout sized from env content and (optionally) screen dimensions."""
    fig_w, fig_h, width_ratios = _compute_figure_dims(env)
    fig, (ax_map, ax_panel) = plt.subplots(
        1, 2,
        figsize=(fig_w, fig_h),
        gridspec_kw={"width_ratios": width_ratios},
    )
    return fig, ax_map, ax_panel


def render_frame(
    env: CoverageEnv,
    save_path: Optional[str] = None,
    show: bool = False,
):
    fig, ax_map, ax_panel = _make_figure(env)
    _render_map(env, ax_map)
    _render_battery_panel(env, ax_panel)
    fig.tight_layout()
    if save_path:
        # No bbox_inches="tight" — we want the saved PNG to reflect the
        # figure size honestly so changes to FIGURE_SIZE_DEMO are visible.
        fig.savefig(save_path, dpi=120)
    if show:
        plt.show()
    return fig


def animate(
    env: CoverageEnv,
    policy_fn: Callable[[CoverageEnv], np.ndarray],
    interval_ms: int = 50,
    save_path: Optional[str] = None,
    interactive: bool = True,
):
    """
    Run policy_fn(env) -> actions until env.is_terminal() (mission complete or
    all drones depleted). interactive=True opens a window (requires a GUI
    backend). save_path writes an MP4 or GIF (requires ffmpeg/pillow).

    Interactive mode includes a 1× / 2× / 5× / 10× / 20× speed slider that
    controls how many ``env.step()`` calls happen between renders. The slider
    only changes how often we *display* a frame — every step uses the same
    ``step_seconds`` and the same explicit-Euler integration, so physics is
    bit-identical regardless of the slider value. We never bump
    ``step_seconds`` itself; that would let drones travel further than
    ``drone_radius`` per step and break wall collision.

    No step cap: all-drones-depleted is physics-bounded (~7340 steps for the
    F450 defaults), so the loop always terminates naturally.
    """
    if interactive:
        import os
        import sys
        if not os.environ.get("MPLBACKEND"):
            backend = "MacOSX" if sys.platform == "darwin" else "TkAgg"
            try:
                matplotlib.use(backend, force=True)
            except Exception as e:
                print(f"GUI backend {backend!r} unavailable ({e}); falling back to headless.")
                interactive = False

    fig, ax_map, ax_panel = _make_figure(env)

    # Speed slider — only when interactive AND not saving (a GIF/MP4 export
    # should run at one fixed speed). Reserve bottom strip for the slider.
    speed_slider = None
    if interactive and save_path is None:
        from matplotlib.widgets import Slider
        fig.subplots_adjust(bottom=0.13)
        ax_slider = fig.add_axes([0.45, 0.04, 0.28, 0.025])
        speed_slider = Slider(
            ax_slider, "sim speed  (steps/frame)",
            valmin=1, valmax=20, valinit=1,
            valstep=[1, 2, 5, 10, 20],
        )

    def update(_frame):
        # Batch N env.step() calls between renders. dt per step is unchanged.
        steps = int(speed_slider.val) if speed_slider is not None else 1
        for _ in range(steps):
            if env.is_terminal():
                break
            actions = policy_fn(env)
            env.step(actions)
        _render_map(env, ax_map)
        _render_battery_panel(env, ax_panel)
        return []

    # Generator-driven frame source: stops as soon as the mission ends
    # (100% coverage or all drones depleted). No step cap — physics bounds it.
    def frame_iter():
        n = 0
        while not env.is_terminal():
            yield n
            n += 1

    # save_count is only used when writing a GIF/MP4; matplotlib needs an
    # upper bound to allocate. 10000 is comfortably above the ~7340-step
    # physics ceiling so the writer pre-allocates enough.
    anim = FuncAnimation(
        fig, update, frames=frame_iter, interval=interval_ms,
        blit=False, save_count=10000, repeat=False,
        cache_frame_data=False,
    )

    if save_path:
        # interval_ms is the gap between frames in milliseconds; fps is the
        # inverse expressed per second, hence 1000 ms/s ÷ interval_ms.
        # 50 ms → 20 fps, 33 ms → 30 fps, 16 ms → 60 fps.
        fps = 1000 // interval_ms
        if save_path.endswith(".gif"):
            anim.save(save_path, writer="pillow", fps=fps)
        else:
            anim.save(save_path, fps=fps)

    if interactive:
        plt.show()
    else:
        plt.close(fig)

    return anim
