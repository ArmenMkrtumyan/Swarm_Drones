"""Central plot styling for the hover + mission benchmarks.

Edit colors, line widths, axis labels, and profile display names here, then
re-run the analyzer (no Isaac/SITL session needed):

    python -m capstone.control.benchmark_hover  flight_logs/
    python -m capstone.control.benchmark_mission --replot --analyze mission_logs/baseline_pid_aua/

Both tools also write an SVG next to every PNG, so cosmetic tweaks can be done
in Inkscape / Illustrator / a text editor as well.

Colors use the matplotlib tab10 family by default so the suite stays
colorblind-friendly without extra fiddling.
"""
from __future__ import annotations


# -----------------------------------------------------------------------------
# Per-trace colors (hover time-series panels)
# -----------------------------------------------------------------------------
COLORS = {
    "altitude":        "#1f77b4",   # blue
    "north":           "#2ca02c",   # green
    "east":            "#8c564b",   # brown
    "roll":            "#d62728",   # red
    "pitch":           "#9467bd",   # purple
    "gyro_mag":        "#ff7f0e",   # orange (post-noise: what controller saw)
    "gyro_mag_truth":  "#1a1a1a",   # near-black (truth: pre-noise reference)
    "wind_mag":        "#17becf",   # teal
    "hover_window":    "#cce5ff",   # pale blue shading
    "mass_drop_event": "#e31a1c",   # bright red marker line
    "gate_pass_edge":  "#7fbf7f",
    "gate_fail_edge":  "#d62728",
    "gate_none_edge":  "#bbb",
    "bar_calm":        "#7fbf7f",
    "bar_disturbed":   "#fdc086",
}

LINE_WIDTHS = {
    "altitude": 1.0,
    "position": 0.9,
    "attitude": 0.8,
    "gyro":     0.8,
    "wind":     0.9,
    "marker":   1.0,
}


# -----------------------------------------------------------------------------
# Axis / legend labels
# -----------------------------------------------------------------------------
LABELS = {
    "altitude_y":  "alt (m)",
    "position_y":  "xy pos (m)",
    "attitude_y":  "roll/pitch (rad)",
    "gyro_y":      "|ang vel| (rad/s)",
    "time_x":      "t (s)",
    "north_line":  "north (forward of home)",
    "east_line":   "east (right of home)",
    "roll_line":   "roll",
    "pitch_line":  "pitch",
}


# -----------------------------------------------------------------------------
# Display names per profile (shown in titles, bar charts). Edit to relabel.
# -----------------------------------------------------------------------------
PROFILE_DISPLAY_NAMES = {
    "calm":            "calm",
    "mass_drop_300g":  "mass_drop_300g",
    "wind5":           "wind5",
    "wind_up3":        "wind_up3",
    "wind_down3":      "wind_down3",
    "imu_noise":       "imu_noise",
    "worst_case":      "worst_case",
}


def display_name(profile: str) -> str:
    return PROFILE_DISPLAY_NAMES.get(profile, profile)


# -----------------------------------------------------------------------------
# Output formats: every savefig writes a PNG; if EMIT_SVG is True it also
# writes an SVG next to it (same stem, .svg extension). SVG is text-editable
# and opens cleanly in Inkscape / Illustrator / any browser.
# -----------------------------------------------------------------------------
EMIT_SVG = True
DPI_PNG = 110


def savefig_dual(fig, png_path) -> None:
    """Save a matplotlib Figure as PNG and (if EMIT_SVG) SVG to a parallel tree.

    Caller passes `<root>/png/<rest>/foo.png`. PNG goes there. SVG goes to
    `<root>/svg/<rest>/foo.svg` — the first `png` directory component in the
    path is replaced with `svg`. If the path contains no `png` component
    (legacy callers), SVG falls back to the same dir as the PNG.
    """
    from pathlib import Path

    p = Path(png_path)
    p.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(p, dpi=DPI_PNG)
    if not EMIT_SVG:
        return
    parts = list(p.parts)
    for i, part in enumerate(parts):
        if part == "png":
            parts[i] = "svg"
            svg_path = Path(*parts).with_suffix(".svg")
            break
    else:
        svg_path = p.with_suffix(".svg")
    svg_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(svg_path)
