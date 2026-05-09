"""
Generate a single algorithm-leaderboard PNG from a sweep CSV.

Reads a `sweep_*_results.csv`, aggregates each metric per policy across
all (map × n_drones × seed) cells, and produces one composite figure
showing how every algorithm in that track ranks across all the metrics
the sweep collected. Each metric is colored independently from green
(best) to red (worst); the cell text shows the actual aggregated value
in human-readable units.

Useful when the per-axis sweep plots
(`sweep_*_coverage_vs_drones.png`, `sweep_*_curves_grid.png`, …) are too
fragmented to see "which algorithm is best overall" at a glance.

Usage:
    python tools/leaderboard.py --csv outputs/sweep_track1_results.csv
    python tools/leaderboard.py \\
        --csv outputs_metaheuristic/sweep_track2_results.csv \\
        --out outputs_metaheuristic/leaderboard.png
    python tools/leaderboard.py \\
        --csv outputs_control_based/csv_txt/sweep_results.csv

If `--out` is omitted, the PNG is written next to the CSV's parent
folder root (e.g. `outputs/leaderboard.png` for `outputs/sweep_track1_results.csv`).
"""

from __future__ import annotations

import argparse
import csv
import sys
from collections import defaultdict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap
import numpy as np


# Metric definitions: (display label, csv field, higher-is-better, value formatter).
# `time_to_*` fields use 'inf' string in CSVs when the threshold was never
# reached — we substitute `time_to_terminal` so the algorithm doesn't get an
# unfair NaN.
METRICS = [
    ("Final coverage", "final_coverage", True,  lambda v: f"{v * 100:.1f}%"),
    ("Time → 100%",    "time_to_100",    False, lambda v: f"{v:.0f}s"),
    ("Overlap area",   "overlap_m2",     False, lambda v: f"{v:.0f}m²"),
    ("Wasted visits",  "wasted_visits",  False, lambda v: f"{v:.0f}"),
    ("Energy used",    "total_energy_j", False, lambda v: f"{v / 1000:.0f}kJ"),
]


def aggregate(rows: list[dict]) -> dict[str, dict[str, float]]:
    """Mean of each metric per policy. Times-to-X with 'inf' use time_to_terminal."""
    by_policy: dict[str, dict[str, list[float]]] = defaultdict(lambda: defaultdict(list))
    for r in rows:
        policy = r["policy"]
        terminal = float(r.get("time_to_terminal", 0.0))
        for label, field, _, _ in METRICS:
            v_raw = r.get(field, "")
            if v_raw == "" or v_raw is None:
                continue
            try:
                v = float(v_raw)
            except (ValueError, TypeError):
                continue
            if not np.isfinite(v):  # 'inf' / NaN
                if field.startswith("time_to_"):
                    v = terminal       # didn't reach the threshold; treat as full run
                else:
                    continue
            by_policy[policy][field].append(v)
    return {p: {k: float(np.mean(vs)) for k, vs in fields.items() if vs}
            for p, fields in by_policy.items()}


def build_leaderboard(csv_path: Path, out_path: Path, title: str = None) -> None:
    rows = list(csv.DictReader(open(csv_path)))
    if not rows:
        raise ValueError(f"empty CSV: {csv_path}")
    agg = aggregate(rows)

    # Filter out metrics no policy has data for.
    active_metrics = []
    for label, field, hib, fmt in METRICS:
        n_with = sum(1 for p_data in agg.values() if field in p_data)
        if n_with >= 2:
            active_metrics.append((label, field, hib, fmt))
    if not active_metrics:
        raise ValueError("no active metrics in CSV")

    policies = sorted(agg.keys())
    # Order policies: keep "Random" first if present (baseline), then alphabetical.
    if "Random" in policies:
        policies = ["Random"] + [p for p in policies if p != "Random"]

    n_p = len(policies)
    n_m = len(active_metrics)

    # Build the [n_p, n_m] matrix of raw values and a 3-level highlight score:
    #   1.0 = per-column best                   → full green
    #   0.4 = within 5 % of the best (relative) → light green
    #   0.0 = otherwise                         → white
    raw = np.full((n_p, n_m), np.nan)
    for j, (label, field, hib, fmt) in enumerate(active_metrics):
        for i, p in enumerate(policies):
            if field in agg[p]:
                raw[i, j] = agg[p][field]

    CLOSE_THRESHOLD = 0.05      # within 5% of the leader → second-place green

    score = np.zeros_like(raw, dtype=float)
    is_best = np.zeros_like(raw, dtype=bool)
    for j, (label, field, hib, fmt) in enumerate(active_metrics):
        col = raw[:, j]
        if np.all(np.isnan(col)):
            continue
        best_val = np.nanmax(col) if hib else np.nanmin(col)
        denom = max(abs(best_val), 1e-9)
        for i in range(n_p):
            if np.isnan(col[i]):
                continue
            gap = abs(col[i] - best_val) / denom
            if gap < 1e-9:
                score[i, j] = 1.0
                is_best[i, j] = True
            elif gap <= CLOSE_THRESHOLD:
                score[i, j] = 0.4

    # Plot.
    fig_w = max(8.0, 1.6 * n_m + 2.5)
    fig_h = max(3.5, 0.55 * n_p + 1.8)
    fig, ax = plt.subplots(figsize=(fig_w, fig_h))

    # 3-anchor colormap: white → light green (at 0.4) → full green (at 1.0).
    cmap = LinearSegmentedColormap.from_list(
        "best_with_runnerup",
        [(0.0, "white"), (0.4, "#c8e6c9"), (1.0, "#5cb85c")],
    )
    ax.imshow(score, cmap=cmap, vmin=0.0, vmax=1.0, aspect="auto")

    ax.set_xticks(range(n_m))
    ax.set_xticklabels(
        [m[0] + (" ↑" if m[2] else " ↓") for m in active_metrics],
        rotation=20, ha="right", fontsize=9,
    )
    ax.set_yticks(range(n_p))
    ax.set_yticklabels(policies, fontsize=10)

    # Per-cell text: raw value, bolded for the per-column best.
    for i in range(n_p):
        for j in range(n_m):
            if np.isnan(raw[i, j]):
                continue
            txt = active_metrics[j][3](raw[i, j])
            ax.text(
                j, i, txt, ha="center", va="center",
                color="black", fontsize=9,
                fontweight="bold" if is_best[i, j] else "normal",
            )

    # Light grid between cells so the table reads as rows/columns.
    ax.set_xticks(np.arange(-0.5, n_m, 1), minor=True)
    ax.set_yticks(np.arange(-0.5, n_p, 1), minor=True)
    ax.grid(which="minor", color="#cccccc", linewidth=0.5)
    ax.tick_params(which="minor", length=0)

    ttl = title or f"Algorithm leaderboard — {csv_path.name}"
    ax.set_title(ttl, fontsize=12, fontweight="bold", pad=12)

    fig.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=140, bbox_inches="tight")
    plt.close(fig)
    print(f"Wrote: {out_path}")


def parse_args():
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--csv", type=str, required=True,
                   help="path to a sweep_*_results.csv")
    p.add_argument("--out", type=str, default=None,
                   help="output PNG (default: leaderboard.png next to "
                        "the CSV's outputs folder)")
    p.add_argument("--title", type=str, default=None)
    return p.parse_args()


def main():
    args = parse_args()
    csv_path = Path(args.csv).resolve()
    if args.out:
        out_path = Path(args.out).resolve()
    else:
        # Walk up to the nearest folder named 'outputs*' and put the PNG there.
        out_root = csv_path.parent
        while out_root.name and not out_root.name.startswith("outputs"):
            out_root = out_root.parent
        if not out_root.name:
            out_root = csv_path.parent
        out_path = out_root / "leaderboard.png"
    build_leaderboard(csv_path, out_path, title=args.title)


if __name__ == "__main__":
    main()
