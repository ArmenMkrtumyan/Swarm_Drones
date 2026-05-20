"""Publication-aesthetic versions of the calm-run, worst-run, and cross-batch
comparison figures.

Produces three PNG figures into `reports/benchmark_hover_report/paper_figs/` ready
for inclusion in a paper:

    paper_figs/
        calm_run.png        — single calm hover run, 4-panel time series
        worst_case_run.png  — single worst_case run, same panels
        comparison.png      — calm vs worst_case median + IQR + raw points

PNG only (300 DPI). The paper's editor opens the existing batch SVGs as the
vector source, so this module deliberately does not emit duplicate SVGs.

Working figures (with PASS/FAIL boxes, gate annotations, etc.) stay untouched
in `<batch>/report/png/` — those are for development. This module is for the
paper's figure folder.

Usage:
    python -m lab.control.paper_figs                    # use defaults
    python -m lab.control.paper_figs \\
        --calm-log  reports/benchmark_hover_report/calm_baseline_runs/flight_..._CALM5.jsonl \\
        --worst-log reports/benchmark_hover_report/batch_worst_case/flight_..._WORST3_SUCCESS.jsonl \\
        --calm-csv  reports/benchmark_hover_report/calm_baseline_runs/report/baseline.csv \\
        --worst-csv reports/benchmark_hover_report/batch_worst_case/report/baseline.csv \\
        --out       reports/benchmark_hover_report/paper_figs

Aesthetic decisions (set in `_apply_paper_style`):
  - Computer Modern serif (LaTeX look) for all text
  - Wong colorblind-safe palette (https://www.nature.com/articles/nmeth.1618)
  - Top + right spines hidden ("open" axes)
  - Very subtle grid, dotted, alpha 0.15
  - Tick marks pointing inward
  - Subfigure label "(a)" "(b)" in the upper-left, no panel titles
"""
from __future__ import annotations

import argparse
import csv
import math
import statistics
import sys
from pathlib import Path

from lab.common import logging as flight_logging
from lab.common.logging import FlightLog
from lab.control import metrics as metrics_mod


# Wong colorblind-safe palette — bluish-green, vermillion, sky blue, etc.
# https://www.nature.com/articles/nmeth.1618
WONG = {
    "black":         "#000000",
    "orange":        "#E69F00",
    "sky_blue":      "#56B4E9",
    "bluish_green":  "#009E73",
    "yellow":        "#F0E442",
    "blue":          "#0072B2",
    "vermillion":    "#D55E00",
    "reddish_purple":"#CC79A7",
}


def _apply_paper_style() -> None:
    """Set matplotlib rcParams for a research-paper aesthetic."""
    import matplotlib as mpl
    mpl.rcParams.update({
        "font.family":        "serif",
        "font.serif":         ["DejaVu Serif", "Times New Roman", "Times"],
        "mathtext.fontset":   "cm",
        "axes.titlesize":     11,
        "axes.labelsize":     10,
        "xtick.labelsize":    8.5,
        "ytick.labelsize":    8.5,
        "legend.fontsize":    8.5,
        "axes.spines.top":    False,
        "axes.spines.right":  False,
        "axes.linewidth":     0.8,
        "xtick.direction":    "in",
        "ytick.direction":    "in",
        "xtick.major.size":   3,
        "ytick.major.size":   3,
        "xtick.minor.visible": True,
        "ytick.minor.visible": False,
        "xtick.minor.size":   1.8,
        "grid.linewidth":     0.4,
        "grid.linestyle":     ":",
        "grid.alpha":         0.4,
        "savefig.dpi":        300,
        "savefig.bbox":       "tight",
        "pdf.fonttype":       42,   # editable text in vector outputs
        "svg.fonttype":       "none",
    })


# -----------------------------------------------------------------------------
# Per-run time series
# -----------------------------------------------------------------------------
def _extract(log: FlightLog) -> dict:
    """Pull parallel arrays from a flight log (subset of bench_hover.extract_series)."""
    ts, alts, ns, es, rolls, pitches, gyros, gyros_truth = [], [], [], [], [], [], [], []
    have_truth = any("gyro_frd_truth" in s for s in log.states)
    for s in log.states:
        ts.append(float(s["t"]))
        pos = s["pos_ned"]
        alts.append(-float(pos[2]))
        ns.append(float(pos[0]))
        es.append(float(pos[1]))
        r, p, _y = (float(x) for x in s["rpy"])
        rolls.append(r); pitches.append(p)
        gyros.append(math.sqrt(sum(float(g) ** 2 for g in s["gyro_frd"])))
        gtsrc = s.get("gyro_frd_truth", s["gyro_frd"])
        gyros_truth.append(math.sqrt(sum(float(g) ** 2 for g in gtsrc)))
    return {
        "t": ts, "alt": alts, "north": ns, "east": es,
        "roll": rolls, "pitch": pitches,
        "gyro": gyros, "gyro_truth": gyros_truth,
        "have_truth": have_truth,
    }


def _draw_run(
    log: FlightLog,
    out_path: Path,
    profile_caption: str,
    drop_t: float | None = None,
) -> None:
    """Render a single 4-panel time series in paper style. Panels: alt,
    xy pos (N green / E orange), roll/pitch, |gyro| (post + truth).
    `profile_caption` shows top-left in place of a title."""
    import matplotlib.pyplot as plt

    s = _extract(log)
    if not s["t"]:
        return

    # Compute hover scoring window so we can shade it.
    profile = "calm"  # detection not needed for shading; recompute for window only
    # Use metrics machinery to find the window:
    cal_event = log.find_event("capstone_disturbance_active")
    if cal_event is not None:
        profile = str(cal_event.get("profile", profile))
    m = metrics_mod.compute_metrics(log, profile, log_path=str(log.path))
    win_t0, win_t1 = m.window_t0, m.window_t1

    # Always crop to scoring window ± 3 s — same on every panel of every run.
    # Drop time (if any) falls inside the scoring window for worst_case so
    # it stays visible without needing to widen the x-range around drop_t.
    if win_t0 is not None and win_t1 is not None:
        x0, x1 = win_t0 - 3.0, win_t1 + 3.0
    else:
        x0, x1 = s["t"][0], s["t"][-1]
    x0 = max(x0, s["t"][0]); x1 = min(x1, s["t"][-1])

    fig, axes = plt.subplots(4, 1, figsize=(7.0, 7.5), sharex=True)

    def _shade(ax):
        if win_t0 is not None and win_t1 is not None:
            ax.axvspan(win_t0, win_t1, color="#dde6f0", alpha=0.55, lw=0)

    def _drop(ax):
        if drop_t is not None:
            ax.axvline(drop_t, color=WONG["vermillion"],
                       linestyle="--", linewidth=0.9, alpha=0.85)

    # 1) altitude
    ax = axes[0]
    ax.plot(s["t"], s["alt"], color=WONG["blue"], linewidth=1.0)
    _shade(ax); _drop(ax)
    ax.set_ylabel(r"altitude $z$ (m)")
    ax.grid(True)

    # 2) xy position
    ax = axes[1]
    ax.plot(s["t"], s["north"], color=WONG["bluish_green"],
            linewidth=0.9, label="north")
    ax.plot(s["t"], s["east"],  color=WONG["orange"],
            linewidth=0.9, label="east")
    _shade(ax); _drop(ax)
    ax.set_ylabel(r"horizontal pos. (m)")
    ax.legend(loc="upper right", frameon=False, ncol=2,
              handlelength=1.6, columnspacing=1.2)
    ax.grid(True)

    # 3) attitude (roll / pitch)
    ax = axes[2]
    ax.plot(s["t"], s["roll"],  color=WONG["vermillion"],
            linewidth=0.9, label="roll")
    ax.plot(s["t"], s["pitch"], color=WONG["reddish_purple"],
            linewidth=0.9, label="pitch")
    _shade(ax); _drop(ax)
    ax.set_ylabel(r"attitude (rad)")
    ax.legend(loc="upper right", frameon=False, ncol=2,
              handlelength=1.6, columnspacing=1.2)
    ax.grid(True)

    # 4) angular velocity magnitude (post-noise + truth when available)
    ax = axes[3]
    ax.plot(s["t"], s["gyro"], color=WONG["orange"], linewidth=0.9,
            label="post-noise (controller input)" if s["have_truth"] else None)
    if s["have_truth"]:
        ax.plot(s["t"], s["gyro_truth"],
                color=WONG["black"], linewidth=0.95, linestyle=":",
                label="truth (pre-noise)")
        ax.legend(loc="upper right", frameon=False, ncol=1, handlelength=1.6)
    # Y-clip the gyro panel for every run (calm + worst alike), based on the
    # data inside the visible x-range so spikes outside the scoring window
    # don't compress the trace into a flat line at the bottom.
    visible = [g for t, g in zip(s["t"], s["gyro"]) if x0 <= t <= x1]
    if visible:
        sorted_g = sorted(visible)
        p99 = sorted_g[int(0.99 * (len(sorted_g) - 1))]
        ax.set_ylim(0, max(p99 * 1.25, 0.02))
    _shade(ax); _drop(ax)
    ax.set_ylabel("angular speed (rad/s)")
    ax.set_xlabel(r"time $t$ (s)")
    ax.grid(True)

    # Focus the x-axis.
    for ax in axes:
        ax.set_xlim(x0, x1)

    # Caption only, no subfigure label — the file name distinguishes them.
    fig.text(0.014, 0.978, profile_caption,
             fontsize=10.5, va="top", ha="left", color="#222")

    fig.tight_layout(rect=[0, 0, 1, 0.955])
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=300)
    plt.close(fig)


# -----------------------------------------------------------------------------
# Comparison plot
# -----------------------------------------------------------------------------
COMPARE_METRICS = [
    ("alt_std_m",       r"alt std (m)"),
    ("pos_rms_north_m", r"pos$_N$ RMS (m)"),
    ("pos_rms_east_m",  r"pos$_E$ RMS (m)"),
    ("roll_rms_rad",    r"roll RMS (rad)"),
    ("pitch_rms_rad",   r"pitch RMS (rad)"),
    ("gyro_rms",        "angular speed RMS (rad/s)"),
]


def _read_csv_metric(rows: list[dict], key: str) -> list[float]:
    out: list[float] = []
    for r in rows:
        v = r.get(key, "")
        if v in ("", None, "None"):
            continue
        try:
            out.append(float(v))
        except (TypeError, ValueError):
            continue
    return out


def _draw_comparison(
    calm_rows: list[dict],
    worst_rows: list[dict],
    out_path: Path,
) -> None:
    """Box-and-strip layout per metric: mean line (heavy black), IQR box,
    raw points overlaid as jittered scatter. Calm-derived gate as a thin
    grey line. Box edges remain the 25th/75th percentile by definition;
    only the central line was changed from median → mean per user request."""
    import matplotlib.pyplot as plt
    import random

    fig, axes = plt.subplots(2, 3, figsize=(9.5, 5.6))
    axes = axes.ravel()

    batches = [
        ("calm",       calm_rows,  WONG["bluish_green"]),
        ("worst case", worst_rows, WONG["vermillion"]),
    ]
    rng = random.Random(0)

    for ax, (key, label) in zip(axes, COMPARE_METRICS):
        all_data: list[list[float]] = []
        for _, rows, _ in batches:
            all_data.append(_read_csv_metric(rows, key))

        positions = list(range(1, len(batches) + 1))
        bp = ax.boxplot(
            all_data,
            positions=positions,
            widths=0.45,
            showfliers=False,
            patch_artist=True,
            # Hide the default median line and draw a mean line in its place.
            medianprops=dict(linewidth=0),
            showmeans=True, meanline=True,
            meanprops=dict(color="black", linewidth=1.4),
            whiskerprops=dict(linewidth=0.8, color="#444"),
            capprops=dict(linewidth=0.8, color="#444"),
        )
        for patch, (_, _, color) in zip(bp["boxes"], batches):
            patch.set_facecolor(color)
            patch.set_alpha(0.22)
            patch.set_edgecolor(color)
            patch.set_linewidth(1.0)

        # Raw points (jittered).
        for x, vals, (_, _, color) in zip(positions, all_data, batches):
            for v in vals:
                jx = x + (rng.random() - 0.5) * 0.18
                ax.plot(jx, v, "o", markersize=3.0,
                        markerfacecolor=color, markeredgecolor="white",
                        markeredgewidth=0.5, alpha=0.9)

        # Calm gate dashed line (subtle grey, not red).
        if key in metrics_mod.CALM_GATES:
            cmp, threshold = metrics_mod.CALM_GATES[key]
            ax.axhline(threshold, color="#666", linestyle="--",
                       linewidth=0.7, alpha=0.85)
            ax.text(0.985, threshold, f"  gate {cmp} {threshold:g}",
                    transform=ax.get_yaxis_transform(),
                    fontsize=7.5, va="bottom", ha="right", color="#666")

        ax.set_xticks(positions)
        ax.set_xticklabels([b[0] for b in batches])
        ax.set_ylabel(label)
        ax.grid(True, axis="y")
        ax.set_xlim(0.5, len(batches) + 0.5)

    fig.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=300)
    plt.close(fig)


# -----------------------------------------------------------------------------
# Radar (spider) comparison — alternative to the box-and-strip plot.
# All 6 metrics on one polar panel. Each axis is `log10(value / gate)`:
#   0   = exactly at the gate (the dashed grey reference ring)
#  -1   = 10× under the gate (passing comfortably)
#  +1   = 10× over the gate (failing badly)
# So calm forms a small inner polygon (every axis < 0), worst case bulges
# outward along the most-affected metrics. Per-run dots on each axis show
# the within-batch spread.
# -----------------------------------------------------------------------------
def _draw_comparison_radar(
    calm_rows: list[dict],
    worst_rows: list[dict],
    out_path: Path,
) -> None:
    import math as _math
    import matplotlib.pyplot as plt

    metric_keys   = [k for k, _ in COMPARE_METRICS]
    metric_labels = [lbl for _, lbl in COMPARE_METRICS]

    # Pull the gate threshold for each metric. CALM_GATES uses (cmp, threshold).
    gates: list[float] = []
    for k in metric_keys:
        cmp_op, thr = metrics_mod.CALM_GATES[k]
        gates.append(float(thr))

    def _norm(values: list[float], gate: float) -> list[float]:
        # log10(value / gate); guard against zeros / negatives.
        return [_math.log10(max(v, 1e-12) / gate) for v in values]

    n = len(metric_keys)
    angles = [2 * _math.pi * i / n for i in range(n)]
    angles_closed = angles + [angles[0]]

    fig = plt.figure(figsize=(7.2, 7.0))
    ax = fig.add_subplot(111, projection="polar")
    ax.set_theta_offset(_math.pi / 2)   # first axis points up
    ax.set_theta_direction(-1)          # clockwise

    # Determine the radial range so all data + gate ring are visible.
    all_logs: list[float] = []
    per_metric_calm:  list[list[float]] = []
    per_metric_worst: list[list[float]] = []
    for k, gate in zip(metric_keys, gates):
        c_vals = _read_csv_metric(calm_rows,  k)
        w_vals = _read_csv_metric(worst_rows, k)
        per_metric_calm.append(_norm(c_vals,  gate))
        per_metric_worst.append(_norm(w_vals, gate))
        all_logs.extend(per_metric_calm[-1])
        all_logs.extend(per_metric_worst[-1])
    r_min = min(min(all_logs) - 0.2, -0.6)
    r_max = max(max(all_logs) + 0.2,  1.4)
    ax.set_ylim(r_min, r_max)

    # Gate ring at r=0 (= log10(value/gate) == 0 means value == gate).
    ax.plot(angles_closed, [0.0] * (n + 1),
            color="#666", linestyle="--", linewidth=0.9, alpha=0.85)

    def _polygon(per_metric: list[list[float]], color: str, label: str):
        # Median per axis, closed for plotting.
        meds = [statistics.median(v) if v else 0.0 for v in per_metric]
        meds_closed = meds + [meds[0]]
        ax.plot(angles_closed, meds_closed,
                color=color, linewidth=1.6, label=label, zorder=4)
        ax.fill(angles_closed, meds_closed, color=color, alpha=0.18, zorder=3)
        # Per-run dots on each axis.
        for ang, vals in zip(angles, per_metric):
            for v in vals:
                ax.plot(ang, v, "o", markersize=3.5,
                        markerfacecolor=color, markeredgecolor="white",
                        markeredgewidth=0.5, alpha=0.85, zorder=5)

    _polygon(per_metric_calm,  WONG["bluish_green"], "calm")
    _polygon(per_metric_worst, WONG["vermillion"],   "worst case")

    # Tick labels at each axis position (the metric name).
    ax.set_xticks(angles)
    ax.set_xticklabels(metric_labels, fontsize=9.5)

    # Radial ticks at log10 = -1, 0, 1 (= 0.1×, 1×, 10× gate).
    radial_ticks = [t for t in (-1.0, -0.5, 0.0, 0.5, 1.0) if r_min <= t <= r_max]
    ax.set_yticks(radial_ticks)
    ax.set_yticklabels(
        [{-1.0: r"$0.1\times$ gate",
          -0.5: r"$0.3\times$",
           0.0: r"gate",
           0.5: r"$3\times$",
           1.0: r"$10\times$ gate"}[t] for t in radial_ticks],
        fontsize=8, color="#444",
    )
    ax.tick_params(axis="y", pad=2)
    ax.grid(True, color="#bbb", linewidth=0.4, alpha=0.6)
    ax.spines["polar"].set_color("#888")
    ax.spines["polar"].set_linewidth(0.6)

    ax.legend(loc="upper right", bbox_to_anchor=(1.18, 1.10),
              frameon=False, fontsize=9.5)

    fig.text(0.5, 0.965,
             "Per-metric degradation relative to the calm-derived gate (log scale)",
             ha="center", fontsize=10.5, color="#222")
    fig.tight_layout(rect=[0, 0, 1, 0.94])
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=300)
    plt.close(fig)


# -----------------------------------------------------------------------------
# Bar comparison — classic mean ± std bar chart with raw points overlaid.
# Six panels (one per metric). Each panel has two bars (calm, worst case) at
# bar height = batch mean, error bars = ±1 std. Individual runs shown as
# small dots on top of each bar so the reader still sees the spread.
# -----------------------------------------------------------------------------
def _draw_comparison_bars(
    calm_rows: list[dict],
    worst_rows: list[dict],
    out_path: Path,
) -> None:
    import matplotlib.pyplot as plt
    import random

    fig, axes = plt.subplots(2, 3, figsize=(9.5, 5.6))
    axes = axes.ravel()

    batches = [
        ("calm",       calm_rows,  WONG["bluish_green"]),
        ("worst case", worst_rows, WONG["vermillion"]),
    ]
    rng = random.Random(0)

    for ax, (key, label) in zip(axes, COMPARE_METRICS):
        positions: list[int] = []
        means: list[float] = []
        stds: list[float] = []
        all_vals: list[list[float]] = []
        colors: list[str] = []

        for i, (_, rows, color) in enumerate(batches):
            vals = _read_csv_metric(rows, key)
            if not vals:
                vals = [0.0]
            positions.append(i + 1)
            means.append(statistics.mean(vals))
            stds.append(statistics.pstdev(vals) if len(vals) > 1 else 0.0)
            all_vals.append(vals)
            colors.append(color)

        # Bars: filled with light tint, edge in saturated color, error bar = ±1 std.
        ax.bar(
            positions, means,
            width=0.6,
            color=[c for c in colors],
            alpha=0.28,
            edgecolor=colors,
            linewidth=1.2,
            yerr=stds,
            capsize=4,
            error_kw=dict(elinewidth=0.9, ecolor="#444"),
            zorder=2,
        )

        # Per-run dots overlaid on each bar (jittered horizontally).
        for x, vals, color in zip(positions, all_vals, colors):
            for v in vals:
                jx = x + (rng.random() - 0.5) * 0.18
                ax.plot(jx, v, "o", markersize=3.0,
                        markerfacecolor=color, markeredgecolor="white",
                        markeredgewidth=0.5, alpha=0.9, zorder=4)

        # Calm gate dashed line.
        if key in metrics_mod.CALM_GATES:
            cmp, threshold = metrics_mod.CALM_GATES[key]
            ax.axhline(threshold, color="#666", linestyle="--",
                       linewidth=0.7, alpha=0.85, zorder=1)
            ax.text(0.985, threshold, f"  gate {cmp} {threshold:g}",
                    transform=ax.get_yaxis_transform(),
                    fontsize=7.5, va="bottom", ha="right", color="#666")

        ax.set_xticks(positions)
        ax.set_xticklabels([b[0] for b in batches])
        ax.set_ylabel(label)
        ax.grid(True, axis="y")
        ax.set_xlim(0.4, len(batches) + 0.6)
        ax.set_ylim(bottom=0)

    fig.text(0.5, 0.985,
             "Mean $\\pm$ 1 std across runs (dots = individual runs)",
             ha="center", fontsize=10.5, color="#222")
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=300)
    plt.close(fig)


# -----------------------------------------------------------------------------
# Driver
# -----------------------------------------------------------------------------
def _pick_default(dirpath: Path, contains: str) -> Path | None:
    if not dirpath.is_dir():
        return None
    candidates = sorted(p for p in dirpath.glob("*.jsonl") if contains in p.stem)
    return candidates[0] if candidates else None


def main(argv: list[str] | None = None) -> int:
    repo = Path(__file__).resolve().parents[2]   # Swarm_Drones/
    bench = repo / "reports" / "benchmark_hover_report"
    default_calm_dir  = bench / "calm_baseline_runs"
    default_worst_dir = bench / "batch_worst_case"

    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--calm-log", type=Path,
                   default=_pick_default(default_calm_dir, "CALM"),
                   help="single calm flight log to render (default: first CALM*.jsonl)")
    p.add_argument("--worst-log", type=Path,
                   default=_pick_default(default_worst_dir, "WORST3_SUCCESS")
                           or _pick_default(default_worst_dir, "WORST"),
                   help="single worst_case log (default: first WORST3_SUCCESS, else first WORST)")
    p.add_argument("--calm-csv", type=Path,
                   default=default_calm_dir / "report" / "baseline.csv")
    p.add_argument("--worst-csv", type=Path,
                   default=default_worst_dir / "report" / "baseline.csv")
    p.add_argument("--out", type=Path, default=bench / "paper_figs")
    args = p.parse_args(argv)

    if args.calm_log is None or not args.calm_log.is_file():
        print(f"error: cannot find calm log ({args.calm_log})", file=sys.stderr)
        return 2
    if args.worst_log is None or not args.worst_log.is_file():
        print(f"error: cannot find worst log ({args.worst_log})", file=sys.stderr)
        return 2
    for csv_path in (args.calm_csv, args.worst_csv):
        if not csv_path.is_file():
            print(f"error: missing {csv_path}", file=sys.stderr)
            return 2

    _apply_paper_style()

    args.out.mkdir(parents=True, exist_ok=True)

    # (a) calm
    log_calm = flight_logging.load(args.calm_log)
    _draw_run(
        log_calm,
        args.out / "calm_run.png",
        profile_caption="Calm hover — no disturbance applied.",
        drop_t=None,
    )
    print(f"  wrote {args.out / 'calm_run.png'}")

    # (b) worst
    log_worst = flight_logging.load(args.worst_log)
    drop_ev = log_worst.find_event("mass_drop_event")
    drop_t = float(drop_ev["t"]) if drop_ev else None
    _draw_run(
        log_worst,
        args.out / "worst_case_run.png",
        profile_caption=("Worst-case stack: 5 m/s wind, 0.300 kg payload "
                         "released at $t=t_d$ (dashed), IMU noise."),
        drop_t=drop_t,
    )
    print(f"  wrote {args.out / 'worst_case_run.png'}")

    # (c) comparison
    with args.calm_csv.open() as f:
        calm_rows = list(csv.DictReader(f))
    with args.worst_csv.open() as f:
        worst_rows = list(csv.DictReader(f))
    _draw_comparison(calm_rows, worst_rows, args.out / "comparison.png")
    print(f"  wrote {args.out / 'comparison.png'}")
    _draw_comparison_radar(calm_rows, worst_rows, args.out / "comparison_2.png")
    print(f"  wrote {args.out / 'comparison_2.png'}")
    _draw_comparison_bars(calm_rows, worst_rows, args.out / "comparison_3.png")
    print(f"  wrote {args.out / 'comparison_3.png'}")

    print(f"\nused logs:")
    print(f"  calm:  {args.calm_log.name}")
    print(f"  worst: {args.worst_log.name}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
