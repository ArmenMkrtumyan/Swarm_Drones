"""Curated comparison plot across multiple hover-benchmark batches.

Each batch is one folder containing a `baseline.csv` produced by
`lab.control.benchmark_hover`. This tool reads N batches and writes a
single comparison plot at <out>/comparison.png + <out>/comparison.svg
(side-by-side, no png/svg subdir split since there's only one file each)
showing median + min/max range per metric per batch — ideal for the
calm-vs-worst_case-vs-RL story.

Usage:
    python -m lab.control.compare_batches \\
        reports/benchmark_hover_report/calm_baseline_runs/report \\
        reports/benchmark_hover_report/batch_worst_case \\
        --labels calm worst_case \\
        --out reports/benchmark_hover_report

    # Later, with RL:
    python -m lab.control.compare_batches \\
        reports/benchmark_hover_report/calm_baseline_runs/report \\
        reports/benchmark_hover_report/batch_worst_case \\
        reports/benchmark_hover_report/batch_rl \\
        --labels calm worst_case rl \\
        --out reports/benchmark_hover_report
"""
from __future__ import annotations

import argparse
import csv
import statistics
import sys
from pathlib import Path

from lab.control import plot_style
from lab.control.metrics import CALM_GATES


METRICS = [
    ("alt_std_m",       "alt std (m)"),
    ("pos_rms_north_m", "north RMS |pos| (m)"),
    ("pos_rms_east_m",  "east RMS |pos| (m)"),
    ("roll_rms_rad",    "roll RMS (rad)"),
    ("pitch_rms_rad",   "pitch RMS (rad)"),
    ("gyro_rms",        "gyro RMS (rad/s)"),
]


def _read_metric(rows: list[dict], key: str) -> list[float]:
    out = []
    for r in rows:
        v = r.get(key, "")
        if v in ("", None, "None"):
            continue
        try:
            out.append(float(v))
        except (TypeError, ValueError):
            continue
    return out


def load_batch(batch_dir: Path) -> list[dict]:
    csv_path = batch_dir / "baseline.csv"
    if not csv_path.is_file():
        raise FileNotFoundError(f"missing {csv_path}")
    with csv_path.open() as f:
        return list(csv.DictReader(f))


def plot(batches: list[tuple[str, list[dict]]], out_path: Path) -> None:
    """One subplot per metric. Within each, one bar per batch with min-max range."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    n = len(METRICS)
    fig, axes = plt.subplots(2, 3, figsize=(13, 7))

    # Color per batch: calm green, others rotate orange/red/etc
    palette = [
        plot_style.COLORS["bar_calm"],
        plot_style.COLORS["bar_disturbed"],
        plot_style.COLORS["roll"],
        plot_style.COLORS["pitch"],
        plot_style.COLORS["gyro_mag"],
    ]
    labels = [lbl for lbl, _ in batches]
    colors = [palette[i % len(palette)] for i in range(len(batches))]

    for ax, (mkey, mtitle) in zip(axes.ravel(), METRICS):
        medians = []
        mins = []
        maxs = []
        for _, rows in batches:
            vals = _read_metric(rows, mkey)
            if not vals:
                medians.append(0.0); mins.append(0.0); maxs.append(0.0)
                continue
            medians.append(statistics.median(vals))
            mins.append(min(vals))
            maxs.append(max(vals))

        x = list(range(len(batches)))
        # Asymmetric error bars: [median-min, max-median]
        yerr_low = [med - mn for med, mn in zip(medians, mins)]
        yerr_high = [mx - med for med, mx in zip(medians, maxs)]
        ax.bar(x, medians, color=colors,
               yerr=[yerr_low, yerr_high], capsize=6, ecolor="#444")
        ax.set_xticks(x)
        ax.set_xticklabels(labels, rotation=20, ha="right")
        ax.set_title(mtitle)
        ax.grid(True, axis="y", alpha=0.3)

        # Calm gate line — clarifies "this is the bar".
        if mkey in CALM_GATES:
            cmp, threshold = CALM_GATES[mkey]
            ax.axhline(threshold, color=plot_style.COLORS["gate_fail_edge"],
                       linestyle="--", linewidth=1.0, alpha=0.7)
            ax.text(
                0.99, threshold, f" gate {cmp} {threshold:g}",
                transform=ax.get_yaxis_transform(),
                fontsize=7.5, va="bottom", ha="right",
                color=plot_style.COLORS["gate_fail_edge"],
            )

        for xi, med in zip(x, medians):
            ax.text(xi, med, f"{med:.3g}", ha="center", va="bottom", fontsize=8)

    fig.suptitle("Hover quality: median + min/max across runs per batch",
                 fontsize=12)
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    plot_style.savefig_dual(fig, out_path)
    plt.close(fig)


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument(
        "batch_dirs", nargs="+", type=Path,
        help="benchmark dirs, each containing baseline.csv",
    )
    p.add_argument(
        "--labels", nargs="+", default=None,
        help="display label per batch (defaults to dir name)",
    )
    p.add_argument(
        "--out", type=Path, default=Path("reports/benchmark_hover_report"),
        help="output dir (writes png/comparison.png + svg/comparison.svg)",
    )
    args = p.parse_args(argv)

    if args.labels and len(args.labels) != len(args.batch_dirs):
        print("error: --labels count must match batch_dirs count", file=sys.stderr)
        return 2

    batches: list[tuple[str, list[dict]]] = []
    for i, d in enumerate(args.batch_dirs):
        try:
            rows = load_batch(d)
        except FileNotFoundError as e:
            print(f"error: {e}", file=sys.stderr)
            return 2
        label = args.labels[i] if args.labels else d.name
        batches.append((label, rows))
        print(f"  {label}: {len(rows)} runs from {d}")

    args.out.mkdir(parents=True, exist_ok=True)
    # Write directly at out_dir (no png/svg subdirs). savefig_dual sees no
    # `png/` component in the path and falls back to writing the SVG sibling
    # in the same directory.
    plot(batches, args.out / "comparison.png")
    print(f"\nwrote {args.out / 'comparison.png'}  +  "
          f"{args.out / 'comparison.svg'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
