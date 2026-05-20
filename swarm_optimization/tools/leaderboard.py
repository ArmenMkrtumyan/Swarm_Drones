"""
Generate algorithm leaderboards: per-track PNGs, aggregated CSVs, and a
combined markdown table.

Two modes:

  (no --csv)    Reads all three archived sweep CSVs, writes:
                  outputs/leaderboards/png/leaderboard_<slug>.png  (one PNG per track + cross-track)
                  outputs/leaderboards/csv/leaderboard_<slug>.csv
                  outputs/leaderboards/leaderboards.md
                Prints a cross-track summary to stdout.

  --csv PATH    Generates a single PNG from one sweep CSV.  Useful after a
                new run to preview results before promoting them.

Usage:
    python tools/leaderboard.py                          # regenerate everything
    python tools/leaderboard.py --csv outputs/classical/eval_track1_results_new.csv
    python tools/leaderboard.py --csv outputs/classical/eval_track1_results_new.csv \\
                                --out outputs/classical/leaderboard_new.png
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
import pandas as pd


# ---------------------------------------------------------------------------
# Shared metric definitions (used by both PNG and table generators)
# ---------------------------------------------------------------------------

# (display label, csv field, higher-is-better, value formatter)
PNG_METRICS = [
    ("Final coverage", "final_coverage", True,  lambda v: f"{v * 100:.1f}%"),
    ("Time → 100%",    "time_to_100",    False, lambda v: f"{v:.0f}s"),
    ("Overlap area",   "overlap_m2",     False, lambda v: f"{v:.0f}m²"),
    ("Wasted visits",  "wasted_visits",  False, lambda v: f"{v:.0f}"),
    ("Energy used",    "total_energy_j", False, lambda v: f"{v / 1000:.0f}kJ"),
]

TIME_PENALTY_S = 1000.0   # applied to non-completing runs in table mode


# ---------------------------------------------------------------------------
# PNG generation (single CSV → one leaderboard image)
# ---------------------------------------------------------------------------

def _aggregate_png(rows: list[dict]) -> dict[str, dict[str, float]]:
    """Mean of each metric per policy; time_to_* inf values fall back to time_to_terminal."""
    by_policy: dict[str, dict[str, list[float]]] = defaultdict(lambda: defaultdict(list))
    for r in rows:
        policy = r["policy"]
        terminal = float(r.get("time_to_terminal", 0.0))
        for _, field, _, _ in PNG_METRICS:
            v_raw = r.get(field, "")
            if v_raw == "" or v_raw is None:
                continue
            try:
                v = float(v_raw)
            except (ValueError, TypeError):
                continue
            if not np.isfinite(v):
                if field.startswith("time_to_"):
                    v = terminal
                else:
                    continue
            by_policy[policy][field].append(v)
    return {p: {k: float(np.mean(vs)) for k, vs in fields.items() if vs}
            for p, fields in by_policy.items()}


def _render_leaderboard_png(rows: list[dict], out_path: Path, title: str | None = None) -> None:
    """Render a colour-coded leaderboard PNG from pre-loaded CSV rows."""
    agg = _aggregate_png(rows)

    active_metrics = [
        m for m in PNG_METRICS
        if sum(1 for pd_ in agg.values() if m[1] in pd_) >= 2
    ]
    if not active_metrics:
        raise ValueError("no active metrics in CSV")

    policies = sorted(agg.keys())
    if "Random" in policies:
        policies = ["Random"] + [p for p in policies if p != "Random"]

    n_p, n_m = len(policies), len(active_metrics)

    raw = np.full((n_p, n_m), np.nan)
    for j, (_, field, _, _) in enumerate(active_metrics):
        for i, p in enumerate(policies):
            if field in agg[p]:
                raw[i, j] = agg[p][field]

    CLOSE_THRESHOLD = 0.05
    score = np.zeros_like(raw, dtype=float)
    is_best = np.zeros_like(raw, dtype=bool)
    for j, (_, field, hib, _) in enumerate(active_metrics):
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

    fig_w = max(8.0, 1.6 * n_m + 2.5)
    fig_h = max(3.5, 0.55 * n_p + 1.8)
    fig, ax = plt.subplots(figsize=(fig_w, fig_h))

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

    for i in range(n_p):
        for j in range(n_m):
            if np.isnan(raw[i, j]):
                continue
            txt = active_metrics[j][3](raw[i, j])
            ax.text(j, i, txt, ha="center", va="center",
                    color="black", fontsize=9,
                    fontweight="bold" if is_best[i, j] else "normal")

    ax.set_xticks(np.arange(-0.5, n_m, 1), minor=True)
    ax.set_yticks(np.arange(-0.5, n_p, 1), minor=True)
    ax.grid(which="minor", color="#cccccc", linewidth=0.5)
    ax.tick_params(which="minor", length=0)

    ax.set_title(title or "Algorithm leaderboard",
                 fontsize=12, fontweight="bold", pad=12)
    fig.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=140, bbox_inches="tight")
    plt.close(fig)
    print(f"Wrote: {out_path}")


def build_leaderboard_png(csv_path: Path, out_path: Path, title: str | None = None) -> None:
    """Render a colour-coded leaderboard PNG from a sweep CSV file."""
    rows = list(csv.DictReader(open(csv_path)))
    if not rows:
        raise ValueError(f"empty CSV: {csv_path}")
    _render_leaderboard_png(rows, out_path, title=title or f"Algorithm leaderboard — {csv_path.name}")


# ---------------------------------------------------------------------------
# Table generation (all three CSVs → markdown + aggregated CSVs)
# ---------------------------------------------------------------------------

def _load_all() -> pd.DataFrame:
    df1 = pd.read_csv("outputs/classical/sweep_track1_results.csv");     df1["track"] = "1"
    df2 = pd.read_csv("outputs/metaheuristic/sweep_track2_results.csv"); df2["track"] = "2"
    df3 = pd.read_csv("outputs/control_based/sweep_track3_results.csv"); df3["track"] = "3"
    combined = pd.concat([df1, df2, df3], ignore_index=True)
    # Random appears in all three; keep Track-1's run only to avoid 3× counting.
    keep = ~((combined["policy"] == "Random") & (combined["track"] != "1"))
    combined = combined[keep].reset_index(drop=True)
    combined["time_to_100"] = combined["time_to_100"].replace([np.inf, -np.inf], np.nan)
    combined["time_to_80"]  = combined["time_to_80"].replace([np.inf, -np.inf], np.nan)
    return combined


def _aggregate_tables(df: pd.DataFrame) -> pd.DataFrame:
    return df.groupby("policy").agg(
        coverage_pct=("final_coverage", lambda s: float(s.mean()) * 100),
        time_to_100_s=("time_to_100", lambda s: float(s.fillna(TIME_PENALTY_S).mean())),
        time_to_80_s=("time_to_80",  lambda s: float(s.fillna(TIME_PENALTY_S).mean())),
        energy_kJ=("total_energy_j", lambda s: float(s.mean()) / 1000.0),
        overlap_m2=("overlap_m2", "mean"),
        wasted_visits=("wasted_visits", "mean"),
        completion_rate=("reason", lambda s: float((s == "COMPLETED").mean())),
        n_runs=("policy", "size"),
    ).round(2)


def _rank_sum(agg: pd.DataFrame) -> pd.Series:
    r = pd.DataFrame(index=agg.index)
    r["c"] = agg["coverage_pct"].rank(ascending=False, method="min")
    r["t"] = agg["time_to_100_s"].rank(ascending=True,  method="min")
    r["e"] = agg["energy_kJ"].rank(ascending=True,  method="min")
    r["o"] = agg["overlap_m2"].rank(ascending=True,  method="min")
    r["w"] = agg["wasted_visits"].rank(ascending=True,  method="min")
    return r.sum(axis=1).sort_values()


def _md_table(agg: pd.DataFrame, ranks: pd.Series, title: str) -> str:
    lines = [f"### {title}", "",
             "| Rank | Policy | Cov % | Time→100 (s) | Time→80 (s) | "
             "Energy (kJ) | Overlap (m²) | Wasted visits | Completion |",
             "|---|---|---|---|---|---|---|---|---|"]
    for i, name in enumerate(ranks.index, 1):
        a = agg.loc[name]
        lines.append(
            f"| {i} | **{name}** | {a['coverage_pct']:.1f} | "
            f"{a['time_to_100_s']:.0f} | {a['time_to_80_s']:.0f} | "
            f"{a['energy_kJ']:.0f} | {a['overlap_m2']:.0f} | "
            f"{a['wasted_visits']:.0f} | {a['completion_rate']*100:.0f} % |"
        )
    return "\n".join(lines)


def _generate_all_leaderboards() -> None:
    """Generate PNGs for all three tracks + cross-track, plus markdown tables and CSVs."""
    out_dir  = Path("outputs/leaderboards")
    png_dir  = out_dir / "png"
    csv_dir  = out_dir / "csv"
    png_dir.mkdir(parents=True, exist_ok=True)
    csv_dir.mkdir(parents=True, exist_ok=True)

    track_configs = [
        ("outputs/classical/sweep_track1_results.csv",
         png_dir / "leaderboard_classical.png",
         "Classical"),
        ("outputs/metaheuristic/sweep_track2_results.csv",
         png_dir / "leaderboard_metaheuristic.png",
         "Metaheuristic"),
        ("outputs/control_based/sweep_track3_results.csv",
         png_dir / "leaderboard_control_based.png",
         "Learning / Control"),
    ]
    for csv_p, png_p, title in track_configs:
        build_leaderboard_png(Path(csv_p), png_p, title=title)

    all_df = _load_all()

    # Cross-track leaderboard PNG (all 14 algorithms on one image).
    import io as _io
    _buf = _io.StringIO()
    all_df.to_csv(_buf, index=False)
    _buf.seek(0)
    _render_leaderboard_png(
        rows=list(csv.DictReader(_buf)),
        out_path=png_dir / "leaderboard_all.png",
        title="Cross-track — all 14 algorithms",
    )

    sections = {
        "classical":     ("Classical",                   all_df[all_df["track"] == "1"]),
        "metaheuristic": ("Metaheuristic",               all_df[all_df["track"] == "2"]),
        "control_based": ("Learning / Control",          all_df[all_df["track"] == "3"]),
        "all":           ("Cross-track (all 14 algorithms)", all_df),
    }

    md_parts: list[str] = [
        "# Multi-metric leaderboards", "",
        "Generated by `tools/leaderboard.py` from the BO-tuned sweep CSVs.",
        "",
        "Each row is the **mean across 36 runs** (3 maps × 4 swarm sizes × 3 seeds). "
        "Rank-sum aggregates 5 ranking metrics; lowest wins. "
        f"Non-completing runs are penalised at {TIME_PENALTY_S:.0f} s on time-to-100.",
        "",
    ]

    for slug, (title, sub_df) in sections.items():
        agg = _aggregate_tables(sub_df)
        ranks = _rank_sum(agg)
        agg_sorted = agg.loc[ranks.index]
        agg_sorted.to_csv(csv_dir / f"leaderboard_{slug}.csv")
        md_parts.append(_md_table(agg_sorted, ranks, title))
        md_parts.append("")

    md_path = out_dir / "leaderboards.md"
    md_path.write_text("\n".join(md_parts))
    print(f"Wrote {md_path}")
    print(f"Wrote CSVs to {csv_dir}/  |  PNGs to {png_dir}/")

    cross_agg   = _aggregate_tables(all_df)
    cross_ranks = _rank_sum(cross_agg)
    print()
    print(_md_table(cross_agg.loc[cross_ranks.index], cross_ranks,
                    "Cross-track (all 14 algorithms)"))


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--csv", type=str, default=None,
                   help="path to a single sweep_*_results.csv (single-PNG mode)")
    p.add_argument("--out", type=str, default=None,
                   help="output PNG path (single-PNG mode only; "
                        "defaults to leaderboard.png next to the CSV)")
    p.add_argument("--title", type=str, default=None,
                   help="custom title for the PNG (single-PNG mode only)")
    return p.parse_args()


def main() -> None:
    args = _parse_args()

    if args.csv:
        csv_path = Path(args.csv).resolve()
        if args.out:
            out_path = Path(args.out).resolve()
        else:
            out_root = csv_path.parent
            while out_root.name and not out_root.name.startswith("outputs"):
                out_root = out_root.parent
            if not out_root.name:
                out_root = csv_path.parent
            out_path = out_root / "leaderboard.png"
        build_leaderboard_png(csv_path, out_path, title=args.title)
    else:
        _generate_all_leaderboards()


if __name__ == "__main__":
    main()
