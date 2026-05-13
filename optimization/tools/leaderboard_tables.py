"""
Build multi-metric leaderboards from the three sweep CSVs (markdown tables).

Companion to `tools/leaderboard.py`, which generates per-track PNG leaderboards.
This tool produces:
    outputs/leaderboards/leaderboards.md           combined doc
    outputs/leaderboards/leaderboard_*.csv         per-track aggregated CSVs

Reads:
    outputs/sweep_track1_results.csv
    outputs/sweep_track2_results.csv
    outputs/sweep_results.csv                     (Track 3)

Metrics scored per policy (mean across all 36 cells = 3 maps × 4 swarm × 3 seeds):
    coverage_pct          (higher = better)
    time_to_100_s         (lower = better; non-completing runs penalized — see below)
    time_to_80_s          (lower = better)
    energy_kJ             (lower = better)
    overlap_m2            (lower = better)
    wasted_visits         (lower = better)
    completion_rate       (fraction of runs that hit 100 % — informational only)

Rank-sum: sum of per-metric ranks (1 = best). Lowest rank-sum wins overall.
Runs that didn't reach 100 % coverage get `TIME_PENALTY_S = 1000` for the
time-to-100 metric (longer than the 734 s physics-bounded max sim time), so
non-completing runs rank below completing ones without losing rank-tie info.
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import numpy as np
import pandas as pd


TIME_PENALTY_S = 1000.0


def load_all() -> pd.DataFrame:
    df1 = pd.read_csv("outputs/sweep_track1_results.csv"); df1["track"] = "1"
    df2 = pd.read_csv("outputs/sweep_track2_results.csv"); df2["track"] = "2"
    df3 = pd.read_csv("outputs/sweep_results.csv");         df3["track"] = "3"
    combined = pd.concat([df1, df2, df3], ignore_index=True)
    # Random appears in all three; keep Track-1's run only to avoid 3× counting.
    keep = ~((combined["policy"] == "Random") & (combined["track"] != "1"))
    combined = combined[keep].reset_index(drop=True)
    # Normalize inf → NaN so we can apply the explicit penalty consistently.
    combined["time_to_100"] = combined["time_to_100"].replace(
        [np.inf, -np.inf], np.nan
    )
    combined["time_to_80"] = combined["time_to_80"].replace(
        [np.inf, -np.inf], np.nan
    )
    return combined


def aggregate(df: pd.DataFrame) -> pd.DataFrame:
    """Per-policy mean of every metric we rank by."""
    return df.groupby("policy").agg(
        coverage_pct=("final_coverage", lambda s: float(s.mean()) * 100),
        time_to_100_s=(
            "time_to_100", lambda s: float(s.fillna(TIME_PENALTY_S).mean())
        ),
        time_to_80_s=(
            "time_to_80", lambda s: float(s.fillna(TIME_PENALTY_S).mean())
        ),
        energy_kJ=("total_energy_j", lambda s: float(s.mean()) / 1000.0),
        overlap_m2=("overlap_m2", "mean"),
        wasted_visits=("wasted_visits", "mean"),
        completion_rate=("reason", lambda s: float((s == "COMPLETED").mean())),
        n_runs=("policy", "size"),
    ).round(2)


def rank_sum(agg: pd.DataFrame) -> pd.Series:
    """Sum of per-metric ranks; lowest wins."""
    r = pd.DataFrame(index=agg.index)
    r["c"] = agg["coverage_pct"].rank(ascending=False, method="min")
    r["t"] = agg["time_to_100_s"].rank(ascending=True, method="min")
    r["e"] = agg["energy_kJ"].rank(ascending=True, method="min")
    r["o"] = agg["overlap_m2"].rank(ascending=True, method="min")
    r["w"] = agg["wasted_visits"].rank(ascending=True, method="min")
    return r.sum(axis=1).sort_values()


def md_table(agg: pd.DataFrame, ranks: pd.Series, title: str) -> str:
    out = [f"### {title}", ""]
    out.append(
        "| Rank | Policy | Cov % | Time→100 (s) | Time→80 (s) | "
        "Energy (kJ) | Overlap (m²) | Wasted visits | Completion |"
    )
    out.append("|---|---|---|---|---|---|---|---|---|")
    for i, name in enumerate(ranks.index, 1):
        a = agg.loc[name]
        out.append(
            f"| {i} | **{name}** | {a['coverage_pct']:.1f} | "
            f"{a['time_to_100_s']:.0f} | {a['time_to_80_s']:.0f} | "
            f"{a['energy_kJ']:.0f} | {a['overlap_m2']:.0f} | "
            f"{a['wasted_visits']:.0f} | {a['completion_rate']*100:.0f} % |"
        )
    return "\n".join(out)


def per_metric_md(agg: pd.DataFrame, title: str) -> str:
    """Per-metric ranking columns (1 = best on that metric)."""
    rc = agg["coverage_pct"].rank(ascending=False, method="min").astype(int)
    rt = agg["time_to_100_s"].rank(ascending=True, method="min").astype(int)
    re_ = agg["energy_kJ"].rank(ascending=True, method="min").astype(int)
    ro = agg["overlap_m2"].rank(ascending=True, method="min").astype(int)
    rw = agg["wasted_visits"].rank(ascending=True, method="min").astype(int)
    order = (rc + rt + re_ + ro + rw).sort_values().index
    out = [f"### {title} — per-metric ranks (1 = best on that metric)", ""]
    out.append("| Policy | Cov | Time→100 | Energy | Overlap | Wasted |")
    out.append("|---|---|---|---|---|---|")
    for name in order:
        out.append(
            f"| {name} | {rc[name]} | {rt[name]} | {re_[name]} | "
            f"{ro[name]} | {rw[name]} |"
        )
    return "\n".join(out)


def main() -> None:
    out_dir = Path("outputs/leaderboards")
    out_dir.mkdir(parents=True, exist_ok=True)

    all_df = load_all()

    sections = {
        "track1": ("Track 1 — Classical", all_df[all_df["track"] == "1"]),
        "track2": ("Track 2 — Metaheuristic", all_df[all_df["track"] == "2"]),
        "track3": ("Track 3 — Learning / Control", all_df[all_df["track"] == "3"]),
        "all":    ("Cross-track (all 14 policies)", all_df),
    }

    md_parts: list[str] = [
        "# Multi-metric leaderboards",
        "",
        "Generated by `tools/leaderboard_tables.py` from the BO-tuned sweep "
        "CSVs (`outputs/sweep_track1_results.csv`, "
        "`outputs/sweep_track2_results.csv`, `outputs/sweep_results.csv`).",
        "",
        "Each row is the **mean across 36 runs** (3 maps × 4 swarm sizes × 3 "
        "seeds). Rank-sum aggregates the 5 ranking metrics: lowest sum wins. "
        f"Non-completing runs are penalized at `{TIME_PENALTY_S:.0f} s` on the "
        "time-to-100 metric (above the 734 s physics-bounded max sim time, so "
        "they sort below completing runs).",
        "",
    ]

    for slug, (title, sub_df) in sections.items():
        agg = aggregate(sub_df)
        ranks = rank_sum(agg)
        agg_sorted = agg.loc[ranks.index]
        agg_sorted.to_csv(out_dir / f"leaderboard_{slug}.csv")
        md_parts.append(md_table(agg_sorted, ranks, title))
        md_parts.append("")
        md_parts.append(per_metric_md(agg_sorted, title))
        md_parts.append("")

    md_path = out_dir / "leaderboards.md"
    md_path.write_text("\n".join(md_parts))
    print(f"Wrote {md_path}")
    print(f"Wrote 4 leaderboard CSVs to {out_dir}/")

    # Also print the cross-track summary to stdout for quick check
    cross_agg = aggregate(all_df)
    cross_ranks = rank_sum(cross_agg)
    print()
    print(md_table(cross_agg.loc[cross_ranks.index], cross_ranks,
                   "Cross-track (all 14 policies)"))


if __name__ == "__main__":
    main()
