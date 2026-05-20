"""Analyze RL hover-adapter sidecars: gain trajectories and adaptive behavior.

For each of the two batches (rl_calm_run, rl_worst_case), this script:
  1. Reads all sidecar JSONLs.
  2. Aligns runs on a common time grid (binned).
  3. Computes mean ± std gain trajectory across runs.
  4. Plots: 8-panel grid (one per gain), one calm-vs-worst pair per panel.
  5. Quantifies "did the policy adapt to disturbance?" — is the worst_case
     gain trajectory different from the calm one?

Outputs to reports/benchmark_hover_report/:
  - rl_gain_trajectories.png + .svg
  - rl_sidecar_summary.csv (per-run final-gain table)
  - rl_action_magnitudes.png + .svg (action |a| over time)
"""
from __future__ import annotations

import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


REPORT_ROOT = Path("reports/benchmark_hover_report")
OUT_DIR = REPORT_ROOT

GAIN_NAMES = [
    "ATC_ANG_RLL_P", "ATC_ANG_PIT_P",
    "ATC_RAT_RLL_P", "ATC_RAT_PIT_P",
    "ATC_RAT_RLL_I", "ATC_RAT_PIT_I",
    "ATC_RAT_RLL_D", "ATC_RAT_PIT_D",
]
BASELINE_GAINS = [4.5, 4.5, 0.135, 0.135, 0.135, 0.135, 0.0036, 0.0036]


def load_sidecars(batch_dir: Path) -> list[list[dict]]:
    runs = []
    for fp in sorted(batch_dir.glob("rl_actions_*.jsonl")):
        rows = [json.loads(l) for l in fp.open()]
        if rows and "gains" in rows[0]:
            runs.append(rows)
    return runs


def align_on_grid(runs: list[list[dict]], dt: float = 0.5, n_bins: int = 70):
    """Build (n_bins, n_gains) mean + std arrays + per-step n. Time grid in
    wall-seconds; runs are 0.5-s cadence so this is straightforward."""
    n_gains = len(runs[0][0]["gains"])
    grid_centers = np.arange(n_bins) * dt + dt / 2
    binned = np.full((len(runs), n_bins, n_gains), np.nan)
    actions_binned = np.full((len(runs), n_bins, n_gains), np.nan)
    for i, run in enumerate(runs):
        for r in run:
            b = int(r["t"] // dt)
            if 0 <= b < n_bins:
                binned[i, b, :] = r["gains"]
                actions_binned[i, b, :] = r["action"]
    mean_g = np.nanmean(binned, axis=0)
    std_g  = np.nanstd(binned, axis=0)
    mean_a = np.nanmean(actions_binned, axis=0)
    std_a  = np.nanstd(actions_binned, axis=0)
    return grid_centers, mean_g, std_g, mean_a, std_a


def plot_gain_trajectories(calm_runs, worst_runs):
    """8-panel grid (one per gain), each showing calm and worst_case mean ± std."""
    dt = 0.5
    n_bins = 70  # 35 wall-sec / 0.5 = 70 bins, comfortable margin
    t, calm_mean, calm_std, _, _ = align_on_grid(calm_runs, dt=dt, n_bins=n_bins)
    _, worst_mean, worst_std, _, _ = align_on_grid(worst_runs, dt=dt, n_bins=n_bins)

    fig, axes = plt.subplots(2, 4, figsize=(15, 7))
    for i, name in enumerate(GAIN_NAMES):
        ax = axes[i // 4, i % 4]
        ax.plot(t, calm_mean[:, i], color="#2ca02c", linewidth=2, label="calm")
        ax.fill_between(t, calm_mean[:, i] - calm_std[:, i], calm_mean[:, i] + calm_std[:, i],
                        alpha=0.25, color="#2ca02c")
        ax.plot(t, worst_mean[:, i], color="#9467bd", linewidth=2, label="worst_case")
        ax.fill_between(t, worst_mean[:, i] - worst_std[:, i], worst_mean[:, i] + worst_std[:, i],
                        alpha=0.25, color="#9467bd")
        ax.axhline(BASELINE_GAINS[i], color="gray", linestyle="--", alpha=0.6, label="baseline")
        ax.set_title(name, fontsize=10)
        ax.set_xlabel("wall sec from hover start")
        ax.set_ylabel("gain value")
        ax.grid(True, alpha=0.3)
        if i == 0:
            ax.legend(fontsize=8, loc="best")
    fig.suptitle("RL gain trajectories during hover (mean ± std across 10 runs)", y=1.02)
    fig.tight_layout()
    fig.savefig(OUT_DIR / "rl_gain_trajectories.png", dpi=160, bbox_inches="tight")
    fig.savefig(OUT_DIR / "rl_gain_trajectories.svg", bbox_inches="tight")
    plt.close(fig)
    print(f"wrote {OUT_DIR / 'rl_gain_trajectories.png'}")


def plot_action_magnitudes(calm_runs, worst_runs):
    """Action magnitude over time — is the policy still acting or has it settled?"""
    dt = 0.5
    n_bins = 70
    fig, axes = plt.subplots(1, 2, figsize=(13, 4.2))

    for ax, runs, label, color in [
        (axes[0], calm_runs, "calm",        "#2ca02c"),
        (axes[1], worst_runs, "worst_case", "#9467bd"),
    ]:
        all_mags = []
        for run in runs:
            mag = []
            t = []
            for r in run:
                a = np.array(r["action"])
                mag.append(float(np.linalg.norm(a)))
                t.append(r["t"])
            all_mags.append((t, mag))
            ax.plot(t, mag, color=color, alpha=0.25, linewidth=1)

        # Compute mean across runs by binning
        binned = np.full((len(runs), n_bins), np.nan)
        for i, (t, mag) in enumerate(all_mags):
            for ti, mi in zip(t, mag):
                b = int(ti // dt)
                if 0 <= b < n_bins:
                    binned[i, b] = mi
        centers = np.arange(n_bins) * dt + dt / 2
        ax.plot(centers, np.nanmean(binned, axis=0), color=color, linewidth=2.5,
                label=f"{label} mean")
        ax.set_xlabel("wall sec from hover start")
        ax.set_ylabel("||action|| (L2 norm)")
        ax.set_title(f"Policy action magnitude — {label}")
        ax.set_ylim(0, 1.5)
        ax.axhline(0.0, color="gray", linewidth=0.5)
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper right")
    fig.suptitle("Did the policy keep acting? Action L2-norm during hover", y=1.02)
    fig.tight_layout()
    fig.savefig(OUT_DIR / "rl_action_magnitudes.png", dpi=160, bbox_inches="tight")
    fig.savefig(OUT_DIR / "rl_action_magnitudes.svg", bbox_inches="tight")
    plt.close(fig)
    print(f"wrote {OUT_DIR / 'rl_action_magnitudes.png'}")


def per_run_summary(calm_runs, worst_runs):
    """Per-run final gains + adaptation magnitude vs baseline."""
    rows = [["batch", "run", "n_steps"] + [f"final_{g}" for g in GAIN_NAMES]]
    def _add(batch_label, runs):
        for i, run in enumerate(runs):
            final = run[-1]["gains"]
            rows.append([batch_label, i + 1, len(run)] + [f"{v:.4f}" for v in final])
    _add("calm", calm_runs)
    _add("worst_case", worst_runs)
    out_csv = OUT_DIR / "rl_sidecar_summary.csv"
    with out_csv.open("w", newline="", encoding="utf-8") as f:
        csv.writer(f).writerows(rows)
    print(f"wrote {out_csv}")


def compute_adaptation_metric(calm_runs, worst_runs):
    """Quantitative answer to 'did the policy adapt to the disturbance?'

    For each gain, compute the difference between calm-mean-final and
    worst_case-mean-final. If the policy is *adapting* (not just running
    the same response every time), these final values should differ
    meaningfully between profiles.
    """
    print("\n=== Adaptation metric: did the policy respond to disturbance? ===")
    print(f"{'Gain':>18s}   {'calm_final':>11s}   {'worst_final':>11s}   {'diff':>9s}   {'rel%':>7s}")
    for i, name in enumerate(GAIN_NAMES):
        cf = np.mean([r[-1]["gains"][i] for r in calm_runs])
        wf = np.mean([r[-1]["gains"][i] for r in worst_runs])
        diff = wf - cf
        rel = 100 * diff / abs(cf) if cf else float("nan")
        flag = "  <-- different" if abs(rel) > 10 else ""
        print(f"  {name:>16s}    {cf:11.5f}   {wf:11.5f}   {diff:9.5f}   {rel:7.1f}%{flag}")


if __name__ == "__main__":
    calm_runs = load_sidecars(REPORT_ROOT / "rl_calm_run")
    worst_runs = load_sidecars(REPORT_ROOT / "rl_worst_case")
    print(f"loaded {len(calm_runs)} calm sidecars, {len(worst_runs)} worst_case sidecars")
    plot_gain_trajectories(calm_runs, worst_runs)
    plot_action_magnitudes(calm_runs, worst_runs)
    per_run_summary(calm_runs, worst_runs)
    compute_adaptation_metric(calm_runs, worst_runs)
