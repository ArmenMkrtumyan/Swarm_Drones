"""Generate additional figures from compare_v0 data for the report.

Outputs to logs/tb_logs/compare_v0/_figures/:
  - eval_box.png      — box plot of all 50 deterministic eval returns per algo
  - eval_bars.png     — bar chart of per-algo seed-mean ± cross-seed std
  - eval_seeds.png    — strip plot of per-seed final means (5 dots per algo)
  - tb_critic_loss.png — TensorBoard-derived critic loss DDPG vs TD3
  - tb_ppo_std.png    — TensorBoard-derived PPO action-std and entropy

All plots get a matching .svg next to them for editing.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

LOG_ROOT = Path("logs/tb_logs/compare_v0")
FIG_DIR = LOG_ROOT / "_figures"
FIG_DIR.mkdir(parents=True, exist_ok=True)

ALGO_COLORS = {"DDPG": "#d62728", "TD3": "#1f77b4", "PPO": "#2ca02c"}
ALGOS = ["DDPG", "TD3", "PPO"]
SEEDS = [0, 1, 2, 3, 4]


def _save(fig, stem: str):
    png = FIG_DIR / f"{stem}.png"
    svg = FIG_DIR / f"{stem}.svg"
    fig.savefig(png, dpi=160, bbox_inches="tight")
    fig.savefig(svg, bbox_inches="tight")
    plt.close(fig)
    print(f"wrote {png}")


# -----------------------------------------------------------------------------
def make_eval_plots():
    eval_data = json.loads((LOG_ROOT / "_eval_results.json").read_text())

    # 1. Box plot of all 50 returns per algo (10 episodes × 5 seeds).
    fig, ax = plt.subplots(figsize=(7, 4.5))
    box_data = []
    for algo in ALGOS:
        all_returns = []
        for seed in SEEDS:
            all_returns.extend(eval_data[algo][str(seed)]["all_returns"])
        box_data.append(all_returns)
    bp = ax.boxplot(box_data, labels=ALGOS, patch_artist=True,
                    medianprops={"color": "black", "linewidth": 2},
                    boxprops={"alpha": 0.7})
    for patch, algo in zip(bp["boxes"], ALGOS):
        patch.set_facecolor(ALGO_COLORS[algo])
    # Overlay individual episode dots
    for i, returns in enumerate(box_data, start=1):
        x = np.random.normal(i, 0.04, size=len(returns))
        ax.scatter(x, returns, alpha=0.5, s=12, color="black", zorder=3)
    ax.axhline(125, color="gray", linestyle="--", alpha=0.5, label="ideal hover (~125)")
    ax.axhline(0, color="black", linestyle=":", alpha=0.4)
    ax.set_ylabel("Episode return (deterministic eval)")
    ax.set_title("Distribution of 50 deterministic eval episodes per algorithm")
    ax.grid(True, alpha=0.3, axis="y")
    ax.legend(loc="lower left")
    fig.tight_layout()
    _save(fig, "eval_box")

    # 2. Bar chart: cross-seed mean ± cross-seed std.
    fig, ax = plt.subplots(figsize=(6, 4.5))
    means = []
    stds = []
    for algo in ALGOS:
        seed_means = [eval_data[algo][str(s)]["mean"] for s in SEEDS]
        means.append(np.mean(seed_means))
        stds.append(np.std(seed_means, ddof=0))
    colors = [ALGO_COLORS[a] for a in ALGOS]
    bars = ax.bar(ALGOS, means, yerr=stds, color=colors, alpha=0.85,
                  capsize=10, edgecolor="black", linewidth=0.7)
    for bar, m, s in zip(bars, means, stds):
        ax.text(bar.get_x() + bar.get_width() / 2, m + s + 1.5,
                f"{m:.1f}\n±{s:.1f}", ha="center", va="bottom", fontsize=10)
    ax.set_ylabel("Mean of seed-means (deterministic eval)")
    ax.set_title("Cross-seed performance (mean ± std across 5 seeds)")
    ax.set_ylim(0, max(means) * 1.25)
    ax.grid(True, alpha=0.3, axis="y")
    fig.tight_layout()
    _save(fig, "eval_bars")

    # 3. Per-seed dotplot — shows how individual seeds disperse around the mean.
    fig, ax = plt.subplots(figsize=(7, 4.5))
    for i, algo in enumerate(ALGOS):
        seed_means = [eval_data[algo][str(s)]["mean"] for s in SEEDS]
        seed_stds = [eval_data[algo][str(s)]["std"] for s in SEEDS]
        x_jit = np.full(len(seed_means), i) + np.random.normal(0, 0.05, len(seed_means))
        ax.errorbar(x_jit, seed_means, yerr=seed_stds, fmt="o",
                    markersize=10, color=ALGO_COLORS[algo],
                    ecolor=ALGO_COLORS[algo], elinewidth=1.5, capsize=4,
                    alpha=0.85, label=f"{algo} (per-seed mean ± per-seed std)")
        # Annotate each seed by its number
        for j, (xv, mv) in enumerate(zip(x_jit, seed_means)):
            ax.annotate(f"s{SEEDS[j]}", (xv, mv), fontsize=8,
                        xytext=(7, 0), textcoords="offset points", va="center")
    ax.set_xticks(range(len(ALGOS)))
    ax.set_xticklabels(ALGOS)
    ax.set_ylabel("Per-seed mean return ± per-seed std")
    ax.set_title("Individual seed performance (n=10 episodes per seed)")
    ax.axhline(125, color="gray", linestyle="--", alpha=0.4, label="ideal hover")
    ax.grid(True, alpha=0.3, axis="y")
    ax.legend(loc="lower left", fontsize=9)
    fig.tight_layout()
    _save(fig, "eval_seeds")


# -----------------------------------------------------------------------------
def make_tb_plots():
    """Plots derived from TensorBoard event data — scalars logged during training."""
    from tensorboard.backend.event_processing.event_accumulator import EventAccumulator

    def load_scalar(run_dir: Path, tag: str):
        ea = EventAccumulator(str(run_dir), size_guidance={"scalars": 100000})
        ea.Reload()
        if tag not in ea.Tags().get("scalars", []):
            return None, None
        evs = ea.Scalars(tag)
        return np.array([e.step for e in evs]), np.array([e.value for e in evs])

    def find_run(algo: str, seed: int) -> Path | None:
        candidates = sorted([p for p in LOG_ROOT.iterdir()
                             if p.is_dir() and p.name.startswith(f"{algo}_seed{seed}")])
        return candidates[-1] if candidates else None

    # Plot 4: critic loss DDPG vs TD3 — the "Q-overestimation" visual evidence.
    fig, ax = plt.subplots(figsize=(8, 4.5))
    for algo in ("DDPG", "TD3"):
        all_curves = []
        for seed in SEEDS:
            rd = find_run(algo, seed)
            if rd is None:
                continue
            steps, vals = load_scalar(rd, "train/critic_loss")
            if steps is None or len(steps) == 0:
                continue
            # Bin for smoothing.
            bin_w = 5000
            n_bins = 200_000 // bin_w
            centers = np.arange(n_bins) * bin_w + bin_w / 2
            binned = np.full(n_bins, np.nan)
            for i in range(n_bins):
                mask = (steps >= i * bin_w) & (steps < (i + 1) * bin_w)
                if mask.any():
                    binned[i] = float(np.mean(vals[mask]))
            all_curves.append(binned)
        stack = np.vstack(all_curves)
        mean = np.nanmean(stack, axis=0)
        std = np.nanstd(stack, axis=0)
        ax.plot(centers, mean, label=f"{algo} (n={len(all_curves)})",
                color=ALGO_COLORS[algo], linewidth=2)
        ax.fill_between(centers, mean - std, mean + std,
                        alpha=0.20, color=ALGO_COLORS[algo])
    ax.set_xlabel("Environment steps")
    ax.set_ylabel("train/critic_loss (mean ± std across seeds)")
    ax.set_title("Critic loss during training: DDPG (single Q) vs TD3 (twin Q + delay)")
    ax.set_yscale("log")
    ax.grid(True, alpha=0.3, which="both")
    ax.legend(loc="upper left")
    fig.tight_layout()
    _save(fig, "tb_critic_loss")

    # Plot 5: PPO action-std and entropy over training (exploration trajectory).
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.2))
    for ax, tag, ylab, title in [
        (axes[0], "train/std",          "policy action std",     "PPO learned action std (exploration magnitude)"),
        (axes[1], "train/entropy_loss", "train/entropy_loss",   "PPO entropy loss (more negative = higher entropy)"),
    ]:
        all_curves = []
        for seed in SEEDS:
            rd = find_run("PPO", seed)
            if rd is None:
                continue
            steps, vals = load_scalar(rd, tag)
            if steps is None or len(steps) == 0:
                continue
            bin_w = 5000
            n_bins = 200_000 // bin_w
            centers = np.arange(n_bins) * bin_w + bin_w / 2
            binned = np.full(n_bins, np.nan)
            for i in range(n_bins):
                mask = (steps >= i * bin_w) & (steps < (i + 1) * bin_w)
                if mask.any():
                    binned[i] = float(np.mean(vals[mask]))
            all_curves.append(binned)
            ax.plot(centers, binned, color=ALGO_COLORS["PPO"], alpha=0.35, linewidth=1)
        if all_curves:
            stack = np.vstack(all_curves)
            mean = np.nanmean(stack, axis=0)
            ax.plot(centers, mean, color=ALGO_COLORS["PPO"], linewidth=2.2,
                    label=f"mean across {len(all_curves)} seeds")
        ax.set_xlabel("Environment steps")
        ax.set_ylabel(ylab)
        ax.set_title(title, fontsize=11)
        ax.grid(True, alpha=0.3)
        ax.legend(loc="best")
    fig.suptitle("PPO exploration during training (TensorBoard scalars)", y=1.02)
    fig.tight_layout()
    _save(fig, "tb_ppo_exploration")


if __name__ == "__main__":
    make_eval_plots()
    make_tb_plots()
    print("done.")
