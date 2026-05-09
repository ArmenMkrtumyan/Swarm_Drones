"""
Grid search over `PSOController` hyperparameters across all maps × drone sizes.

Sweeps the three PSO knobs that drive behavior — `inertia` (w),
`cognitive` (c1), `social` (c2) — over a small cartesian grid, evaluating
each config on every (map, n_drones, seed) cell of the standard sweep
grid. Reports the top configurations by mean composite score.

Outputs (to `outputs/csv_txt/` and `outputs/png/`):
    grid_search_pso.csv        — one row per (config, map, n_drones, seed)
    grid_search_pso_top.txt    — top-K configs ranked by mean score
    grid_search_pso_heatmap_w{value}.png  — c1 × c2 score heatmap per inertia

Usage:
    python tools/grid_search_pso.py
    python tools/grid_search_pso.py --inertia-grid 0.0 0.5 \\
        --cognitive-grid 1.0 2.0 --social-grid 1.0 2.0
"""

from __future__ import annotations

import argparse
import csv
import sys
import time
from dataclasses import dataclass
from itertools import product
from pathlib import Path

# Make project root importable.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from constants import DATA_DIR, MAPS_DIR, PLOTS_DIR
from controllers.pso import PSOConfig, PSOController
from environment import CoverageEnv, DroneConfig, SimConfig
from maze import FREE, load_map
from score import composite_score


DEFAULT_GRID = {
    "inertia":   [0.0, 0.5, 1.0],
    "cognitive": [0.5, 1.5, 3.0],
    "social":    [0.5, 1.5, 3.0],
}


@dataclass
class PSORunResult:
    inertia: float
    cognitive: float
    social: float
    map_label: str
    n_drones: int
    seed: int
    final_coverage: float
    overlap_m2: float
    wasted_visits: int
    energy_used_j: float
    time_to_terminal: float
    score: float


def run_one(
    grid: np.ndarray, *,
    map_label: str, n_drones: int, seed: int,
    inertia: float, cognitive: float, social: float,
) -> PSORunResult:
    env = CoverageEnv(grid=grid, n_drones=n_drones,
                      sim=SimConfig(), drone=DroneConfig())
    env.reset(seed=seed)
    cfg = PSOConfig(inertia=inertia, cognitive=cognitive, social=social)
    pso = PSOController(cfg=cfg, hover_drone_idx=None, seed=seed)
    initial_energy = sum(d.battery_j for d in env.drones)

    while not env.is_terminal():
        env.step(pso(env))

    energy_used = initial_energy - sum(d.battery_j for d in env.drones)
    free_mask = env.grid == FREE
    free_area_m2 = float(free_mask.sum()) * (env.sim_cfg.meters_per_cell ** 2)
    unique_visited = int((env.covered & free_mask).sum())
    energy_budget = env.battery_cfg.initial_energy_j * env.n_drones

    s = composite_score(
        coverage_fraction=env.coverage_fraction(),
        overlap_m2=env.overlap_cells_m2(),
        free_area_m2=free_area_m2,
        wasted_visits=env.wasted_visits_total(),
        unique_cells_visited=unique_visited,
        energy_used_j=float(energy_used),
        energy_budget_j=energy_budget,
    )
    return PSORunResult(
        inertia=inertia, cognitive=cognitive, social=social,
        map_label=map_label, n_drones=n_drones, seed=seed,
        final_coverage=env.coverage_fraction(),
        overlap_m2=env.overlap_cells_m2(),
        wasted_visits=env.wasted_visits_total(),
        energy_used_j=float(energy_used),
        time_to_terminal=env.time_seconds,
        score=s,
    )


def plot_heatmap(results: list[PSORunResult], grid_axes: dict,
                 out_dir: Path) -> Path:
    """One combined PNG: 1 panel per inertia value, c1 × c2 axes per panel."""
    inertias = grid_axes["inertia"]
    cognitives = grid_axes["cognitive"]
    socials = grid_axes["social"]

    agg: dict = {}
    for r in results:
        agg.setdefault((r.inertia, r.cognitive, r.social), []).append(r.score)
    mean_score = {k: float(np.mean(v)) for k, v in agg.items()}
    best_cfg = max(mean_score, key=mean_score.get)
    best_score = mean_score[best_cfg]
    vmin, vmax = min(mean_score.values()), max(mean_score.values())

    fig, axes = plt.subplots(1, len(inertias),
                             figsize=(4 * len(inertias) + 0.5, 3.6),
                             sharey=True)
    if len(inertias) == 1:
        axes = [axes]
    for ax, w in zip(axes, inertias):
        Z = np.array([
            [mean_score.get((w, c1, c2), float("nan")) for c2 in socials]
            for c1 in cognitives
        ])
        im = ax.imshow(Z, origin="lower", aspect="auto",
                       cmap="viridis", vmin=vmin, vmax=vmax)
        ax.set_xticks(range(len(socials)))
        ax.set_xticklabels([str(s) for s in socials], fontsize=8)
        ax.set_yticks(range(len(cognitives)))
        ax.set_yticklabels([str(c) for c in cognitives], fontsize=8)
        ax.set_xlabel("social  c2", fontsize=9)
        if ax is axes[0]:
            ax.set_ylabel("cognitive  c1", fontsize=9)
        ax.set_title(f"inertia w = {w}", fontsize=10)
        for i, c1 in enumerate(cognitives):
            for j, c2 in enumerate(socials):
                v = mean_score.get((w, c1, c2), None)
                if v is not None:
                    is_best = (w, c1, c2) == best_cfg
                    color = "white" if v < (vmin + vmax) / 2 else "black"
                    ax.text(j, i, f"{v:.2f}", ha="center", va="center",
                            color=color, fontsize=8,
                            fontweight="bold" if is_best else "normal")

    fig.suptitle(
        f"PSO — grid search composite scores  (best: {best_score:+.3f})",
        fontsize=11, fontweight="bold",
    )
    fig.colorbar(im, ax=axes, fraction=0.025, pad=0.04, label="composite score")
    out = out_dir / "grid_search_pso_heatmap.png"
    fig.savefig(out, dpi=130, bbox_inches="tight")
    plt.close(fig)
    return out


def parse_args():
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--map-files", type=str, nargs="+", default=[
        str(MAPS_DIR / "open_33.npy"),
        str(MAPS_DIR / "partial_33.npy"),
        str(MAPS_DIR / "closed_33.npy"),
    ])
    p.add_argument("--drones-list", type=int, nargs="+", default=[2, 5, 10, 20])
    p.add_argument("--seeds", type=int, nargs="+", default=[0, 1])
    p.add_argument("--inertia-grid",   type=float, nargs="+",
                   default=DEFAULT_GRID["inertia"])
    p.add_argument("--cognitive-grid", type=float, nargs="+",
                   default=DEFAULT_GRID["cognitive"])
    p.add_argument("--social-grid",    type=float, nargs="+",
                   default=DEFAULT_GRID["social"])
    p.add_argument("--top-k", type=int, default=10)
    return p.parse_args()


def main():
    args = parse_args()

    grid_axes = {
        "inertia":   args.inertia_grid,
        "cognitive": args.cognitive_grid,
        "social":    args.social_grid,
    }

    maps = []
    for path in args.map_files:
        p = Path(path)
        maps.append((p.stem, load_map(str(p))))

    configs = list(product(grid_axes["inertia"],
                           grid_axes["cognitive"],
                           grid_axes["social"]))
    n_runs = (len(configs) * len(maps) * len(args.drones_list)
              * len(args.seeds))

    print(f"=== PSO grid search ===")
    print(f"  maps:        {[m[0] for m in maps]}")
    print(f"  n_drones:    {args.drones_list}")
    print(f"  seeds:       {args.seeds}")
    print(f"  inertia:     {grid_axes['inertia']}")
    print(f"  cognitive:   {grid_axes['cognitive']}")
    print(f"  social:      {grid_axes['social']}")
    print(f"  configs:     {len(configs)}")
    print(f"  total runs:  {n_runs}")
    print()

    results: list[PSORunResult] = []
    t0 = time.time()
    runs_done = 0
    for w, c1, c2 in configs:
        for mlabel, mgrid in maps:
            for n in args.drones_list:
                for seed in args.seeds:
                    r = run_one(
                        mgrid, map_label=mlabel, n_drones=n, seed=seed,
                        inertia=w, cognitive=c1, social=c2,
                    )
                    results.append(r)
                    runs_done += 1
        # Per-config summary (mean over all 24 cells × |seeds|).
        cfg_results = [r for r in results
                       if r.inertia == w and r.cognitive == c1 and r.social == c2]
        mean_s = float(np.mean([r.score for r in cfg_results]))
        mean_cov = float(np.mean([r.final_coverage for r in cfg_results])) * 100
        print(f"  [{runs_done}/{n_runs}] w={w} c1={c1} c2={c2}  "
              f"mean score={mean_s:+.3f}  mean cov={mean_cov:.1f}%")

    dt = time.time() - t0
    print(f"\nTotal wall time: {dt/60:.1f} min")

    # ---- write CSV ----
    DATA_DIR.mkdir(parents=True, exist_ok=True)
    PLOTS_DIR.mkdir(parents=True, exist_ok=True)
    csv_path = DATA_DIR / "grid_search_pso.csv"
    fieldnames = [
        "inertia", "cognitive", "social",
        "map_label", "n_drones", "seed",
        "final_coverage", "overlap_m2", "wasted_visits",
        "energy_used_j", "time_to_terminal", "score",
    ]
    with csv_path.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        for r in results:
            w.writerow({k: getattr(r, k) for k in fieldnames})
    print(f"Wrote: {csv_path}")

    # ---- top-K (aggregated by config, mean over all cells) ----
    agg: dict = {}
    for r in results:
        agg.setdefault((r.inertia, r.cognitive, r.social), []).append(r)
    summary = sorted(
        [
            (key,
             float(np.mean([r.score for r in rs])),
             float(np.mean([r.final_coverage for r in rs])) * 100,
             float(np.mean([r.overlap_m2 for r in rs])),
             float(np.mean([r.wasted_visits for r in rs])),
             float(np.mean([r.energy_used_j for r in rs])) / 1000,
            )
            for key, rs in agg.items()
        ],
        key=lambda x: -x[1],
    )

    lines = []
    lines.append(f"PSO grid search — top {args.top_k} configs "
                 f"(mean across {len(maps)} maps × "
                 f"{len(args.drones_list)} drone counts × "
                 f"{len(args.seeds)} seeds)\n\n")
    lines.append(f"{'rank':>4s}  {'inertia':>7s}  {'cognit':>7s}  "
                 f"{'social':>7s}  {'score':>7s}  {'cov':>6s}  "
                 f"{'overlap':>9s}  {'wasted':>7s}  {'energy':>9s}\n")
    lines.append("-" * 90 + "\n")
    for rank, (key, sc, cov, ov, wa, en) in enumerate(summary[: args.top_k], 1):
        w_, c1_, c2_ = key
        lines.append(
            f"{rank:>4d}  {w_:>7.2f}  {c1_:>7.2f}  {c2_:>7.2f}  "
            f"{sc:>+7.3f}  {cov:>5.1f}%  {ov:>7.0f}m²  {wa:>7.0f}  "
            f"{en:>6.1f}kJ\n"
        )
    top_path = DATA_DIR / "grid_search_pso_top.txt"
    top_path.write_text("".join(lines))
    print()
    print("".join(lines), end="")
    print(f"Wrote: {top_path}")

    # ---- combined heatmap ----
    out = plot_heatmap(results, grid_axes, PLOTS_DIR)
    print(f"Wrote: {out}")


if __name__ == "__main__":
    main()
