"""
Grid search over `GAController` hyperparameters across all maps × drone sizes.

Sweeps the three GA knobs that drive behavior — `elite_fraction`,
`p_crossover`, `p_mutation` — over a small cartesian grid, evaluating
each config on every (map, n_drones, seed) cell of the standard sweep
grid. Reports the top configurations by mean composite score.

Outputs (to `outputs/csv_txt/` and `outputs/png/`):
    grid_search_ga.csv            — one row per (config, map, n_drones, seed)
    grid_search_ga_top.txt        — top-K configs ranked by mean score
    grid_search_ga_heatmap_e{value}.png  — pcx × pmut heatmap per elite_fraction

Usage:
    python tools/grid_search_ga.py
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
from controllers.ga import GAConfig, GAController
from environment import CoverageEnv, DroneConfig, SimConfig
from maze import FREE, load_map
from score import composite_score


DEFAULT_GRID = {
    "elite_fraction": [0.2, 0.4, 0.6],
    "p_crossover":    [0.1, 0.4, 0.7],
    "p_mutation":     [0.05, 0.15, 0.30],
}


@dataclass
class GARunResult:
    elite_fraction: float
    p_crossover: float
    p_mutation: float
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
    elite_fraction: float, p_crossover: float, p_mutation: float,
) -> GARunResult:
    env = CoverageEnv(grid=grid, n_drones=n_drones,
                      sim=SimConfig(), drone=DroneConfig())
    env.reset(seed=seed)
    cfg = GAConfig(
        elite_fraction=elite_fraction,
        p_crossover=p_crossover,
        p_mutation=p_mutation,
    )
    ga = GAController(cfg=cfg, hover_drone_idx=None, seed=seed)
    initial_energy = sum(d.battery_j for d in env.drones)

    while not env.is_terminal():
        env.step(ga(env))

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
    return GARunResult(
        elite_fraction=elite_fraction,
        p_crossover=p_crossover,
        p_mutation=p_mutation,
        map_label=map_label, n_drones=n_drones, seed=seed,
        final_coverage=env.coverage_fraction(),
        overlap_m2=env.overlap_cells_m2(),
        wasted_visits=env.wasted_visits_total(),
        energy_used_j=float(energy_used),
        time_to_terminal=env.time_seconds,
        score=s,
    )


def plot_heatmap(results, grid_axes, out_dir: Path) -> Path:
    """One combined PNG: 1 panel per elite_fraction, p_crossover × p_mutation per panel."""
    elites = grid_axes["elite_fraction"]
    pcxs = grid_axes["p_crossover"]
    pmuts = grid_axes["p_mutation"]

    agg: dict = {}
    for r in results:
        agg.setdefault((r.elite_fraction, r.p_crossover, r.p_mutation),
                       []).append(r.score)
    mean_score = {k: float(np.mean(v)) for k, v in agg.items()}
    best_cfg = max(mean_score, key=mean_score.get)
    best_score = mean_score[best_cfg]
    vmin, vmax = min(mean_score.values()), max(mean_score.values())

    fig, axes = plt.subplots(1, len(elites),
                             figsize=(4 * len(elites) + 0.5, 3.6),
                             sharey=True)
    if len(elites) == 1:
        axes = [axes]
    for ax, e in zip(axes, elites):
        Z = np.array([
            [mean_score.get((e, pc, pm), float("nan")) for pm in pmuts]
            for pc in pcxs
        ])
        im = ax.imshow(Z, origin="lower", aspect="auto",
                       cmap="viridis", vmin=vmin, vmax=vmax)
        ax.set_xticks(range(len(pmuts)))
        ax.set_xticklabels([str(s) for s in pmuts], fontsize=8)
        ax.set_yticks(range(len(pcxs)))
        ax.set_yticklabels([str(c) for c in pcxs], fontsize=8)
        ax.set_xlabel("p_mutation", fontsize=9)
        if ax is axes[0]:
            ax.set_ylabel("p_crossover", fontsize=9)
        ax.set_title(f"elite_fraction = {e}", fontsize=10)
        for i, pc in enumerate(pcxs):
            for j, pm in enumerate(pmuts):
                v = mean_score.get((e, pc, pm), None)
                if v is not None:
                    is_best = (e, pc, pm) == best_cfg
                    color = "white" if v < (vmin + vmax) / 2 else "black"
                    ax.text(j, i, f"{v:.2f}", ha="center", va="center",
                            color=color, fontsize=8,
                            fontweight="bold" if is_best else "normal")
    fig.suptitle(
        f"GA — grid search composite scores  (best: {best_score:+.3f})",
        fontsize=11, fontweight="bold",
    )
    fig.colorbar(im, ax=axes, fraction=0.025, pad=0.04, label="composite score")
    out = out_dir / "grid_search_ga_heatmap.png"
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
    p.add_argument("--elite-grid",     type=float, nargs="+",
                   default=DEFAULT_GRID["elite_fraction"])
    p.add_argument("--pcrossover-grid", type=float, nargs="+",
                   default=DEFAULT_GRID["p_crossover"])
    p.add_argument("--pmutation-grid",  type=float, nargs="+",
                   default=DEFAULT_GRID["p_mutation"])
    p.add_argument("--top-k", type=int, default=10)
    return p.parse_args()


def main():
    args = parse_args()

    grid_axes = {
        "elite_fraction": args.elite_grid,
        "p_crossover":    args.pcrossover_grid,
        "p_mutation":     args.pmutation_grid,
    }

    maps = []
    for path in args.map_files:
        p = Path(path)
        maps.append((p.stem, load_map(str(p))))

    configs = list(product(grid_axes["elite_fraction"],
                           grid_axes["p_crossover"],
                           grid_axes["p_mutation"]))
    n_runs = (len(configs) * len(maps) * len(args.drones_list)
              * len(args.seeds))

    print(f"=== GA grid search ===")
    print(f"  maps:        {[m[0] for m in maps]}")
    print(f"  n_drones:    {args.drones_list}")
    print(f"  seeds:       {args.seeds}")
    print(f"  elite:       {grid_axes['elite_fraction']}")
    print(f"  p_crossover: {grid_axes['p_crossover']}")
    print(f"  p_mutation:  {grid_axes['p_mutation']}")
    print(f"  configs:     {len(configs)}")
    print(f"  total runs:  {n_runs}")
    print()

    results = []
    t0 = time.time()
    runs_done = 0
    for ef, pc, pm in configs:
        for mlabel, mgrid in maps:
            for n in args.drones_list:
                for seed in args.seeds:
                    r = run_one(
                        mgrid, map_label=mlabel, n_drones=n, seed=seed,
                        elite_fraction=ef, p_crossover=pc, p_mutation=pm,
                    )
                    results.append(r)
                    runs_done += 1
        cfg_results = [r for r in results
                       if r.elite_fraction == ef and r.p_crossover == pc
                       and r.p_mutation == pm]
        mean_s = float(np.mean([r.score for r in cfg_results]))
        mean_cov = float(np.mean([r.final_coverage for r in cfg_results])) * 100
        print(f"  [{runs_done}/{n_runs}] e={ef} pcx={pc} pmut={pm}  "
              f"mean score={mean_s:+.3f}  mean cov={mean_cov:.1f}%")

    dt = time.time() - t0
    print(f"\nTotal wall time: {dt/60:.1f} min")

    # ---- write CSV ----
    DATA_DIR.mkdir(parents=True, exist_ok=True)
    PLOTS_DIR.mkdir(parents=True, exist_ok=True)
    csv_path = DATA_DIR / "grid_search_ga.csv"
    fieldnames = [
        "elite_fraction", "p_crossover", "p_mutation",
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

    # ---- top-K ----
    agg: dict = {}
    for r in results:
        agg.setdefault((r.elite_fraction, r.p_crossover, r.p_mutation),
                       []).append(r)
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
    lines.append(f"GA grid search — top {args.top_k} configs "
                 f"(mean across {len(maps)} maps × "
                 f"{len(args.drones_list)} drone counts × "
                 f"{len(args.seeds)} seeds)\n\n")
    lines.append(f"{'rank':>4s}  {'elite':>6s}  {'p_cx':>6s}  "
                 f"{'p_mut':>6s}  {'score':>7s}  {'cov':>6s}  "
                 f"{'overlap':>9s}  {'wasted':>7s}  {'energy':>9s}\n")
    lines.append("-" * 90 + "\n")
    for rank, (key, sc, cov, ov, wa, en) in enumerate(summary[: args.top_k], 1):
        ef, pc, pm = key
        lines.append(
            f"{rank:>4d}  {ef:>6.2f}  {pc:>6.2f}  {pm:>6.2f}  "
            f"{sc:>+7.3f}  {cov:>5.1f}%  {ov:>7.0f}m²  {wa:>7.0f}  "
            f"{en:>6.1f}kJ\n"
        )
    top_path = DATA_DIR / "grid_search_ga_top.txt"
    top_path.write_text("".join(lines))
    print()
    print("".join(lines), end="")
    print(f"Wrote: {top_path}")

    # ---- combined heatmap ----
    out = plot_heatmap(results, grid_axes, PLOTS_DIR)
    print(f"Wrote: {out}")


if __name__ == "__main__":
    main()
