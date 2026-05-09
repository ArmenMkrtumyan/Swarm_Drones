"""
Grid search over `ACOController` hyperparameters across all maps × drone sizes.

Sweeps the three core ACO knobs — `pheromone_weight` (α),
`heuristic_weight` (β), `evaporation_rate` (ρ) — over a small cartesian
grid. Same shape and outputs as `grid_search_pso.py` / `grid_search_ga.py`.

Outputs (to `outputs/csv_txt/` and `outputs/png/`):
    grid_search_aco.csv
    grid_search_aco_top.txt
    grid_search_aco_heatmap_rho{value}.png

Usage:
    python tools/grid_search_aco.py
"""

from __future__ import annotations

import argparse
import csv
import sys
import time
from dataclasses import dataclass
from itertools import product
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from constants import DATA_DIR, MAPS_DIR, PLOTS_DIR
from controllers.aco import ACOConfig, ACOController
from environment import CoverageEnv, DroneConfig, SimConfig
from maze import FREE, load_map
from score import composite_score


DEFAULT_GRID = {
    "pheromone_weight": [0.5, 1.0, 2.0],     # α
    "heuristic_weight": [1.0, 2.0, 4.0],     # β
    "evaporation_rate": [0.05, 0.15, 0.30],  # ρ
}


@dataclass
class ACORunResult:
    pheromone_weight: float
    heuristic_weight: float
    evaporation_rate: float
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
    pheromone_weight: float, heuristic_weight: float, evaporation_rate: float,
) -> ACORunResult:
    env = CoverageEnv(grid=grid, n_drones=n_drones,
                      sim=SimConfig(), drone=DroneConfig())
    env.reset(seed=seed)
    cfg = ACOConfig(
        pheromone_weight=pheromone_weight,
        heuristic_weight=heuristic_weight,
        evaporation_rate=evaporation_rate,
    )
    aco = ACOController(cfg=cfg, hover_drone_idx=None, seed=seed)
    initial_energy = sum(d.battery_j for d in env.drones)

    while not env.is_terminal():
        env.step(aco(env))

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
    return ACORunResult(
        pheromone_weight=pheromone_weight,
        heuristic_weight=heuristic_weight,
        evaporation_rate=evaporation_rate,
        map_label=map_label, n_drones=n_drones, seed=seed,
        final_coverage=env.coverage_fraction(),
        overlap_m2=env.overlap_cells_m2(),
        wasted_visits=env.wasted_visits_total(),
        energy_used_j=float(energy_used),
        time_to_terminal=env.time_seconds,
        score=s,
    )


def plot_heatmap(results, grid_axes, out_dir: Path) -> Path:
    """One combined PNG: 1 panel per evaporation_rate, α × β per panel."""
    rhos = grid_axes["evaporation_rate"]
    alphas = grid_axes["pheromone_weight"]
    betas = grid_axes["heuristic_weight"]

    agg: dict = {}
    for r in results:
        agg.setdefault((r.evaporation_rate, r.pheromone_weight,
                        r.heuristic_weight), []).append(r.score)
    mean_score = {k: float(np.mean(v)) for k, v in agg.items()}
    best_cfg = max(mean_score, key=mean_score.get)
    best_score = mean_score[best_cfg]
    vmin, vmax = min(mean_score.values()), max(mean_score.values())

    fig, axes = plt.subplots(1, len(rhos),
                             figsize=(4 * len(rhos) + 0.5, 3.6),
                             sharey=True)
    if len(rhos) == 1:
        axes = [axes]
    for ax, rho in zip(axes, rhos):
        Z = np.array([
            [mean_score.get((rho, a, b), float("nan")) for b in betas]
            for a in alphas
        ])
        im = ax.imshow(Z, origin="lower", aspect="auto",
                       cmap="viridis", vmin=vmin, vmax=vmax)
        ax.set_xticks(range(len(betas)))
        ax.set_xticklabels([str(b) for b in betas], fontsize=8)
        ax.set_yticks(range(len(alphas)))
        ax.set_yticklabels([str(a) for a in alphas], fontsize=8)
        ax.set_xlabel("heuristic β", fontsize=9)
        if ax is axes[0]:
            ax.set_ylabel("pheromone α", fontsize=9)
        ax.set_title(f"evaporation ρ = {rho}", fontsize=10)
        for i, a in enumerate(alphas):
            for j, b in enumerate(betas):
                v = mean_score.get((rho, a, b), None)
                if v is not None:
                    is_best = (rho, a, b) == best_cfg
                    color = "white" if v < (vmin + vmax) / 2 else "black"
                    ax.text(j, i, f"{v:.2f}", ha="center", va="center",
                            color=color, fontsize=8,
                            fontweight="bold" if is_best else "normal")
    fig.suptitle(
        f"ACO — grid search composite scores  (best: {best_score:+.3f})",
        fontsize=11, fontweight="bold",
    )
    fig.colorbar(im, ax=axes, fraction=0.025, pad=0.04, label="composite score")
    out = out_dir / "grid_search_aco_heatmap.png"
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
    p.add_argument("--alpha-grid", type=float, nargs="+",
                   default=DEFAULT_GRID["pheromone_weight"])
    p.add_argument("--beta-grid",  type=float, nargs="+",
                   default=DEFAULT_GRID["heuristic_weight"])
    p.add_argument("--rho-grid",   type=float, nargs="+",
                   default=DEFAULT_GRID["evaporation_rate"])
    p.add_argument("--top-k", type=int, default=10)
    return p.parse_args()


def main():
    args = parse_args()

    grid_axes = {
        "pheromone_weight": args.alpha_grid,
        "heuristic_weight": args.beta_grid,
        "evaporation_rate": args.rho_grid,
    }

    maps = []
    for path in args.map_files:
        p = Path(path)
        maps.append((p.stem, load_map(str(p))))

    configs = list(product(grid_axes["pheromone_weight"],
                           grid_axes["heuristic_weight"],
                           grid_axes["evaporation_rate"]))
    n_runs = (len(configs) * len(maps) * len(args.drones_list)
              * len(args.seeds))

    print(f"=== ACO grid search ===")
    print(f"  maps:        {[m[0] for m in maps]}")
    print(f"  n_drones:    {args.drones_list}")
    print(f"  seeds:       {args.seeds}")
    print(f"  α grid:      {grid_axes['pheromone_weight']}")
    print(f"  β grid:      {grid_axes['heuristic_weight']}")
    print(f"  ρ grid:      {grid_axes['evaporation_rate']}")
    print(f"  configs:     {len(configs)}")
    print(f"  total runs:  {n_runs}")
    print()

    results = []
    t0 = time.time()
    runs_done = 0
    for a, b, rho in configs:
        for mlabel, mgrid in maps:
            for n in args.drones_list:
                for seed in args.seeds:
                    r = run_one(
                        mgrid, map_label=mlabel, n_drones=n, seed=seed,
                        pheromone_weight=a, heuristic_weight=b,
                        evaporation_rate=rho,
                    )
                    results.append(r)
                    runs_done += 1
        cfg_results = [r for r in results
                       if r.pheromone_weight == a and r.heuristic_weight == b
                       and r.evaporation_rate == rho]
        mean_s = float(np.mean([r.score for r in cfg_results]))
        mean_cov = float(np.mean([r.final_coverage for r in cfg_results])) * 100
        print(f"  [{runs_done}/{n_runs}] α={a} β={b} ρ={rho}  "
              f"mean score={mean_s:+.3f}  mean cov={mean_cov:.1f}%")

    dt = time.time() - t0
    print(f"\nTotal wall time: {dt/60:.1f} min")

    # ---- write CSV ----
    DATA_DIR.mkdir(parents=True, exist_ok=True)
    PLOTS_DIR.mkdir(parents=True, exist_ok=True)
    csv_path = DATA_DIR / "grid_search_aco.csv"
    fieldnames = [
        "pheromone_weight", "heuristic_weight", "evaporation_rate",
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
        agg.setdefault((r.pheromone_weight, r.heuristic_weight,
                        r.evaporation_rate), []).append(r)
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
    lines.append(f"ACO grid search — top {args.top_k} configs "
                 f"(mean across {len(maps)} maps × "
                 f"{len(args.drones_list)} drone counts × "
                 f"{len(args.seeds)} seeds)\n\n")
    lines.append(f"{'rank':>4s}  {'α':>5s}  {'β':>5s}  {'ρ':>5s}  "
                 f"{'score':>7s}  {'cov':>6s}  "
                 f"{'overlap':>9s}  {'wasted':>7s}  {'energy':>9s}\n")
    lines.append("-" * 80 + "\n")
    for rank, (key, sc, cov, ov, wa, en) in enumerate(summary[: args.top_k], 1):
        a, b, rho = key
        lines.append(
            f"{rank:>4d}  {a:>5.2f}  {b:>5.2f}  {rho:>5.2f}  "
            f"{sc:>+7.3f}  {cov:>5.1f}%  {ov:>7.0f}m²  {wa:>7.0f}  "
            f"{en:>6.1f}kJ\n"
        )
    top_path = DATA_DIR / "grid_search_aco_top.txt"
    top_path.write_text("".join(lines))
    print()
    print("".join(lines), end="")
    print(f"Wrote: {top_path}")

    # ---- combined heatmap ----
    out = plot_heatmap(results, grid_axes, PLOTS_DIR)
    print(f"Wrote: {out}")


if __name__ == "__main__":
    main()
