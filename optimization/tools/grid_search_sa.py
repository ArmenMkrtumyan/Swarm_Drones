"""
Grid search over `SAController` hyperparameters across all maps × drone sizes.

Sweeps the three core SA knobs — `T_initial`, `cooling_rate`,
`perturb_radius`. Same shape and outputs as the other Track 2 grid searches.

Outputs (to `outputs/csv_txt/` and `outputs/png/`):
    grid_search_sa.csv
    grid_search_sa_top.txt
    grid_search_sa_heatmap_T{value}.png

Usage:
    python tools/grid_search_sa.py
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
from controllers.sa import SAConfig, SAController
from environment import CoverageEnv, DroneConfig, SimConfig
from maze import FREE, load_map
from score import composite_score


DEFAULT_GRID = {
    "T_initial":      [0.5, 1.0, 5.0],
    "cooling_rate":   [0.95, 0.99, 0.999],
    "perturb_radius": [3.0, 6.0, 10.0],
}


@dataclass
class SARunResult:
    T_initial: float
    cooling_rate: float
    perturb_radius: float
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
    grid, *, map_label, n_drones, seed,
    T_initial, cooling_rate, perturb_radius,
):
    env = CoverageEnv(grid=grid, n_drones=n_drones,
                      sim=SimConfig(), drone=DroneConfig())
    env.reset(seed=seed)
    cfg = SAConfig(T_initial=T_initial, cooling_rate=cooling_rate,
                   perturb_radius=perturb_radius)
    sa = SAController(cfg=cfg, hover_drone_idx=None, seed=seed)
    initial_energy = sum(d.battery_j for d in env.drones)

    while not env.is_terminal():
        env.step(sa(env))

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
    return SARunResult(
        T_initial=T_initial, cooling_rate=cooling_rate,
        perturb_radius=perturb_radius,
        map_label=map_label, n_drones=n_drones, seed=seed,
        final_coverage=env.coverage_fraction(),
        overlap_m2=env.overlap_cells_m2(),
        wasted_visits=env.wasted_visits_total(),
        energy_used_j=float(energy_used),
        time_to_terminal=env.time_seconds,
        score=s,
    )


def plot_heatmap(results, grid_axes, out_dir: Path) -> Path:
    """One combined PNG: 1 panel per T_initial, cooling_rate × perturb_radius per panel."""
    Ts = grid_axes["T_initial"]
    rates = grid_axes["cooling_rate"]
    perturbs = grid_axes["perturb_radius"]

    agg: dict = {}
    for r in results:
        agg.setdefault((r.T_initial, r.cooling_rate, r.perturb_radius),
                       []).append(r.score)
    mean_score = {k: float(np.mean(v)) for k, v in agg.items()}
    best_cfg = max(mean_score, key=mean_score.get)
    best_score = mean_score[best_cfg]
    vmin, vmax = min(mean_score.values()), max(mean_score.values())

    fig, axes = plt.subplots(1, len(Ts),
                             figsize=(4 * len(Ts) + 0.5, 3.6),
                             sharey=True)
    if len(Ts) == 1:
        axes = [axes]
    for ax, T in zip(axes, Ts):
        Z = np.array([
            [mean_score.get((T, cr, pr), float("nan")) for pr in perturbs]
            for cr in rates
        ])
        im = ax.imshow(Z, origin="lower", aspect="auto",
                       cmap="viridis", vmin=vmin, vmax=vmax)
        ax.set_xticks(range(len(perturbs)))
        ax.set_xticklabels([str(p) for p in perturbs], fontsize=8)
        ax.set_yticks(range(len(rates)))
        ax.set_yticklabels([str(r) for r in rates], fontsize=8)
        ax.set_xlabel("perturb σ", fontsize=9)
        if ax is axes[0]:
            ax.set_ylabel("cooling α", fontsize=9)
        ax.set_title(f"T_initial = {T}", fontsize=10)
        for i, cr in enumerate(rates):
            for j, pr in enumerate(perturbs):
                v = mean_score.get((T, cr, pr), None)
                if v is not None:
                    is_best = (T, cr, pr) == best_cfg
                    color = "white" if v < (vmin + vmax) / 2 else "black"
                    ax.text(j, i, f"{v:.2f}", ha="center", va="center",
                            color=color, fontsize=8,
                            fontweight="bold" if is_best else "normal")
    fig.suptitle(
        f"SA — grid search composite scores  (best: {best_score:+.3f})",
        fontsize=11, fontweight="bold",
    )
    fig.colorbar(im, ax=axes, fraction=0.025, pad=0.04, label="composite score")
    out = out_dir / "grid_search_sa_heatmap.png"
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
    p.add_argument("--T-grid",       type=float, nargs="+",
                   default=DEFAULT_GRID["T_initial"])
    p.add_argument("--cooling-grid", type=float, nargs="+",
                   default=DEFAULT_GRID["cooling_rate"])
    p.add_argument("--perturb-grid", type=float, nargs="+",
                   default=DEFAULT_GRID["perturb_radius"])
    p.add_argument("--top-k", type=int, default=10)
    return p.parse_args()


def main():
    args = parse_args()
    grid_axes = {
        "T_initial":      args.T_grid,
        "cooling_rate":   args.cooling_grid,
        "perturb_radius": args.perturb_grid,
    }

    maps = [(Path(p).stem, load_map(p)) for p in args.map_files]
    configs = list(product(grid_axes["T_initial"],
                           grid_axes["cooling_rate"],
                           grid_axes["perturb_radius"]))
    n_runs = (len(configs) * len(maps) * len(args.drones_list)
              * len(args.seeds))

    print(f"=== SA grid search ===")
    print(f"  T_initial:     {grid_axes['T_initial']}")
    print(f"  cooling_rate:  {grid_axes['cooling_rate']}")
    print(f"  perturb_radius: {grid_axes['perturb_radius']}")
    print(f"  total runs:    {n_runs}\n")

    results = []
    t0 = time.time()
    runs_done = 0
    for T, cr, pr in configs:
        for mlabel, mgrid in maps:
            for n in args.drones_list:
                for seed in args.seeds:
                    r = run_one(
                        mgrid, map_label=mlabel, n_drones=n, seed=seed,
                        T_initial=T, cooling_rate=cr, perturb_radius=pr,
                    )
                    results.append(r)
                    runs_done += 1
        cfg_results = [r for r in results
                       if r.T_initial == T and r.cooling_rate == cr
                       and r.perturb_radius == pr]
        mean_s = float(np.mean([r.score for r in cfg_results]))
        mean_cov = float(np.mean([r.final_coverage for r in cfg_results])) * 100
        print(f"  [{runs_done}/{n_runs}] T={T} α={cr} σ={pr}  "
              f"score={mean_s:+.3f}  cov={mean_cov:.1f}%")

    dt = time.time() - t0
    print(f"\nTotal wall time: {dt/60:.1f} min")

    DATA_DIR.mkdir(parents=True, exist_ok=True)
    PLOTS_DIR.mkdir(parents=True, exist_ok=True)
    csv_path = DATA_DIR / "grid_search_sa.csv"
    fieldnames = [
        "T_initial", "cooling_rate", "perturb_radius",
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

    agg: dict = {}
    for r in results:
        agg.setdefault((r.T_initial, r.cooling_rate, r.perturb_radius),
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
    lines.append(f"SA grid search — top {args.top_k} configs\n\n")
    lines.append(f"{'rank':>4s}  {'T':>5s}  {'α':>6s}  {'σ':>5s}  "
                 f"{'score':>7s}  {'cov':>6s}  "
                 f"{'overlap':>9s}  {'wasted':>7s}  {'energy':>9s}\n")
    lines.append("-" * 80 + "\n")
    for rank, (key, sc, cov, ov, wa, en) in enumerate(summary[: args.top_k], 1):
        T, cr, pr = key
        lines.append(
            f"{rank:>4d}  {T:>5.2f}  {cr:>6.3f}  {pr:>5.2f}  "
            f"{sc:>+7.3f}  {cov:>5.1f}%  {ov:>7.0f}m²  {wa:>7.0f}  "
            f"{en:>6.1f}kJ\n"
        )
    top_path = DATA_DIR / "grid_search_sa_top.txt"
    top_path.write_text("".join(lines))
    print()
    print("".join(lines), end="")
    print(f"Wrote: {top_path}")

    out = plot_heatmap(results, grid_axes, PLOTS_DIR)
    print(f"Wrote: {out}")


if __name__ == "__main__":
    main()
