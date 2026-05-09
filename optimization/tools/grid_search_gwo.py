"""
Grid search over `GWOController` hyperparameters across all maps × drone sizes.

Sweeps `a_initial`, `a_final`, `decay_steps`. Same shape and outputs as
the other Track 2 grid searches.

Outputs (to `outputs/csv_txt/` and `outputs/png/`):
    grid_search_gwo.csv
    grid_search_gwo_top.txt
    grid_search_gwo_heatmap_decay{value}.png
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
from controllers.gwo import GWOConfig, GWOController
from environment import CoverageEnv, DroneConfig, SimConfig
from maze import FREE, load_map
from score import composite_score


DEFAULT_GRID = {
    "a_initial":   [2.0, 3.0, 5.0],
    "a_final":     [0.0, 0.5, 1.0],
    "decay_steps": [500, 1500, 3000],
}


@dataclass
class GWORunResult:
    a_initial: float
    a_final: float
    decay_steps: int
    map_label: str
    n_drones: int
    seed: int
    final_coverage: float
    overlap_m2: float
    wasted_visits: int
    energy_used_j: float
    time_to_terminal: float
    score: float


def run_one(grid, *, map_label, n_drones, seed,
            a_initial, a_final, decay_steps):
    env = CoverageEnv(grid=grid, n_drones=n_drones,
                      sim=SimConfig(), drone=DroneConfig())
    env.reset(seed=seed)
    cfg = GWOConfig(a_initial=a_initial, a_final=a_final,
                    decay_steps=decay_steps)
    gwo = GWOController(cfg=cfg, hover_drone_idx=None, seed=seed)
    initial_energy = sum(d.battery_j for d in env.drones)

    while not env.is_terminal():
        env.step(gwo(env))

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
    return GWORunResult(
        a_initial=a_initial, a_final=a_final, decay_steps=decay_steps,
        map_label=map_label, n_drones=n_drones, seed=seed,
        final_coverage=env.coverage_fraction(),
        overlap_m2=env.overlap_cells_m2(),
        wasted_visits=env.wasted_visits_total(),
        energy_used_j=float(energy_used),
        time_to_terminal=env.time_seconds,
        score=s,
    )


def plot_heatmap(results, grid_axes, out_dir: Path) -> Path:
    """One combined PNG: 1 panel per decay_steps, a_initial × a_final per panel."""
    decays = grid_axes["decay_steps"]
    a0s = grid_axes["a_initial"]
    afs = grid_axes["a_final"]

    agg: dict = {}
    for r in results:
        agg.setdefault((r.decay_steps, r.a_initial, r.a_final),
                       []).append(r.score)
    mean_score = {k: float(np.mean(v)) for k, v in agg.items()}
    best_cfg = max(mean_score, key=mean_score.get)
    best_score = mean_score[best_cfg]
    vmin, vmax = min(mean_score.values()), max(mean_score.values())

    fig, axes = plt.subplots(1, len(decays),
                             figsize=(4 * len(decays) + 0.5, 3.6),
                             sharey=True)
    if len(decays) == 1:
        axes = [axes]
    for ax, decay in zip(axes, decays):
        Z = np.array([
            [mean_score.get((decay, a0, af), float("nan")) for af in afs]
            for a0 in a0s
        ])
        im = ax.imshow(Z, origin="lower", aspect="auto",
                       cmap="viridis", vmin=vmin, vmax=vmax)
        ax.set_xticks(range(len(afs)))
        ax.set_xticklabels([str(v) for v in afs], fontsize=8)
        ax.set_yticks(range(len(a0s)))
        ax.set_yticklabels([str(v) for v in a0s], fontsize=8)
        ax.set_xlabel("a_final", fontsize=9)
        if ax is axes[0]:
            ax.set_ylabel("a_initial", fontsize=9)
        ax.set_title(f"decay_steps = {decay}", fontsize=10)
        for i, a0 in enumerate(a0s):
            for j, af in enumerate(afs):
                v = mean_score.get((decay, a0, af), None)
                if v is not None:
                    is_best = (decay, a0, af) == best_cfg
                    color = "white" if v < (vmin + vmax) / 2 else "black"
                    ax.text(j, i, f"{v:.2f}", ha="center", va="center",
                            color=color, fontsize=8,
                            fontweight="bold" if is_best else "normal")
    fig.suptitle(
        f"GWO — grid search composite scores  (best: {best_score:+.3f})",
        fontsize=11, fontweight="bold",
    )
    fig.colorbar(im, ax=axes, fraction=0.025, pad=0.04, label="composite score")
    out = out_dir / "grid_search_gwo_heatmap.png"
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
    p.add_argument("--a0-grid",    type=float, nargs="+",
                   default=DEFAULT_GRID["a_initial"])
    p.add_argument("--af-grid",    type=float, nargs="+",
                   default=DEFAULT_GRID["a_final"])
    p.add_argument("--decay-grid", type=int,   nargs="+",
                   default=DEFAULT_GRID["decay_steps"])
    p.add_argument("--top-k", type=int, default=10)
    return p.parse_args()


def main():
    args = parse_args()
    grid_axes = {
        "a_initial":   args.a0_grid,
        "a_final":     args.af_grid,
        "decay_steps": args.decay_grid,
    }

    maps = [(Path(p).stem, load_map(p)) for p in args.map_files]
    configs = list(product(grid_axes["a_initial"],
                           grid_axes["a_final"],
                           grid_axes["decay_steps"]))
    n_runs = (len(configs) * len(maps) * len(args.drones_list)
              * len(args.seeds))

    print(f"=== GWO grid search ===")
    print(f"  a_initial:   {grid_axes['a_initial']}")
    print(f"  a_final:     {grid_axes['a_final']}")
    print(f"  decay_steps: {grid_axes['decay_steps']}")
    print(f"  total runs:  {n_runs}\n")

    results = []
    t0 = time.time()
    runs_done = 0
    for a0, af, ds in configs:
        for mlabel, mgrid in maps:
            for n in args.drones_list:
                for seed in args.seeds:
                    r = run_one(
                        mgrid, map_label=mlabel, n_drones=n, seed=seed,
                        a_initial=a0, a_final=af, decay_steps=ds,
                    )
                    results.append(r)
                    runs_done += 1
        cfg_results = [r for r in results
                       if r.a_initial == a0 and r.a_final == af
                       and r.decay_steps == ds]
        mean_s = float(np.mean([r.score for r in cfg_results]))
        mean_cov = float(np.mean([r.final_coverage for r in cfg_results])) * 100
        print(f"  [{runs_done}/{n_runs}] a0={a0} af={af} decay={ds}  "
              f"score={mean_s:+.3f}  cov={mean_cov:.1f}%")

    dt = time.time() - t0
    print(f"\nTotal wall time: {dt/60:.1f} min")

    DATA_DIR.mkdir(parents=True, exist_ok=True)
    PLOTS_DIR.mkdir(parents=True, exist_ok=True)
    csv_path = DATA_DIR / "grid_search_gwo.csv"
    fieldnames = [
        "a_initial", "a_final", "decay_steps",
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
        agg.setdefault((r.a_initial, r.a_final, r.decay_steps),
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
    lines.append(f"GWO grid search — top {args.top_k} configs\n\n")
    lines.append(f"{'rank':>4s}  {'a0':>5s}  {'af':>5s}  {'decay':>6s}  "
                 f"{'score':>7s}  {'cov':>6s}  "
                 f"{'overlap':>9s}  {'wasted':>7s}  {'energy':>9s}\n")
    lines.append("-" * 80 + "\n")
    for rank, (key, sc, cov, ov, wa, en) in enumerate(summary[: args.top_k], 1):
        a0, af, ds = key
        lines.append(
            f"{rank:>4d}  {a0:>5.2f}  {af:>5.2f}  {ds:>6d}  "
            f"{sc:>+7.3f}  {cov:>5.1f}%  {ov:>7.0f}m²  {wa:>7.0f}  "
            f"{en:>6.1f}kJ\n"
        )
    top_path = DATA_DIR / "grid_search_gwo_top.txt"
    top_path.write_text("".join(lines))
    print()
    print("".join(lines), end="")
    print(f"Wrote: {top_path}")

    out = plot_heatmap(results, grid_axes, PLOTS_DIR)
    print(f"Wrote: {out}")


if __name__ == "__main__":
    main()
