"""
Verify the grid-search winning gains generalize across (map × n_drones).

`tools/grid_search.py` tunes a single operating point (partial_33 × n=5).
Before promoting those gains to defaults, we want to know whether they
also win on different maps and different swarm sizes — or whether the
optimum is operating-point-specific.

This script runs four policies on the same (map, n_drones, seed) grid
the existing sweep uses:
    - PF-default       (PFConfig defaults)
    - PF-tuned         (best gains from grid_search PF)
    - Consensus-default
    - Consensus-tuned

For each cell, prints the mean composite score per policy and the
default → tuned delta. Wins/losses are summarized at the bottom.

Outputs:
    outputs/images/verify_tuned.csv   per-run records
    outputs/images/verify_tuned.txt   summary table

Usage:
    python tools/verify_tuned.py                  # uses defaults below
    python tools/verify_tuned.py --seeds 0 1 2 3 4
"""

from __future__ import annotations

import argparse
import csv
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import numpy as np

from constants import DATA_DIR, MAPS_DIR
from controllers.consensus import ConsensusConfig, ConsensusController
from controllers.potential_fields import PFConfig, PotentialFieldsController
from environment import CoverageEnv, DroneConfig, SimConfig
from maze import FREE, load_map
from score import composite_score


# Tuned values from `tools/grid_search.py` on partial_33 × n=5 (top-1 for both
# controllers). Same triplet won both — likely the structural intuition
# generalizes ("higher attract + wider personal space").
TUNED = dict(attract_gain=3.0, drone_repel_gain=5.0, drone_repel_range=4.0)


def make_policies():
    """The four (label, callable) pairs we'll evaluate per cell."""
    return [
        ("PF-default",        PotentialFieldsController(hover_drone_idx=None)),
        ("PF-tuned",          PotentialFieldsController(
            cfg=PFConfig(**TUNED), hover_drone_idx=None)),
        ("Consensus-default", ConsensusController(hover_drone_idx=None)),
        ("Consensus-tuned",   ConsensusController(
            cfg=ConsensusConfig(**TUNED), hover_drone_idx=None)),
    ]


def run_one(grid: np.ndarray, n_drones: int, seed: int, policy_fn) -> dict:
    env = CoverageEnv(
        grid=grid, n_drones=n_drones,
        sim=SimConfig(), drone=DroneConfig(),
    )
    env.reset(seed=seed)
    initial_energy = sum(d.battery_j for d in env.drones)
    while not env.is_terminal():
        env.step(policy_fn(env))
    energy_used = initial_energy - sum(d.battery_j for d in env.drones)

    free_mask = env.grid == FREE
    free_area_m2 = float(free_mask.sum()) * (env.sim_cfg.meters_per_cell ** 2)
    unique_visited = int((env.covered & free_mask).sum())
    energy_budget = env.battery_cfg.initial_energy_j * env.n_drones

    return {
        "final_coverage": env.coverage_fraction(),
        "overlap_m2": env.overlap_cells_m2(),
        "wasted_visits": env.wasted_visits_total(),
        "energy_used_j": float(energy_used),
        "time_to_terminal": env.time_seconds,
        "score": composite_score(
            coverage_fraction=env.coverage_fraction(),
            overlap_m2=env.overlap_cells_m2(),
            free_area_m2=free_area_m2,
            wasted_visits=env.wasted_visits_total(),
            unique_cells_visited=unique_visited,
            energy_used_j=float(energy_used),
            energy_budget_j=energy_budget,
        ),
    }


def parse_args():
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--map-files", type=str, nargs="+", default=[
        str(MAPS_DIR / "open_33.npy"),
        str(MAPS_DIR / "partial_33.npy"),
        str(MAPS_DIR / "closed_33.npy"),
    ])
    p.add_argument("--drones-list", type=int, nargs="+", default=[2, 5, 10, 20])
    p.add_argument("--seeds", type=int, nargs="+", default=[0, 1, 2])
    return p.parse_args()


def main():
    args = parse_args()

    maps = []
    for p in args.map_files:
        grid = load_map(p)
        maps.append((Path(p).stem, grid))

    policies = make_policies()
    n_runs = len(maps) * len(args.drones_list) * len(args.seeds) * len(policies)
    print(f"=== Verify tuned gains on {n_runs} runs "
          f"({len(maps)} maps × {len(args.drones_list)} drone counts × "
          f"{len(args.seeds)} seeds × {len(policies)} policies) ===")
    print(f"  tuned values: {TUNED}")
    print()

    results = []
    t0 = time.time()
    for mlabel, grid in maps:
        for n in args.drones_list:
            for seed in args.seeds:
                for pname, pfn in policies:
                    t_run = time.time()
                    r = run_one(grid, n, seed, pfn)
                    r.update(map_label=mlabel, n_drones=n, seed=seed, policy=pname)
                    results.append(r)
                    print(f"  {mlabel:<12s} n={n:>2d} seed={seed} "
                          f"{pname:<19s}  cov={r['final_coverage']:6.1%}  "
                          f"score={r['score']:>+5.3f}  ({time.time() - t_run:.1f}s)")
    dt = time.time() - t0
    print(f"\nTotal wall time: {dt:.0f} s  ({dt / 60:.1f} min)")

    # ---- write CSV ----
    DATA_DIR.mkdir(parents=True, exist_ok=True)
    csv_path = DATA_DIR / "verify_tuned.csv"
    fieldnames = [
        "policy", "map_label", "n_drones", "seed",
        "final_coverage", "overlap_m2", "wasted_visits",
        "energy_used_j", "time_to_terminal", "score",
    ]
    with csv_path.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        for r in results:
            w.writerow({k: r[k] for k in fieldnames})
    print(f"Wrote: {csv_path}")

    # ---- aggregate by (map, n_drones, policy) over seeds ----
    from collections import defaultdict
    agg = defaultdict(list)
    for r in results:
        agg[(r["map_label"], r["n_drones"], r["policy"])].append(r["score"])

    summary_lines = []
    for controller in ["PF", "Consensus"]:
        summary_lines.append(f"\n=== {controller}: default vs tuned ===")
        summary_lines.append(
            f"  {'map':<12s} {'n':>3s}  {'default':>8s}  {'tuned':>8s}  "
            f"{'Δscore':>8s}   verdict")
        summary_lines.append("  " + "-" * 70)
        wins = losses = ties = 0
        for mlabel, _ in maps:
            for n in args.drones_list:
                d = np.mean(agg.get((mlabel, n, f"{controller}-default"), [0]))
                t = np.mean(agg.get((mlabel, n, f"{controller}-tuned"), [0]))
                delta = t - d
                if abs(delta) < 0.01:
                    verdict = "tie"
                    ties += 1
                elif delta > 0:
                    verdict = "TUNED ✓"
                    wins += 1
                else:
                    verdict = "default"
                    losses += 1
                summary_lines.append(
                    f"  {mlabel:<12s} {n:>3d}  {d:>+7.3f}  {t:>+7.3f}  "
                    f"{delta:>+7.3f}   {verdict}")
        summary_lines.append(
            f"  --> {controller}: tuned wins {wins}, "
            f"loses {losses}, ties {ties} of {wins + losses + ties} cells")

    summary = "\n".join(summary_lines) + "\n"
    print(summary)

    txt_path = DATA_DIR / "verify_tuned.txt"
    txt_path.write_text(summary)
    print(f"Wrote: {txt_path}")


if __name__ == "__main__":
    main()
