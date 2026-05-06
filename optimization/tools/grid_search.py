"""
Grid search over the most-impactful hyperparameters for `PotentialFieldsController`
or `ConsensusController`.

Strategy:
    For the chosen controller, build a small cartesian grid over three knobs
    (the ones that most directly shape behaviour):

        attract_gain        — how aggressively each drone chases its target
        drone_repel_gain    — how strongly drones push each other apart
        drone_repel_range   — radius (in cells) within which repulsion acts

    Each (attract, repel_gain, repel_range) cell is evaluated by running
    the controller on a fixed (map, n_drones, seed) tuple — multiple seeds
    are averaged so a lucky seed doesn't win on noise. A single composite
    score from `score.composite_score` collapses the multi-criteria
    objective into a scalar.

    Defaults are chosen so a full grid finishes in ~15-30 minutes:
        3 × 3 × 3 = 27 configs × 3 seeds = 81 runs
        (one map, one n_drones; sweep across maps/swarms is what
        `tools/sweep.py` is for — this tool tunes a *single* operating point.)

Outputs (to `outputs/images/`):
    grid_search_<policy>.csv       — every config + per-seed metrics + score
    grid_search_<policy>_top.txt   — top-K configs ranked by mean score
    grid_search_<policy>_heatmap.png — score heatmap, one panel per repel_range

Usage:
    python tools/grid_search.py --policy pf
    python tools/grid_search.py --policy consensus --map-file outputs/maps/closed_33.npy --drones 4
"""

from __future__ import annotations

import argparse
import csv
import sys
import time
from dataclasses import dataclass
from itertools import product
from pathlib import Path
from typing import Optional

# Make project root importable.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from constants import DATA_DIR, MAPS_DIR, PLOTS_DIR
from controllers.consensus import ConsensusConfig, ConsensusController
from controllers.potential_fields import PFConfig, PotentialFieldsController
from environment import CoverageEnv, DroneConfig, SimConfig
from maze import FREE, load_map
from score import DEFAULT_WEIGHTS, composite_score


# Default grid — 3 × 3 × 3 = 27 configs. Centered around each controller's
# defaults so the existing operating point falls inside the grid.
DEFAULT_GRID = {
    "attract_gain":     [0.5, 1.5, 3.0],
    "drone_repel_gain": [2.0, 5.0, 10.0],
    "drone_repel_range": [1.5, 2.5, 4.0],
}


@dataclass
class GridResult:
    attract_gain: float
    drone_repel_gain: float
    drone_repel_range: float
    seed: int
    final_coverage: float
    overlap_m2: float
    wasted_visits: int
    energy_used_j: float
    time_to_terminal: float
    score: float


def make_controller(
    policy: str,
    attract: float,
    repel_gain: float,
    repel_range: float,
):
    """Instantiate the chosen controller with the grid-point gains."""
    if policy == "pf":
        cfg = PFConfig(
            attract_gain=attract,
            drone_repel_gain=repel_gain,
            drone_repel_range=repel_range,
        )
        return PotentialFieldsController(cfg=cfg, hover_drone_idx=None)
    elif policy == "consensus":
        cfg = ConsensusConfig(
            attract_gain=attract,
            drone_repel_gain=repel_gain,
            drone_repel_range=repel_range,
        )
        return ConsensusController(cfg=cfg, hover_drone_idx=None)
    else:
        raise ValueError(f"unknown policy: {policy}")


def run_one(
    policy: str,
    *,
    grid: np.ndarray,
    n_drones: int,
    seed: int,
    attract: float,
    repel_gain: float,
    repel_range: float,
) -> GridResult:
    """Run the controller-with-these-gains once on (grid, n_drones, seed)."""
    env = CoverageEnv(
        grid=grid,
        n_drones=n_drones,
        sim=SimConfig(),
        drone=DroneConfig(),
    )
    env.reset(seed=seed)
    controller = make_controller(policy, attract, repel_gain, repel_range)
    initial_energy = sum(d.battery_j for d in env.drones)

    while not env.is_terminal():
        env.step(controller(env))

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
    return GridResult(
        attract_gain=attract,
        drone_repel_gain=repel_gain,
        drone_repel_range=repel_range,
        seed=seed,
        final_coverage=env.coverage_fraction(),
        overlap_m2=env.overlap_cells_m2(),
        wasted_visits=env.wasted_visits_total(),
        energy_used_j=float(energy_used),
        time_to_terminal=env.time_seconds,
        score=s,
    )


def plot_score_heatmap(
    rows: list[GridResult],
    grid_axes: dict,
    out_path: Path,
    policy: str,
) -> None:
    """One panel per drone_repel_range; axes = attract_gain × drone_repel_gain."""
    ranges = grid_axes["drone_repel_range"]
    attracts = grid_axes["attract_gain"]
    repels = grid_axes["drone_repel_gain"]

    fig, axes = plt.subplots(1, len(ranges), figsize=(4.5 * len(ranges), 4.0))
    if len(ranges) == 1:
        axes = [axes]

    # Aggregate by (attract, repel_gain, repel_range) → mean score
    mean_score: dict = {}
    for r in rows:
        key = (r.attract_gain, r.drone_repel_gain, r.drone_repel_range)
        mean_score.setdefault(key, []).append(r.score)
    mean_score = {k: float(np.mean(v)) for k, v in mean_score.items()}

    all_scores = list(mean_score.values())
    vmin, vmax = min(all_scores), max(all_scores)

    for ax, rr in zip(axes, ranges):
        Z = np.array([
            [mean_score.get((a, rg, rr), float("nan")) for rg in repels]
            for a in attracts
        ])
        im = ax.imshow(Z, origin="lower", aspect="auto",
                       cmap="viridis", vmin=vmin, vmax=vmax)
        ax.set_xticks(range(len(repels)))
        ax.set_xticklabels(repels)
        ax.set_yticks(range(len(attracts)))
        ax.set_yticklabels(attracts)
        ax.set_xlabel("drone_repel_gain")
        ax.set_ylabel("attract_gain")
        ax.set_title(f"repel_range = {rr}")
        # Annotate each cell with the score.
        for i, a in enumerate(attracts):
            for j, rg in enumerate(repels):
                v = mean_score.get((a, rg, rr), None)
                if v is not None:
                    ax.text(j, i, f"{v:.2f}", ha="center", va="center",
                            color="white" if v < (vmin + vmax) / 2 else "black",
                            fontsize=8)
        fig.colorbar(im, ax=ax, fraction=0.046, pad=0.04)

    fig.suptitle(
        f"Grid search — {policy.upper()}: composite score (higher = better)",
        fontsize=12, fontweight="bold",
    )
    fig.tight_layout()
    fig.savefig(out_path, dpi=130)
    plt.close(fig)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--policy", choices=["pf", "consensus"], required=True)
    p.add_argument("--map-file", type=str,
                   default=str(MAPS_DIR / "partial_33.npy"),
                   help="single map for the grid search (default partial_33)")
    p.add_argument("--drones", type=int, default=5,
                   help="swarm size for the grid search (default 5)")
    p.add_argument("--seeds", type=int, nargs="+", default=[0, 1, 2],
                   help="seeds to average over (default 0 1 2)")
    p.add_argument("--top-k", type=int, default=10,
                   help="how many top configs to print at the end")
    # Optional grid overrides — useful for sparser-swarm targeting where the
    # default centered-on-defaults grid covers the wrong region.
    p.add_argument("--attract-grid", type=float, nargs="+", default=None,
                   help="override attract_gain values (default 0.5 1.5 3.0)")
    p.add_argument("--repel-gain-grid", type=float, nargs="+", default=None,
                   help="override drone_repel_gain values (default 2.0 5.0 10.0)")
    p.add_argument("--repel-range-grid", type=float, nargs="+", default=None,
                   help="override drone_repel_range values (default 1.5 2.5 4.0)")
    p.add_argument("--out-tag", type=str, default=None,
                   help="suffix added to output filenames "
                        "(e.g. 'sparse_n2' → grid_search_pf_sparse_n2.csv). "
                        "Default: empty (overwrites the main file).")
    return p.parse_args()


def main() -> None:
    args = parse_args()
    grid_path = Path(args.map_file)
    if not grid_path.exists():
        raise FileNotFoundError(f"map file not found: {grid_path}")
    grid = load_map(str(grid_path))

    grid_axes = {
        "attract_gain":     args.attract_grid     or DEFAULT_GRID["attract_gain"],
        "drone_repel_gain": args.repel_gain_grid  or DEFAULT_GRID["drone_repel_gain"],
        "drone_repel_range": args.repel_range_grid or DEFAULT_GRID["drone_repel_range"],
    }
    configs = list(product(
        grid_axes["attract_gain"],
        grid_axes["drone_repel_gain"],
        grid_axes["drone_repel_range"],
    ))

    n_runs = len(configs) * len(args.seeds)
    print(f"=== Grid search — {args.policy.upper()} ===")
    print(f"  map:        {grid_path.name}  ({grid.shape[0]}×{grid.shape[1]})")
    print(f"  drones:     {args.drones}")
    print(f"  seeds:      {args.seeds}")
    print(f"  grid:       {grid_axes}")
    print(f"  configs:    {len(configs)}  (×{len(args.seeds)} seeds = {n_runs} runs)")
    print(f"  weights:    {DEFAULT_WEIGHTS}")
    print()

    results: list[GridResult] = []
    t0 = time.time()
    for i, (a, rg, rr) in enumerate(configs, 1):
        per_seed: list[GridResult] = []
        for seed in args.seeds:
            t_run = time.time()
            r = run_one(
                args.policy,
                grid=grid, n_drones=args.drones, seed=seed,
                attract=a, repel_gain=rg, repel_range=rr,
            )
            per_seed.append(r)
            results.append(r)
            print(f"  [{i:>2d}/{len(configs)}] a={a:>4.1f} rg={rg:>5.1f} "
                  f"rr={rr:>4.1f} seed={seed}  "
                  f"cov={r.final_coverage:6.1%}  "
                  f"score={r.score:>+5.3f}  "
                  f"({time.time() - t_run:.1f}s)")
        mean_s = float(np.mean([r.score for r in per_seed]))
        mean_c = float(np.mean([r.final_coverage for r in per_seed])) * 100
        print(f"      => mean score={mean_s:+.3f}  mean cov={mean_c:.1f}%")
    dt = time.time() - t0
    print(f"\nTotal wall time: {dt:.0f} s ({dt/60:.1f} min)")

    # ---- write CSV ----
    DATA_DIR.mkdir(parents=True, exist_ok=True)
    PLOTS_DIR.mkdir(parents=True, exist_ok=True)
    csv_path = DATA_DIR / f"grid_search_{args.policy}{('_' + args.out_tag) if args.out_tag else ''}.csv"
    with csv_path.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=[
            "attract_gain", "drone_repel_gain", "drone_repel_range", "seed",
            "final_coverage", "overlap_m2", "wasted_visits",
            "energy_used_j", "time_to_terminal", "score",
        ])
        w.writeheader()
        for r in results:
            w.writerow(r.__dict__)
    print(f"Wrote: {csv_path}")

    # ---- aggregate by config (mean over seeds) ----
    agg: dict = {}
    for r in results:
        key = (r.attract_gain, r.drone_repel_gain, r.drone_repel_range)
        agg.setdefault(key, []).append(r)
    summary = sorted(
        [
            (k,
             float(np.mean([x.score for x in v])),
             float(np.mean([x.final_coverage for x in v])),
             float(np.mean([x.overlap_m2 for x in v])),
             float(np.mean([x.wasted_visits for x in v])),
             float(np.mean([x.energy_used_j for x in v])) / 1000,
             )
            for k, v in agg.items()
        ],
        key=lambda x: -x[1],
    )

    # ---- top-K to stdout AND a tracked .txt file ----
    top_path = DATA_DIR / f"grid_search_{args.policy}{('_' + args.out_tag) if args.out_tag else ''}_top.txt"
    lines = []
    lines.append(f"Top {args.top_k} configs for {args.policy.upper()} on "
                 f"{grid_path.name} (n={args.drones}, seeds={args.seeds})\n")
    lines.append(f"{'rank':>4s}  {'attract':>7s}  {'repel_g':>7s}  "
                 f"{'repel_r':>7s}  {'score':>7s}  {'cov':>6s}  "
                 f"{'overlap':>9s}  {'wasted':>7s}  {'energy':>9s}\n")
    lines.append("-" * 80 + "\n")
    for rank, (key, s, cov, ov, w, e_kj) in enumerate(summary[: args.top_k], 1):
        a, rg, rr = key
        lines.append(
            f"{rank:>4d}  {a:>7.2f}  {rg:>7.2f}  {rr:>7.2f}  "
            f"{s:>+7.3f}  {cov:>5.1%}  {ov:>7.0f}m²  {w:>7.0f}  {e_kj:>6.1f}kJ\n"
        )
    top_path.write_text("".join(lines))
    print()
    print("".join(lines), end="")
    print(f"Wrote: {top_path}")

    # ---- heatmap PNG ----
    heatmap_path = PLOTS_DIR / f"grid_search_{args.policy}{('_' + args.out_tag) if args.out_tag else ''}_heatmap.png"
    plot_score_heatmap(results, grid_axes, heatmap_path, args.policy)
    print(f"Wrote: {heatmap_path}")


if __name__ == "__main__":
    main()
