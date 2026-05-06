"""
Random search across all 7 hyperparameters for PF or Consensus.

Why random over grid:
    Bergstra & Bengio (2012, "Random Search for Hyper-Parameter Optimization")
    showed that for high-dimensional problems where most parameters don't
    matter much, random search at fixed budget covers the relevant axes
    much faster than grid search. PF/Consensus have 7 knobs each — a 3-value
    grid would be 3⁷ = 2187 configs; random search at 100 trials covers the
    same volume far better.

Search space (uniform in the ranges below):
    attract_gain              [0.3, 4.0]
    drone_repel_gain          [0.5, 12.0]
    drone_repel_range         [0.3, 5.0]   cells
    wall_repel_gain           [0.5, 5.0]
    wall_repel_range          [0.5, 3.0]   cells
    yaw_align_gain            [2.0, 12.0]
    velocity_align_threshold  [0.01, 0.2]  cells/s

Each trial = N seeds averaged. Score is `score.composite_score`.

Outputs (to `outputs/images/`):
    random_search_<policy>.csv         — all trials with per-seed metrics
    random_search_<policy>_top.txt     — top-K by mean score
    random_search_<policy>_progress.png — best-so-far score vs trial number

Usage:
    python tools/random_search.py --policy pf --trials 100
    python tools/random_search.py --policy consensus --trials 60 --drones 5
"""

from __future__ import annotations

import argparse
import csv
import sys
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

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


# Search space — uniform sampling within each [lo, hi] interval.
# `broad` is the initial exploration range; `refined` is a tighter region
# around the top-10 cluster from the broad search (used to confirm the
# winner is robust and squeeze out marginal gains).
SEARCH_SPACES = {
    "broad": {
        "attract_gain":             (0.3, 4.0),
        "drone_repel_gain":         (0.5, 12.0),
        "drone_repel_range":        (0.3, 5.0),
        "wall_repel_gain":          (0.5, 5.0),
        "wall_repel_range":         (0.5, 3.0),
        "yaw_align_gain":           (2.0, 12.0),
        "velocity_align_threshold": (0.01, 0.2),
    },
    "refined": {
        "attract_gain":             (1.5, 3.5),
        "drone_repel_gain":         (2.0, 8.0),
        "drone_repel_range":        (2.5, 5.0),
        "wall_repel_gain":          (2.0, 5.0),
        "wall_repel_range":         (0.8, 2.0),
        "yaw_align_gain":           (2.0, 6.0),
        "velocity_align_threshold": (0.05, 0.2),
    },
}
SEARCH_SPACE = SEARCH_SPACES["broad"]   # default; overridden by `--space refined`


@dataclass
class TrialResult:
    trial: int
    seed: int
    attract_gain: float
    drone_repel_gain: float
    drone_repel_range: float
    wall_repel_gain: float
    wall_repel_range: float
    yaw_align_gain: float
    velocity_align_threshold: float
    final_coverage: float
    overlap_m2: float
    wasted_visits: int
    energy_used_j: float
    time_to_terminal: float
    score: float


def sample_config(rng: np.random.Generator) -> dict[str, float]:
    """Draw one config from the search space."""
    return {k: float(rng.uniform(lo, hi)) for k, (lo, hi) in SEARCH_SPACE.items()}


def make_controller(policy: str, cfg_dict: dict):
    """Build the controller from a sampled config dict."""
    if policy == "pf":
        return PotentialFieldsController(cfg=PFConfig(**cfg_dict),
                                         hover_drone_idx=None)
    elif policy == "consensus":
        return ConsensusController(cfg=ConsensusConfig(**cfg_dict),
                                   hover_drone_idx=None)
    raise ValueError(f"unknown policy: {policy}")


def evaluate(
    policy: str, cfg_dict: dict, *, grid: np.ndarray, n_drones: int, seed: int,
) -> dict:
    """Run one (policy, cfg, seed) and return the metric bundle."""
    env = CoverageEnv(grid=grid, n_drones=n_drones,
                      sim=SimConfig(), drone=DroneConfig())
    env.reset(seed=seed)
    controller = make_controller(policy, cfg_dict)
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
    return {
        "final_coverage": env.coverage_fraction(),
        "overlap_m2": env.overlap_cells_m2(),
        "wasted_visits": env.wasted_visits_total(),
        "energy_used_j": float(energy_used),
        "time_to_terminal": env.time_seconds,
        "score": s,
    }


def plot_progress(per_trial_mean_score: list[float], out_path: Path,
                  policy: str, default_score: float) -> None:
    """Best-so-far score vs trial number, with the default config marked."""
    best = np.maximum.accumulate(per_trial_mean_score)
    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.plot(range(1, len(best) + 1), best, color="#1f77b4",
            linewidth=2, label="best mean score so far")
    ax.scatter(range(1, len(per_trial_mean_score) + 1), per_trial_mean_score,
               color="#aaa", s=12, label="per-trial mean", zorder=2)
    ax.axhline(default_score, color="#d62728", linestyle="--",
               linewidth=1.5, label=f"default config = {default_score:+.3f}")
    ax.set_xlabel("Trial number")
    ax.set_ylabel("Composite score")
    ax.set_title(f"Random search progress — {policy.upper()}")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="lower right")
    fig.tight_layout()
    fig.savefig(out_path, dpi=130)
    plt.close(fig)


def parse_args():
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--policy", choices=["pf", "consensus"], required=True)
    p.add_argument("--trials", type=int, default=100,
                   help="number of random configs to sample (default 100)")
    p.add_argument("--seeds", type=int, nargs="+", default=[0, 1, 2],
                   help="env seeds averaged per trial (default 0 1 2)")
    p.add_argument("--drones", type=int, default=5,
                   help="swarm size for the search (default 5)")
    p.add_argument("--map-file", type=str,
                   default=str(MAPS_DIR / "partial_33.npy"))
    p.add_argument("--seed", type=int, default=42,
                   help="RNG seed for the config sampler (default 42)")
    p.add_argument("--top-k", type=int, default=10)
    p.add_argument("--out-tag", type=str, default=None,
                   help="suffix added to output filenames")
    p.add_argument("--space", choices=list(SEARCH_SPACES.keys()),
                   default="broad",
                   help="sampling-range preset: 'broad' (initial exploration) "
                        "or 'refined' (tighter region around top-10 from broad). "
                        "Default: broad.")
    return p.parse_args()


def main():
    args = parse_args()
    grid_path = Path(args.map_file)
    if not grid_path.exists():
        raise FileNotFoundError(f"map file not found: {grid_path}")
    grid = load_map(str(grid_path))

    rng = np.random.default_rng(args.seed)
    tag = f"_{args.out_tag}" if args.out_tag else ""

    # Swap the module-level SEARCH_SPACE to the chosen preset so sample_config
    # and the printed-search summary both pick it up.
    global SEARCH_SPACE
    SEARCH_SPACE = SEARCH_SPACES[args.space]

    n_runs = args.trials * len(args.seeds)
    print(f"=== Random search — {args.policy.upper()} ===")
    print(f"  map:      {grid_path.name}  ({grid.shape[0]}×{grid.shape[1]})")
    print(f"  drones:   {args.drones}")
    print(f"  seeds:    {args.seeds}")
    print(f"  trials:   {args.trials}  (×{len(args.seeds)} seeds = {n_runs} runs)")
    print(f"  search:   {SEARCH_SPACE}")
    print(f"  weights:  {DEFAULT_WEIGHTS}")
    print()

    # First: evaluate the default config at the same operating point so the
    # progress plot can show the baseline we're trying to beat.
    cfg_default = (PFConfig() if args.policy == "pf"
                   else ConsensusConfig())
    default_dict = {k: getattr(cfg_default, k) for k in SEARCH_SPACE.keys()}
    print("  evaluating default config for baseline ...")
    default_scores = []
    for seed in args.seeds:
        m = evaluate(args.policy, default_dict,
                     grid=grid, n_drones=args.drones, seed=seed)
        default_scores.append(m["score"])
    default_score = float(np.mean(default_scores))
    print(f"  default mean score = {default_score:+.3f}")
    print()

    results: list[TrialResult] = []
    per_trial_mean: list[float] = []
    t0 = time.time()
    for ti in range(args.trials):
        cfg = sample_config(rng)
        per_seed_scores = []
        for seed in args.seeds:
            t_run = time.time()
            m = evaluate(args.policy, cfg,
                         grid=grid, n_drones=args.drones, seed=seed)
            results.append(TrialResult(
                trial=ti, seed=seed,
                **cfg, **m,
            ))
            per_seed_scores.append(m["score"])
        mean_s = float(np.mean(per_seed_scores))
        per_trial_mean.append(mean_s)
        best_so_far = float(np.maximum.accumulate(per_trial_mean)[-1])
        marker = " ✓ NEW BEST" if mean_s == best_so_far and mean_s > default_score else ""
        print(
            f"  [{ti+1:>3d}/{args.trials}]  "
            f"a={cfg['attract_gain']:>4.2f} rg={cfg['drone_repel_gain']:>5.2f} "
            f"rr={cfg['drone_repel_range']:>4.2f} | "
            f"score={mean_s:+.3f} (best so far {best_so_far:+.3f}){marker}"
        )
    dt = time.time() - t0
    print(f"\nTotal wall time: {dt:.0f} s ({dt/60:.1f} min)")

    # ---- write CSV ----
    DATA_DIR.mkdir(parents=True, exist_ok=True)
    PLOTS_DIR.mkdir(parents=True, exist_ok=True)
    csv_path = DATA_DIR / f"random_search_{args.policy}{tag}.csv"
    with csv_path.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(asdict(results[0]).keys()))
        w.writeheader()
        for r in results:
            w.writerow(asdict(r))
    print(f"Wrote: {csv_path}")

    # ---- top-K aggregated by trial (mean over seeds) ----
    by_trial: dict = {}
    for r in results:
        by_trial.setdefault(r.trial, []).append(r)
    summary: list[tuple[int, dict, float, float, float, int, float]] = []
    for ti, rs in by_trial.items():
        mean_score = float(np.mean([r.score for r in rs]))
        cov = float(np.mean([r.final_coverage for r in rs]))
        ov  = float(np.mean([r.overlap_m2 for r in rs]))
        wa  = float(np.mean([r.wasted_visits for r in rs]))
        en  = float(np.mean([r.energy_used_j for r in rs])) / 1000
        cfg = {k: getattr(rs[0], k) for k in SEARCH_SPACE.keys()}
        summary.append((ti, cfg, mean_score, cov, ov, wa, en))
    summary.sort(key=lambda x: -x[2])

    top_lines: list[str] = []
    top_lines.append(
        f"Top {args.top_k} configs for {args.policy.upper()} on "
        f"{grid_path.name} (n={args.drones}, seeds={args.seeds}, "
        f"trials={args.trials})\n"
    )
    top_lines.append(f"Default config mean score: {default_score:+.3f}\n\n")
    top_lines.append(
        f"{'rank':>4s}  {'score':>7s}  {'cov':>6s}  {'overlap':>9s}  "
        f"{'wasted':>7s}  {'energy':>9s}    config\n"
    )
    top_lines.append("-" * 105 + "\n")
    for rank, (ti, cfg, sc, cov, ov, wa, en) in enumerate(summary[: args.top_k], 1):
        cfg_repr = (
            f"a={cfg['attract_gain']:.2f} dRg={cfg['drone_repel_gain']:.2f} "
            f"dRr={cfg['drone_repel_range']:.2f} wRg={cfg['wall_repel_gain']:.2f} "
            f"wRr={cfg['wall_repel_range']:.2f} yA={cfg['yaw_align_gain']:.2f} "
            f"vTh={cfg['velocity_align_threshold']:.3f}"
        )
        top_lines.append(
            f"{rank:>4d}  {sc:>+7.3f}  {cov:>5.1%}  {ov:>7.0f}m²  "
            f"{wa:>7.0f}  {en:>6.1f}kJ    {cfg_repr}\n"
        )
    top_path = DATA_DIR / f"random_search_{args.policy}{tag}_top.txt"
    top_path.write_text("".join(top_lines))
    print()
    print("".join(top_lines), end="")
    print(f"Wrote: {top_path}")

    # ---- progress plot ----
    progress_path = PLOTS_DIR / f"random_search_{args.policy}{tag}_progress.png"
    plot_progress(per_trial_mean, progress_path, args.policy, default_score)
    print(f"Wrote: {progress_path}")


if __name__ == "__main__":
    main()
