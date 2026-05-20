"""
Full evaluation over (map × n_drones × policy × seed) for Track 3 control/learning controllers.

Used to study scalability: how does each controller behave as the swarm
grows from 2 → 20 drones, on maps from "fully open" → "almost closed"?
Each MARL checkpoint must match the n_drones it was trained on (PPO's
network is fixed-size); the eval picks the right checkpoint for each
drone count via the `--marl-base` template.

Outputs (new runs — archived results stay without the _new suffix):
    outputs/control_based/eval_track3_results_new.csv
    outputs/control_based/eval/png/eval_track3_{coverage_vs_drones,...}_new.png
    outputs/control_based/eval/svg/eval_track3_{...}_new.svg

Usage:
    python tools/eval/eval_track3.py \\
        --map-files outputs/maps/open_33.npy outputs/maps/partial_33.npy outputs/maps/closed_33.npy \\
        --drones-list 2 5 10 20 \\
        --seeds-per-config 3
"""

from __future__ import annotations

import argparse
import csv
import sys
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Callable, Optional

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from constants import MARL_DIR, CONTROL_BASED_DIR
from controllers import (
    ConsensusController,
    PotentialFieldsController,
    PSOController,
    GAController,
    ACOController,
    SAController,
    GWOController,
)
from controllers.consensus import ConsensusConfig
from controllers.potential_fields import PFConfig
from controllers.pso import PSOConfig
from controllers.ga import GAConfig
from controllers.aco import ACOConfig
from controllers.sa import SAConfig
from controllers.gwo import GWOConfig
from environment import CoverageEnv, DroneConfig, SimConfig
from maze import load_map


POLICY_COLORS = {
    "Random":    "#888888",
    "PF":        "#1f77b4",
    "Consensus": "#2ca02c",
    "PSO":       "#ff7f0e",
    "GA":        "#8c564b",
    "ACO":       "#e377c2",
    "SA":        "#7f7f7f",
    "GWO":       "#bcbd22",
    "MARL":      "#d62728",
    "MARL+S":    "#9467bd",
}
POLICY_ORDER = ["Random", "PF", "Consensus", "PSO", "GA", "ACO", "SA", "GWO", "MARL", "MARL+S"]


@dataclass
class RunResult:
    policy: str
    map_label: str           # filename stem, e.g. "open_33"
    n_drones: int
    seed: int
    final_coverage: float
    time_to_50: float
    time_to_80: float
    time_to_100: float
    total_energy_j: float
    overlap_m2: float
    wasted_visits: int
    time_to_terminal: float
    reason: str
    coverage_curve: list[float] = field(default_factory=list)

    def headline(self) -> dict:
        d = asdict(self)
        d.pop("coverage_curve")
        return d


def random_policy(env: CoverageEnv, rng: np.random.Generator) -> np.ndarray:
    """Gaussian-acceleration baseline; drone 0 is NOT pinned in sweep mode
    (we want all drones active for fair scaling comparison)."""
    return rng.normal(0.0, 1.5, size=(env.n_drones, 3))


def run_one(
    policy_name: str,
    policy_fn: Optional[Callable],
    *,
    grid: np.ndarray,
    map_label: str,
    n_drones: int,
    seed: int,
) -> RunResult:
    env = CoverageEnv(
        grid=grid,
        n_drones=n_drones,
        sim=SimConfig(),
        drone=DroneConfig(),
    )
    env.reset(seed=seed)
    initial_energy = sum(d.battery_j for d in env.drones)

    coverage_curve: list[float] = [env.coverage_fraction()]
    time_to_50 = time_to_80 = time_to_100 = float("inf")

    if policy_name == "Random":
        rng = np.random.default_rng(seed)
        step_fn = lambda: env.step(random_policy(env, rng))
    else:
        step_fn = lambda: env.step(policy_fn(env))

    while not env.is_terminal():
        step_fn()
        cov = env.coverage_fraction()
        coverage_curve.append(cov)
        if cov >= 0.50 and time_to_50 == float("inf"):
            time_to_50 = env.time_seconds
        if cov >= 0.80 and time_to_80 == float("inf"):
            time_to_80 = env.time_seconds
        if cov >= 1.00 and time_to_100 == float("inf"):
            time_to_100 = env.time_seconds

    total_used = initial_energy - sum(d.battery_j for d in env.drones)
    if env.is_done():
        reason = "COMPLETED"
    elif env.all_depleted():
        reason = "ALL_DEPLETED"
    else:
        reason = "TRUNCATED"

    return RunResult(
        policy=policy_name,
        map_label=map_label,
        n_drones=n_drones,
        seed=seed,
        final_coverage=env.coverage_fraction(),
        time_to_50=time_to_50,
        time_to_80=time_to_80,
        time_to_100=time_to_100,
        total_energy_j=float(total_used),
        overlap_m2=env.overlap_cells_m2(),
        wasted_visits=env.wasted_visits_total(),
        time_to_terminal=env.time_seconds,
        reason=reason,
        coverage_curve=coverage_curve,
    )


def make_stateful_factory(controller_cls, cfg, seed):
    """Create a fresh stateful controller instance (GA/ACO/SA/GWO)."""
    return controller_cls(cfg=cfg, hover_drone_idx=None, seed=seed)


def make_marl(n_drones: int, marl_base: str) -> Optional[Callable]:
    """Load the MARL checkpoint for this drone count, or return None if
    missing (sweep will skip MARL for that n_drones)."""
    try:
        from controllers import MARLController
    except ImportError:
        print(f"  [warn] torch/SB3 unavailable — skipping MARL")
        return None
    if MARLController is None:
        return None

    path_str = marl_base.format(n=n_drones)
    if not Path(path_str).exists():
        print(f"  [warn] MARL checkpoint missing for n={n_drones}: {path_str}")
        return None
    return MARLController(checkpoint=path_str, deterministic=False,
                          hover_drone_idx=None)


def make_marl_shaped(n_drones: int, base: str) -> Optional[Callable]:
    """Same as make_marl but for the reward-shaped checkpoint set. Kept as a
    sibling so the [warn] line clearly attributes the missing path."""
    try:
        from controllers import MARLController
    except ImportError:
        return None
    if MARLController is None:
        return None
    path_str = base.format(n=n_drones)
    if not Path(path_str).exists():
        print(f"  [warn] MARL+S checkpoint missing for n={n_drones}: {path_str}")
        return None
    return MARLController(checkpoint=path_str, deterministic=False,
                          hover_drone_idx=None)


# ---------------------------------------------------------------------------
# plotting
# ---------------------------------------------------------------------------

def _facet_by_map(results: list[RunResult], drones_list: list[int],
                  metric_fn, ylabel: str, title: str, out_path: Path,
                  ylim_bottom: float = 0,
                  ylim_top: Optional[float] = None) -> None:
    """
    One panel per map. X = n_drones, Y = mean over seeds of `metric_fn(r)`,
    one line per policy with std error bars.
    """
    map_labels = sorted({r.map_label for r in results})
    fig, axes = plt.subplots(
        1, len(map_labels), figsize=(5 * len(map_labels), 4.5), sharey=True,
    )
    if len(map_labels) == 1:
        axes = [axes]

    for ax, mlabel in zip(axes, map_labels):
        for policy in POLICY_ORDER:
            means = []
            stds = []
            xs = []
            for n in drones_list:
                vals = [
                    metric_fn(r) for r in results
                    if r.map_label == mlabel
                    and r.policy == policy
                    and r.n_drones == n
                ]
                if not vals:
                    continue
                xs.append(n)
                means.append(np.mean(vals))
                stds.append(np.std(vals))
            if xs:
                ax.errorbar(
                    xs, means, yerr=stds,
                    label=policy,
                    color=POLICY_COLORS.get(policy, "#666"),
                    marker="o", linewidth=2, capsize=3,
                )
        ax.set_xlabel("Number of drones")
        ax.set_xticks(drones_list)
        ax.set_title(mlabel)
        ax.grid(True, alpha=0.3)
        if ylim_bottom is not None or ylim_top is not None:
            ax.set_ylim(bottom=ylim_bottom, top=ylim_top)
    axes[0].set_ylabel(ylabel)
    axes[-1].legend(loc="best", fontsize=9)
    fig.suptitle(title, fontsize=12, fontweight="bold")
    fig.tight_layout()
    fig.savefig(out_path, dpi=130)
    plt.close(fig)


def plot_coverage_vs_drones(results, drones_list, out_path):
    _facet_by_map(
        results, drones_list,
        lambda r: r.final_coverage * 100,
        ylabel="Final coverage (%)",
        title="Final coverage vs swarm size  ↑ better",
        out_path=out_path,
        ylim_bottom=0, ylim_top=105,
    )


def plot_time_to_80_vs_drones(results, drones_list, out_path):
    def metric(r):
        return r.time_to_80 if r.time_to_80 != float("inf") else r.time_to_terminal
    _facet_by_map(
        results, drones_list, metric,
        ylabel="Sim seconds",
        title="Time to 80% coverage vs swarm size  ↓ better\n"
              "(capped at run-end if 80% never reached)",
        out_path=out_path,
        ylim_bottom=0,
    )


def plot_efficiency_vs_drones(results, drones_list, out_path):
    def metric(r):
        cov_pct = r.final_coverage * 100
        return r.wasted_visits / max(1.0, cov_pct)
    _facet_by_map(
        results, drones_list, metric,
        ylabel="Wasted entries / % coverage",
        title="Re-coverage waste vs swarm size  ↓ better\n"
              "(redundant entries normalized by % covered)",
        out_path=out_path,
        ylim_bottom=0,
    )


def plot_curves_grid(results, drones_list, out_path):
    """rows=maps, cols=drone counts; each cell = mean coverage curves of all 4 policies."""
    map_labels = sorted({r.map_label for r in results})
    n_rows = len(map_labels)
    n_cols = len(drones_list)
    fig, axes = plt.subplots(
        n_rows, n_cols,
        figsize=(3.4 * n_cols, 2.8 * n_rows),
        sharey=True, sharex=True,
    )
    if n_rows == 1:
        axes = np.array([axes])
    if n_cols == 1:
        axes = axes.reshape(-1, 1)

    for r_idx, mlabel in enumerate(map_labels):
        for c_idx, n in enumerate(drones_list):
            ax = axes[r_idx, c_idx]
            for policy in POLICY_ORDER:
                curves = [
                    r.coverage_curve for r in results
                    if r.map_label == mlabel
                    and r.policy == policy
                    and r.n_drones == n
                ]
                if not curves:
                    continue
                max_len = max(len(c) for c in curves)
                padded = np.array([c + [c[-1]] * (max_len - len(c)) for c in curves])
                mean = padded.mean(axis=0) * 100
                t = np.arange(max_len) * 0.1
                ax.plot(t, mean, label=policy,
                        color=POLICY_COLORS.get(policy, "#666"),
                        linewidth=1.5)
            ax.grid(True, alpha=0.3)
            ax.set_ylim(0, 105)
            if r_idx == 0:
                ax.set_title(f"n={n}")
            if c_idx == 0:
                ax.set_ylabel(f"{mlabel}\nCoverage (%)", fontsize=9)
            if r_idx == n_rows - 1:
                ax.set_xlabel("Sim time (s)")
    axes[0, 0].legend(loc="lower right", fontsize=8)
    fig.suptitle("Coverage curves — rows = maps, cols = swarm size",
                 fontsize=12, fontweight="bold")
    fig.tight_layout()
    fig.savefig(out_path, dpi=130)
    plt.close(fig)


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------

def parse_args():
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--map-files", type=str, nargs="+", required=True,
                   help="paths to .npy maps to evaluate on")
    p.add_argument("--drones-list", type=int, nargs="+",
                   default=[2, 5, 10, 20],
                   help="drone counts to sweep (default 2 5 10 20)")
    p.add_argument("--seeds-per-config", type=int, default=3,
                   help="seeds per (map, n_drones) cell (default 3)")
    p.add_argument("--marl-base", type=str,
                   default=str(MARL_DIR / "marl_ppo_n{n}.zip"),
                   help="checkpoint path template; '{n}' is replaced with the "
                        "drone count. Default: outputs/marl/marl_ppo_n{n}.zip")
    p.add_argument("--marl-shaped-base", type=str, default=None,
                   help="optional second MARL checkpoint set with reward "
                        "shaping; '{n}' is replaced with the drone count. "
                        "When set, runs as a separate policy 'MARL+S' "
                        "alongside the default 'MARL' for direct comparison.")
    p.add_argument("--skip-marl", action="store_true",
                   help="skip MARL entirely (don't try to load any checkpoint)")
    return p.parse_args()


def main():
    args = parse_args()

    # Load maps
    maps: list[tuple[str, np.ndarray]] = []
    for path in args.map_files:
        p = Path(path)
        grid = load_map(str(p))
        label = p.stem
        h, w = grid.shape
        n_free = int((grid == 0).sum())
        # Openness reported against the interior only — the mandatory boundary
        # ring (always walls) shouldn't make a fully-open map look <100 %.
        interior_total = max(1, (h - 2) * (w - 2))
        n_free_interior = int((grid[1:-1, 1:-1] == 0).sum())
        print(f"  loaded {label}  ({h}×{w} cells, {n_free} free, "
              f"{100 * n_free_interior / interior_total:.1f}% interior open)")
        maps.append((label, grid))

    seeds = list(range(args.seeds_per_config))

    # Stateless controllers — reusable across runs.
    pf = PotentialFieldsController(cfg=PFConfig.dense(), hover_drone_idx=None)
    consensus = ConsensusController(cfg=ConsensusConfig.dense(), hover_drone_idx=None)
    pso = PSOController(cfg=PSOConfig(), hover_drone_idx=None)

    # Stateful controllers (GA/ACO/SA/GWO) need fresh instances per run.
    STATEFUL_POLICIES = {
        "GA":  (GAController, GAConfig()),
        "ACO": (ACOController, ACOConfig()),
        "SA":  (SAController, SAConfig()),
        "GWO": (GWOController, GWOConfig()),
    }

    n_policies = 3 + 1 + len(STATEFUL_POLICIES)  # Random + PF + Consensus + PSO + stateful
    if not args.skip_marl:
        n_policies += 1
        if args.marl_shaped_base is not None:
            n_policies += 1
    total_runs = (
        len(maps) * len(args.drones_list)
        * n_policies
        * len(seeds)
    )
    print(f"\n=== Sweep — {total_runs} runs total ===")
    print(f"  maps: {[m[0] for m in maps]}")
    print(f"  drone counts: {args.drones_list}")
    print(f"  seeds per config: {args.seeds_per_config}")
    print(f"  policies: Random, PF, Consensus, PSO, GA, ACO, SA, GWO"
          + (", MARL" if not args.skip_marl else ""))
    print()

    results: list[RunResult] = []
    t0 = time.time()
    for mlabel, grid in maps:
        for n_drones in args.drones_list:
            # MARL must be loaded once per drone count.
            marl = None
            marl_shaped = None
            if not args.skip_marl:
                marl = make_marl(n_drones, args.marl_base)
                if args.marl_shaped_base is not None:
                    marl_shaped = make_marl_shaped(n_drones, args.marl_shaped_base)

            for seed in seeds:
                # Stateless policies
                policies_this_run: list[tuple[str, Callable]] = [
                    ("Random", None),
                    ("PF", pf),
                    ("Consensus", consensus),
                    ("PSO", pso),
                ]
                # Fresh stateful controllers per (n_drones, seed)
                for name in STATEFUL_POLICIES:
                    cls, cfg = STATEFUL_POLICIES[name]
                    policies_this_run.append(
                        (name, make_stateful_factory(cls, cfg, seed))
                    )
                if marl is not None:
                    policies_this_run.append(("MARL", marl))
                if marl_shaped is not None:
                    policies_this_run.append(("MARL+S", marl_shaped))

                for policy_name, policy_fn in policies_this_run:
                    t_run0 = time.time()
                    r = run_one(
                        policy_name, policy_fn,
                        grid=grid, map_label=mlabel,
                        n_drones=n_drones, seed=seed,
                    )
                    results.append(r)
                    t100 = (
                        f"{r.time_to_100:.0f}s"
                        if r.time_to_100 != float("inf") else "—"
                    )
                    print(f"  {mlabel:<14s} n={n_drones:>2d}  {policy_name:<9s}  "
                          f"seed={seed}  cov={r.final_coverage:6.1%}  "
                          f"t100={t100:>6s}  reason={r.reason:<13s}  "
                          f"({time.time() - t_run0:.1f}s)")
    print(f"\nTotal wall time: {time.time() - t0:.1f} s\n")

    CONTROL_BASED_DIR.mkdir(parents=True, exist_ok=True)
    csv_path = CONTROL_BASED_DIR / "eval_track3_results_new.csv"
    with csv_path.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(results[0].headline().keys()))
        w.writeheader()
        for r in results:
            w.writerow(r.headline())
    print(f"Wrote: {csv_path}")

    eval_png = CONTROL_BASED_DIR / "eval" / "png"
    eval_svg = CONTROL_BASED_DIR / "eval" / "svg"
    eval_png.mkdir(parents=True, exist_ok=True)
    eval_svg.mkdir(parents=True, exist_ok=True)
    plot_specs = [
        ("eval_track3_coverage_vs_drones_new",  plot_coverage_vs_drones),
        ("eval_track3_time_to_80_vs_drones_new", plot_time_to_80_vs_drones),
        ("eval_track3_efficiency_vs_drones_new", plot_efficiency_vs_drones),
        ("eval_track3_curves_grid_new",          plot_curves_grid),
    ]
    for name, fn in plot_specs:
        fn(results, args.drones_list, eval_png / f"{name}.png")
        fn(results, args.drones_list, eval_svg / f"{name}.svg")
    print(f"Wrote 4 eval plots: PNG → {eval_png}/, SVG → {eval_svg}/")

    # Summary
    print("\n=== Mean final coverage by (map × policy × n_drones) ===")
    for mlabel, _ in maps:
        print(f"\n  {mlabel}:")
        header = "    n_drones  " + "  ".join(f"{p:>9s}" for p in POLICY_ORDER)
        print(header)
        for n in args.drones_list:
            row = f"    {n:>8d}  "
            for p in POLICY_ORDER:
                vals = [r.final_coverage for r in results
                        if r.map_label == mlabel
                        and r.policy == p
                        and r.n_drones == n]
                if vals:
                    row += f"  {np.mean(vals) * 100:>7.1f}%"
                else:
                    row += f"  {'—':>8s}"
            print(row)


if __name__ == "__main__":
    main()
