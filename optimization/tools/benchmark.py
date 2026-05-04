"""
Benchmark harness for Track-3 controllers.

Runs every policy on the same map+seed pairs, records per-step coverage
curves and headline end-of-run metrics, dumps a CSV, and saves
comparison plots. Used to produce the figures in `outputs/images/` for
the report.

Policies compared:
    - Random (Gaussian acceleration baseline)
    - PotentialFieldsController
    - ConsensusController
    - MARLController (loaded from outputs/marl_ppo.zip)

Metrics per run:
    - final coverage fraction
    - time to 50% / 80% / 100% (sim seconds; ∞ if not reached)
    - total energy used (J, summed over drones)
    - overlap_cells_m2 (territory touched by ≥ 2 drones)
    - wasted_visits_total (sum of redundant entries)
    - time_to_terminal (sim seconds when env.is_terminal())
    - reason: COMPLETED / ALL_DEPLETED

Usage:
    python tools/benchmark.py                             # default 5 seeds × 2 maps
    python tools/benchmark.py --seeds 1 2 3 4 5 6 7 8     # custom seed list
    python tools/benchmark.py --grid 21 --drones 4        # bigger world
"""

from __future__ import annotations

import argparse
import csv
import sys
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Callable, Optional

# Make project root importable.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from constants import IMAGES_DIR, OUTPUTS_DIR
from controllers import (
    ConsensusController,
    MARLController,
    PotentialFieldsController,
)
from environment import CoverageEnv, DroneConfig, SimConfig
from maze import random_obstacles, recursive_backtracker


@dataclass
class RunResult:
    policy: str
    map_kind: str
    seed: int
    final_coverage: float
    time_to_50: float           # sim seconds; np.inf if unreached
    time_to_80: float
    time_to_100: float
    total_energy_j: float
    overlap_m2: float
    wasted_visits: int
    time_to_terminal: float
    reason: str                 # "COMPLETED" | "ALL_DEPLETED"
    coverage_curve: list[float] = field(default_factory=list)  # per-step

    def headline_dict(self) -> dict:
        d = asdict(self)
        d.pop("coverage_curve")
        return d


def random_policy(env: CoverageEnv, rng: np.random.Generator) -> np.ndarray:
    """
    Same Gaussian-acceleration baseline as `tools/demo.py:random_policy_with_hover`,
    minus the demo's brake-and-realign machinery — pure random forces.
    Returns shape (n_drones, 3); drone 0 is pinned to zero so the hover
    sanity check semantics still apply.
    """
    actions = rng.normal(0.0, 1.5, size=(env.n_drones, 3))
    actions[0] = 0.0
    return actions


def make_grid(map_kind: str, size: int, seed: int) -> np.ndarray:
    if map_kind == "maze":
        return recursive_backtracker(size, seed=seed)
    return random_obstacles(size, density=0.20, seed=seed)


def run_one(
    policy_name: str,
    policy_fn: Optional[Callable],
    *,
    map_kind: str,
    seed: int,
    grid_size: int,
    n_drones: int,
) -> RunResult:
    """
    Run a single policy on a single map+seed. `policy_fn` is None for the
    random baseline (the random_policy function is wrapped here to take
    a seeded rng).
    """
    grid = make_grid(map_kind, grid_size, seed)
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

    final_energy = sum(d.battery_j for d in env.drones)
    total_used = initial_energy - final_energy

    if env.is_done():
        reason = "COMPLETED"
    elif env.all_depleted():
        reason = "ALL_DEPLETED"
    else:
        reason = "TRUNCATED"

    return RunResult(
        policy=policy_name,
        map_kind=map_kind,
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


# ---------------------------------------------------------------------------
# plotting
# ---------------------------------------------------------------------------

POLICY_COLORS = {
    "Random":    "#888888",
    "PF":        "#1f77b4",
    "Consensus": "#2ca02c",
    "MARL":      "#d62728",
}


def plot_coverage_curves(results: list[RunResult], out_path: Path,
                         title_suffix: str = "") -> None:
    """Mean ± std coverage curve per policy, averaged over seeds and map types."""
    fig, ax = plt.subplots(figsize=(9, 5))

    by_policy: dict[str, list[list[float]]] = {}
    for r in results:
        by_policy.setdefault(r.policy, []).append(r.coverage_curve)

    for policy, curves in by_policy.items():
        # Pad shorter curves with their final value so all align.
        max_len = max(len(c) for c in curves)
        padded = np.array([c + [c[-1]] * (max_len - len(c)) for c in curves])
        mean = padded.mean(axis=0)
        std = padded.std(axis=0)
        t = np.arange(max_len) * 0.1   # step_seconds
        color = POLICY_COLORS.get(policy, None)
        ax.plot(t, mean * 100, label=policy, color=color, linewidth=2)
        ax.fill_between(t, (mean - std) * 100, (mean + std) * 100,
                        color=color, alpha=0.15)

    ax.set_xlabel("Sim time (seconds)")
    ax.set_ylabel("Coverage (%)")
    ax.set_ylim(0, 105)
    ax.set_title(f"Coverage vs time — mean ± std over runs{title_suffix}")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="lower right")
    fig.tight_layout()
    fig.savefig(out_path, dpi=130)
    plt.close(fig)


def plot_metric_bars(results: list[RunResult], out_path: Path) -> None:
    """4-panel summary: final coverage, time-to-80%, energy, wasted visits."""
    by_policy: dict[str, list[RunResult]] = {}
    for r in results:
        by_policy.setdefault(r.policy, []).append(r)

    policies = list(by_policy.keys())
    colors = [POLICY_COLORS.get(p, "#666") for p in policies]

    fig, axes = plt.subplots(2, 2, figsize=(11, 7))

    # Panel 1: final coverage (higher = better)
    means = [np.mean([r.final_coverage for r in by_policy[p]]) * 100 for p in policies]
    stds = [np.std([r.final_coverage for r in by_policy[p]]) * 100 for p in policies]
    axes[0, 0].bar(policies, means, yerr=stds, color=colors, capsize=4)
    axes[0, 0].set_ylabel("Final coverage (%)")
    axes[0, 0].set_title("Final coverage  ↑ better")
    axes[0, 0].set_ylim(0, 105)
    axes[0, 0].grid(True, alpha=0.3, axis="y")

    # Panel 2: time-to-80 (lower = better; replace inf with run-end time)
    def cap_inf(rr: list[RunResult], attr: str) -> list[float]:
        return [getattr(r, attr) if getattr(r, attr) != float("inf")
                else r.time_to_terminal for r in rr]

    t80_means = [np.mean(cap_inf(by_policy[p], "time_to_80")) for p in policies]
    t80_stds = [np.std(cap_inf(by_policy[p], "time_to_80")) for p in policies]
    axes[0, 1].bar(policies, t80_means, yerr=t80_stds, color=colors, capsize=4)
    axes[0, 1].set_ylabel("Sim seconds")
    axes[0, 1].set_title("Time to 80% coverage  ↓ better\n(capped at run-end if unreached)")
    axes[0, 1].set_ylim(bottom=0)
    axes[0, 1].grid(True, alpha=0.3, axis="y")

    # Panel 3: total energy used (lower = better, but this scales with run length)
    e_means = [np.mean([r.total_energy_j for r in by_policy[p]]) / 1000 for p in policies]
    e_stds = [np.std([r.total_energy_j for r in by_policy[p]]) / 1000 for p in policies]
    axes[1, 0].bar(policies, e_means, yerr=e_stds, color=colors, capsize=4)
    axes[1, 0].set_ylabel("Total energy used (kJ)")
    axes[1, 0].set_title("Total energy  ↓ better\n(swarm-summed; tied to run length)")
    axes[1, 0].set_ylim(bottom=0)
    axes[1, 0].grid(True, alpha=0.3, axis="y")

    # Panel 4: wasted visits per cell covered (lower = more efficient)
    eff = []
    eff_std = []
    for p in policies:
        ratios = []
        for r in by_policy[p]:
            covered_cells = r.final_coverage  # fraction; size cancels
            if covered_cells > 0:
                ratios.append(r.wasted_visits / max(1.0, covered_cells * 100))
            else:
                ratios.append(r.wasted_visits)
        eff.append(np.mean(ratios))
        eff_std.append(np.std(ratios))
    axes[1, 1].bar(policies, eff, yerr=eff_std, color=colors, capsize=4)
    axes[1, 1].set_ylabel("Wasted visits per % coverage")
    axes[1, 1].set_title("Re-coverage waste  ↓ better\n(redundant entries normalized by % covered)")
    axes[1, 1].set_ylim(bottom=0)
    axes[1, 1].grid(True, alpha=0.3, axis="y")

    fig.suptitle("Controller comparison summary  (mean ± std over all runs)",
                 fontsize=12, fontweight="bold")
    fig.tight_layout()
    fig.savefig(out_path, dpi=130)
    plt.close(fig)


def plot_per_map_breakdown(results: list[RunResult], out_path: Path) -> None:
    """Final coverage broken down by map_kind."""
    map_kinds = sorted({r.map_kind for r in results})
    policies = sorted({r.policy for r in results}, key=lambda p: list(POLICY_COLORS).index(p)
                      if p in POLICY_COLORS else 99)

    fig, ax = plt.subplots(figsize=(9, 5))
    x = np.arange(len(map_kinds))
    width = 0.8 / len(policies)

    for i, p in enumerate(policies):
        means = []
        stds = []
        for mk in map_kinds:
            vals = [r.final_coverage * 100 for r in results
                    if r.policy == p and r.map_kind == mk]
            means.append(np.mean(vals) if vals else 0)
            stds.append(np.std(vals) if vals else 0)
        ax.bar(x + i * width - 0.4 + width / 2, means, width,
               yerr=stds, label=p, color=POLICY_COLORS.get(p, None),
               capsize=3)

    ax.set_xticks(x)
    ax.set_xticklabels(map_kinds)
    ax.set_ylabel("Final coverage (%)")
    ax.set_ylim(0, 105)
    ax.set_title("Final coverage by map type  ↑ better")
    ax.grid(True, alpha=0.3, axis="y")
    ax.legend(loc="lower right")
    fig.tight_layout()
    fig.savefig(out_path, dpi=130)
    plt.close(fig)


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------

def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--seeds", type=int, nargs="+", default=[7, 11, 17, 23, 29],
                   help="seeds to evaluate (default: 5 seeds)")
    p.add_argument("--maps", choices=["random", "maze", "both"], default="both",
                   help="map generators to test")
    p.add_argument("--grid", type=int, default=15)
    p.add_argument("--drones", type=int, default=4)
    p.add_argument("--marl-checkpoint", type=str,
                   default=str(OUTPUTS_DIR / "marl_ppo.zip"))
    p.add_argument("--skip-marl", action="store_true",
                   help="skip MARL (e.g., if checkpoint missing)")
    p.add_argument("--marl-stochastic", action="store_true",
                   help="use stochastic policy for MARL evaluation "
                        "(default deterministic)")
    return p.parse_args()


def main() -> None:
    args = parse_args()
    map_kinds = ["random", "maze"] if args.maps == "both" else [args.maps]

    # Build the policy list. Each entry: (display name, callable or None for Random).
    policies: list[tuple[str, Optional[Callable]]] = [
        ("Random", None),
        ("PF", PotentialFieldsController()),
        ("Consensus", ConsensusController()),
    ]
    if not args.skip_marl:
        try:
            marl = MARLController(
                checkpoint=args.marl_checkpoint,
                deterministic=not args.marl_stochastic,
            )
            policies.append(("MARL", marl))
        except FileNotFoundError as e:
            print(f"[WARN] {e}")
            print("[WARN] Skipping MARL. Run `python tools/train_marl.py` first "
                  "or pass --skip-marl.")

    print(f"=== Benchmark — {len(policies)} policies × {len(map_kinds)} map kinds "
          f"× {len(args.seeds)} seeds = {len(policies) * len(map_kinds) * len(args.seeds)} runs ===")
    print(f"  grid_size: {args.grid}, n_drones: {args.drones}")
    print(f"  policies:  {[p[0] for p in policies]}")
    print(f"  map_kinds: {map_kinds}")
    print(f"  seeds:     {args.seeds}")
    print()

    results: list[RunResult] = []
    t0 = time.time()
    for map_kind in map_kinds:
        for seed in args.seeds:
            for policy_name, policy_fn in policies:
                t_run0 = time.time()
                r = run_one(
                    policy_name, policy_fn,
                    map_kind=map_kind, seed=seed,
                    grid_size=args.grid, n_drones=args.drones,
                )
                t_run = time.time() - t_run0
                results.append(r)
                t_to_100 = (
                    f"{r.time_to_100:.0f}s" if r.time_to_100 != float("inf")
                    else "—"
                )
                print(f"  {map_kind:6s} seed={seed:>3d}  {policy_name:9s}  "
                      f"cov={r.final_coverage:6.1%}  "
                      f"t100={t_to_100:>6s}  "
                      f"overlap={r.overlap_m2:>5.0f}m²  "
                      f"wasted={r.wasted_visits:>4d}  "
                      f"reason={r.reason:<13s}  ({t_run:.1f}s wall)")
            print()
    print(f"Total wall time: {time.time() - t0:.1f} s\n")

    # ---- write CSV ----
    IMAGES_DIR.mkdir(parents=True, exist_ok=True)
    csv_path = IMAGES_DIR / "benchmark_results.csv"
    with csv_path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(results[0].headline_dict().keys()))
        writer.writeheader()
        for r in results:
            writer.writerow(r.headline_dict())
    print(f"Wrote: {csv_path}")

    # ---- plots ----
    out_curve = IMAGES_DIR / "benchmark_coverage_curves.png"
    out_bars = IMAGES_DIR / "benchmark_summary_bars.png"
    out_per_map = IMAGES_DIR / "benchmark_per_map.png"

    plot_coverage_curves(results, out_curve)
    plot_metric_bars(results, out_bars)
    plot_per_map_breakdown(results, out_per_map)

    # Per-map curves too — splits the story when one map type behaves differently.
    for mk in map_kinds:
        sub = [r for r in results if r.map_kind == mk]
        out = IMAGES_DIR / f"benchmark_coverage_curves_{mk}.png"
        plot_coverage_curves(sub, out, title_suffix=f"  ({mk} maps)")
        print(f"Wrote: {out}")

    print(f"Wrote: {out_curve}")
    print(f"Wrote: {out_bars}")
    print(f"Wrote: {out_per_map}")

    # ---- summary table to stdout ----
    print("\n=== Mean across all runs ===")
    by_policy: dict[str, list[RunResult]] = {}
    for r in results:
        by_policy.setdefault(r.policy, []).append(r)
    print(f"  {'policy':<10s}  {'cov':>6s}  {'t→80%':>7s}  "
          f"{'t→100%':>7s}  {'energy(kJ)':>10s}  {'overlap(m²)':>11s}  {'wasted':>6s}")
    for p, rs in by_policy.items():
        cov = np.mean([r.final_coverage for r in rs]) * 100
        t80_vals = [r.time_to_80 if r.time_to_80 != float("inf") else r.time_to_terminal
                    for r in rs]
        t100_vals = [r.time_to_100 for r in rs if r.time_to_100 != float("inf")]
        t80 = np.mean(t80_vals)
        t100 = np.mean(t100_vals) if t100_vals else float("inf")
        energy = np.mean([r.total_energy_j for r in rs]) / 1000
        overlap = np.mean([r.overlap_m2 for r in rs])
        wasted = np.mean([r.wasted_visits for r in rs])
        t100_s = f"{t100:.0f}s" if t100 != float("inf") else "—"
        print(f"  {p:<10s}  {cov:>5.1f}%  {t80:>6.0f}s  "
              f"{t100_s:>7s}  {energy:>10.1f}  {overlap:>11.0f}  {wasted:>6.0f}")


if __name__ == "__main__":
    main()
