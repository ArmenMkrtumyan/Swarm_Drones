"""
Multi-axis sweep for Track 2 metaheuristic controllers.

Companion to `tools/sweep.py` (Track 3). Runs Random plus the five
Track 2 controllers — PSO, GA, ACO, SA, GWO — at their tuned-default
configs over the same (map × n_drones × seed) grid, dumps a CSV, and
saves comparison plots.

Each controller's defaults come from its corresponding `tools/grid_search_{algo}.py`
top-1 result. Re-run with non-default configs by importing the controllers
in your own script if needed.

Outputs (to `outputs/csv_txt/` and `outputs/png/`):
    sweep_track2_results.csv               raw per-run records
    sweep_track2_coverage_vs_drones.png    final coverage line plot
    sweep_track2_time_to_80_vs_drones.png  time-to-80% line plot
    sweep_track2_efficiency_vs_drones.png  wasted-visits-per-%-coverage
    sweep_track2_curves_grid.png           rows=maps, cols=drones grid

Usage:
    python tools/sweep_track2.py \\
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

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from constants import MAPS_DIR, OUTPUTS_DIR
from controllers import (
    ACOController, GAController, GWOController, PSOController, SAController,
)
from environment import CoverageEnv, DroneConfig, SimConfig
from maze import load_map


POLICY_COLORS = {
    "Random":    "#888888",
    "PSO":       "#1f77b4",
    "GA":        "#2ca02c",
    "ACO":       "#d62728",
    "SA":        "#9467bd",
    "GWO":       "#ff7f0e",
}
POLICY_ORDER = ["Random", "PSO", "GA", "ACO", "SA", "GWO"]


@dataclass
class RunResult:
    policy: str
    map_label: str
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
    """Gaussian-acceleration baseline; drone 0 is NOT pinned (full eval)."""
    return rng.normal(0.0, 1.5, size=(env.n_drones, 3))


def make_policy(name: str, seed: int) -> Optional[Callable]:
    """Instantiate a Track 2 controller fresh per (n_drones, seed) cell.

    Stateful controllers (PSO/GA/ACO/SA/GWO) all carry per-episode memory,
    so they MUST be reinstantiated for each run rather than reused across
    cells.
    """
    if name == "PSO":
        return PSOController(hover_drone_idx=None, seed=seed)
    if name == "GA":
        return GAController(hover_drone_idx=None, seed=seed)
    if name == "ACO":
        return ACOController(hover_drone_idx=None, seed=seed)
    if name == "SA":
        return SAController(hover_drone_idx=None, seed=seed)
    if name == "GWO":
        return GWOController(hover_drone_idx=None, seed=seed)
    return None  # Random — handled inline


def run_one(
    policy_name: str,
    *,
    grid: np.ndarray, map_label: str, n_drones: int, seed: int,
) -> RunResult:
    env = CoverageEnv(grid=grid, n_drones=n_drones,
                      sim=SimConfig(), drone=DroneConfig())
    env.reset(seed=seed)
    initial_energy = sum(d.battery_j for d in env.drones)

    coverage_curve: list[float] = [env.coverage_fraction()]
    time_to_50 = time_to_80 = time_to_100 = float("inf")

    if policy_name == "Random":
        rng = np.random.default_rng(seed)
        step_fn = lambda: env.step(random_policy(env, rng))
    else:
        controller = make_policy(policy_name, seed)
        step_fn = lambda: env.step(controller(env))

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
        policy=policy_name, map_label=map_label,
        n_drones=n_drones, seed=seed,
        final_coverage=env.coverage_fraction(),
        time_to_50=time_to_50, time_to_80=time_to_80, time_to_100=time_to_100,
        total_energy_j=float(total_used),
        overlap_m2=env.overlap_cells_m2(),
        wasted_visits=env.wasted_visits_total(),
        time_to_terminal=env.time_seconds,
        reason=reason,
        coverage_curve=coverage_curve,
    )


# ---------------------------------------------------------------------------
# plotting (mirror tools/sweep.py)
# ---------------------------------------------------------------------------

def _facet_by_map(results, drones_list, metric_fn, ylabel, title, out_path,
                  ylim_bottom=0, ylim_top=None):
    map_labels = sorted({r.map_label for r in results})
    fig, axes = plt.subplots(
        1, len(map_labels), figsize=(5 * len(map_labels), 4.5), sharey=True,
    )
    if len(map_labels) == 1:
        axes = [axes]

    for ax, mlabel in zip(axes, map_labels):
        for policy in POLICY_ORDER:
            means, stds, xs = [], [], []
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
        title="Track 2 — Final coverage vs swarm size  ↑ better",
        out_path=out_path, ylim_bottom=0, ylim_top=105,
    )


def plot_time_to_80_vs_drones(results, drones_list, out_path):
    def metric(r):
        return r.time_to_80 if r.time_to_80 != float("inf") else r.time_to_terminal
    _facet_by_map(
        results, drones_list, metric,
        ylabel="Sim seconds",
        title="Track 2 — Time to 80% coverage  ↓ better\n"
              "(capped at run-end if 80% never reached)",
        out_path=out_path, ylim_bottom=0,
    )


def plot_efficiency_vs_drones(results, drones_list, out_path):
    def metric(r):
        cov_pct = r.final_coverage * 100
        return r.wasted_visits / max(1.0, cov_pct)
    _facet_by_map(
        results, drones_list, metric,
        ylabel="Wasted entries / % coverage",
        title="Track 2 — Re-coverage waste vs swarm size  ↓ better",
        out_path=out_path, ylim_bottom=0,
    )


def plot_curves_grid(results, drones_list, out_path):
    map_labels = sorted({r.map_label for r in results})
    n_rows, n_cols = len(map_labels), len(drones_list)
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
    fig.suptitle("Track 2 — Coverage curves (rows=maps, cols=swarm size)",
                 fontsize=12, fontweight="bold")
    fig.tight_layout()
    fig.savefig(out_path, dpi=130)
    plt.close(fig)


def parse_args():
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--map-files", type=str, nargs="+", default=[
        str(MAPS_DIR / "open_33.npy"),
        str(MAPS_DIR / "partial_33.npy"),
        str(MAPS_DIR / "closed_33.npy"),
    ])
    p.add_argument("--drones-list", type=int, nargs="+", default=[2, 5, 10, 20])
    p.add_argument("--seeds-per-config", type=int, default=3)
    return p.parse_args()


def main():
    args = parse_args()

    maps = []
    for path in args.map_files:
        p = Path(path)
        grid = load_map(str(p))
        h, w = grid.shape
        n_free = int((grid == 0).sum())
        n_free_int = int((grid[1:-1, 1:-1] == 0).sum())
        interior = max(1, (h - 2) * (w - 2))
        print(f"  loaded {p.stem}  ({h}×{w}, {n_free} free, "
              f"{100 * n_free_int / interior:.1f}% interior open)")
        maps.append((p.stem, grid))

    seeds = list(range(args.seeds_per_config))
    policies = POLICY_ORDER
    total = len(maps) * len(args.drones_list) * len(policies) * len(seeds)
    print(f"\n=== Track 2 sweep — {total} runs total ===")
    print(f"  policies:     {policies}")
    print(f"  drone counts: {args.drones_list}")
    print(f"  seeds:        {seeds}")
    print()

    results: list[RunResult] = []
    t0 = time.time()
    for mlabel, mgrid in maps:
        for n in args.drones_list:
            for policy in policies:
                for seed in seeds:
                    t_run0 = time.time()
                    r = run_one(
                        policy, grid=mgrid, map_label=mlabel,
                        n_drones=n, seed=seed,
                    )
                    results.append(r)
                    t100 = (
                        f"{r.time_to_100:.0f}s"
                        if r.time_to_100 != float("inf") else "—"
                    )
                    print(f"  {mlabel:<12s} n={n:>2d}  {policy:<8s}  "
                          f"seed={seed}  cov={r.final_coverage:6.1%}  "
                          f"t100={t100:>6s}  reason={r.reason:<13s}  "
                          f"({time.time() - t_run0:.1f}s)")
    print(f"\nTotal wall time: {time.time() - t0:.1f} s\n")

    # ---- CSV (stays at outputs/ root) ----
    OUTPUTS_DIR.mkdir(parents=True, exist_ok=True)
    csv_path = OUTPUTS_DIR / "sweep_track2_results.csv"
    with csv_path.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(results[0].headline().keys()))
        w.writeheader()
        for r in results:
            w.writerow(r.headline())
    print(f"Wrote: {csv_path}")

    # ---- plots: PNG → outputs/sweep/png/, SVG → outputs/sweep/svg/ ----
    sweep_png = OUTPUTS_DIR / "sweep" / "png"
    sweep_svg = OUTPUTS_DIR / "sweep" / "svg"
    sweep_png.mkdir(parents=True, exist_ok=True)
    sweep_svg.mkdir(parents=True, exist_ok=True)
    plot_specs = [
        ("sweep_track2_coverage_vs_drones",  plot_coverage_vs_drones),
        ("sweep_track2_time_to_80_vs_drones", plot_time_to_80_vs_drones),
        ("sweep_track2_efficiency_vs_drones", plot_efficiency_vs_drones),
        ("sweep_track2_curves_grid",          plot_curves_grid),
    ]
    for name, fn in plot_specs:
        fn(results, args.drones_list, sweep_png / f"{name}.png")
        fn(results, args.drones_list, sweep_svg / f"{name}.svg")
    print(f"Wrote 4 sweep plots: PNG → {sweep_png}/, SVG → {sweep_svg}/")

    # ---- summary table ----
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
