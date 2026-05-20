"""
Bayesian-Optimization hyperparameter search for all swarm controllers.

Replaces the older `grid_search_<algo>.py` family. Uses Optuna's TPE sampler
(Tree-structured Parzen Estimator) which fits two density estimators — one for
"good" trials, one for "bad" — and proposes new configs from the good density.
Sample-efficient: ~30 trials typically beat the previous 27-config grid in
final score, AND lets us tune more knobs at once without the
3^k explosion problem.

Each trial:
    1. Build the controller's Config dataclass with sampled hyperparameters.
    2. Evaluate over a grid of (map × n_drones × seed) cells.
    3. Average the composite_score (score.py) across cells.
    4. Return the mean — Optuna maximizes.

Per-controller hyperparameter spaces are declared in `SPACES` below — one
function `space_fn(trial) -> dict` per controller. Add a new controller by
adding (1) its entry in `CONTROLLERS` (class + Config) and (2) its `space_fn`.

Yaw / stuck / blacklist / arrival_radius knobs are intentionally NOT tuned:
they're physics-bounded or behavior-shaped, not coverage-objective knobs.

Usage:
    python3 tools/tuning/bo_search.py --controller boustrophedon --n-trials 30 \\
            --output outputs/bo/boustrophedon.json

The output JSON has `best_value`, `best_params`, and the full trial history.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import asdict, fields
from pathlib import Path

# Project root importable so `from environment import ...` works.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

import numpy as np
import optuna
from optuna.samplers import TPESampler

from environment import CoverageEnv, DroneConfig, SimConfig
from maze import FREE, load_map
from tools.score import composite_score


# ---- Evaluation harness ----------------------------------------------------

# Default evaluation grid for the BO objective. Smaller than the full sweep
# grid in `tools/eval/eval_track1.py` (which uses 4 n_drones × 3 maps × 3 seeds = 36
# cells) so each trial finishes in roughly a minute — Optuna burns trials
# fast and we don't need the full sweep for ranking.
EVAL_MAPS = ["open_33", "partial_33", "closed_33"]
EVAL_N_DRONES = [5, 10]
EVAL_SEEDS = [1, 42]


def _evaluate(controller_cls, cfg, ctrl_kwargs: dict) -> float:
    """Run one config across the (map × n_drones × seed) grid; return mean
    composite_score across cells."""
    scores: list[float] = []
    for map_name in EVAL_MAPS:
        grid = load_map(f"maps/{map_name}.npy")
        for n_drones in EVAL_N_DRONES:
            for seed in EVAL_SEEDS:
                env = CoverageEnv(
                    grid=grid,
                    n_drones=n_drones,
                    sim=SimConfig(step_seconds=0.1),
                    drone=DroneConfig(
                        sensor_range=1.6, max_speed=1.8, max_accel=2.5
                    ),
                )
                env.reset(seed=seed)
                # Most controllers accept `seed=` (for their internal RNG);
                # the ones that don't (e.g. boustrophedon) need it omitted.
                kwargs = dict(ctrl_kwargs)
                try:
                    ctrl = controller_cls(cfg=cfg, **kwargs, seed=seed)
                except TypeError:
                    ctrl = controller_cls(cfg=cfg, **kwargs)
                while not env.is_terminal():
                    env.step(ctrl(env))
                free_mask = env.grid == FREE
                unique_visited = int((env.covered & free_mask).sum())
                energy_used = sum(
                    env.battery_state(i)["used_energy_j"]
                    for i in range(env.n_drones)
                )
                energy_budget = env.n_drones * env.battery_cfg.initial_energy_j
                mpc = env.sim_cfg.meters_per_cell
                free_area_m2 = float(free_mask.sum()) * mpc * mpc
                scores.append(
                    composite_score(
                        coverage_fraction=env.coverage_fraction(),
                        overlap_m2=env.overlap_cells_m2(),
                        free_area_m2=free_area_m2,
                        wasted_visits=env.wasted_visits_total(),
                        unique_cells_visited=unique_visited,
                        energy_used_j=energy_used,
                        energy_budget_j=energy_budget,
                    )
                )
    return float(np.mean(scores))


# ---- Per-controller hyperparameter spaces ----------------------------------
#
# Each `space_fn(trial) -> dict` returns the kwargs to pass into the
# controller's Config dataclass. Use trial.suggest_float / suggest_int /
# suggest_categorical. Bounds are deliberately broad but anchored around the
# previous grid-search-found defaults.

def _boustrophedon_space(trial):
    return dict(
        lane_spacing       = trial.suggest_float("lane_spacing", 1.0, 4.0),
        attract_gain       = trial.suggest_float("attract_gain", 0.5, 5.0, log=True),
        attract_damp_gain  = trial.suggest_float("attract_damp_gain", 0.05, 1.0, log=True),
        drone_repel_gain   = trial.suggest_float("drone_repel_gain", 0.5, 10.0, log=True),
        drone_repel_range  = trial.suggest_float("drone_repel_range", 1.0, 5.0),
        wall_repel_gain    = trial.suggest_float("wall_repel_gain", 0.5, 5.0, log=True),
    )


def _spiral_space(trial):
    return dict(
        pitch              = trial.suggest_float("pitch", 2.0, 6.0),
        attract_gain       = trial.suggest_float("attract_gain", 0.5, 5.0, log=True),
        attract_damp_gain  = trial.suggest_float("attract_damp_gain", 0.05, 1.0, log=True),
        drone_repel_gain   = trial.suggest_float("drone_repel_gain", 2.0, 20.0, log=True),
        drone_repel_range  = trial.suggest_float("drone_repel_range", 1.0, 5.0),
        wall_repel_gain    = trial.suggest_float("wall_repel_gain", 0.5, 5.0, log=True),
    )


def _voronoi_space(trial):
    return dict(
        attract_gain       = trial.suggest_float("attract_gain", 0.5, 5.0, log=True),
        attract_damp_gain  = trial.suggest_float("attract_damp_gain", 0.05, 1.0, log=True),
        drone_repel_gain   = trial.suggest_float("drone_repel_gain", 0.5, 10.0, log=True),
        drone_repel_range  = trial.suggest_float("drone_repel_range", 1.0, 6.0),
        wall_repel_gain    = trial.suggest_float("wall_repel_gain", 0.5, 5.0, log=True),
    )


def _grid_decomposition_space(trial):
    return dict(
        block_size         = trial.suggest_int("block_size", 4, 12),
        attract_gain       = trial.suggest_float("attract_gain", 0.5, 5.0, log=True),
        attract_damp_gain  = trial.suggest_float("attract_damp_gain", 0.03, 0.5, log=True),
        drone_repel_gain   = trial.suggest_float("drone_repel_gain", 1.0, 10.0, log=True),
        drone_repel_range  = trial.suggest_float("drone_repel_range", 1.0, 5.0),
        wall_repel_gain    = trial.suggest_float("wall_repel_gain", 0.5, 5.0, log=True),
    )


def _stc_space(trial):
    return dict(
        attract_gain       = trial.suggest_float("attract_gain", 1.0, 8.0, log=True),
        attract_damp_gain  = trial.suggest_float("attract_damp_gain", 0.05, 1.0, log=True),
        drone_repel_gain   = trial.suggest_float("drone_repel_gain", 2.0, 20.0, log=True),
        drone_repel_range  = trial.suggest_float("drone_repel_range", 2.0, 6.0),
        wall_repel_gain    = trial.suggest_float("wall_repel_gain", 0.5, 5.0, log=True),
    )


def _pso_space(trial):
    return dict(
        inertia            = trial.suggest_float("inertia", 0.0, 1.0),
        cognitive          = trial.suggest_float("cognitive", 0.5, 5.0, log=True),
        social             = trial.suggest_float("social", 0.0, 3.0),
        attract_damp_gain  = trial.suggest_float("attract_damp_gain", 0.0, 1.0),
        wall_repel_gain    = trial.suggest_float("wall_repel_gain", 0.5, 5.0, log=True),
    )


def _ga_space(trial):
    return dict(
        p_mutation         = trial.suggest_float("p_mutation", 0.05, 0.6),
        p_crossover        = trial.suggest_float("p_crossover", 0.05, 0.6),
        elite_fraction     = trial.suggest_float("elite_fraction", 0.05, 0.4),
        attract_gain       = trial.suggest_float("attract_gain", 0.5, 5.0, log=True),
        attract_damp_gain  = trial.suggest_float("attract_damp_gain", 0.05, 1.0, log=True),
        drone_repel_gain   = trial.suggest_float("drone_repel_gain", 1.0, 10.0, log=True),
    )


def _sa_space(trial):
    return dict(
        T_initial          = trial.suggest_float("T_initial", 0.5, 10.0, log=True),
        cooling_rate       = trial.suggest_float("cooling_rate", 0.90, 0.999),
        perturb_radius     = trial.suggest_float("perturb_radius", 1.0, 10.0),
        attract_gain       = trial.suggest_float("attract_gain", 0.5, 5.0, log=True),
        attract_damp_gain  = trial.suggest_float("attract_damp_gain", 0.05, 1.0, log=True),
        drone_repel_gain   = trial.suggest_float("drone_repel_gain", 1.0, 10.0, log=True),
    )


def _aco_space(trial):
    return dict(
        pheromone_weight   = trial.suggest_float("pheromone_weight", 0.0, 3.0),
        heuristic_weight   = trial.suggest_float("heuristic_weight", 0.5, 6.0),
        evaporation_rate   = trial.suggest_float("evaporation_rate", 0.01, 0.5),
        target_search_radius = trial.suggest_float("target_search_radius", 3.0, 15.0),
        attract_gain       = trial.suggest_float("attract_gain", 0.5, 5.0, log=True),
        attract_damp_gain  = trial.suggest_float("attract_damp_gain", 0.05, 1.0, log=True),
    )


def _gwo_space(trial):
    return dict(
        a_initial          = trial.suggest_float("a_initial", 1.0, 3.0),
        a_final            = trial.suggest_float("a_final", 0.0, 1.0),
        decay_steps        = trial.suggest_int("decay_steps", 50, 1000, log=True),
        attract_damp_gain  = trial.suggest_float("attract_damp_gain", 0.05, 1.0, log=True),
        wall_repel_gain    = trial.suggest_float("wall_repel_gain", 0.5, 5.0, log=True),
    )


def _pf_space(trial):
    return dict(
        attract_gain       = trial.suggest_float("attract_gain", 0.5, 5.0, log=True),
        attract_damp_gain  = trial.suggest_float("attract_damp_gain", 0.03, 0.5, log=True),
        drone_repel_gain   = trial.suggest_float("drone_repel_gain", 1.0, 10.0, log=True),
        drone_repel_range  = trial.suggest_float("drone_repel_range", 1.0, 6.0),
        wall_repel_gain    = trial.suggest_float("wall_repel_gain", 0.5, 5.0, log=True),
        wall_repel_range   = trial.suggest_float("wall_repel_range", 1.0, 4.0),
    )


def _consensus_space(trial):
    return dict(
        attract_gain       = trial.suggest_float("attract_gain", 0.5, 5.0, log=True),
        attract_damp_gain  = trial.suggest_float("attract_damp_gain", 0.05, 1.0, log=True),
        drone_repel_gain   = trial.suggest_float("drone_repel_gain", 1.0, 10.0, log=True),
        drone_repel_range  = trial.suggest_float("drone_repel_range", 1.0, 6.0),
        wall_repel_gain    = trial.suggest_float("wall_repel_gain", 0.5, 5.0, log=True),
    )


# ---- Controller registry ---------------------------------------------------

def _import_pair(modname: str, ctrl_cls: str, cfg_cls: str):
    """Lazy import so e.g. a torch-less env can still load this module for
    non-MARL controllers."""
    mod = __import__(f"controllers.{modname}", fromlist=[ctrl_cls, cfg_cls])
    return getattr(mod, ctrl_cls), getattr(mod, cfg_cls)


CONTROLLERS = {
    "boustrophedon":   (("boustrophedon", "BoustrophedonController", "BoustrophedonConfig"), _boustrophedon_space),
    "spiral":          (("spiral", "SpiralController", "SpiralConfig"), _spiral_space),
    "voronoi":         (("voronoi_partition", "VoronoiPartitionController", "VoronoiPartitionConfig"), _voronoi_space),
    "grid_decomp":     (("grid_decomposition", "GridDecompositionController", "GridDecompositionConfig"), _grid_decomposition_space),
    "stc":             (("stc", "STCController", "STCConfig"), _stc_space),
    "pso":             (("pso", "PSOController", "PSOConfig"), _pso_space),
    "ga":              (("ga", "GAController", "GAConfig"), _ga_space),
    "sa":              (("sa", "SAController", "SAConfig"), _sa_space),
    "aco":             (("aco", "ACOController", "ACOConfig"), _aco_space),
    "gwo":             (("gwo", "GWOController", "GWOConfig"), _gwo_space),
    "pf":              (("potential_fields", "PotentialFieldsController", "PFConfig"), _pf_space),
    "consensus":       (("consensus", "ConsensusController", "ConsensusConfig"), _consensus_space),
}


def run_one(controller_name: str, n_trials: int, output_path: Path,
            seed: int = 42) -> dict:
    """Run BO for one controller. Returns the result dict (also written to
    `output_path` as JSON)."""
    (import_spec, space_fn) = CONTROLLERS[controller_name]
    ctrl_cls, cfg_cls = _import_pair(*import_spec)

    # Constructor kwargs shared by all controllers — they all accept hover_drone_idx.
    ctrl_kwargs = dict(hover_drone_idx=None)

    def objective(trial: optuna.Trial) -> float:
        params = space_fn(trial)
        # Guard: only pass fields the dataclass actually has (catches typos).
        valid_fields = {f.name for f in fields(cfg_cls)}
        missing = set(params) - valid_fields
        if missing:
            raise ValueError(f"{controller_name}: unknown config fields {missing}")
        cfg = cfg_cls(**params)
        return _evaluate(ctrl_cls, cfg, ctrl_kwargs)

    sampler = TPESampler(seed=seed)
    study = optuna.create_study(direction="maximize", sampler=sampler,
                                study_name=f"{controller_name}_bo")
    t0 = time.time()
    study.optimize(objective, n_trials=n_trials, show_progress_bar=False)
    elapsed = time.time() - t0

    result = {
        "controller": controller_name,
        "n_trials": n_trials,
        "wall_seconds": elapsed,
        "best_value": float(study.best_value),
        "best_params": dict(study.best_params),
        "trials": [
            {"number": t.number, "value": float(t.value) if t.value is not None else None,
             "params": dict(t.params)}
            for t in study.trials
        ],
    }
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, "w") as f:
        json.dump(result, f, indent=2)
    return result


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--controller", required=True, choices=list(CONTROLLERS.keys()))
    parser.add_argument("--n-trials", type=int, default=30)
    parser.add_argument("--output", type=str, default=None,
                        help="JSON output path (default: outputs/bo/<controller>.json)")
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()

    out = Path(args.output) if args.output else Path(f"outputs/bo/{args.controller}.json")
    print(f"[bo_search] controller={args.controller} n_trials={args.n_trials} -> {out}")
    print(f"[bo_search] eval grid: maps={EVAL_MAPS} n_drones={EVAL_N_DRONES} "
          f"seeds={EVAL_SEEDS} ({len(EVAL_MAPS) * len(EVAL_N_DRONES) * len(EVAL_SEEDS)} cells/trial)")

    result = run_one(args.controller, n_trials=args.n_trials, output_path=out, seed=args.seed)
    print()
    print(f"[bo_search] DONE in {result['wall_seconds']:.1f}s")
    print(f"[bo_search] best score: {result['best_value']:+.4f}")
    print(f"[bo_search] best params: {json.dumps(result['best_params'], indent=2)}")


if __name__ == "__main__":
    main()
