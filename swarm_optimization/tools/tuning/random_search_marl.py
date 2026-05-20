"""
Random search over MARL (PPO) hyperparameters AND reward shaping weights.

Each trial:
    1. Sample a config from the search space.
    2. Train a fresh PPO model on `CoverageGymEnv` for `--steps-per-trial`
       env steps (default 30k — about 1/5 of the standard training budget,
       sufficient signal for ranking but cheap enough that 20-30 trials fit
       in under an hour).
    3. Evaluate with 5 stochastic episodes; compute composite score from
       `score.composite_score` per episode and average.

Why random over grid:
    Same Bergstra & Bengio (2012) argument — most PPO hyperparams have
    weak individual effect, so random samples cover the relevant axes
    better than a grid at fixed budget. Especially apt here because each
    trial is *expensive* (full training run), so sample efficiency matters.

Search space (uniform unless noted):
    learning_rate            log-uniform [1e-5, 1e-3]
    n_steps                  choice [256, 512, 1024]
    batch_size               choice [32, 64, 128]   (always divides n_steps)
    n_epochs                 int uniform [3, 20]
    gamma                    uniform [0.9, 0.999]
    gae_lambda               uniform [0.85, 0.99]
    clip_range               uniform [0.1, 0.4]
    ent_coef                 log-uniform [1e-4, 0.05]
    overlap_penalty_per_m2   uniform [0.0, 0.05]
    wasted_visit_penalty     uniform [0.0, 0.05]
    energy_penalty_per_kj    uniform [0.0, 1.0]

`net_arch` is held at [128, 128] — small MLP; tuning it on top of the
above is unlikely to beat what hyperparameter sampling already does for
this problem.

Outputs:
    outputs/images/random_search_marl.csv          — per-trial config + score
    outputs/images/random_search_marl_top.txt      — top-K
    outputs/images/random_search_marl_progress.png — best-so-far vs trial

Usage:
    python tools/tuning/random_search_marl.py                      # 20 trials, n=5
    python tools/tuning/random_search_marl.py --trials 30 --drones 5
    python tools/tuning/random_search_marl.py --steps-per-trial 50000
"""

from __future__ import annotations

import argparse
import csv
import sys
import time
from dataclasses import dataclass
from pathlib import Path

# Make project root importable.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from constants import DATA_DIR, PLOTS_DIR
from controllers.marl_env import CoverageGymEnv
from tools.score import composite_score

try:
    from stable_baselines3 import PPO
    from stable_baselines3.common.env_util import make_vec_env
except ImportError as e:
    raise ImportError(
        "Random MARL search needs stable-baselines3. "
        "pip install stable-baselines3 torch"
    ) from e


# Categorical choices that respect PPO's n_steps × n_envs / batch_size constraint.
N_STEPS_CHOICES = [256, 512, 1024]
BATCH_SIZE_CHOICES = [32, 64, 128]


@dataclass
class TrialResult:
    trial: int
    learning_rate: float
    n_steps: int
    batch_size: int
    n_epochs: int
    gamma: float
    gae_lambda: float
    clip_range: float
    ent_coef: float
    overlap_penalty_per_m2: float
    wasted_visit_penalty: float
    energy_penalty_per_kj: float
    eval_mean_score: float
    eval_mean_coverage: float
    eval_mean_overlap_m2: float
    eval_mean_wasted: float
    eval_mean_energy_kj: float
    train_seconds: float


def sample_config(rng: np.random.Generator) -> dict:
    """Draw one MARL config."""
    lr = float(10 ** rng.uniform(-5, -3))
    ent = float(10 ** rng.uniform(-4, np.log10(0.05)))
    return {
        "learning_rate":  lr,
        "n_steps":        int(rng.choice(N_STEPS_CHOICES)),
        "batch_size":     int(rng.choice(BATCH_SIZE_CHOICES)),
        "n_epochs":       int(rng.integers(3, 21)),
        "gamma":          float(rng.uniform(0.9, 0.999)),
        "gae_lambda":     float(rng.uniform(0.85, 0.99)),
        "clip_range":     float(rng.uniform(0.1, 0.4)),
        "ent_coef":       ent,
        "overlap_penalty_per_m2": float(rng.uniform(0.0, 0.05)),
        "wasted_visit_penalty":   float(rng.uniform(0.0, 0.05)),
        "energy_penalty_per_kj":  float(rng.uniform(0.0, 1.0)),
    }


def make_env_factory(grid_size: int, n_drones: int, max_ep_steps: int,
                     map_kind: str, cfg: dict):
    """Factory used by `make_vec_env`."""
    def _make():
        return CoverageGymEnv(
            grid_size=grid_size,
            n_drones=n_drones,
            max_steps=max_ep_steps,
            map_kind=map_kind,
            overlap_penalty_per_m2=cfg["overlap_penalty_per_m2"],
            wasted_visit_penalty=cfg["wasted_visit_penalty"],
            energy_penalty_per_kj=cfg["energy_penalty_per_kj"],
        )
    return _make


def evaluate_model(model, env: CoverageGymEnv, *, n_episodes: int,
                   energy_budget_j: float) -> dict:
    """Run n stochastic eval episodes; return mean composite score + components."""
    scores, covs, ovs, was, ens = [], [], [], [], []
    for k in range(n_episodes):
        obs, _ = env.reset(seed=10_000 + k)
        terminated = truncated = False
        while not (terminated or truncated):
            action, _ = model.predict(obs, deterministic=False)
            obs, _, terminated, truncated, info = env.step(action)
        # Reach into the underlying CoverageEnv for the metrics composite_score wants.
        ce = env.env
        from maze import FREE
        free_mask = ce.grid == FREE
        free_area_m2 = float(free_mask.sum()) * (ce.sim_cfg.meters_per_cell ** 2)
        unique_visited = int((ce.covered & free_mask).sum())
        energy_used_j = (ce.battery_cfg.initial_energy_j * ce.n_drones) - sum(
            d.battery_j for d in ce.drones
        )
        s = composite_score(
            coverage_fraction=ce.coverage_fraction(),
            overlap_m2=ce.overlap_cells_m2(),
            free_area_m2=free_area_m2,
            wasted_visits=ce.wasted_visits_total(),
            unique_cells_visited=unique_visited,
            energy_used_j=float(energy_used_j),
            energy_budget_j=energy_budget_j,
        )
        scores.append(s)
        covs.append(ce.coverage_fraction())
        ovs.append(ce.overlap_cells_m2())
        was.append(ce.wasted_visits_total())
        ens.append(energy_used_j / 1000)
    return {
        "score": float(np.mean(scores)),
        "coverage": float(np.mean(covs)),
        "overlap_m2": float(np.mean(ovs)),
        "wasted": float(np.mean(was)),
        "energy_kj": float(np.mean(ens)),
    }


def parse_args():
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--trials", type=int, default=20,
                   help="number of random configs to train (default 20)")
    p.add_argument("--steps-per-trial", type=int, default=30_000,
                   help="PPO env steps per trial (default 30k)")
    p.add_argument("--n-eval-episodes", type=int, default=5)
    p.add_argument("--drones", type=int, default=5)
    p.add_argument("--grid", type=int, default=33,
                   help="env grid_size used during training (default 33)")
    p.add_argument("--max-ep-steps", type=int, default=1500)
    p.add_argument("--map-kind", choices=["random", "maze"], default="random")
    p.add_argument("--n-envs", type=int, default=4)
    p.add_argument("--seed", type=int, default=42)
    p.add_argument("--top-k", type=int, default=10)
    return p.parse_args()


def main():
    args = parse_args()
    rng = np.random.default_rng(args.seed)

    print(f"=== MARL random search ===")
    print(f"  grid:           {args.grid}")
    print(f"  n_drones:       {args.drones}")
    print(f"  trials:         {args.trials}  ({args.steps_per_trial:,} env steps each)")
    print(f"  eval episodes:  {args.n_eval_episodes}")
    print()

    results: list[TrialResult] = []
    per_trial_score: list[float] = []
    t0 = time.time()
    for ti in range(args.trials):
        cfg = sample_config(rng)
        factory = make_env_factory(
            args.grid, args.drones, args.max_ep_steps, args.map_kind, cfg,
        )
        vec_env = make_vec_env(factory, n_envs=args.n_envs,
                               seed=args.seed + ti)
        eval_env = factory()  # standalone for eval

        t_train = time.time()
        try:
            model = PPO(
                "MlpPolicy",
                vec_env,
                policy_kwargs=dict(net_arch=[128, 128]),
                learning_rate=cfg["learning_rate"],
                n_steps=cfg["n_steps"],
                batch_size=cfg["batch_size"],
                n_epochs=cfg["n_epochs"],
                gamma=cfg["gamma"],
                gae_lambda=cfg["gae_lambda"],
                clip_range=cfg["clip_range"],
                ent_coef=cfg["ent_coef"],
                verbose=0,
                seed=args.seed + ti,
                device="cpu",
            )
            model.learn(total_timesteps=args.steps_per_trial,
                        progress_bar=False)
            train_dt = time.time() - t_train

            # `eval_env.env` (the underlying CoverageEnv) is built on first
            # reset(); call once so the battery config is accessible.
            eval_env.reset(seed=0)
            energy_budget = (eval_env.env.battery_cfg.initial_energy_j
                             * eval_env.n_drones)

            ev = evaluate_model(model, eval_env,
                                n_episodes=args.n_eval_episodes,
                                energy_budget_j=energy_budget)
        except Exception as e:
            print(f"  [{ti+1}/{args.trials}]  FAILED: {e}")
            train_dt = time.time() - t_train
            ev = {"score": -1.0, "coverage": 0.0, "overlap_m2": 0.0,
                  "wasted": 0.0, "energy_kj": 0.0}

        r = TrialResult(
            trial=ti,
            learning_rate=cfg["learning_rate"],
            n_steps=cfg["n_steps"],
            batch_size=cfg["batch_size"],
            n_epochs=cfg["n_epochs"],
            gamma=cfg["gamma"],
            gae_lambda=cfg["gae_lambda"],
            clip_range=cfg["clip_range"],
            ent_coef=cfg["ent_coef"],
            overlap_penalty_per_m2=cfg["overlap_penalty_per_m2"],
            wasted_visit_penalty=cfg["wasted_visit_penalty"],
            energy_penalty_per_kj=cfg["energy_penalty_per_kj"],
            eval_mean_score=ev["score"],
            eval_mean_coverage=ev["coverage"],
            eval_mean_overlap_m2=ev["overlap_m2"],
            eval_mean_wasted=ev["wasted"],
            eval_mean_energy_kj=ev["energy_kj"],
            train_seconds=train_dt,
        )
        results.append(r)
        per_trial_score.append(ev["score"])
        best_so_far = float(np.maximum.accumulate(per_trial_score)[-1])
        marker = " ✓ NEW BEST" if ev["score"] == best_so_far else ""
        print(
            f"  [{ti+1:>2d}/{args.trials}]  "
            f"lr={cfg['learning_rate']:.1e} ent={cfg['ent_coef']:.2e} "
            f"ovP={cfg['overlap_penalty_per_m2']:.3f} wsP={cfg['wasted_visit_penalty']:.3f} "
            f"enP={cfg['energy_penalty_per_kj']:.2f} | "
            f"score={ev['score']:+.3f} cov={ev['coverage']:.0%} "
            f"(train {train_dt:.0f}s, best {best_so_far:+.3f}){marker}"
        )

    dt = time.time() - t0
    print(f"\nTotal wall time: {dt/60:.1f} min")

    # ---- write CSV ----
    DATA_DIR.mkdir(parents=True, exist_ok=True)
    PLOTS_DIR.mkdir(parents=True, exist_ok=True)
    csv_path = DATA_DIR / "random_search_marl.csv"
    from dataclasses import asdict
    with csv_path.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(asdict(results[0]).keys()))
        w.writeheader()
        for r in results:
            w.writerow(asdict(r))
    print(f"Wrote: {csv_path}")

    # ---- top-K ----
    summary = sorted(results, key=lambda r: -r.eval_mean_score)
    top_path = DATA_DIR / "random_search_marl_top.txt"
    lines = []
    lines.append(f"Top {args.top_k} MARL configs after {args.trials} trials "
                 f"({args.steps_per_trial:,} steps each, n_drones={args.drones})\n\n")
    lines.append(
        f"{'rank':>4s}  {'score':>7s}  {'cov':>5s}  {'overlap':>8s}  "
        f"{'wasted':>7s}  {'energy':>8s}    {'lr':>8s}  {'ent':>8s}  "
        f"{'ovP':>5s}  {'wsP':>5s}  {'enP':>5s}\n"
    )
    lines.append("-" * 110 + "\n")
    for rank, r in enumerate(summary[: args.top_k], 1):
        lines.append(
            f"{rank:>4d}  {r.eval_mean_score:>+7.3f}  {r.eval_mean_coverage:>4.0%}  "
            f"{r.eval_mean_overlap_m2:>7.0f}m²  {r.eval_mean_wasted:>7.0f}  "
            f"{r.eval_mean_energy_kj:>5.1f}kJ    "
            f"{r.learning_rate:>8.1e}  {r.ent_coef:>8.1e}  "
            f"{r.overlap_penalty_per_m2:>5.3f}  {r.wasted_visit_penalty:>5.3f}  "
            f"{r.energy_penalty_per_kj:>5.2f}\n"
        )
    top_path.write_text("".join(lines))
    print()
    print("".join(lines), end="")
    print(f"Wrote: {top_path}")

    # ---- progress plot ----
    fig, ax = plt.subplots(figsize=(8, 4.5))
    best = np.maximum.accumulate(per_trial_score)
    ax.plot(range(1, len(best) + 1), best, color="#1f77b4",
            linewidth=2, label="best mean score so far")
    ax.scatter(range(1, len(per_trial_score) + 1), per_trial_score,
               color="#aaa", s=12, label="per-trial mean")
    ax.set_xlabel("Trial number")
    ax.set_ylabel("Composite score (mean over 5 eval episodes)")
    ax.set_title(f"MARL random search — {args.trials} trials × {args.steps_per_trial//1000}k steps")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="lower right")
    fig.tight_layout()
    out = PLOTS_DIR / "random_search_marl_progress.png"
    fig.savefig(out, dpi=130)
    plt.close(fig)
    print(f"Wrote: {out}")


if __name__ == "__main__":
    main()
