"""
Train an Independent-PPO (parameter-sharing) policy on `CoverageGymEnv`.

Saves the trained model to `outputs/marl_ppo.zip` so the benchmark
harness and `controllers.marl.MARLController` can load it.

Usage:
    python tools/train_marl.py                        # default 100k steps
    python tools/train_marl.py --steps 250000         # longer training
    python tools/train_marl.py --grid 15 --drones 4   # bigger swarm
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

# Make project root importable.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import numpy as np

from constants import OUTPUTS_DIR
from controllers.marl_env import CoverageGymEnv

try:
    from stable_baselines3 import PPO
    from stable_baselines3.common.callbacks import BaseCallback, EvalCallback
    from stable_baselines3.common.env_util import make_vec_env
    from stable_baselines3.common.monitor import Monitor
except ImportError as e:  # pragma: no cover
    raise ImportError(
        "Training requires stable-baselines3. "
        "Install with: pip install stable-baselines3 torch"
    ) from e


class CoverageTracker(BaseCallback):
    """
    Logs episode coverage % to console every `log_every` env steps.
    Stable-Baselines3's built-in logger doesn't track our custom info,
    so we do it ourselves.
    """

    def __init__(self, log_every: int = 5000, verbose: int = 0):
        super().__init__(verbose)
        self.log_every = log_every
        self._next_log = log_every
        self._ep_coverage: list[float] = []
        self._ep_steps: list[int] = []
        self._ep_done_counts: list[int] = []

    def _on_step(self) -> bool:
        # SB3 fires _on_step each env.step. Inspect 'infos' / 'dones'.
        infos = self.locals.get("infos", [])
        dones = self.locals.get("dones", np.zeros(len(infos), dtype=bool))
        for info, done in zip(infos, dones):
            if not done:
                continue
            self._ep_coverage.append(info.get("coverage", 0.0))
            self._ep_steps.append(info.get("steps", 0))
            self._ep_done_counts.append(1 if info.get("is_done") else 0)

        if self.num_timesteps >= self._next_log:
            recent = self._ep_coverage[-30:]
            recent_steps = self._ep_steps[-30:]
            recent_done = self._ep_done_counts[-30:]
            if recent:
                print(
                    f"  t={self.num_timesteps:>7d}  "
                    f"ep_cov={np.mean(recent):.1%}  "
                    f"ep_len={np.mean(recent_steps):.0f}  "
                    f"100% rate={np.mean(recent_done):.0%}  "
                    f"(last {len(recent)} eps)"
                )
            self._next_log += self.log_every
        return True


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--steps", type=int, default=100_000,
                   help="total PPO timesteps (default 100k)")
    p.add_argument("--grid", type=int, default=11,
                   help="grid side length used during training (default 11)")
    p.add_argument("--drones", type=int, default=3,
                   help="number of drones during training (default 3)")
    p.add_argument("--max-ep-steps", type=int, default=1500,
                   help="hard step cap per episode (default 1500)")
    p.add_argument("--map-kind", choices=["random", "maze"], default="random",
                   help="map generator used for training")
    p.add_argument("--n-envs", type=int, default=4,
                   help="parallel envs for PPO rollout (default 4)")
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--out", type=str,
                   default=str(OUTPUTS_DIR / "marl_ppo.zip"),
                   help="output checkpoint path")
    return p.parse_args()


def main() -> None:
    args = parse_args()

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    print("=== Training shared-policy PPO on CoverageGymEnv ===")
    print(f"  grid_size:      {args.grid}")
    print(f"  n_drones:       {args.drones}")
    print(f"  map_kind:       {args.map_kind}")
    print(f"  total steps:    {args.steps:,}")
    print(f"  max ep steps:   {args.max_ep_steps:,}")
    print(f"  parallel envs:  {args.n_envs}")
    print(f"  output:         {out_path}")
    print()

    def make_one_env():
        return CoverageGymEnv(
            grid_size=args.grid,
            n_drones=args.drones,
            max_steps=args.max_ep_steps,
            map_kind=args.map_kind,
        )

    vec_env = make_vec_env(make_one_env, n_envs=args.n_envs, seed=args.seed)

    # Eval env (separate from training; same generator). Used by EvalCallback
    # to checkpoint the best policy seen so far based on rolling eval reward.
    # Without this, PPO + entropy bonus regularly drifts off-peak as training
    # continues — we observed coverage peaking at ~25-75k steps then declining.
    eval_env = Monitor(make_one_env())

    model = PPO(
        "MlpPolicy",
        vec_env,
        # Smaller MLP than SB3 default (64×64 → enough for our 96-dim obs)
        policy_kwargs=dict(net_arch=[128, 128]),
        learning_rate=3e-4,
        n_steps=512,
        batch_size=128,
        n_epochs=10,
        gamma=0.99,
        gae_lambda=0.95,
        clip_range=0.2,
        # Lower entropy bonus than SB3 default — prevents the policy from
        # drifting back to random late in training.
        ent_coef=0.005,
        verbose=1,
        seed=args.seed,
        device="cpu",   # MLP this small is faster on CPU than M1 MPS overhead
    )

    # Save best-by-eval-reward to a sibling path; also keep the final
    # checkpoint at args.out for inspection. The `--out` path receives the
    # best model at the end (we copy it from the eval-best dir).
    best_dir = out_path.parent / f".best_{out_path.stem}"
    best_dir.mkdir(exist_ok=True)
    eval_cb = EvalCallback(
        eval_env,
        best_model_save_path=str(best_dir),
        n_eval_episodes=5,
        eval_freq=max(1, 5_000 // args.n_envs),  # eval ~ every 5k env steps
        deterministic=False,                     # match how MARLController runs
        render=False,
        verbose=0,
    )
    track_cb = CoverageTracker(log_every=5000)

    t0 = time.time()
    model.learn(
        total_timesteps=args.steps,
        callback=[eval_cb, track_cb],
        progress_bar=False,
    )
    dt = time.time() - t0
    print(f"\nTraining took {dt / 60:.1f} min  ({args.steps / dt:.0f} steps/sec)")

    # Promote the eval-best checkpoint to args.out. If EvalCallback never
    # saved one (rare — only if training was very short), fall back to final.
    best_zip = best_dir / "best_model.zip"
    if best_zip.exists():
        import shutil
        shutil.copy(str(best_zip), str(out_path))
        print(f"Saved BEST checkpoint to: {out_path}  "
              f"(eval mean reward: {eval_cb.best_mean_reward:.1f})")
    else:
        model.save(str(out_path))
        print(f"Saved final checkpoint (no eval-best found): {out_path}")

    # Reload best for the final eval (so the printed summary reflects what
    # MARLController will actually load).
    model = PPO.load(str(out_path), device="cpu")

    # Quick eval — 5 episodes, STOCHASTIC (matches benchmark behavior).
    print("\n=== Quick eval (5 episodes, stochastic) ===")
    eval_env = make_one_env()
    coverages, lengths, dones = [], [], []
    for k in range(5):
        obs, _ = eval_env.reset(seed=1000 + k)
        terminated = truncated = False
        while not (terminated or truncated):
            action, _ = model.predict(obs, deterministic=False)
            obs, _, terminated, truncated, info = eval_env.step(action)
        coverages.append(info["coverage"])
        lengths.append(info["steps"])
        dones.append(1 if info["is_done"] else 0)
        print(
            f"  ep {k+1}: cov={info['coverage']:.1%}  "
            f"steps={info['steps']}  done={bool(info['is_done'])}"
        )
    print(
        f"  mean: cov={np.mean(coverages):.1%}  "
        f"steps={np.mean(lengths):.0f}  100% rate={np.mean(dones):.0%}"
    )


if __name__ == "__main__":
    main()
