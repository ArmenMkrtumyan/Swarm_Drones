"""DDPG vs TD3 vs PPO comparison on HoverPretrain-v0.

Runs every (algo, seed) pair listed in the config. Each run:
  - identical env + DR + reward shaping (same yaml block);
  - identical network arch and learning rate;
  - identical total timesteps;
  - own TensorBoard log dir;
  - own checkpoint saved at the end.

Off-policy (DDPG, TD3) share buffer/target-net/noise hyperparams. On-policy
(PPO) has its own rollout/GAE/clip block. Both are sized for "comparable
timesteps" — same wall of env transitions, just consumed differently.

Usage (one shot, all seeds × all algos):

    lab/rl/.rl_venv/Scripts/python.exe -m lab.rl.train_compare \\
        --config lab/rl/cfg/compare_v0.yaml

Single (algo, seed) for smoke testing:

    ... --config ... --only DDPG:0 --total-timesteps 5000
"""

from __future__ import annotations

import argparse
import json
import logging
import math
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from lab.rl.envs.hover_pretrain_v0 import (
    DRRange, GainSpec, PretrainConfig,
)
from lab.rl.envs.sb3_vec_adapter import HoverPretrainSB3VecEnv


log = logging.getLogger("lab.rl.train_compare")


def _build_pretrain_cfg(raw: dict, num_envs_override: int | None = None) -> PretrainConfig:
    e = raw["env"]
    dr = raw["dr"]
    return PretrainConfig(
        num_envs=int(num_envs_override or e["num_envs"]),
        device=str(e["device"]),
        hover_alt_m=float(e["hover_alt_m"]),
        max_episode_steps=int(e["max_episode_steps"]),
        inner_steps_per_action=int(e["inner_steps_per_action"]),
        physics_dt=float(e["physics_dt"]),
        crash_alt_m=float(e["crash_alt_m"]),
        crash_pos_xy_m=float(e["crash_pos_xy_m"]),
        crash_attitude_rad=math.radians(float(e["crash_attitude_deg"])),
        survival_bonus=float(e["survival_bonus"]),
        crash_penalty=float(e["crash_penalty"]),
        reward_scale=float(e["reward_scale"]),
        base_step_reward=float(e["base_step_reward"]),
        w_alt_err=float(e["w_alt_err"]),
        w_pos_err=float(e["w_pos_err"]),
        w_vel=float(e["w_vel"]),
        w_attitude=float(e["w_attitude"]),
        w_gyro=float(e["w_gyro"]),
        w_action_smooth=float(e["w_action_smooth"]),
        dr_mass=DRRange(**dr["mass"]),
        dr_motor_tau=DRRange(**dr["motor_tau"]),
        dr_K_thrust_jitter=DRRange(**dr["K_thrust_jitter"]),
        dr_K_drag=DRRange(**dr["K_drag"]),
        dr_gyro_bias=DRRange(**dr["gyro_bias"]),
        dr_gyro_noise_sigma=DRRange(**dr["gyro_noise_sigma"]),
        dr_init_attitude_rad=DRRange(
            lo=math.radians(float(dr["init_attitude_deg"]["lo"])),
            hi=math.radians(float(dr["init_attitude_deg"]["hi"])),
        ),
        dr_init_alt_offset=DRRange(**dr["init_alt_offset"]),
        dr_init_xy_offset=DRRange(**dr["init_xy_offset"]),
        dr_initial_gain_mistune_pct=DRRange(**dr["initial_gain_mistune_pct"]),
        gains=[GainSpec(**g) for g in raw["gains"]],
    )


def _episode_reward_callback():
    """Build a callback that emits rollout/ep_rew_mean to TB.

    Our HoverPretrainSB3VecEnv isn't Monitor-wrapped (it's a torch-batched
    VecEnv, awkward to wrap), so SB3's stock rollout/* logging stays empty.
    We accumulate per-env returns manually across the auto-reset boundary.
    """
    from stable_baselines3.common.callbacks import BaseCallback

    class EpisodeRewardCallback(BaseCallback):
        def __init__(self, verbose: int = 0) -> None:
            super().__init__(verbose)
            self._ep_returns: list[float] = []

        def _on_training_start(self) -> None:
            self._running_sum = np.zeros(self.training_env.num_envs)

        def _on_step(self) -> bool:
            r = self.locals["rewards"]
            d = self.locals["dones"]
            self._running_sum += r
            for i, done in enumerate(d):
                if done:
                    self._ep_returns.append(float(self._running_sum[i]))
                    self._running_sum[i] = 0.0
            if self._ep_returns:
                window = self._ep_returns[-256:]
                self.logger.record("rollout/ep_rew_mean",
                                   float(sum(window) / len(window)))
                self.logger.record("rollout/ep_count", len(self._ep_returns))
            return True

    return EpisodeRewardCallback


def _build_action_noise(num_envs: int, action_dim: int, sigma: float):
    from stable_baselines3.common.noise import NormalActionNoise
    # SB3 off-policy expects a single (action_dim,) noise sampler at the
    # API; per-env independence is handled internally.
    return NormalActionNoise(
        mean=np.zeros(action_dim, dtype=np.float32),
        sigma=np.full(action_dim, sigma, dtype=np.float32),
    )


def _make_model(algo: str, raw: dict, vec_env, seed: int, tb_dir: str):
    shared = raw["shared"]
    common_kwargs: dict[str, Any] = dict(
        learning_rate=float(shared["learning_rate"]),
        gamma=float(shared["gamma"]),
        seed=int(seed),
        tensorboard_log=tb_dir,
        device=str(shared["device"]),
        policy_kwargs=dict(net_arch=list(shared["policy_kwargs"]["net_arch"])),
        verbose=1,
    )

    action_dim = vec_env.action_space.shape[0]

    algo = algo.upper()
    if algo == "DDPG":
        from stable_baselines3 import DDPG
        return DDPG(
            "MlpPolicy", vec_env,
            buffer_size=int(shared["buffer_size"]),
            learning_starts=int(shared["learning_starts"]),
            batch_size=int(shared["batch_size"]),
            tau=float(shared["tau"]),
            train_freq=tuple(shared["train_freq"]),
            gradient_steps=int(shared["gradient_steps"]),
            action_noise=_build_action_noise(
                vec_env.num_envs, action_dim, float(shared["action_noise_sigma"])),
            **common_kwargs,
        )
    if algo == "TD3":
        from stable_baselines3 import TD3
        td3 = raw["td3"]
        return TD3(
            "MlpPolicy", vec_env,
            buffer_size=int(shared["buffer_size"]),
            learning_starts=int(shared["learning_starts"]),
            batch_size=int(shared["batch_size"]),
            tau=float(shared["tau"]),
            train_freq=tuple(shared["train_freq"]),
            gradient_steps=int(shared["gradient_steps"]),
            target_policy_noise=float(td3["target_policy_noise"]),
            target_noise_clip=float(td3["target_noise_clip"]),
            policy_delay=int(td3["policy_delay"]),
            action_noise=_build_action_noise(
                vec_env.num_envs, action_dim, float(shared["action_noise_sigma"])),
            **common_kwargs,
        )
    if algo == "SAC":
        from stable_baselines3 import SAC
        sac = raw.get("sac", {})
        ent_coef = sac.get("ent_coef", "auto")
        target_entropy = sac.get("target_entropy", "auto")
        # Convert string numerics if user passed them as strings.
        if isinstance(ent_coef, str) and ent_coef not in ("auto",):
            ent_coef = float(ent_coef)
        if isinstance(target_entropy, str) and target_entropy not in ("auto",):
            target_entropy = float(target_entropy)
        return SAC(
            "MlpPolicy", vec_env,
            buffer_size=int(shared["buffer_size"]),
            learning_starts=int(shared["learning_starts"]),
            batch_size=int(shared["batch_size"]),
            tau=float(shared["tau"]),
            train_freq=tuple(shared["train_freq"]),
            gradient_steps=int(shared["gradient_steps"]),
            ent_coef=ent_coef,
            target_update_interval=int(sac.get("target_update_interval", 1)),
            target_entropy=target_entropy,
            **common_kwargs,
        )
    if algo == "PPO":
        from stable_baselines3 import PPO
        ppo = raw["ppo"]
        if "device" in ppo:
            common_kwargs["device"] = str(ppo["device"])
        return PPO(
            "MlpPolicy", vec_env,
            n_steps=int(ppo["n_steps"]),
            batch_size=int(ppo["batch_size"]),
            n_epochs=int(ppo["n_epochs"]),
            gae_lambda=float(ppo["gae_lambda"]),
            clip_range=float(ppo["clip_range"]),
            ent_coef=float(ppo["ent_coef"]),
            vf_coef=float(ppo["vf_coef"]),
            max_grad_norm=float(ppo["max_grad_norm"]),
            **common_kwargs,
        )
    raise ValueError(f"unknown algo: {algo}")


def _train_one(algo: str, seed: int, raw: dict, total_timesteps: int) -> dict:
    """Train a single (algo, seed) run, save checkpoint, return summary dict."""
    log_root = Path(raw["log_root"])
    ckpt_root = Path(raw["checkpoint_root"])
    log_root.mkdir(parents=True, exist_ok=True)
    ckpt_root.mkdir(parents=True, exist_ok=True)

    run_name = f"{algo}_seed{seed}"
    tb_dir = str(log_root)
    ckpt_path = ckpt_root / f"{run_name}.zip"

    log.info("=== START %s (total_timesteps=%d) ===", run_name, total_timesteps)
    cfg = _build_pretrain_cfg(raw)
    vec_env = HoverPretrainSB3VecEnv(cfg)
    vec_env.seed(int(seed))

    model = _make_model(algo, raw, vec_env, seed=seed, tb_dir=tb_dir)
    EpisodeRewardCallback = _episode_reward_callback()

    # PPO logs per-rollout (n_steps*n_envs transitions); off-policy logs per
    # env-step. To get comparable TB granularity across all algos, set
    # log_interval=1 for PPO (every rollout) and 10 for DDPG/TD3 (every 10
    # env-step calls = 80 transitions).
    log_interval = 1 if algo.upper() == "PPO" else 10
    try:
        model.learn(
            total_timesteps=int(total_timesteps),
            callback=EpisodeRewardCallback(),
            tb_log_name=run_name,
            log_interval=log_interval,
            progress_bar=False,
        )
    finally:
        model.save(ckpt_path)
        log.info("saved %s", ckpt_path)
        vec_env.close()

    return {
        "algo": algo,
        "seed": int(seed),
        "checkpoint": str(ckpt_path),
        "tb_log_name": run_name,
        "tb_dir": tb_dir,
        "total_timesteps": int(total_timesteps),
    }


def main() -> None:
    ap = argparse.ArgumentParser(description="DDPG/TD3/PPO comparison runner.")
    ap.add_argument("--config", required=True)
    ap.add_argument(
        "--only", default=None,
        help="Run a single (ALGO:SEED) pair, e.g. DDPG:0. Useful for smoke tests.",
    )
    ap.add_argument(
        "--total-timesteps", type=int, default=None,
        help="Override total_timesteps (e.g. 5000 for smoke).",
    )
    ap.add_argument("--log-level", default="INFO")
    args = ap.parse_args()

    logging.basicConfig(level=args.log_level.upper(),
                        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s")

    raw = yaml.safe_load(Path(args.config).read_text())
    total_timesteps = int(args.total_timesteps or raw["shared"]["total_timesteps"])

    if args.only:
        algo, seed = args.only.split(":")
        pairs = [(algo.upper(), int(seed))]
    else:
        pairs = [(a, s) for a in raw["algos"] for s in raw["seeds"]]

    log.info("schedule: %d runs", len(pairs))
    summaries = []
    for algo, seed in pairs:
        try:
            summaries.append(_train_one(algo, seed, raw, total_timesteps))
        except Exception:
            log.exception("run %s_seed%d FAILED — continuing with next", algo, seed)

    out = Path(raw["log_root"]) / "_runs_summary.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(summaries, indent=2))
    log.info("wrote %s (%d runs)", out, len(summaries))


if __name__ == "__main__":
    main()
