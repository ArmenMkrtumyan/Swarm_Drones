"""Stage-3 Phase-1 trainer: pure-torch vectorized hover pre-training.

Trains a policy against the in-process torch simulator (no Isaac, no SITL).
The trained policy is a drop-in initialization for ``train_hover.py``
fine-tuning through real ArduPilot SITL.

Usage:

    cd Swarm_Drones
    lab/rl/.rl_venv/Scripts/python.exe -m lab.rl.train_pretrain \\
        --config lab/rl/cfg/pretrain_v0.yaml

Outputs:

    logs/tb_logs/pretrain_v0/         TensorBoard
    logs/checkpoints/pretrain_v0/     periodic + final SB3 zips

Open TensorBoard:

    lab/rl/.rl_venv/Scripts/python.exe -m tensorboard.main --logdir logs/tb_logs
"""

from __future__ import annotations

import argparse
import logging
import math
from pathlib import Path

import yaml

from lab.rl.envs.hover_pretrain_v0 import (
    DRRange, GainSpec, PretrainConfig,
)
from lab.rl.envs.sb3_vec_adapter import HoverPretrainSB3VecEnv


log = logging.getLogger("lab.rl.train_pretrain")


def load_config(path: str) -> tuple[PretrainConfig, dict]:
    raw = yaml.safe_load(Path(path).read_text())
    e = raw["env"]
    dr = raw["dr"]
    cfg = PretrainConfig(
        num_envs=int(e["num_envs"]),
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
    return cfg, raw


def main() -> None:
    ap = argparse.ArgumentParser(description="Train HoverPretrain-v0.")
    ap.add_argument("--config", required=True)
    ap.add_argument("--smoke", action="store_true",
                    help="Run a tiny verification (low total_timesteps, low envs)")
    ap.add_argument("--log-level", default="INFO")
    args = ap.parse_args()

    logging.basicConfig(level=args.log_level.upper(),
                        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s")

    cfg, raw = load_config(args.config)
    if args.smoke:
        cfg.num_envs = min(64, cfg.num_envs)
        raw["total_timesteps"] = 5000
        raw["checkpoint_every_steps"] = 1000

    Path(raw["log_dir"]).mkdir(parents=True, exist_ok=True)
    Path(raw["checkpoint_dir"]).mkdir(parents=True, exist_ok=True)

    log.info("creating env: num_envs=%d, device=%s", cfg.num_envs, cfg.device)
    vec_env = HoverPretrainSB3VecEnv(cfg)

    from stable_baselines3 import DDPG, PPO, SAC, TD3
    from stable_baselines3.common.callbacks import (
        BaseCallback, CallbackList, CheckpointCallback,
    )

    name = str(raw["algo"]).upper()
    table = {"PPO": PPO, "SAC": SAC, "TD3": TD3, "DDPG": DDPG}
    AlgoClass = table[name]

    model = AlgoClass(
        "MlpPolicy",
        vec_env,
        seed=int(raw["seed"]),
        tensorboard_log=raw["log_dir"],
        **raw["algo_kwargs"],
    )

    class EpisodeRewardCallback(BaseCallback):
        """Logs running mean reward across all parallel envs to TensorBoard.
        SB3's stock rollout/* logging needs Monitor wrappers, which are awkward
        for our batched VecEnv — so we accumulate per-env rewards manually
        across the auto-reset boundary and log mean episode return.
        """
        def __init__(self, verbose: int = 0) -> None:
            super().__init__(verbose)
            self._ep_returns: list[float] = []
            self._running_sum = None
            self._running_len = None

        def _on_training_start(self) -> None:
            import numpy as np
            self._running_sum = np.zeros(self.training_env.num_envs)
            self._running_len = np.zeros(self.training_env.num_envs)

        def _on_step(self) -> bool:
            import numpy as np
            r = self.locals["rewards"]
            d = self.locals["dones"]
            self._running_sum += r
            self._running_len += 1
            for i, done in enumerate(d):
                if done:
                    self._ep_returns.append(float(self._running_sum[i]))
                    self._running_sum[i] = 0.0
                    self._running_len[i] = 0.0
            if self._ep_returns:
                window = self._ep_returns[-256:]
                mean_r = float(sum(window) / len(window))
                self.logger.record("rollout/ep_rew_mean", mean_r)
                self.logger.record("rollout/ep_count", len(self._ep_returns))
            return True

    cb = CallbackList([
        CheckpointCallback(
            save_freq=int(raw["checkpoint_every_steps"]) // cfg.num_envs,
            save_path=raw["checkpoint_dir"],
            name_prefix="pretrain_v0",
        ),
        EpisodeRewardCallback(),
    ])

    log.info("starting training: %d total timesteps over %d envs",
             raw["total_timesteps"], cfg.num_envs)
    try:
        model.learn(total_timesteps=int(raw["total_timesteps"]),
                    callback=cb, log_interval=1)
    finally:
        out = Path(raw["checkpoint_dir"]) / "pretrain_v0_final.zip"
        model.save(out)
        log.info("saved final model to %s", out)
        vec_env.close()


if __name__ == "__main__":
    main()
