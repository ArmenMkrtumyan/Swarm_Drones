"""Tiny YAML → HoverEnvConfig + algo-config loader.

Kept separate from the env (which can be constructed code-only for tests) and
from the trainer (which only needs a dict for SB3 kwargs). The conversion is
mechanical but enforces the shape we expect, so a typo in YAML fails fast
instead of silently mistraining.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

from capstone.rl.envs.hover_pid_tuner_v0 import GainSpec, HoverEnvConfig


@dataclass
class AlgoConfig:
    name: str
    kwargs: dict[str, Any]
    action_noise: dict[str, Any]
    total_timesteps: int
    log_dir: str
    checkpoint_dir: str
    checkpoint_every_steps: int
    seed: int


@dataclass
class LoadedConfig:
    env: HoverEnvConfig
    algo: AlgoConfig


def load_config(path: str | Path) -> LoadedConfig:
    p = Path(path)
    raw = yaml.safe_load(p.read_text())

    env_raw = raw["env"]
    gains_raw = raw["gains"]
    gains = [
        GainSpec(
            name=g["name"],
            baseline=float(g["baseline"]),
            lo=float(g["lo"]),
            hi=float(g["hi"]),
            delta_per_step=float(g["delta_per_step"]),
        )
        for g in gains_raw
    ]
    env_cfg = HoverEnvConfig(
        mavlink_url=env_raw["mavlink_url"],
        hover_alt_m=float(env_raw["hover_alt_m"]),
        step_interval_s=float(env_raw["step_interval_s"]),
        max_episode_steps=int(env_raw["max_episode_steps"]),
        settle_timeout_s=float(env_raw["settle_timeout_s"]),
        init_jitter_pct=float(env_raw.get("init_jitter_pct", 0.0)),
        crash_alt_m=float(env_raw["crash_alt_m"]),
        crash_pos_xy_m=float(env_raw["crash_pos_xy_m"]),
        # YAML stores degrees for human edit-ability; env stores radians.
        crash_attitude_rad=math.radians(float(env_raw["crash_attitude_deg"])),
        survival_bonus=float(env_raw["survival_bonus"]),
        crash_penalty=float(env_raw["crash_penalty"]),
        reward_scale=float(env_raw["reward_scale"]),
        base_step_reward=float(env_raw["base_step_reward"]),
        w_alt_err=float(env_raw["w_alt_err"]),
        w_pos_err=float(env_raw["w_pos_err"]),
        w_vel=float(env_raw["w_vel"]),
        w_attitude=float(env_raw["w_attitude"]),
        w_gyro=float(env_raw["w_gyro"]),
        w_action_smooth=float(env_raw["w_action_smooth"]),
        gains=gains,
    )
    algo_cfg = AlgoConfig(
        name=str(raw["algo"]),
        kwargs=dict(raw["algo_kwargs"]),
        action_noise=dict(raw["action_noise"]),
        total_timesteps=int(raw["total_timesteps"]),
        log_dir=str(raw["log_dir"]),
        checkpoint_dir=str(raw["checkpoint_dir"]),
        checkpoint_every_steps=int(raw["checkpoint_every_steps"]),
        seed=int(raw["seed"]),
    )
    return LoadedConfig(env=env_cfg, algo=algo_cfg)
