"""Stage-3 hover RL training entry point.

Run from the .rl_venv with lab/ on PYTHONPATH:

    cd Swarm_Drones
    lab/rl/.rl_venv/Scripts/python.exe -m lab.rl.train_hover \
        --config lab/rl/cfg/hover_v0.yaml

Prereqs (set up by the user once before training):
  1. ArduPilot SITL launched with an extra MAVLink endpoint on port 14552:
        sim_vehicle.py -v ArduCopter -f X -N -w \\
          --add-param-file=.../sitl/params.parm \\
          -A '--home 40.192,44.50446,1200,0' \\
          --model JSON:192.168.208.1 \\
          --map --console \\
          --out=udp:127.0.0.1:14551 \\
          --out=udp:127.0.0.1:14552      <-- this is the new one
  2. Isaac Sim running with the bridge connected to SITL (the usual capstone
     setup — F450 USD, AUA scene).
  3. SR1_* MAVLink streams configured so ATTITUDE / LOCAL_POSITION_NED arrive
     at >=10 Hz.

The trainer connects to 14552, so the bridge (JSON FDM, separate protocol)
and arm_hover.py (UDP 14551) are unaffected.
"""

from __future__ import annotations

import argparse
import logging
import os
from pathlib import Path

import numpy as np

from lab.rl.config_loader import LoadedConfig, load_config
from lab.rl.envs.hover_pid_tuner_v0 import HoverPidTunerEnv


log = logging.getLogger("lab.rl.train")


def build_env(cfg: LoadedConfig):
    env = HoverPidTunerEnv(cfg=cfg.env)
    return env


def build_action_noise(cfg: LoadedConfig, action_dim: int):
    """Translate the YAML action_noise block into an SB3 ActionNoise."""
    from stable_baselines3.common.noise import (
        NormalActionNoise,
        OrnsteinUhlenbeckActionNoise,
    )

    spec = cfg.algo.action_noise
    sigma = float(spec.get("sigma", 0.1))
    mean = np.zeros(action_dim, dtype=np.float32)
    std = np.full(action_dim, sigma, dtype=np.float32)
    kind = str(spec.get("type", "normal")).lower()
    if kind == "normal":
        return NormalActionNoise(mean=mean, sigma=std)
    if kind in ("ou", "ornstein_uhlenbeck"):
        return OrnsteinUhlenbeckActionNoise(mean=mean, sigma=std)
    raise ValueError(f"unknown action_noise.type={kind}")


def build_model(cfg: LoadedConfig, env):
    from stable_baselines3 import PPO, SAC, TD3, DDPG

    name = cfg.algo.name.upper()
    table = {"TD3": TD3, "SAC": SAC, "PPO": PPO, "DDPG": DDPG}
    if name not in table:
        raise ValueError(f"unsupported algo {name}; pick from {list(table)}")
    AlgoClass = table[name]

    kwargs = dict(cfg.algo.kwargs)
    if name in ("TD3", "DDPG"):
        kwargs["action_noise"] = build_action_noise(cfg, env.action_space.shape[0])
    # PPO / SAC don't take action_noise.

    return AlgoClass(
        "MlpPolicy",
        env,
        seed=cfg.algo.seed,
        tensorboard_log=cfg.algo.log_dir,
        **kwargs,
    )


def main() -> None:
    ap = argparse.ArgumentParser(description="Train HoverPidTuner-v0.")
    ap.add_argument("--config", required=True, type=str,
                    help="path to YAML config (e.g. lab/rl/cfg/hover_v0.yaml)")
    ap.add_argument("--resume", type=str, default=None,
                    help="path to a saved SB3 model .zip to resume from")
    ap.add_argument("--log-level", default="INFO")
    args = ap.parse_args()

    logging.basicConfig(level=args.log_level.upper(), format="%(asctime)s [%(levelname)s] %(name)s: %(message)s")

    cfg = load_config(args.config)
    Path(cfg.algo.log_dir).mkdir(parents=True, exist_ok=True)
    Path(cfg.algo.checkpoint_dir).mkdir(parents=True, exist_ok=True)

    env = build_env(cfg)

    from stable_baselines3.common.callbacks import CheckpointCallback

    if args.resume:
        log.info("resuming from %s", args.resume)
        from stable_baselines3 import PPO, SAC, TD3, DDPG  # noqa
        AlgoClass = {"TD3": TD3, "SAC": SAC, "PPO": PPO, "DDPG": DDPG}[cfg.algo.name.upper()]
        # Override tensorboard_log so the resumed run logs to THIS config's
        # log_dir (not whatever path was baked in when the saved model was
        # created — typically Phase 1's pretrain_v0 dir).
        model = AlgoClass.load(
            args.resume, env=env,
            tensorboard_log=cfg.algo.log_dir,
        )
    else:
        model = build_model(cfg, env)

    cb = CheckpointCallback(
        save_freq=cfg.algo.checkpoint_every_steps,
        save_path=cfg.algo.checkpoint_dir,
        name_prefix="hover_v0",
    )

    log.info("starting training: %d timesteps", cfg.algo.total_timesteps)
    try:
        model.learn(total_timesteps=cfg.algo.total_timesteps, callback=cb,
                    log_interval=10)
    finally:
        final_path = Path(cfg.algo.checkpoint_dir) / "hover_v0_final.zip"
        model.save(final_path)
        log.info("saved final model to %s", final_path)
        env.close()


if __name__ == "__main__":
    main()
