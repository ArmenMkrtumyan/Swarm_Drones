# lab.rl — Stage 3 (RL hover)

Adaptive ATC PID tuner riding on top of ArduPilot SITL. Architecture follows
RLDroneSim (Ghazaryan/Arzanyan/Madoyan, AUA): the agent observes telemetry via
MAVLink and writes back gain deltas via `PARAM_SET`. ArduPilot keeps doing the
low-level control. Isaac Sim handles physics through the existing bridge.

## Layout

```
lab/rl/
├── .rl_venv/                  windows venv, gitignored
├── cfg/hover_v0.yaml          training config (env + algo + gains)
├── envs/hover_pid_tuner_v0.py Gymnasium env that wraps MAVLink
├── config_loader.py           yaml → dataclasses
├── train_hover.py             SB3 launcher
├── requirements.txt           pip freeze of .rl_venv
└── README.md                  this file
```

## One-time setup

```powershell
# create venv + install deps (already done if you can see this file)
cd C:\Users\user1811\Desktop\armen-capstone\Swarm_Drones\capstone\rl
python -m venv .rl_venv
.rl_venv\Scripts\python.exe -m pip install -r requirements.txt
```

## Per-run setup

Three processes, in this order:

1. **ArduPilot SITL** in WSL with an extra `--out` for the RL agent:

```bash
cd /mnt/c/Users/user1811/Desktop/armen-capstone/ardupilot
Tools/autotest/sim_vehicle.py -v ArduCopter -f X -N -w \
  --add-param-file=/mnt/c/Users/user1811/Desktop/armen-capstone/Swarm_Drones/sitl/params.parm \
  -A '--home 40.192,44.50446,1200,0' \
  --model JSON:192.168.208.1 \
  --map --console \
  --out=udp:127.0.0.1:14551 \
  --out=udp:127.0.0.1:14552
```

The new line is `--out=udp:127.0.0.1:14552`. 14551 stays available for
`arm_hover.py` and any manual MAVLink tooling.

2. **Isaac Sim** with `AUA_world_500m.usd` + the bridge, the same as your hover
   benchmark workflow.

3. **Trainer** in a Windows PowerShell at the project root:

```powershell
cd C:\Users\user1811\Desktop\armen-capstone\Swarm_Drones
$env:PYTHONPATH = "$PWD"
capstone\rl\.rl_venv\Scripts\python.exe -m lab.rl.train_hover `
  --config capstone\rl\cfg\hover_v0.yaml
```

TensorBoard:

```powershell
capstone\rl\.rl_venv\Scripts\python.exe -m tensorboard.main --logdir logs\tb_logs
```

Open http://localhost:6006 in a browser.

## Action / observation / reward (v0)

* **Action (8 dims)** — bounded `[-1, 1]` per gain, scaled by `delta_per_step`.
  Default action set is the attitude inner loop:
  `ATC_ANG_*_P`, `ATC_RAT_*_P`, `ATC_RAT_*_I`, `ATC_RAT_*_D` for roll and pitch.
  Yaw and PSC are intentionally fixed in v0; add to YAML to extend.
* **Observation (19 dims)** — `[8 normalized gains, alt_err, pos_n, pos_e,
  vel_n, vel_e, vel_d, roll, pitch, gx, gy, gz]`.
* **Reward** — per step: `base_step_reward − Σ wᵢ · errᵢ` over altitude, xy
  position, velocity magnitude, attitude angle, gyro, and action smoothness.
  Crash → `crash_penalty` (one-shot). Episode timeout (120 steps = 60 s) ends
  truncated, not terminated, so SB3 bootstraps from the final value.

## Why these defaults

* `step_interval_s = 0.5` — pushes a `PARAM_SET` every 0.5 sim seconds so
  ArduPilot has time to settle into the new gain before we measure. Don't go
  below ~0.2 s without rate-limiting.
* `total_timesteps = 50000` — at ~1 step/wall-second, that's ~14 hours. Start
  with ~5000 to verify the loop is sound, then scale.
* `init_jitter_pct = 0.0` — start without resetting to varied gains, pivot to
  0.1 once basic learning is observed (jitter improves robustness but slows
  early reward).

## Next iterations

* v1: Lua reset script in ArduPilot SITL (per RLDroneSim) so resets are
  deterministic and don't require land/disarm/takeoff cycles.
* v1: domain randomization via the existing `lab.control.disturbance`
  profiles — wind, mass drop, IMU noise — randomized per episode.
* v2: PSC gains in the action set after attitude tuning converges.
* v2: parallel SITL+Isaac instances for sample throughput.
