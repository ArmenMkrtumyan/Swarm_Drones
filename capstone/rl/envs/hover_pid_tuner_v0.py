"""HoverPidTuner-v0 — Gymnasium environment for adaptive ATC PID tuning.

Architecture follows RLDroneSim (Ghazaryan/Arzanyan/Madoyan, AUA): the RL agent
sits *above* a real ArduPilot SITL flight stack. The agent observes telemetry,
emits incremental deltas on a configured set of PID gains every step, and pushes
them into ArduPilot via MAVLink ``PARAM_SET``. ArduPilot keeps doing the
low-level control. We never bypass it.

This is v0 — the action set is the attitude-loop gains (ATC_RAT_*_P/I/D and
ATC_ANG_*_P, roll+pitch), since that is where the F450 wheelbase fix changed
the dynamics most. Yaw and position gains can be added later by editing the
config YAML.

Connection model
----------------
SITL must be launched with an extra MAVLink endpoint for this env, on top of the
14551 endpoint that ``arm_hover.py`` uses. Add to the user's normal launch line::

    --out=udp:127.0.0.1:14552

We bind ``udpin:localhost:14552``. The bridge's JSON-FDM connection to Isaac
Sim is unrelated — this env doesn't talk to Isaac directly.

Reset semantics (v0, intentionally simple)
------------------------------------------
A clean SITL reset would need the Lua reset script that RLDroneSim adds. We
don't have that yet, so each ``reset()`` here:

  1. Disarms (or no-op if already disarmed).
  2. Restores the *baseline* gains from config (so episodes start IID).
  3. Optionally jitters the starting gains by ``init_jitter_pct`` for diversity.
  4. Switches to GUIDED, arms, takes off to ``hover_alt_m``.
  5. Waits for altitude-and-velocity settle.

This costs ~30-60 s wall per reset. v1 should add a Lua reset script.
"""

from __future__ import annotations

import logging
import math
import time
from dataclasses import dataclass, field
from typing import Any

import numpy as np
from pymavlink import mavutil

try:
    import gymnasium as gym
    from gymnasium import spaces
except ImportError as e:
    raise ImportError(
        "gymnasium is required for HoverPidTunerEnv. "
        "Install via the .rl_venv: pip install gymnasium"
    ) from e


log = logging.getLogger(__name__)


# -----------------------------------------------------------------------------
# Config dataclass — mirrors hover_v0.yaml, but enforces types and defaults so
# the env can also be constructed in a unit test without touching disk.
# -----------------------------------------------------------------------------
@dataclass
class GainSpec:
    """One ATC/PSC gain we let the policy modulate."""
    name: str            # MAVLink param name, e.g. "ATC_RAT_PIT_P"
    baseline: float      # default value applied at reset
    lo: float            # absolute lower clamp (safety net)
    hi: float            # absolute upper clamp (safety net)
    delta_per_step: float  # |action| of 1.0 maps to this absolute change


@dataclass
class HoverEnvConfig:
    mavlink_url: str = "udpin:localhost:14552"
    hover_alt_m: float = 3.0
    step_interval_s: float = 0.5         # sim seconds per env step
    max_episode_steps: int = 120         # 60 s of hover at 0.5 s/step
    settle_timeout_s: float = 30.0       # wall-clock cap on takeoff settle
    init_jitter_pct: float = 0.0         # 0 = always use exact baseline
    crash_alt_m: float = 0.5             # below this AGL → crash
    crash_pos_xy_m: float = 8.0          # >this from home XY → out-of-bounds
    crash_attitude_rad: float = math.radians(60.0)
    survival_bonus: float = 0.0          # +reward at clean episode end
    crash_penalty: float = -50.0         # one-shot reward on crash
    reward_scale: float = 1.0
    # Per-step reward weights (sum into negative cost; reward = base - cost):
    base_step_reward: float = 1.0        # nudges policy to keep the drone alive
    w_alt_err: float = 1.0
    w_pos_err: float = 0.5
    w_vel: float = 0.05
    w_attitude: float = 0.5
    w_gyro: float = 0.05
    w_action_smooth: float = 0.01        # discourages thrashing
    gains: list[GainSpec] = field(default_factory=list)


# Default action set for v0 — the attitude-loop gains. PSC/yaw can be added
# via config without touching the env code.
DEFAULT_GAINS: list[GainSpec] = [
    # Outer angle P
    GainSpec("ATC_ANG_RLL_P", 4.5, 1.0, 12.0, delta_per_step=0.20),
    GainSpec("ATC_ANG_PIT_P", 4.5, 1.0, 12.0, delta_per_step=0.20),
    # Inner rate P
    GainSpec("ATC_RAT_RLL_P", 0.135, 0.02, 0.50, delta_per_step=0.010),
    GainSpec("ATC_RAT_PIT_P", 0.135, 0.02, 0.50, delta_per_step=0.010),
    # Inner rate I
    GainSpec("ATC_RAT_RLL_I", 0.135, 0.01, 0.50, delta_per_step=0.010),
    GainSpec("ATC_RAT_PIT_I", 0.135, 0.01, 0.50, delta_per_step=0.010),
    # Inner rate D
    GainSpec("ATC_RAT_RLL_D", 0.0036, 0.0, 0.05, delta_per_step=0.0010),
    GainSpec("ATC_RAT_PIT_D", 0.0036, 0.0, 0.05, delta_per_step=0.0010),
]


# -----------------------------------------------------------------------------
# Env
# -----------------------------------------------------------------------------
class HoverPidTunerEnv(gym.Env):
    """Adaptive PID-tuning hover env. See module docstring for architecture."""

    metadata = {"render_modes": []}

    def __init__(self, cfg: HoverEnvConfig | None = None):
        self.cfg = cfg or HoverEnvConfig(gains=list(DEFAULT_GAINS))
        if not self.cfg.gains:
            self.cfg.gains = list(DEFAULT_GAINS)

        # Observation: per-gain normalized values (n_gains) +
        #   [alt_err, pos_n, pos_e, vel_n, vel_e, vel_d,
        #    roll, pitch, gx, gy, gz]  (11 telemetry dims)
        n_g = len(self.cfg.gains)
        obs_dim = n_g + 11
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(obs_dim,), dtype=np.float32
        )
        # Action: per-gain delta in [-1, 1]. Scaled to gain.delta_per_step.
        self.action_space = spaces.Box(
            low=-1.0, high=1.0, shape=(n_g,), dtype=np.float32
        )

        self._mav: mavutil.mavfile | None = None
        self._home_xy_m: tuple[float, float] | None = None
        self._step_count = 0
        self._last_action = np.zeros(n_g, dtype=np.float32)
        self._current_gains = np.array(
            [g.baseline for g in self.cfg.gains], dtype=np.float64
        )

    # ---------------------------------------------------------------- mavlink

    def _ensure_mav(self) -> mavutil.mavfile:
        if self._mav is None:
            log.info("connecting MAVLink at %s", self.cfg.mavlink_url)
            self._mav = mavutil.mavlink_connection(self.cfg.mavlink_url)
            self._mav.wait_heartbeat(timeout=15)
            log.info("heartbeat received from sys=%d comp=%d",
                     self._mav.target_system, self._mav.target_component)
        return self._mav

    def _send_param(self, name: str, value: float) -> None:
        m = self._ensure_mav()
        m.mav.param_set_send(
            m.target_system, m.target_component,
            name.encode("ascii"),
            float(value),
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
        )

    def _read_telemetry(self, timeout_s: float = 0.2) -> dict[str, Any] | None:
        """Drain pending MAVLink and return the latest available telemetry.

        We pull the most recent ATTITUDE + LOCAL_POSITION_NED + RAW_IMU. We
        don't request streams here -- arm_hover.py / the user's SITL params
        already configure SR1_*; if they aren't streaming we'll time out and
        the caller can decide what to do.
        """
        m = self._ensure_mav()
        snap: dict[str, Any] = {}
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            msg = m.recv_match(blocking=False)
            if msg is None:
                time.sleep(0.005)
                continue
            t = msg.get_type()
            if t == "ATTITUDE":
                snap["attitude"] = msg
            elif t == "LOCAL_POSITION_NED":
                snap["local_pos"] = msg
            elif t == "RAW_IMU":
                snap["raw_imu"] = msg
            elif t == "VFR_HUD":
                snap["vfr"] = msg
            # Bail out early once we have everything we need.
            if {"attitude", "local_pos"}.issubset(snap):
                break
        return snap or None

    # ----------------------------------------------------------------- reset

    def _set_mode(self, mode: str) -> None:
        m = self._ensure_mav()
        mode_id = m.mode_mapping().get(mode)
        if mode_id is None:
            raise RuntimeError(f"mode {mode} not in mode map")
        m.mav.set_mode_send(
            m.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id,
        )

    def _arm(self, force: bool = False) -> None:
        m = self._ensure_mav()
        m.mav.command_long_send(
            m.target_system, m.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,                # arm
            21196 if force else 0,
            0, 0, 0, 0, 0,
        )

    def _disarm(self, force: bool = False) -> None:
        m = self._ensure_mav()
        m.mav.command_long_send(
            m.target_system, m.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0,                # disarm
            21196 if force else 0,
            0, 0, 0, 0, 0,
        )

    def _takeoff(self, alt_m: float) -> None:
        m = self._ensure_mav()
        m.mav.command_long_send(
            m.target_system, m.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, float(alt_m),
        )

    def _restore_baseline_gains(self, rng: np.random.Generator) -> None:
        for i, g in enumerate(self.cfg.gains):
            jitter = 0.0
            if self.cfg.init_jitter_pct > 0:
                jitter = rng.uniform(
                    -self.cfg.init_jitter_pct, self.cfg.init_jitter_pct,
                ) * g.baseline
            v = float(np.clip(g.baseline + jitter, g.lo, g.hi))
            self._current_gains[i] = v
            self._send_param(g.name, v)

    def reset(self, *, seed: int | None = None, options: dict | None = None,
              ) -> tuple[np.ndarray, dict[str, Any]]:
        super().reset(seed=seed)
        rng = np.random.default_rng(seed)
        self._step_count = 0
        self._last_action[:] = 0.0
        self._home_xy_m = None

        self._ensure_mav()
        try:
            self._disarm(force=True)
        except Exception:
            pass  # tolerated: probably already disarmed
        time.sleep(1.0)

        self._restore_baseline_gains(rng)
        time.sleep(0.5)

        self._set_mode("GUIDED")
        time.sleep(0.5)
        self._arm()
        time.sleep(2.0)
        self._takeoff(self.cfg.hover_alt_m)

        # Wait for altitude-and-velocity settle (or timeout). If reset is
        # broken (e.g., ArduPilot rejected arm after a crash), this just
        # times out and the next episode starts garbage. User-supervised
        # workflow: stop Isaac, restart Isaac, episodes recover.
        deadline = time.time() + self.cfg.settle_timeout_s
        last_snap = None
        while time.time() < deadline:
            snap = self._read_telemetry(timeout_s=0.2)
            if snap and "local_pos" in snap and "attitude" in snap:
                last_snap = snap
                lp = snap["local_pos"]
                if (-lp.z) >= self.cfg.hover_alt_m - 0.2 and \
                   abs(lp.vz) < 0.2 and \
                   math.hypot(lp.vx, lp.vy) < 0.3:
                    break
            time.sleep(0.1)
        if last_snap is None:
            raise RuntimeError("no telemetry during reset settle")

        lp = last_snap["local_pos"]
        self._home_xy_m = (float(lp.x), float(lp.y))
        obs = self._observe(last_snap)
        return obs, {"settled": True}

    # ------------------------------------------------------------------ step

    def _apply_action(self, action: np.ndarray) -> None:
        action = np.clip(action, -1.0, 1.0).astype(np.float64)
        for i, g in enumerate(self.cfg.gains):
            new_val = float(np.clip(
                self._current_gains[i] + action[i] * g.delta_per_step,
                g.lo, g.hi,
            ))
            if new_val != self._current_gains[i]:
                self._current_gains[i] = new_val
                self._send_param(g.name, new_val)

    def _observe(self, snap: dict[str, Any]) -> np.ndarray:
        lp = snap["local_pos"]
        att = snap["attitude"]
        home = self._home_xy_m or (0.0, 0.0)
        # Per-gain values normalized to [0, 1] over (lo, hi).
        gains_norm = np.array([
            (self._current_gains[i] - g.lo) / max(g.hi - g.lo, 1e-9)
            for i, g in enumerate(self.cfg.gains)
        ], dtype=np.float32)
        alt_err = (-float(lp.z)) - self.cfg.hover_alt_m
        pos_n = float(lp.x) - home[0]
        pos_e = float(lp.y) - home[1]
        vel_n = float(lp.vx)
        vel_e = float(lp.vy)
        vel_d = float(lp.vz)
        roll = float(att.roll)
        pitch = float(att.pitch)
        gx = float(att.rollspeed)
        gy = float(att.pitchspeed)
        gz = float(att.yawspeed)
        telem = np.array(
            [alt_err, pos_n, pos_e, vel_n, vel_e, vel_d,
             roll, pitch, gx, gy, gz],
            dtype=np.float32,
        )
        return np.concatenate([gains_norm, telem]).astype(np.float32)

    def _reward(self, snap: dict[str, Any], action: np.ndarray,
                ) -> tuple[float, dict[str, float]]:
        c = self.cfg
        lp = snap["local_pos"]
        att = snap["attitude"]
        home = self._home_xy_m or (0.0, 0.0)
        alt_err = abs((-float(lp.z)) - c.hover_alt_m)
        pos_err = math.hypot(float(lp.x) - home[0], float(lp.y) - home[1])
        vel = math.hypot(float(lp.vx), float(lp.vy)) + abs(float(lp.vz))
        attitude = math.hypot(float(att.roll), float(att.pitch))
        gyro = math.hypot(float(att.rollspeed), float(att.pitchspeed))
        smooth = float(np.linalg.norm(action - self._last_action))
        cost = (c.w_alt_err   * alt_err
                + c.w_pos_err  * pos_err
                + c.w_vel      * vel
                + c.w_attitude * attitude
                + c.w_gyro     * gyro
                + c.w_action_smooth * smooth)
        r = (c.base_step_reward - cost) * c.reward_scale
        return r, {
            "alt_err": alt_err, "pos_err": pos_err, "vel": vel,
            "attitude": attitude, "gyro": gyro, "smooth": smooth,
            "cost": cost,
        }

    def _check_done(self, snap: dict[str, Any]) -> tuple[bool, bool, str]:
        c = self.cfg
        lp = snap["local_pos"]
        att = snap["attitude"]
        home = self._home_xy_m or (0.0, 0.0)
        alt = -float(lp.z)
        if alt < c.crash_alt_m:
            return True, False, "crashed_low_alt"
        if math.hypot(float(lp.x) - home[0], float(lp.y) - home[1]) > c.crash_pos_xy_m:
            return True, False, "out_of_bounds"
        if abs(float(att.roll)) > c.crash_attitude_rad or \
           abs(float(att.pitch)) > c.crash_attitude_rad:
            return True, False, "tipped"
        if self._step_count >= c.max_episode_steps:
            # Timeout = truncated, not terminated.
            return False, True, "timeout"
        return False, False, ""

    def step(self, action: np.ndarray,
             ) -> tuple[np.ndarray, float, bool, bool, dict[str, Any]]:
        action = np.asarray(action, dtype=np.float32)
        self._apply_action(action)

        # Sleep for one sim step. SITL is real-time, so wall == sim.
        time.sleep(self.cfg.step_interval_s)

        snap = self._read_telemetry(timeout_s=0.5)
        if snap is None or "local_pos" not in snap or "attitude" not in snap:
            # Treat telemetry stall as a crash; we have no usable obs.
            obs = np.zeros(self.observation_space.shape, dtype=np.float32)
            return obs, self.cfg.crash_penalty, True, False, {"reason": "telemetry_stall"}

        self._step_count += 1
        terminated, truncated, reason = self._check_done(snap)
        reward, info = self._reward(snap, action)
        if terminated:
            reward += self.cfg.crash_penalty
        elif truncated:
            reward += self.cfg.survival_bonus
        info["reason"] = reason
        info["step"] = self._step_count
        info["gains"] = self._current_gains.copy()
        self._last_action = action.copy()
        obs = self._observe(snap)
        return obs, float(reward), terminated, truncated, info

    def close(self) -> None:
        if self._mav is not None:
            try:
                self._mav.close()
            except Exception:
                pass
            self._mav = None
