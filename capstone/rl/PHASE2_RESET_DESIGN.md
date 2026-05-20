# Phase-2 SITL fine-tune: hands-off reset architecture

Three layers, in order of leverage. Each layer reduces dependence on the next.

```
Layer 1 (most leverage):  Crash avoidance  → policy almost never crashes
Layer 2 (cheap path):     Soft reset       → ~10s, no process restart
Layer 3 (backstop):       Crash recovery   → ~20s, bridge teleport, rare
```

Total target: **<90 s/episode end-to-end**, fully unattended for hours.

---

## Layer 1 — Crash avoidance

Diffs to `capstone/rl/envs/hover_pid_tuner_v0.py`.

### 1a. Add `SoftLimitConfig` to `HoverEnvConfig`

```python
@dataclass
class HoverEnvConfig:
    # ... existing fields ...

    # Soft-terminate thresholds. Episode ends, drone is still flyable.
    soft_alt_err_m:        float = 2.0
    soft_attitude_rad:     float = math.radians(30.0)
    soft_xy_m:             float = 5.0
    soft_gyro_rad_s:       float = math.radians(120.0)
    soft_terminate_penalty: float = -10.0   # reward, additive

    # Hard-crash thresholds (kept as backstop).
    crash_alt_m:           float = 0.5
    crash_pos_xy_m:        float = 50.0
    crash_attitude_rad:    float = math.radians(60.0)
    crash_penalty:         float = -50.0

    # Safety override: when ANY soft threshold trips, instead of letting the
    # policy keep acting, snap gains to baseline and switch to LOITER for the
    # remainder of the reset cycle.
    safety_override_enabled: bool = True
```

### 1b. Replace `_check_done` to distinguish soft / crash / truncated

```python
def _check_done(self, snap):
    c = self.cfg
    lp, att = snap["local_pos"], snap["attitude"]
    home = self._home_xy_m or (0.0, 0.0)
    alt   = -float(lp.z)
    xy    = math.hypot(float(lp.x) - home[0], float(lp.y) - home[1])
    roll  = abs(float(att.roll))
    pitch = abs(float(att.pitch))
    gyro  = math.hypot(float(att.rollspeed), float(att.pitchspeed))

    # Hard crash (rare with soft limits in place).
    if alt < c.crash_alt_m:                                      return "crashed",      "low_alt"
    if xy  > c.crash_pos_xy_m:                                   return "crashed",      "out_of_bounds_hard"
    if roll  > c.crash_attitude_rad or pitch > c.crash_attitude_rad: return "crashed",  "tipped"

    # Soft terminate (drone still flyable).
    if abs(alt - c.hover_alt_m) > c.soft_alt_err_m:              return "soft_term",    "alt_drift"
    if xy > c.soft_xy_m:                                         return "soft_term",    "xy_drift"
    if roll > c.soft_attitude_rad or pitch > c.soft_attitude_rad: return "soft_term",   "tilt"
    if gyro > c.soft_gyro_rad_s:                                 return "soft_term",    "spin"

    if self._step_count >= c.max_episode_steps:                  return "truncated",    "timeout"
    return "ongoing", ""
```

### 1c. Update `step()` to invoke safety override

```python
def step(self, action):
    action = np.asarray(action, dtype=np.float32)
    self._apply_action(action)
    time.sleep(self.cfg.step_interval_s)
    snap = self._read_telemetry(timeout_s=0.5)
    if snap is None or "local_pos" not in snap or "attitude" not in snap:
        return self._observation_zero(), self.cfg.crash_penalty, True, False, {"reason": "telemetry_stall"}

    self._step_count += 1
    state, reason = self._check_done(snap)
    reward, info = self._reward(snap, action)

    terminated, truncated = False, False
    if state == "crashed":
        reward += self.cfg.crash_penalty
        terminated = True
    elif state == "soft_term":
        reward += self.cfg.soft_terminate_penalty
        terminated = True
        if self.cfg.safety_override_enabled:
            self._safety_override()        # snap gains, LOITER, give the FC time
    elif state == "truncated":
        reward += self.cfg.survival_bonus
        truncated = True

    info["reason"], info["step"] = reason, self._step_count
    info["gains"] = self._current_gains.copy()
    info["state"] = state
    self._last_action = action.copy()
    return self._observe(snap), float(reward), terminated, truncated, info


def _safety_override(self):
    """Stabilize the drone after a soft-terminate so reset() can take off normally."""
    for i, g in enumerate(self.cfg.gains):
        self._send_param(g.name, g.baseline)
        self._current_gains[i] = g.baseline
    try:
        self._set_mode("LOITER")
    except Exception:
        pass
    time.sleep(2.0)            # let LOITER stabilize
```

---

## Layer 2 — Soft reset (no disarm)

Replace today's "disarm → arm → takeoff" cycle with "MAV_CMD_DO_REPOSITION home + restore gains."

### 2a. New `_reposition_home` helper

```python
def _reposition_home(self):
    """In-flight reposition to the takeoff point. ~3-5 s wall."""
    m = self._ensure_mav()
    self._set_mode("GUIDED")
    home_n, home_e = self._home_xy_m or (0.0, 0.0)
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_DO_REPOSITION,
        0,
        -1,                    # default ground speed
        1,                     # MAV_DO_REPOSITION_FLAGS_CHANGE_MODE
        0, float("nan"),       # radius / yaw (unset)
        home_n, home_e,        # NED in current frame; needs home_position locked
        self.cfg.hover_alt_m,
    )
```

### 2b. Refactor `reset()`

```python
def reset(self, *, seed=None, options=None):
    super().reset(seed=seed)
    rng = np.random.default_rng(seed)
    self._step_count = 0
    self._last_action[:] = 0.0
    self._ensure_mav()

    fast_path = options is not None and options.get("fast_reset", False)

    if fast_path:
        # Soft reset: drone is in the air, just reposition + reset gains.
        self._restore_baseline_gains(rng)
        self._reposition_home()
        self._wait_settle(timeout_s=10.0)        # tighter than 30s cold start
    else:
        # Cold start: full disarm/arm/takeoff. Used at the very first reset
        # and after Layer-3 crash recovery.
        try: self._disarm(force=True)
        except Exception: pass
        time.sleep(1.0)
        self._restore_baseline_gains(rng)
        time.sleep(0.5)
        self._set_mode("GUIDED")
        time.sleep(0.5)
        self._arm()
        time.sleep(2.0)
        self._takeoff(self.cfg.hover_alt_m)
        self._wait_settle(timeout_s=self.cfg.settle_timeout_s)

    snap = self._read_telemetry(timeout_s=1.0)
    lp = snap["local_pos"]
    self._home_xy_m = (float(lp.x), float(lp.y)) if not fast_path else self._home_xy_m
    return self._observe(snap), {"settled": True, "fast_reset": fast_path}
```

### 2c. Wire the trainer to use `fast_reset`

```python
# in train_hover.py — after each model.learn() episode end
obs, info = env.reset(options={"fast_reset": last_state != "crashed"})
```

**Result:** soft reset ~10 s, cold reset ~30 s. Crashes only trigger cold resets.

---

## Layer 3 — Crash recovery via bridge teleport

When a hard crash happens (after Layer-1 mitigations, this should be <5% of episodes), recover without restarting any process.

### 3a. New module `capstone/rl/bridge_client.py`

A thin TCP/UDP client to send teleport commands to the Isaac bridge. The bridge already opens a JSON-FDM port for motor commands; add a second port for control commands.

```python
import json, socket
from dataclasses import dataclass

@dataclass
class BridgeClient:
    host: str = "127.0.0.1"
    port: int = 14557                    # new port for control RPCs

    def _send(self, payload: dict) -> dict:
        s = socket.create_connection((self.host, self.port), timeout=5.0)
        s.sendall((json.dumps(payload) + "\n").encode())
        resp = s.recv(4096)
        s.close()
        return json.loads(resp.decode())

    def teleport_drone(self, pos_xyz, vel_xyz=(0,0,0),
                       quat_wxyz=(1,0,0,0)):
        return self._send({
            "cmd": "teleport_drone",
            "position": list(pos_xyz),
            "velocity": list(vel_xyz),
            "quat":     list(quat_wxyz),
        })

    def pause(self):  return self._send({"cmd": "pause"})
    def play(self):   return self._send({"cmd": "play"})
```

### 3b. Bridge-side patch (Isaac process, ~30 lines)

Add a control listener thread to whatever bridge file already runs in Isaac (`Nvidia_SITL_connecter.py` or its successor). Cache the rigid-body prim handle once at startup, then handle JSON commands:

```python
# pseudo-code; actual API names depend on your IsaacSim version
import omni.timeline
from pxr import Gf
from omni.isaac.dynamic_control import _dynamic_control as dc

class BridgeControl:
    def __init__(self, drone_prim_path, motor_prim_paths, port=14557):
        self._dc = dc.acquire_dynamic_control_interface()
        self._drone = self._dc.get_rigid_body(drone_prim_path)
        self._motors = [self._dc.get_rigid_body(p) for p in motor_prim_paths]
        threading.Thread(target=self._serve, args=(port,), daemon=True).start()

    def teleport_drone(self, pos, vel, quat_wxyz):
        omni.timeline.get_timeline_interface().pause()
        # write rigid-body state directly into PhysX
        pose = dc.Transform()
        pose.p = pos
        pose.r = (quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0])  # xyzw
        self._dc.set_rigid_body_pose(self._drone, pose)
        self._dc.set_rigid_body_linear_velocity(self._drone, vel)
        self._dc.set_rigid_body_angular_velocity(self._drone, (0, 0, 0))
        # zero motor omegas so spin-up restarts cleanly
        for m in self._motors:
            self._dc.set_rigid_body_angular_velocity(m, (0, 0, 0))
        omni.timeline.get_timeline_interface().play()
```

### 3c. New module `capstone/rl/crash_recovery.py`

```python
import logging, time, math
from pymavlink import mavutil
from capstone.rl.bridge_client import BridgeClient

log = logging.getLogger(__name__)

class CrashRecovery:
    """Bridge teleport + EKF settle. Called by the env after a hard crash."""

    def __init__(self, bridge: BridgeClient, mav: mavutil.mavfile,
                 home_xyz=(0.0, 0.0, 3.0)):
        self.bridge = bridge
        self.mav = mav
        self.home_xyz = home_xyz

    def run(self):
        log.warning("crash recovery: teleporting drone home")
        self.mav.mav.set_mode_send(self.mav.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 0)   # STABILIZE
        time.sleep(0.5)
        self.bridge.teleport_drone(self.home_xyz, (0,0,0), (1,0,0,0))
        # Force EKF re-seed via gyro/accel calibration
        self.mav.mav.command_long_send(
            self.mav.target_system, self.mav.target_component,
            mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
            0, 1, 0, 0, 0, 0, 0, 0,                # gyro cal
        )
        time.sleep(3.0)                            # let EKF settle with new pose
        log.info("crash recovery complete")
```

### 3d. Wire it in the env

```python
# hover_pid_tuner_v0.py reset() — add fallback path
def reset(self, *, seed=None, options=None):
    ...
    if options and options.get("after_crash"):
        self._crash_recovery.run()                 # bridge teleport + EKF settle
        # then fall through to cold-start path

# in the trainer
last_state = info.get("state", "ongoing")
need_recovery = (last_state == "crashed")
obs, info = env.reset(options={
    "fast_reset":  not need_recovery,
    "after_crash": need_recovery,
})
```

---

## Optional Layer 4 — ArduPilot Lua reset script (sub-5 s resets)

Drop into `ArduPilot/scripts/reset_pids.lua`. Triggered by `MAV_CMD_USER_1`. Inside the FC, so reset is instant — no MAVLink param-set round-trip.

```lua
-- reset_pids.lua — restore the 8 ATC PIDs to baseline on demand
local BASELINES = {
    ATC_ANG_RLL_P = 4.5,    ATC_ANG_PIT_P = 4.5,
    ATC_RAT_RLL_P = 0.135,  ATC_RAT_PIT_P = 0.135,
    ATC_RAT_RLL_I = 0.135,  ATC_RAT_PIT_I = 0.135,
    ATC_RAT_RLL_D = 0.0036, ATC_RAT_PIT_D = 0.0036,
}
function on_user1_cmd(p1, p2, p3, p4)
    for name, val in pairs(BASELINES) do
        param:set(name, val)
    end
    gcs:send_text(6, "RL: PIDs reset to baseline")
    return MAV_RESULT_ACCEPTED
end
return on_user1_cmd
```

Then in the env: `command_long_send(MAV_CMD_USER_1, ...)` instead of looping `param_set_send` 8 times. Saves ~2 s per reset.

---

## Implementation order

| # | Change | File | Lines |
|---|---|---|---|
| 1 | `SoftLimitConfig` + `_check_done` + safety override | `hover_pid_tuner_v0.py` | ~50 |
| 2 | `_reposition_home` + `fast_reset` path | `hover_pid_tuner_v0.py` | ~40 |
| 3 | `BridgeClient` (env-side) | new `bridge_client.py` | ~40 |
| 4 | Bridge control listener (Isaac-side) | existing bridge file | ~50 |
| 5 | `CrashRecovery` | new `crash_recovery.py` | ~40 |
| 6 | Wire `after_crash` reset path | `hover_pid_tuner_v0.py` + trainer | ~20 |
| 7 | (Optional) Lua reset script | new `ardupilot/scripts/reset_pids.lua` | ~20 |

**Total: ~260 lines, single dev-day.**

## Smoke test plan

After implementing 1–6:

```bash
# 1. Manual: launch SITL, bridge, Isaac. Confirm bridge_client.teleport works.
# 2. Force-crash test: in env, manually trigger the crash path; verify recovery runs <30 s.
# 3. Soft-terminate test: tune gains aggressively to trigger soft-terminate; verify reset <15 s.
# 4. 20-episode unattended run from a Phase-1 checkpoint; report wall-clock distribution
#    (median, p95). Target: median <90 s, p95 <150 s.
```

## What this DOESN'T solve

- **EKF divergence after teleport.** Layer-3 calibration helps but isn't bulletproof. If Lua-based EKF reset is needed, see https://ardupilot.org/copter/docs/parameters.html#ek3-source-options.
- **Multi-SITL parallelism** for higher throughput — separate effort. RLDroneSim repo is the reference.
- **Catastrophic SITL hang** (process freeze, MAVLink stops). Last-resort: supervisor that subprocess.kill + relaunch SITL after N consecutive failures. Easy to bolt on top.
