"""RL hover adapter — runs a trained policy against live SITL+bridge for benchmarking.

Connects to ArduCopter SITL at MAVLink port 14552 (the "RL adapter" port your
SITL launch line already opens), reads telemetry at 2 Hz, calls
``model.predict(obs)``, sends MAVLink ``PARAM_SET`` to nudge ATC PIDs. Logs
its own action trace to a sidecar JSONL. Does NOT take off, disarm, or change
flight modes — assumes the drone is already hovering. Stop with Ctrl-C.

Usage (with the existing SITL + Isaac bridge already running):

    capstone/rl/.rl_venv/Scripts/python.exe -m capstone.rl.rl_hover_adapter \\
        --algo PPO \\
        --checkpoint logs/checkpoints/compare_v0/PPO_seed2.zip \\
        --duration 90 \\
        --sidecar reports/benchmark_hover_report/rl_calm_run/rl_actions_RL1.jsonl

The adapter exits when ``--duration`` seconds have elapsed. The bridge keeps
writing its normal flight JSONL — that file is what
``capstone.control.benchmark_hover`` post-processes for the apples-to-apples
metrics. The sidecar is purely for inspecting what the policy did.

Observation / action specs match :mod:`capstone.rl.envs.hover_pid_tuner_v0`,
which is the deployment-side env this policy was *designed* to control.
"""

from __future__ import annotations

import argparse
import json
import logging
import signal
import sys
import time
from dataclasses import asdict
from pathlib import Path

import numpy as np
import yaml
from pymavlink import mavutil

from capstone.rl.envs.hover_pid_tuner_v0 import DEFAULT_GAINS, GainSpec


log = logging.getLogger("capstone.rl.adapter")


def load_gain_spec(path: str | Path) -> list[GainSpec]:
    """Load the `gains:` block from a training-config yaml (e.g. compare_v1.yaml).

    The yaml schema matches what `capstone.rl.config_loader.load_config` parses
    for training — same keys (`name`, `baseline`, `lo`, `hi`, `delta_per_step`).
    Used so the deployment-side adapter sees the exact same action set the
    policy was trained on.
    """
    raw = yaml.safe_load(Path(path).read_text())
    if "gains" not in raw:
        raise KeyError(f"{path}: missing top-level `gains:` block")
    return [
        GainSpec(
            name=str(g["name"]),
            baseline=float(g["baseline"]),
            lo=float(g["lo"]),
            hi=float(g["hi"]),
            delta_per_step=float(g["delta_per_step"]),
        )
        for g in raw["gains"]
    ]


# -----------------------------------------------------------------------------
# Telemetry helpers — same approach as hover_pid_tuner_v0 but stripped of
# takeoff / settle / reward / done logic.
# -----------------------------------------------------------------------------
def read_telemetry(m: mavutil.mavfile, timeout_s: float = 0.4,
                   counter: dict | None = None) -> dict | None:
    snap: dict = {}
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        msg = m.recv_match(blocking=False)
        if msg is None:
            time.sleep(0.005)
            continue
        t = msg.get_type()
        if counter is not None:
            counter[t] = counter.get(t, 0) + 1
        if t == "ATTITUDE":              snap["attitude"]  = msg
        elif t == "LOCAL_POSITION_NED":  snap["local_pos"] = msg
        if {"attitude", "local_pos"}.issubset(snap):
            break
    return snap or None


def drain_telemetry(m: mavutil.mavfile, cache: dict,
                    until: float, counter: dict | None = None) -> None:
    """Non-blocking drain into the cache. Updates `cache['attitude']` and
    `cache['local_pos']` (with their arrival timestamps) whenever a new
    message arrives — but never blocks more than briefly so the policy loop
    stays at the requested cadence even under packet jitter."""
    while time.time() < until:
        msg = m.recv_match(blocking=False)
        if msg is None:
            time.sleep(0.002)
            continue
        t = msg.get_type()
        if counter is not None:
            counter[t] = counter.get(t, 0) + 1
        now = time.time()
        if t == "ATTITUDE":
            cache["attitude"] = msg
            cache["attitude_t"] = now
        elif t == "LOCAL_POSITION_NED":
            cache["local_pos"] = msg
            cache["local_pos_t"] = now


def request_streams(m: mavutil.mavfile, hz: int = 10) -> None:
    """Explicitly request ATTITUDE + LOCAL_POSITION_NED on this MAVLink endpoint.

    Stream rates that the user's SR1_* params set apply only to the *primary*
    MAVLink endpoint. A secondary `--out=...:14552` connection often won't
    receive the streams we need unless we ask explicitly via SET_MESSAGE_INTERVAL.
    """
    interval_us = int(1_000_000 / max(hz, 1))
    for msg_id in (
        mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
        mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
    ):
        m.mav.command_long_send(
            m.target_system, m.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            float(msg_id),
            float(interval_us),
            0, 0, 0, 0, 0,
        )


def send_param(m: mavutil.mavfile, name: str, value: float) -> None:
    m.mav.param_set_send(
        m.target_system, m.target_component,
        name.encode("ascii"),
        float(value),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
    )


def build_obs(snap: dict, gains: list[GainSpec], current_vals: np.ndarray,
              hover_alt_m: float, home_xy: tuple[float, float]
              ) -> np.ndarray:
    lp, att = snap["local_pos"], snap["attitude"]
    gains_norm = np.array(
        [(current_vals[i] - g.lo) / max(g.hi - g.lo, 1e-9)
         for i, g in enumerate(gains)],
        dtype=np.float32,
    )
    alt_err = (-float(lp.z)) - hover_alt_m
    pos_n = float(lp.x) - home_xy[0]
    pos_e = float(lp.y) - home_xy[1]
    vel_n = float(lp.vx); vel_e = float(lp.vy); vel_d = float(lp.vz)
    roll = float(att.roll); pitch = float(att.pitch)
    gx = float(att.rollspeed); gy = float(att.pitchspeed); gz = float(att.yawspeed)
    telem = np.array(
        [alt_err, pos_n, pos_e, vel_n, vel_e, vel_d,
         roll, pitch, gx, gy, gz],
        dtype=np.float32,
    )
    return np.concatenate([gains_norm, telem]).astype(np.float32)


def apply_action(m: mavutil.mavfile, action: np.ndarray,
                 gains: list[GainSpec], current_vals: np.ndarray) -> int:
    """Apply the action delta and PARAM_SET any gain that actually changed.

    Returns the count of params sent (for logging)."""
    action = np.clip(action, -1.0, 1.0).astype(np.float64)
    sent = 0
    for i, g in enumerate(gains):
        new_val = float(np.clip(
            current_vals[i] + action[i] * g.delta_per_step,
            g.lo, g.hi,
        ))
        if abs(new_val - current_vals[i]) > 1e-9:
            current_vals[i] = new_val
            send_param(m, g.name, new_val)
            sent += 1
    return sent


# -----------------------------------------------------------------------------
# Main loop
# -----------------------------------------------------------------------------
def main() -> int:
    ap = argparse.ArgumentParser(description="Run a trained PID-tuning policy against live SITL.")
    ap.add_argument("--algo", required=True, choices=["DDPG", "TD3", "PPO", "SAC"])
    ap.add_argument("--checkpoint", required=True,
                    help="Path to a SB3 .zip checkpoint. The action dim of the checkpoint must match "
                         "the number of gains loaded from --gain-spec (or DEFAULT_GAINS if no spec).")
    ap.add_argument("--gain-spec", default=None,
                    help="Path to a training-config yaml (e.g. capstone/rl/cfg/compare_v1.yaml) "
                         "whose `gains:` block defines the action set the policy was trained on. "
                         "If omitted, falls back to DEFAULT_GAINS (8 attitude gains, v0-compatible only).")
    ap.add_argument("--mavlink-url", default="udpin:localhost:14552",
                    help="MAVLink endpoint to bind to. Must match a SITL --out= line.")
    ap.add_argument("--hover-alt", type=float, default=3.0)
    ap.add_argument("--step-interval-s", type=float, default=0.5,
                    help="Seconds between policy queries — match training.")
    ap.add_argument("--duration", type=float, default=90.0,
                    help="Max wall-clock seconds for the policy loop AFTER hover is detected. "
                         "Adapter also auto-exits if the drone starts descending.")
    ap.add_argument("--wait-for-hover", action="store_true", default=True,
                    help="(default) Block until alt_err < 0.3 m and |vel| < 0.3 m/s before starting.")
    ap.add_argument("--no-wait-for-hover", dest="wait_for_hover", action="store_false",
                    help="Start the policy loop immediately, even if drone is still climbing.")
    ap.add_argument("--hover-detect-timeout-s", type=float, default=120.0,
                    help="Max wall-clock to wait for stable hover before giving up.")
    ap.add_argument("--exit-alt-drop-m", type=float, default=1.5,
                    help="If altitude falls more than this below hover_alt mid-run, exit immediately (drone is landing).")
    ap.add_argument("--sidecar", default=None,
                    help="Optional path to write a per-step JSONL action trace.")
    ap.add_argument("--no-restore", action="store_true",
                    help="Do NOT push baseline gains at startup. Use if you've manually set custom gains.")
    ap.add_argument("--log-level", default="INFO")
    args = ap.parse_args()

    logging.basicConfig(level=args.log_level.upper(),
                        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s")

    log.info("connecting MAVLink at %s", args.mavlink_url)
    m = mavutil.mavlink_connection(args.mavlink_url)
    # pymavlink's wait_heartbeat() picks the first HEARTBEAT it sees, which on
    # a multi-client bus is often a GCS/proxy probe (sys=0 or autopilot=INVALID).
    # If we latch sys=0, every PARAM_SET and SET_MESSAGE_INTERVAL goes nowhere
    # and the policy loop spins forever waiting for telemetry. Filter for a
    # real autopilot heartbeat instead.
    autopilot_deadline = time.time() + 30.0
    autopilot_found = False
    while time.time() < autopilot_deadline:
        msg = m.recv_match(type="HEARTBEAT", blocking=True, timeout=5.0)
        if msg is None:
            log.warning("no HEARTBEAT yet on %s — is SITL --out=...:14552 set?", args.mavlink_url)
            continue
        src_sys = msg.get_srcSystem()
        if src_sys == 0:
            log.info("ignoring sys=0 heartbeat (proxy/GCS probe)")
            continue
        if int(msg.autopilot) == mavutil.mavlink.MAV_AUTOPILOT_INVALID:
            log.info("ignoring autopilot=INVALID heartbeat from sys=%d comp=%d (GCS)",
                     src_sys, msg.get_srcComponent())
            continue
        m.target_system = src_sys
        m.target_component = msg.get_srcComponent()
        log.info("autopilot heartbeat from sys=%d comp=%d (autopilot type=%d)",
                 m.target_system, m.target_component, int(msg.autopilot))
        autopilot_found = True
        break
    if not autopilot_found:
        log.error("no autopilot heartbeat within 30 s. SITL is unreachable or only "
                  "GCS/proxy heartbeats are arriving on this port.")
        return 5

    # Request the streams we need at 10 Hz. Without this, a secondary
    # `--out=...:14552` endpoint often gets only HEARTBEAT and SYS_STATUS,
    # which means snap["local_pos"] / snap["attitude"] never populate and
    # the wait-for-hover loop spins indefinitely.
    log.info("requesting ATTITUDE + LOCAL_POSITION_NED at 50 Hz on this endpoint")
    request_streams(m, hz=50)
    time.sleep(1.0)

    if args.gain_spec is not None:
        gains = load_gain_spec(args.gain_spec)
        log.info("loaded %d gains from %s", len(gains), args.gain_spec)
    else:
        gains = list(DEFAULT_GAINS)
        log.info("using DEFAULT_GAINS (%d gains) — pass --gain-spec for v1+", len(gains))
    current_vals = np.array([g.baseline for g in gains], dtype=np.float64)

    if not args.no_restore:
        log.info("restoring baseline ATC gains")
        for i, g in enumerate(gains):
            send_param(m, g.name, g.baseline)
            current_vals[i] = g.baseline
        time.sleep(1.0)

    # Load policy.
    log.info("loading %s checkpoint %s", args.algo, args.checkpoint)
    if args.algo == "DDPG":
        from stable_baselines3 import DDPG as Cls
    elif args.algo == "TD3":
        from stable_baselines3 import TD3 as Cls
    elif args.algo == "SAC":
        from stable_baselines3 import SAC as Cls
    else:
        from stable_baselines3 import PPO as Cls
    model = Cls.load(args.checkpoint, device="cpu")

    expected = int(np.asarray(model.action_space.shape).prod())
    if expected != len(gains):
        log.error("action-dim mismatch: checkpoint expects %d, gain spec has %d. "
                  "Pass --gain-spec pointing at the yaml the policy was trained on.",
                  expected, len(gains))
        return 4

    # Wait for hover stability before locking home_xy + starting policy. This
    # makes the adapter self-aligning — the user can launch it any time after
    # SITL/bridge are up; the policy loop starts only once the drone is at
    # hover altitude with low velocity.
    home_xy = None
    if args.wait_for_hover:
        log.info("waiting for stable hover (alt_err < 0.3 m, |vel| < 0.3 m/s)...")
        hover_start = None
        wait_deadline = time.time() + args.hover_detect_timeout_s
        msg_counter: dict[str, int] = {}
        last_diag = time.time()
        last_lp_alt = None
        while time.time() < wait_deadline:
            snap = read_telemetry(m, timeout_s=0.5, counter=msg_counter)
            if snap is None or "local_pos" not in snap:
                # Periodic diagnostic so user can see what messages ARE arriving.
                if time.time() - last_diag >= 5.0:
                    if msg_counter:
                        top = sorted(msg_counter.items(), key=lambda kv: -kv[1])[:6]
                        log.info("waiting... msgs in last 5s by type: %s",
                                 ", ".join(f"{k}={v}" for k, v in top))
                    else:
                        log.warning("waiting... NO MAVLink messages received yet — check the SITL --out=...:14552 line")
                    msg_counter.clear()
                    last_diag = time.time()
                continue
            lp = snap["local_pos"]
            alt = -float(lp.z)
            alt_err = abs(alt - args.hover_alt)
            v_xy = (float(lp.vx) ** 2 + float(lp.vy) ** 2) ** 0.5
            v_d = abs(float(lp.vz))
            stable = alt_err < 0.3 and v_xy < 0.3 and v_d < 0.2

            # One-line "still waiting" trace at most every 5 s, including the
            # current state so user sees why stability isn't met.
            if time.time() - last_diag >= 5.0:
                log.info("waiting... alt=%.2f m  alt_err=%.2f  v_xy=%.2f  v_d=%.2f  stable=%s",
                         alt, alt_err, v_xy, v_d, stable)
                last_diag = time.time()

            if stable:
                if hover_start is None:
                    hover_start = time.time()
                if time.time() - hover_start > 1.0:
                    home_xy = (float(lp.x), float(lp.y))
                    log.info("hover detected — home_xy=(%.2f, %.2f) alt_err=%.2f v_xy=%.2f",
                             home_xy[0], home_xy[1], alt_err, v_xy)
                    break
            else:
                hover_start = None
        if home_xy is None:
            log.error("no stable hover within %.1f s. Make sure arm_hover finished takeoff first.",
                      args.hover_detect_timeout_s)
            return 3
    else:
        log.info("--no-wait-for-hover: locking home from first telemetry")
        deadline = time.time() + 10.0
        while home_xy is None and time.time() < deadline:
            snap = read_telemetry(m, timeout_s=0.5)
            if snap and "local_pos" in snap:
                lp = snap["local_pos"]
                home_xy = (float(lp.x), float(lp.y))
                log.info("home_xy locked at (%.2f, %.2f)", *home_xy)
        if home_xy is None:
            log.error("no telemetry within 10 s — is SITL+bridge running and streaming SR1_*?")
            return 2

    sidecar_f = open(args.sidecar, "w", encoding="utf-8") if args.sidecar else None

    # Graceful Ctrl-C: write final state + close.
    stop = {"now": False}
    def _sigint(*_):
        log.warning("Ctrl-C — stopping adapter")
        stop["now"] = True
    signal.signal(signal.SIGINT, _sigint)

    started = time.time()
    n_steps = 0
    n_stalls = 0
    last_action = np.zeros(len(gains), dtype=np.float32)
    log.info("running for %.1f s — policy step every %.2f s", args.duration, args.step_interval_s)

    # Cached telemetry across iterations. Stale data within 1 s is OK at hover
    # dynamics; only stall if both caches are older than that. Seed from the
    # snap that detected hover (always has both ATTITUDE and LOCAL_POSITION_NED).
    tel_cache: dict = {}
    if snap and "attitude" in snap:
        tel_cache["attitude"] = snap["attitude"]
        tel_cache["attitude_t"] = time.time()
    if snap and "local_pos" in snap:
        tel_cache["local_pos"] = snap["local_pos"]
        tel_cache["local_pos_t"] = time.time()
    # If either is missing, the first drain in the loop will populate it (and
    # the age-1s gate will treat absent fields as stale until we get one).
    tel_cache.setdefault("attitude_t", 0.0)
    tel_cache.setdefault("local_pos_t", 0.0)

    try:
        while not stop["now"]:
            t_loop = time.time()
            if t_loop - started >= args.duration:
                break

            # Drain whatever's queued during this step interval. We give the
            # drain ~70 % of the step window; the rest is for predict + send.
            drain_until = t_loop + 0.7 * args.step_interval_s
            drain_telemetry(m, tel_cache, drain_until)

            now = time.time()
            age_att = now - tel_cache.get("attitude_t", 0.0)
            age_lp  = now - tel_cache.get("local_pos_t", 0.0)
            if age_att > 1.0 or age_lp > 1.0:
                n_stalls += 1
                log.warning("telemetry stall: ATTITUDE age=%.2fs LOCAL_POSITION_NED age=%.2fs",
                            age_att, age_lp)
                # Don't act on stale data; loop again.
                continue

            # Auto-exit on descent.
            alt = -float(tel_cache["local_pos"].z)
            if alt < args.hover_alt - args.exit_alt_drop_m:
                log.info("alt %.2f < hover_alt - %.2f → drone is landing, exiting",
                         alt, args.exit_alt_drop_m)
                break

            obs = build_obs(tel_cache, gains, current_vals, args.hover_alt, home_xy)
            action, _ = model.predict(obs, deterministic=True)
            action = np.asarray(action, dtype=np.float32)
            n_sent = apply_action(m, action, gains, current_vals)

            if sidecar_f is not None:
                sidecar_f.write(json.dumps({
                    "t": time.time() - started,
                    "step": n_steps,
                    "age_att_s": round(age_att, 3),
                    "age_lp_s":  round(age_lp, 3),
                    "obs": obs.tolist(),
                    "action": action.tolist(),
                    "gains": current_vals.tolist(),
                    "n_param_set": n_sent,
                }) + "\n")
                sidecar_f.flush()

            n_steps += 1
            elapsed = time.time() - t_loop
            sleep_for = args.step_interval_s - elapsed
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                log.warning("loop overran budget by %.3f s", -sleep_for)
    finally:
        if sidecar_f is not None:
            sidecar_f.close()
        elapsed_s = time.time() - started
        cadence = n_steps / max(elapsed_s, 0.01)
        log.info("adapter stopped after %d steps in %.1f s (%.2f Hz, %d stalls)",
                 n_steps, elapsed_s, cadence, n_stalls)
        # Don't disarm or change mode — bridge / user decides what happens to
        # the drone after the adapter exits.

    return 0


if __name__ == "__main__":
    sys.exit(main())
