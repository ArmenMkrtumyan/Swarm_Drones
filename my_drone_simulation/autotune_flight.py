"""Drive ArduCopter's onboard AUTOTUNE flight mode and save the tuned gains.

Flow:
  1. Connect to SITL on UDP 14551.
  2. Wait for EKF POS_HORIZ_ABS (same gate debug_arm_takeoff uses).
  3. Arm in GUIDED, take off to TAKEOFF_ALT (reusing takeoff_and_hold from
     debug_arm_takeoff so we use the proven climb path).
  4. Switch to ALT_HOLD — AUTOTUNE wants a stable manual-throttle base mode
     and ALT_HOLD holds altitude with sticks centered (RC override neutral).
  5. Send RC override = neutral (1500 across roll/pitch/yaw, mid throttle).
  6. Switch to AUTOTUNE (mode 15). ArduCopter performs ±AUTOTUNE_AGGR-sized
     twitches per axis as configured by AUTOTUNE_AXES.
  7. Stream STATUSTEXT for progress. On "AutoTune: Success" or "AutoTune: Saved"
     the tuned gains are stored to ATC_RAT_*_P/I/D internally.
  8. Land + disarm — disarm-while-autotune-was-the-mode is what persists the
     tuned gains to EEPROM (per ArduCopter convention).
  9. Read back the new gains via PARAM_REQUEST_READ and write them to
     autotune_results/autotune_<ts>.parm for paste-back into
     sitl_params_test.parm.

Key parameters (set in sitl_params_test.parm):
  AUTOTUNE_AXES = 7       (roll + pitch + yaw)
  AUTOTUNE_AGGR = 0.075   (slightly conservative; default is 0.1)

Critical: gains revert if pilot switches out of AUTOTUNE before disarming.
We follow the ArduCopter documented procedure exactly: stay in AUTOTUNE
until success, switch to LAND, disarm.
"""
from pymavlink import mavutil
import os
import sys
import time

# Reuse helpers from debug_arm_takeoff so we don't duplicate them
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from debug_arm_takeoff import (
    MASTER, TAKEOFF_ALT, ack_name,
    wait_heartbeat, drain_messages, wait_command_ack,
    is_armed, wait_armed,
    set_param, set_mode, arm,
    wait_for_ekf_ready,
    takeoff_and_hold,
)

# How long to settle in ALT_HOLD before starting AUTOTUNE. Need stable hover.
PRE_AUTOTUNE_HOLD_S = 8.0

# RC channel midpoints (sticks centered). PWM range 1000-2000.
RC_NEUTRAL = 1500
# RC throttle midpoint for ALT_HOLD = no climb/descend command. ALT_HOLD
# treats throttle as a climb-rate target with a deadband around mid-stick.
RC_THROTTLE_NEUTRAL = 1500

# AUTOTUNE has no fixed runtime — it's per-axis adaptive. Cap at 15 minutes
# total which is more than enough for QuadX with AGGR=0.075. If we're still
# running after this, something is wrong.
AUTOTUNE_MAX_DURATION_S = 900.0

# Param names ArduCopter overwrites with the tuned values (so we read these
# back at the end and dump to autotune_results/).
TUNED_PARAMS = [
    "ATC_RAT_RLL_P", "ATC_RAT_RLL_I", "ATC_RAT_RLL_D",
    "ATC_RAT_PIT_P", "ATC_RAT_PIT_I", "ATC_RAT_PIT_D",
    "ATC_RAT_YAW_P", "ATC_RAT_YAW_I", "ATC_RAT_YAW_D",
    "ATC_RAT_RLL_FLTD", "ATC_RAT_PIT_FLTD",  # filter cutoffs are also tuned
    "ATC_ANG_RLL_P", "ATC_ANG_PIT_P", "ATC_ANG_YAW_P",
]

AUTOTUNE_RESULTS_DIR = os.path.abspath(os.path.join(
    os.path.dirname(__file__), "..", "..", "autotune_results"))


def stream_rc_neutral(master):
    """One-shot RC override with all sticks neutral. Repeat periodically
    during AUTOTUNE so SITL doesn't think the link dropped."""
    master.mav.rc_channels_override_send(
        master.target_system, master.target_component,
        RC_NEUTRAL, RC_NEUTRAL, RC_THROTTLE_NEUTRAL, RC_NEUTRAL,
        65535, 65535, 65535, 65535,  # ch5-8 unchanged
    )


def clear_rc_override(master):
    master.mav.rc_channels_override_send(
        master.target_system, master.target_component,
        0, 0, 0, 0, 0, 0, 0, 0,  # 0 = release override
    )


def run_autotune(master, max_duration_s=AUTOTUNE_MAX_DURATION_S, rc_hz=5):
    """Switch to AUTOTUNE mode and watch progress until completion or timeout.

    Returns a string outcome: "success", "failed", "timeout", or "aborted".
    """
    print("\n=== Switching to AUTOTUNE ===")
    set_mode(master, "AUTOTUNE")
    interval = 1.0 / rc_hz
    start = time.time()
    last_rc = 0.0
    last_progress_print = 0.0
    outcome = None
    seen_messages = []

    while time.time() - start < max_duration_s:
        now = time.time()
        elapsed = now - start

        # Keep RC alive
        if now - last_rc >= interval:
            stream_rc_neutral(master)
            last_rc = now

        # Drain messages
        msg = master.recv_match(blocking=True, timeout=0.2)
        if msg is None:
            continue
        mtype = msg.get_type()

        if mtype == "STATUSTEXT":
            text = msg.text.strip()
            seen_messages.append((elapsed, text))
            print(f"  [+{elapsed:6.1f}s] STATUSTEXT: {text}")
            t = text.lower()
            if "autotune" in t and ("success" in t or "saved" in t or "complete" in t):
                outcome = "success"
                break
            if "autotune" in t and ("fail" in t or "stopped" in t or "abort" in t):
                outcome = "failed"
                break

        elif mtype == "HEARTBEAT":
            armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            mode = msg.custom_mode
            if not armed:
                print(f"  [+{elapsed:6.1f}s] HEARTBEAT armed=False mode={mode} — drone disarmed mid-autotune!")
                outcome = "aborted"
                break
            if mode != 15:  # not in AUTOTUNE anymore
                print(f"  [+{elapsed:6.1f}s] HEARTBEAT: mode changed to {mode} — autotune mode lost!")
                outcome = "aborted"
                break

        elif mtype == "GLOBAL_POSITION_INT":
            if now - last_progress_print >= 5.0:
                rel_alt = msg.relative_alt / 1000.0
                vz = msg.vz / 100.0
                print(f"  [+{elapsed:6.1f}s] alt={rel_alt:5.2f}m  vz={vz:+.2f}m/s  (autotune in progress)")
                last_progress_print = now

    if outcome is None:
        outcome = "timeout"
        print(f"\n  AUTOTUNE timed out after {max_duration_s:.0f}s.")

    return outcome


def request_param(master, name, timeout=2.0):
    """Read a single parameter via PARAM_REQUEST_READ."""
    master.mav.param_request_read_send(
        master.target_system, master.target_component,
        name.encode("utf-8"), -1,
    )
    end = time.time() + timeout
    while time.time() < end:
        msg = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
        if msg is None:
            continue
        if msg.param_id == name or msg.param_id.startswith(name + "\x00"):
            return float(msg.param_value)
    return None


def dump_tuned_params(master):
    """Read every TUNED_PARAM and write them to a timestamped file."""
    print(f"\n=== Reading tuned gains ===")
    os.makedirs(AUTOTUNE_RESULTS_DIR, exist_ok=True)
    out_path = os.path.join(
        AUTOTUNE_RESULTS_DIR,
        f"autotune_{time.strftime('%Y%m%d_%H%M%S')}.parm")

    values = {}
    for name in TUNED_PARAMS:
        v = request_param(master, name)
        values[name] = v
        if v is None:
            print(f"  {name:22s} = <no response>")
        else:
            print(f"  {name:22s} = {v:.6f}")

    with open(out_path, "w") as f:
        f.write(f"# Autotune results captured {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
        f.write(f"# Drone: 1.365 kg post-audit Hawks F450 build, post-bridge-fix\n")
        f.write(f"# Source params: sitl_params_test.parm baseline P/I=0.03, D=0.006\n")
        f.write(f"# Paste relevant values back into sitl_params_test.parm.\n")
        f.write(f"#\n")
        for name in TUNED_PARAMS:
            v = values.get(name)
            if v is not None:
                f.write(f"{name} {v:.6f}\n")
            else:
                f.write(f"# {name}: read failed\n")
    print(f"\n  Saved to: {out_path}")
    return out_path


def land_and_wait_for_disarm(master, timeout=30.0):
    """Switch to LAND mode and wait until the autopilot disarms. Disarming
    while the AUTOTUNE-tuned gains are active is what persists them to EEPROM.
    """
    print("\n=== Landing ===")
    set_mode(master, "LAND")
    end = time.time() + timeout
    last_rc = 0.0
    last_print = 0.0
    while time.time() < end:
        now = time.time()
        if now - last_rc >= 0.2:
            # Keep RC alive even during land in case ArduCopter wants it.
            stream_rc_neutral(master)
            last_rc = now

        msg = master.recv_match(blocking=True, timeout=0.5)
        if msg is None:
            continue
        mtype = msg.get_type()
        if mtype == "STATUSTEXT":
            print(f"  STATUSTEXT: {msg.text}")
            if "Disarm" in msg.text or "disarm" in msg.text.lower():
                print("  Disarmed — gains saved to EEPROM.")
                return True
        elif mtype == "HEARTBEAT":
            armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            if not armed:
                print("  HEARTBEAT shows disarmed.")
                return True
        elif mtype == "GLOBAL_POSITION_INT":
            if now - last_print >= 1.5:
                rel_alt = msg.relative_alt / 1000.0
                vz = msg.vz / 100.0
                print(f"  [land] alt={rel_alt:5.2f}m  vz={vz:+.2f}m/s")
                last_print = now
    print("  Land timed out — drone may still be airborne.")
    return False


def main():
    print(f"Connecting to SITL at {MASTER}...")
    master = mavutil.mavlink_connection(MASTER)
    wait_heartbeat(master)

    drain_messages(master, duration=2.0)

    if not wait_for_ekf_ready(master, timeout=60.0):
        print("EKF not ready — aborting.")
        return

    print("\n=== Switching to GUIDED for arming + takeoff ===")
    set_mode(master, "GUIDED")

    arm(master, force=False)
    armed = wait_armed(master, timeout=8.0)
    if not armed:
        print("Failed to arm — aborting.")
        return

    print(f"\n=== Takeoff to {TAKEOFF_ALT}m and brief hold ===")
    if not takeoff_and_hold(master, TAKEOFF_ALT,
                            hold_s=PRE_AUTOTUNE_HOLD_S,
                            climb_timeout=20.0,
                            rate_hz=5):
        print("Climb timed out before reaching target altitude — aborting.")
        return

    print("\n=== Switching to ALT_HOLD for autotune base ===")
    set_mode(master, "ALT_HOLD")
    # Stream neutral RC so ALT_HOLD doesn't misinterpret stick state.
    for _ in range(20):
        stream_rc_neutral(master)
        time.sleep(0.05)

    drain_messages(master, duration=2.0)

    outcome = run_autotune(master)
    print(f"\n=== AUTOTUNE outcome: {outcome.upper()} ===")

    if outcome == "success":
        # Dump gains BEFORE land+disarm, while autotuned values are active in
        # the live param table. They'll persist to EEPROM on disarm.
        dump_tuned_params(master)

    # Always try to land cleanly to release the drone safely.
    land_and_wait_for_disarm(master)

    # Release RC override (otherwise SITL stays under our control).
    clear_rc_override(master)

    print("\n=== Done ===")
    if outcome == "success":
        print("  Tuned gains saved to autotune_results/.")
        print("  Paste relevant values into sitl_params/sitl_params_test.parm,")
        print("  then re-run debug_arm_takeoff.py to verify.")
    else:
        print(f"  AUTOTUNE finished with outcome: {outcome}.")
        print("  Gains are NOT saved; sitl_params_test.parm baseline is unchanged.")


if __name__ == "__main__":
    main()
