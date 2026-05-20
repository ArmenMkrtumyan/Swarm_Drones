import asyncio
import socket
import struct
import json
import os
import datetime
import time
import math
import numpy as np

import omni.usd
import omni.kit.app
import omni.timeline

from pxr import Usd, UsdPhysics, UsdGeom
from isaacsim.core.api.simulation_context import SimulationContext
from isaacsim.core.experimental.prims import RigidPrim
from isaacsim.sensors.physics import _sensor

# =========================================================
# CAPSTONE DISTURBANCE HARNESS (optional)
# =========================================================
# Edit CAPSTONE_PROFILE below to switch between disturbance profiles.
# Stage-1 hover-robustness profiles (fixed-direction, fixed-time):
#   "calm"               -- no disturbance (DEFAULT; behavior identical to pre-harness)
#   "mass_drop_300g"     -- carries +300 g payload, drops it after 5 s of hover
#                           (>= 1.5 m altitude).
#   "wind5"              -- OU wind gust ~5 m/s peaks along world +X (fixed direction)
#   "wind_up3"           -- OU updraft ~3 m/s peaks along world +Z (pushes drone up)
#   "wind_down3"         -- OU downdraft ~3 m/s peaks along world -Z (pushes drone down)
#   "imu_noise"          -- gyro/accel Gaussian + bias-walk on values sent to SITL
#   "worst_case"         -- wind5 (fixed +X) + mass_drop_300g + imu_noise stacked
#
# Stage-2 mission-robustness profiles (added 2026-05-12; random per bridge restart):
#   "wind5_rand"         -- OU wind ~5 m/s, direction sampled from {+X, -X, +Y, -Y}
#                           at bridge construction. Fresh seed (None default) =
#                           different direction every restart.
#   "mission_worst_case" -- wind5_rand + 400 g mass_drop with RANDOM drop time
#                           in [38, 58] s post-airborne (~ "20 s after WP1, before
#                           WP3" on a 100 m square at WP_SPD=8 m/s) + imu_noise.
#                           Drop time re-samples on each disarm/rearm cycle.
#
# Re-import this script in the Isaac Script Editor after editing.
# Implementation in capstone/control/disturbance.py.
CAPSTONE_PROFILE = "calm"
CAPSTONE_PROFILE_SEED: int | None = None  # set int for reproducible runs

import sys as _sys  # noqa: E402  (package shim has to follow stdlib imports)
# capstone/ moved into Swarm_Drones/ on 2026-05-04 so it would be tracked by
# the existing git repo. The path-shim now adds Swarm_Drones to sys.path so
# `from capstone.X import Y` still works.
_CAPSTONE_ROOT = r"C:\Users\user1811\Desktop\armen-capstone\Swarm_Drones"
if _CAPSTONE_ROOT not in _sys.path:
    _sys.path.insert(0, _CAPSTONE_ROOT)

import importlib as _importlib  # noqa: E402

try:
    from capstone.control import disturbance as _disturbance
    # Isaac Script Editor reuses sys.modules across re-runs of this script,
    # so a `from ... import` after editing the source returns the cached old
    # module. Force a reload so edits to capstone/control/disturbance.py
    # always take effect on bridge re-import.
    _disturbance = _importlib.reload(_disturbance)
    print(f"[capstone] disturbance harness loaded (reloaded); "
          f"CAPSTONE_PROFILE={CAPSTONE_PROFILE!r}; "
          f"available={sorted(_disturbance.PROFILE_FACTORIES)}")
except Exception as _e:
    _disturbance = None
    print(f"[capstone] disturbance harness unavailable ({_e!r}) -- forcing calm")



# =========================================================
# PERSISTENT STATE ACROSS RE-RUNS IN ISAAC SCRIPT EDITOR
# =========================================================
try:
    _ARDUPILOT_BRIDGE_RUNNING
except NameError:
    _ARDUPILOT_BRIDGE_RUNNING = False

try:
    _ARDUPILOT_BRIDGE_SOCKET
except NameError:
    _ARDUPILOT_BRIDGE_SOCKET = None

try:
    _ARDUPILOT_BRIDGE_SIM
except NameError:
    _ARDUPILOT_BRIDGE_SIM = None

try:
    _ARDUPILOT_BRIDGE_TASK
except NameError:
    _ARDUPILOT_BRIDGE_TASK = None

try:
    _ARDUPILOT_BRIDGE_LOGGER
except NameError:
    _ARDUPILOT_BRIDGE_LOGGER = None


# =========================================================
# FLIGHT LOGGER
# =========================================================
# Writes a single unified JSONL file per bridge session under
# Swarm_Drones/logs/flight_logs/. Each line is a single event with:
#   t:    Isaac physics/simulation seconds when available
#   src:  one of "sitl->isaac" | "isaac->sitl" | "bridge"
# Then source-specific payload fields. Post-flight analysis just reads the
# file top to bottom — both sides of the loop are on one timeline.
#
# Rate limits:
#   - sitl->isaac: every packet (SITL sends ~SCHED_LOOP_RATE per second, so ~100 Hz)
#   - isaac->sitl: at most once per STATE_LOG_PERIOD_S (50 Hz keeps files small)
#   - bridge:      only on lifecycle events (startup, home_locked, shutdown, errors)
STATE_LOG_PERIOD_S = 0.02   # 50 Hz
DEBUG_PRINT_HZ = 1.0     # do not spam the 1000 Hz physics callback
PWM_DEBUG_PRINT_HZ = 1.0 # rate-limit motor/PWM debug output

FLIGHT_LOG_DIR = r"C:\Users\user1811\Desktop\armen-capstone\Swarm_Drones\logs\flight_logs"


# Self-documenting schema written into each flight log as a `log_schema` event,
# so a reader can grep one file and learn what every abbreviated key means
# without leaving the log. `logview.py` renders this as the LEGEND section.
# Keep this in sync with the `log_state_sent` / `log_sitl_packet` entries.
STATE_SCHEMA = {
    "t":              "sim time (seconds since bridge start)",
    "src":            "isaac->sitl  (this stream is bridge state sent to ArduCopter)",
    "gyro_frd":       "gyroscope sent to SITL, body FRD [gx_fwd, gy_right, gz_down] (rad/s). POST-noise for imu_noise/worst_case profiles.",
    "accel_frd":      "linear accel sent to SITL, body FRD (m/s^2). POST-noise for imu_noise/worst_case profiles.",
    "gyro_frd_truth": "(present only on profiles that inject IMU noise) clean gyro reading before noise is added — same units/frame as gyro_frd. Used so analysis can isolate real attitude motion from synthetic sensor noise.",
    "accel_frd_truth":"(present only on profiles that inject IMU noise) clean accel reading before noise is added — same units/frame as accel_frd.",
    "pos_ned":        "position NED, anchored at home_lock [north_m, east_m, down_m]",
    "rel_altitude":   "altitude above home (m). Convenience field: rel_altitude = -pos_ned[2]",
    "vel_ned":        "velocity NED [vn_north, ve_east, vd_down]  (m/s, +D = downward)",
    "rpy":            "attitude [roll, pitch, yaw]  (radians, ZYX intrinsic)",
    "home_locked":    "true once EKF settled and bridge anchored its NED frame",
    "payload_kg":     "extra mass currently applied as world-Z down-force (kg). 0 for calm/wind/imu; 0.300 -> 0.0 for mass_drop_300g",
    "thrust_total_N": "sum of per-motor thrust applied this tick after lag, lift, ground effect (N)",
    "wind_world":     "(present only when a wind disturbance is active) wind in world frame [wx, wy, wz]  (m/s)",
}
SITL_PACKET_SCHEMA = {
    "t":   "sim time (seconds since bridge start)",
    "src": "sitl->isaac  (this stream is PWM packets received from ArduCopter)",
    "pwm": "PWM commanded to each motor [M1=FR_CCW, M2=RL_CCW, M3=FL_CW, M4=RR_CW]  (us, 1000-2000)",
}
EVENT_SCHEMA = {
    "t":     "sim time (seconds since bridge start)",
    "src":   "bridge  (lifecycle events: setup, calibration, home_locked, mass_drop_event, etc.)",
    "event": "event name (string) — see TIMELINE",
}


class FlightLogger:
    def __init__(self, path):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        self.path = path
        self.f = open(path, "a", buffering=1)  # line-buffered so tail -f works
        self._last_state_t = -1e9

    def _write(self, entry):
        try:
            self.f.write(json.dumps(entry, separators=(",", ":")))
            self.f.write("\n")
        except Exception as e:
            print(f"[flight_logger] write error: {e!r}")

    def log_event(self, t, event, **payload):
        entry = {"t": round(t, 4), "src": "bridge", "event": event}
        entry.update(payload)
        self._write(entry)

    def log_sitl_packet(self, t, pwm):
        # Slimmed 2026-05-05: dropped magic / frame / addr (constants on every
        # packet — captured once in the `first_sitl_packet` event instead).
        entry = {
            "t": round(t, 4),
            "src": "sitl->isaac",
            "pwm": [float(x) for x in pwm.tolist()[:4]],
        }
        self._write(entry)

    def log_state_sent(self, t, *, gyro_frd, accel_frd,
                       pos_ned, vel_ned,
                       rpy, home_locked,
                       payload_kg=0.0, thrust_total_N=0.0,
                       wind_world=None,
                       gyro_frd_truth=None, accel_frd_truth=None):
        if t - self._last_state_t < STATE_LOG_PERIOD_S:
            return
        self._last_state_t = t
        entry = {
            "t": round(t, 4),
            "src": "isaac->sitl",
            "gyro_frd": [round(float(x), 4) for x in gyro_frd],
            "accel_frd": [round(float(x), 4) for x in accel_frd],
            "pos_ned": [round(float(x), 4) for x in pos_ned],
            # Convenience: altitude above home anchor. Same as -pos_ned[2],
            # exposed as its own field so log readers don't have to flip a sign
            # mentally on every line.
            "rel_altitude": round(-float(pos_ned[2]), 4),
            "vel_ned": [round(float(x), 4) for x in vel_ned],
            "rpy": [round(float(x), 4) for x in rpy],
            "home_locked": bool(home_locked),
            # Per-tick "what is the drone carrying / how hard is it pushing":
            #   payload_kg     -- extra mass currently applied as down-force
            #                     (0 for calm/wind/imu, 0.300 -> 0.0 for mass_drop_300g).
            #   thrust_total_N -- sum of per-motor thrust applied this tick after
            #                     lag, translational lift, ground effect.
            "payload_kg": round(float(payload_kg), 4),
            "thrust_total_N": round(float(thrust_total_N), 4),
        }
        # Optional: capstone disturbance wind in world frame. Omitted entirely
        # when no wind is being injected (calm profile or pre-home-lock) to
        # keep file sizes down for the common case.
        if wind_world is not None:
            entry["wind_world"] = [round(float(x), 4) for x in wind_world]
        # Optional: pre-noise IMU truth for profiles that inject IMU noise.
        # Only emitted when distinct from gyro_frd / accel_frd, so calm logs
        # stay the same size and shape as before.
        if gyro_frd_truth is not None:
            entry["gyro_frd_truth"] = [round(float(x), 4) for x in gyro_frd_truth]
        if accel_frd_truth is not None:
            entry["accel_frd_truth"] = [round(float(x), 4) for x in accel_frd_truth]
        self._write(entry)

    def close(self):
        try:
            self.f.close()
        except Exception:
            pass


# =========================================================
# PATHS
# =========================================================
ROBOT_PATH = "/World/hawks_work_f450_jetsonnano_steereocam"
BASE_LINK_PATH = f"{ROBOT_PATH}/base_link"
IMU_SENSOR_PATH = f"{BASE_LINK_PATH}/Imu_Sensor"

# Order MUST match ArduPilot QuadX servo output:
#   pwm[0] = SERVO1 = Motor 1 = Front-Right, CCW
#   pwm[1] = SERVO2 = Motor 2 = Rear-Left,  CCW
#   pwm[2] = SERVO3 = Motor 3 = Front-Left, CW
#   pwm[3] = SERVO4 = Motor 4 = Rear-Right, CW
MOTOR_LINK_PATHS = [
    f"{ROBOT_PATH}/front_right_motor_link",
    f"{ROBOT_PATH}/rear_left_motor_link",
    f"{ROBOT_PATH}/front_left_motor_link",
    f"{ROBOT_PATH}/rear_right_motor_link",
]

# =========================================================
# TIMING / STARTUP
# =========================================================
DESIRED_PHYSICS_DT = 0.001          # 1000 Hz
MIN_ACCEPTABLE_PHYSICS_HZ = 950.0

REQUIRED_SETTLE_SECONDS = 2.0
MAX_SETTLE_SPEED = 0.10             # m/s
MAX_SETTLE_GYRO = 0.10              # rad/s

# =========================================================
# ORIENTATION / FRAMES
# =========================================================
# The Hawks F450 body frame is FLU (+X=forward, +Y=left, +Z=up) — verified
# from URDF-imported motor-link positions (CLAUDE.md §8). Older notes called
# it RFU; that was wrong. ArduPilot uses FRD (+X=forward, +Y=right, +Z=down).
# T_FLU_TO_FRD is diag(1, -1, -1): forward axis stays the same, Y (left↔right)
# and Z (up↔down) flip. T² = I so it's its own inverse — handy because the
# same matrix relabels home-aligned ENU → NED for position/velocity below.
#
# PRIOR BUG (2026-05-02): this was wrongly defined as RFU→FRD =
#   [[0,1,0],[1,0,0],[0,0,-1]]
# which swaps X↔Y on top of flipping Z. Applied to FLU body data, that
# REPORTED PITCH AS ROLL and ROLL AS PITCH back to ArduPilot — every
# attitude/gyro/accel/position the controller saw had roll and pitch swapped.
# Fix verified by comparing bridge log to physical observation: drone
# tipped LEFT, bridge logged as +roll (right-wing-down).
T_FLU_TO_FRD = np.array([
    [1.0,  0.0,  0.0],
    [0.0, -1.0,  0.0],
    [0.0,  0.0, -1.0],
], dtype=np.float64)

# NED reference frame is anchored to the drone's initial orientation at home-lock.
# "North" in ArduPilot = drone's forward direction at takeoff. This sidesteps any
# Isaac world-frame ambiguity (ENU vs NEU vs arbitrary).

# =========================================================
# MOTOR MODEL — omega^2 thrust + yaw torque with first-order lag
# =========================================================
PWM_IDLE = 1000.0
# PWM at which total thrust = m * g (auto-calibrates K_THRUST). Set to match
# MOT_THST_HOVER=0.55 (real F450 3S/920kv hover thrust fraction) so that bridge
# T/W ratio is realistic ~1.82, not the previously-implied 4.0.
# Math: MOT_THST_HOVER = (omega_hover / omega_max)^2; with 0.55, hover PWM
# fraction = sqrt(0.55) = 0.742, so PWM_HOVER = 1000 + 0.742*1000 = 1742.
PWM_HOVER = 1742.0
PWM_MAX = 2000.0
# Rotor speed cap at PWM_MAX (≈ 7640 RPM) — A2212 920kv + 9450 prop on 3S,
# under load (real motors lose ~25% of no-load RPM with prop attached).
# Was previously 1000 (overestimate).
OMEGA_MAX_RAD_S = 800.0
MOTOR_TIME_CONSTANT_S = 0.05        # first-order response from commanded to actual omega; was 0.03 (optimistic — real A2212+ESC respond in 30-100ms; 0.05 is mid-range and safer for sim-to-real transfer of attitude gains)
K_TORQUE_OVER_K_THRUST = 0.02       # reaction-torque / thrust ratio (m); ~0.02 for 8–10" props

# Reaction-torque sign per motor, applied as K_Q * omega² * MOTOR_SPIN_DIR
# about body +Z (FLU body, +Z = up). Derivation:
#   A CCW prop (viewed from above) is pushed by the motor with +Z body torque;
#   Newton's 3rd law puts the reaction on the body as -Z body torque (CW).
# So CCW-prop motors use -1 here; CW-prop motors use +1.
# Matches ArduPilot QuadX: pwm[0]=FR=CCW, pwm[1]=RL=CCW, pwm[2]=FL=CW, pwm[3]=RR=CW.
MOTOR_SPIN_DIR = np.array([-1.0, -1.0, +1.0, +1.0], dtype=np.float32)

# Visual propeller spin: continuous-type rotor joints from the URDF, axis = +Z
# body. Same order as MOTOR_LINK_PATHS = [FR, RL, FL, RR].
# Rotor angular velocity sign in body +Z is the OPPOSITE of MOTOR_SPIN_DIR
# (which is the body's reaction torque sign). So FR/RL spin CCW (+Z) and
# FL/RR spin CW (-Z) viewed from above -- matches ArduPilot QuadX physical
# direction.
ROTOR_JOINT_NAMES = [
    "front_right_rotor_joint",
    "rear_left_rotor_joint",
    "front_left_rotor_joint",
    "rear_right_rotor_joint",
]
ROTOR_SPIN_SIGN = (-MOTOR_SPIN_DIR).astype(np.float32)  # [+1, +1, -1, -1]

# Thrust axis = body +Z in FLU (= world +Z at rest).
MOTOR_THRUST_DIR_LOCAL = np.array([0.0, 0.0, 1.0], dtype=np.float32)

# Motor positions relative to base_link, in meters, in **FLU** body frame
# (+X = forward, +Y = left, +Z = up). Same order as MOTOR_LINK_PATHS:
# [FR, RL, FL, RR]. Verified from live USD probe (2026-05-02):
#   FR (front-right): body (+0.159, -0.159, +0.008)
#   RL (rear-left):   body (-0.159, +0.159, +0.008)
#   FL (front-left):  body (+0.159, +0.159, +0.008)
#   RR (rear-right):  body (-0.159, -0.159, +0.008)
# Used to compute the net roll/pitch torque analytically so we can apply the
# entire wrench (force + torque) to the articulation root (base_link) in one
# call. PRIOR BUG: these constants were RFU-laid-out (matching the older note
# that the body was RFU). The actual USD body frame is FLU, so the positions
# were 90° off — this swapped roll↔pitch in the analytical torque, producing
# the cross-coupled "drone slides sideways instead of moving forward" pattern.
MOTOR_ARM_LENGTH = 0.159   # |x|, |y| coord from base origin (= 225 mm half-diagonal / sqrt(2)); F450 has 450 mm motor-to-motor diagonal
ROTOR_OFFSET_Z = 0.008     # motor link Z offset above base_link origin (from USD probe)
MOTOR_POS_REL_BASE = np.array([
    [+MOTOR_ARM_LENGTH, -MOTOR_ARM_LENGTH, ROTOR_OFFSET_Z],  # FR: forward (+X), right (-Y)
    [-MOTOR_ARM_LENGTH, +MOTOR_ARM_LENGTH, ROTOR_OFFSET_Z],  # RL: rear   (-X), left  (+Y)
    [+MOTOR_ARM_LENGTH, +MOTOR_ARM_LENGTH, ROTOR_OFFSET_Z],  # FL: forward (+X), left  (+Y)
    [-MOTOR_ARM_LENGTH, -MOTOR_ARM_LENGTH, ROTOR_OFFSET_Z],  # RR: rear   (-X), right (-Y)
], dtype=np.float64)

# =========================================================
# AERODYNAMIC MODEL (translational drag + translational lift + ground effect)
# =========================================================
# Quadratic body drag: F_drag = -K_DRAG * v * |v| applied at composite COM.
# Calibrated so vertical terminal velocity ≈ 22 m/s for the F450 at 1.365 kg
# (m·g = K_DRAG · v_term²  →  K_DRAG ≈ 13.4 / 22² ≈ 0.028 N·s²/m²).
# Isotropic for now — F450 has slightly more frontal drag when pitched forward
# but this first-pass single coefficient captures the dominant effect.
K_DRAG = 0.028

# Translational lift bonus in forward flight (Bauersfeld & Scaramuzza, 2021):
# multirotors gain ~10-20% thrust efficiency at 5-9 m/s as rotors enter clean
# air. Modeled as a Gaussian on per-motor thrust centered at PEAK_SPEED.
TRANS_LIFT_PEAK_GAIN = 0.15        # max bonus at peak speed
TRANS_LIFT_PEAK_SPEED = 7.0        # m/s, dip center for F450
TRANS_LIFT_WIDTH = 4.0             # m/s, Gaussian sigma — falloff width

# Ground effect: per-rotor thrust enhancement when rotor altitude AGL drops
# below ~1 rotor diameter. Approximation: T_eff = T * (1 + GAIN · (1 - z/D)²)
# for z < D, else T. Assumes flat ground at world Z = GROUND_REFERENCE_Z.
GROUND_EFFECT_DIAM = 0.239         # m, 9450 prop diameter
GROUND_EFFECT_GAIN = 0.10          # peak +10% boost at z=0
ROTOR_Z_IN_BODY = 0.023            # m, rotor offset above base_link (body frame)
GROUND_REFERENCE_Z = 0.0           # m, world Z of "ground" — flat-ground
                                   # assumption; for accurate AGL on uneven
                                   # terrain, raycast downward instead.

# Aerodynamic angular damping: τ_damp = -ANG_DAMPING * ω_body. Added
# 2026-05-02 (was previously claimed in CLAUDE.md but never actually wired
# into the bridge — verified absent before this commit).
# Values chosen to match Bauersfeld/Scaramuzza-style F450 modeling:
# real props at hover have significant rotational drag (each rotating prop
# acts as a viscous brake on body roll/pitch); body aerodynamic damping
# adds more. Without it, sim was operating at the edge of controller
# stability — small EKF/timing variations made some flights tumble while
# others hovered with the same gains.
ANG_DAMPING = (0.05, 0.05, 0.04)   # N·m / (rad/s) for roll, pitch, yaw
                                   # Yaw bumped from 0.015 -> 0.04 on
                                   # 2026-05-02 evening: with the lower
                                   # value, yaw drift during ALT_HOLD
                                   # entry was ~25°/s for several seconds
                                   # (105° accumulated drift), which
                                   # broke autotune's "level" check
                                   # between twitches. 0.04 is closer to
                                   # the roll/pitch values and reflects
                                   # real-prop yaw drag better — even at
                                   # axial spin, the prop has skin
                                   # friction proportional to body yaw
                                   # rate, not just RPM.

# =========================================================
# GLOBALS
# =========================================================
t0 = time.perf_counter()


def prim_exists(path: str) -> bool:
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(path)
    return prim.IsValid()


def decode_sitl_packet(data: bytes):
    """
    Decode ArduPilot JSON backend actuator packet.
    Supports both 16-channel and 32-channel variants.
    """
    if len(data) < 8:
        return None

    endian = "<"
    magic, frame_rate, frame_count = struct.unpack_from("<HHI", data, 0)

    if magic not in (18458, 29569):
        magic, frame_rate, frame_count = struct.unpack_from(">HHI", data, 0)
        endian = ">"

    if magic == 18458:
        n = 16
    elif magic == 29569:
        n = 32
    else:
        return None

    expected = 8 + 2 * n
    if len(data) < expected:
        return None

    pwm = struct.unpack_from(f"{endian}{n}H", data, 8)
    return {
        "magic": magic,
        "frame_rate": frame_rate,
        "frame_count": frame_count,
        "pwm": np.array(pwm, dtype=np.float32),
    }


def pwm_to_omega_cmd(pwm_us: np.ndarray) -> np.ndarray:
    """
    Map PWM microseconds to commanded rotor angular velocity in rad/s.
    PWM_IDLE -> 0, PWM_MAX -> OMEGA_MAX_RAD_S, linear between.
    """
    pwm_clamped = np.clip(pwm_us, PWM_IDLE, PWM_MAX)
    return (pwm_clamped - PWM_IDLE) / (PWM_MAX - PWM_IDLE) * OMEGA_MAX_RAD_S


def compute_total_drone_mass(stage, robot_root: str) -> float:
    """Sum authored masses on all rigid bodies under the drone root."""
    root = stage.GetPrimAtPath(robot_root)
    total = 0.0
    for p in Usd.PrimRange(root):
        if p.HasAPI(UsdPhysics.MassAPI):
            m_attr = UsdPhysics.MassAPI(p).GetMassAttr()
            if m_attr and m_attr.HasAuthoredValue():
                total += float(m_attr.Get())
    return total


def compute_composite_com_local_base(stage, robot_root: str):
    """Composite center-of-mass of all rigid bodies in base_link's local frame.

    The wrench is applied at base_link; if base_link's origin isn't at the
    articulation's aggregate COM, a symmetric collective produces a constant
    lever-arm torque that the attitude controller has to fight. Returning the
    COM in base_link-local lets us apply the wrench *at* the COM so per-motor
    differentials are the only source of roll/pitch torque.

    Returns (total_mass_kg, com_local_base as np.ndarray shape (3,)).
    """
    root = stage.GetPrimAtPath(robot_root)
    base_prim = stage.GetPrimAtPath(f"{robot_root}/base_link")
    xf_cache = UsdGeom.XformCache(Usd.TimeCode.Default())

    def _world_mat(prim):
        return xf_cache.GetLocalToWorldTransform(prim)

    def _world_pos(prim):
        t = _world_mat(prim).ExtractTranslation()
        return np.array([t[0], t[1], t[2]], dtype=np.float64)

    def _world_rot(prim):
        # Upper-left 3x3 of Gf.Matrix4d. USD stores row-major with row-vector
        # semantics (v_world = v_body * M), so the *rows* of M are the body
        # axes in world (not columns). To get a standard column-vector
        # rotation matrix R such that R @ v_body = v_world, we have to
        # TRANSPOSE on read: R[i,j] = m[j][i]. Then standard NumPy column-
        # vector conventions apply for R @, R.T @, etc.
        #
        # PRIOR BUG (2026-05-02): this used to read R[i,j] = m[i][j], which
        # silently treated USD's row-vec matrix as a column-vec rotation —
        # equivalent to using M instead of M^T. Result: every CoM and rotated
        # vector came out with X and Y signs flipped. Bridge reported the
        # composite CoM as -8.8 mm rearward when it's actually +8.8 mm
        # forward → analytical wrench applied at mirror-image point and
        # with opposite-sign pitch torque. That's why the drone kept drifting
        # forward despite controller corrections — the controller was
        # commanding the right thing, the bridge was applying the wrong sign.
        m = _world_mat(prim)
        R = np.empty((3, 3), dtype=np.float64)
        for i in range(3):
            for j in range(3):
                R[i, j] = m[j][i]
        return R

    base_wpos = _world_pos(base_prim)
    base_wrot = _world_rot(base_prim)

    total_mass = 0.0
    weighted_world = np.zeros(3, dtype=np.float64)
    for p in Usd.PrimRange(root):
        if not p.HasAPI(UsdPhysics.MassAPI):
            continue
        api = UsdPhysics.MassAPI(p)
        ma = api.GetMassAttr()
        if not (ma and ma.HasAuthoredValue()):
            continue
        m_kg = float(ma.Get())
        body_wpos = _world_pos(p)
        body_wrot = _world_rot(p)
        com_local_body = np.zeros(3, dtype=np.float64)
        ca = api.GetCenterOfMassAttr()
        if ca and ca.HasAuthoredValue():
            v = ca.Get()
            com_local_body = np.array([v[0], v[1], v[2]], dtype=np.float64)
        # body_wrot @ com_local_body gives the local COM offset expressed in world.
        com_world = body_wpos + body_wrot @ com_local_body
        total_mass += m_kg
        weighted_world += m_kg * com_world

    if total_mass <= 0.0:
        return 0.0, np.zeros(3, dtype=np.float64)

    com_world_composite = weighted_world / total_mass
    delta_world = com_world_composite - base_wpos
    com_local_base = base_wrot.T @ delta_world
    return total_mass, com_local_base


def quat_wxyz_to_rot(q):
    """
    Quaternion [w, x, y, z] -> rotation matrix
    """
    w, x, y, z = q
    return np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - z*w),     2*(x*z + y*w)],
        [    2*(x*y + z*w), 1 - 2*(x*x + z*z),     2*(y*z - x*w)],
        [    2*(x*z - y*w),     2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ], dtype=np.float64)


def rot_to_rpy_zyx(R):
    """
    Rotation matrix -> roll, pitch, yaw
    """
    pitch = -math.asin(np.clip(R[2, 0], -1.0, 1.0))
    roll = math.atan2(R[2, 1], R[2, 2])
    yaw = math.atan2(R[1, 0], R[0, 0])
    return roll, pitch, yaw


def safe_remove_callback(sim_ctx, name: str):
    try:
        sim_ctx.remove_physics_callback(name)
        print(f"Removed old physics callback: {name}")
    except Exception:
        print(f"No old physics callback to remove: {name}")


def close_udp_socket():
    """Tear down the UDP listener and release port 9002 to the OS.

    On Windows, plain socket.close() does NOT immediately return the port
    to the kernel -- a closed-but-not-shut-down socket can linger and keep
    port 9002 unavailable for the next bridge session. shutdown(SHUT_RDWR)
    explicitly tells the kernel "I'm done, release the binding now". UDP
    sockets aren't "connected" so shutdown raises OSError 10057 / ENOTCONN
    on some stacks; we ignore that. The close() that follows then frees the
    file descriptor cleanly.
    """
    global _ARDUPILOT_BRIDGE_SOCKET

    if _ARDUPILOT_BRIDGE_SOCKET is not None:
        try:
            try:
                _ARDUPILOT_BRIDGE_SOCKET.shutdown(socket.SHUT_RDWR)
            except OSError:
                # UDP not-connected / already-shutdown -- expected, not a problem.
                pass
            _ARDUPILOT_BRIDGE_SOCKET.close()
            print("Closed old UDP socket (port 9002 released).")
        except Exception as e:
            print("Warning closing old UDP socket:", repr(e))
        finally:
            _ARDUPILOT_BRIDGE_SOCKET = None


def stop_bridge():
    """
    Safe manual stop. You can run stop_bridge() in Isaac later if needed.
    """
    global _ARDUPILOT_BRIDGE_RUNNING
    global _ARDUPILOT_BRIDGE_TASK
    global _ARDUPILOT_BRIDGE_SIM
    global _ARDUPILOT_BRIDGE_LOGGER

    print("Stopping bridge...")

    try:
        if _ARDUPILOT_BRIDGE_SIM is not None:
            safe_remove_callback(_ARDUPILOT_BRIDGE_SIM, "ardupilot_bridge")
    except Exception as e:
        print("Warning removing callback during stop:", repr(e))

    close_udp_socket()

    try:
        if _ARDUPILOT_BRIDGE_TASK is not None and not _ARDUPILOT_BRIDGE_TASK.done():
            _ARDUPILOT_BRIDGE_TASK.cancel()
            print("Cancelled bridge task.")
    except Exception as e:
        print("Warning cancelling bridge task:", repr(e))

    if _ARDUPILOT_BRIDGE_LOGGER is not None:
        try:
            _ARDUPILOT_BRIDGE_LOGGER.log_event(time.perf_counter() - t0, "bridge_stopped")
            _ARDUPILOT_BRIDGE_LOGGER.close()
            print(f"Flight log closed: {_ARDUPILOT_BRIDGE_LOGGER.path}")
        except Exception as e:
            print("Warning closing logger:", repr(e))
        _ARDUPILOT_BRIDGE_LOGGER = None

    _ARDUPILOT_BRIDGE_TASK = None
    _ARDUPILOT_BRIDGE_RUNNING = False
    print("Bridge stopped.")


async def setup_bridge():
    global _ARDUPILOT_BRIDGE_SOCKET
    global _ARDUPILOT_BRIDGE_SIM
    global _ARDUPILOT_BRIDGE_LOGGER

    print("=== ArduPilot <-> Isaac Bridge Setup ===")

    # Fresh per-session flight log
    session_name = datetime.datetime.now().strftime("flight_%Y%m%d_%H%M%S.jsonl")
    _ARDUPILOT_BRIDGE_LOGGER = FlightLogger(os.path.join(FLIGHT_LOG_DIR, session_name))
    print(f"Flight log: {_ARDUPILOT_BRIDGE_LOGGER.path}")
    _ARDUPILOT_BRIDGE_LOGGER.log_event(
        0.0, "bridge_setup_started",
        robot=ROBOT_PATH,
        motor_order=["FR_CCW", "RL_CCW", "FL_CW", "RR_CW"],
    )
    # Self-documenting legend: every abbreviated key in this log file's three
    # streams gets a one-line plain-English description. Use `logview.py` to
    # render it as a LEGEND section, or just `head` the JSONL file.
    # One log_schema entry per (stream, field) pair — keeps each line short
    # and grep-friendly. `logview.py` regroups them into the LEGEND section.
    for _stream, _schema in (
        ("state", STATE_SCHEMA),
        ("sitl_packet", SITL_PACKET_SCHEMA),
        ("events", EVENT_SCHEMA),
    ):
        for _field, _desc in _schema.items():
            _ARDUPILOT_BRIDGE_LOGGER.log_event(
                0.0, "log_schema",
                stream=_stream, field=_field, desc=_desc,
            )

    timeline = omni.timeline.get_timeline_interface()
    if not timeline.is_playing():
        raise RuntimeError("Press Play first, then run this script.")

    print("ROBOT_PATH:", ROBOT_PATH)
    print("BASE_LINK_PATH:", BASE_LINK_PATH)
    print("IMU_SENSOR_PATH:", IMU_SENSOR_PATH)
    print("MOTOR_LINK_PATHS:")
    for p in MOTOR_LINK_PATHS:
        print(" ", p)

    paths_to_check = [BASE_LINK_PATH, IMU_SENSOR_PATH] + MOTOR_LINK_PATHS
    missing = [p for p in paths_to_check if not prim_exists(p)]
    if missing:
        raise RuntimeError("Missing prim paths:\n" + "\n".join(missing))

    sim = SimulationContext.instance()
    if sim is None:
        print("No active SimulationContext found. Creating one...")
        sim = SimulationContext()
        await sim.initialize_simulation_context_async()

    _ARDUPILOT_BRIDGE_SIM = sim

    app = omni.kit.app.get_app()

    # Force physics dt
    try:
        sim.set_physics_dt(DESIRED_PHYSICS_DT)
    except Exception as e:
        print("Warning: set_physics_dt failed:", repr(e))

    await app.next_update_async()
    await app.next_update_async()

    safe_remove_callback(sim, "ardupilot_bridge")
    close_udp_socket()

    try:
        current_dt = sim.get_physics_dt()
        current_hz = 1.0 / current_dt if current_dt and current_dt > 0 else 0.0
        print(f"Current physics dt={current_dt:.6f}s ({current_hz:.1f} Hz)")
    except Exception as e:
        raise RuntimeError(f"Could not read physics dt: {repr(e)}")

    if current_hz < MIN_ACCEPTABLE_PHYSICS_HZ:
        raise RuntimeError(
            f"Physics rate is too low: {current_hz:.1f} Hz. "
            f"Expected ~1000 Hz. Refusing to run bridge."
        )

    udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    udp_sock.bind(("0.0.0.0", 9002))
    udp_sock.setblocking(False)
    _ARDUPILOT_BRIDGE_SOCKET = udp_sock

    base = RigidPrim(BASE_LINK_PATH)
    motor_bodies = [RigidPrim(p) for p in MOTOR_LINK_PATHS]
    imu_interface = _sensor.acquire_imu_sensor_interface()

    # Visual propeller spin -- kinematic write of rotor joint positions each
    # tick. Rotor links are 10 g each and we set joint state directly (no
    # torque applied through the controller), so reaction on the body is zero
    # by construction. If the articulation can't be initialized (older USD
    # without rotor joints, API mismatch, etc.) the bridge keeps flying with
    # this feature silently disabled.
    rotor_state = {
        "art": None,
        "dof_indices": None,
        "buffer": None,
        "angles": np.zeros(4, dtype=np.float32),
        "warned": False,
    }
    try:
        from isaacsim.core.prims import Articulation as _Articulation
        _art = _Articulation(prim_paths_expr=ROBOT_PATH)
        _art.initialize()
        # The articulation has 30 named joints in this URDF but only 4 actual
        # DOFs (the four continuous rotor joints; everything else is fixed).
        # set_joint_positions expects shape (num_envs, num_dofs), so we size
        # buffer/index lookup against the DOF count, not joint_names.
        _num_dofs = int(np.asarray(_art.get_joint_positions()).reshape(1, -1).shape[1])
        _dof_names = None
        for _attr in ("dof_names", "joint_names"):
            if hasattr(_art, _attr):
                _cand = getattr(_art, _attr)
                if _cand is not None and len(_cand) == _num_dofs:
                    _dof_names = list(_cand)
                    break
        if _dof_names is None:
            raise RuntimeError(
                f"articulation reports num_dofs={_num_dofs} but no name "
                f"attribute matches that length"
            )
        rotor_state["dof_indices"] = np.array(
            [_dof_names.index(n) for n in ROTOR_JOINT_NAMES], dtype=np.int64,
        )
        rotor_state["buffer"] = np.zeros(_num_dofs, dtype=np.float32)
        rotor_state["art"] = _art
        print(f"[propeller-visual] enabled. num_dofs={_num_dofs}  "
              f"dof_names={_dof_names}  "
              f"motor_to_dof={rotor_state['dof_indices'].tolist()}  "
              f"spin_sign={ROTOR_SPIN_SIGN.tolist()}")
    except Exception as _e:
        print(f"[propeller-visual] disabled ({type(_e).__name__}: {_e}); "
              f"flight unaffected")

    # Calibrate K_THRUST so total thrust = m*g when all motors run at omega_hover.
    # Also grab the composite COM in base_link-local frame so the wrench can be
    # applied at the aggregate COM (not at base_link origin) — otherwise a
    # symmetric collective leaves a constant lever-arm torque for the attitude
    # controller to fight.
    stage = omni.usd.get_context().get_stage()
    total_mass, com_local_base = compute_composite_com_local_base(stage, ROBOT_PATH)
    if total_mass < 0.1:
        print(f"WARNING: computed drone mass {total_mass:.3f} kg looks wrong; falling back to 1.365 kg")
        total_mass = 1.365
        com_local_base = np.zeros(3, dtype=np.float64)
    omega_hover = (PWM_HOVER - PWM_IDLE) / (PWM_MAX - PWM_IDLE) * OMEGA_MAX_RAD_S
    K_THRUST = (total_mass * 9.81) / (4.0 * omega_hover ** 2)
    K_TORQUE = K_THRUST * K_TORQUE_OVER_K_THRUST
    print(f"Motor model: m={total_mass:.3f}kg  omega_hover={omega_hover:.1f}rad/s  "
          f"K_T={K_THRUST:.4e} N/(rad/s)^2  K_Q={K_TORQUE:.4e} Nm/(rad/s)^2")
    print(f"Composite COM in base_link frame (mm): "
          f"[{com_local_base[0]*1000:+.2f}, {com_local_base[1]*1000:+.2f}, {com_local_base[2]*1000:+.2f}]  "
          f"(|offset| = {np.linalg.norm(com_local_base)*1000:.2f} mm)")
    if _ARDUPILOT_BRIDGE_LOGGER is not None:
        _ARDUPILOT_BRIDGE_LOGGER.log_event(
            0.0, "motor_model_calibrated",
            total_mass_kg=float(total_mass),
            omega_hover_rad_s=float(omega_hover),
            K_thrust=float(K_THRUST),
            K_torque=float(K_TORQUE),
            com_local_base_m=[float(x) for x in com_local_base],
            physics_hz=float(current_hz),
        )

    # First-order lag state for rotor angular velocity (per-motor)
    omega_actual = np.zeros(4, dtype=np.float32)

    # Disturbance profile: instantiated fresh per bridge session.
    # When _disturbance is None (capstone package not on path), profile stays
    # None and all hooks no-op -> behavior identical to pre-harness bridge.
    profile = None
    if _disturbance is not None:
        try:
            profile = _disturbance.make(CAPSTONE_PROFILE, seed=CAPSTONE_PROFILE_SEED)
            print(f"[capstone] disturbance profile active: name={profile.name!r}  "
                  f"mass_multiplier={profile.mass_multiplier:.3f}")
            if _ARDUPILOT_BRIDGE_LOGGER is not None:
                _ARDUPILOT_BRIDGE_LOGGER.log_event(
                    0.0, "capstone_disturbance_active",
                    profile=profile.name,
                    mass_multiplier=float(profile.mass_multiplier),
                    seed=CAPSTONE_PROFILE_SEED,
                )
        except Exception as e:
            print(f"[capstone] disturbance init failed ({e!r}); running calm")
            profile = None

    await app.next_update_async()

    try:
        positions, orientations = base.get_world_poses()
        home_pos_world = np.array(positions[0], dtype=np.float64)
    except Exception as e:
        raise RuntimeError(f"Failed reading base pose: {repr(e)}")

    home_R0_world_body = np.eye(3, dtype=np.float64)  # will be captured at home-lock
    home_locked = False
    settle_start = None

    last_addr = None
    first_packet_seen = False
    last_pwm = np.array([1000.0, 1000.0, 1000.0, 1000.0], dtype=np.float32)
    last_debug_print = 0.0
    last_pwm_debug_print = 0.0
    sim_time = 0.0  # deterministic physics time sent to SITL; do NOT use wall-clock here
    imu_warned_invalid = False

    print("Bridge init complete.")
    print("Initial world position:", home_pos_world)
    print("Listening on UDP 9002...")
    print("Waiting to lock home after the vehicle settles.")

    def physics_step(dt):
        nonlocal home_pos_world, home_R0_world_body, home_locked, settle_start
        nonlocal last_addr, first_packet_seen, last_pwm, last_debug_print, last_pwm_debug_print
        nonlocal sim_time, imu_warned_invalid
        nonlocal omega_actual

        # SITL JSON timestamp must be simulation/physics time. Using wall-clock
        # makes the closed-loop behavior depend on computer load and console lag.
        if dt and dt > 0.0:
            sim_time += float(dt)

        # -------------------------------------------------
        # 1) Receive latest SITL actuator packet
        # -------------------------------------------------
        while True:
            try:
                data, addr = udp_sock.recvfrom(2048)
            except BlockingIOError:
                break
            except Exception as e:
                print("Socket recv error:", repr(e))
                return

            pkt = decode_sitl_packet(data)
            if pkt is None:
                continue

            if not first_packet_seen:
                print(f"First SITL packet from {addr}")
                first_packet_seen = True
                if _ARDUPILOT_BRIDGE_LOGGER is not None:
                    # `addr` and `frame_first` captured ONCE here so the
                    # per-packet log_sitl_packet entries can stay slim.
                    _ARDUPILOT_BRIDGE_LOGGER.log_event(
                        sim_time, "first_sitl_packet",
                        addr=list(addr),
                        frame_first=int(pkt["frame_count"]),
                        magic=int(pkt["magic"]),
                    )

            last_addr = addr
            last_pwm = pkt["pwm"][:4].copy()
            if _ARDUPILOT_BRIDGE_LOGGER is not None:
                _ARDUPILOT_BRIDGE_LOGGER.log_sitl_packet(sim_time, pkt["pwm"])

        # -------------------------------------------------
        # 2) Read base state + IMU
        # -------------------------------------------------
        try:
            positions, orientations = base.get_world_poses()
            pos_world = np.array(positions[0], dtype=np.float64)
            quat_world = np.array(orientations[0], dtype=np.float64)

            linear_velocities, angular_velocities = base.get_velocities()
            vel_world = np.array(linear_velocities[0], dtype=np.float64)

            imu_reading = imu_interface.get_sensor_reading(
                IMU_SENSOR_PATH,
                use_latest_data=True,
                read_gravity=True,
            )

            imu_valid = bool(getattr(imu_reading, "is_valid", False))

            if imu_valid:
                imu_ang = np.array([
                    imu_reading.ang_vel_x,
                    imu_reading.ang_vel_y,
                    imu_reading.ang_vel_z,
                ], dtype=np.float64)

                imu_acc = np.array([
                    imu_reading.lin_acc_x,
                    imu_reading.lin_acc_y,
                    imu_reading.lin_acc_z,
                ], dtype=np.float64)

                # Body FLU (Isaac) -> Body FRD (ArduPilot). Same matrix for gyro and accel.
                gyro_body = T_FLU_TO_FRD @ imu_ang
                accel_body = T_FLU_TO_FRD @ imu_acc
            else:
                if not imu_warned_invalid:
                    print("WARNING: IMU reading invalid. Check IMU_SENSOR_PATH.")
                    imu_warned_invalid = True

                gyro_body = np.zeros(3, dtype=np.float64)
                accel_body = np.zeros(3, dtype=np.float64)

            speed = float(np.linalg.norm(vel_world))
            gyro_norm = float(np.linalg.norm(gyro_body))

            # R_world_body in Isaac conventions (body FLU, world Z-up).
            R_world_body = quat_wxyz_to_rot(quat_world)

            # Express body orientation relative to the drone's initial attitude at home.
            # R_rel is body-at-now in the drone's initial body-frame coordinates.
            R_rel = home_R0_world_body.T @ R_world_body

            # Body-FLU-in-initial-frame  ->  Body-FRD-in-NED
            R_ned_body_frd = T_FLU_TO_FRD @ R_rel @ T_FLU_TO_FRD

            roll, pitch, yaw = rot_to_rpy_zyx(R_ned_body_frd)
            heading_deg = (math.degrees(yaw) + 360.0) % 360.0

            # World-frame vectors: rotate into the drone's initial body frame (FLU),
            # then relabel axes to NED.
            rel_world = pos_world - home_pos_world
            pos_init = home_R0_world_body.T @ rel_world
            vel_init = home_R0_world_body.T @ vel_world

            pos_ned = T_FLU_TO_FRD @ pos_init
            vel_ned = T_FLU_TO_FRD @ vel_init

        except Exception as e:
            print("State/read error:", repr(e))
            return

        # -------------------------------------------------
        # 3) Lock home only after the vehicle is settled
        # -------------------------------------------------
        now_sim = sim_time
        if not home_locked:
            if speed < MAX_SETTLE_SPEED and gyro_norm < MAX_SETTLE_GYRO:
                if settle_start is None:
                    settle_start = now_sim
                elif (now_sim - settle_start) >= REQUIRED_SETTLE_SECONDS:
                    home_pos_world = pos_world.copy()
                    # Anchor NED "north" to the drone's current forward direction.
                    home_R0_world_body = R_world_body.copy()
                    home_locked = True
                    print(f"Home locked after settling: pos={home_pos_world}")
                    print(f"  initial body-to-world rotation (used as NED anchor):\n{home_R0_world_body}")
                    # ----- Motor-link physics geometry probe -----
                    # User reported visual motors moving inward at sim start.
                    # Print actual link positions in body frame so we can
                    # confirm physics is using the nominal F450 layout
                    # (±0.159, ±0.159) regardless of what visuals show.
                    motor_geom_log: list[dict] = []
                    nominal_names = ["FR (+X,-Y)", "RL (-X,+Y)", "FL (+X,+Y)", "RR (-X,-Y)"]
                    print("  motor link physics positions (body frame, m):")
                    print(f"    {'name':<14s}  {'actual_x':>9s}  {'actual_y':>9s}  {'actual_z':>9s}     "
                          f"{'nom_x':>7s}  {'nom_y':>7s}  {'nom_z':>7s}     dist_err(mm)")
                    R_w_b_inv = R_world_body.T
                    for i, mb in enumerate(motor_bodies):
                        try:
                            mp_w, _ = mb.get_world_poses()
                            mp_world = np.array(mp_w[0], dtype=np.float64)
                            # World position of motor link → body frame relative to base.
                            mp_body = R_w_b_inv @ (mp_world - pos_world)
                            nom = MOTOR_POS_REL_BASE[i]
                            err_mm = float(np.linalg.norm(mp_body - nom)) * 1000.0
                            print(f"    {nominal_names[i]:<14s}  "
                                  f"{mp_body[0]:+9.4f}  {mp_body[1]:+9.4f}  {mp_body[2]:+9.4f}     "
                                  f"{nom[0]:+7.3f}  {nom[1]:+7.3f}  {nom[2]:+7.3f}     {err_mm:8.2f}")
                            motor_geom_log.append({
                                "name": nominal_names[i],
                                "actual_body": [float(x) for x in mp_body],
                                "nominal_body": [float(x) for x in nom],
                                "err_mm": err_mm,
                            })
                        except Exception as _e:
                            print(f"    {nominal_names[i]:<14s}  probe failed: {_e!r}")
                    if _ARDUPILOT_BRIDGE_LOGGER is not None:
                        _ARDUPILOT_BRIDGE_LOGGER.log_event(
                            sim_time, "home_locked",
                            home_pos_world=home_pos_world.tolist(),
                            R0_world_body=home_R0_world_body.tolist(),
                            motor_link_geometry=motor_geom_log,
                        )
            else:
                settle_start = None

        # -------------------------------------------------
        # 4) PWM -> commanded omega -> first-order lag -> thrust (omega^2) + yaw torque
        # -------------------------------------------------
        omega_cmd = pwm_to_omega_cmd(last_pwm)
        alpha = min(1.0, dt / MOTOR_TIME_CONSTANT_S)
        omega_actual += (omega_cmd - omega_actual) * alpha

        # Visual propeller spin: advance rotor joint angles by omega*dt and
        # write them kinematically. No torque path -- body dynamics unaffected.
        if rotor_state["art"] is not None and dt and dt > 0.0:
            angles = rotor_state["angles"]
            angles += omega_actual * ROTOR_SPIN_SIGN * float(dt)
            np.mod(angles, 2.0 * np.pi, out=angles)
            rotor_state["buffer"][rotor_state["dof_indices"]] = angles
            try:
                rotor_state["art"].set_joint_positions(
                    rotor_state["buffer"].reshape(1, -1)
                )
            except Exception as _e:
                if not rotor_state["warned"]:
                    print(f"[propeller-visual] set_joint_positions failed "
                          f"({type(_e).__name__}: {_e}); disabling visual spin")
                    rotor_state["warned"] = True
                rotor_state["art"] = None

        omega_sq = omega_actual * omega_actual
        per_motor_thrust = K_THRUST * omega_sq                         # N per motor, along local +Z
        per_motor_yaw_torque = K_TORQUE * omega_sq * MOTOR_SPIN_DIR    # N·m per motor, about local +Z

        # -------------------------------------------------
        # 4a) Aerodynamic effects: translational drag + lift + ground effect
        # -------------------------------------------------
        # Body-frame velocity (world velocity rotated into body frame).
        # CAPSTONE: subtract wind in world frame so the drone "feels" apparent
        # flow in drag + lift. Wind is zero before home lock to avoid pushing
        # the drone around during settle.
        if profile is not None and home_locked:
            wind_world = profile.wind_world_mps(dt if dt and dt > 0.0 else 0.001)
        else:
            wind_world = np.zeros(3, dtype=np.float64)
        vel_body = R_world_body.T @ (vel_world - wind_world)

        # Translational lift: per-motor thrust gets a Gaussian bonus around the
        # F450's translational-lift sweet spot near 7 m/s. Affects both thrust
        # along body +Z AND derived roll/pitch torques (since they're computed
        # from per_motor_thrust below).
        v_horizontal = float(math.sqrt(vel_body[0] * vel_body[0] + vel_body[1] * vel_body[1]))
        lift_factor = 1.0 + TRANS_LIFT_PEAK_GAIN * math.exp(
            -((v_horizontal - TRANS_LIFT_PEAK_SPEED) ** 2) / (2.0 * TRANS_LIFT_WIDTH * TRANS_LIFT_WIDTH)
        )
        per_motor_thrust = per_motor_thrust * lift_factor

        # Ground effect: per-rotor altitude AGL determines individual thrust
        # boost. Computed per-rotor so a tilted drone gets asymmetric ground
        # effect (one rotor closer to ground than another).
        for i in range(4):
            rotor_pos_body = np.array(
                [MOTOR_POS_REL_BASE[i, 0], MOTOR_POS_REL_BASE[i, 1], ROTOR_Z_IN_BODY],
                dtype=np.float64,
            )
            rotor_world_z = pos_world[2] + (R_world_body @ rotor_pos_body)[2]
            agl = rotor_world_z - GROUND_REFERENCE_Z
            if agl < GROUND_EFFECT_DIAM:
                gnd_factor = 1.0 + GROUND_EFFECT_GAIN * (1.0 - max(0.0, agl) / GROUND_EFFECT_DIAM) ** 2
                per_motor_thrust[i] *= gnd_factor

        # Translational drag: quadratic in body-frame velocity, applied at
        # composite COM as a body-frame force. Adds no torque (applied at COM).
        speed_body = float(np.linalg.norm(vel_body))
        F_drag_body = -K_DRAG * vel_body * speed_body  # vector, body frame

        # Do not apply forces before home lock
        if not home_locked:
            per_motor_thrust[:] = 0.0
            per_motor_yaw_torque[:] = 0.0
            F_drag_body[:] = 0.0

        # CAPSTONE: IMU noise on the values that go OUT to SITL and the log.
        # Internal `imu_ang` (used for damping torque) stays clean -- otherwise
        # noise feeds back into the physics and we get compounding chaos.
        # Skipped before home lock so settle detection remains noise-free.
        # We also stash the pre-noise truth so the bridge log can carry both
        # for analysis (post-noise = what controller saw, truth = real motion).
        gyro_truth_log: np.ndarray | None = None
        accel_truth_log: np.ndarray | None = None
        if profile is not None and home_locked:
            gyro_n, accel_n = profile.imu_noise(dt if dt and dt > 0.0 else 0.001)
            if (np.any(gyro_n != 0.0) or np.any(accel_n != 0.0)):
                gyro_truth_log = gyro_body.copy()
                accel_truth_log = accel_body.copy()
            gyro_body = gyro_body + gyro_n
            accel_body = accel_body + accel_n

        # -------------------------------------------------
        # 4b) Apply forces PER MOTOR LINK.
        #
        # This keeps the clean old structure, but fixes the force model for an
        # articulated Isaac drone: each motor thrust enters through its own
        # motor link instead of dumping the full 4-motor force into base_link.
        # Drag + angular damping remain body-level effects on base_link.
        # -------------------------------------------------
        try:
            # Per-motor thrust + yaw-reaction torque, applied at each motor link.
            for i in range(4):
                F_i = np.array(
                    [[0.0, 0.0, float(per_motor_thrust[i])]],
                    dtype=np.float32,
                )
                tau_i = np.array(
                    [[0.0, 0.0, float(per_motor_yaw_torque[i])]],
                    dtype=np.float32,
                )
                pos_i = np.array([[0.0, 0.0, 0.0]], dtype=np.float32)
                motor_bodies[i].apply_forces_and_torques_at_pos(
                    forces=F_i,
                    torques=tau_i,
                    positions=pos_i,
                    local_frame=True,
                )

            # Body-level drag at composite CoM.
            drag_force = np.array(
                [[float(F_drag_body[0]),
                  float(F_drag_body[1]),
                  float(F_drag_body[2])]],
                dtype=np.float32,
            )

            # CAPSTONE: mass perturbation as a world-frame down force on base,
            # rotated into body frame so it composes with drag (also body frame
            # because apply_forces_and_torques_at_pos uses local_frame=True).
            # mass_load_N returns negative for added weight (Isaac world Z is up),
            # 0.0 for calm profiles or after a mass_drop has released its payload.
            if profile is not None and home_locked:
                # Tick stateful sub-disturbances (currently: mass_drop) before
                # we read mass_load_N, so the payload reflects the current sim
                # tick (hovering or already dropped).
                altitude_m = -float(pos_ned[2])
                just_dropped = profile.update(
                    dt if dt and dt > 0.0 else 0.001,
                    altitude_m=altitude_m,
                )
                if just_dropped and _ARDUPILOT_BRIDGE_LOGGER is not None:
                    payload_kg = float(profile.mass_drop.payload_kg) if profile.mass_drop else 0.0
                    _ARDUPILOT_BRIDGE_LOGGER.log_event(
                        sim_time, "mass_drop_event",
                        payload_kg=payload_kg,
                        altitude_m=altitude_m,
                    )
                    print(f"[capstone] mass_drop fired at t={sim_time:.2f}s "
                          f"alt={altitude_m:.2f}m payload={payload_kg*1000:.0f}g released")
                mass_force_z = profile.mass_load_N(float(total_mass))
                if mass_force_z != 0.0:
                    F_mass_world = np.array(
                        [0.0, 0.0, mass_force_z],
                        dtype=np.float64,
                    )
                    F_mass_body = R_world_body.T @ F_mass_world
                    drag_force[0, 0] += float(F_mass_body[0])
                    drag_force[0, 1] += float(F_mass_body[1])
                    drag_force[0, 2] += float(F_mass_body[2])

            # Optional trim cancellation caused by applying vertical thrust at
            # motor positions while composite CoM is slightly offset. This keeps
            # hover stable in sim. Later, for sim-to-real, make this scaleable.
            cx, cy, _cz = (float(com_local_base[0]),
                            float(com_local_base[1]),
                            float(com_local_base[2]))
            F_total_now = float(np.sum(per_motor_thrust))
            tau_ff_x = +cy * F_total_now
            tau_ff_y = -cx * F_total_now
            tau_ff_z = 0.0

            if home_locked and imu_valid:
                body_torque = np.array([[
                    tau_ff_x - ANG_DAMPING[0] * float(imu_ang[0]),
                    tau_ff_y - ANG_DAMPING[1] * float(imu_ang[1]),
                    tau_ff_z - ANG_DAMPING[2] * float(imu_ang[2]),
                ]], dtype=np.float32)
            else:
                body_torque = np.array(
                    [[tau_ff_x, tau_ff_y, tau_ff_z]], dtype=np.float32
                )

            com_pos = np.array(
                [[float(com_local_base[0]),
                  float(com_local_base[1]),
                  float(com_local_base[2])]],
                dtype=np.float32,
            )
            base.apply_forces_and_torques_at_pos(
                forces=drag_force,
                torques=body_torque,
                positions=com_pos,
                local_frame=True,
            )
        except Exception as e:
            print("apply forces/torques error:", repr(e))
            return

        # Unified flight log: state computed this tick (sent to SITL iff home_locked).
        # CAPSTONE: include wind_world in the log when a wind disturbance is active,
        # so wind is reproducible and plotable from the log alone.
        wind_log = None
        if profile is not None and profile.wind.sigma_mps > 0.0:
            wind_log = wind_world
        # Per-tick "what is the drone carrying / pushing right now":
        #   - payload_kg is the disturbance's current extra mass (mass_drop
        #     reads time-varying, worst_case constant, others zero).
        #   - thrust_total_N sums the per-motor thrust we applied this tick,
        #     after lag, lift, and ground effect.
        # Computed inline from `mass_load_N` instead of calling the
        # convenience `payload_kg()` method, so this works even when
        # Isaac's Script Editor serves a stale capstone.control.disturbance
        # that predates the helper. (See feedback_isaac_script_editor_caching.)
        if profile is not None:
            payload_kg = -float(profile.mass_load_N(float(total_mass))) / 9.81
        else:
            payload_kg = 0.0
        thrust_total_N = float(np.sum(per_motor_thrust))
        if _ARDUPILOT_BRIDGE_LOGGER is not None:
            _ARDUPILOT_BRIDGE_LOGGER.log_state_sent(
                sim_time,
                gyro_frd=gyro_body,
                accel_frd=accel_body,
                pos_ned=pos_ned,
                vel_ned=vel_ned,
                rpy=(roll, pitch, yaw),
                home_locked=home_locked,
                payload_kg=payload_kg,
                thrust_total_N=thrust_total_N,
                wind_world=wind_log,
                gyro_frd_truth=gyro_truth_log,
                accel_frd_truth=accel_truth_log,
            )

        # -------------------------------------------------
        # 5) Debug print — rate-limited only
        # -------------------------------------------------
        now_wall = time.time()
        if now_wall - last_debug_print >= 1.0 / DEBUG_PRINT_HZ:
            hz = (1.0 / dt) if dt > 0 else 0.0
            wind_str = ""
            if profile is not None and profile.wind.sigma_mps > 0.0:
                wind_mag = float(np.linalg.norm(wind_world))
                wind_str = (f" | wind_world={np.round(wind_world, 2).tolist()} "
                            f"|w|={wind_mag:.2f} m/s")
            print(
                f"sim_t={sim_time:.3f}s | dt={dt:.6f}s ({hz:.1f} Hz) | "
                f"home_locked={home_locked} | heading={heading_deg:.1f} | "
                f"PWM={np.round(last_pwm, 1).tolist()} | "
                f"thrust={np.round(per_motor_thrust, 3).tolist()} | "
                f"imu_valid={imu_valid} | "
                f"gyro={np.round(gyro_body, 3).tolist()} | "
                f"accel={np.round(accel_body, 3).tolist()} | "
                f"pos_ned={np.round(pos_ned, 3).tolist()} | "
                f"vel_ned={np.round(vel_ned, 3).tolist()} | "
                f"rpy={[round(roll, 3), round(pitch, 3), round(yaw, 3)]} | "
                f"speed={speed:.3f}"
                f"{wind_str}"
            )
            last_debug_print = now_wall

        if np.any(last_pwm > 1000.0) and now_wall - last_pwm_debug_print >= 1.0 / PWM_DEBUG_PRINT_HZ:
            print(
                f"PWM DEBUG | sim_t={sim_time:.3f}s | PWM={np.round(last_pwm, 1).tolist()} | "
                f"omega_cmd={np.round(omega_cmd, 1).tolist()} | "
                f"omega_actual={np.round(omega_actual, 1).tolist()} | "
                f"thrust={np.round(per_motor_thrust, 3).tolist()} | "
                f"yaw_torque={np.round(per_motor_yaw_torque, 4).tolist()}"
            )
            last_pwm_debug_print = now_wall
        # -------------------------------------------------
        # 6) Do not send JSON until home is locked
        # -------------------------------------------------
        if not home_locked:
            return

        # -------------------------------------------------
        # 7) Build and send JSON state packet
        # -------------------------------------------------
        try:
            # Important: this timestamp is Isaac physics time, not wall-clock time.
            reply = {
                "timestamp": sim_time,
                "imu": {
                    "gyro": gyro_body.tolist(),
                    "accel_body": accel_body.tolist(),
                },
                "position": pos_ned.tolist(),
                "velocity": vel_ned.tolist(),
                "attitude": [float(roll), float(pitch), float(yaw)],
            }

            if last_addr is not None:
                payload = ("\n" + json.dumps(reply, separators=(",", ":")) + "\n").encode("utf-8")
                udp_sock.sendto(payload, last_addr)

        except Exception as e:
            print("State/send error:", repr(e))
            return

    sim.add_physics_callback("ardupilot_bridge", physics_step)
    print("Physics callback installed.")
    print("Bridge is running.")


async def start_bridge():
    global _ARDUPILOT_BRIDGE_RUNNING

    if _ARDUPILOT_BRIDGE_RUNNING:
        print("Bridge already running. Call stop_bridge() first if you want to restart it.")
        return

    _ARDUPILOT_BRIDGE_RUNNING = True

    try:
        await setup_bridge()
    except Exception as e:
        print("Bridge failed to start:", repr(e))
        stop_bridge()


def run_bridge():
    """
    Start exactly one bridge task.
    In Isaac Script Editor, pressing Run on this file once should start the bridge.
    """
    global _ARDUPILOT_BRIDGE_TASK

    if _ARDUPILOT_BRIDGE_TASK is not None and not _ARDUPILOT_BRIDGE_TASK.done():
        print("Bridge task already exists.")
        return

    _ARDUPILOT_BRIDGE_TASK = asyncio.ensure_future(start_bridge())


# =========================================================
# START HERE
# =========================================================
# Press Run once in Isaac Script Editor.
# To stop later, run:
#     stop_bridge()
run_bridge()