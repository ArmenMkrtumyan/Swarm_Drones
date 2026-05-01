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
# armen-capstone/flight_logs/. Each line is a single event with:
#   t:    seconds since bridge module import (matches the timestamp sent to SITL)
#   src:  one of "sitl->isaac" | "isaac->sitl" | "bridge"
# Then source-specific payload fields. Post-flight analysis just reads the
# file top to bottom — both sides of the loop are on one timeline.
#
# Rate limits:
#   - sitl->isaac: every packet (SITL sends ~SCHED_LOOP_RATE per second, so ~100 Hz)
#   - isaac->sitl: at most once per STATE_LOG_PERIOD_S (50 Hz keeps files small)
#   - bridge:      only on lifecycle events (startup, home_locked, shutdown, errors)
STATE_LOG_PERIOD_S = 0.02   # 50 Hz

FLIGHT_LOG_DIR = r"C:\Users\user1811\Desktop\armen-capstone\flight_logs"


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

    def log_sitl_packet(self, t, pwm, magic, frame_count, addr):
        entry = {
            "t": round(t, 4),
            "src": "sitl->isaac",
            "pwm": [float(x) for x in pwm.tolist()[:4]],
            "magic": int(magic),
            "frame": int(frame_count),
            "addr": list(addr) if addr else None,
        }
        self._write(entry)

    def log_state_sent(self, t, *, gyro_frd, accel_frd, pos_ned, vel_ned,
                       rpy, heading_deg, home_locked):
        if t - self._last_state_t < STATE_LOG_PERIOD_S:
            return
        self._last_state_t = t
        self._write({
            "t": round(t, 4),
            "src": "isaac->sitl",
            "gyro_frd": [round(float(x), 4) for x in gyro_frd],
            "accel_frd": [round(float(x), 4) for x in accel_frd],
            "pos_ned": [round(float(x), 4) for x in pos_ned],
            "vel_ned": [round(float(x), 4) for x in vel_ned],
            "rpy": [round(float(x), 4) for x in rpy],
            "heading_deg": round(float(heading_deg), 2),
            "home_locked": bool(home_locked),
        })

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
# The Hawks F450 body frame is RFU (+X=right, +Y=forward, +Z=up), verified from
# motor-link positions. ArduPilot uses FRD (+X=forward, +Y=right, +Z=down).
# T is also its own inverse (T²=I): handy because the same matrix also converts
# ENU-aligned-to-initial-heading → NED for position/velocity below.
T_RFU_TO_FRD = np.array([
    [0.0, 1.0,  0.0],
    [1.0, 0.0,  0.0],
    [0.0, 0.0, -1.0],
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
MOTOR_TIME_CONSTANT_S = 0.03        # first-order response from commanded to actual omega
K_TORQUE_OVER_K_THRUST = 0.02       # reaction-torque / thrust ratio (m); ~0.02 for 8–10" props

# Reaction-torque sign per motor, applied as K_Q * omega² * MOTOR_SPIN_DIR
# about body +Z (RFU, +Z = up). Derivation:
#   A CCW prop (viewed from above) is pushed by the motor with +Z body torque;
#   Newton's 3rd law puts the reaction on the body as -Z body torque (CW).
# So CCW-prop motors use -1 here; CW-prop motors use +1.
# Matches ArduPilot QuadX: pwm[0]=FR=CCW, pwm[1]=RL=CCW, pwm[2]=FL=CW, pwm[3]=RR=CW.
MOTOR_SPIN_DIR = np.array([-1.0, -1.0, +1.0, +1.0], dtype=np.float32)

# Thrust axis = body +Z in RFU (= world +Z at rest).
MOTOR_THRUST_DIR_LOCAL = np.array([0.0, 0.0, 1.0], dtype=np.float32)

# Motor positions relative to base_link (RFU body), in meters. Same order as
# MOTOR_LINK_PATHS: [FR, RL, FL, RR]. Verified from USD probe. These are used
# to compute the net roll/pitch torque analytically so we can apply the entire
# wrench (force + torque) to the articulation root (base_link) in one call.
# This avoids subtle force-propagation quirks when applying per-motor thrust
# to non-root bodies of a PhysX articulation.
MOTOR_ARM_LENGTH = 0.159   # motor X/Y coord from base origin (= 225 mm half-diagonal / sqrt(2)); F450 has 450 mm motor-to-motor diagonal
MOTOR_POS_REL_BASE = np.array([
    [+MOTOR_ARM_LENGTH, +MOTOR_ARM_LENGTH, 0.0],  # FR: +X right, +Y forward
    [-MOTOR_ARM_LENGTH, -MOTOR_ARM_LENGTH, 0.0],  # RL
    [-MOTOR_ARM_LENGTH, +MOTOR_ARM_LENGTH, 0.0],  # FL
    [+MOTOR_ARM_LENGTH, -MOTOR_ARM_LENGTH, 0.0],  # RR
], dtype=np.float64)

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
        # semantics (v' = v * M), so the rotation part's columns are the body
        # axes expressed in world. Reading m[i][j] directly into R[i,j] keeps
        # that convention; to convert world -> body we multiply by R.T later.
        m = _world_mat(prim)
        R = np.empty((3, 3), dtype=np.float64)
        for i in range(3):
            for j in range(3):
                R[i, j] = m[i][j]
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
    global _ARDUPILOT_BRIDGE_SOCKET

    if _ARDUPILOT_BRIDGE_SOCKET is not None:
        try:
            _ARDUPILOT_BRIDGE_SOCKET.close()
            print("Closed old UDP socket.")
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
        time.perf_counter() - t0, "bridge_setup_started",
        robot=ROBOT_PATH,
        motor_order=["FR_CCW", "RL_CCW", "FL_CW", "RR_CW"],
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
            time.perf_counter() - t0, "motor_model_calibrated",
            total_mass_kg=float(total_mass),
            omega_hover_rad_s=float(omega_hover),
            K_thrust=float(K_THRUST),
            K_torque=float(K_TORQUE),
            com_local_base_m=[float(x) for x in com_local_base],
            physics_hz=float(current_hz),
        )

    # First-order lag state for rotor angular velocity (per-motor)
    omega_actual = np.zeros(4, dtype=np.float32)

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
    imu_warned_invalid = False

    print("Bridge init complete.")
    print("Initial world position:", home_pos_world)
    print("Listening on UDP 9002...")
    print("Waiting to lock home after the vehicle settles.")

    def physics_step(dt):
        nonlocal home_pos_world, home_R0_world_body, home_locked, settle_start
        nonlocal last_addr, first_packet_seen, last_pwm, last_debug_print, imu_warned_invalid
        nonlocal omega_actual

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
                    _ARDUPILOT_BRIDGE_LOGGER.log_event(
                        time.perf_counter() - t0, "first_sitl_packet",
                        addr=list(addr),
                    )

            last_addr = addr
            last_pwm = pkt["pwm"][:4].copy()
            if _ARDUPILOT_BRIDGE_LOGGER is not None:
                _ARDUPILOT_BRIDGE_LOGGER.log_sitl_packet(
                    time.perf_counter() - t0,
                    pkt["pwm"],
                    pkt["magic"],
                    pkt["frame_count"],
                    addr,
                )

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

                # Body RFU (Isaac) -> Body FRD (ArduPilot). Same matrix for gyro and accel.
                gyro_body = T_RFU_TO_FRD @ imu_ang
                accel_body = T_RFU_TO_FRD @ imu_acc
            else:
                if not imu_warned_invalid:
                    print("WARNING: IMU reading invalid. Check IMU_SENSOR_PATH.")
                    imu_warned_invalid = True

                gyro_body = np.zeros(3, dtype=np.float64)
                accel_body = np.zeros(3, dtype=np.float64)

            speed = float(np.linalg.norm(vel_world))
            gyro_norm = float(np.linalg.norm(gyro_body))

            # R_world_body in Isaac conventions (body RFU, world Z-up).
            R_world_body = quat_wxyz_to_rot(quat_world)

            # Express body orientation relative to the drone's initial attitude at home.
            # R_rel is body-at-now in the drone's initial body-frame coordinates.
            R_rel = home_R0_world_body.T @ R_world_body

            # Body-RFU-in-initial-frame  ->  Body-FRD-in-NED
            R_ned_body_frd = T_RFU_TO_FRD @ R_rel @ T_RFU_TO_FRD

            roll, pitch, yaw = rot_to_rpy_zyx(R_ned_body_frd)
            heading_deg = (math.degrees(yaw) + 360.0) % 360.0

            # World-frame vectors: rotate into the drone's initial body frame (RFU),
            # then relabel axes to NED.
            rel_world = pos_world - home_pos_world
            pos_init = home_R0_world_body.T @ rel_world
            vel_init = home_R0_world_body.T @ vel_world

            pos_ned = T_RFU_TO_FRD @ pos_init
            vel_ned = T_RFU_TO_FRD @ vel_init

        except Exception as e:
            print("State/read error:", repr(e))
            return

        # -------------------------------------------------
        # 3) Lock home only after the vehicle is settled
        # -------------------------------------------------
        now_perf = time.perf_counter()
        if not home_locked:
            if speed < MAX_SETTLE_SPEED and gyro_norm < MAX_SETTLE_GYRO:
                if settle_start is None:
                    settle_start = now_perf
                elif (now_perf - settle_start) >= REQUIRED_SETTLE_SECONDS:
                    home_pos_world = pos_world.copy()
                    # Anchor NED "north" to the drone's current forward direction.
                    home_R0_world_body = R_world_body.copy()
                    home_locked = True
                    print(f"Home locked after settling: pos={home_pos_world}")
                    print(f"  initial body-to-world rotation (used as NED anchor):\n{home_R0_world_body}")
                    if _ARDUPILOT_BRIDGE_LOGGER is not None:
                        _ARDUPILOT_BRIDGE_LOGGER.log_event(
                            time.perf_counter() - t0, "home_locked",
                            home_pos_world=home_pos_world.tolist(),
                            R0_world_body=home_R0_world_body.tolist(),
                        )
            else:
                settle_start = None

        # -------------------------------------------------
        # 4) PWM -> commanded omega -> first-order lag -> thrust (omega^2) + yaw torque
        # -------------------------------------------------
        omega_cmd = pwm_to_omega_cmd(last_pwm)
        alpha = min(1.0, dt / MOTOR_TIME_CONSTANT_S)
        omega_actual += (omega_cmd - omega_actual) * alpha

        omega_sq = omega_actual * omega_actual
        per_motor_thrust = K_THRUST * omega_sq                         # N per motor, along local +Z
        per_motor_yaw_torque = K_TORQUE * omega_sq * MOTOR_SPIN_DIR    # N·m per motor, about local +Z

        # Do not apply forces before home lock
        if not home_locked:
            per_motor_thrust[:] = 0.0
            per_motor_yaw_torque[:] = 0.0

        # -------------------------------------------------
        # 4b) Apply single wrench to the articulation root (base_link) AT the
        # composite COM. Motor thrusts are (0, 0, F_i) at positions
        # (x_i, y_i, 0) in body RFU; torque about the COM is:
        #   torque_i = (r_i - com) × F_i
        #            = ((y_i - com_y) F_i, -(x_i - com_x) F_i, 0)
        # Applying the aggregated force *at* com_local_base means PhysX adds no
        # extra lever-arm term from the application-point-vs-COM gap, so a
        # symmetric collective produces zero net roll/pitch torque.
        # -------------------------------------------------
        try:
            F_total_z = float(np.sum(per_motor_thrust))
            total_force = np.array([[0.0, 0.0, F_total_z]], dtype=np.float32)

            dx = MOTOR_POS_REL_BASE[:, 0] - com_local_base[0]
            dy = MOTOR_POS_REL_BASE[:, 1] - com_local_base[1]
            tau_x = float(np.sum(dy * per_motor_thrust))   # (y_i - com_y) * F_i
            tau_y = float(np.sum(-dx * per_motor_thrust))  # -(x_i - com_x) * F_i
            tau_z = float(np.sum(per_motor_yaw_torque))
            total_torque = np.array([[tau_x, tau_y, tau_z]], dtype=np.float32)

            positions = np.array(
                [[com_local_base[0], com_local_base[1], com_local_base[2]]],
                dtype=np.float32,
            )

            base.apply_forces_and_torques_at_pos(
                forces=total_force,
                torques=total_torque,
                positions=positions,
                local_frame=True,
            )
        except Exception as e:
            print("apply forces/torques error:", repr(e))
            return

        # Unified flight log: state computed this tick (sent to SITL iff home_locked)
        if _ARDUPILOT_BRIDGE_LOGGER is not None:
            _ARDUPILOT_BRIDGE_LOGGER.log_state_sent(
                time.perf_counter() - t0,
                gyro_frd=gyro_body,
                accel_frd=accel_body,
                pos_ned=pos_ned,
                vel_ned=vel_ned,
                rpy=(roll, pitch, yaw),
                heading_deg=heading_deg,
                home_locked=home_locked,
            )

        # -------------------------------------------------
        # 5) Debug print
        # -------------------------------------------------
        now = time.time()
        if now - last_debug_print > 1.0:
            hz = (1.0 / dt) if dt > 0 else 0.0
            print(
                f"dt={dt:.6f}s ({hz:.1f} Hz) | "
                f"home_locked={home_locked} | "
                f"heading={heading_deg:.1f} | "
                f"PWM={last_pwm.tolist()} | "
                f"thrust={np.round(per_motor_thrust, 3).tolist()} | "
                f"imu_valid={imu_valid} | "
                f"gyro={np.round(gyro_body, 3).tolist()} | "
                f"accel={np.round(accel_body, 3).tolist()} | "
                f"speed={speed:.3f}"
            )

            print(
                f"send pos_ned={np.round(pos_ned, 3).tolist()} | "
                f"vel_ned={np.round(vel_ned, 3).tolist()} | "
                f"attitude={[round(roll, 3), round(pitch, 3), round(yaw, 3)]}"
            )

            last_debug_print = now
        if np.any(last_pwm > 1000.0):
            print(
                f"ARMED PWM DEBUG | PWM={last_pwm.tolist()} | "
                f"omega_cmd={np.round(omega_cmd, 1).tolist()} | "
                f"omega_actual={np.round(omega_actual, 1).tolist()} | "
                f"thrust={np.round(per_motor_thrust, 3).tolist()} | "
                f"yaw_torque={np.round(per_motor_yaw_torque, 4).tolist()}"
            )
        print(
            f"send pos_ned={np.round(pos_ned, 3).tolist()} | "
            f"vel_ned={np.round(vel_ned, 3).tolist()} | "
            f"attitude={[round(roll, 3), round(pitch, 3), round(yaw, 3)]}"
        )
        # -------------------------------------------------
        # 6) Do not send JSON until home is locked
        # -------------------------------------------------
        if not home_locked:
            return

        # -------------------------------------------------
        # 7) Build and send JSON state packet
        # -------------------------------------------------
        try:
            reply = {
                "timestamp": time.perf_counter() - t0,
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