import asyncio
import socket
import struct
import json
import time
import math
import numpy as np

import omni.usd
import omni.kit.app
import omni.timeline

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


# =========================================================
# PATHS
# =========================================================
ROBOT_PATH = "/World/hawks_work_f450_jetsonnano_steereocam"
BASE_LINK_PATH = f"{ROBOT_PATH}/base_link"
IMU_SENSOR_PATH = f"{BASE_LINK_PATH}/Imu_Sensor"

MOTOR_LINK_PATHS = [
    f"{ROBOT_PATH}/front_left_motor_link",
    f"{ROBOT_PATH}/front_right_motor_link",
    f"{ROBOT_PATH}/rear_left_motor_link",
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
# Adjust only if heading gets worse instead of better.
YAW_OFFSET_DEG = 96.0

# Isaac local body frame (FLU) -> ArduPilot body frame (FRD)
T_FLU_TO_FRD = np.diag([1.0, -1.0, -1.0])

# =========================================================
# THRUST TUNING (CONSERVATIVE DEBUG VALUES)
# =========================================================
HOVER_THRUST = np.array([6.2, 6.2, 5, 5], dtype=np.float32)

PWM_IDLE = 1000.0
PWM_HOVER = 1500.0
THRUST_SCALE = 0.20

# Assumes each motor link's local +Z is the thrust axis
MOTOR_THRUST_DIR_LOCAL = np.array([0.0, 0.0, 1.0], dtype=np.float32)

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


def pwm_to_hover_norm(pwm_us: np.ndarray) -> np.ndarray:
    """
    Convert PWM into a simple normalized thrust command around hover.
    1000 -> 0.0
    1500 -> 1.0
    clipped to avoid absurd thrust during early bring-up
    """
    x = (pwm_us - PWM_IDLE) / (PWM_HOVER - PWM_IDLE)
    return np.clip(x, 0.0, 2.2)


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


def rotz_ned_deg(deg: float):
    a = math.radians(deg)
    c = math.cos(a)
    s = math.sin(a)
    return np.array([
        [c, -s, 0.0],
        [s,  c, 0.0],
        [0.0, 0.0, 1.0],
    ], dtype=np.float64)


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

    _ARDUPILOT_BRIDGE_TASK = None
    _ARDUPILOT_BRIDGE_RUNNING = False
    print("Bridge stopped.")


async def setup_bridge():
    global _ARDUPILOT_BRIDGE_SOCKET
    global _ARDUPILOT_BRIDGE_SIM

    print("RUNNING NEW BRIDGE v2 - local_frame=True, THRUST_SCALE=0.20")
    print("=== ArduPilot <-> Isaac Bridge Setup ===")

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

    await app.next_update_async()

    try:
        positions, orientations = base.get_world_poses()
        home_pos_world = np.array(positions[0], dtype=np.float64)
    except Exception as e:
        raise RuntimeError(f"Failed reading base pose: {repr(e)}")

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
        nonlocal home_pos_world, home_locked, settle_start
        nonlocal last_addr, first_packet_seen, last_pwm, last_debug_print, imu_warned_invalid

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

            last_addr = addr
            last_pwm = pkt["pwm"][:4].copy()

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

            R_YAW = rotz_ned_deg(YAW_OFFSET_DEG)

            R_world_body_flu = quat_wxyz_to_rot(quat_world)
            R_ned0_body_frd = T_FLU_TO_FRD @ R_world_body_flu @ T_FLU_TO_FRD
            R_ned_body_frd = R_YAW @ R_ned0_body_frd

            roll, pitch, yaw = rot_to_rpy_zyx(R_ned_body_frd)
            heading_deg = (math.degrees(yaw) + 360.0) % 360.0

            rel_world = pos_world - home_pos_world

            pos_ned0 = np.array([
                rel_world[0],
                -rel_world[1],
                -rel_world[2],
            ], dtype=np.float64)

            vel_ned0 = np.array([
                vel_world[0],
                -vel_world[1],
                -vel_world[2],
            ], dtype=np.float64)

            pos_ned = R_YAW @ pos_ned0
            vel_ned = R_YAW @ vel_ned0

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
                    home_locked = True
                    print(f"Home locked after settling: {home_pos_world}")
            else:
                settle_start = None

        # -------------------------------------------------
        # 4) Convert PWM -> thrust and apply in LOCAL frame
        # -------------------------------------------------
        thrust_norm = pwm_to_hover_norm(last_pwm)
        per_motor_thrust = THRUST_SCALE * HOVER_THRUST * thrust_norm

        # Do not apply thrust before home lock
        if not home_locked:
            per_motor_thrust[:] = 0.0

        for body, thrust in zip(motor_bodies, per_motor_thrust):
            if thrust <= 0.0:
                continue
            try:
                force = (MOTOR_THRUST_DIR_LOCAL * float(thrust)).reshape(1, 3).astype(np.float32)
                body.apply_forces(force, local_frame=True)
            except Exception as e:
                print("apply_forces error:", repr(e))
                return

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
                f"thrust_norm={np.round(thrust_norm, 3).tolist()} | "
                f"per_motor_thrust={np.round(per_motor_thrust, 4).tolist()}"
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