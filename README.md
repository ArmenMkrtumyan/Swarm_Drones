# Swarm_Drones

Bridge between Isaac Sim and ArduPilot SITL for the F450 capstone, plus the `capstone/` package for stage tooling and disturbance harness.

## Layout

- `my_drone_simulation/` — Isaac↔SITL bridge (`Nvidia_SITL_connecter.py`), takeoff and mission scripts.
- `capstone/` — stage gates, disturbance profiles, metrics, mission runner.
- `sitl_params/` — ArduCopter SITL param files; `sitl_params_test.parm` is the active one loaded by the WSL launch command.
- `scene/` — drone USDs.
- `optimization/` — tuning experiments.

## Motor convention (ArduPilot QuadX)

The bridge sends/receives 4 PWM channels in this order, and assumes these spin directions. Verified against `ardupilot/libraries/AP_Motors/AP_MotorsMatrix.cpp:592-601` (`MOTOR_FRAME_TYPE_X`).

| Channel | SERVO | Position | Spin (viewed from above) | Angle from forward |
|--------:|------:|----------|--------------------------|-------------------:|
| pwm[0] | SERVO1 | front-right (FR) | CCW | +45° |
| pwm[1] | SERVO2 | rear-left  (RL) | CCW | -135° |
| pwm[2] | SERVO3 | front-left (FL) | CW  | -45° |
| pwm[3] | SERVO4 | rear-right (RR) | CW  | +135° |

In the bridge:
- `MOTOR_LINK_PATHS` is in `[FR, RL, FL, RR]` order to match.
- `MOTOR_SPIN_DIR = [-1, -1, +1, +1]` is the **body reaction-torque sign** (CCW prop pushes the body CW = -Z body, so CCW prop → -1).
- `ROTOR_SPIN_SIGN = -MOTOR_SPIN_DIR = [+1, +1, -1, -1]` is the **rotor's own angular velocity sign in body +Z**, used for visual propeller spin.

If you ever rewire motors or change frame type, both the bridge constants and this table must be updated together.

## Network ports

- UDP 9002 — Isaac↔SITL state and PWM (JSON FDM).
- UDP 14551 — MAVLink to ArduCopter SITL (used by `arm_takeoff.py`, mission scripts, `capstone/missions/runner.py`).

## SITL launch

Run from WSL inside `ardupilot/`:

```bash
Tools/autotest/sim_vehicle.py -v ArduCopter -f X -N -w \
  --add-param-file=/mnt/c/Users/user1811/Desktop/armen-capstone/Swarm_Drones/sitl_params/sitl_params_test.parm \
  -A '--home 40.192,44.50446,1200,0' \
  --model JSON:192.168.208.1 \
  --map --console \
  --out=udp:127.0.0.1:14551
```
