# Motor convention (ArduPilot QuadX)

The bridge sends/receives 4 PWM channels in this order, and assumes these spin directions. Verified against `ardupilot/libraries/AP_Motors/AP_MotorsMatrix.cpp:592-601` (`MOTOR_FRAME_TYPE_X`).

| Channel | SERVO | Position | Spin (viewed from above) | Angle from forward |
|--------:|------:|----------|--------------------------|-------------------:|
| pwm[0] | SERVO1 | front-right (FR) | CCW | +45° |
| pwm[1] | SERVO2 | rear-left  (RL) | CCW | -135° |
| pwm[2] | SERVO3 | front-left (FL) | CW  | -45° |
| pwm[3] | SERVO4 | rear-right (RR) | CW  | +135° |

In the bridge (`lab/bridge/Nvidia_SITL_connecter.py`):

- `MOTOR_LINK_PATHS` is in `[FR, RL, FL, RR]` order to match.
- `MOTOR_SPIN_DIR = [-1, -1, +1, +1]` is the **body reaction-torque sign** (CCW prop pushes the body CW = -Z body, so CCW prop → -1).
- `ROTOR_SPIN_SIGN = -MOTOR_SPIN_DIR = [+1, +1, -1, -1]` is the **rotor's own angular velocity sign in body +Z**, used for visual propeller spin.

If you ever rewire motors or change frame type, both the bridge constants and this table must be updated together.
