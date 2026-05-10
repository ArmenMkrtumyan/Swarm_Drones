"""Vectorized torch quadrotor dynamics for Stage-3 Phase-1 pre-training.

Pure functions over tensors with a leading batch dimension. No Isaac, no
SITL, no MAVLink -- just math. Mirrors the F450 physics constants used by
the live bridge (Swarm_Drones/my_drone_simulation/Nvidia_SITL_connecter.py)
so the policy trained against this module transfers to the real bridge.

Modules:
- motor_model     : PWM -> omega -> thrust + reaction torque, with first-order
                    motor lag, translational lift, ground effect, body drag.
- body            : rigid-body 6-DOF integration with quaternion attitude.
- atc_psc_mirror  : cascaded angle->rate PID + altitude PSC + F450 X-mixer.
"""
