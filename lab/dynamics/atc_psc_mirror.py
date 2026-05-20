"""Approximate ArduPilot ATC + PSC mirror — torch-vectorized.

Goal: produce a controller with the *same structure* as ArduPilot's attitude
+ position stack, so a policy that learns "increase ATC_RAT_PIT_P → less
overshoot" transfers from this mirror to real ArduPilot SITL.

What this is NOT: a perfect re-implementation of ArduPilot. We deliberately
skip notch filters, advanced anti-windup, output saturation handling, and
controller-mode switches. Domain randomization in the env covers the gap
between this mirror and real ArduPilot; Phase 2 SITL fine-tuning closes the
last mile.

Structure (per env, vectorized over batch dim B):

    altitude_err  --[PSC_POSZ_P]-->  vel_z_target
    vel_z_target - vel_z --[PSC_VELZ_P + I + D]--> accel_z_demand
    accel_z_demand * mass + m*g  -->  thrust_total_demand (N)

    attitude_err (roll, pitch) --[ATC_ANG_*_P]--> rate_target (body)
    rate_err = rate_target - gyro_body
    rate_err --[ATC_RAT_*_{P,I,D}]--> torque_body_demand (3-vec, Nm)

    F450 X-mixer:
        per_motor_thrust = thrust_total/4 + roll_term + pitch_term + yaw_term
        omega_cmd = sqrt(thrust / K_T)
        pwm_cmd = PWM_IDLE + omega/OMEGA_MAX * (PWM_MAX - PWM_IDLE)

The full state of the controller is the integrators (I-terms) and the previous
rate error (for D-term). They are returned alongside the PWM output so the env
can re-feed them next step.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import NamedTuple

import torch
from torch import Tensor

from lab.dynamics.motor_model import (
    OMEGA_MAX_RAD_S, PWM_IDLE, PWM_MAX, MOTOR_ARM_LENGTH, K_TORQUE_OVER_K_THRUST,
    MOTOR_SPIN_DIR,
)


# -----------------------------------------------------------------------------
# Gain container (the *target* of RL adaptation)
# -----------------------------------------------------------------------------


@dataclass
class PidGains:
    """All controller gains, per env. All tensors shape ``(B,)`` unless noted.

    Names match ArduPilot's MAVLink param names so the policy's gain-delta
    semantics are 1:1 between Phase 1 and Phase 2 (where the same param names
    drive real ArduPilot).
    """
    # Position controller (altitude only in v0).
    PSC_POSZ_P:  Tensor
    PSC_VELZ_P:  Tensor
    PSC_VELZ_I:  Tensor
    PSC_VELZ_D:  Tensor
    # Attitude controller, outer angle loop (P only — matches ArduCopter default).
    ATC_ANG_RLL_P: Tensor
    ATC_ANG_PIT_P: Tensor
    ATC_ANG_YAW_P: Tensor
    # Attitude controller, inner rate loop (P + I + D).
    ATC_RAT_RLL_P: Tensor;  ATC_RAT_RLL_I: Tensor;  ATC_RAT_RLL_D: Tensor
    ATC_RAT_PIT_P: Tensor;  ATC_RAT_PIT_I: Tensor;  ATC_RAT_PIT_D: Tensor
    ATC_RAT_YAW_P: Tensor;  ATC_RAT_YAW_I: Tensor;  ATC_RAT_YAW_D: Tensor

    @staticmethod
    def make_default(B: int, *, device="cpu", dtype=torch.float32) -> "PidGains":
        """Stock ArduCopter F450 defaults — same numbers used in
        :data:`lab.rl.envs.hover_pid_tuner_v0.DEFAULT_GAINS`."""
        kw = {"device": device, "dtype": dtype}
        def f(v): return torch.full((B,), float(v), **kw)
        return PidGains(
            PSC_POSZ_P=f(1.0),
            PSC_VELZ_P=f(5.0), PSC_VELZ_I=f(2.0), PSC_VELZ_D=f(0.0),
            ATC_ANG_RLL_P=f(4.5), ATC_ANG_PIT_P=f(4.5), ATC_ANG_YAW_P=f(4.5),
            ATC_RAT_RLL_P=f(0.135), ATC_RAT_RLL_I=f(0.135), ATC_RAT_RLL_D=f(0.0036),
            ATC_RAT_PIT_P=f(0.135), ATC_RAT_PIT_I=f(0.135), ATC_RAT_PIT_D=f(0.0036),
            ATC_RAT_YAW_P=f(0.18),  ATC_RAT_YAW_I=f(0.018), ATC_RAT_YAW_D=f(0.0),
        )


# -----------------------------------------------------------------------------
# Controller state (integrators + previous rate error for D)
# -----------------------------------------------------------------------------


@dataclass
class ControllerState:
    """Per-env integrator + prev-error state. All shape ``(B,)`` except where noted."""
    velz_integral:    Tensor   # (B,)  PSC vel-z I-term integrator
    rate_integral:    Tensor   # (B, 3)  ATC rate I-term per-axis
    prev_rate_err:    Tensor   # (B, 3)  for D-term
    velz_err_prev:    Tensor   # (B,)    for PSC D-term

    @staticmethod
    def zeros(B: int, *, device="cpu", dtype=torch.float32) -> "ControllerState":
        kw = {"device": device, "dtype": dtype}
        return ControllerState(
            velz_integral=torch.zeros(B, **kw),
            rate_integral=torch.zeros(B, 3, **kw),
            prev_rate_err=torch.zeros(B, 3, **kw),
            velz_err_prev=torch.zeros(B, **kw),
        )


# -----------------------------------------------------------------------------
# Mixer geometry constants — per F450 X-config
# -----------------------------------------------------------------------------

# Per-motor sign patterns for (roll, pitch, yaw) demand decomposition. Index
# order [FR, RL, FL, RR] matches the bridge / motor_model.MOTOR_POS_BODY.
#
# Derivation: motor thrust at body position (x, y) along +Z produces torque
# (T*y, -T*x, 0). For unit thrust at each motor, the y-coefficient gives the
# roll mixing, the (-x)-coefficient gives the pitch mixing.
#
# For F450:
#   FR (+L, -L) -> roll=-1, pitch=-1
#   RL (-L, +L) -> roll=+1, pitch=+1
#   FL (+L, +L) -> roll=+1, pitch=-1
#   RR (-L, -L) -> roll=-1, pitch=+1
#
# Yaw mixing follows the spin direction: CW props (FL, RR) get +1, CCW (FR, RL)
# get -1 — same as the body's yaw reaction torque sign.
ROLL_PATTERN  = (-1.0, +1.0, +1.0, -1.0)
PITCH_PATTERN = (-1.0, +1.0, -1.0, +1.0)
YAW_PATTERN   = MOTOR_SPIN_DIR


# -----------------------------------------------------------------------------
# Reference / target container
# -----------------------------------------------------------------------------


@dataclass
class HoverTarget:
    """Reference setpoints for hover (kept simple for v0)."""
    target_alt: Tensor       # (B,)
    target_roll: Tensor      # (B,)  rad — usually 0
    target_pitch: Tensor     # (B,)  rad — usually 0
    target_yaw: Tensor       # (B,)  rad — usually current yaw
    target_vel_z: Tensor     # (B,)  m/s — usually 0

    @staticmethod
    def hover_at(alt: float, B: int, *, device="cpu", dtype=torch.float32):
        kw = {"device": device, "dtype": dtype}
        z = torch.zeros(B, **kw)
        return HoverTarget(
            target_alt=torch.full((B,), float(alt), **kw),
            target_roll=z.clone(), target_pitch=z.clone(),
            target_yaw=z.clone(), target_vel_z=z.clone(),
        )


# -----------------------------------------------------------------------------
# Cascaded controller step
# -----------------------------------------------------------------------------


class ControllerOut(NamedTuple):
    pwm_cmd:        Tensor       # (B, 4) commanded PWM in us
    state:          ControllerState
    debug:          dict         # introspection: thrust_demand, rate_target, etc


def _safe_sqrt_thrust_to_omega(thrust: Tensor, K_thrust: Tensor) -> Tensor:
    """Convert per-motor thrust (N) to omega (rad/s), saturating at OMEGA_MAX.

    Negative thrust demands clamp to 0 (motor can't reverse).
    """
    thrust_clamped = torch.clamp(thrust, min=0.0)
    omega = torch.sqrt(thrust_clamped / K_thrust.view(-1, 1))
    return torch.clamp(omega, max=OMEGA_MAX_RAD_S)


def _omega_to_pwm(omega: Tensor) -> Tensor:
    """Inverse of :func:`motor_model.pwm_to_omega_cmd`. Clamps to [IDLE, MAX]."""
    pwm = PWM_IDLE + (omega / OMEGA_MAX_RAD_S) * (PWM_MAX - PWM_IDLE)
    return torch.clamp(pwm, PWM_IDLE, PWM_MAX)


def cascaded_step(
    *,
    pos_z: Tensor,            # (B,) world Z (m)
    vel_z: Tensor,            # (B,) world Z velocity (m/s, +up)
    roll: Tensor,             # (B,) rad
    pitch: Tensor,            # (B,) rad
    yaw: Tensor,              # (B,) rad
    gyro_body: Tensor,        # (B, 3) rad/s body
    target: HoverTarget,
    gains: PidGains,
    state: ControllerState,
    K_thrust: Tensor,         # (B,) for omega <-> thrust mapping (from MotorParams)
    mass_kg: Tensor,          # (B,)
    dt: float,
    integral_clamp_velz: float = 5.0,
    integral_clamp_rate: float = 1.0,
) -> ControllerOut:
    """One cascaded PID step. Returns commanded PWM and updated controller state.

    The cascading happens in three nested loops (matching ArduPilot's
    structure):

    1. **PSC altitude**: alt_err -> vel_z_target -> accel_z_demand
       -> thrust_total_demand (N).
    2. **ATC outer angle**: attitude_err -> rate_target (body).
    3. **ATC inner rate**: rate_err -> body torque demand (Nm).

    Then the F450 X-mixer turns (thrust_total, tau_x, tau_y, tau_z) into 4
    per-motor thrusts, and finally PWM via the inverse motor-model map.
    """
    g = 9.81

    # -- Stage 1: altitude / vertical-velocity outer + inner ------------------
    alt_err = target.target_alt - pos_z
    vel_z_target = target.target_vel_z + gains.PSC_POSZ_P * alt_err
    velz_err = vel_z_target - vel_z

    velz_int_new = torch.clamp(
        state.velz_integral + velz_err * dt,
        -integral_clamp_velz, integral_clamp_velz,
    )
    velz_d = (velz_err - state.velz_err_prev) / dt
    accel_z_demand = (
        gains.PSC_VELZ_P * velz_err
        + gains.PSC_VELZ_I * velz_int_new
        + gains.PSC_VELZ_D * velz_d
    )
    thrust_total = mass_kg * (g + accel_z_demand)             # (B,) total N
    thrust_total = torch.clamp(thrust_total, min=0.0)

    # -- Stage 2: attitude outer (P only, per ArduCopter ATC default) ---------
    roll_err  = target.target_roll  - roll
    pitch_err = target.target_pitch - pitch
    # Yaw error wrapped to [-pi, pi].
    yaw_err = torch.remainder(target.target_yaw - yaw + torch.pi, 2 * torch.pi) - torch.pi
    rate_target = torch.stack([
        gains.ATC_ANG_RLL_P * roll_err,
        gains.ATC_ANG_PIT_P * pitch_err,
        gains.ATC_ANG_YAW_P * yaw_err,
    ], dim=-1)                                                # (B, 3)

    # -- Stage 3: rate inner (P + I + D) --------------------------------------
    rate_err = rate_target - gyro_body                         # (B, 3)
    rate_int_new = torch.clamp(
        state.rate_integral + rate_err * dt,
        -integral_clamp_rate, integral_clamp_rate,
    )
    rate_d = (rate_err - state.prev_rate_err) / dt
    tau_x = (gains.ATC_RAT_RLL_P * rate_err[:, 0]
             + gains.ATC_RAT_RLL_I * rate_int_new[:, 0]
             + gains.ATC_RAT_RLL_D * rate_d[:, 0])
    tau_y = (gains.ATC_RAT_PIT_P * rate_err[:, 1]
             + gains.ATC_RAT_PIT_I * rate_int_new[:, 1]
             + gains.ATC_RAT_PIT_D * rate_d[:, 1])
    tau_z = (gains.ATC_RAT_YAW_P * rate_err[:, 2]
             + gains.ATC_RAT_YAW_I * rate_int_new[:, 2]
             + gains.ATC_RAT_YAW_D * rate_d[:, 2])
    torque_body = torch.stack([tau_x, tau_y, tau_z], dim=-1)   # (B, 3)

    # -- Stage 4: mixer -------------------------------------------------------
    # Per-motor thrust = throttle_per + roll_term + pitch_term + yaw_term.
    # roll_term: tau_x = sum y_i * dT_i. With y_i = pattern * L, dT_i for
    # roll = (pattern_i / (4L)) * tau_x. (Each motor contributes 1/4 of the
    # total torque when its pattern is +1.)
    L = MOTOR_ARM_LENGTH
    # Yaw mixing scale: at hover, dtau_z ≈ k_yaw * dT_motor with sign per
    # motor. Approximate k_yaw using the omega <-> thrust relationship at
    # hover. We use a simple linearization: dtau_z = (K_TORQUE/K_THRUST)
    # * dT_motor * spin (per motor at the same omega). 1/(4*K_TORQUE/K_THRUST)
    # converts a tau_z demand to a per-motor thrust delta with the right
    # sign pattern.
    k_yaw = K_TORQUE_OVER_K_THRUST                             # m

    rp = torch.tensor(ROLL_PATTERN,  device=pos_z.device, dtype=pos_z.dtype)
    pp = torch.tensor(PITCH_PATTERN, device=pos_z.device, dtype=pos_z.dtype)
    yp = torch.tensor(YAW_PATTERN,   device=pos_z.device, dtype=pos_z.dtype)

    thrust_per = (
        thrust_total.view(-1, 1) / 4.0
        + (rp.view(1, -1) / (4.0 * L)) * tau_x.view(-1, 1)
        + (pp.view(1, -1) / (4.0 * L)) * tau_y.view(-1, 1)
        + (yp.view(1, -1) / (4.0 * k_yaw)) * tau_z.view(-1, 1)
    )                                                          # (B, 4)

    omega_cmd = _safe_sqrt_thrust_to_omega(thrust_per, K_thrust)
    pwm_cmd = _omega_to_pwm(omega_cmd)

    new_state = ControllerState(
        velz_integral=velz_int_new,
        rate_integral=rate_int_new,
        prev_rate_err=rate_err,
        velz_err_prev=velz_err,
    )
    debug = {
        "thrust_total": thrust_total,
        "torque_body":  torque_body,
        "rate_target":  rate_target,
        "rate_err":     rate_err,
        "thrust_per":   thrust_per,
        "omega_cmd":    omega_cmd,
    }
    return ControllerOut(pwm_cmd=pwm_cmd, state=new_state, debug=debug)
