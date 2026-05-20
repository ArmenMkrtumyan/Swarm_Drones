"""F450 motor + aero model — torch-vectorized port of the live bridge.

Faithful re-implementation of the math in
``Swarm_Drones/lab/bridge/Nvidia_SITL_connecter.py`` (Section 4 of
that file) so that a policy trained against this module transfers to the
ArduPilot SITL + Isaac Sim setup with minimal sim-to-sim gap.

Conventions
-----------
- All tensors carry a leading batch dimension ``B``. We never iterate over the
  batch in Python.
- Motor index order matches the bridge / ArduPilot QuadX: ``[FR, RL, FL, RR]``.
- Per-motor forces are along **body +Z** (FLU). Body frame = +X forward,
  +Y left, +Z up.
- Yaw reaction torque sign per motor encoded in :data:`MOTOR_SPIN_DIR`:
  CCW props produce -Z body torque on the body (Newton 3rd law) so they have
  ``-1``; CW props have ``+1``.

Calibration
-----------
:func:`compute_K_thrust` reproduces the bridge's auto-cal: choose ``K_THRUST``
so that all-four-motors-at-omega-hover gives total thrust = mass * g, where
``omega_hover`` is :data:`PWM_HOVER` mapped through :func:`pwm_to_omega_cmd`.

This calibration runs once per env reset (mass is randomized) and the
resulting ``K_thrust`` is stored in :class:`MotorParams`.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import NamedTuple

import torch
from torch import Tensor


# -----------------------------------------------------------------------------
# Bridge-equivalent constants (see Nvidia_SITL_connecter.py lines ~280-370).
# These are the *nominal* F450 numbers; per-env DR perturbs them.
# -----------------------------------------------------------------------------

PWM_IDLE: float = 1000.0
PWM_HOVER: float = 1742.0       # MOT_THST_HOVER = 0.55 -> sqrt(0.55) -> 1742
PWM_MAX: float = 2000.0
OMEGA_MAX_RAD_S: float = 800.0  # ~7640 RPM cap

# Motor first-order ESC+motor lag time constant (s). Real A2212 + ESC respond
# in 30-100 ms; nominal 0.05.
MOTOR_TAU_S_NOMINAL: float = 0.05

# Reaction-torque / thrust ratio (m). ~0.02 for 8-10" props.
K_TORQUE_OVER_K_THRUST: float = 0.02

# Quadratic drag coefficient (N s^2 / m^2), tuned so vertical terminal
# velocity ~22 m/s for F450 at 1.365 kg.
K_DRAG_NOMINAL: float = 0.028

# Translational lift (Bauersfeld & Scaramuzza 2021): Gaussian bonus on
# per-motor thrust centered at ~7 m/s.
TRANS_LIFT_PEAK_GAIN: float = 0.15
TRANS_LIFT_PEAK_SPEED: float = 7.0   # m/s
TRANS_LIFT_WIDTH: float = 4.0        # m/s sigma

# Ground effect: per-rotor thrust enhancement when AGL < 1 prop diameter.
GROUND_EFFECT_DIAM: float = 0.239    # 9450 prop diameter
GROUND_EFFECT_GAIN: float = 0.10

# F450 motor positions in body frame (FLU), rad order [FR, RL, FL, RR].
# F450 wheelbase = 450 mm motor-to-motor diagonal (verified post-fix
# 2026-05-05). |x| = |y| = 0.225 / sqrt(2) = 0.159 m.
MOTOR_ARM_LENGTH: float = 0.159
ROTOR_Z_IN_BODY: float = 0.023       # rotor disk z above base origin (m)

# Per-motor body-frame position [4, 3]: [FR, RL, FL, RR] x [x, y, z].
MOTOR_POS_BODY: tuple[tuple[float, float, float], ...] = (
    (+MOTOR_ARM_LENGTH, -MOTOR_ARM_LENGTH, ROTOR_Z_IN_BODY),  # FR
    (-MOTOR_ARM_LENGTH, +MOTOR_ARM_LENGTH, ROTOR_Z_IN_BODY),  # RL
    (+MOTOR_ARM_LENGTH, +MOTOR_ARM_LENGTH, ROTOR_Z_IN_BODY),  # FL
    (-MOTOR_ARM_LENGTH, -MOTOR_ARM_LENGTH, ROTOR_Z_IN_BODY),  # RR
)

# Reaction-torque direction per motor about body +Z: CCW -> -1, CW -> +1.
# QuadX: pwm[0]=FR=CCW, pwm[1]=RL=CCW, pwm[2]=FL=CW, pwm[3]=RR=CW.
MOTOR_SPIN_DIR: tuple[float, float, float, float] = (-1.0, -1.0, +1.0, +1.0)

GRAVITY: float = 9.81
NOMINAL_MASS_KG: float = 1.365


# -----------------------------------------------------------------------------
# Per-env parameter container -- everything that DR perturbs lives here.
# Every field is a float tensor of shape (B,) (batch). Buffers shaped (B, 4)
# or (B, 4, 3) are constructed from these lazily.
# -----------------------------------------------------------------------------


@dataclass
class MotorParams:
    """Per-env, DR-randomized parameters of the F450 dynamics.

    All fields are 1D tensors of shape ``(B,)`` (one value per env), except
    where otherwise noted. Construct on the same device as the state tensors.
    """

    mass_kg: Tensor                 # (B,)
    motor_tau_s: Tensor             # (B,) first-order lag
    K_thrust: Tensor                # (B,) auto-calibrated from mass
    K_torque: Tensor                # (B,) = K_torque_over_K_thrust * K_thrust
    K_drag: Tensor                  # (B,)
    # Motor positions in body frame, (B, 4, 3). Constant across batch unless
    # we add airframe-geometry DR; kept per-env so future DR can perturb.
    motor_pos_body: Tensor          # (B, 4, 3)
    motor_spin_dir: Tensor          # (B, 4)

    @staticmethod
    def make_nominal(B: int, *, device: torch.device | str = "cpu",
                     dtype: torch.dtype = torch.float32) -> "MotorParams":
        """Construct a batch with all envs at nominal F450 values.

        Useful for unit tests; production code calls :meth:`make_dr` instead.
        """
        kw = {"device": device, "dtype": dtype}
        mass = torch.full((B,), NOMINAL_MASS_KG, **kw)
        tau = torch.full((B,), MOTOR_TAU_S_NOMINAL, **kw)
        K_T = compute_K_thrust(mass, omega_hover=omega_at_pwm(PWM_HOVER, **kw))
        K_Q = K_T * K_TORQUE_OVER_K_THRUST
        K_D = torch.full((B,), K_DRAG_NOMINAL, **kw)
        pos = torch.tensor(MOTOR_POS_BODY, **kw)        # (4, 3)
        pos = pos.unsqueeze(0).expand(B, -1, -1).clone()  # (B, 4, 3)
        spin = torch.tensor(MOTOR_SPIN_DIR, **kw)       # (4,)
        spin = spin.unsqueeze(0).expand(B, -1).clone()    # (B, 4)
        return MotorParams(
            mass_kg=mass, motor_tau_s=tau,
            K_thrust=K_T, K_torque=K_Q, K_drag=K_D,
            motor_pos_body=pos, motor_spin_dir=spin,
        )


# -----------------------------------------------------------------------------
# Helper math
# -----------------------------------------------------------------------------


def omega_at_pwm(pwm: float | Tensor, *, device=None, dtype=None) -> Tensor:
    """Map a PWM (us) to commanded omega (rad/s).

    Linear: PWM_IDLE -> 0, PWM_MAX -> OMEGA_MAX. Inputs below idle clamp to 0,
    above max clamp to OMEGA_MAX. Mirrors the bridge's
    :func:`pwm_to_omega_cmd`.
    """
    if not isinstance(pwm, Tensor):
        pwm = torch.tensor(float(pwm), device=device, dtype=dtype)
    pwm_clamped = torch.clamp(pwm, PWM_IDLE, PWM_MAX)
    return (pwm_clamped - PWM_IDLE) / (PWM_MAX - PWM_IDLE) * OMEGA_MAX_RAD_S


def pwm_to_omega_cmd(pwm: Tensor) -> Tensor:
    """Vectorized PWM (us) -> commanded omega (rad/s).

    Args:
        pwm: tensor of shape (B, 4).

    Returns:
        tensor of shape (B, 4) of commanded omega in rad/s.
    """
    pwm_clamped = torch.clamp(pwm, PWM_IDLE, PWM_MAX)
    return (pwm_clamped - PWM_IDLE) / (PWM_MAX - PWM_IDLE) * OMEGA_MAX_RAD_S


def compute_K_thrust(mass_kg: Tensor, omega_hover: Tensor) -> Tensor:
    """Auto-calibrate K_thrust so 4 * K * omega_hover^2 == mass * g.

    Mirrors the bridge's auto-cal at line ~794. ``mass_kg`` and
    ``omega_hover`` may be scalars, vectors, or tensors broadcastable to a
    common shape; the return matches that shape.
    """
    return (mass_kg * GRAVITY) / (4.0 * omega_hover * omega_hover)


# -----------------------------------------------------------------------------
# Forward dynamics step
# -----------------------------------------------------------------------------


class MotorStep(NamedTuple):
    """Output of :func:`motor_step` -- per-motor forces/torques + new omega."""
    omega_next: Tensor              # (B, 4) rad/s
    thrust_per_motor: Tensor        # (B, 4) N along body +Z
    yaw_torque_per_motor: Tensor    # (B, 4) N*m about body +Z (signed)


def motor_step(
    omega_actual: Tensor,
    pwm_cmd: Tensor,
    dt: float | Tensor,
    params: MotorParams,
    *,
    body_vel_body: Tensor | None = None,
    pos_world_z: Tensor | None = None,
    R_world_body: Tensor | None = None,
    enable_translational_lift: bool = True,
    enable_ground_effect: bool = True,
) -> MotorStep:
    """One physics step of motor + aero math.

    Steps (matching the bridge order):

    1. ``omega_actual`` follows commanded omega via first-order lag with
       per-env time constant ``motor_tau_s``.
    2. Per-motor thrust = ``K_thrust * omega^2``.
    3. Per-motor yaw torque = ``K_torque * omega^2 * spin_dir``.
    4. Translational lift (Gaussian factor on thrust) if enabled and
       ``body_vel_body`` provided.
    5. Per-rotor ground effect if enabled and ``pos_world_z`` +
       ``R_world_body`` provided.

    Args:
        omega_actual: (B, 4) current per-motor omega.
        pwm_cmd: (B, 4) commanded PWM (us).
        dt: scalar or (B,) timestep (s).
        params: per-env motor parameters.
        body_vel_body: optional (B, 3) body-frame velocity for trans-lift.
        pos_world_z: optional (B,) world-frame z (m). For ground-effect AGL.
        R_world_body: optional (B, 3, 3) rotation, for asymmetric ground
            effect when the airframe is tilted (per-rotor AGL differs).

    Returns:
        :class:`MotorStep`: ``omega_next``, ``thrust_per_motor``,
        ``yaw_torque_per_motor``.
    """
    # 1) First-order lag toward commanded omega.
    omega_cmd = pwm_to_omega_cmd(pwm_cmd)             # (B, 4)
    if isinstance(dt, Tensor):
        # (B,) -> (B, 1) for broadcasting against (B, 4).
        dt_b = dt.view(-1, 1)
    else:
        dt_b = float(dt)
    tau = params.motor_tau_s.view(-1, 1)              # (B, 1)
    alpha = torch.clamp(dt_b / tau, max=1.0)
    omega_next = omega_actual + (omega_cmd - omega_actual) * alpha

    # 2-3) Thrust and yaw torque from omega^2.
    omega_sq = omega_next * omega_next                 # (B, 4)
    K_T = params.K_thrust.view(-1, 1)                  # (B, 1)
    K_Q = params.K_torque.view(-1, 1)                  # (B, 1)
    thrust = K_T * omega_sq                            # (B, 4)
    yaw_torque = K_Q * omega_sq * params.motor_spin_dir  # (B, 4)

    # 4) Translational lift -- multiplies per-motor thrust uniformly.
    if enable_translational_lift and body_vel_body is not None:
        v_horiz = torch.linalg.norm(body_vel_body[..., :2], dim=-1)  # (B,)
        bell = torch.exp(-((v_horiz - TRANS_LIFT_PEAK_SPEED) ** 2)
                         / (2.0 * TRANS_LIFT_WIDTH * TRANS_LIFT_WIDTH))
        lift_factor = 1.0 + TRANS_LIFT_PEAK_GAIN * bell  # (B,)
        thrust = thrust * lift_factor.unsqueeze(-1)

    # 5) Ground effect -- per-rotor AGL via R_world_body.
    if enable_ground_effect and pos_world_z is not None and R_world_body is not None:
        # Per-rotor world Z = base_z + (R_wb @ rotor_pos_body)[2]
        # body pos: (B, 4, 3); R: (B, 3, 3).
        # rotor_world_offset_z[b, m] = sum_k R[b, 2, k] * pos[b, m, k]
        # = einsum('bk,bmk->bm', R[:, 2], pos)
        offset_z = torch.einsum("bk,bmk->bm",
                                R_world_body[:, 2, :],
                                params.motor_pos_body)            # (B, 4)
        rotor_world_z = pos_world_z.view(-1, 1) + offset_z         # (B, 4)
        agl = rotor_world_z  # ground at world Z = 0 (flat-ground assumption)
        # gnd_factor = 1 + GAIN * (1 - agl/D)^2 for agl in [0, D], else 1.
        agl_clamped = torch.clamp(agl, min=0.0, max=GROUND_EFFECT_DIAM)
        in_effect = (agl < GROUND_EFFECT_DIAM)
        gnd_factor = 1.0 + GROUND_EFFECT_GAIN * (1.0 - agl_clamped / GROUND_EFFECT_DIAM) ** 2
        gnd_factor = torch.where(in_effect, gnd_factor, torch.ones_like(gnd_factor))
        thrust = thrust * gnd_factor

    return MotorStep(
        omega_next=omega_next,
        thrust_per_motor=thrust,
        yaw_torque_per_motor=yaw_torque,
    )


def body_drag_force(vel_body: Tensor, K_drag: Tensor) -> Tensor:
    """Quadratic body drag: F = -K * v * |v|.

    Args:
        vel_body: (B, 3) body-frame velocity.
        K_drag: (B,) drag coefficient.

    Returns:
        (B, 3) drag force in body frame.
    """
    speed = torch.linalg.norm(vel_body, dim=-1, keepdim=True)  # (B, 1)
    return -K_drag.view(-1, 1) * vel_body * speed


# -----------------------------------------------------------------------------
# Aggregate body wrench -- combines per-motor + drag into total F, tau
# -----------------------------------------------------------------------------


class BodyWrench(NamedTuple):
    """Total body-frame force and torque from per-motor thrust + drag."""
    force_body: Tensor              # (B, 3)
    torque_body: Tensor             # (B, 3)


def aggregate_wrench(
    thrust_per_motor: Tensor,
    yaw_torque_per_motor: Tensor,
    motor_pos_body: Tensor,
    drag_force_body: Tensor,
) -> BodyWrench:
    """Convert per-motor thrust + reaction torque into total body-frame F, tau.

    Each motor produces thrust along body +Z at its motor-link location, plus
    a yaw torque about body +Z. The thrust at an offset position generates
    roll and pitch torques about the base origin via the cross product
    ``r x F``. We sum all four to get the total body-frame torque.

    Args:
        thrust_per_motor: (B, 4) thrust along body +Z, per motor.
        yaw_torque_per_motor: (B, 4) yaw reaction torque, per motor.
        motor_pos_body: (B, 4, 3) motor positions in body frame.
        drag_force_body: (B, 3) translational drag force at COM (no torque).

    Returns:
        :class:`BodyWrench`: total ``force_body`` and ``torque_body`` (no
        gravity included; that's added in :mod:`body`).
    """
    B = thrust_per_motor.shape[0]
    # Per-motor force vector: (B, 4, 3) along body +Z.
    F_per = torch.zeros(B, 4, 3, device=thrust_per_motor.device,
                        dtype=thrust_per_motor.dtype)
    F_per[..., 2] = thrust_per_motor

    # Total thrust (sum) + drag.
    F_thrust_total = F_per.sum(dim=1)                            # (B, 3)
    F_total = F_thrust_total + drag_force_body                    # (B, 3)

    # Per-motor torque from r x F. r is motor_pos_body, F is along body +Z.
    # cross(r, F) for F = (0, 0, T): tau = (T*ry, -T*rx, 0).
    rx = motor_pos_body[..., 0]                                   # (B, 4)
    ry = motor_pos_body[..., 1]                                   # (B, 4)
    T = thrust_per_motor                                          # (B, 4)
    tau_x_per = T * ry                                            # roll torque
    tau_y_per = -T * rx                                           # pitch torque
    # Plus per-motor yaw reaction torque about body +Z.
    tau_x = tau_x_per.sum(dim=1)
    tau_y = tau_y_per.sum(dim=1)
    tau_z = yaw_torque_per_motor.sum(dim=1)
    torque_body = torch.stack([tau_x, tau_y, tau_z], dim=-1)      # (B, 3)

    return BodyWrench(force_body=F_total, torque_body=torque_body)
