"""Rigid-body 6-DOF integration for the F450 — torch-vectorized.

State and equations
-------------------
We integrate the standard quadrotor 6-DOF model in body-fixed coordinates for
torque (Newton-Euler) and world-frame for translation:

  position:    p_w_dot   = v_w
  velocity:    v_w_dot   = (R_wb @ F_b) / m + g_w
  attitude:    q_dot     = 0.5 * q ⊗ [0, ω_b]              (unit quaternion)
  ang. vel.:   ω_b_dot   = I^{-1} (τ_b - ω_b × (I @ ω_b))

Where ``q`` is unit quaternion ``(w, x, y, z)`` and ``R_wb`` rotates a body
vector into the world frame. Gravity is ``(0, 0, -g)`` in world coords.

The integrator is **semi-implicit Euler** (velocity update with current
acceleration, position update with new velocity). This is stable enough at
the 200-400 Hz physics step and is what Isaac Sim's PhysX defaults to. RK4
is overkill for hover and slows training noticeably.

Conventions match :mod:`lab.dynamics.motor_model`:
- Body frame FLU (+X forward, +Y left, +Z up).
- World frame FLU/ENU equivalent (Z up).
- Quaternions Hamilton-style ``(w, x, y, z)``.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import NamedTuple

import torch
from torch import Tensor


# Nominal F450 inertia (kg m²). Standard literature values for a 1.365 kg
# F450 with 0.225 m half-diagonal. DR perturbs these per env.
I_XX_NOMINAL: float = 0.0123
I_YY_NOMINAL: float = 0.0123
I_ZZ_NOMINAL: float = 0.0249

GRAVITY_W: tuple[float, float, float] = (0.0, 0.0, -9.81)


# -----------------------------------------------------------------------------
# State containers
# -----------------------------------------------------------------------------


@dataclass
class BodyState:
    """Vectorized rigid-body state. All tensors leading-batch shape ``(B, …)``.

    pos_world : (B, 3)
    vel_world : (B, 3)
    quat_wb   : (B, 4) Hamilton (w, x, y, z), rotates body -> world
    ang_vel_b : (B, 3) body-frame angular velocity (rad/s)
    """

    pos_world: Tensor
    vel_world: Tensor
    quat_wb: Tensor
    ang_vel_b: Tensor

    @staticmethod
    def at_rest(B: int, *, hover_alt: float = 0.0,
                device: torch.device | str = "cpu",
                dtype: torch.dtype = torch.float32) -> "BodyState":
        kw = {"device": device, "dtype": dtype}
        pos = torch.zeros(B, 3, **kw)
        pos[:, 2] = hover_alt
        vel = torch.zeros(B, 3, **kw)
        q = torch.zeros(B, 4, **kw)
        q[:, 0] = 1.0   # identity quaternion
        ang = torch.zeros(B, 3, **kw)
        return BodyState(pos_world=pos, vel_world=vel, quat_wb=q, ang_vel_b=ang)


@dataclass
class InertiaParams:
    """Per-env mass + diagonal inertia tensor."""
    mass_kg: Tensor       # (B,)
    I_diag: Tensor        # (B, 3) diagonal inertia (Ixx, Iyy, Izz)

    @staticmethod
    def make_nominal(B: int, *, device="cpu", dtype=torch.float32) -> "InertiaParams":
        kw = {"device": device, "dtype": dtype}
        m = torch.full((B,), 1.365, **kw)
        I = torch.tensor([I_XX_NOMINAL, I_YY_NOMINAL, I_ZZ_NOMINAL], **kw)
        I = I.unsqueeze(0).expand(B, -1).clone()
        return InertiaParams(mass_kg=m, I_diag=I)


# -----------------------------------------------------------------------------
# Quaternion helpers (Hamilton convention, unit quaternions)
# -----------------------------------------------------------------------------


def quat_to_R(q: Tensor) -> Tensor:
    """Quaternion ``(w, x, y, z)`` -> rotation matrix ``R_wb`` (body -> world).

    Args:
        q: (B, 4) Hamilton quaternion. Need not be exactly unit; we trust the
           caller to normalize via :func:`quat_normalize` before use.

    Returns:
        (B, 3, 3) rotation matrix.
    """
    w, x, y, z = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
    xx = x * x; yy = y * y; zz = z * z
    wx = w * x; wy = w * y; wz = w * z
    xy = x * y; xz = x * z; yz = y * z
    R = torch.stack([
        torch.stack([1 - 2 * (yy + zz), 2 * (xy - wz),     2 * (xz + wy)], dim=-1),
        torch.stack([2 * (xy + wz),     1 - 2 * (xx + zz), 2 * (yz - wx)], dim=-1),
        torch.stack([2 * (xz - wy),     2 * (yz + wx),     1 - 2 * (xx + yy)], dim=-1),
    ], dim=-2)
    return R


def quat_normalize(q: Tensor, eps: float = 1e-8) -> Tensor:
    """Normalize a (..., 4) quaternion."""
    return q / (torch.linalg.norm(q, dim=-1, keepdim=True) + eps)


def quat_derivative(q: Tensor, omega_b: Tensor) -> Tensor:
    """Quaternion time derivative: q_dot = 0.5 * q ⊗ [0, ω_b].

    Hamilton product expanded for ω in body frame.
    """
    w, x, y, z = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
    wx, wy, wz = omega_b[..., 0], omega_b[..., 1], omega_b[..., 2]
    qd_w = 0.5 * (-x * wx - y * wy - z * wz)
    qd_x = 0.5 * ( w * wx - z * wy + y * wz)
    qd_y = 0.5 * ( z * wx + w * wy - x * wz)
    qd_z = 0.5 * (-y * wx + x * wy + w * wz)
    return torch.stack([qd_w, qd_x, qd_y, qd_z], dim=-1)


def euler_to_quat(roll: Tensor, pitch: Tensor, yaw: Tensor) -> Tensor:
    """Convert ZYX-intrinsic Euler angles (roll, pitch, yaw) to quaternion.

    Roll about body X, then pitch about body Y, then yaw about body Z. Same
    convention ArduPilot reports in MAVLink ATTITUDE messages.
    """
    cr = torch.cos(roll * 0.5);   sr = torch.sin(roll * 0.5)
    cp = torch.cos(pitch * 0.5);  sp = torch.sin(pitch * 0.5)
    cy = torch.cos(yaw * 0.5);    sy = torch.sin(yaw * 0.5)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return torch.stack([w, x, y, z], dim=-1)


def quat_to_euler(q: Tensor) -> Tensor:
    """Convert quaternion to (roll, pitch, yaw), ZYX intrinsic.

    Returns: (B, 3) tensor of (roll, pitch, yaw) in radians.
    """
    w, x, y, z = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
    # Standard formulas; pitch is clamped to avoid asin domain errors.
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = torch.atan2(sinr_cosp, cosr_cosp)
    sinp = torch.clamp(2 * (w * y - z * x), -1.0, 1.0)
    pitch = torch.asin(sinp)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = torch.atan2(siny_cosp, cosy_cosp)
    return torch.stack([roll, pitch, yaw], dim=-1)


# -----------------------------------------------------------------------------
# Integration step
# -----------------------------------------------------------------------------


class StepInfo(NamedTuple):
    state: BodyState
    accel_world: Tensor      # (B, 3) translational accel (incl. gravity)
    ang_accel_body: Tensor   # (B, 3) angular accel in body frame


def body_step(
    state: BodyState,
    force_body: Tensor,
    torque_body: Tensor,
    inertia: InertiaParams,
    dt: float | Tensor,
) -> StepInfo:
    """One semi-implicit Euler step of the 6-DOF rigid-body equations.

    Args:
        state: current :class:`BodyState`.
        force_body: (B, 3) total force in body frame (motor + drag).
        torque_body: (B, 3) total torque in body frame (motor + reaction).
        inertia: per-env mass + inertia.
        dt: scalar or (B,) timestep (s).

    Returns:
        :class:`StepInfo` with new state and the accels (useful for IMU
        emulation or smoothness rewards).
    """
    if isinstance(dt, Tensor):
        dt_b = dt.view(-1, 1)
    else:
        dt_b = dt

    # Translational accel: rotate body force to world, add gravity, divide by m.
    R = quat_to_R(state.quat_wb)                                      # (B, 3, 3)
    F_world = (R @ force_body.unsqueeze(-1)).squeeze(-1)              # (B, 3)
    g_w = torch.tensor(GRAVITY_W, device=F_world.device, dtype=F_world.dtype)
    a_w = F_world / inertia.mass_kg.view(-1, 1) + g_w                 # (B, 3)

    # Angular accel via Newton-Euler (diagonal-inertia approximation).
    I = inertia.I_diag                                                # (B, 3)
    Iw = I * state.ang_vel_b                                          # (B, 3)
    cross = torch.cross(state.ang_vel_b, Iw, dim=-1)                  # (B, 3)
    alpha = (torque_body - cross) / I                                 # (B, 3)

    # Semi-implicit Euler (update vel/omega first, then pos/quat with new).
    vel_new = state.vel_world + a_w * dt_b
    omega_new = state.ang_vel_b + alpha * dt_b
    pos_new = state.pos_world + vel_new * dt_b

    # Quaternion update with new omega, then renormalize.
    qdot = quat_derivative(state.quat_wb, omega_new)
    quat_new = quat_normalize(state.quat_wb + qdot * dt_b)

    new_state = BodyState(
        pos_world=pos_new,
        vel_world=vel_new,
        quat_wb=quat_new,
        ang_vel_b=omega_new,
    )
    return StepInfo(state=new_state, accel_world=a_w, ang_accel_body=alpha)


# -----------------------------------------------------------------------------
# Body-frame velocity helper (needed by motor_model trans-lift + drag)
# -----------------------------------------------------------------------------


def world_vel_to_body(vel_world: Tensor, quat_wb: Tensor) -> Tensor:
    """Rotate (B, 3) world velocity into body frame using quaternion."""
    R = quat_to_R(quat_wb)                                # (B, 3, 3) body->world
    # body -> world is R; world -> body is R^T. Apply via einsum.
    return torch.einsum("bji,bj->bi", R, vel_world)
