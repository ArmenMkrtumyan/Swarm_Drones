"""HoverPretrain-v0 — torch-vectorized GPU pre-training env (Phase 1 of Plan C).

Same observation and action specification as :class:`HoverPidTunerEnv` (Phase
2) so a policy trained here transfers 1:1 to the SITL fine-tune harness:

  obs (19): [8 normalized gains, alt_err, pos_n, pos_e,
             vel_n, vel_e, vel_d, roll, pitch, gx, gy, gz]
  act (8):  per-gain delta in [-1, 1], scaled to delta_per_step.

Tracked on a *batch* dim B; single-env semantics emulated by B=1. We expose
the SB3 ``VecEnv``-shaped interface (reset/step batch tensors) so SB3 PPO/SAC
can consume it without wrapping in DummyVecEnv. Single-policy GPU inference
across all envs.

Architecture
------------
At each env step, for each env in the batch:

1. Apply policy action: nudge `current_gains[i] += action[i] * delta_per_step`,
   clamp to (lo, hi).
2. Run the cascaded PID controller for ``inner_steps_per_action`` physics
   ticks at ``physics_dt`` (so the controller sees the new gains and
   stabilizes for a meaningful sim-time before the next obs).
3. Feed PWM into the motor model + aero, integrate body dynamics.
4. After the inner loop, sample observation, compute reward, check done.

The default ``inner_steps_per_action = 200`` × ``physics_dt = 0.0025`` =
0.5 sim seconds per env step — same cadence as Phase 2.

Domain randomization
--------------------
Per env, on `reset()`, sample mass, motor τ, K_drag, gyro bias, init
attitude, init altitude offset, initial gain mistuning. See
:meth:`_sample_dr_params`.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np
import torch
from torch import Tensor

from lab.dynamics.body import (
    BodyState, InertiaParams, body_step, euler_to_quat, quat_to_euler,
    quat_to_R, world_vel_to_body, I_XX_NOMINAL, I_YY_NOMINAL, I_ZZ_NOMINAL,
)
from lab.dynamics.motor_model import (
    MotorParams, MOTOR_TAU_S_NOMINAL, K_DRAG_NOMINAL, NOMINAL_MASS_KG,
    PWM_HOVER, K_TORQUE_OVER_K_THRUST,
    aggregate_wrench, body_drag_force, compute_K_thrust, motor_step,
    omega_at_pwm, pwm_to_omega_cmd, MOTOR_POS_BODY, MOTOR_SPIN_DIR,
)
from lab.dynamics.atc_psc_mirror import (
    ControllerState, HoverTarget, PidGains, cascaded_step,
)


# -----------------------------------------------------------------------------
# Config — mirrors the YAML used in cfg/pretrain_v0.yaml.
# Keep field names aligned with hover_v0.yaml's "env" + "gains" sections so
# the same loader works for both.
# -----------------------------------------------------------------------------


@dataclass
class GainSpec:
    name: str
    baseline: float
    lo: float
    hi: float
    delta_per_step: float


# Default action set: same 8 gains as hover_pid_tuner_v0. Hard-coded so we can
# seed weight transfer; YAML overrides at training time.
DEFAULT_GAINS: list[GainSpec] = [
    GainSpec("ATC_ANG_RLL_P", 4.5, 1.0, 12.0, 0.20),
    GainSpec("ATC_ANG_PIT_P", 4.5, 1.0, 12.0, 0.20),
    GainSpec("ATC_RAT_RLL_P", 0.135, 0.02, 0.50, 0.010),
    GainSpec("ATC_RAT_PIT_P", 0.135, 0.02, 0.50, 0.010),
    GainSpec("ATC_RAT_RLL_I", 0.135, 0.01, 0.50, 0.010),
    GainSpec("ATC_RAT_PIT_I", 0.135, 0.01, 0.50, 0.010),
    GainSpec("ATC_RAT_RLL_D", 0.0036, 0.0, 0.05, 0.0010),
    GainSpec("ATC_RAT_PIT_D", 0.0036, 0.0, 0.05, 0.0010),
]


@dataclass
class DRRange:
    """Domain-randomization range — uniform sampling in [lo, hi]."""
    lo: float
    hi: float


@dataclass
class PretrainConfig:
    # Env shape
    num_envs: int = 1024
    device: str = "cpu"          # "cuda" once we move off CPU torch
    dtype: torch.dtype = torch.float32

    # Episode shape
    hover_alt_m: float = 3.0
    max_episode_steps: int = 120
    inner_steps_per_action: int = 200    # 200 * 0.0025 s = 0.5 sim s per step
    physics_dt: float = 0.0025

    # Termination thresholds
    crash_alt_m: float = 0.5
    crash_pos_xy_m: float = 8.0
    crash_attitude_rad: float = math.radians(60.0)

    # Reward (mirrors HoverEnvConfig in the SITL env so reward transfers)
    survival_bonus: float = 0.0
    crash_penalty: float = -50.0
    reward_scale: float = 1.0
    base_step_reward: float = 1.0
    w_alt_err: float = 1.0
    w_pos_err: float = 0.5
    w_vel: float = 0.05
    w_attitude: float = 0.5
    w_gyro: float = 0.05
    w_action_smooth: float = 0.01

    # Domain randomization. Each is uniform.
    dr_mass: DRRange = field(default_factory=lambda: DRRange(1.10, 1.60))
    dr_motor_tau: DRRange = field(default_factory=lambda: DRRange(0.030, 0.080))
    dr_K_thrust_jitter: DRRange = field(default_factory=lambda: DRRange(0.85, 1.15))
    dr_K_drag: DRRange = field(default_factory=lambda: DRRange(0.020, 0.040))
    dr_gyro_bias: DRRange = field(default_factory=lambda: DRRange(-0.03, 0.03))
    dr_gyro_noise_sigma: DRRange = field(default_factory=lambda: DRRange(0.0, 0.05))
    dr_init_attitude_rad: DRRange = field(default_factory=lambda: DRRange(-math.radians(5), math.radians(5)))
    dr_init_alt_offset: DRRange = field(default_factory=lambda: DRRange(-0.3, 0.3))
    dr_init_xy_offset: DRRange = field(default_factory=lambda: DRRange(-0.5, 0.5))
    dr_initial_gain_mistune_pct: DRRange = field(default_factory=lambda: DRRange(-0.20, 0.20))
    # If True, half the envs sample initial gains uniformly across full
    # [lo, hi] range (challenging — some configurations are inherently
    # oscillatory or sluggish, policy must adapt). Other half use baseline
    # ± mistune_pct (mild). Curriculum mix forces the policy to handle both
    # regimes, while still seeing many "easy" trajectories early.
    dr_uniform_gain_init_fraction: float = 0.5

    gains: list[GainSpec] = field(default_factory=lambda: list(DEFAULT_GAINS))


# -----------------------------------------------------------------------------
# The env
# -----------------------------------------------------------------------------


class HoverPretrainVecEnv:
    """Torch-vectorized hover pre-training env over `num_envs` parallel drones.

    Not a strict subclass of :class:`gymnasium.vector.VectorEnv` — we expose a
    minimal SB3-VecEnv-compatible surface (reset/step batched, observation /
    action space scalars, ``num_envs``) and let the trainer build the SB3
    wrapper around it.

    All state lives on the configured device. ``reset()`` and ``step()``
    return numpy arrays on CPU (SB3 expects numpy at the API boundary), but
    internal state is torch.
    """

    metadata = {"render_modes": []}

    def __init__(self, cfg: PretrainConfig | None = None) -> None:
        self.cfg = cfg or PretrainConfig()
        if not self.cfg.gains:
            self.cfg.gains = list(DEFAULT_GAINS)
        self.device = torch.device(self.cfg.device)
        self.dtype = self.cfg.dtype
        self.num_envs = self.cfg.num_envs
        self.n_gains = len(self.cfg.gains)

        # Spaces (single-env shape; SB3 VecEnv duplicates per env).
        from gymnasium import spaces
        obs_dim = self.n_gains + 11
        self.single_observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(obs_dim,), dtype=np.float32,
        )
        self.observation_space = self.single_observation_space
        self.single_action_space = spaces.Box(
            low=-1.0, high=1.0, shape=(self.n_gains,), dtype=np.float32,
        )
        self.action_space = self.single_action_space

        # Per-gain bounds + delta_per_step as tensors, shape (n_gains,).
        self._gain_lo = torch.tensor([g.lo for g in self.cfg.gains],
                                      device=self.device, dtype=self.dtype)
        self._gain_hi = torch.tensor([g.hi for g in self.cfg.gains],
                                      device=self.device, dtype=self.dtype)
        self._gain_baseline = torch.tensor([g.baseline for g in self.cfg.gains],
                                            device=self.device, dtype=self.dtype)
        self._gain_delta = torch.tensor([g.delta_per_step for g in self.cfg.gains],
                                         device=self.device, dtype=self.dtype)

        # Per-env state initialized in reset.
        self._allocated = False

    # ------------------------------------------------------------ DR sampling

    def _u(self, r: DRRange, shape) -> Tensor:
        """Uniform sample in [r.lo, r.hi]."""
        return torch.empty(shape, device=self.device, dtype=self.dtype).uniform_(r.lo, r.hi)

    def _sample_dr_params(self) -> None:
        """Sample per-env DR parameters and rebuild MotorParams + InertiaParams."""
        B = self.num_envs
        mass = self._u(self.cfg.dr_mass, (B,))
        tau = self._u(self.cfg.dr_motor_tau, (B,))
        K_drag = self._u(self.cfg.dr_K_drag, (B,))
        K_thrust_jitter = self._u(self.cfg.dr_K_thrust_jitter, (B,))

        # K_thrust auto-cal'd from mass at hover_omega, then jittered for
        # variability beyond the analytical hover-perfect value.
        omega_hover = omega_at_pwm(PWM_HOVER, device=self.device, dtype=self.dtype)
        K_thrust_nom = compute_K_thrust(mass, omega_hover)
        K_thrust = K_thrust_nom * K_thrust_jitter
        K_torque = K_thrust * K_TORQUE_OVER_K_THRUST

        pos = torch.tensor(MOTOR_POS_BODY, device=self.device, dtype=self.dtype)
        pos = pos.unsqueeze(0).expand(B, -1, -1).contiguous()
        spin = torch.tensor(MOTOR_SPIN_DIR, device=self.device, dtype=self.dtype)
        spin = spin.unsqueeze(0).expand(B, -1).contiguous()

        self._motor_params = MotorParams(
            mass_kg=mass, motor_tau_s=tau,
            K_thrust=K_thrust, K_torque=K_torque, K_drag=K_drag,
            motor_pos_body=pos, motor_spin_dir=spin,
        )
        I = torch.tensor([I_XX_NOMINAL, I_YY_NOMINAL, I_ZZ_NOMINAL],
                          device=self.device, dtype=self.dtype)
        I = I.unsqueeze(0).expand(B, -1).contiguous()
        # Inertia scales roughly with mass; keep it simple: scale by mass / nominal.
        I = I * (mass / NOMINAL_MASS_KG).view(-1, 1)
        self._inertia = InertiaParams(mass_kg=mass, I_diag=I)

        # Per-env gyro bias and noise sigma.
        self._gyro_bias = self._u(self.cfg.dr_gyro_bias, (B, 3))
        self._gyro_noise_sigma = self._u(self.cfg.dr_gyro_noise_sigma, (B,))

    # --------------------------------------------------------------- lifecycle

    def reset(self, *, seed: int | None = None) -> tuple[np.ndarray, dict]:
        if seed is not None:
            torch.manual_seed(seed)
        B = self.num_envs

        # 1) DR + dynamics params
        self._sample_dr_params()

        # 2) Body state — start at hover_alt with small DR perturbations.
        pos = torch.zeros(B, 3, device=self.device, dtype=self.dtype)
        pos[:, 0] = self._u(self.cfg.dr_init_xy_offset, (B,))
        pos[:, 1] = self._u(self.cfg.dr_init_xy_offset, (B,))
        pos[:, 2] = self.cfg.hover_alt_m + self._u(self.cfg.dr_init_alt_offset, (B,))
        vel = torch.zeros(B, 3, device=self.device, dtype=self.dtype)
        roll = self._u(self.cfg.dr_init_attitude_rad, (B,))
        pitch = self._u(self.cfg.dr_init_attitude_rad, (B,))
        yaw = torch.zeros(B, device=self.device, dtype=self.dtype)
        q = euler_to_quat(roll, pitch, yaw)
        ang_b = torch.zeros(B, 3, device=self.device, dtype=self.dtype)
        self._state = BodyState(pos_world=pos, vel_world=vel,
                                quat_wb=q, ang_vel_b=ang_b)

        # 3) Motor omega — start at hover (assume already armed/spun-up).
        pwm_hover = torch.full((B, 4), PWM_HOVER, device=self.device, dtype=self.dtype)
        self._omega = pwm_to_omega_cmd(pwm_hover).clone()

        # 4) Controller state
        self._ctrl_state = ControllerState.zeros(B, device=self.device, dtype=self.dtype)

        # 5) Initial gains — mix of (baseline + mistune) and (uniform-random)
        # so the policy sees both easy and hard starts. See PretrainConfig.
        mistune_pct = self._u(self.cfg.dr_initial_gain_mistune_pct, (B, self.n_gains))
        mild = torch.clamp(
            self._gain_baseline.unsqueeze(0) * (1.0 + mistune_pct),
            self._gain_lo.unsqueeze(0), self._gain_hi.unsqueeze(0),
        )
        uniform = self._gain_lo.unsqueeze(0) + (self._gain_hi - self._gain_lo).unsqueeze(0) * \
            torch.rand(B, self.n_gains, device=self.device, dtype=self.dtype)
        # Per-env: with probability uniform_fraction, use random uniform gains;
        # else use mild mistune around baseline.
        use_uniform = (torch.rand(B, 1, device=self.device, dtype=self.dtype)
                       < self.cfg.dr_uniform_gain_init_fraction)
        self._current_gains = torch.where(use_uniform, uniform, mild)

        # 6) Episode bookkeeping
        self._step_count = torch.zeros(B, device=self.device, dtype=torch.long)
        self._last_action = torch.zeros(B, self.n_gains, device=self.device, dtype=self.dtype)
        self._home_xy = self._state.pos_world[:, :2].clone()

        self._allocated = True
        obs = self._observe()
        return obs.cpu().numpy(), {}

    # ------------------------------------------------------ inner physics loop

    def _build_gains_struct(self) -> PidGains:
        """Translate the (B, 8) action gain vector into a full PidGains struct.

        Gains the policy doesn't tune are held at their baseline (the v0
        action set covers attitude only; PSC + yaw use defaults).
        """
        B = self.num_envs
        defaults = PidGains.make_default(B, device=self.device, dtype=self.dtype)
        # Map action indices -> PidGains fields. Names must match cfg.gains in
        # PLAN_C.md / hover_v0.yaml.
        name_to_field = {g.name: i for i, g in enumerate(self.cfg.gains)}
        # Overwrite tuned-by-policy gains:
        for name, idx in name_to_field.items():
            setattr(defaults, name, self._current_gains[:, idx])
        return defaults

    def _physics_step(self, gains: PidGains, dt: float) -> None:
        """One physics tick: controller -> motor -> body -> integrate."""
        rpy = quat_to_euler(self._state.quat_wb)
        gyro_observed = self._state.ang_vel_b  # truth for the controller; bias/noise are obs-side

        ctrl_out = cascaded_step(
            pos_z=self._state.pos_world[:, 2],
            vel_z=self._state.vel_world[:, 2],
            roll=rpy[:, 0], pitch=rpy[:, 1], yaw=rpy[:, 2],
            gyro_body=gyro_observed,
            target=HoverTarget.hover_at(self.cfg.hover_alt_m, self.num_envs,
                                         device=self.device, dtype=self.dtype),
            gains=gains, state=self._ctrl_state,
            K_thrust=self._motor_params.K_thrust,
            mass_kg=self._inertia.mass_kg, dt=dt,
        )
        self._ctrl_state = ctrl_out.state

        vel_b = world_vel_to_body(self._state.vel_world, self._state.quat_wb)
        R = quat_to_R(self._state.quat_wb)
        ms = motor_step(
            self._omega, ctrl_out.pwm_cmd, dt, self._motor_params,
            body_vel_body=vel_b,
            pos_world_z=self._state.pos_world[:, 2],
            R_world_body=R,
        )
        self._omega = ms.omega_next
        F_drag_b = body_drag_force(vel_b, self._motor_params.K_drag)
        wrench = aggregate_wrench(
            ms.thrust_per_motor, ms.yaw_torque_per_motor,
            self._motor_params.motor_pos_body, F_drag_b,
        )
        info = body_step(self._state, wrench.force_body, wrench.torque_body,
                         self._inertia, dt)
        self._state = info.state

    # ----------------------------------------------------------------- observe

    def _observe(self) -> Tensor:
        """Build the (B, obs_dim) observation tensor (float32, on device)."""
        B = self.num_envs
        # Normalized current gains in [0, 1].
        gains_norm = (self._current_gains - self._gain_lo.unsqueeze(0)) / \
                     (self._gain_hi.unsqueeze(0) - self._gain_lo.unsqueeze(0)).clamp_min(1e-9)

        rpy = quat_to_euler(self._state.quat_wb)
        # Apply gyro bias + noise to the observed gyro (so policy learns
        # robustness to imperfect sensor reads — same shape ArduPilot SITL has).
        gyro_obs = self._state.ang_vel_b + self._gyro_bias
        if self.cfg.dr_gyro_noise_sigma.hi > 0:
            gyro_obs = gyro_obs + torch.randn_like(gyro_obs) * \
                       self._gyro_noise_sigma.unsqueeze(-1)

        alt_err = self._state.pos_world[:, 2:3] - self.cfg.hover_alt_m
        pos_ne = self._state.pos_world[:, :2] - self._home_xy
        vel_ned = self._state.vel_world.clone()
        # Convert vel from world (FLU/ENU) to NED-ish: north=+x, east=+y, down=-z.
        # Phase 2 reads NED from MAVLink; here we keep [north, east, down]
        # to match exactly.
        vel_ned[:, 1] = -vel_ned[:, 1]  # east = -y_world (FLU left → east-positive)
        vel_ned[:, 2] = -vel_ned[:, 2]  # down = -z_world

        roll = rpy[:, 0:1]
        pitch = rpy[:, 1:2]

        obs = torch.cat([
            gains_norm,                # (B, 8)
            alt_err,                   # (B, 1)
            pos_ne,                    # (B, 2)
            vel_ned,                   # (B, 3)
            roll, pitch,               # (B, 2)
            gyro_obs,                  # (B, 3)
        ], dim=-1)
        return obs.to(torch.float32)

    # ------------------------------------------------------------ reward / done

    def _reward(self, action: Tensor) -> Tensor:
        c = self.cfg
        rpy = quat_to_euler(self._state.quat_wb)
        alt_err = (self._state.pos_world[:, 2] - c.hover_alt_m).abs()
        pos_err = torch.linalg.norm(
            self._state.pos_world[:, :2] - self._home_xy, dim=-1)
        vel = torch.linalg.norm(self._state.vel_world, dim=-1)
        attitude = torch.linalg.norm(rpy[:, :2], dim=-1)  # roll + pitch
        gyro = torch.linalg.norm(self._state.ang_vel_b[:, :2], dim=-1)
        smooth = torch.linalg.norm(action - self._last_action, dim=-1)
        cost = (c.w_alt_err   * alt_err
                + c.w_pos_err  * pos_err
                + c.w_vel      * vel
                + c.w_attitude * attitude
                + c.w_gyro     * gyro
                + c.w_action_smooth * smooth)
        return (c.base_step_reward - cost) * c.reward_scale

    def _check_done(self) -> tuple[Tensor, Tensor]:
        c = self.cfg
        rpy = quat_to_euler(self._state.quat_wb)
        alt = self._state.pos_world[:, 2]
        xy_dev = torch.linalg.norm(self._state.pos_world[:, :2] - self._home_xy, dim=-1)
        crashed = (alt < c.crash_alt_m) | (xy_dev > c.crash_pos_xy_m) | \
                  (rpy[:, 0].abs() > c.crash_attitude_rad) | \
                  (rpy[:, 1].abs() > c.crash_attitude_rad)
        truncated = self._step_count >= c.max_episode_steps
        return crashed, truncated

    # ----------------------------------------------------------------- step API

    def step(self, action: np.ndarray | Tensor):
        if isinstance(action, np.ndarray):
            action = torch.from_numpy(action).to(self.device, dtype=self.dtype)
        action = torch.clamp(action, -1.0, 1.0)

        # Apply gain delta + clamp.
        delta = action * self._gain_delta.unsqueeze(0)
        self._current_gains = torch.clamp(
            self._current_gains + delta,
            self._gain_lo.unsqueeze(0), self._gain_hi.unsqueeze(0),
        )

        # Run inner physics loop with the new gains.
        gains_struct = self._build_gains_struct()
        for _ in range(self.cfg.inner_steps_per_action):
            self._physics_step(gains_struct, self.cfg.physics_dt)

        self._step_count += 1

        crashed, truncated = self._check_done()
        reward = self._reward(action)
        # Apply terminal bonuses.
        reward = reward + crashed.to(self.dtype) * self.cfg.crash_penalty
        reward = reward + (truncated & ~crashed).to(self.dtype) * self.cfg.survival_bonus

        # Auto-reset done envs (SB3 VecEnv expects this).
        done = crashed | truncated
        if done.any():
            self._auto_reset_envs(done)

        self._last_action = action.clone()
        obs = self._observe()
        info = [{} for _ in range(self.num_envs)]
        return (obs.cpu().numpy(),
                reward.cpu().numpy(),
                crashed.cpu().numpy(),
                truncated.cpu().numpy(),
                info)

    def _auto_reset_envs(self, done_mask: Tensor) -> None:
        """Reset only the envs flagged in done_mask, in-place."""
        idx = done_mask.nonzero(as_tuple=False).flatten()
        if len(idx) == 0:
            return
        n = len(idx)

        # Resample DR for these envs (re-do the full _sample_dr_params subset).
        for_lo, for_hi = self.cfg.dr_mass.lo, self.cfg.dr_mass.hi
        new_mass = torch.empty(n, device=self.device, dtype=self.dtype).uniform_(for_lo, for_hi)
        self._motor_params.mass_kg[idx] = new_mass
        self._motor_params.motor_tau_s[idx] = torch.empty(n, device=self.device, dtype=self.dtype)\
            .uniform_(self.cfg.dr_motor_tau.lo, self.cfg.dr_motor_tau.hi)
        omega_hover = omega_at_pwm(PWM_HOVER, device=self.device, dtype=self.dtype)
        K_thrust_nom = compute_K_thrust(new_mass, omega_hover)
        K_thrust_jitter = torch.empty(n, device=self.device, dtype=self.dtype)\
            .uniform_(self.cfg.dr_K_thrust_jitter.lo, self.cfg.dr_K_thrust_jitter.hi)
        self._motor_params.K_thrust[idx] = K_thrust_nom * K_thrust_jitter
        self._motor_params.K_torque[idx] = self._motor_params.K_thrust[idx] * K_TORQUE_OVER_K_THRUST
        self._motor_params.K_drag[idx] = torch.empty(n, device=self.device, dtype=self.dtype)\
            .uniform_(self.cfg.dr_K_drag.lo, self.cfg.dr_K_drag.hi)
        self._inertia.mass_kg[idx] = new_mass
        I_nom = torch.tensor([I_XX_NOMINAL, I_YY_NOMINAL, I_ZZ_NOMINAL],
                              device=self.device, dtype=self.dtype)
        self._inertia.I_diag[idx] = I_nom * (new_mass / NOMINAL_MASS_KG).view(-1, 1)
        self._gyro_bias[idx] = torch.empty(n, 3, device=self.device, dtype=self.dtype)\
            .uniform_(self.cfg.dr_gyro_bias.lo, self.cfg.dr_gyro_bias.hi)
        self._gyro_noise_sigma[idx] = torch.empty(n, device=self.device, dtype=self.dtype)\
            .uniform_(self.cfg.dr_gyro_noise_sigma.lo, self.cfg.dr_gyro_noise_sigma.hi)

        # Reset body state for these envs.
        self._state.pos_world[idx, 0] = torch.empty(n, device=self.device, dtype=self.dtype)\
            .uniform_(self.cfg.dr_init_xy_offset.lo, self.cfg.dr_init_xy_offset.hi)
        self._state.pos_world[idx, 1] = torch.empty(n, device=self.device, dtype=self.dtype)\
            .uniform_(self.cfg.dr_init_xy_offset.lo, self.cfg.dr_init_xy_offset.hi)
        self._state.pos_world[idx, 2] = self.cfg.hover_alt_m + torch.empty(
            n, device=self.device, dtype=self.dtype
        ).uniform_(self.cfg.dr_init_alt_offset.lo, self.cfg.dr_init_alt_offset.hi)
        self._state.vel_world[idx] = 0.0
        roll = torch.empty(n, device=self.device, dtype=self.dtype)\
            .uniform_(self.cfg.dr_init_attitude_rad.lo, self.cfg.dr_init_attitude_rad.hi)
        pitch = torch.empty(n, device=self.device, dtype=self.dtype)\
            .uniform_(self.cfg.dr_init_attitude_rad.lo, self.cfg.dr_init_attitude_rad.hi)
        yaw = torch.zeros(n, device=self.device, dtype=self.dtype)
        self._state.quat_wb[idx] = euler_to_quat(roll, pitch, yaw)
        self._state.ang_vel_b[idx] = 0.0
        # Reset omega to hover.
        pwm_hover = torch.full((n, 4), PWM_HOVER, device=self.device, dtype=self.dtype)
        self._omega[idx] = pwm_to_omega_cmd(pwm_hover)
        # Reset controller state.
        self._ctrl_state.velz_integral[idx] = 0.0
        self._ctrl_state.rate_integral[idx] = 0.0
        self._ctrl_state.prev_rate_err[idx] = 0.0
        self._ctrl_state.velz_err_prev[idx] = 0.0
        # Reset gains with same mild/uniform mix as full reset() does.
        mistune_pct = torch.empty(n, self.n_gains, device=self.device, dtype=self.dtype)\
            .uniform_(self.cfg.dr_initial_gain_mistune_pct.lo, self.cfg.dr_initial_gain_mistune_pct.hi)
        mild = torch.clamp(
            self._gain_baseline.unsqueeze(0) * (1.0 + mistune_pct),
            self._gain_lo.unsqueeze(0), self._gain_hi.unsqueeze(0),
        )
        uniform = self._gain_lo.unsqueeze(0) + (self._gain_hi - self._gain_lo).unsqueeze(0) * \
            torch.rand(n, self.n_gains, device=self.device, dtype=self.dtype)
        use_uniform = (torch.rand(n, 1, device=self.device, dtype=self.dtype)
                       < self.cfg.dr_uniform_gain_init_fraction)
        self._current_gains[idx] = torch.where(use_uniform, uniform, mild)
        # Reset episode bookkeeping.
        self._step_count[idx] = 0
        self._last_action[idx] = 0.0
        self._home_xy[idx] = self._state.pos_world[idx, :2]

    def close(self) -> None:
        pass
