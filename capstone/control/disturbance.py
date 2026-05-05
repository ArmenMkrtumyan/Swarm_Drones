"""Disturbance injection for the Stage-1 hover-robustness battery.

Defines profiles that the live bridge can opt into via a feature flag:

  - wind_world_mps(t)  -> 3-vec wind velocity in world frame (m/s).
                          Subtracted from body velocity inside the drag term
                          so the drone "feels" wind as apparent flow.
  - imu_noise(dt)      -> (gyro_noise[3], accel_noise[3]) in body FRD,
                          added to bridge IMU outputs before they go to SITL.
  - mass_load_N()      -> extra downward force in world-Z (negative = down)
                          applied to base_link to simulate a payload without
                          retuning Isaac inertias.

This module is pure numpy — runs without Isaac, so capstone/tests/ can
exercise it on the dev machine.

Wind model: zero-mean Ornstein-Uhlenbeck process per axis, plus an optional
mean wind. OU stationary std == sigma. Time constant tau sets the gust
"chunkiness" — tau ~= 1.0 s is consistent with low-altitude hobby-scale
turbulence (Dryden simplification for a single drone in calm air).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Callable

import numpy as np


# -----------------------------------------------------------------------------
# Reproducible RNG hookup
# -----------------------------------------------------------------------------
def _rng(seed: int | None) -> np.random.Generator:
    return np.random.default_rng(None if seed is None else int(seed))


# -----------------------------------------------------------------------------
# Wind: Ornstein-Uhlenbeck per axis + optional mean
# -----------------------------------------------------------------------------
@dataclass
class WindOU:
    """Stationary OU gust process. std == sigma, time constant == tau."""
    sigma_mps: float = 0.0           # gust 1-sigma (per axis)
    tau_s: float = 1.0               # correlation time
    mean_world_mps: tuple[float, float, float] = (0.0, 0.0, 0.0)
    seed: int | None = None
    _state: np.ndarray = field(default_factory=lambda: np.zeros(3))
    _rng_obj: np.random.Generator | None = None

    def __post_init__(self):
        self._rng_obj = _rng(self.seed)
        self._state = np.zeros(3)

    def step(self, dt: float) -> np.ndarray:
        """Advance the OU state by dt, return current wind vector (world mps)."""
        if self.sigma_mps <= 0.0 or self.tau_s <= 0.0 or dt <= 0.0:
            return np.array(self.mean_world_mps, dtype=np.float64)
        # Exact discretization of OU:
        #   x_{k+1} = x_k * exp(-dt/tau) + sigma * sqrt(1 - exp(-2 dt/tau)) * N(0,1)
        decay = math.exp(-dt / self.tau_s)
        noise_std = self.sigma_mps * math.sqrt(max(0.0, 1.0 - decay * decay))
        self._state = self._state * decay + noise_std * self._rng_obj.standard_normal(3)
        return self._state + np.array(self.mean_world_mps, dtype=np.float64)


# -----------------------------------------------------------------------------
# IMU noise: white Gaussian on gyro + accel, plus slow bias random walk
# -----------------------------------------------------------------------------
@dataclass
class ImuNoise:
    """Gaussian + bias-random-walk IMU noise model.

    gyro_white_sigma: rad/s, per axis, applied each call.
    accel_white_sigma: m/s^2, per axis.
    gyro_bias_walk_sigma: rad/s per sqrt(s) — bias drifts over time.
    accel_bias_walk_sigma: m/s^2 per sqrt(s).
    """
    gyro_white_sigma: float = 0.0
    accel_white_sigma: float = 0.0
    gyro_bias_walk_sigma: float = 0.0
    accel_bias_walk_sigma: float = 0.0
    seed: int | None = None
    _gyro_bias: np.ndarray = field(default_factory=lambda: np.zeros(3))
    _accel_bias: np.ndarray = field(default_factory=lambda: np.zeros(3))
    _rng_obj: np.random.Generator | None = None

    def __post_init__(self):
        self._rng_obj = _rng(self.seed)
        self._gyro_bias = np.zeros(3)
        self._accel_bias = np.zeros(3)

    def step(self, dt: float) -> tuple[np.ndarray, np.ndarray]:
        if dt <= 0.0:
            return np.zeros(3), np.zeros(3)
        sqrt_dt = math.sqrt(dt)
        if self.gyro_bias_walk_sigma > 0.0:
            self._gyro_bias += (
                self.gyro_bias_walk_sigma * sqrt_dt * self._rng_obj.standard_normal(3)
            )
        if self.accel_bias_walk_sigma > 0.0:
            self._accel_bias += (
                self.accel_bias_walk_sigma * sqrt_dt * self._rng_obj.standard_normal(3)
            )
        gyro_w = (
            self.gyro_white_sigma * self._rng_obj.standard_normal(3)
            if self.gyro_white_sigma > 0.0 else np.zeros(3)
        )
        accel_w = (
            self.accel_white_sigma * self._rng_obj.standard_normal(3)
            if self.accel_white_sigma > 0.0 else np.zeros(3)
        )
        return gyro_w + self._gyro_bias, accel_w + self._accel_bias


# -----------------------------------------------------------------------------
# Mass drop: stateful payload that releases mid-hover
# -----------------------------------------------------------------------------
@dataclass
class MassDrop:
    """Carry an absolute payload, then drop it after some seconds of hover.

    Models "drone takes off heavy, releases its cargo at altitude". The drop
    fires once `hover_elapsed_s >= drop_after_hover_s`; "hover" is defined by
    `altitude_m >= hover_alt_threshold_m` (single threshold, no velocity gate).
    Once dropped, `current_payload_kg()` returns 0 forever — the bridge will
    naturally stop applying any extra weight.
    """
    payload_kg: float = 0.300
    drop_after_hover_s: float = 5.0
    hover_alt_threshold_m: float = 1.5
    _hover_elapsed_s: float = 0.0
    _dropped: bool = False
    _drop_sim_time_s: float | None = None

    def update(self, dt: float, altitude_m: float) -> bool:
        """Advance internal state. Returns True iff the drop fires this tick."""
        if self._dropped or dt <= 0.0:
            return False
        if altitude_m < self.hover_alt_threshold_m:
            return False
        self._hover_elapsed_s += float(dt)
        if self._hover_elapsed_s >= self.drop_after_hover_s:
            self._dropped = True
            return True
        return False

    def current_payload_kg(self) -> float:
        return 0.0 if self._dropped else self.payload_kg

    def reset(self) -> None:
        """Re-attach the payload (clears _dropped + hover timer).

        Called by the bridge after a disarm so the next takeoff in the same
        Isaac session starts fresh -- enables N-run batches without
        re-importing the bridge between every run.
        """
        self._hover_elapsed_s = 0.0
        self._dropped = False
        self._drop_sim_time_s = None


# -----------------------------------------------------------------------------
# Composite profile
# -----------------------------------------------------------------------------
@dataclass
class DisturbanceProfile:
    """Bundle of disturbances the bridge can sample each physics step.

    A profile is `calm()` if all its components are zero — which makes it
    safe to enable unconditionally and feature-flag the *contents*, not the
    import.
    """
    name: str
    wind: WindOU = field(default_factory=WindOU)
    imu: ImuNoise = field(default_factory=ImuNoise)
    mass_multiplier: float = 1.0  # 1.10 = +10% mass (used by worst_case)
    # When set, takes precedence over `mass_multiplier`: payload is absolute
    # (kg), gets released mid-hover. mass_multiplier stays 1.0.
    mass_drop: MassDrop | None = None

    def wind_world_mps(self, dt: float) -> np.ndarray:
        return self.wind.step(dt)

    def imu_noise(self, dt: float) -> tuple[np.ndarray, np.ndarray]:
        return self.imu.step(dt)

    def update(self, dt: float, altitude_m: float | None = None) -> bool:
        """Tick stateful sub-disturbances. Returns True iff a mass drop fired."""
        if self.mass_drop is not None and altitude_m is not None:
            return self.mass_drop.update(dt, altitude_m)
        return False

    def reset(self) -> None:
        """Re-arm any stateful sub-disturbances (e.g. mass_drop payload).

        No-op for profiles without state (calm/wind/imu). Bridge calls this
        on disarm so a single bridge session can run N takeoffs back-to-back.
        """
        if self.mass_drop is not None:
            self.mass_drop.reset()

    def mass_load_N(self, base_mass_kg: float, gravity_mps2: float = 9.81) -> float:
        """World-Z force (negative = down) representing the perturbation payload.

        Returns 0.0 for a calm profile, or for a mass_drop profile after the
        payload has been released.
        """
        if self.mass_drop is not None:
            return -self.mass_drop.current_payload_kg() * gravity_mps2
        return -(self.mass_multiplier - 1.0) * base_mass_kg * gravity_mps2

    def payload_kg(self, base_mass_kg: float, gravity_mps2: float = 9.81) -> float:
        """Current extra mass (kg) being applied as a downward force.

        Mirrors `mass_load_N` semantics but in mass units, so the bridge can
        log "what is the drone currently carrying" on every state sample —
        0.0 for calm/wind/imu, time-varying for mass_drop, constant for
        worst_case (which uses mass_multiplier).
        """
        return -self.mass_load_N(base_mass_kg, gravity_mps2) / gravity_mps2


# -----------------------------------------------------------------------------
# Built-in profile factories — one per gate row in capstone.control.metrics
# -----------------------------------------------------------------------------
def calm(*, seed: int | None = None) -> DisturbanceProfile:
    return DisturbanceProfile(name="calm")


def wind_lateral(peak_mps: float = 2.0, *, seed: int | None = None) -> DisturbanceProfile:
    """Lateral wind: mean wind = sigma along world +X plus OU gusts.

    Sigma is chosen so peaks ~ 3*sigma reach `peak_mps`. Mean wind is set to
    sigma so total wind = mean + zero-mean OU stays positive most of the time
    (i.e. wind blows in a steady direction with gusts on top, not back-and-forth
    around zero).
    """
    sigma = peak_mps / 3.0
    wind = WindOU(sigma_mps=sigma, tau_s=1.0, mean_world_mps=(sigma, 0.0, 0.0), seed=seed)
    return DisturbanceProfile(name=f"wind{int(peak_mps)}", wind=wind)


def wind_vertical(
    peak_mps: float = 3.0,
    *,
    direction: str = "up",
    seed: int | None = None,
) -> DisturbanceProfile:
    """Vertical wind: updraft (`direction='up'`) or downdraft (`direction='down'`).

    Stresses the altitude controller specifically -- lateral force is zero by
    construction, so position-hold should look like calm but altitude has to
    fight a directly-applied vertical drag force. Mean wind = sigma along
    world ±Z plus zero-mean OU gusts.
    """
    if direction not in ("up", "down"):
        raise ValueError(f"direction must be 'up' or 'down', got {direction!r}")
    sigma = peak_mps / 3.0
    sign = +1.0 if direction == "up" else -1.0
    wind = WindOU(
        sigma_mps=sigma,
        tau_s=1.0,
        mean_world_mps=(0.0, 0.0, sign * sigma),
        seed=seed,
    )
    return DisturbanceProfile(name=f"wind_{direction}{int(peak_mps)}", wind=wind)


def mass_drop_payload(
    payload_kg: float = 0.300,
    drop_after_hover_s: float = 5.0,
    hover_alt_threshold_m: float = 1.5,
    *,
    seed: int | None = None,
) -> DisturbanceProfile:
    """Drone takes off carrying `payload_kg`, drops it after `drop_after_hover_s`
    of being above `hover_alt_threshold_m`.

    Naming: name encodes payload mass in grams, e.g. `mass_drop_300g`.
    """
    return DisturbanceProfile(
        name=f"mass_drop_{int(round(payload_kg * 1000))}g",
        mass_drop=MassDrop(
            payload_kg=payload_kg,
            drop_after_hover_s=drop_after_hover_s,
            hover_alt_threshold_m=hover_alt_threshold_m,
        ),
    )


def imu_noise_default(*, seed: int | None = None) -> DisturbanceProfile:
    imu = ImuNoise(
        gyro_white_sigma=0.02,           # rad/s
        accel_white_sigma=0.10,          # m/s^2
        gyro_bias_walk_sigma=0.001,      # rad/s per sqrt(s) -- slow drift
        accel_bias_walk_sigma=0.005,
        seed=seed,
    )
    return DisturbanceProfile(name="imu_noise", imu=imu)


def worst_case(*, seed: int | None = None) -> DisturbanceProfile:
    """All three currently-active disturbance kinds applied simultaneously.

    Stress test: 5 m/s OU wind along world +X (matches `wind5`), +300 g
    payload that drops after 5 s of hover above 1.5 m (matches
    `mass_drop_300g`), and gyro/accel Gaussian + bias-walk IMU noise
    (matches `imu_noise`). The drone has to fight wind drag *and* trust
    noisy state estimates *while* carrying extra weight that releases
    mid-hover -- the controller must re-trim immediately after the drop.
    """
    sigma = 5.0 / 3.0
    wind = WindOU(
        sigma_mps=sigma,
        tau_s=1.0,
        mean_world_mps=(sigma, 0.0, 0.0),
        seed=seed,
    )
    imu = ImuNoise(
        gyro_white_sigma=0.02,
        accel_white_sigma=0.10,
        gyro_bias_walk_sigma=0.001,
        accel_bias_walk_sigma=0.005,
        # Use a different deterministic offset than the wind so the two random
        # streams aren't correlated when both are seeded.
        seed=None if seed is None else seed + 1,
    )
    return DisturbanceProfile(
        name="worst_case",
        wind=wind,
        imu=imu,
        mass_drop=MassDrop(
            payload_kg=0.300,
            drop_after_hover_s=5.0,
            hover_alt_threshold_m=1.5,
        ),
    )


# Convenience map: profile-name (matching metrics.PROFILE_GATES) -> factory.
PROFILE_FACTORIES: dict[str, Callable[..., DisturbanceProfile]] = {
    "calm": calm,
    "mass_drop_300g": lambda **k: mass_drop_payload(0.300, 5.0, **k),
    "wind5": lambda **k: wind_lateral(5.0, **k),
    "wind_up3": lambda **k: wind_vertical(3.0, direction="up", **k),
    "wind_down3": lambda **k: wind_vertical(3.0, direction="down", **k),
    "imu_noise": imu_noise_default,
    "worst_case": worst_case,
}


def make(profile_name: str, *, seed: int | None = None) -> DisturbanceProfile:
    if profile_name not in PROFILE_FACTORIES:
        raise KeyError(f"unknown disturbance profile: {profile_name!r}; "
                       f"available: {sorted(PROFILE_FACTORIES)}")
    return PROFILE_FACTORIES[profile_name](seed=seed)
