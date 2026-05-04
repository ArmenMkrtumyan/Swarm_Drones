"""Unit tests for capstone.control.disturbance — pure-numpy, no Isaac."""
from __future__ import annotations

import numpy as np
import pytest

from capstone.control.disturbance import (
    DisturbanceProfile,
    ImuNoise,
    WindOU,
    calm,
    imu_noise_default,
    make,
    mass_plus,
    wind_lateral,
)


def test_calm_profile_is_zero():
    p = calm()
    w = p.wind_world_mps(0.001)
    g, a = p.imu_noise(0.001)
    assert np.allclose(w, 0.0)
    assert np.allclose(g, 0.0)
    assert np.allclose(a, 0.0)
    assert p.mass_load_N(1.0) == 0.0


def test_mass_plus_load_is_downward():
    p = mass_plus(0.10)
    # +10% of 1 kg at 9.81 m/s^2 -> 0.981 N down (negative world-Z).
    assert p.mass_load_N(1.0) == pytest.approx(-0.981)
    # Doesn't change wind/IMU.
    assert np.allclose(p.wind_world_mps(0.001), 0.0)


def test_ou_wind_stationary_std_matches_sigma():
    """Run a long OU trajectory and check sample std ~ sigma."""
    rng_seed = 12345
    sigma = 1.0
    w = WindOU(sigma_mps=sigma, tau_s=0.5, seed=rng_seed)
    samples = np.stack([w.step(0.01) for _ in range(20_000)])
    # Drop a burn-in to let the OU state forget its zero initial condition.
    s = samples[5_000:]
    # Per-axis sample std should be within 10% of sigma.
    per_axis_std = s.std(axis=0)
    assert np.allclose(per_axis_std, sigma, atol=0.1), per_axis_std


def test_wind_lateral_has_mean_along_x():
    """wind_lateral mean drift is along +X by construction.

    OU samples at tau=1.0 s are heavily correlated, so even 4000 post-burnin
    samples (~40 s) only give ~40 effective samples on a tau=1 axis. Off-axis
    means therefore have wide error bars. The strong assertion is that the
    +X mean dominates the off-axis components.
    """
    p = wind_lateral(2.0, seed=42)
    samples = np.stack([p.wind_world_mps(0.01) for _ in range(20_000)])
    burn_in = samples[5_000:]
    means = burn_in.mean(axis=0)
    assert means[0] > 0.4
    assert means[0] > 1.5 * abs(means[1])
    assert means[0] > 1.5 * abs(means[2])


def test_imu_noise_white_only_has_no_drift():
    n = ImuNoise(gyro_white_sigma=0.02, accel_white_sigma=0.1, seed=7)
    gs = []
    for _ in range(5_000):
        g, a = n.step(0.001)
        gs.append(g)
    gs = np.stack(gs)
    # White noise => std ~ sigma, mean ~ 0
    assert abs(gs.mean()) < 0.005
    assert 0.015 < gs.std() < 0.025


def test_imu_bias_walk_drifts_over_time():
    """Random-walk bias has growing variance: late |bias| > early |bias|.

    Compares the per-axis bias *magnitude* (not signed mean across axes,
    which can cancel by symmetry).
    """
    n = ImuNoise(gyro_bias_walk_sigma=0.05, seed=99)
    early = np.stack([n.step(0.001)[0] for _ in range(1_000)])
    late = np.stack([n.step(0.001)[0] for _ in range(1_000)])
    early_mag = np.linalg.norm(early.mean(axis=0))
    late_mag = np.linalg.norm(late.mean(axis=0))
    assert late_mag > early_mag


def test_make_known_profiles():
    for name in ("calm", "mass+10",
                 "wind2", "wind5", "wind_up3", "wind_down3",
                 "imu_noise", "worst_case"):
        p = make(name, seed=0)
        assert isinstance(p, DisturbanceProfile)
        assert p.name == name


def test_wind_up_pushes_along_plus_z():
    """Updraft: world wind has positive Z component (air moving up).

    OU samples at tau=1.0 are highly correlated, so we can't drive off-axis
    sample means to zero with a finite run -- assert that the +Z mean
    dominates, not that off-axis means vanish."""
    p = make("wind_up3", seed=42)
    samples = np.stack([p.wind_world_mps(0.01) for _ in range(20_000)])[5_000:]
    means = samples.mean(axis=0)
    assert means[2] > 0.5, means
    assert means[2] > 1.5 * abs(means[0])
    assert means[2] > 1.5 * abs(means[1])


def test_wind_down_pushes_along_minus_z():
    p = make("wind_down3", seed=42)
    samples = np.stack([p.wind_world_mps(0.01) for _ in range(20_000)])[5_000:]
    means = samples.mean(axis=0)
    assert means[2] < -0.5, means
    assert -means[2] > 1.5 * abs(means[0])
    assert -means[2] > 1.5 * abs(means[1])


def test_worst_case_combines_all_three_disturbances():
    """worst_case stacks wind + mass + IMU noise at their stronger settings."""
    p = make("worst_case", seed=0)
    # Wind matches wind5 envelope.
    assert p.wind.sigma_mps == pytest.approx(5.0 / 3.0)
    # Mass matches mass+10.
    assert p.mass_multiplier == pytest.approx(1.10)
    # IMU noise matches imu_noise_default.
    assert p.imu.gyro_white_sigma == pytest.approx(0.02)
    assert p.imu.accel_white_sigma == pytest.approx(0.10)
    assert p.imu.gyro_bias_walk_sigma > 0
    # All three actually do something when called.
    w = p.wind_world_mps(0.01)
    g, a = p.imu_noise(0.01)
    assert any(abs(x) > 0 for x in w)        # wind is non-zero (mean wind)
    assert any(abs(float(x)) > 0 for x in g) or any(abs(float(x)) > 0 for x in a)
    assert p.mass_load_N(1.365) < 0           # extra weight pulls down


def test_wind5_is_stronger_than_wind2():
    """Same seed, wind5 should have larger steady-state magnitude than wind2."""
    p2 = make("wind2", seed=2026)
    p5 = make("wind5", seed=2026)
    s2 = np.stack([p2.wind_world_mps(0.01) for _ in range(10_000)])[3_000:]
    s5 = np.stack([p5.wind_world_mps(0.01) for _ in range(10_000)])[3_000:]
    rms2 = np.sqrt((s2 * s2).sum(axis=1).mean())
    rms5 = np.sqrt((s5 * s5).sum(axis=1).mean())
    # 2 -> 5 m/s peaks: sigma 2.5x larger, so RMS magnitude should grow ~2.5x.
    assert rms5 > 2.0 * rms2


def test_make_rejects_unknown():
    import pytest
    with pytest.raises(KeyError):
        make("teleport", seed=0)


def test_seeded_runs_are_reproducible():
    p1 = make("wind2", seed=123)
    p2 = make("wind2", seed=123)
    s1 = np.stack([p1.wind_world_mps(0.01) for _ in range(100)])
    s2 = np.stack([p2.wind_world_mps(0.01) for _ in range(100)])
    assert np.array_equal(s1, s2)
