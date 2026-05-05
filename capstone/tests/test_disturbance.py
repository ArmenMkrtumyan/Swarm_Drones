"""Unit tests for capstone.control.disturbance — pure-numpy, no Isaac."""
from __future__ import annotations

import numpy as np
import pytest

from capstone.control.disturbance import (
    DisturbanceProfile,
    ImuNoise,
    MassDrop,
    WindOU,
    calm,
    imu_noise_default,
    make,
    mass_drop_payload,
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
    for name in ("calm", "mass_drop_300g",
                 "wind5", "wind_up3", "wind_down3",
                 "imu_noise", "worst_case"):
        p = make(name, seed=0)
        assert isinstance(p, DisturbanceProfile)
        assert p.name == name


def test_make_rejects_retired_profiles():
    """`mass+10` and `wind2` were dropped entirely 2026-05-05; the underlying
    mass_plus() helper was deleted. They must not be selectable."""
    import pytest as _pt
    for name in ("mass+10", "wind2"):
        with _pt.raises(KeyError):
            make(name, seed=0)
    # The helper itself is gone — importing it should fail.
    with _pt.raises(ImportError):
        from capstone.control.disturbance import mass_plus  # noqa: F401


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
    """worst_case stacks wind5 + mass_drop_300g + imu_noise."""
    p = make("worst_case", seed=0)
    # Wind matches wind5 envelope.
    assert p.wind.sigma_mps == pytest.approx(5.0 / 3.0)
    # Mass: dynamic mass_drop_300g, not the old static multiplier.
    assert p.mass_multiplier == pytest.approx(1.0)
    assert p.mass_drop is not None
    assert p.mass_drop.payload_kg == pytest.approx(0.300)
    assert p.mass_drop.drop_after_hover_s == pytest.approx(5.0)
    # IMU noise matches imu_noise_default.
    assert p.imu.gyro_white_sigma == pytest.approx(0.02)
    assert p.imu.accel_white_sigma == pytest.approx(0.10)
    assert p.imu.gyro_bias_walk_sigma > 0
    # All three actually do something when called.
    w = p.wind_world_mps(0.01)
    g, a = p.imu_noise(0.01)
    assert any(abs(x) > 0 for x in w)        # wind is non-zero (mean wind)
    assert any(abs(float(x)) > 0 for x in g) or any(abs(float(x)) > 0 for x in a)
    # Pre-drop: payload pulls down. Post-drop (after >=5s above threshold): zero.
    assert p.mass_load_N(1.365) == pytest.approx(-0.300 * 9.81)
    for _ in range(700):
        p.update(0.01, altitude_m=2.0)
    assert p.mass_load_N(1.365) == 0.0


def test_wind5_dominates_calm_envelope():
    """wind5 should have substantial wind energy vs. calm (which is exactly 0)."""
    p_calm = make("calm", seed=2026)
    p5 = make("wind5", seed=2026)
    s_calm = np.stack([p_calm.wind_world_mps(0.01) for _ in range(2_000)])
    s5 = np.stack([p5.wind_world_mps(0.01) for _ in range(10_000)])[3_000:]
    rms5 = np.sqrt((s5 * s5).sum(axis=1).mean())
    assert np.allclose(s_calm, 0.0)
    # Sigma 5/3 across 3 axes -> RMS magnitude ~ sigma*sqrt(3) ~ 2.9 m/s.
    assert rms5 > 2.0


def test_make_rejects_unknown():
    import pytest
    with pytest.raises(KeyError):
        make("teleport", seed=0)


def test_seeded_runs_are_reproducible():
    p1 = make("wind5", seed=123)
    p2 = make("wind5", seed=123)
    s1 = np.stack([p1.wind_world_mps(0.01) for _ in range(100)])
    s2 = np.stack([p2.wind_world_mps(0.01) for _ in range(100)])
    assert np.array_equal(s1, s2)


# -----------------------------------------------------------------------------
# MassDrop state machine
# -----------------------------------------------------------------------------
def test_mass_drop_default_factory_makes_300g_5s():
    p = mass_drop_payload()
    assert p.name == "mass_drop_300g"
    assert p.mass_drop is not None
    assert p.mass_drop.payload_kg == pytest.approx(0.300)
    assert p.mass_drop.drop_after_hover_s == pytest.approx(5.0)


def test_mass_drop_pre_hover_keeps_payload():
    """Below the hover-altitude threshold, the timer doesn't advance."""
    md = MassDrop(payload_kg=0.300, drop_after_hover_s=5.0, hover_alt_threshold_m=1.5)
    # Spend 10 seconds at altitude 0.5 m (below threshold) — payload stays.
    for _ in range(1_000):
        fired = md.update(0.01, altitude_m=0.5)
        assert fired is False
    assert md.current_payload_kg() == pytest.approx(0.300)
    assert md._dropped is False


def test_mass_drop_fires_after_threshold_seconds_of_hover():
    md = MassDrop(payload_kg=0.300, drop_after_hover_s=5.0, hover_alt_threshold_m=1.5)
    # 4 s above threshold: not yet.
    fired_count = 0
    for _ in range(400):
        if md.update(0.01, altitude_m=2.0):
            fired_count += 1
    assert fired_count == 0
    assert md.current_payload_kg() == pytest.approx(0.300)
    # Cross the 5 s mark.
    for _ in range(110):
        if md.update(0.01, altitude_m=2.0):
            fired_count += 1
    # Should have fired exactly once across the whole run.
    assert fired_count == 1
    assert md.current_payload_kg() == 0.0
    # Subsequent ticks never re-fire.
    for _ in range(100):
        assert md.update(0.01, altitude_m=2.0) is False


def test_mass_drop_load_N_zero_after_drop():
    p = mass_drop_payload(payload_kg=0.300, drop_after_hover_s=1.0)
    # Before the drop: load = -0.300 * 9.81 ≈ -2.943 N.
    assert p.mass_load_N(1.365) == pytest.approx(-0.300 * 9.81)
    # Tick past the drop trigger.
    for _ in range(150):
        p.update(0.01, altitude_m=2.0)
    assert p.mass_load_N(1.365) == 0.0


def test_mass_drop_reset_re_attaches_payload():
    """After reset(), MassDrop forgets it ever fired so the next takeoff in
    the same bridge session starts with the payload re-attached."""
    md = MassDrop(payload_kg=0.300, drop_after_hover_s=1.0)
    for _ in range(150):
        md.update(0.01, altitude_m=2.0)
    assert md._dropped is True
    assert md.current_payload_kg() == 0.0
    md.reset()
    assert md._dropped is False
    assert md.current_payload_kg() == pytest.approx(0.300)
    assert md._hover_elapsed_s == 0.0
    # Can fire again after reset.
    fired_again = False
    for _ in range(150):
        if md.update(0.01, altitude_m=2.0):
            fired_again = True
    assert fired_again is True


def test_profile_reset_is_noop_for_calm():
    """profile.reset() must not raise on a profile without mass_drop state."""
    for name in ("calm", "wind5", "imu_noise"):
        p = make(name, seed=0)
        p.reset()  # must not raise
        # mass_load_N still consistent after reset.
        assert p.mass_load_N(1.365) == 0.0


def test_worst_case_reset_re_attaches_300g():
    """The whole reason this hook exists: batch-run worst_case without
    re-importing the bridge. After reset, payload is back to 300 g."""
    p = make("worst_case", seed=0)
    for _ in range(700):
        p.update(0.01, altitude_m=2.0)
    assert p.mass_load_N(1.365) == 0.0  # dropped
    p.reset()
    assert p.mass_load_N(1.365) == pytest.approx(-0.300 * 9.81)


def test_mass_drop_profile_update_routes_to_mass_drop():
    """DisturbanceProfile.update() forwards (dt, altitude_m) into MassDrop."""
    p = mass_drop_payload(payload_kg=0.100, drop_after_hover_s=0.5)
    fired = False
    for _ in range(60):
        if p.update(0.01, altitude_m=2.0):
            fired = True
    assert fired is True
    assert p.mass_drop.current_payload_kg() == 0.0
    # Calm profile: update is a no-op and never reports a drop.
    p_calm = make("calm", seed=0)
    for _ in range(100):
        assert p_calm.update(0.01, altitude_m=2.0) is False
