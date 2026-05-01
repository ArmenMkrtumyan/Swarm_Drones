# Verification scripts

`verification_scripts/` collects standalone scripts that each prove one specific aspect of the model is correct — hand-computed expectations + assertions, kept as instructor-facing artifacts separate from `tools/` (which holds runnable demos and the map editor).

| Script | Verifies |
|---|---|
| [`test_overlap.py`](../verification_scripts/test_overlap.py) | Visit-count overlap metrics (`total_visits`, `self_revisits`, `cross_overlap_visits`, `wasted_visits_total`) against a deterministic 2-drone scripted scenario. 9 hand-computed assertions. |
| [`test_flight_time.py`](../verification_scripts/test_flight_time.py) | Battery / energy model (`P_hover + k·v²` with mass-aware `P_hover`, voltage cutoff). Runs `env.step()` to depletion in hover and cruise regimes; asserts agreement with the analytical formula < 0.5 %. Reports model output (17.0 min hover full-pack, 12.2 min hover cutoff, 9.8 min cruise cutoff at 9.0 m/s) alongside the community-data hover range and translational-lift–inferred cruise bound. |
| [`test_distance.py`](../verification_scripts/test_distance.py) | Tile scale (`meters_per_cell = 5.0`) and motion integration. Asserts F450 derived dimensions, that constant-velocity path length matches `v·t·m_per_cell` within 0.01 %, that sensor wedge area falls within ±30 % of the continuous ½·r²·θ, and that range-to-cutoff equals `cruise_endurance × cruise_speed × m_per_cell` within 0.5 %. |

Each script has a headless mode (default — runs assertions, exits 0/1) and a `--gui` mode (live animation, with a `1× / 5× / 10× / 20×` speed slider where applicable). Run with:

```bash
python verification_scripts/<script>.py [--gui]
```

Each script's docstring documents the scenario, assertions, expected values, run modes, and any reality-check comparisons in detail — open the source for the full description.
