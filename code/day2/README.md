# Day 2: estimation experiments

This is the Day 2 front end: experiment recipes, live capture, replay, plots and
the two small participant decisions. The reusable filter lives in
[kalman](../kalman/README.md); gate geometry and race policies live in
[race_common](../race_common/README.md). Start with the
[E1-E7 worksheet](../../docs/kalman_filter_practical.md).

Activate the environment using the [installation guide](../../docs/software_setup.md#activate-and-check).
From the package root:

```text
cd code
python -m day2.simulate
python -m unittest kalman.test_filter day2.test_experiments day2.test_replay day2.test_live
```

The baseline shows motion, estimation error, model uncertainty and a mixed-sensor
NIS anomaly locator. It includes a position dropout at 7-10 s and a position
outlier at 12 s. E3 supplies full per-sensor diagnostics. Truth follows prescribed
motion; changing the estimate does not change the simulated flight.

Before tuning, use the [sensor calibration exercise](../../docs/sensor_bias_calibration_exercise.md).
Calibration addresses offsets and scale; Q/R express uncertainty. Preserve
raw/derived labels and validate a correction or a decision to retain the existing
calibration on separate data.

For the model and interpretation, see the [equations of motion](../../docs/quadrotor_equations_of_motion.md),
[glossary](../../docs/glossary.md), [detector-system bridge](../../docs/detector_system_analogies.md)
and [collider-tracking analogies](../../docs/collider_tracking_kalman_analogies.md).
The process input is gravity-excluded world acceleration; position and velocity
form the six-state estimate. Day 1's velocity/tilt controllers use different
interfaces and gain units.

## E3-E5 experiment runner

Prefer a short motor-free live capture where the required channels are supported;
the [worksheet](../../docs/kalman_filter_practical.md#live-first-with-a-quick-fallback)
has the capture command, safety checks and five-minute fallback rule.
Reuse `live.py`'s `prepared.csv` and `recording.json`, for example from `code/`:

```text
python -m day2.experiments --study covariance --recording ../data/team-live/prepared.csv --metadata ../data/team-live/recording.json --output ../E3/recorded
```

The same arguments support position gating and offline position-dropout studies.
The original file stays unchanged; software outliers/dropouts affect only the
analysis and are declared in the saved settings. Recorded studies use actual
prepared timestamps, initialize from the first observation once, and fuse only
available position observations. No flow, gyro or range is invented. Position
initialization uses the configured measurement variance, with zero initial
velocity and its configured prior variance. Recording gaps above 0.30 s stop
analysis. A 30-second capture covers the outlier at/after 12 s and dropout recovery.
If the recording begins without position observations, analysis starts at the
first available position. Earlier rows are omitted, with their count and the
initialization time saved in metadata; timestamps retain the original time origin.
A recording with no position observations cannot anchor this comparison.

Per-case recordings are saved under `CASE/recording/`. Figures show observations
and estimates with model bands, not errors against truth. RMSE/coverage fields
are blank/null, seed trials are unavailable, and pulls are plotted against log
time rather than assuming regular sensor intervals. Onboard observations retain
their shared-IMU/cascaded-estimation caveat. This is a comparison, not independent
experimental validation.

Replace the two paths with the checked shared recording for the first fallback.
The controlled fallback below is always available without hardware or data files;
it also supports full sensor ablations, stationarity, and truth-based coverage.

From `code/` with the environment activated:

```text
python -m day2.experiments --study covariance --output ../E3/R-comparison
python -m unittest day2.test_experiments
```

The [E3-E5 worksheet](../../docs/kalman_filter_practical.md#e3-tune-uncertainty-not-just-smoothness)
provides the other commands and time budgets. The runner handles sensor
scheduling, matched noise draws, fault scenarios, diagnostics, plotting and
metrics. Participants choose Q/R, implement `innovation_threshold()` and
`stationary_update_allowed()` in `exercise_policies.py`, then explain the results.
The two starter functions deliberately disable gating and ZUPT. Use
`--check-policies E4` or `--check-policies E5` to check an edit; these checks
report failures until the corresponding decision is implemented. The ordinary
infrastructure unit tests pass with the unfinished starter functions.

Each **new** output directory contains:

- `comparison.png`: every run's XYZ position/velocity errors and model 95% bands.
  Each changed `CASE/comparison.png` also shows that case against baseline alone.
- `CASE/seed-NNN/states.csv`: truth, estimates, marginal uncertainties and IMU input.
- `CASE/seed-NNN/updates.csv`: each measurement, predicted measurement, innovation,
  full S, whitened innovation, NIS, threshold and acceptance, captured before the
  next sensor update overwrites the filter diagnostics. Components 0/1/2 refer
  to XYZ for position/ZUPT, XY for flow and Z for scalar range; unused fields are blank.
- `diagnostics.png` in the first seed's per-case directory: all pre-gate NIS,
  pull histograms and correlations, including rejected updates. Lags count
  regularly spaced sensor updates, skipping gaps. These plots combine all
  intervals; `summary.json` separates startup, nominal, dropout/fault or rest/motion.
- `stationarity.csv` and `stationarity.png` for stationary scenarios: causal
  sensor-window features, policy requests and truth labels used only for scoring.
- `metrics.csv`, `summary.json` and `aggregate.json`: per-run interval RMSE,
  maximum error, six-component marginal coverage, per-sensor diagnostic summaries,
  false stationary updates, and across-seed means/sample standard deviations.
  Coverage is a fraction; a single run's across-seed standard deviation is unavailable.
- `settings.json` and `policies.py.txt`: configurations, seeds, actual sensor-noise
  assumptions, policy source/hash and the declared initial prior.

`--study single --r-scale 10 --acceleration-std 0.08` selects assumptions directly;
`--study process --acceleration-std 0.08` compares a chosen Q with the baseline Q;
`--seed 20 --seed-count 20` repeats an experiment on held-out seeds. Actual sensor
noise stays fixed. `--scenario` selects nominal, dropout, position outlier,
range step, manoeuvre/input delay or rest/motion data. `--disable position flow`
removes those updates; unavailable flow also removes the stationary motion cue.

IMU samples are 50 Hz, position/range 10 Hz, and flow 25 Hz. The synthetic IMU
is level and nonrotating; range is a direct world-height proxy. Truth follows
prescribed motion, while the estimator starts from a declared mean
`[0,0,1,0,0,0]` and the configured covariance. The stationary scenario adds a
0.03 m/s2 X acceleration bias; the manoeuvre scenario delays input by 0.2 s.
Zero-velocity pseudo-updates run at at most 10 Hz. Conditional selection and
overlapping windows create dependence; investigate coverage rather than treating
those updates as independent evidence. These are estimation experiments,
not a closed-loop flight simulation or a hardware stationary detector.

`--export-prepared` additionally writes a declared synthetic `prepared.csv` and
`recording.json` for the first case/seed, usable by the E6 replay helper. This
provides a meaningful cooked fallback when a checked real recording is unavailable;
its provenance remains synthetic, with hardware calibration/validation outstanding.

## Files and participant changes

| Location | Responsibility |
| --- | --- |
| `experiments.py` | E3-E5 recipes, complete diagnostics and saved comparisons. |
| `experiment_data.py` | Matched sensor streams, recorded inputs and isolated faults. |
| `exercise_policies.py` | Participant-owned E4 threshold and E5 stationary decision. |
| `live.py`, `replay.py` | Safe motor-free acquisition and offline figures/commands. |
| `simulate.py` | E1 baseline display, using the shared simulation. |
| `../kalman/filter.py` | Prediction and updates; optional bias-state/adaptive-Q work. |
| `../kalman/pipeline.py` | Sensor callbacks, including the optional adapter extension. |
| `../kalman/recording.py` | Prepared-file validation and incremental filtering. |
| `../race_common/strategy.py` | E7 clearance/braking-aware policy study. |

Run `python -m day3.uncertainty` from `code/` for E7's gate-clearance example.
It uses the same estimation baseline, with gate geometry and speed decisions
added in the Day 3 front end. The recommended speed does not change prescribed
truth motion, so it cannot establish lap-time improvements.

For E6, use `python -m day2.live` with the assigned URI, `--motors-disabled`
and a new output directory, or `python -m day2.replay` with prepared input and
metadata. The [worksheet](../../docs/kalman_filter_practical.md#e6-compare-filters-on-live-drone-data)
provides full commands, fallback paths and a time budget. Neither commands motors.
Fused-position replay is cascaded estimation with shared-IMU correlations.

[Race profiles and integration](../race_common/README.md#fallback-profiles)
describe what remains onboard and what an optional live-feedback extension needs.
A position-only race log is not an inertial recording. The team retains the
checked flight fallback when integration or safety evidence is incomplete.
