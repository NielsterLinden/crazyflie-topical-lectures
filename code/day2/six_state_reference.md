# Six-state experiment reference

The tools below support optional asynchronous-sensor, sensor-selection and cascaded-estimation studies. Their observations, formats and command-line settings are described separately from the vertical experiment. Onboard state estimates can share IMU information with the laptop filter; agreement between those estimates is not independent accuracy evidence.


The shared six-state filter lives in [kalman](../kalman/README.md).
The optional tools below add asynchronous position, flow and range observations,
matched fault scenarios, automatic stationary decisions and per-sensor diagnostics.
They use their own prepared-data format. For the scheduled vertical experiment,
follow the notebook and sections above.

Run the optional six-state simulation and tests from `code/`:

```text
python -m day2.simulate
python -m unittest kalman.test_filter day2.test_experiments day2.test_replay day2.test_live
```

### Six-state experiment runner

For an optional cascaded comparison, reuse `live.py`'s `prepared.csv` and `recording.json`, for example from `code/`:

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
Injected faults use elapsed time from the first file sample, while plots and
saved events keep the recorded clock. Thus a recording starting at 100 s has
the 7-10 s position dropout at timestamps 107-110 s and its outlier at/after 112 s.
The same 0.30 s sample-gap limit also applies to six-state replay.

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
python -m day2.experiments --study covariance --output ../E3/V-comparison
python -m unittest day2.test_experiments
```

The runner saves matched sensor streams, scenarios, diagnostics and metrics. Choose Q/V, implement `predicted_residual_threshold()` and
`stationary_update_allowed()` in `exercise_policies.py`, then explain the results.
For optional notebook studies, `extra_notebook_tools.compare()` accepts decision
functions directly. For command-line work, put the decisions in `exercise_policies.py`. Both starters
deliberately disable gating and ZUPT. Use
`--check-policies E4` or `--check-policies E5` to check an edit; these checks
report failures until the corresponding decision is implemented. The ordinary
software tests pass with the unfinished starter functions; they check the
supplied tools rather than your E4/E5 decisions.

Each **new** output directory contains:

- `comparison.png`: every run's XYZ position/velocity errors and model 95% bands.
  Each changed `CASE/comparison.png` also shows that case against baseline alone.
- `CASE/seed-NNN/states.csv`: truth, estimates, marginal uncertainties and IMU input.
- `CASE/seed-NNN/updates.csv`: each measurement, predicted measurement, predicted residual,
  full predicted residual covariance R, whitened predicted residual, chi-square, threshold and acceptance, captured before the
  next sensor update overwrites the filter diagnostics. Components 0/1/2 refer
  to XYZ for position/ZUPT, XY for flow and Z for scalar range; unused fields are blank.
  Columns `predicted_residual_0` to `_2` contain the residual vector;
  `R_predicted_00` to `R_predicted_22` contain its covariance matrix.
- `diagnostics.png` in the first seed's per-case directory: all pre-gate chi-square,
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
Position-dropout studies require source position observations before, during and
after the intended gap, and at least two seconds for recovery. Short or already
missing-data recordings are rejected before a comparison is saved. Use the
checked 30-second capture or explicitly choose synthetic input instead.

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
`recording.json` for the first case/seed, usable by the six-state replay helper. This
provides prepared simulated data when a checked real recording is unavailable.
The metadata identifies them as synthetic; hardware calibration and validation
still need real measurements.

## Files and participant changes

| Location | Responsibility |
| --- | --- |
| `experiments.py` | Optional six-state recipes, complete diagnostics and saved comparisons. |
| `experiment_data.py` | Matched sensor streams, recorded inputs and isolated faults. |
| `exercise_policies.py` | Threshold and stationary decisions for six-state studies. |
| `live.py`, `replay.py` | Safe motor-free acquisition and offline figures/commands. |
| `simulate.py` | Optional six-state baseline display, using the shared simulation. |
| `../kalman/filter.py` | Prediction and updates; optional bias-state/adaptive-Q work. |
| `../kalman/pipeline.py` | Sensor callbacks, including the optional adapter extension. |
| `../kalman/recording.py` | Prepared-file validation and incremental filtering. |
| `../race_common/strategy.py` | E7 clearance/braking-aware policy study. |

Run `python -m day3.uncertainty` from `code/` for an additional six-state gate-clearance example.
It uses the same estimation baseline, with gate geometry and speed decisions
added in the Day 3 front end. The recommended speed does not change prescribed
truth motion, so it cannot establish lap-time improvements.

For an additional cascaded-estimation study, use `python -m day2.live` with the assigned URI, `--prepare-for-handling`
and a new output directory, or `python -m day2.replay` with prepared input and
metadata. The [six-state recording guide](../../docs/sensor_recording_contract.md#live-six-state-session-and-replay-fallback)
provides commands and prepared-data examples. Neither commands motors.
Their diagnostic CSVs store position predicted residuals in
`predicted_residual_x_m`, `predicted_residual_y_m` and `predicted_residual_z_m`,
with blank fields when no position observation arrives.
Fused-position replay is cascaded estimation with shared-IMU correlations.

[Race profiles and integration](../race_common/README.md#fallback-profiles)
describe what remains onboard and what an optional live-feedback extension needs.
A position-only race log is not an inertial recording. The team retains the
checked flight fallback when integration or safety evidence is incomplete.
