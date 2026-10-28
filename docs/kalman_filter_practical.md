# Kalman filter practical tasks and deliverables

We use the [starter code](../code/day2/README.md) to test a state estimator
for the hoop race. The [shared conventions](practical_workflow.md) help us make
fair comparisons: keep a baseline and test on data not used for tuning.

## Scope and schedule

E1-E7 describe the scheduled experiments. Follow their linked calibration,
recording and software instructions as needed; the glossary is a lookup aid.
The longer theory notes, detector analogies, external courses and optional
sensor/ensemble extensions are additional material. The live helper supplies
the orientation conversion for E6; the full quaternion derivation is optional
background. Motor-disabled handling and data-provenance checks apply whichever
live or recording route you use.

Allow 3 hours 35 minutes, excluding the lecture and breaks. Keep one shared team
notebook or notes file, at most one page for today, with links to saved outputs.
The deliverables below can be shown or discussed; no separate exercise reports
are needed. Two teams volunteer to present a result or diagnosis for five minutes
each, followed by 20 minutes of discussion. We can compare other results during
practical work or in parallel groups.

| Exercise | Minutes | Result |
| --- | --- | --- |
| E1: baseline and model | 25 | Explain what the supplied filter assumes. |
| E2: sensor calibration | 45 | Produce and independently check calibration constants. |
| E3: covariance tuning | 35 | Compare accuracy with uncertainty consistency. |
| E4: outlier rejection | 30 | Implement and validate dimension-aware [NIS](glossary.md#innovation-and-nis) gates. |
| E5: dropout and stationarity | 25 | Explain [observability](glossary.md#observability) and conditional zero-velocity updates. |
| E6: live drone data | 30 | Compare two filters on the same motor-free live stream; prepared replay is the immediate fallback. |
| E7: race decision | 25 | Use uncertainty to justify a clearance/speed policy. |

E1-E7 are the core; flight and stretch exercises are optional. Use the shared
recording pack if acquisition takes too long. It identifies calibration
conditions and held-out data; note what your team actually measured.
Never reduce safety checks to recover time.
Before E2 recording, use the [sensor recording guide](sensor_recording_contract.md)
to check available channels, existing onboard processing, timestamps, units and
reference labels. Identify separate recordings for calibration and validation.
E6's minimum supported replay
and optional raw-flow/Lighthouse extensions are defined there too.

## Preparation

The [Day 2 notebook](../code/day2/day2_kalman.ipynb) runs these experiments using
the same filter and diagnostic routines as the command-line tools. Use either within
the same time slots. Its optional motor-free capture cell runs the same live
logger without a terminal, with a fixed duration and overall timeout. Follow the
[hand-held recording preparation](sensor_recording_contract.md#prepare-the-drone-for-hand-held-recording):
place the drone down, close other radio clients, select the assigned URI and
opt into capture. The helper sets `stabilizer.stop=1`, verifies the lock and zero
motor outputs, and prints **STOP VERIFIED** before hand motion may begin.
Keep a checked recording or simulation available.
In the notebook, implement the E4/E5 decisions in their
cells and save the notebook; for CLI work, edit `exercise_policies.py` instead.
The `--check-policies` commands test that module, not unsaved notebook functions.
To use those CLI checks on a notebook decision, copy the function into the
corresponding module function first.

From the repository root, activate the native environment. Docker is optional.
On macOS/Linux:

```bash
source .venv/bin/activate
cd code
python -m unittest kalman.test_filter
python -m day2.simulate
```

On Windows, first use `.\.venv\Scripts\Activate.ps1`, then the same `cd` and
Python commands. For notebooks, run `jupyter lab` from the repository root and
open `code/day2/day2_kalman.ipynb` in the file browser; this also exposes `docs/`
for the worksheet and handling-guide links.

The [glossary](glossary.md) defines the filter and uncertainty terms used below.
The supplied filter implements prediction and updates. E3-E5 use
`day2.experiments` for matched sensor streams, scenarios, diagnostics,
plots and metrics. Participants choose noise assumptions and edit two short
decisions in `exercise_policies.py`; no experiment framework or report generator
needs to be written. All commands below run from `code/` and work in Bash and
PowerShell. Use a new output directory for each run.

### Live first, with a quick fallback

Prefer your drone's data for E2 and E6. E3, E4's position gate and E5's position
dropout can reuse a short motor-free capture through the same experiment runner.
Within E3's existing first five minutes, place the drone on a stable surface,
close cfclient and other control scripts, keep clear of propellers, and use the assigned exported
`CF_URI` (or add `--uri` with that URI):

```text
python -m day2.live --output ../data/team-live --duration 30 --prepare-for-handling
```

Wait for **STOP VERIFIED**, then include a few seconds at rest, slow hand translation and rest again, with a clear
Lighthouse view. Check the stationary world acceleration and position behaviour
as described in [E6](#e6-compare-filters-on-live-drone-data). Capture sets only
`stabilizer.stop=1` and sends no flight commands. Keep its raw data and metadata; no new converter
is needed. E2's cfclient raw-sensor calibration logs are separate from this
prepared position/acceleration stream.

If connection, timing, frame, bias or unexpected motion features take more than
five minutes to understand, switch to the **checked real recording** supplied as
`data/e6-prepared.csv` and `data/e6-recording.json`. Replace the team-live paths
in the commands below with those two files. Keep one note of the anomaly, rather
than debugging through the slot or silently cleaning it away.

The commands labelled **controlled fallback** run without any recording or radio;
they always remain available. Their known truth supports RMSE/coverage studies.
The live helper provides position and acceleration, not raw gyro/flow/range for
all E3-E5 tests. Use controlled data for those sensor-specific studies and the
stationary decision. On recordings, missing truth makes error/coverage fields
blank; onboard position is an observation, not an independent reference.

## E1: identify the model and reproduce the baseline

**Task.** Explore the supplied tests and simulation with the filter unchanged,
using `kalman/filter.py`, `kalman/pipeline.py`, and `day2/simulate.py` to understand the baseline.

1. Write the state, units, coordinate frame, initial covariance, and the
   matrices F, B, [Q](glossary.md#filter-covariances), H, and R for each measurement type.
2. Derive F and B by integrating constant acceleration over one sample interval.
   Identify acceleration as the process input and position/velocity as the states.
3. Derive `Q = B Sigma_a B^T` for an independent acceleration perturbation held
   over each interval. Distinguish its units and sample-time dependence from a
   continuous white-acceleration spectral-density model.
4. Compute the actual sensor rates from the simulator's time step and update
   conditions, rather than relying on comments. Mark the position dropout at
   7-10 seconds and the position outlier at 12 seconds. Before viewing the plot,
   predict their signatures in estimated motion, covariance and innovations;
   use the nominal interval as the comparison.
5. State what body-frame accelerometer data need before `on_imu`: calibration,
   attitude rotation, gravity compensation, and consistent units/timestamps.

**Deliverable.** A model sketch, baseline figure, test result, and exact command,
code revision, seed, and configuration used.

**Completion check.** A team member can trace one prediction and one update,
identify the dimensions of every matrix, and explain the outlier response.

## E2: calibrate before tuning

**Task.** Follow the [45-minute calibration exercise](sensor_bias_calibration_exercise.md#e2-core-45-minutes):
5 minutes for inventory, 10 for data, 15 for analysis, 10 for held-out validation
and 5 for the E3 handover.
For live handling, use the [firmware-stop preparation](sensor_recording_contract.md#prepare-the-drone-for-hand-held-recording)
and keep clear of fitted propellers.

1. Inventory every installed sensor: log variable, raw/derived status, units,
   frame, rate, existing onboard correction, and external reference.
2. Record 60 seconds at rest for calibration and a separate 60 seconds for
   validation. Inspect gyro axes and accelerometer magnitude. If acquisition is
   blocked for five minutes, use the checked supplied IMU recording pair; label
   which hardware checks remain unperformed.
3. Estimate per-axis gyro bias and covariance, then test an additional correction
   or justified retention of the existing calibration on the held-out recording
   without refitting. A one-pose gravity check cannot determine accelerometer
   bias/scale. Mark those parameters unknown rather than fitting them here.
4. Save the tested values/decision and one held-out comparison. If stationary
   position is available, estimate its spread for E3; otherwise use the declared
   supplied/default position-noise assumption. Label onboard position as fused;
   its spread is not an independent absolute-accuracy measurement. Gyro covariance
   is not the six-state filter's position R.

**Deliverable.** A sensor inventory, gyro bias/covariance and correction decision
with units, frame and source, an accelerometer gravity check, one held-out
raw/corrected (or justified unchanged) comparison, and the E3 noise assumption.
Keep recordings or a shared link; these are working data, not an extra report.

**Completion check.** Explain the gyro result, the gravity check and what the
reference can identify. Demonstrate improvement on held-out data, or explain
why keeping the existing correction is justified. Record unresolved checks and
continue E3 with supported data; do not use a failed live stream for tuning.
Do not hide fixed biases by inflating R or subtract an onboard correction twice.

Six-orientation accelerometer fits, long-term drift, and flow/range/barometer/
Lighthouse characterization are [optional studies](sensor_bias_calibration_exercise.md#optional-sensor-characterization),
including when supplied data are available. They do not extend the E2 slot or
block offline tuning. Actual flight still depends on the team's hardware checks.

## E3: tune uncertainty, not just smoothness

**Task.** In the 35-minute core, predict the effect of changing R, inspect the matched
comparison, then choose and test one acceleration-noise assumption for Q.
Spend about 5 minutes on predictions, 20 on the comparisons and 10 discussing
the result. Keep the actual simulated sensor noise unchanged.

```text
python -m day2.experiments --study covariance --recording ../data/team-live/prepared.csv --metadata ../data/team-live/recording.json --output ../E3/R-live
```

**Controlled fallback**, also useful for checking accuracy/coverage against truth:

```text
python -m day2.experiments --study covariance --output ../E3/R-comparison
```

The runner compares baseline, R multiplied by 0.1 and R multiplied by 10.
These are **variance** multipliers applied to the available position, flow and range; standard
deviations are multiplied by their square roots. Before opening the figures,
predict measurement following, uncertainty and innovation width.

Choose an acceleration standard deviation in m/s2 for a fourth configuration, replacing
the illustrative `0.08` below with your choice. This run restores baseline R:

```text
python -m day2.experiments --study process --acceleration-std 0.08 --recording ../data/team-live/prepared.csv --metadata ../data/team-live/recording.json --output ../E3/Q-live
```

For the controlled fallback, omit `--recording` and `--metadata` and choose a
new output directory. Keep the same input source for the R and Q comparisons.

Compare `comparison.png` and the per-run `diagnostics.png`. Trace one saved
update through `y = z - H x_minus`, `S`, and
[whitening](glossary.md#whitening-and-cholesky-factorization):
`L = cholesky(S)`, `pull = solve(L, y)`. The supplied code does the bookkeeping;
explain why `pull @ pull` equals NIS. Read per-axis coverage, RMSE and maximum
error in `metrics.csv` for simulations, and per-sensor pull means and widths in
`summary.json` for either source. Synthetic regular streams also have lag-one
correlations; recorded data show pulls versus their actual log times. Separate
startup from nominal behaviour. All three runs use identical recorded samples;
their agreement with onboard position alone cannot establish accuracy.

These diagnostics preserve every pre-gate update, including rejections. The
older E1 simulator's mixed-sensor maximum NIS is useful for locating anomalies;
chi-square comparisons use individual updates of a stated dimension.

**Deliverable.** Link the saved comparisons and choose Q/R with a short explanation
of units, assumptions and one prediction that did or did not hold.

**Completion check.** Explain why a smooth estimate can have misleading
uncertainty. A single run need not have exactly 95% coverage; simulation has truth,
whereas hardware needs an independent reference to establish accuracy.

**Extension.** Develop on seeds 0-4, then validate the chosen configuration on
seeds 20-39. Add `--seed 20 --seed-count 20` to the same command; the runner saves
every run and aggregates mean/spread in `aggregate.json`. Ensemble study and
additional custom analysis are optional within this slot.

## E4: reject outliers without rejecting manoeuvres

**Task.** In the 30-minute core, implement the short `innovation_threshold()` function in
`exercise_policies.py`. Choose a false-rejection probability, for example
alpha = 0.01, and use `chi2.ppf(1 - alpha, df=m)` (`chi2` is already imported
from SciPy). Explain why the
dimensions are position 3, flow 2, range 1 and zero velocity 3.
The supplied starter returns `None`, leaving gating disabled.

Spend about 10 minutes on the decision and checks, 10 comparing clean/outlier
runs, and 10 interpreting them. The runner applies your function to each update
and saves the pre-gate diagnostics automatically.

```text
python -m day2.experiments --check-policies E4
python -m unittest day2.test_experiments
python -m day2.experiments --study gating --scenario nominal --recording ../data/team-live/prepared.csv --metadata ../data/team-live/recording.json --output ../E4/observed
python -m day2.experiments --study gating --scenario outlier --recording ../data/team-live/prepared.csv --metadata ../data/team-live/recording.json --output ../E4/injected
```

**Controlled fallback:** omit `--recording` and `--metadata` in the two commands,
using new output directories. That gives a known clean baseline and one isolated
outlier. The supplied recording is the first fallback for acquisition problems.

The policy check reports unfinished/incorrect decisions; it is expected to fail
before the edit. The supplied tests check that rejection leaves the state and covariance unchanged.
Both experiments compare ungated and participant-gated estimates on identical
samples. In recorded-data mode the original is retained and the outlier is added
only to an analysis copy, at the first position sample at/after 12 seconds.
The capture must be long enough; otherwise use the checked recording or controlled
fallback. An unmodified real recording may itself contain anomalies. Inspect
the position outlier, recovery, and rejected counts
by sensor/interval in `summary.json`. Choose another `--alpha` if useful and
predict its effect before rerunning.

**Deliverable.** The short policy edit and check result, with links to the two
comparisons and a sentence explaining the threshold choice and rejection pattern.
CLI output includes a copy of the policy and its hash. With the notebook route,
keep the edited notebook alongside the saved settings, diagnostics and CSVs.

**Completion check.** Explain why large NIS can indicate a bad observation or a
bad model. Compare clean rejection rates with alpha only where the model is
consistent; accepted-only residuals are truncated and cannot establish that.

**Extension.** Replace `--scenario outlier` with `range-step` (a 0.35 m raised-floor
proxy at 8-11 s) or `manoeuvre` (a valid acceleration pulse with a declared 0.2 s
IMU-input delay), omitting recording arguments. Diagnose one cause at a time. Additional scenarios and tests
are optional; their generation and logging are already supplied.

## E5: missing measurements and stationary constraints

**Task.** In the 25-minute core, use the first 5 minutes to predict and inspect one sensor-loss
comparison, 15 for a stationary decision and rerun, then 5 for discussion.

```text
python -m day2.experiments --study dropout --recording ../data/team-live/prepared.csv --metadata ../data/team-live/recording.json --output ../E5/recorded-loss
```

This removes position updates **only in replay**, comparing original sensing,
3/6-second dropouts and propagation after the initial position. It never blocks
the radio or changes onboard fusion. Use a recording of at least 15 seconds for
the CLI's three- and six-second gaps. The notebook's three-second gap needs at
least 12 seconds. Both routes check that position updates exist before, during
and after the intended gap, including two seconds of recovery. An unsuitable
recording is rejected; select a checked 30-second recording or explicitly use
the synthetic fallback rather than treating an unchanged result as a dropout test.

**Controlled fallback and full sensor study:**

```text
python -m day2.experiments --study dropout --output ../E5/sensor-loss
python -m day2.experiments --study stationarity --output ../E5/starter
```

The first command supplies full sensing, 3/6-second position dropouts, removal of
each sensor separately, and removal of both absolute sources (position and range).
Pick one comparison to explain, opening its `CASE/comparison.png`: predict which position/velocity components
remain constrained and inspect their error and uncertainty. The other results
are available for discussion without implementing more scenarios.

For the second command, inspect `stationarity.png` in the per-run directory.
The simulated drone rests at 0-4 s and 14-18 s, with slow translation between;
6-12 s is constant velocity. Synthetic gyro, acceleration, motor-status and flow
streams are provided. A small residual acceleration bias makes drift visible.

Edit `stationary_update_allowed()` in `exercise_policies.py`. It receives a causal
one-second window summary. Choose thresholds for window duration, gyro,
acceleration norm/variation, motor status and flow speed. Use the plotted
observations to justify them. The starter returns `False`, so it adds no
[zero-velocity updates](glossary.md#zupt).

```text
python -m day2.experiments --check-policies E5
python -m day2.experiments --study stationarity --output ../E5/your-policy
```

Quiet IMU readings and idle motors also occur during hand-carried constant-speed
motion. Use the fresh flow motion cue here; decline the constraint when that
cue is unavailable. This cue measures horizontal motion; a hardware detector
also needs evidence for vertical rest. Truth labels score the decision but never feed the policy.
Compare rest/motion plots and `false_stationary_updates` in `summary.json`;
the runner prints how many accepted ZUPTs occurred during actual motion.
Passing the synthetic checks alone does not validate a hardware detector.

**Deliverable.** Explain one sensor-loss result, save your short stationary policy
and check result, and link the rest/motion comparison. Note thresholds and one
limitation in the existing shared notes.

**Completion check.** Explain observability, growing uncertainty during missing
information and why a stationary constraint needs more evidence than a 1 g norm.
Flow aids both filtering and stationarity selection here; overlapping windows
and conditional updates introduce dependence. Interpret the resulting bands
against simulation truth rather than assuming independent information.

**Extension.** Test loss of the motion cue with
`--study stationarity --disable flow`, or study more seeds. Additional detectors,
fault combinations and custom scenarios can follow after the core.

## E6: compare filters on live drone data

**Task.** Place the drone on a stable surface and run the supplied `day2.live`
helper. Wait for **STOP VERIFIED** before moving it slowly by hand, keeping
clear of fitted propellers. Both filters see the same live samples. No acquisition/conversion
code is required from participants. The
[recording guide](sensor_recording_contract.md#live-e6-session-and-replay-fallback)
defines the saved input and fallback preparation. The helper sets only
`stabilizer.stop=1`, verifies the locked state and zero motor outputs, and leaves
the stop engaged afterwards. It does not change the onboard estimator or send
flight commands. Place the drone down and power-cycle it before returning to
normal preflight checks for flight.

| Minutes | Activity |
| --- | --- |
| 0-5 | Connect, check stationary readings and channel/frame/calibration assumptions. If unavailable, use prepared replay immediately. |
| 5-10 | Record rest, slow translation and gentle rotation by hand; view both filter configurations together. |
| 10-25 | Compare position/velocity, uncertainty bands and innovations; inspect saved outputs or replay the same capture. |
| 25-30 | Add interval notes and save one comparison/limitation in the shared notes. |

Activate the native hardware environment and close cfclient before connecting.
Use the assigned exported `CF_URI`, or add `--uri` with that assigned URI.
From the repository root, in Bash or PowerShell:

```text
cd code
python -m day2.live --output ../E6/live --duration 60 --prepare-for-handling
cd ..
```

The native Matplotlib window shows a rolling ten seconds of XYZ position,
velocity and innovations for both configurations, with position uncertainty
bands. Include a few stationary seconds before and after motion. Check that
gravity-removed acceleration printed in the terminal is near zero while stationary, including when
gently tilted; an implausible result means stop and check conventions/calibration.
Do not hide it by inflating covariance. Motion must remain slow with a clear
Lighthouse view. No flight, deliberate radio blocking or unsafe sensor occlusion
is needed. Closing the window or Ctrl+C ends capture and retains usable samples.
`--no-show` saves outputs without the live window.

The new output directory contains `raw.csv`, `prepared.csv`, `recording.json`,
`baseline.csv/.png/.json`, `comparison.csv/.png/.json` and `live-comparison.png`.
The metadata records channels, conversion, clocks, correction and input hashes;
add the observed rest/motion intervals. Outputs are never overwritten. Failed
captures retain partial raw/prepared data and status, with filter plots when at
least two samples were processed. The helper needs `acc.x/y/z`,
`stateEstimate.qx/qy/qz/qw`, `stateEstimate.x/y/z`, `supervisor.info`,
`stabilizer.stop` and `motor.m1/m2/m3/m4`.
Missing channels, stale data or excessive gaps stop capture rather than invent
inputs; use the prepared recording instead of debugging through the slot.

To reproduce the live comparison exactly, from the repository root:

```text
cd code
python -m day2.replay ../E6/live/prepared.csv --metadata ../E6/live/recording.json --output ../E6/replayed-baseline
python -m day2.replay ../E6/live/prepared.csv --metadata ../E6/live/recording.json --position-std 0.12 --output ../E6/replayed-comparison
cd ..
```

**Immediate fallback.** Instructors retain one tested real recording,
`data/e6-prepared.csv` and `data/e6-recording.json`. It completes the same learning
goal without hardware; switch within the first five minutes if connection or
positioning fails:

```text
cd code
python -m day2.replay ../data/e6-prepared.csv --metadata ../data/e6-recording.json --output ../E6/baseline
python -m day2.replay ../data/e6-prepared.csv --metadata ../data/e6-recording.json --position-std 0.12 --output ../E6/comparison
cd ..
```

If the live stream connects but shows unexpected features, the same five-minute
limit applies. Preserve it for later investigation and use the checked recording
for the comparison. When live acquisition was completed earlier, its prepared
recording can also be reused here; no second capture is necessary.

**Always-available controlled fallback**, if a checked real recording is unavailable:

```text
cd code
python -m day2.experiments --study single --export-prepared --output ../E6/cooked
python -m day2.replay ../E6/cooked/prepared.csv --metadata ../E6/cooked/recording.json --output ../E6/cooked-baseline
python -m day2.replay ../E6/cooked/prepared.csv --metadata ../E6/cooked/recording.json --position-std 0.12 --output ../E6/cooked-comparison
cd ..
```

These are meaningful synthetic motion/filter comparisons, labelled as such.
They support the concepts but do not establish calibration or experimental
performance of the team's drone.

The baseline position standard deviation is 0.04 m; 0.12 m is three times the
standard deviation, **nine times the variance**. Keep other filter settings and
data fixed. Predict which filter follows position measurements more closely,
then use the existing plots to check the prediction together.
Each command saves CSV states/innovations/uncertainties, a PNG figure,
and metadata/filter settings. Choose a new output prefix for reruns; outputs are
not overwritten. Velocity is in the CSV, while the figure displays position,
model uncertainty bands and position innovations.

1. Explain how world acceleration was obtained, what calibration was applied,
   and whether timestamps are acquisition times or logging-time proxies. Do not
   claim unknown delay compensation. Link the recording's channel/conversion map;
   no frame-conversion implementation is required here. The live helper rotates
   specific force using the latest causal onboard quaternion, then removes world
   gravity. `--acc-bias-g BX BY BZ` optionally subtracts an additional validated
   body-frame bias in g; default zero retains existing onboard calibration.
   Do not subtract an onboard correction twice.
2. Compare the runs over identical rest/motion intervals: apparent drift, model
   position sigma and innovation behaviour. If no position-dropout segment is
   supplied, state that and use E5 for that study. These are not independent
   accuracy/coverage measurements without a separate reference.
3. When onboard attitude or position contributed, label the result
   **[cascaded estimation](glossary.md#cascaded-estimation)** and discuss shared IMU information. Never call it independent
   raw Lighthouse/IMU fusion or treat onboard position as truth.

**Deliverable.** Two output sets, captured or supplied recording/metadata or stable
hashed locations, calibration/preparation reference, commands and a short note
comparing intervals and identifying one simulation-to-data discrepancy. List
unidentifiable quantities and sensor updates not exercised by this core.

**Completion check.** Another team reproduces the two runs and the participant
can explain changed weighting/uncertainty without interpreting smoothness as
accuracy. Live capture and prepared replay are equally valid completion paths.
If real data are unavailable, the controlled fallback completes the filter
comparison; label real-data analysis outstanding, not experimentally completed.
The tiny `examples/replay.synthetic.csv` remains a format check only.

**Optional adapter extension, separately budgeted.** Implement TODO 6 for your
own acquisition/replay after the core. Retain raw sensor and normalized event
files, `recording.json`, conversion/calibration code and tests. Address ordering,
late/duplicate/invalid observations, gaps and clock wrap as described in the
recording format. Justify any flow velocity/range-to-height conversion; omit
unsupported updates rather than inventing timing or fusing derived data twice.
This is not part of the 30-minute core and remains motor-free.

## E7: turn covariance into a racing decision

Use the supplied geometry for the simulation study here. For the physical race,
we measure and check the gates with the [supplied survey tool](hoop_survey_challenge.md)
during race-day preparation. Synthetic example coordinates are not a measured course.

**Task.** Explore how uncertainty can inform a race decision using
`Hoop.plane_covariance()` and `conservative_clearance()` in `race_common/hoop.py`
to compare the fixed-speed baseline with a covariance-dependent policy in
`race_common/strategy.py`. Run its Day 3 front end from `code/`:

```text
python -m day3.uncertainty
```

For separate comparisons, use the same seed and save each scenario:

```text
python -m day3.uncertainty --scenario nominal --seed 7 --output ../E7/baseline/nominal.png --no-show
python -m day3.uncertainty --scenario dropout --seed 7 --output ../E7/baseline/dropout.png --no-show
python -m day3.uncertainty --scenario outlier --seed 7 --output ../E7/baseline/outlier.png --no-show
```

Save the fixed-speed baseline first. After editing `recommended_speed()` in
`race_common/strategy.py`, repeat these commands with `../E7/candidate/` as the
output directory. The scenarios share noise draws: dropout removes positions
from 7-10 s; outlier adds 0.65 m to one Y observation at 12 s. The default
`combined` scenario includes both faults. Omit `--no-show` for interactive plots.

1. Project position covariance into the hoop plane. Include estimated radial
   offset, drone size, tracking margin, and an explicit confidence multiplier.
2. Evaluate a vertical hoop and a horizontal hoop; show why different covariance
   components matter. State whether geometry is surveyed or itself uncertain.
3. Define slow-down and non-positive-clearance behaviour. Never force a minimum
   forward speed through a hoop whose clearance check fails. Incorporate approach
   distance and braking limits before proposing a live speed change.
4. Run the same nominal, dropout, and outlier scenarios for both policies. The
   starter's recommended speed does not feed back into its prescribed truth
   trajectory: these are policy decisions, not measured lap-time improvements.
   A lap-time claim needs the closed-loop simulation or qualified repeated flight.

**Deliverable.** Policy/tests, projected uncertainty and clearance plots,
speed/hold decisions, comparison table, and recommendation for race-day profile.

**Completion check.** Explain one case where slowing down or declining to cross
is preferable. Do not interpret a multiple of worst-axis sigma as an exact
multivariate collision probability, or add estimator uncertainty to an empirical
tracking margin without discussing overlap and possible double counting.

## Submission and closing discussion

Share today's section of the same team notebook or notes file: at most one page,
one useful comparison plot, one failure/diagnosis and your race-day recommendation.
Link saved runs, settings and calibration data. State raw/derived channels,
timing assumptions and any unperformed work. No separate report, `manifest.json`
or manually compiled `metrics.csv` is required. Use the
[short template](templates/kalman_filter_report.md) if helpful.

For automated analysis, an optional `metrics.csv` can use:
`exercise,run_id,config_id,seed,data_id,interval,sensor,m,
n_updates,accepted,rejected,position_rmse_m,max_position_error_m,coverage95_x,
coverage95_y,coverage95_z,nis_mean`. Use empty values for unavailable or
inapplicable quantities, never zero to mean missing. Sensor-specific rows keep
different NIS dimensions separate. If quoting success rates, include numerator,
denominator, uncertainty, and the definition of a valid crossing.

Discuss what we measured, what we assumed and what the data cannot determine.
Explain a residual, pull and NIS as you would for a collider track fit. Why is an
in-loop residual not an independent performance measurement? Two teams present;
we can share other results during practical work or in parallel groups.

## Stretch exercises

Once the core evidence is complete, teams interested in going further can
explore one of these extensions:

| Extension | Task | Additional evidence |
| --- | --- | --- |
| Bias state | Add accelerometer bias states with a stated random-walk model. | Recovery of injected bias, covariance, and observability under dropout; compare fixed calibration. |
| Adaptive Q | Increase model uncertainty using a justified manoeuvre indicator. | Held-out consistency and accuracy versus fixed Q, including false triggers. |
| Robust range | Compare NIS rejection with [Tukey weighting](glossary.md#innovation-gating-and-tukey-weighting) for a raised floor. | Effective noise variance, rejection/weight history, and recovery; never mistake floor change for independent Lighthouse truth. |
| Process input versus measurement | Design two explicit models using acceleration differently. | State and noise definitions, discretization, correlations, and a matched-data comparison without double-counting IMU information. |
| Attitude error-state EKF | Extend to quaternion attitude and IMU bias estimation. | Stationary and turning tests, quaternion norm, bias observability, and comparison to the six-state baseline. |
| Joint estimation/control | Tune an estimator and constrained PD/feed-forward or LQG controller together. | Fixed safety limits, closed-loop tracking/saturation/effort results, completion rate, and separation of estimator error from controller lag. |

The [race qualification and fallback policy](race_practical.md#r5-qualify-and-freeze-the-race-configuration)
applies throughout. Before using live code for control, the team completes and
cross-checks simulation, independent calibration, connection, emergency-stop and
low-speed tests, recording their outcomes. Keep the onboard attitude stabilization active.
