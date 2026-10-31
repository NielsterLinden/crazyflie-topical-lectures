# Kalman filter practical tasks and deliverables

The question for E3-E7 is concrete: **how can we estimate velocity when one sensor gives noisy distances and another gives acceleration that drifts when integrated?** Use timestamped accelerometer and downward ToF readings from a level drone moving vertically over a fixed flat floor. The onboard state estimator continues to serve flight, separately from this motor-free experiment.

## Scope and schedule

Open the [Day 2 notebook](../code/day2/day2_kalman.ipynb) in the native Jupyter environment used on Day 1. The [glossary](glossary.md) is a lookup aid; theory notes and detector analogies are optional background. E1 follows a simulated particle track, E2 checks calibration, and E3 builds a two-state height/velocity filter. Participants complete four expressions; logging, plots and checks are supplied.

Logging, plots and file handling are supplied throughout; the lecture demonstration is ready to run.
Coding centres on E1/E3 and short policy functions in E4/E7; the other exercises focus on choices and interpretation.

| Exercise | Minutes | Teams implement or choose | Supplied | Data |
| --- | --- | --- | --- | --- |
| E1 | 25 | Track simulation, filter and pull analysis; AI assistance welcome. | Equations and a complete reference alternative. | Simulation with known truth. |
| E2 | 45 | Assess bias/noise, choose a correction and interpret normalization. | Capture, calibration calculations and held-out comparison. | Live stationary IMU preferred; checked or simulated pair available. |
| E3 | 50 | Four prediction/gain/update expressions; choose Q/V. | Model, residuals, covariance update, checks and comparisons. | Live/saved acceleration and ToF; simulation fallback. |
| E4 | 25 | Chi-square threshold function and rejection probability. | Fault injection and gated/ungated runs using your E3 filter. | E3 data replay with an injected range outlier. |
| E5 | 30 | Gap/rest intervals and uncertainty; derive the velocity constraint. | Your E3 functions plus supplied velocity-constraint update. | E3 replay for range loss; simulation for valid/false rest constraints. |
| E6 | 15 | Select independent input; predict and assess the frozen filter. | Capture/replay, frozen calibration/settings and diagnostics. | Fresh live/saved data preferred; independent simulation available. |
| E7 | 25 | Clearance-to-speed rule and margins. | Covariance projection, gate geometry and baseline decisions. | E3/E5 covariance; illustrative geometry and assumed horizontal uncertainty. |

Live acquisition is motor-free. E6's simulation checks implementation consistency; fresh measured data test transfer to the actual drone.

The total remains **215 minutes**. E3 is split across lunch; see the [day programme](day_2_kalman.md). Keep the edited notebook and one useful comparison in the team's existing notes. No separate exercise reports are needed.

## Position measurements reveal velocity

Before E1, run the notebook's short lecture demonstration with state (x, v)
and position measurements only. Prediction creates position–velocity covariance;
a position residual then corrects velocity through
$\Delta v=C_{vx}^{pred}r/(C_{xx}^{pred}+V)$.
Compare the printed correction with the same update after setting the
cross-covariance to zero. The two plots show estimated position and velocity
with uncertainty bands. E1 uses the spatial analogue: z replaces time and
slope replaces velocity. The demonstration sits within the lecture; E1-E7
retain their scheduled time.

## E1: follow a particle track

**Task.** In 25 minutes, consider a 2D track-following problem. A detector consists
of 10 parallel planes, each 1 m tall (extent in x), placed at
$z=1,2,\ldots,10\,\mathrm{m}$. Each plane measures the track position x with
Gaussian resolution $\sigma=0.01\,\mathrm{m}$. At every plane the slope $dx/dz$
receives an independent Gaussian scattering kick with standard deviation 0.02.
The local process noise adds $0.02^2$ to the slope variance; position is unchanged
at the plane.

1. **Simulate tracks.** Draw a true starting position and slope at z = 0
   (your choice, for example independent zero-mean Gaussians with standard
   deviations 0.1 m and 0.1). Propagate through the planes, include the scattering
   kicks, and generate noisy position measurements.
2. **Write a Kalman filter** with state vector (x, slope), processing the planes
   one by one: predict, then update. Choose a sensible initial state with a large
   covariance. Find the filtered state and covariance C at the last plane.
3. **Run 1,000 tracks.** For both final-state components, histogram
   $\mathrm{pull}_i=(\hat x_i-x_i^{\mathrm{true}})/\sqrt{C_{ii}}$.
   Fit or report the mean and standard deviation, and discuss whether they are
   consistent with a standard normal distribution.

Use Python, NumPy and Matplotlib, with the lecture notation: state x,
covariance C, transport F, process noise Q, projection H, measurement variance V
and gain K. [Wouter Hulsbergen's Tracking and Vertexing slides](https://www.nikhef.nl/~wouterh/teaching/TrackingAndVertexing/index.html)
provide background.

Because time is short, we suggest feeding this prompt into your favourite AI.
Check that you understand its prediction and update, especially the scattering
order and covariance. The person who produces the most beautiful illustration
before noon gets a **pakje stroopwafels**.

**Conventions to agree.** The reference measures first and scatters afterwards.
Its filtered state at the last plane is therefore the incoming state, before
that plane's kick. The kick covariance at a plane is
$Q_{\mathrm{plane}}=\mathrm{diag}(0,0.02^2)$. Over the next gap d,

$$F=\begin{pmatrix}1&d\\0&1\end{pmatrix},\qquad
Q_{\mathrm{gap}}=FQ_{\mathrm{plane}}F^T
=0.02^2\begin{pmatrix}d^2&d\\d&1\end{pmatrix},\qquad
H=(1,0),\qquad V=0.01^2\,\mathrm{m}^2.$$

A kick changes position uncertainty only after propagation. The first gap from
z = 0 to z = 1 contains no scattering plane. The reference starts truth with the
example 0.1 m / 0.1 spreads, and uses a zero-mean fit with initial standard
deviations 1 m / 1. A broad fit prior can affect pulls; check its influence
rather than imposing unit width by rescaling the errors.

**Reference.** Try your implementation first, or enable the notebook's reference.
From `code/` with the course environment active:

```text
python -m day2.tracking --tracks 1000 --seed 7 --output ../day2/tracking/trial-01
```

A fresh output directory receives individual fits and covariances (`tracks.csv`),
summary statistics (`summary.json`), an illustrated track (`track.png`, `track.svg`)
and pull histograms (`pulls.png`). The reference filter is checked against an
independent joint Gaussian solution using the same fit prior.

**Deliverable.** Keep your code, one illustration, the seed and the two pull
means/widths in the notebook.

**Completion check.** Trace one predict/update step and explain the final
position/slope uncertainties, including any departure from unit pull width.

**Optional plot challenge.** Reproduce the green prediction bands below:
grey planes, measured hits, widening bands between planes and contraction at
each update. Include a dotted initial straight-line reference. Propagate
$\sigma_x^2(d)=C_{xx}+2dC_{xt}+d^2C_{tt}$ after adding the local slope-kick
variance; label the bands as one standard deviation.

![Reference track with hit measurements and widening one-sigma prediction bands.](../code/day2/tracking_reference.svg)

**Optional geometry check.** The reference's all-plane control includes virtual
hits beyond the active interval $-0.5\leq x\leq0.5\,\mathrm{m}$ and reports the
number of fully contained tracks. With `--finite-aperture`, omit those hits but keep
all 1,000 attempted tracks. Material is assumed to extend beyond the readout.
Missing-hit information and selecting contained tracks change the statistical
problem, so Gaussian pull calibration needs care. With `--outgoing`, the final
unobserved kick increases slope variance by $0.02^2$ without changing the mean.

## Preparation

Follow the notebook's capture cell, or use the supplied command from `code/` with the environment active:

```text
python -m day2.vertical_live --output ../data/vertical-training --prepare-for-handling
```

Set the assigned exported `CF_URI` or supply `--uri`. Close cfclient and other radio clients. Place the drone on a stable, level surface, clear of propellers, with the downward ToF beam on a flat, stationary floor. The helper sets `stabilizer.stop=1`, verifies the locked state and zero motor outputs, and monitors them throughout capture. **Wait for STOP VERIFIED** before handling. It then checks five stationary seconds; wait for **Calibration check passed** before moving gently up and down. Keep level and hold still again by the 15-second prompt. The 20-second capture ends automatically. The stop remains engaged; put the drone down and power-cycle before normal preflight checks.

Arrange a stable raised support with an opening beneath the sensor, 5-20 cm above the fixed floor. The ToF beam should remain clear during the initial stationary calibration and later motion.

The capture reads `acc.x/y/z` in g and `range.zrange` in millimetres, without onboard position, velocity or orientation estimates. The range is distance to the floor at the sensor, not a Lighthouse/world-coordinate height. Filtering assumes that range approximates vertical height. Tilting the drone, moving over an obstacle or losing the floor violates that model.

The first five stationary seconds estimate the Z specific-force mean, including gravity, and the range spread. The model uses that spread, floored at 1 mm, as range standard deviation; absolute accuracy needs an independent reference. Subtract the stationary specific-force mean and multiply by 9.80665 to obtain approximate vertical acceleration. This one-axis correction retains existing onboard calibration; it does not determine accelerometer scale or a full attitude model. Initial level/rest and gentle-motion consistency checks reject unsuitable recordings.

Each measurement has a **firmware log timestamp**, unwrapped from 24-bit milliseconds into seconds. Its range update is applied at that timestamp after prediction over the measured dt. The current acceleration sample approximates the preceding interval. Log time is a proxy for sensor acquisition time; sensor delays are not compensated. Both channels share a log block, but need not have identical internal sensor latency. Duplicate/reversed timestamps and gaps above 0.30 s stop processing.

### Live first, with a quick fallback

One training capture serves E3-E5. If connection, geometry or data checks stall, spend at most five minutes troubleshooting. To continue with a fallback, choose a checked sensor recording, or leave `recording_directory=None` for simulation. Failed selections clear the old dataset and raise an error. Never silently substitute data.

A sensor recording contains `raw.csv` and `recording.json`. CSV columns are `time_s,acc_x_g,acc_y_g,acc_z_g,range_mm`; metadata declares columns, status, units/conventions and a SHA-256 of the raw bytes. Invalid ranges are omitted from measurement updates while valid acceleration still drives prediction. The initial calibration and first filtering range must be valid. UTF-8 files with a BOM are accepted.

The controlled fallback simulates exactly this rest/up/down/rest experiment with known truth. It supports RMSE and coverage checks; measured data without an independent reference support consistency checks instead. Additional [six-state tools](../code/day2/README.md#additional-six-state-experiments) cover asynchronous sensors and cascaded onboard-estimate comparisons.

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
   blocked for five minutes, select a checked IMU pair or set
   `use_simulated_imu = True` in E2C to generate independent, labelled calibration
   and validation files in `E2-simulated/` under the run folder. Label which
   hardware checks remain unperformed.
3. Estimate per-axis gyro bias and covariance, then test an additional correction
   or justified retention of the existing calibration on the held-out recording
   without refitting. A one-pose gravity check cannot determine accelerometer
   bias/scale. Mark those parameters unknown rather than fitting them here.
   Compare the self-standardized calibration histogram with held-out samples
   normalized by the same frozen mean and RMS. Predict which mean and RMS are
   fixed by construction, and interpret one channel within the existing
   held-out discussion.
4. Save the tested values/decision and one held-out comparison. E3 estimates
   ToF range variability in its own initial stationary check. Gyro covariance
   describes a different sensor and is kept separate from the range variance V.

**Deliverable.** A sensor inventory, gyro bias/covariance and correction decision
with units, frame and source, an accelerometer gravity check, one held-out
raw/corrected (or justified unchanged) comparison, and the E3 noise assumption.
Keep recordings or a shared link; these are working data, not an extra report.

**Completion check.** Explain the gyro result, the gravity check and what the
reference can identify. Demonstrate improvement on held-out data, or explain
why keeping the existing correction is justified. Record unresolved checks and
continue E3 with supported data; do not use a failed live stream for tuning.
Do not hide fixed biases by inflating V or subtract an onboard correction twice.

Six-orientation accelerometer fits, long-term drift, and flow/range/barometer/
Lighthouse characterization are [optional studies](sensor_bias_calibration_exercise.md#optional-sensor-characterization),
including when supplied data are available. They do not extend the E2 slot or
block offline tuning. Actual flight still depends on the team's hardware checks.

## E3: construct and tune a height-velocity filter

**Task (50 minutes).** Spend 25 minutes on data selection/capture and four expressions in the notebook: predicted state, predicted covariance, gain and corrected state. Acquisition uses at most five of those minutes. The two-state vector is `[z, vz]`, with height in metres and velocity in m/s, upwards positive. Use
\[
F=\begin{pmatrix}1&\Delta t\\0&1\end{pmatrix},\quad
B=\begin{pmatrix}\tfrac12\Delta t^2\\\Delta t\end{pmatrix},\quad H=(1,0),\quad Q=BB^T\sigma_a^2.
\]
Calculate `x_predicted = F @ x + B * a`, `C_predicted = F @ C @ F.T + Q`, the predicted residual and its variance, then the gain and state correction. The Joseph covariance update and focused checks are supplied. Use exact recorded dt, rather than assuming a nominal sample period.

Trace one positive height residual through both corrections. The velocity gain is `C_predicted[1,0] / R`: explain how a **height measurement changes velocity** through cross-covariance. Repeat one update with that cross-covariance set to zero and with a different V. Explain the limiting cases of a precise measurement and a very uncertain measurement.

Use the remaining 25 minutes for matched comparisons and discussion. Compare acceleration integration, range differentiation and the KF on the same timestamps. Then vary V by 0.1 and 10 and choose one acceleration standard deviation for Q. Keep observed sensor noise, initial covariance and calibration fixed. Read pre-gate [pull](glossary.md#whitening-and-cholesky-factorization) mean/width, error and marginal 95% coverage where simulation truth exists. Startup is separate from later behaviour; the supplied summary excludes the first two filtering seconds. A smooth curve is not evidence of accuracy.

**Completion:** explain and calculate one prediction/update, including velocity correction and covariance; retain one matched V comparison and a justified Q choice. Save the edited notebook. The data/calibration/configuration, timestamped CSVs and comparison plots are saved by the helper.

## E4: one outlier and its rejection

**Task (25 minutes).** Predict the effect of adding 0.15 m to a single range observation at/after 12 s. The runner changes a copy of the data. Compare ungated and gated estimates on that identical copy.

Implement the one-line `our_residual_threshold(dimension, alpha)` using `chi2.ppf(1-alpha, df=dimension)`. This experiment has **one degree of freedom**; the function also states how dimension enters the rule. Choose and explain alpha. The starter returns None, leaving rejection disabled.

Trace the anomalous sample: predicted residual, variance, prospective gain, chi-square, acceptance, state correction and recovery. Check clean-data rejection too. A valid measurement can look incompatible when Q/V, timing or the motion model is wrong. A raised floor changes the range model; do not interpret every large residual as random sensor noise.

**Completion:** one explained rejection, its effect on state/covariance, and one plausible model error that gating would conceal. Rejected residuals remain in the diagnostic plots; accepted-only distributions are selected data.

## E5: missing range and a known-rest constraint

**Task (30 minutes).** Spend 15 minutes suppressing range updates at 8-11 s in replay while keeping acceleration samples. Predict and inspect height/velocity uncertainty, drift, gain and correction when range returns. No radio blocking or onboard fusion change is involved.

For the next 15 minutes use the controlled recording's known rest at 15-20 s. Derive a velocity measurement `m=0, H=[0,1]` with a finite variance. Choose and justify its standard deviation and a subinterval, then compare no constraint, a valid constraint and a deliberately false constraint at 8-9 s during simulated motion. Examine both error and covariance. A zero-velocity update supplies information; it is not an overwrite of the state with zero variance.

**Completion:** explain covariance growth without a measurement, covariance reduction with justified information, and the bias caused by false information. Automatic stationary detection is an optional extension; quiet inertial readings alone do not establish rest.

## E6: frozen assumptions on fresh data

**Task (15 minutes).** Use five minutes to select a new capture or supplied recording, five to compare and five to discuss. Freeze the E3 calibration and chosen Q/V; do not fit the validation data. An identical input hash is rejected. Initial height is established from the new recording and initial velocity is declared zero at rest, with configured finite uncertainty.

Use the same bounded capture and handling procedure if collecting again. If acquisition stalls, spend at most five minutes troubleshooting; the controlled fallback with a different noise seed lets you continue. It generates data with the frozen stationary correction and declared acceleration/range noise settings. This tests implementation consistency under those assumptions; their transfer to the drone requires fresh measured data. Identify its source; experimental validation remains outstanding with simulation alone.

**Completion:** check one prediction on fresh data and decide whether the frozen assumptions remain credible. Keep one comparison and one limitation. Do not claim independent accuracy from agreement with a sensor that the filter already uses.

## E7: carry the filter's uncertainty into a race decision

**Task (25 minutes).** Take actual covariance at the same timestamp from E3 and E5's range-gap run. Use its height variance as the vertical entry of a position covariance; label the horizontal entries as assumptions because this filter estimates only vertical motion.

Project into a gate plane using `C_plane = U.T @ C_position @ U`. Compare a vertical and horizontal opening. Explain why increasing height uncertainty affects one projection but not the other. Mapping height variance to world Z assumes a known fixed floor; floor/survey uncertainty needs its own allowance.

Include radial offset, drone radius, tracking margin and a chosen sigma allowance in clearance. Write a small speed rule with a slower regime for small positive clearance and zero forward speed when clearance is non-positive. Explain how approach distance and braking would matter physically. A worst-axis sigma multiplier is not an exact collision probability; overlapping tracking-error and estimator margins can double-count uncertainty.

**Completion:** show the chain **missing measurement → larger covariance → changed clearance → justified action**. Keep the projections and candidate decisions. These recommendations do not alter the prescribed motion or establish faster lap times. The actual course is surveyed and checked on Day 3.

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
coverage95_y,coverage95_z,chi2_mean`. Use empty values for unavailable or
inapplicable quantities, never zero to mean missing. Sensor-specific rows keep
different chi-square dimensions separate. If quoting success rates, include numerator,
denominator, uncertainty, and the definition of a valid crossing.

Discuss what we measured, what we assumed and what the data cannot determine.
Explain a residual, pull and chi-square as you would for a collider track fit. Why is an
in-loop residual not an independent performance measurement? Two teams present;
we can share other results during practical work or in parallel groups.

## Stretch exercises

Once the core evidence is complete, teams interested in going further can
explore one of these extensions:

| Extension | Task | Additional evidence |
| --- | --- | --- |
| Bias state | Add accelerometer bias states with a stated random-walk model. | Recovery of injected bias, covariance, and observability under dropout; compare fixed calibration. |
| Adaptive Q | Increase model uncertainty using a justified manoeuvre indicator. | Held-out consistency and accuracy versus fixed Q, including false triggers. |
| Robust range | Compare chi-square rejection with [Tukey weighting](glossary.md#predicted-residual-gating-and-tukey-weighting) for a raised floor. | Effective noise variance, rejection/weight history, and recovery; never mistake floor change for independent Lighthouse truth. |
| Process input versus measurement | Design two explicit models using acceleration differently. | State and noise definitions, discretization, correlations, and a matched-data comparison without double-counting IMU information. |
| Attitude error-state EKF | Extend to quaternion attitude and IMU bias estimation. | Stationary and turning tests, quaternion norm, bias observability, and comparison to the six-state baseline. |
| Joint estimation/control | Tune an estimator and constrained PD/feed-forward or LQG controller together. | Fixed safety limits, closed-loop tracking/saturation/effort results, completion rate, and separation of estimator error from controller lag. |

The [race qualification and fallback policy](race_practical.md#r5-qualify-and-freeze-the-race-configuration)
applies throughout. Before using live code for control, the team completes and
cross-checks simulation, independent calibration, connection, emergency-stop and
low-speed tests, recording their outcomes. Keep the onboard attitude stabilization active.
