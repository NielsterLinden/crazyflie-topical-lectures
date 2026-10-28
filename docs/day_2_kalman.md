# Day 2: Kalman filtering

[Programme and setup](../README.md) | [Day 1: control](day_1_control.md) | [Day 3: race](day_3_race.md)

Today we explore sensor calibration and state estimation: how well does a
filter's quoted uncertainty describe its errors? Simulation, live motor-free
measurements and saved recordings give us complementary ways to find out.

## Schedule

[Indico event 8196](https://indico.nikhef.nl/event/8196/), 4 November 2026,
Europe/Amsterdam. Have the shared recordings and metadata ready before E2/E3.
[Calibration](glossary.md#bias-and-calibration) comes before
[covariance](glossary.md#covariance) tuning, using checked measurements or supplied data.
E6 uses the supplied motor-free live helper; a tested recording is the immediate
fallback. Writing acquisition or conversion code is not part of its 30 minutes.
Prefer live acquisition where supported; reuse one short capture for E3/E4/E5
position studies and E6. Cap acquisition/data troubleshooting at five minutes
within the existing slots, then switch to the checked recording. Controlled
synthetic scenarios supply truth and the sensor-specific studies the live helper
cannot support. Preserve unexpected observations for later investigation.

| Time | Activity |
| --- | --- |
| 09:00-10:30 | State estimation and Kalman-filter lecture, examples and practical briefing. |
| 10:30-10:45 | **Break.** |
| 10:45-11:55 | Kalman-filter practical: E1-E2, model/baseline and sensor calibration (25 + 45 min). |
| 11:55-12:30 | E3: uncertainty tuning and consistency (35 min). |
| 12:30-13:30 | **Lunch.** |
| 13:30-14:25 | Sensors/data practical: E4-E5, outliers, dropout and stationary constraints (30 + 25 min). |
| 14:25-15:00 | Catch-up, sensor demonstrations or motor-free data collection/validation extension; check live-helper readiness if useful. |
| 15:00-15:15 | **Break.** |
| 15:15-16:10 | Crazyflie experiments: E6-E7, live motor-free estimation (replay fallback) and uncertainty-based race decisions (30 + 25 min). |
| 16:10-16:30 | Compare results with another team and finish short notes. |
| 16:30-17:00 | Shared results and closing discussion. |
| 17:30-18:30 | **Booked pizza, Vertex.** |
| 19:00-21:00 | **Booked social event, ALOHA.** |

Core exercises total 215 minutes. Teams keep short shared notes. During the
16:30 closing session, two teams volunteer to present an experimental result or
diagnosis for five minutes each, followed by 20 minutes of shared discussion.
We compare other results during practical work or in parallel groups.
Lunch and evening bookings follow the public timetable.

## Learning goals and lecture material

Day 1 showed how noise, bias and lag affect feedback. Today we study the estimator
behind those inputs. The discrete position-velocity model lets us explore states,
process inputs and measurements, [Q/R/P](glossary.md#filter-covariances),
[innovations](glossary.md#innovation-and-nis), [pulls](glossary.md#pull-and-coverage),
[NIS](glossary.md#innovation-and-nis), [observability](glossary.md#observability)
and dropouts. The resulting uncertainty gives us a basis for race-day choices.

Controller gains and Q/R serve different purposes. The six-state filter takes
world acceleration as its [process input](glossary.md#process-input-and-process-noise). Andreas's controllers operate at the
velocity and tilt command layers. Closing the loop with the estimate needs
code connecting these command layers, and a simulation in which the controller's
commands change the drone's motion.

The core participant filter runs on the laptop. The race drone retains its
separately configured onboard estimator and controller. The
[Day 3 handover](day_3_race.md#how-day-2-feeds-into-the-race) explains how calibration
and uncertainty studies inform race choices, and what a live-feedback extension
would require. E2 includes sensor calibration and E6 uses live data during slow
hand motion. E2's cfclient recording uses the manual `stabilizer.stop=1` steps;
the live E6 helper engages and verifies the firmware stop automatically. Follow
the [hand-held recording preparation](sensor_recording_contract.md#prepare-the-drone-for-hand-held-recording)
and wait for verification before handling. Both include analysis; neither requires flight.
The 14:25-15:00 block offers optional demonstrations or motor-free collection.

### Optional background

The lecture and E1-E7 worksheet introduce the filter model and diagnostics used
in the practical. The notes below add derivations and detector-system context;
the scheduled experiments do not depend on reading them in full.

- [Quadrotor equations](quadrotor_equations_of_motion.md): frame conventions and
  inertial measurements underlying the simplified estimator.
- [Collider-tracking Kalman analogies](collider_tracking_kalman_analogies.md):
  propagation, material effects, hit updates, alignment, pulls, and efficiency.
- [Detector-system analogies](detector_system_analogies.md): residuals, chi-square,
  calibration nuisance parameters, pseudo-experiments, and performance working points.

## Practical sessions

Open the [Day 2 notebook](../code/day2/day2_kalman.ipynb) in the same Jupyter
environment used on Day 1. Use it to compare filter assumptions, implement the
outlier and stationary-update policies, and analyse recorded data. Its E1-E7
sections correspond to the worksheet;
the command-line examples below are alternatives for the same work.

The [E1-E7 worksheet](kalman_filter_practical.md) suggests a route through the
experiments; [shared conventions](practical_workflow.md) help comparisons. Before recording, agree channels,
timestamps, frame/unit conversions and calibration/held-out data using the
[recording guide](sensor_recording_contract.md). Day 1's state-estimate CSVs
alone do not supply the sensor data needed by E2/E6.

E3-E5 use a supplied experiment runner for matched scenarios, per-sensor
diagnostics and saved plots/metrics. The hands-on work is choosing Q/R and
implementing two short gate/stationarity decisions, then checking and explaining
their effects. The runner already records inputs, makes plots and repeats runs
with different random seeds; larger studies and extra fault scenarios are extensions.

| Exercise | Main task |
| --- | --- |
| E1 | Explain the supplied model and reproduce the baseline. |
| E2 | Check stationary IMU data and validate one gyro correction decision on held-out data; record the position-noise assumption for E3. |
| E3 | Tune Q/R and compare accuracy with uncertainty consistency. |
| E4 | Select dimension-aware NIS gates and test outlier rejection. |
| E5 | Investigate dropouts, observability, and conditional [zero-velocity updates](glossary.md#zupt). |
| E6 | Compare two filters on one live stream without commanding motors; retain recording/replay as fallback. |
| E7 | Use projected covariance to justify hoop-clearance and speed decisions. |

Complete the [45-minute E2 calibration check](sensor_bias_calibration_exercise.md#e2-core-45-minutes)
before tuning; the broader sensor studies are optional extensions. Powered flight
also needs the team's hardware/preflight checks. Calibration addresses systematic offsets; noise covariance
describes random variation. Onboard estimates combine filtered sensor information
and can share inputs with the participant filter.

Use the [starter code](../code/day2/README.md). With the native environment
activated, run from the repository root:

```bash
cd code
python -m day2.simulate
python -m unittest kalman.test_filter
cd ..
jupyter lab
```

Docker is optional. Use supplied geometry for simulation; determining the actual
physical gate geometry is a separate [Day 3 assignment](hoop_survey_challenge.md).

## Notes and closing discussion

At most one page in the shared team notebook or notes file is enough: a useful filter
comparison, a calibration finding, a failure or surprise and a race-day idea,
with links to calibration checks, pre-gate diagnostics and the two E6
outputs. The [short template](templates/kalman_filter_report.md) is optional;
the [worksheet](kalman_filter_practical.md#submission-and-closing-discussion)
lists what to keep. The supplied helper handles live acquisition; prepared data
provide the fallback. Developing another adapter or detailed timing tests needs extra time.

In discussion, separate measurements from assumptions and say what the data
cannot determine. Simulation and offline replay complete the core.

If time permits, try bias states, adaptive Q, robust range fusion, an attitude
[error-state EKF](glossary.md#ekf-and-error-state-ekf) or joint estimator/controller tuning. Keep the known-good fallback.
