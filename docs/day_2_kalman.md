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
E3-E7 estimate height and vertical velocity from timestamped accelerometer and ToF measurements, using a level drone over a fixed flat floor. Reuse one motor-free capture for E3-E5; E6 checks frozen assumptions on fresh data. Capture includes a five-second rest calibration. If acquisition stalls, a checked sensor recording or the matching simulation lets you continue; spend at most five minutes troubleshooting.

| Time | Activity |
| --- | --- |
| 09:00-10:30 | State estimation and Kalman-filter lecture, position–velocity demonstration and practical briefing. |
| 10:30-10:45 | **Break.** |
| 10:45-11:55 | Kalman-filter practical: E1-E2, track following and sensor calibration (25 + 45 min). |
| 11:55-12:30 | E3, part 1: construct the timestamped height-velocity filter (35 min). |
| 12:30-13:30 | **Lunch.** |
| 13:30-13:45 | E3, part 2: matched Q/V comparisons and discussion (15 min). |
| 13:45-14:10 | E4: one range outlier and a chi-square gate (25 min). |
| 14:10-14:40 | E5: missing range and known-rest velocity constraints (30 min). |
| 14:40-15:00 | Catch-up, sensor demonstrations or preparation of fresh validation data. |
| 15:00-15:15 | **Break.** |
| 15:15-15:30 | E6: frozen filter settings on fresh data (15 min). |
| 15:30-15:55 | E7: use filter covariance for gate-clearance decisions (25 min). |
| 15:55-16:30 | Compare results with another team and finish short notes. |
| 16:30-17:00 | Shared results and closing discussion. |
| 17:30-18:30 | **Booked pizza, Vertex.** |
| 19:00-21:00 | **Booked social event, ALOHA.** |

The [E1 tracking exercise](kalman_filter_practical.md#e1-follow-a-particle-track)
uses an AI-assisted implementation or the supplied reference to check 1,000-track
pulls. The illustration challenge runs until noon, within the existing practical.
Core exercises total 215 minutes. Teams keep short shared notes. During the
16:30 closing session, two teams volunteer to present an experimental result or
diagnosis for five minutes each, followed by 20 minutes of shared discussion.
We compare other results during practical work or in parallel groups.
Lunch and evening bookings follow the public timetable.

## Learning goals and lecture material

Day 1 showed how noise, bias and lag affect feedback. Today we study the estimator
behind those inputs. The discrete position-velocity model lets us explore states,
process inputs and measurements, [Q/V/C](glossary.md#filter-covariances),
[predicted residuals](glossary.md#predicted-residual-and-chi-square), [pulls](glossary.md#pull-and-coverage),
[chi-square](glossary.md#predicted-residual-and-chi-square), [observability](glossary.md#observability)
and dropouts. The resulting uncertainty gives us a basis for race-day choices.

Controller gains and Q/V serve different purposes. E1 propagates position and
slope through scattering material; E3 replaces distance with time and estimates
height and velocity, taking acceleration as its [process input](glossary.md#process-input-and-process-noise). Andreas's controllers operate at the
velocity and tilt command layers. Closing the loop with the estimate needs
code connecting these command layers, and a simulation in which the controller's
commands change the drone's motion.

The core participant filter runs on the laptop. The race drone retains its
separately configured onboard estimator and controller. The
[Day 3 handover](day_3_race.md#how-day-2-feeds-into-the-race) explains how calibration
and uncertainty studies inform race choices, and what a live-feedback extension
would require. E2 includes sensor calibration; E3-E7 fuse sensor-level acceleration and range. E2's cfclient recording uses the manual `stabilizer.stop=1` steps; the vertical capture helper sets and verifies it automatically and monitors zero motor outputs throughout. Follow the [hand-held recording preparation](sensor_recording_contract.md#prepare-the-drone-for-hand-held-recording) and wait for verification and the stationary calibration prompt before moving gently. The 14:40-15:00 block offers catch-up or data collection.

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
environment used on Day 1. Complete four filter expressions, compare assumptions and inspect measured or simulated data. Its E1-E7 sections follow the worksheet. Logging and plotting are supplied so the practical work focuses on the recursive estimation.

The [E1-E7 worksheet](kalman_filter_practical.md) suggests a route through the
experiments; [shared conventions](practical_workflow.md) help comparisons. Before recording, agree channels,
timestamps, frame/unit conversions and calibration/held-out data using the
[recording guide](sensor_recording_contract.md). Day 1's state-estimate CSVs
alone do not supply the sensor data needed by E2/E6.

E3 exposes each prediction and measurement update: participants calculate the gain and explain how a height measurement corrects velocity through cross-covariance. E4 challenges the filter with an outlier; E5 removes range information and adds a justified velocity constraint. E6 checks frozen choices; E7 uses covariance from those runs. Automatic stationary detection and additional six-state sensor studies are optional.

| Exercise | Main task |
| --- | --- |
| E1 | Simulate and filter 1,000 particle tracks; explain final position/slope pulls. |
| E2 | Check stationary IMU data and validate one gyro correction decision on held-out data; identify the separate range-noise check for E3. |
| E3 | Complete prediction/gain/update expressions, then compare Q/V assumptions. |
| E4 | Select dimension-aware chi-square gates and test outlier rejection. |
| E5 | Trace covariance during range loss and compare justified/false [zero-velocity updates](glossary.md#zupt). |
| E6 | Test frozen calibration and Q/V on a fresh recording or independent simulation. |
| E7 | Project covariance from the filter runs into gate-clearance and speed decisions. |

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
with links to calibration checks, pre-gate diagnostics and the E6 fresh-data
comparison. The [short template](templates/kalman_filter_report.md) is optional;
the [worksheet](kalman_filter_practical.md#submission-and-closing-discussion)
lists what to keep. The supplied helper handles live acquisition; prepared data
provide the fallback. Developing another adapter or detailed timing tests needs extra time.

In discussion, separate measurements from assumptions and say what the data
cannot determine. Simulation and offline replay complete the core.

If time permits, try bias states, adaptive Q, robust range fusion, an attitude
[error-state EKF](glossary.md#ekf-and-error-state-ekf) or joint estimator/controller tuning. Keep the known-good fallback.
