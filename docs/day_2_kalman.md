# Day 2: Kalman filtering

[Programme and setup](../README.md) | [Day 1](day_1_control.md) | [Day 3](day_3_race.md)

The morning lecture introduces the theory, history and applications of Kalman
filters. We then investigate how sensor calibration, uncertainty, outliers and
missing measurements affect state estimation. Open the
[Day 2 numbered notebooks](../code/day2/README.md) in the Jupyter environment used
on Day 1; they contain the working instructions, code and completion checks.

## Schedule

[Indico event 8196](https://indico.nikhef.nl/event/8196/), 4 November 2026,
Europe/Amsterdam.

| Time | Activity |
| --- | --- |
| 09:00-10:30 | State estimation and Kalman-filter lecture, position–velocity demonstration and practical briefing. |
| 10:30-10:45 | **Break.** |
| 10:45-11:55 | E1-E2: track following and sensor calibration (25 + 45 min). |
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

The exercises total 215 minutes; E3 continues after lunch. The E1 illustration
challenge closes at noon. Two teams share a result or diagnosis for five minutes
each at 16:30, followed by 20 minutes of discussion.

## Learning goals

Use the [state and covariance notation](glossary.md#filter-covariances) to explain
a prediction, measurement update and velocity correction. Distinguish calibration
from noise modelling, interpret predicted residuals and chi-square, and test
frozen assumptions on separate data. Use the resulting uncertainty to justify a
race-day margin or speed choice.

E1 uses a particle track; E3-E7 use sensor-level acceleration and range to
estimate height and vertical velocity. Live acquisition is motor-free, with a
five-minute troubleshooting limit and explicit saved/simulated fallbacks.
Follow the notebook's handling steps: wait for **STOP VERIFIED** before handling.
The helper monitors zero motor outputs throughout and leaves the firmware stop
engaged. Place the drone down and power-cycle before normal flight preflight.
Detailed formats and the cfclient alternative are in the
[recording reference](sensor_recording_contract.md).

## Connection to Day 3

Day 1's outer position loop and Day 2's estimator serve different purposes.
The [Day 3 handover](day_3_race.md#how-day-2-feeds-into-the-race) explains how the
calibration and uncertainty results inform planning while onboard estimation
and control remain responsible for flight.

## Notes and closing discussion

Use the notebook's completion checks and keep one useful comparison, a calibration
finding and a race-day recommendation in the team's existing notes. Link the
recordings, settings and plots rather than copying them. The
[shared conventions](practical_workflow.md) and optional
[notes template](templates/kalman_filter_report.md) support this record.

### Optional background

- [Filter reference and extensions](kalman_filter_practical.md): tracking
  conventions, interpretation caveats and additional studies.
- [Calibration reference](sensor_bias_calibration_exercise.md): reference
  conditions and optional sensor characterization.
- [Quadrotor equations](quadrotor_equations_of_motion.md): frames and IMU measurements.
- [Collider-tracking analogies](collider_tracking_kalman_analogies.md) and
  [detector-system analogies](detector_system_analogies.md).
