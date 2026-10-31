# Day 3: race

[Programme](../README.md) | [Day 1](day_1_control.md) | [Day 2](day_2_kalman.md)

Today we turn Day 1's tracking results and Day 2's calibration and uncertainty
studies into a checked racing line. We aim to justify a path and speed, make a
team flight decision, and explain the recorded motion.

Open the [Day 3 notebook](../code/day3/day3_race.ipynb) in the same Jupyter
environment as Days 1-2. It contains the working instructions and completion
checks, including measured-data and labelled offline routes. The
[flight checklist](race_practical.md) is available during physical operations.

## Schedule

[Indico event 8196](https://indico.nikhef.nl/event/8196/),
5 November 2026, Europe/Amsterdam. Timed competition starts at
**T = 14:00**. Agree the timetable and shared flight slots at the morning
briefing. If the start time changes, update the deadlines for estimator and controller
changes as well.

| Time | Activity |
| --- | --- |
| 09:00-09:30 | Challenge introduction: R1 rules/roles/handover (15 min), safety and flight-slot briefing (15 min). |
| 09:30-10:10 | R2: shared gate survey with supplied code and independent geometry checks. |
| 10:10-10:30 | Survey retries, hardware troubleshooting or handover. |
| 10:30-10:45 | **Break.** Share survey files and any labelled fallback geometry before the next practical. |
| 10:45-11:20 | R3: choose a margin budget and compare one speed change. |
| 11:20-11:50 | R4: reuse one Day 1 and one Day 2 result, check the selected plan and run the flight-tool dry run. |
| 11:50-12:20 | R5: team flight checks, low-speed test and configuration handover. |
| 12:20-12:30 | Catch-up and record frozen estimator configuration. |
| 12:30-13:30 | **Lunch.** Estimator freezes at 12:30 and controller at 13:00. Record both before lunch if not returning until 13:30. |
| 13:30-14:00 | All algorithm changes stopped; final safety checks and flight-slot preparation. |
| 14:00-15:40 | R6: shared timed competition, including recovery and battery changes. |
| 15:40-16:15 | R7: post-flight analysis and short notes, within Indico's race block. |
| 16:15-17:00 | Shared results and closing discussion. |

Racing stops at 15:40 to leave 35 minutes for R7. Teams keep short shared notes;
up to five teams volunteer to present a result or diagnosis for five minutes
each during the closing session, followed by 20 minutes of shared discussion.
We compare other results during practical work or in parallel groups.

## What runs on the drone and on the laptop?

| Function | Drone firmware | Laptop Python |
| --- | --- | --- |
| Estimation | Fuses IMU/deck observations into position, velocity and attitude. | Logs channels; may run a [shadow estimator](glossary.md#shadow-estimator). |
| Tracking | Closes position, attitude and angular-rate loops and drives motors. | Plans and streams world XYZ position and fixed-yaw targets. |
| Monitoring | Runs onboard protections. | Records telemetry and checks tracking, crossings and timing; abort landing is best-effort. |

The basic runner does **not send acceleration feed-forward**.
Day 1's position loop produced velocity commands; its gains have different
units from the race interface. Preserve the checked command interface and
onboard stabilization. The fast firmware loops run locally; streaming still
needs a timely radio connection and a separate emergency-stop operator.

## How Day 2 feeds into the race

Use calibration, covariance and dropout/outlier results to choose margins and
interpret tracking. Python corrections and Q/V settings are **not automatically applied to firmware**.
Changing the replay filter alone changes its diagnostic results, not the flown
baseline. Day 2's height filter uses sensor-level accelerometer/ToF readings;
it estimates vertical uncertainty only. Horizontal planning uncertainty needs
separate evidence or an explicitly stated assumption. Optional onboard-estimate
replays use fused, correlated observations.

A filter in the live feedback path is an optional extension requiring a tested
[adapter](glossary.md#adapter-and-runner), frame/unit/timing checks and low-speed
qualification. Keep the checked baseline until that work is ready.

## Freeze points and fallback

The schedule fixes estimator, controller and final algorithm deadlines.
Apply the [configuration checks and fallback rules](race_practical.md#r5-qualify-and-freeze-the-race-configuration)
before flight, including after material changes. A failed check means a checked
fallback or no flight; retain the failed attempt.

## Analysis and discussion

Use the notebook to compare planned and recorded motion. Keep one useful plot,
the chosen margin/speed, outcome and next change in the team's existing notes.
Physical observations establish crossing validity; estimated tracking and planned
duration answer different questions. The [plotting reference](post_flight_analysis.md)
documents terminal options and telemetry alignment.

## Reference and optional material

Look up [survey details](hoop_survey_challenge.md),
[course fields and launch commands](course_interface.md) or
[plotting options](post_flight_analysis.md) when needed.
The notebook includes the core instructions.

**Optional:** the [planner notes](trajectory_construction.md) explain its bounds,
limitations and jerk-continuity extension. The
[race extensions](race_practical.md#stretch-exercises) cover racing lines,
adaptive speed, survey uncertainty and banked turns. Uploaded polynomials and
external full-state control need their own tested tools and qualification;
onboard attitude stabilization remains active. The external LQG benchmark also
disables Mellinger's position-feedback gains during its outer-loop control.
See the [flight interface](course_interface.md#control-and-telemetry-interface)
and [profile descriptions](../code/race_common/README.md#fallback-profiles).
The [notes template](templates/race_report.md) is optional.

