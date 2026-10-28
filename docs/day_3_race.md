# Day 3: race

[Programme and setup](../README.md) | [Day 1: control](day_1_control.md) | [Day 2: Kalman filtering](day_2_kalman.md)

The morning brings shared gate measurements, trajectory choices and flight
[qualification](glossary.md#qualification-and-fallback); the afternoon brings the race and a look at what actually happened.
We compare speed, repeatability and valid crossings across runs.

## Schedule

[Indico event 8196](https://indico.nikhef.nl/event/8196/),
5 November 2026, Europe/Amsterdam. Timed competition starts at
**T = 14:00**. Instructors confirm the timetable and shared flight slots before
R1; changes to T require publishing the corresponding freeze times too.

| Time | Activity |
| --- | --- |
| 09:00-09:30 | Challenge introduction: R1 rules/roles/handover (15 min), safety and flight-slot briefing (15 min). |
| 09:30-10:10 | R2: shared gate survey with supplied code and independent geometry checks. |
| 10:10-10:30 | Survey retries, hardware troubleshooting or handover. |
| 10:30-10:45 | **Break.** Share survey files and any labelled fallback geometry before the next practical. |
| 10:45-11:20 | R3: conservative and candidate strategies. |
| 11:20-11:50 | R4: earlier evidence, planned-course checks and dry runs. |
| 11:50-12:20 | R5: team flight checks, low-speed test and configuration handover. |
| 12:20-12:30 | Catch-up and record frozen estimator configuration. |
| 12:30-13:30 | **Lunch.** Estimator freezes at 12:30 and controller at 13:00. Record both before lunch if not returning until 13:30. |
| 13:30-14:00 | All algorithm changes stopped; final safety checks and flight-slot preparation. |
| 14:00-15:40 | R6: shared timed competition, including recovery and battery changes. |
| 15:40-16:15 | R7: post-flight analysis and short notes, within Indico's race block. |
| 16:15-17:00 | Shared results and closing discussion. |

Instructors allocate flight slots. While waiting, continue motor-free work or
analyse supplied recordings. A slot is usable only if the team's flight checks pass; if they
cannot be completed in time, use a qualified fallback or do not fly.
Racing stops at 15:40 to leave 35 minutes for R7. Teams keep short shared notes;
up to five teams volunteer to present a result or diagnosis for five minutes
each during the closing session, followed by 20 minutes of shared discussion.
We compare other results during practical work or in parallel groups.

## Practical sessions

The [R1-R7 worksheet](race_practical.md) offers a route through the race practical,
with [shared conventions](practical_workflow.md); the [short notes template](templates/race_report.md)
is optional.

| Exercise | Main task |
| --- | --- |
| R1 | Define rules, limits, roles, and controller/estimator handover. |
| R2 | Use the supplied survey tool and independently check the fitted gates. |
| R3 | Plan conservative and candidate strategies with justified margins. |
| R4 | Review Day 1/2 validation, compare planned-course checks, and record qualification gaps. |
| R5 | Qualify and freeze a specific configuration and fallback. |
| R6 | Record every serious attempt and independent crossing observations. |
| R7 | Analyse trajectory errors, repeatability, and failure causes. |

## Morning: determine the course and qualify the system

Use Day 1's response, saturation and sign/frame checks, and Day 2's calibration
and uncertainty results. Mind the interface: the conservative race runner sends
position setpoints; Andreas's position loop produces velocity. Do not copy gains
between these layers. A new racing line or continuous reference needs its own
limit and adapter checks; Day 1's tests did not simulate the race.

Agree the rules, measure and check the gates with the [supplied survey tool](hoop_survey_challenge.md),
compare strategies in simulation, and qualify the chosen controller/estimator.
Writing a survey method or fitter is an optional extension. The
[equations of motion](quadrotor_equations_of_motion.md) offer optional background
on the dynamics; the R1-R7 worksheet describes the checks used in the race.
The [course format and flight-tool guide](course_interface.md) explains how survey data
become inputs to the flight tools.
R4 reuses earlier tests and adds course checks; building an integrated race
simulator is optional and needs extra time.

Use the supplied [trajectory construction and preview tool](trajectory_construction.md)
to turn survey files into an ordered course and inspect the path for R2/R3.
It provides the planned reference for the practical. The qualified runner's own
dry run checks the path that its flight interface will execute.
Its [limitations and optional jerk-continuity extension](trajectory_construction.md#limitations-of-the-baseline)
offer further exploration if time remains.

## What runs on the drone and on the laptop?

[PID](glossary.md#pd-and-pid) and [Mellinger](glossary.md#mellinger-controller)
run in the drone firmware, where they control motor outputs.
In the conservative race setup:

| Function | Drone firmware | Laptop Python |
| --- | --- | --- |
| Measurements and estimation | Reads [IMU](glossary.md#imu)/decks; fuses observations into position, velocity and attitude. | Receives selected logged channels; may run a [shadow estimator](glossary.md#shadow-estimator). |
| Planning | Executes received references. | Builds the course/reference and chooses its timing. |
| Tracking | Closes position, attitude and angular-rate loops and drives motors. | Streams world XYZ position and fixed yaw setpoints. |
| Monitoring | Runs onboard protections. | Records telemetry and checks tracking, crossings and command timing; abort landing is best-effort. |

The basic runner does **not send acceleration feed-forward**
([feed-forward explained](glossary.md#feed-forward)), even if the planner
computed velocity and acceleration. Selecting Mellinger alone does not change
that interface. Laptop/radio delays affect command delivery and offboard feedback;
the fast onboard stabilization loops do not wait for a telemetry round trip.
Streaming still needs a timely connection and a separate emergency-stop operator.

Two optional execution paths change this division, not the need for qualification:
an uploaded polynomial is evaluated onboard, while an external outer-loop
controller can use telemetry to send full-state/acceleration references. In the
supplied external [LQG](glossary.md#lqr-and-lqg) implementation, Mellinger position-feedback gains are
temporarily disabled, but onboard attitude stabilization remains active.
These paths need their own tested tools and documented handover; neither is enabled by a
profile label or a change of controller number. See the
[flight-tool guide](course_interface.md#control-and-telemetry-interface).

### How Day 2 feeds into the race

Day 2's E1/E3-E5 simulations and E6 live/replayed data run the participant
filter on the laptop, without changing the onboard estimator. E2's calibration
checks identify sensor limitations; saved Python corrections and Q/R settings
are **not automatically applied to firmware**. E7 uses uncertainty to motivate
clearance/speed choices; implementing them in live control needs a qualified adapter.

Bring those findings into R3/R4: choose defensible margins, compare candidate
references and identify dropout/outlier risks. Changing the replay filter alone
will not change the flown baseline trajectory or its feedback. A shadow estimate
is useful for diagnostics, but is not a safety gate unless a tested runner
actually consumes it. Onboard estimates used as replay inputs remain fused,
correlated data, not independent raw measurements or position truth.

Putting the participant filter in the live feedback path is an optional extension:
it needs an explicit adapter, frame/unit and timing checks, coupled closed-loop
tests and separate low-speed qualification before the freeze. The onboard
attitude loop stays active. Keep the qualified baseline if that work is not ready;
there is no requirement to replace the firmware Kalman filter for the race.

## Freeze points and fallback

Apply the [R5 qualification and freeze policy](race_practical.md#r5-qualify-and-freeze-the-race-configuration).
Its deadlines are relative to the scheduled timed competition, not qualification
flights. The [race profiles](../code/race_common/README.md#fallback-profiles)
describe conservative, estimation-only, and full operation. Instructor fallback
geometry must be labelled; teams still discuss how its geometry was checked.

## Afternoon: race and compare

Follow [R6](race_practical.md#r6-execute-and-record-the-timed-runs) for official
attempts and [R7](race_practical.md#r7-compare-planned-estimated-and-physical-performance)
for analysis. Retain failed/aborted runs and independent physical-crossing
observations alongside telemetry; use the instructor-published timing and penalties.

## Post-flight visualization

With the native environment activated, from the repository root:
Use this single-line command in Bash or PowerShell.

```bash
python scripts/plot_race.py race-telemetry.csv --course path/to/team-course.json --phase race
```

The plot overlays sent goals and onboard position estimates in 3D and three
projections, shows gate/hoop positions, and plots x/y/z errors versus time. It
uses host-time alignment, not independent ground truth. For survey overlays,
saved PNGs, and takeoff/landing selection, see the
[post-flight plotting options](post_flight_analysis.md).

## Notes and closing discussion

At most one page in the shared team notes is enough: the survey/strategy, a trajectory
plot, the outcome and what we would change, with links to saved observations, course,
telemetry and the shared preflight/attempt log rather than copies. See the
[worksheet](race_practical.md#submission-and-closing-discussion).
We will discuss repeatability and unresolved questions, not just rank lap times.

