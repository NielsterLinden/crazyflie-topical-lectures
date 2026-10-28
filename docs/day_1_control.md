# Day 1: feedback control

[Programme and setup](../README.md) | [Day 2: Kalman filtering](day_2_kalman.md) | [Day 3: race](day_3_race.md)

We start at **10:00** with an introduction and software checks, then turn to
the control-theory lecture at **10:45**, covering feedback, stability and PID
controllers. The [glossary](glossary.md) explains the terminology
used throughout the practicals. Most of the day is hands-on: model the drone,
tune controllers in simulation, and check their behaviour using recorded data.
We finish the practical with a scheduled first takeoff, hover and landing using
the qualified onboard controller. Each team completes and cross-checks its preflight record.

## Schedule

[Indico event 8196](https://indico.nikhef.nl/event/8196/), 3 November 2026,
Europe/Amsterdam. Exercise times include checking and recording results.
Each team keeps short shared notes. During the 16:30 closing session, two teams
volunteer to present one result or difficulty from their experiments, with five
minutes per team. We then spend 20 minutes discussing the results together.
These are short presentations, not additional flight slots. Other teams can
compare their results during practical work or in parallel groups.

| Time | Activity |
| --- | --- |
| 10:00-10:15 | General introduction: goals, three-day overview, drone hardware/sensors, Lighthouse and Flow decks. |
| 10:15-10:45 | Software verification: launch cfclient, detect the Crazyradio, connect to the assigned drone without arming or starting motors, and run a first simulation. |
| 10:45-11:30 | Feedback-control lecture and simulation briefing. |
| 11:30-12:15 | Drone simulation: C1-C2, model, mass/baseline checks and unchanged comparison. |
| 12:15-12:30 | C3: record the gain hypothesis and first position-loop comparison. |
| 12:30-13:30 | **Booked PhD lunch, Veltman Centre.** |
| 13:30-14:40 | Simulation/controller tuning: finish C3 (20 min), C4 (25 min), C5 (25 min). |
| 14:40-15:00 | C6: one matched noise/limit comparison (20 min); larger scans are optional. |
| 15:00-15:15 | **Break.** |
| 15:15-15:35 | C7: replay or motor-disabled sign checks and handover (20 min). |
| 15:35-15:45 | Catch-up, flight-slot briefing and final preflight review. |
| 15:45-16:30 | First supervised hover: team takeoff-hover-land slots with cfclient and the qualified onboard controller. |
| 16:30-17:00 | Shared results and closing discussion. |

The C1-C7 core totals 170 minutes, followed by 10 minutes for catch-up/preflight
and a protected 45-minute first-hover session. Extra tuning and participant-code
flight experiments do not consume that hover session or safety-check time.

Having the software installed via the [setup guide](software_setup.md) before Day 1
leaves more time for experiments. The
morning check allows minor fixes, not a full installation. If a problem takes
longer, pair up on a working laptop; a shared machine can help where available.

## Learning goals and lecture material

The [hardware and sensor guide](drone_hardware.md) connects plant, sensors,
reference, feedback and actuators, including the difference between Flow's
surface-relative measurements and Lighthouse's room coordinates.

By the end of the day, you should be able to explain tracking lag,
[damping](glossary.md#response-metrics),
disturbance rejection and [saturation](glossary.md#saturation), and use measurements to justify a controller
choice. Weigh the flight-ready drone and check how its controller uses mass.
Practise a supervised takeoff, settled hover and landing with the baseline setup.
In cfclient, record hardware condition, active controller/estimator, position and
attitude frames, telemetry timing, battery/propulsion behaviour and safety limits.
Check the setup before tuning; powered measurements use the team's documented
flight checks and a separate emergency-stop operator.

### Optional background

The lecture, C1-C7 worksheet and software guide introduce the models used in the
scheduled exercises. The notes below provide longer derivations and connections
to detector systems; explore them as interests or questions arise.

- [Quadrotor equations of motion](quadrotor_equations_of_motion.md): frames,
  translation, rotation, hover, banked turns, and IMU measurements.
- [Gravitational-wave detector analogies](gravitational_wave_control_analogies.md):
  plants, loop shaping, lock acquisition, control noise, and calibration.
- [Detector-system analogies](detector_system_analogies.md): links to experimental
  instrumentation and the limits of those analogies.

## Practical sessions

We follow Andreas's sequence: simulation, motor-disabled checks, then supervised
baseline hover and analysis. Applying participant-written control in flight remains
an optional, separately qualified extension. The [software guide](day_1_software.md) and
`andreas/day1_control.ipynb` use his position [PID](glossary.md#pd-and-pid), velocity-to-tilt controller,
[cascade](glossary.md#cascade), toy [plants](glossary.md#plant) and replay helpers. Follow the
[C1-C7 worksheet](controller_practical.md) for experiments, timings and suggested notes,
and the [shared conventions](practical_workflow.md) for recording comparisons.
Stretch exercises are optional.

The [cfclient baseline-check guide](day_1_baseline_checks.md) gives step-by-step
instructions and evidence requirements. Initial connection happens at 10:15;
remaining unpowered measurements accompany C1/C2. B5's powered measurements belong
to the protected first-hover session, not an optional addition to C7.

| Exercise | Main task |
| --- | --- |
| C1 | Derive the plant/controller model, signal diagram, and units; weigh the flight-ready drone and check controller mass assumptions. |
| C2 | Reproduce Andreas's position, velocity and cascade experiments. |
| C3 | Tune gains with step responses, damping, lag, and settling criteria. |
| C4 | Compare velocity-loop gains and nested-loop behaviour. |
| C5 | Compare PD/PID disturbance rejection, [anti-windup](glossary.md#anti-windup) and recovery. |
| C6 | Test command limits, sensing noise, and robustness. |
| C7 | Check command signs using replay or motor-disabled capture; prepare the hardware and race handover. |

Use [Andreas's practical runner](day_1_software.md). With `.venv` activated,
run from the repository root:

```bash
cd code
python -m day1 --output ../day1/baseline --no-show
python -m unittest andreas.test_practical
cd ..
```

Complete the unpowered preflight checks before the first hover. Simulator results
do not authorize flight, and the first hover does not test participant-written code.
Keep onboard attitude stabilization and the emergency-stop path. The supplied simulator
uses optional noisy-state feedback and approximate velocity/tilt inner loops.
Applying its gains to hardware requires validation at the matching command layer.

## First hover: 15:45-16:30

Plan one short takeoff-hover-land attempt per team on its assigned, checked
drone. Agree slots and operator roles early enough to fit the group into the
45 minutes, including handovers and recovery; do not leave this to spare time.
Only fly simultaneously in separately approved flight areas. Use shared hardware
if a team's drone is unavailable, with the same preflight checks.

Follow [B5's cfclient procedure](day_1_baseline_checks.md#b5-battery-and-first-supervised-hover):
the team's pilot uses Command Based Flight to take off to the agreed low
altitude, hold briefly, then land. A separate emergency-stop operator is ready
throughout. Use the unchanged qualified onboard estimator/controller, a clear
positioning-qualified area and the battery/preflight record. Do not introduce a
new Python controller, gain tuning, trajectory streaming or handover experiment.
Save the standard logs and discuss settling, drift and thrust/battery behaviour.

While one team flies, the next can prepare outside the active flight area;
the others can look at live telemetry or the preceding team's hover logs.
One observation about settling, drift or battery behaviour is useful for the
closing discussion. The group can use the flight slots as shared experiment time.

This is a planned part of Day 1. If the team's flight checks fail, stop and use
a checked shared-drone
demonstration or recordings, and mark the team's own hover outstanding. Never
relax checks to meet the timetable. The closing discussion still starts at 16:30.

## Notes and closing discussion

One short entry in the shared team notebook or notes file is enough: at most one page
on a useful comparison, a failure or surprise, and what we would try next.
Links to saved outputs avoid copying them; the [short template](templates/controller_report.md) is optional.
See the [worksheet](controller_practical.md#submission-and-closing-discussion).

Keep C1's mass/configuration and B1-B6 checks as one preflight record with
logs/settings, frame/timing measurements, acceptance criteria, team cross-checks and open
checks. The [preflight guide](practical_workflow.md#hardware-preflight-mass-and-controller-assumptions)
has **Set/Store** and reboot-readback steps. No flight is needed for this check.
Label supplied data, missing measurements and the first-hover outcome; keep tuning separate.

At the closing discussion, compare position, velocity and integral feedback:
what worked, what failed, and why? First-hover observations and reasons for not
flying are both useful; a justified no-flight decision is not a failed exercise.
Feed-forward and the acceleration-command example lead into Day 3's continuous
trajectories. [Loop margins](glossary.md#bandwidth-and-loop-margins),
[LQR/LQG](glossary.md#lqr-and-lqg) and [MPC](glossary.md#mpc) are optional; the simpler controller
and qualified fallback remain available.
