# Day 1: feedback control

[Programme and setup](../README.md) | [Day 2](day_2_kalman.md) | [Day 3](day_3_race.md)

We use simulation and measurements to understand feedback, tune controllers
and compare their predictions with a real drone. The
[Day 1 notebook guide](../code/day1/README.md) gives the working sequence;
the notebooks contain the experiments, equations and flight instructions.

## Schedule

[Indico event 8196](https://indico.nikhef.nl/event/8196/), 3 November 2026,
Europe/Amsterdam. These blocks follow the notebook programme.

| Time | Activity |
| --- | --- |
| 10:00-10:15 | Introduction: goals of the three days. |
| 10:15-10:45 | Software check: Jupyter primer, the first simulation window, the connection to the group's drone. |
| 10:45-11:30 | Feedback-control lecture: feedback, stability and PID controllers. |
| 11:30-12:30 | Simulation, block 1: keyboard flight and system identification, `2Dsim_01_keyboard_flying` and `2Dsim_02_system_identification`. |
| 12:30-13:30 | **Booked PhD lunch, Veltman Centre.** |
| 13:30-14:30 | Simulation, block 2: feedback loops and the velocity loop, the bridge to the hardware, `2Dsim_03_basic_control` and `2Dsim_04_velocity_loop`. |
| 14:30-14:45 | Introduction to the Crazyflie hardware. |
| 14:45-16:00 | The real drone: connection, first flights under the firmware, the hand test and the identification flight, `drone_01_connect` to `drone_04_identification`; the analysis notebooks alongside. |
| 16:00-16:15 | **Break.** |
| 16:15-17:00 | The real drone, continued: tuning the position loop in `drone_05_tuning`, with the flight plan and predictions from `drone_analysis_01_flight_plan`. |

Have the [software setup](software_setup.md) ready beforehand. If a setup or
hardware problem blocks progress, share a working laptop or checked recordings
and retain the unresolved checks. The notebook guide marks further simulation
experiments as optional.

## Learning goals

Explain how feedback, damping, integral action, feed-forward and command limits
affect tracking, stability and disturbance rejection. Use a measured response
to justify a controller choice, including its command layer, signs and units.
The [hardware overview](drone_hardware.md) connects the sensors and actuators
to this model; the [glossary](glossary.md) explains the terminology.

## Flight session

Complete and cross-check the [hardware baseline checks](day_1_baseline_checks.md),
including the [mass/controller check](practical_workflow.md#hardware-preflight-mass-and-controller-assumptions).
Keep baseline verification separate from tuning. Follow the flight and abort
instructions in the flight notebooks, `drone_02_firmware_flight`,
`drone_04_identification` and `drone_05_tuning`; they contain powered flights.

All groups fly at the same time in the marked area. One person is at the
keyboard with the cursor in the abort box, one person watches the drone; the
flight area must be clear and the onboard stabilization active. Test the abort
interaction before flying. The group members without the drone laptop run the
model of the drone in `drone_analysis_01_flight_plan` and prepare the
prediction for each flight; swap roles between runs.
If the notebook flight is unavailable, [B5's cfclient procedure](day_1_baseline_checks.md#b5-battery-and-first-supervised-hover)
provides a baseline hover alternative. Use checked shared hardware or recordings
when checks fail, and record an unperformed hover as outstanding.

## Notes and closing discussion

Keep one useful comparison, a failure or surprise, and a possible next change
in the team's shared notes, with links to saved runs and the preflight record.
The [shared conventions](practical_workflow.md) describe what makes a comparison
repeatable; the [notes template](templates/controller_report.md) offers optional prompts.

The figure and the table of each group are shown on one of the following days.
Two teams volunteer to present one result or difficulty for five minutes each,
followed by 20 minutes of discussion. Compare model predictions, measured
response and uncertainty, including justified decisions not to fly. These
observations feed into Day 2's estimation studies and Day 3's trajectory choices.

### Optional background

- [Additional controller comparisons](controller_practical.md) and
  [CLI tools](controller_cli_examples.md): single-axis models, gain studies and replay.
- [Quadrotor equations of motion](quadrotor_equations_of_motion.md): frames,
  translation, rotation, hover, banked turns, and IMU measurements.
- [Gravitational-wave detector analogies](gravitational_wave_control_analogies.md):
  plants, loop shaping, lock acquisition, control noise, and calibration.
- [Detector-system analogies](detector_system_analogies.md): experimental context.
