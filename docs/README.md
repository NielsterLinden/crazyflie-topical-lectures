# Documentation guide

The [hardware overview](drone_hardware.md) introduces the drone, Lighthouse and
Flow decks. The [installation guide](software_setup.md) covers setup, and the
[course overview](../README.md) describes the three days.

| Day | Programme | Experiments | Optional notes prompts |
| --- | --- | --- | --- |
| 1 | [Feedback control](day_1_control.md) | [C1-C7](controller_practical.md) | [Day 1 notes](templates/controller_report.md) |
| 2 | [Kalman filtering](day_2_kalman.md) | [E1-E7](kalman_filter_practical.md) | [Day 2 notes](templates/kalman_filter_report.md) |
| 3 | [Race](day_3_race.md) | [R1-R7](race_practical.md) | [Day 3 notes](templates/race_report.md) |

We compare results as we work. One shared team notebook or notes file is enough:
at most one page per day plus links to outputs; no separate exercise reports
are needed. The [shared conventions](practical_workflow.md) help others repeat
your comparisons. The
[qualification and freeze policy](race_practical.md#r5-qualify-and-freeze-the-race-configuration)
applies to all physical flight, including a fallback supplied by instructors.
The [team responsibilities](practical_workflow.md#responsibility-for-the-experiment)
describe the four-person cross-checks, shared decisions and flight roles.

## For the scheduled exercises

Start with the day's programme and worksheet in the table above. They introduce
the model and questions for each experiment and point to the relevant commands.
Use the guides below at the indicated stage; there is no need to read every
software reference page before starting.

| Stage | Guides to use |
| --- | --- |
| Before Day 1 | [Software setup](software_setup.md) and its software-only completion checks. |
| Updates and sharing | [Git workflow](course_git.md): team branches, merging course updates and proposing improvements. |
| Day 1 C1-C7 and first hover | [Controller software](day_1_software.md), [hardware baseline checks](day_1_baseline_checks.md) and the guided notebook. Complete the applicable preflight checks before flying. |
| Day 2 E2-E6 | [Sensor calibration](sensor_bias_calibration_exercise.md), the relevant [recording instructions](sensor_recording_contract.md) and [experiment commands](../code/day2/README.md). |
| Day 3 R1-R7 | [Course/runner interface](course_interface.md), [gate survey](hoop_survey_challenge.md), [trajectory construction](trajectory_construction.md) and [post-flight analysis](post_flight_analysis.md). |

Consult the [glossary](glossary.md) when a term is unfamiliar. Safety, recording
provenance and flight checks are part of the practical work, including when a
fallback is used. The longer notes below are optional reading, not additional
preparation or submissions.

## Further reading: theory and experimental context

These notes expand on the lectures and worksheet models. Read the sections that
interest you; the full derivations and detector comparisons are additional material.

- [Practical glossary](glossary.md): control, estimation and flight terminology.
- [Rotations and quaternions](rotations_and_quaternions.md).
- [Quadrotor equations of motion](quadrotor_equations_of_motion.md).
- [Detector-system terminology](detector_system_analogies.md).
- [Collider-tracking Kalman analogies](collider_tracking_kalman_analogies.md).
- [Gravitational-wave feedback analogies](gravitational_wave_control_analogies.md).

### Further courses and worked examples

- [Brian Douglas: What Is PID Control?](https://www.youtube.com/watch?v=wkfEZmsQqiA):
  an introduction to feedback and PID; see also the
  [Understanding PID Control playlist](https://www.youtube.com/playlist?list=PLn8PRpmsu08pQBgjxYFXSsODEF3Jqmm-y).
- [ETH Zurich: Quadrotors, Control and Estimation](https://www.dfall.ethz.ch/pands.php):
  Crazyflie lecture notes and practical sheets progressing from modelling and
  simulation to PID/LQR control and flight. Its software uses MATLAB/Simulink
  and C++; our exercises keep the supplied Python environment.
- [Roger Labbe: Kalman and Bayesian Filters in Python](https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python):
  executable explanations and exercises; chapters 4, 6 and 8 are particularly
  useful for scalar filtering, multivariate filtering and filter design.
- [MIT: Underactuated Robotics](https://underactuated.mit.edu/):
  lecture notes and videos; the quadrotor sections connect planar dynamics,
  tilt and thrust to feedback control. LQR and trajectory optimization offer
  further reading for the stretch goals.
- [NASA NESC: Fundamentals of Kalman Filtering and Estimation](https://nescacademy.nasa.gov/video/7287e7815cc049bdbaf4dc04830107341d):
  Chris D'Souza's seminar and slides connect estimation and least squares to
  practical aerospace applications.

These are optional references; the supplied environment and exercises contain
the material needed for the three days.

## Software reference

These pages provide commands and implementation detail for the exercise route
above. Consult the relevant sections while working; they are not a separate
reading assignment.

- [Day 1 controller experiments](day_1_software.md): controller models,
  guided notebook, simulation/replay and motor-disabled sign checks.

- [Course and qualified-runner interface](course_interface.md), with a disabled synthetic example.
- [Day 2 sensor acquisition/replay contract](sensor_recording_contract.md), with a synthetic row-format example.
- [Sensor calibration](sensor_bias_calibration_exercise.md).
- [Gate measurement with the supplied survey tool](hoop_survey_challenge.md).
- [Trajectory construction and preview](trajectory_construction.md).
- [Starter APIs and simulations](../code/day2/README.md).
- [Code layout and tests](../code/README.md).
- [Post-flight trajectory analysis](post_flight_analysis.md).

Raw observations, onboard estimates, simulated truth, and independent crossing
observations have different meanings; distinguishing them helps us interpret the results.
