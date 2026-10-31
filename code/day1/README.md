# Day 1: feedback control, from the simulator to the real drone

Andreas Freise, 28.10.2026

Hands-on material for the first day of the topical lectures. In the morning
you learn the basics of feedback control by flying a simulated drone in
Jupyter notebooks. In the afternoon the same position controller flies a real
Crazyflie, by radio, from the same notebook environment.

The notebooks are meant to be worked through in order. Each one is a task
sheet: read the text, run the cells, change the code where the text asks you
to. Keep `module.py` and `crazyflie.py` in the same directory as the
notebooks.

## Schedule

The day in [Indico event 8196](https://indico.nikhef.nl/event/8196/),
3 November 2026, and the notebooks that belong to each block.

* 10:00 to 10:15. Introduction: goals of the three days, the drone and its sensors.
* 10:15 to 10:45. Software check. Everyone runs `00_jupyter_example`
  top to bottom and the first cells of `2Dsim_01_keyboard_flying` (this opens a
  separate plot window and needs the Qt backend from the full installation).
  One person per group connects to the group's drone with cfclient and then
  runs `python crazyflie.py status` in a terminal.
* 10:45 to 11:30. Lecture: feedback, stability, PID controllers.
* 11:30 to 12:30. Simulation, block 1: `2Dsim_01_keyboard_flying` (10 min, flying
  without a controller is hard) and `2Dsim_02_system_identification` (measure
  the motor offset and the hover voltage of your drone; the sections marked
  *optional* can be skipped).
* 12:30 to 13:30. Lunch.
* 13:30 to 15:00. Simulation, block 2: `2Dsim_03_basic_control` (tilt loop,
  vertical loop with and without feedforward, the integrator, then the
  cascade for horizontal position; the last section is optional) and
  `2Dsim_04_velocity_loop` (a velocity loop between position and tilt, with
  a speed limit in physical units). Part 4 is the bridge to the
  afternoon: its position loop produces a velocity command, and that is
  exactly what the real drone accepts by radio.
  Groups that finish early: `2Dsim_05_race` (the race, flown by a
  preprogrammed sequence of setpoints, with an inline video) or
  `extra_2Dsim_01_race_interactive` (the same race flown with the keyboard,
  Qt backend).
* 15:00 to 15:15. Break.
* 15:15 to 16:30. The real drone, see below.
* 16:30 to 17:00. Results and discussion.

## The real drone, 15:15 to 16:30

Each group has one drone, but four people. One group member connects to the
drone and runs the flight notebooks, one watches the drone and the abort key
during flights, and the others work on the prediction and the analysis in
notebook `drone_04_analysis`, which needs no drone. Swap roles between runs.

1. 15:15 to 15:40, all groups at their desks: `drone_01_connect`, then
   `drone_02_handtest`. List the drones in range, pick yours by the label
   on its sticker, see it blink and read its status. Then the measurement
   protocol with the motors off: the drone still on the desk, then moved by
   hand forward and to the left. It gives the direction of the axes and the
   noise and the drift of the position estimate, saved to `data/drone_id.csv`.
   Part 3 refuses to fly without that file.
2. From 15:40, groups in turn in the flying area: `drone_03_position_loop`.
   First the identification flight under the firmware's controllers, a
   velocity command step whose response gives the time constant `tau` of the
   velocity loop. Then the design: `omega0` and `zeta` chosen, gains from
   `tau`. Then the flight with the designed gains: take-off and hover under
   the onboard controller, handover, 4 s hold, a 0.5 m step, 8 s hold,
   landing. One run takes about 20 s in the air and three minutes with setup
   and saving. Save every run under its own label, then tune: faster, more
   or less damping, both axes, each against the prediction of the model.
3. `drone_04_analysis`, part B: the measured step responses of your runs next
   to the two predictions (the first order model with the measured `tau` and
   the simulator of the morning), rise time, overshoot, settling, the hold
   phase next to the noise and drift of part 2, battery and motor headroom.
   The reference recordings under `data/reference/` serve while your pilot is
   still flying.

Flying rules: the marked area only, one person at the keyboard with the cursor
in the abort box (Enter lands the drone), one person watching, nobody within
arm's length of a running drone, a full battery. The loop lands by itself
when the drone leaves a 1 m box around its start, when the radio link or
the position estimate fails, or when the battery sags.

## Files

* `00_jupyter_example`: Jupyter primer. Skip if you have used notebooks before.
* `2Dsim_01_keyboard_flying`: first flight, keyboard control of the raw motor voltages.
* `2Dsim_02_system_identification`: measure your simulated drone (motor offset, hover voltage, clipping limits).
* `2Dsim_03_basic_control`: the first feedback loops. Tilt PD, then a flat cascade for altitude and lateral position.
* `2Dsim_04_velocity_loop`: a velocity loop between the position controller and the attitude loop, with the speed limit at the right place.
* `2Dsim_05_race`: the race, flown by a preprogrammed sequence of setpoints. The end of the simulator part.
* `drone_01_connect`: the drones in range, yours by its label, connect, status, a blink of its LEDs, the height sensor.
* `drone_02_handtest`: what the drone measures, with the motors off: the direction of the axes, the noise and the drift of the position estimate, from a guided protocol by hand. Writes `data/drone_id.csv`, which part 3 requires.
* `drone_03_position_loop`: the identification flight (response time of the velocity loop), the design of the position loop from it, the flight with a step response, tuning.
* `drone_04_analysis`: predicted and measured step responses, the hold phase against the noise of part 2.

Extras, not part of the day, for groups that finish early and for later:

* `extra_2Dsim_01_race_interactive`: the same race as `2Dsim_05_race`, flown with the keyboard (Qt backend).
* `extra_2Dsim_02_model_based_control`: the race with an algebraic inversion of the dynamics instead of the cascade.
* `extra_2Dsim_03_trajectories`: smooth trajectories through the waypoints with feedforward, on top of the inversion.

Supporting files:

* `module.py`: the simulator (`Drone`, `Plotter`, `PIDController`, `accel_to_phi_F`, `render_replay`).
* `crazyflie.py`: the radio link, logging, the two flight loops, saving and loading of runs. Also a terminal tool: `python crazyflie.py find | status | flow | recover | reboot | charge`, and for the instructors `config` and `relabel`.
* `drone_parameters.md`: where the simulator's parameters come from.
* `data/reference/`: recorded sensor identification, identification flight and two step flights, for the analysis without a drone.
* `testing/`: the tests, run from this folder with `python testing/test_module.py` and `python testing/test_crazyflie.py`. The second one runs the flight loops against a fake drone and needs no hardware; it is also the check that cflib and the loops work in your environment.

## Software

The simulation notebooks need numpy, scipy and matplotlib, as pinned in the
course requirements. `2Dsim_01_keyboard_flying` and `extra_2Dsim_01_race_interactive` open a separate animated window and
need the Qt backend, which the full installation provides through cfclient;
all other notebooks plot inline. The flight notebooks need cflib, also part of
the full installation. The radio address of your drone follows from the label
on its sticker, entered at the top of each flight notebook; `drone_01_connect`
lists the drones in range and checks yours.

## Drone model

The simulated drone is a 2D slice (y, z, phi) of a Crazyflie 2.1, with
parameters matched to the high-fidelity Simulink model in Richardson and
Pretorius, MATEC Web of Conferences 417, 04007 (2025). See
`drone_parameters.md` for the full parameter list and where each value comes
from. The real drone uses its optical flow deck and height sensor for the
position estimate; the horizontal position loop is the only loop that runs on
the laptop, the velocity and attitude loops run in the firmware.
