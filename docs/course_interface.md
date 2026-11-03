# Course files and flight interface

This guide describes the shared course file format and flight-tool interface.
The [survey guide](hoop_survey_challenge.md) covers gate measurements.
The [synthetic course](../examples/course.synthetic.json) illustrates
the format only: it is **not measured, qualified, or enabled for flight**.
The [trajectory planner](trajectory_construction.md) builds and plots
ordered measured crossings and laps without a radio. Its exported course uses
this format; its polynomial reference is not automatically the runner's path.

## Flight-tool handover

Day 1's `drone_03_handtest.ipynb` checks a position-to-velocity callback with
motors stopped. `drone_05_tuning.ipynb` uses the adjacent `crazyflie.py`
helper for takeoff, hover, the step and landing; onboard velocity/attitude loops
remain active. The flight helper settles at hover and sets the target to the
measured position before handing over. Check command continuity at that switch
([bumpless handover](glossary.md#bumpless-handover)). The group's sign, frame,
mass, power and preflight checks apply before that powered sequence.

Day 2's live/replay tools save the [recording pack](sensor_recording_contract.md)
used for uncertainty and residual studies. These laptop-filter settings supply
evidence for Day 3's planning choices; their deployment to a flight controller
is a separate integration step.

For Day 3, agree a [run card](glossary.md#run-card-and-configuration-freeze): the
tested race tool's file, Python interpreter, directory/version, actual controller/estimator,
command units, limits, operator and stop path. Confirm access on the shared flight
laptop at R1, along with the race rules and times. The conservative race tool
streams position targets; the firmware position PID supplies velocity targets,
as Day 1's outer loop did. Check frame, units and timing before reusing numerical
gains. Keep a tested baseline and shared recordings available.

Use [the flight notebook](../code/day3/03_flight.ipynb) for the dry run and
flight. The run card supplies the runner file, its Python interpreter and working
directory. The notebook launches it in that environment, checks the exact course,
and retains its course-name confirmation and landing-on-abort behaviour.
Its **Connect** cell selects the drone from `LABEL` and leaves firmware stop
engaged. Afterwards, power-cycle the grounded drone and repeat preflight, keeping
the discovered URI without rerunning Connect. Cross-check with the stop operator
ready before enabling flight. Each dry run has a separate report; each flight
reads an archived copy of its exact course alongside its telemetry and console log.
Only one radio client controls a drone at a time.

The supplied runner also accepts an explicit `--tuning` JSON file. Its protocol
is checked during the dry run; older tools fail before flight instead of silently
ignoring the settings. The file binds one family and its factor to a captured
baseline URI. Core choices are horizontal position P gains (PID) or horizontal
acceleration process noise (Kalman). The optional `parameter_group` selects one
allowlisted stretch group described in the flight notebook: position/velocity
P, I or D gains, or vertical-acceleration, velocity or position process noise.
Factors scale positive baseline values; zero-valued groups are rejected.
Active controller/estimator and all baseline
values are checked before any tuning write. Writes and restoration require fresh
acknowledgements. The per-attempt `.tuning.json` and `.parameters.json` files record
the request, original/applied/restored values and any failure. Restoration runs
after landing/stop handling and before the connection closes. Failed restoration
keeps the attempt unsuccessful; verify baseline values before another flight.

**Before R2 acquisition**, try the example JSON in the flight notebook's dry run;
teams can help each other check its structure. Fix format mismatches before
surveying; successful validation does not qualify synthetic or measured geometry.
If the flight program, limits or run card are missing, continue with simulation
or recordings. Note what is missing and use a tested shared alternative
or supplied recordings.

## Course JSON format

The example contains every required field. All numeric values must be finite;
lengths are metres, times seconds, and vectors are three numbers in the same
right-handed Lighthouse world frame, with Z upward. Document origin, orientation,
geometry ID and the physical point used for drone position in the survey record.

| Field | Meaning |
| --- | --- |
| `course_name` | Nonempty course identifier, also used for interactive confirmation. |
| `enabled_for_flight` | JSON boolean; keep `false` until the team's preflight checks pass and the pilot and stop operator confirm readiness. Record that decision before enabling the course. |
| `uri_environment_variable` | Radio setting used by the standalone runner; the flight notebook supplies its `LABEL`-selected URI directly. |
| `yaw_deg` | Fixed world-frame heading reference in degrees, held throughout the course. |
| `start`, `finish` | Airborne trajectory endpoints; the runner checks compatibility between the resting location and takeoff start. |
| `hoops` | Nonempty ordered list of crossing events; repeated passages through a physical gate each have their own entry. |
| `limits` | Shared planning/monitoring settings listed below, checked by the team; preserve common limits when comparing strategies. |

Each crossing has a unique `name`, `centre`, unit `normal`, and positive `radius_m`.
Crossing progresses from the negative to positive side of
`dot(position - centre, normal)`. For a return crossing, repeat the same geometry
with the opposite normal and a different event name. For multiple laps, explicitly
repeat crossing events in order; there is no `laps` field. Link repeated event
names to the physical gate ID in your separate survey output.

For a rectangle, preserve full width/height/axes in `gates.json`, but use a
conservative inscribed circle for this runner: `radius_m <= min(width, height)/2`.
Use physical clear opening dimensions, not the outer frame. Optional plotting
fields are `shape: rectangle`, `width_m`, `height_m`, and orthogonal unit
`width_axis` / `height_axis` vectors in the gate plane.
Physical thickness, floor/support obstacles and swept-drone volume need separate
qualification; the circular aperture does not model them.

The participant planner also accepts `frame_margin_m` on each crossing: finite
physical padding beyond the inner opening, per side (default 0.10 m, to be
checked against the real frame/supports). It adds drone radius and clearance
when checking bypasses. This is planner metadata, not a change to the qualified
runner's scoring or collision checks; see the [trajectory guide](trajectory_construction.md).

All `limits` fields are required:

| Field | Units / interpretation |
| --- | --- |
| `setpoint_rate_hz` | Positive host position-command rate, Hz. |
| `target_speed_m_s`, `maximum_speed_m_s` | Positive planned/maximum speed; target must not exceed maximum. |
| `maximum_acceleration_m_s2` | Positive reference-acceleration bound, m/s2. |
| `maximum_tilt_deg` | Positive measured/planned tilt limit, degrees. |
| `maximum_tracking_error_m` | Positive measured-to-command position-error limit. |
| `telemetry_timeout_s` | Positive maximum telemetry age. |
| `drone_radius_m` | Positive conservative envelope radius, including cage. |
| `clearance_margin_m` | Additional positive aperture clearance; not part of the physical gate size. |
| `pre_post_distance_m` | Positive approach/exit waypoint offset along the normal. |
| `takeoff_duration_s`, `landing_duration_s` | Positive transition durations; actual runner may extend them for safe motion. |
| `bounds_min`, `bounds_max` | Opposite corners of the allowed box; each minimum strictly below maximum. |

Start, finish and gate centres must lie inside the box; every radius must exceed
`drone_radius_m + clearance_margin_m`. Planning checks the *full path*, not only
centres. The pilot and second team member also check support/floor/cage interference,
visibility, hardware state, start alignment, timing and actual tracking. Additional
shared safety settings are frozen on the run card, not tuned as survey
outputs. Do not silently add fields that the runner might ignore.

## Control and telemetry interface

The conservative launcher sends world-frame XYZ **position setpoints in metres**
and fixed yaw in **degrees** to onboard control; it does not accept participant
motor/PWM or offboard attitude commands. The onboard estimator supplies feedback.
The supplied conservative runner streams low-level position setpoints from the
laptop; it does not upload a high-level-commander trajectory. Continued host
scheduling and radio delivery are needed during takeoff, racing and landing.
Its interruption checks and abort landing are best-effort, not an independent
emergency stop. An uploaded onboard trajectory is a different execution path
requiring its own qualification and run-card description.
An offboard filter can be assessed in shadow mode without changing flight.
Using a participant controller/observer to drive hardware requires a separately
documented and tested [adapter](glossary.md#adapter-and-runner), including the
command interface, rate, timing, limits, aborts and active controller/estimator.

The operator returns the exact course, settings/run card, console events and CSV
for every attempt, including aborts. The [offline plotter](../scripts/plot_race.py)
requires `event,host_monotonic_s`: `position` rows contain `x_m,y_m,z_m` and may
include `firmware_timestamp_ms,roll_deg,pitch_deg`; `command` rows contain
`command_x_m,command_y_m,command_z_m,phase`. Phases distinguish takeoff, race and
landing. Command rows describe sent commands; rejected references must not be
mislabelled as sent. Separate diagnostic rows may add sensor/power channels.
Firmware and laptop timestamps use different clocks. Record which clock each
timestamp uses and how the clocks were aligned. Use separate observations to
check physical accuracy and the agreed timing method for race results.
