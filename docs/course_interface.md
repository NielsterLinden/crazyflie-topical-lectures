# Course files and flight commands

This guide describes the shared course file format and flight-tool interface.
The [survey guide](hoop_survey_challenge.md) covers gate measurements.
The [synthetic course](../examples/course.synthetic.json) illustrates
the format only: it is **not measured, qualified, or enabled for flight**.
It can be inspected locally without a drone:

```bash
python -m json.tool examples/course.synthetic.json
```

The [participant trajectory tool](trajectory_construction.md) builds and plots
ordered measured crossings and laps without a radio. Its exported course uses
this format; its polynomial reference is not automatically the runner's path.

## What instructors provide, and when

If offering optional powered work after Day 1 C7, agree a
[run card](glossary.md#run-card-and-configuration-freeze): a short record of the tested flight
interface, command units, actual controller/estimator, operator and stop path.
For Andreas's Day 1 position-loop experiment, it also identifies the qualified
velocity-command launcher/sequence, fixed-yaw/world-to-body mapping, transfer
between controllers with a continuous command at the switch
([bumpless handover](glossary.md#bumpless-handover)), step bounds and
returned state/command recording. That experiment is
not the position-setpoint course launcher below. Without a qualified Day 1
launcher, complete the hand check using the [firmware-stop preparation](sensor_recording_contract.md#prepare-the-drone-for-hand-held-recording)
or supplied-data replay and
record powered application as outstanding. There is no expectation to build a
new launcher for this event; simulation and replay cover the core.
Before Day 2 E2/E6, share one reusable [recording pack](sensor_recording_contract.md).
Before Day 3 R1, make the checked flight program available on a shared laptop (or
separately supplied tools), its exact launch command/version, the limits, race
rules and times. These are event-specific resources, **not files in the participant
ZIP**. No public reference implementation or survey answer is required.

For the course interface below, the supplied launcher exposes:

```text
python flight_runner.py --config TEAM_COURSE.json --dry-run --plot
python flight_runner.py --config CHECKED_COURSE.json --fly --log UNIQUE_RUN.csv
```

Run those commands in the instructor-provided tool directory/environment, not
the course package. The run card supplies that directory and launcher; if
using a module launcher it supplies the equivalent command with these arguments.
The first command does not connect to a radio. The team's pilot uses the second
after the preflight checks and second-member cross-check pass, with the stop
operator ready. The team then explicitly enables the checked course; the launcher
requests the exact course name as confirmation. Never bypass confirmation.
The operator exports the assigned URI into the environment variable named by
`uri_environment_variable`; merely setting a non-exported shell variable is
insufficient. Only one radio client controls a drone at a time.

**Before R2 acquisition**, try your candidate JSON with the shared dry-run launcher;
teams can help each other check its structure. Fix format mismatches before
surveying; successful validation does not qualify synthetic or measured geometry.
If the flight program, limits or run card are missing, continue with simulation
or recordings. Note what is missing and use a tested shared alternative
or supplied recordings. The starter's named profiles are configuration labels,
not implementations of this launcher.

## Course JSON format

The example contains every required field. All numeric values must be finite;
lengths are metres, times seconds, and vectors are three numbers in the same
right-handed Lighthouse world frame, with Z upward. Document origin, orientation,
geometry ID and the physical point used for drone position in the survey record.

| Field | Meaning |
| --- | --- |
| `course_name` | Nonempty course identifier, also used for interactive confirmation. |
| `enabled_for_flight` | JSON boolean; keep `false` until the team's preflight checks pass and the pilot and stop operator confirm readiness. Record that decision before enabling the course. |
| `uri_environment_variable` | Name of exported radio URI variable, normally `CF_URI`. No drone address is embedded in the example. |
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
documented and tested [adapter](glossary.md#adapter-and-runner), including rate, timing, limits and aborts;
selecting a profile string does not create one.

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
