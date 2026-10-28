# Measure the gates with the supplied survey tool

We use the instructor-provided `scripts/measure_hoop.py` for R2. There is no need
to design a survey method or write fitting code. The exercise is to collect a
useful measurement, check the geometry and understand what could bias it.

Allow 40 minutes for a shared survey of the course, with 20 minutes reserved for
retries or hardware problems. Teams can use the same survey files and share
collection/checking roles; a separate full survey by every team is not required.
The number of gates and available survey stations determine what fits. If it
takes longer, use checked fallback geometry or reduce the course for everyone,
not the safety checks. Agree the final shared course before qualification.

## Assignment

1. Confirm the Lighthouse frame and stable position estimate. Keep motors
   disabled, with propellers removed or a secure restraint preventing propeller
   contact. Close cfclient
   before connecting the script; only one radio client uses the drone at a time.
2. Use a pointer/jig referenced to the drone's estimated position. Account for
   any offset: pressing a deck or cage against the rim can shift the fitted gate.
   The script does not correct that offset. With fixed attitude, a known constant
   world-frame tip offset is added to the fitted centre when building the course;
   keep the original JSON and record the correction. Changing attitude makes this
   a sample-by-sample correction, outside this exercise. Avoid covering receivers.
3. Run the appropriate command below on the laptop. At
   `Command [Enter=record, q=quit]:`, press **Enter on the laptop keyboard** to
   start recording the trace. Move the pointer around the complete inner opening,
   then press **Enter in the same terminal** to stop. Confirm `Fit this trace?`
   to fit the recorded points. One team member can operate the laptop while
   another moves the drone, keeping its attitude steady and receivers visible.
   Trace all four edges/corners for a rectangle. Select the
   [normal](glossary.md#clearance-and-gate-normal) sign for the
   intended crossing direction; reversing it does not move the physical plane.
4. Check the centre, normal and aperture against independent manual measurements
   or a second pass not used in the fit. Examine the printed residuals, but do not
   treat a small residual as proof of absolute accuracy. Repeat poor captures.
5. Keep the JSON and check results. Use the geometry in the course configuration
   and plot the planned path for the team's flight checks.

## Commands

From the project root with the native environment active, set the assigned
`CF_URI` as described in [software setup](software_setup.md#team-radio-configuration).
If needed, add `--uri radio://...` with that exact assigned address. The script
refuses to connect when neither is set; it does not pick a default drone.
These single-line commands work in Bash and PowerShell:

```text
python scripts/measure_hoop.py --shape circle --name hoop-1 --output survey/hoop-1.json
python scripts/measure_hoop.py --shape rectangle --name gate-2 --output survey/gate-2.json
```

Use a different name/output for each physical gate. Do not run both commands for
the same gate unless comparing fits. Existing files are protected unless `--force`
is supplied. Default sample spacing is 0.02 m; `--sample-spacing 0.01` retains a
denser trace. The trace starts and stops at the laptop's Enter prompts; turning
the drone upside down and back does not trigger recording. Move slowly and watch
the physical drone. The script logs position and sends no arming or flight commands;
it is not a motor interlock. Physical restraint and the agreed handling procedure
still apply. No custom firmware or Docker is needed.

## Constraints and interface

The [course format and flight-tool guide](course_interface.md) describes the full course file.
A survey JSON describes one physical gate. The team builds the course, checks
the shared dry-run output and completes its preflight checks before flying.

- All positions are metres in the same right-handed Lighthouse frame as the
  flight estimator, with Z upward. Record what physical point is being measured.
- Circle output contains `name`, `centre`, unit `normal`, `radius_m`, and a
  `survey` object with sampled `points_m`, sample count and plane/radial residuals.
- Rectangle output adds `shape`, `width_axis`, `height_axis`, `width_m`, `height_m`
  and outline residuals. Its `radius_m` is half the smaller dimension: the circular
  runner's conservative inscribed aperture. Keep the full rectangle output for
  plotting; width or diagonal must not be used as the radius.
- Copy the runner's gate fields into the course's `hoops` list, assigning each
  required crossing a unique name and normal direction. Preserve a separate map
  from repeated crossings to the physical survey file. Keep
  `enabled_for_flight=false` until qualification.
- Budget separately for drone size, gate thickness/supports, raised floor,
  survey/estimator/tracking error and safety margin. Fit residuals alone are not
  that error budget. A horizontal hoop needs its measured orientation too.
- A moved gate or changed Lighthouse frame invalidates the survey; recheck it
  before flight. Never use unqualified autonomous flight to locate an obstacle.

## Deliverables

Keep the survey JSONs, their independent check, the course configuration and one
geometry/path plot. The JSON already stores points and fit residuals; there is
no separate survey report or new reconstruction code to submit. Note the shared
data source and any pointer offset or unresolved error in the team notes.

**Acceptance check:** explain the centre, normal sign, aperture and main
uncertainty, and show how you checked them. A second team member cross-checks
the physical setup and margins before a powered crossing. Developing your own
survey/fitter is an optional extension for further exploration.

## Time limit and fallback

If collection or validation overruns, keep the attempted measurements and switch
to an already checked shared survey or instructor fallback. Label it as supplied
geometry and retain its source and validation record. Use the same agreed course for all
timed competitors. If no physical geometry is qualified, work with recordings or
simulation instead. Never relax clearance checks to meet the timetable.
