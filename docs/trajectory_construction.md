# Construct and preview a race trajectory

The supplied planner supports course construction and preview in Day 3 R2/R3.
For those exercises, follow the example commands, measured-gate workflow and
model/flight checks below. The section **Stretch goal: continuous and limited
jerk** is an optional development project, with extra time for implementation
and validation. The baseline's limitations still inform the course checks.
It needs only the native simulation environment: no radio, Docker or drone.
Commands below run from the package root in bash or PowerShell after activating
the environment described in the [setup guide](software_setup.md).

## Try it without hardware

```bash
python scripts/plan_trajectory.py --course examples/course.synthetic.json --output plans/first --show
python scripts/plan_trajectory.py --course examples/course.shuttle.synthetic.json --laps 2 --output plans/two-laps --show
```

The second example crosses one opening forward and back on each of two laps:
four crossings in total. These are synthetic examples, not measured flight courses.
Omit `--show` on a machine without a display. The saved preview still works.
Choose a new output directory for each comparison; existing results are not overwritten.

Each output directory contains:

- `course.json`: runner-format geometry, ordered uniquely named crossings, and
  **flight disabled**, even when the input file had flight enabled.
- `preview.png`: planned reference in 3D and XY/XZ/YZ projections, opening
  outlines, crossing arrows, start/finish and speed/acceleration histories.
- `trajectory.csv`: `time_s`, XYZ position in metres, velocity in m/s and
  acceleration in m/s2. All vectors use the Lighthouse world frame.
- `summary.json`: planned duration, conservative speed/acceleration bounds,
  each crossing's reference time, and input file paths.

## Use measured gates

1. Collect and independently check gate JSONs using the [survey tool](hoop_survey_challenge.md).
2. Copy a synthetic course to a team template and edit its name, airborne start
   and finish, and agreed bounds/limits. Choose these for the actual takeoff
   location and qualified room volume. Keep the survey files alongside it.
3. Order the surveyed gates with repeated `--crossing NAME:+` or `NAME:-` options.
   `NAME` is the survey's `name` field; `+` follows its normal and `-` reverses it.
   `--laps` repeats the whole ordered crossing sequence.

For a survey named `gate-1`, forward and return crossings on two laps:

```bash
python scripts/plan_trajectory.py --course team-template.json --gate-survey survey/gate-1.json --crossing gate-1:+ --crossing gate-1:- --laps 2 --name team-two-laps --output plans/measured --show
```

Repeat `--gate-survey` for several openings and `--crossing` for their intended
order. Without survey options, the planner uses the template's existing `hoops`
sequence. Every generated crossing receives a lap/event prefix for identification.
For an out-and-back course, put the finish on the start side of the gate; otherwise
reaching it may require an additional unlisted crossing. Reversing direction at a
shared approach/exit point brings the reference velocity to zero there.

Rectangle outlines retain measured width, height and axes, including horizontal
gates. Planning uses the conservative inscribed-circle aperture expected by the
[course interface](course_interface.md). Pointer/jig offsets are not corrected
automatically: apply and record the correction described in the survey guide first.
The tool reads its geometry from JSON files. Edit gate
geometry or endpoints in JSON and rerun to inspect a different layout.

## Model and checks

Each gate adds an approach point, centre and exit point along its normal.
Piecewise [quintic polynomials](glossary.md#trajectory-continuity) join them with continuous position, velocity and
acceleration; acceleration is zero at each join and endpoint speed is zero.
Gate-centre velocity follows the requested normal. [Jerk](glossary.md#jerk-and-snap) can jump at joins;
the optional extension below explores higher-order continuity.

The implementation uses Bernstein polynomials. Their control-point convex hulls
bound the entire reference and its derivatives between plotted samples as well.
The planner stretches time uniformly until the conservative speed, acceleration
and nominal tilt bounds are met. Consequently `target_speed_m_s` sets the initial
timing, which can be stretched to satisfy those bounds. The resulting duration
can be conservative. Bounds apply to the reference position;
cage/swept-volume [clearance](glossary.md#clearance-and-gate-normal)
to walls and supports still needs a separate check.

The planner rejects insufficient circular aperture margin and out-of-box control
hulls. Each gate occupies a **finite footprint** in its plane. At each plane
intersection:

- Passage through the clear opening must be a listed crossing, in order and in
  the specified direction.
- The opening's boundary/frame exclusion band cannot be crossed.
- Passing outside the finite gate footprint plus clearance is allowed and does
  not count as a gate passage. Separate openings can share a plane.

Each gate can specify `frame_margin_m`: physical extent beyond the measured
inner opening, in metres **per side**. The default is **0.10 m**, an assumption
to review against the actual frame and supports, not a measured dimension or a
guarantee of safety. It extends a circle radially or a rectangle on all four
sides. Use enough padding to enclose projecting supports; irregular obstacles
not covered by that footprint still need separate checks. For example:

```json
"frame_margin_m": 0.15
```

Keep this field in each course/survey gate, or use `--frame-margin 0.15` to
override all gates when planning. The chosen/default value is written into the
exported course for every crossing. Bypass clearance additionally includes
`drone_radius_m + clearance_margin_m`. Thus a 0.50 m opening radius, 0.15 m frame
padding, 0.12 m drone radius and 0.08 m clearance require a bypass radius **greater
than 0.85 m**. The orange dashed outline marks that outer bypass boundary; the
grey outline marks the opening. For a rectangle the outer envelope is a
conservative expanded box in the gate's measured axes.

Segments lying along a gate plane are allowed only when their entire curve can
be shown to stay outside the finite padded footprint. Ambiguous contact fails
conservatively. These are plane-intersection checks, not a complete swept-volume
collision model: gate thickness, nearby parallel passes, raised floors, supports
outside the padded footprint, Lighthouse visibility, tracking error, sensor
uncertainty and actuator response still need qualification.

## Limitations of the baseline

The baseline makes the following modelling and planning choices:

| Current choice | Consequence |
| --- | --- |
| Quintic segments with zero acceleration at every join | Position, velocity and acceleration are continuous (C2), but jerk can jump. The reference may demand abrupt changes in thrust rate and attitude rate. |
| Fixed centre crossings and approach/exit offsets | No racing-line optimization; a shared turnaround point forces a stop. |
| Uniform time stretching using convex-hull bounds | Simple, conservative timing; no local speed optimization or explicit jerk limit. |
| Geometric reference and nominal tilt bound | No closed-loop tracking prediction, motor/thrust or body-rate limits, battery model, or race-time distribution. |

The finite-footprint and flight-interface limitations above still apply.
Smoothing the reference may help tracking, but does not by itself establish a
faster or safer physical flight.

## Stretch goal: continuous and limited jerk

If the baseline is working and time remains, explore this offline; it is **not
required for the race** and does not need an extra report. Keep the supplied
planner and qualified flight fallback unchanged. There is no extra scheduled
slot: stop before this displaces qualification or crosses the configuration freeze.

Jerk is the third derivative of position:

\[
j(t)=\frac{d^3r(t)}{dt^3}.
\]

The aim is **continuous, bounded jerk**, allowing acceleration to change smoothly
throughout the flight. The planner specifies this smoothness explicitly; the
controller then tracks the resulting reference. Actuator lag can also smooth
motion, accompanied by tracking error. The current position-setpoint interface
sends position and yaw; using the planner's velocity/acceleration arrays as
feed-forward commands requires a compatible flight interface.

One approachable experiment:

1. Inspect `build_trajectory` in `code/trajectory_tools/planner.py`. Plot jerk
   within each segment, and evaluate its left/right limits at every join.
   Ordinary sampled plots or finite differences alone can miss a jump.
2. Replace the quintic construction with degree-seven (septic) segments. Each
   axis then has eight coefficients, enough to specify position, velocity,
   acceleration **and jerk at both ends**. Share those derivatives at every join.
   Setting acceleration and jerk to zero at each knot gives a simple first
   version. Allowing nonzero shared derivatives offers further timing choices.
3. SciPy's existing `BPoly.from_derivatives` can build this reference: provide
   `[position, velocity, acceleration, jerk]` at each knot, in physical-time units.
   Retain the approach/centre/exit geometry and crossing directions initially,
   so the comparison changes smoothness rather than everything at once.
4. Recheck endpoints, left/right continuity through the third derivative,
   whole-path bounds, crossing order, finite frame clearance, speed,
   acceleration and nominal tilt. Add an explicit jerk bound in m/s3 and a test
   that checks it over the whole curve, for example using derivative control hulls.
   Stretching duration by a factor `s` reduces jerk by `s**3` for a fixed curve.
5. Compare the baseline and extension under the same course and existing limits:
   plot acceleration/jerk, record join jumps, peak jerk and planned duration.
   Do not claim timing gain merely by relaxing a limit. If a suitable closed-loop
   simulator is available, also compare tracking and saturation; otherwise leave
   that benefit as an untested hypothesis. Link the comparison in the existing notes.

A further extension is a joint minimum-snap fit, minimizing
\(\int \|d^4r/dt^4\|^2\,dt\) with waypoint and derivative-continuity constraints,
rather than setting all knot accelerations/jerks to zero. Neither a septic fit
nor minimum-snap optimization automatically respects actuator limits or supplies
a qualified flight adapter. Any hardware use needs the same separate review as
other changes to the racing reference.

## From preview to flight

The preview shows **the planned geometric reference**. Its duration covers the
reference between airborne endpoints. Predicting flown motion and race-time
spread requires a closed-loop simulation; actual timing comes from flight
measurements.
Keep actual geometry, source surveys and any corrections with the saved output;
the summary's file paths are not a substitute for retaining those files.

The existing qualified runner may generate a different spline from `course.json`.
The CSV supports offline analysis. Executing it requires an adapter to the
chosen flight interface. Compare the runner's own dry-run plot and limits
before qualification. Flying this exact polynomial would need a separately
reviewed runner adapter. Do not turn on flight solely because this preview passed.

For the practical, saving this preview and checking it together is enough;
no additional report or planner implementation is expected. Reuse the approved
fallback when time or hardware prevents qualifying a candidate.

Offline tests, from `code`:

```bash
python -m unittest trajectory_tools.test_planner
```
