# Hardware checks and cfclient reference

This is the shared reference for hardware readiness, battery limits and
cfclient diagnostics across the three days. The [programme](day_1_control.md#schedule)
gives the timetable; the [Day 1 notebooks](../code/day1/README.md) contain the
working experiments and flight sequence.

Use B1, B2 and B6 to check the fitted drone and intended flight configuration,
and B5's battery checks before every flight. Notebook measurements can supply
the evidence for B3-B4; the cfclient procedures below offer another way to
inspect and record it. B5's cfclient hover is an alternative when the notebook
flight is unavailable.

These checks establish a measured baseline for later tuning experiments.
The group members work through the checklist together, cross-check each
other's measurements and agree that a stage is complete before continuing.
Record unresolved checks and the group's decision; use the
[shared team responsibilities](practical_workflow.md#responsibility-for-the-experiment).
Start with the supplied baseline configuration and preserve it for comparison.
Check and document corrections to identified setup errors; do not optimize gains,
thrust scaling, estimator noise settings, or safety thresholds during these checks.
Detailed sensor-bias/noise calibration belongs to Day 2.
Its [recording guide](sensor_recording_contract.md) adds the raw/derived sensor
channels and timing metadata not covered by this Day 1 baseline log.

The [glossary](glossary.md) explains control and telemetry terms used below. Without
working hardware or passing flight checks, use instructor-supplied recordings and label the
source and outstanding live checks. A replay does not qualify a drone to fly.

## B1: physical configuration and mass (unpowered)

Disconnect USB and the battery before handling propellers or adjusting hardware.
Check free/undamaged propellers, secure battery/decks/cage, propeller-to-cage
clearance, and approximately centred payload placement. A visual balance check
is not an inertia or centre-of-mass calibration. Do not diagnose imbalance by
running motors in your hand or while holding the drone down.

Weigh and document the complete configuration using the
[mass and cfclient Set/Store procedure](practical_workflow.md#hardware-preflight-mass-and-controller-assumptions).
Reconnect only after mechanical inspection; leave motors stopped for B2-B4.

**Record:** drone/battery IDs, configuration/photo, measured g/kg, scale resolution,
mass-setting applicability, original/current/stored settings, and reboot readback.

## B2: cfclient connection, firmware and controller identity

1. Launch `cfclient` from the activated native environment. Select the assigned
   radio address, click **Scan**, select the assigned URI, then **Connect**.
   Match the connected drone to the team record; do not use a neighbour's URI.
2. Enable **Console**, **Parameters**, **[Log TOC](glossary.md#telemetry-and-log-toc)**, **Log Blocks**, **Plotter**,
   **Flight Control**, and **Lighthouse Positioning** as needed through
   **View > Tabs**. Names/capitalization may differ slightly between releases.
3. Save the firmware build/version and deck/self-test messages from **Console**.
   Record cfclient/cflib versions and radio identity. Missing decks, failed tests,
   estimator-reset loops, or stale telemetry require investigation before flight.
4. In **Parameters**, read `stabilizer.controller` and `stabilizer.estimator`.
   PID is controller `1`, Mellinger is `2`; the Kalman estimator is estimator `2`.
   Record current values, applicable controller gains/mass settings, and any
   saved settings rather than changing them to match another team's drone.
   Check the intended script for selection overrides and its in-run readbacks.
5. For Lighthouse flights, in **Lighthouse Positioning**, verify the shared geometry/calibration
   is present and the expected base stations are visible. Record the geometry
   version and frame convention; do not move stations or overwrite geometry as
   part of this baseline. For the Flow-deck experiments, use the notebook's
   position/height checks and local-frame convention. Gate-location surveying
   remains the Day 3 exercise.

**Record:** console extract, URI/drone identity, firmware/client versions, deck
inventory, parameter snapshot, geometry ID, and any mismatch or warning.

## B3: create and save cfclient logs

Use the notebook's saved data when it covers the measurements needed below.
For additional cfclient diagnostics, create the relevant logging blocks as follows.

For stationary and hand-held measurements, place the drone on the surface and
connect cfclient. In **Parameters > stabilizer > stop**, enter `1`, press Enter,
check the displayed value and visually check stationary propellers. Keep this
stop engaged during B3-B4 and follow the [hand-held recording preparation](sensor_recording_contract.md#prepare-the-drone-for-hand-held-recording).
The same log configurations can later be used for flight after the power cycle
and preflight checks below.

1. In **Log TOC**, find the channels below and inspect their units/types. Missing
   channels are recorded as unavailable, not replaced with zeros. Investigate
   firmware-specific alternatives together before powered work.
2. Open **Settings > Logging configurations**, choose **Create config**, name
   each block, select its variables and move them with the add arrow, choose a
   **Logging period** of **50 ms**, then **Save**. Use separate blocks:

| Block | Variables | Purpose |
| --- | --- | --- |
| Pose | `stateEstimate.x/y/z`, `stabilizer.roll/pitch/yaw` | Estimated position and attitude; six floats, 24 bytes. |
| Velocity | `stateEstimate.vx/vy/vz` | Estimated velocity; three floats, 12 bytes. |
| Power | `pm.vbat`, `stabilizer.thrust`, `ctrltarget.z`; optionally `ctrlMel.cmd_thrust` | Battery voltage, raw controller thrust, and onboard target altitude; at most four floats, 16 bytes. |
| Motors | `motor.m1/m2/m3/m4` | Individual raw motor commands; at most 16 bytes with 32-bit values. |

The slash shorthand means separate variables, for example `stateEstimate.x`,
`stateEstimate.y`, and `stateEstimate.z`. Each block must fit the 26-byte payload;
do not combine all channels into one block. The optional Mellinger debug channel
is meaningful only for that controller, not a generic thrust measurement.

3. In **Log Blocks**, enable **Start** and **Write to file** for the relevant
   blocks. In **Plotter**, choose a block to inspect its traces. Start with Pose
   for unpowered checks; enable other blocks when needed. Avoid unnecessary
   subscriptions that burden the radio, and do not run another radio script
   concurrently with cfclient on the same drone.
4. Record at least 10 s of stationary data on a level, undisturbed surface.
   Note the interval and any warnings. Then stop file writing and the blocks.
   **Settings > Open config folder** opens cfclient's configuration directory;
   find the CSV files under `logdata/` in the connection's timestamped folder.
   Copy those files and the logging configuration into the team evidence folder.

**Record:** original CSVs, block/channel definitions and requested periods,
stationary means/spreads and drift, and timestamps of warnings. Label onboard
state estimates as estimates, not raw sensor data or independent truth.

## B4: coordinates, attitude and timing (motors stopped)

- With the drone physically level and stationary, inspect roll/pitch, XYZ,
  estimated velocity and yaw over the recorded interval. Report variability,
  apparent drift and any continuing oscillation; avoid inferring a sensor bias
  from one fused-state trace alone.
- With `stabilizer.stop=1` set and checked as above, use a ruler or marked positions
  to move the drone by a known displacement
  along each world axis, returning to a stationary hold at each endpoint. Record
  at least 5 s per endpoint, expected sign/distance, and uncertainty. Compare
  endpoint means with the independent displacement, including vertical motion.
  Keep the same physical reference point and attitude and avoid obscuring the
  Lighthouse sensors. This checks scale/frame consistency, not a full calibration.
- Turn the drone by a known, modest angle on the surface and check yaw changes
  consistently with the frame convention. Treat +180/-180 degree wrapping as
  one continuous angle, not a physical jump.
- For each CSV block, compute timestamp intervals, actual mean rate, maximum
  gap and interval spread; compare with the requested 50 ms. State the timestamp
  units and account for wrap/restarts. A gap is evidence of missing/late samples,
  not by itself proof of radio packet loss. Different blocks are asynchronous.
- cfclient CSV timestamps are drone timestamps: they do not measure end-to-end
  [latency or host command jitter](glossary.md#latency-jitter-and-dropout).
  Record those quantities as **not measured**
  unless the chosen runner captures appropriate host command/arrival timestamps.
  Do not subtract unsynchronized drone and laptop clocks. Scripted-flight checks
  should include logged command rate/jitter, stale-data and deadline handling.

**Record:** stationary summaries, displacement/yaw comparison with uncertainties,
time-series plots, per-block timing statistics and explicitly unmeasured quantities.

## B5: battery and first supervised hover

### Battery checks for every flight

For the Crazyflie 2.1-class drones with a standard **single-cell (1S) LiPo**,
start with a fully charged, checked pack. Inspect the pack and leads: a swollen,
punctured, leaking, damaged or unusually hot pack stays out of use; hand it to
the organizer for safe handling. With USB disconnected and motors stopped for
at least 60 s, read `pm.vbat` in the notebook's status cell or cfclient before
takeoff. Close cfclient before connecting from the notebook.

The flight types use different limits:

| Flight | Before takeoff | During flight |
| --- | --- | --- |
| Day 1 notebook experiments and cfclient baseline hover | The notebook specifies **3.5 V at rest** as the lower limit; below this, recharge or use another checked pack. Use a full battery for tuning comparisons. | The notebook control loop lands **below 3.1 V under load**. For the cfclient hover alternative, the operator monitors voltage and lands at that threshold. |
| Race | Start at **4.10 V or above**; below this, recharge or use another checked pack. | The operator monitors voltage and begins a controlled landing at **3.50 V or below under load**, or at the first low-battery warning. |

The Day 1 values follow `drone_04_identification.ipynb` and its existing helper.
Its automatic voltage check runs in the control loop after takeoff and settling;
power telemetry is recorded for later analysis. Follow the notebook's abort
instructions for weak lift, unusual sag, excessive drift/tilt or unstable flight.
The race uses a larger battery reserve; its tools need a checked live display
or enforcement path for those stricter limits. These thresholds describe the
respective experiments, rather than manufacturer discharge limits or estimates
of remaining flight time. Check the fitted drone/battery/cage during rehearsal.
Passing the resting-voltage check still requires a satisfactory first hover.

After landing, save resting/loaded voltage and battery ID. Recharge before
another flight if its pre-takeoff check fails. Let a warm pack cool to room
temperature before charging.

For charging, connect the battery to the Crazyflie and use its onboard USB
charger. On the surface, connect cfclient and set/check **Parameters > stabilizer > stop = 1**;
leave it engaged and keep propellers clear. The rear-left blue
LED blinks while charging and becomes steady when charging completes. Then
disconnect USB, wait 60 s with motors stopped and check `pm.vbat` again. Use
the designated nonflammable charging surface, keep packs clear of loose metal
and combustible materials, and supervise charging throughout. The charging
indication follows Bitcraze's [Crazyflie 2.x guide](https://www.bitcraze.io/documentation/tutorials/getting-started-with-crazyflie-2-x/);
see also its [charger safety instructions](https://www.bitcraze.io/battery-charger-500mA/).

Record battery ID, charge-complete indication, resting voltage and loaded
voltage under comparable conditions. For other battery types or platforms,
establish compatible charging and flight limits before powered work.

### First hover

The notebook gives the normal flight sequence. For the cfclient baseline-hover
alternative during the [flight session](day_1_control.md#flight-session),
complete the physical, configuration, battery and B6 checks using the notebook
measurements or the cfclient diagnostics above:

1. Clear the flight volume and have the named emergency-stop operator ready.
   After the firmware-stop measurements/charging, place the drone down and
   power-cycle it to clear the lock. Reconnect cfclient and repeat the preflight
   checks; setting `stabilizer.stop` back to `0` does not unlock it.
   Start recording Pose, Velocity, Power and Motors.
2. Use **Flight Control > Command Based Flight > Take off** to the altitude
   agreed for the flight area.
3. Once at the target altitude, hover for **10 seconds**, observing position,
   attitude and battery voltage. Keep the reference fixed during this check.
4. Select **Land**, wait for the motors to stop, then stop recording.

Land earlier if a battery or B6 limit is reached, or motion becomes unstable.
If the checks fail or the hover cannot be completed safely, use supplied logs
for the analysis. Record whether the hover was completed, aborted or not
attempted, what prevented completion and which checks remain outstanding.

Compare the actual position/attitude trace, target altitude, voltage before and
during hover, thrust command and motor-command spread. Report hover error,
drift/oscillation, voltage sag, sustained command level, and possible saturation
or imbalance. Raw thrust/[PWM](glossary.md#actuator) percentage is **not** measured force, RPM, remaining
lift percentage, or a direct estimate of payload mass. Motor asymmetry can come
from normal feedback as well as balance/propulsion problems. Compare motor
commands with attitude and battery voltage in the logs. With USB and battery
disconnected, check propeller condition, cage clearance and payload balance.
Keep motor settings unchanged during these baseline checks. Record any unresolved
cause and use another checked drone or supplied logs if the problem prevents a
stable hover.

Begin landing for failed lift, excessive drift/tilt, position jumps, low battery
or missing telemetry; use the emergency stop if continued motor operation risks
contact. An emergency motor stop can cause a fall. No repeated floor-skating
takeoffs, increased thrust scaling, or relaxed limits to force a pass.

**Record:** hover outcome and any reason for stopping or not attempting it,
operators, conditions, resting/loaded voltage and sag, hover metrics,
motor/thrust plots and raw logs. Label supplied data and outstanding checks.

## B6: limits, decision and unchanged baseline

Before flight, record the shared flight volume, altitude, speed, tilt/tracking limits,
telemetry-age/command-deadline thresholds, stop procedure and fallback. Read these
from the intended runner/configuration, cross-check within your group and agree
on their meaning before proceeding; they are not all
cfclient parameters. Identify whether a limit monitors actual motion, reference
commands, or only simulation. Review the emergency-stop control without activating
motors. For the cfclient first hover, identify the operator's abort criteria and
active onboard protections; do not claim the Python race runner's checks are active.
Validate fault handling in software, never by deliberately creating an unsafe live fault.
Emergency motor stop can cause a fall.

Make a baseline table with columns:
`check_id, observation, units, uncertainty_or_resolution, agreed_acceptance,
status, evidence_path, checked_by, cross_checked_by, outstanding_action`.
Use **pass**, **fail**, **not measured**, or **not applicable**. Record acceptance
criteria before looking at results, using the shared course limits and documented
hardware constraints. Record unresolved or unspecified limits and settle them
before attempting flight. A failed check means diagnosis, tested fallback or no
flight, not a more permissive threshold. Record original settings, each checked
correction and the new verified baseline. The pilot and stop operator confirm
that all flight-critical checks pass before takeoff, following the whole group's
agreement to proceed.

Keep one preflight record with original logs/settings and B4/B5 plots. Link
existing notebook measurements and the record from your team notes.
Without hardware, use shared recordings and list the live checks
still needed; the instructor-supplied recordings can be reused across teams.

## Optimization is a separate experiment

For tuning, follow the notebook experiment and compare matched conditions.
Start in simulation, preserve the unchanged baseline, change one identified
factor at a time, and retain failures and held-out validation. Simulator gains
are not interchangeable with onboard PID parameters. Hardware optimization needs
separate simulation, team cross-checks and low-speed testing; it is a separate experiment.

Keep the baseline onboard attitude gains unchanged. Thrust-conversion calibration,
actuator identification and cage-induced inertia effects are optional extensions,
not settings to guess until takeoff works. Keep these distinct from the physical
mass measurement and Day 2 sensor calibration.
