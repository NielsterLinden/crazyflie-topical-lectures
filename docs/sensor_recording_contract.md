# Day 2: acquisition and replay contract

E2/E6 use sensor channels, reference conditions and timing metadata agreed before
acquisition. These extend the Day 1 Pose/Power/Motors logs with the observations
needed for calibration and inertial replay. We work through calibration together.
E6 uses a supplied live helper, with prepared replay as fallback; developing
another adapter is an optional extension with its own time allocation.
Prefer live measurements and reuse each capture across comparisons. The prepared
position/acceleration file also works directly with the
[E3-E5 experiment runner](../code/day2/README.md#e3-e5-experiment-runner).
Keep the checked recording pack available from the start; switch within five
minutes if unexpected live-data issues block the planned study. Retain raw data
and a short note for later diagnosis. Preparation checks channels, units, clocks
and reference conditions while preserving the observations and their provenance.
All acquisition is with motors disabled, using the worksheet's restraint/propeller
policy. No flight command is needed for this contract.

## Required handoff before acquisition

One shared recording pack supports the teams across the exercises. For the
[E2 core](sensor_bias_calibration_exercise.md#e2-core-45-minutes), provide a
60-second stationary IMU calibration recording and a separate 60-second
validation recording, including gyro and acceleration with their reference
conditions, units and existing corrections. Position is useful for E3's initial
noise assumption when available. The sensor-specific studies beyond this pair
are optional; they do not add acquisition or analysis requirements to E2.
Before E2, check its sample and metadata together: channels/firmware, world/body
frames, geometry, reference conditions and calibration/held-out split. Teams can
build the channel inventory from cfclient and share the work. Missing sensors or
references remain labelled limitations, not extra instructor data-production tasks.
Before E3, check the prepared files below with the supplied helper once and agree
which channels they cover. Do this before spending the acquisition slot. Never
substitute fused position for a raw Lighthouse observation or simulator truth.

The [synthetic CSV](../examples/sensor-events.synthetic.csv) demonstrates the
normalized row format only; it is not a calibration, motion, or validation dataset.

## cfclient channel inventory and logging blocks

Use the [Day 1 logging procedure](day_1_baseline_checks.md#b3-create-and-save-cfclient-logs)
to create these **additional**, separate blocks. Check every name/type in
**[Log TOC](glossary.md#telemetry-and-log-toc)**
for the actual firmware. The suggested periods specify logging frequency;
sensor-acquisition timing is recorded separately. Enable only the blocks needed for
the current experiment; retain original CSVs and configuration definitions.

| Block and suggested period | Common channels | Meaning and conversion for normalized replay |
| --- | --- | --- |
| [IMU](glossary.md#imu), 20 ms | `acc.x`, `acc.y`, `acc.z`, `gyro.x`, `gyro.y`, `gyro.z` | Six floats, 24 bytes. [Specific force](glossary.md#specific-force) in body axes: g to m/s2 using 9.80665. Gyro: degrees/s to rad/s. These channels may already include onboard bias compensation/filtering. |
| Orientation, 20 ms | `stateEstimate.qx`, `.qy`, `.qz`, `.qw` | Four floats, 16 bytes; unit [quaternion](glossary.md#quaternion-and-so3) ordered x,y,z,w. Derived onboard attitude, sharing inputs with other onboard estimates. Verify body-to-world convention against the hardware/frame note. |
| Position cross-check, 50 ms | `stateEstimate.x`, `.y`, `.z` | Three floats, 12 bytes; world metres, already fused. Store as `position_onboard`, never raw Lighthouse or truth. |
| Range, 50 ms | `range.zrange` | Downward sensor range, commonly mm; convert to metres. Record invalid-return rules from the driver/TOC and reference surface. Do not assume every number is valid or vertical height. |
| Pressure, 50 ms | `baro.pressure`, `baro.temp`, `baro.asl` where available | Pressure commonly hPa, temperature degrees C, and pressure-derived altitude metres. Convert pressure to Pa if storing the normalized `pressure` event; label altitude as derived. |
| Flow, 20 ms initially | `motion.deltaX`, `motion.deltaY`, `motion.squal`, `motion.motion`, optionally `motion.shutter` | Pixel/image increments and quality/status. Velocity conversion requires timing and geometry; see below. Preserve original values. Use stored types or verify converted block size stays below 26 bytes. |

Slash/dot shorthand in the table means separate complete variable names.
If a channel is missing, label it unavailable and choose a documented
supported subset or supplied dataset. Do not invent a log variable, silently fill
missing values with zero, or claim an absent sensor was calibrated.

**Flow timing caveat:** the normal log samples the latest flow registers. Logging
slower than the driver can skip increments; repeated snapshots are not new
measurements. The interval between log packets is not necessarily the integration
interval of that increment. The firmware's axis swap/sign convention and angular
motion also matter. Unless the supplied dataset/adapter preserves true increment
duration and provenance, use these logs for stationary offset/quality studies,
not quantitative velocity fusion. Flow fusion in E6 is optional when that contract
cannot be met; E5's simulated flow experiment remains available. See the
[Flow measurement model](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/functional-areas/sensor-to-control/state_estimators/#measurement-models).

**Lighthouse caveat:** `stateEstimate.*` is not a raw sweep-angle record. Raw
angle acquisition requires a specifically supported capture path and calibration/
geometry metadata, supplied by the instructor if used. The core replay may use
onboard position as an explicitly **[cascaded](glossary.md#cascaded-estimation)** measurement, with shared-IMU
correlations discussed; it is not independent raw Lighthouse/IMU fusion.

## Normalized event CSV and metadata

For the optional adapter extension, teams retain original cfclient files and implement a converter to
this interchange format (or an equivalent documented format with an explicit map):

```text
time_s,sensor,x,y,z,w,flow_dt_s,valid
```

One row is one new observation at that channel's measurement/log time. `valid`
is `1` or `0`; unused components are empty. Preserve rejected/invalid rows for
diagnostics, but do not feed invalid/non-finite data into the filter. Supported
event meanings are:

| `sensor` | Components and frame |
| --- | --- |
| `specific_force_body` | x,y,z in m/s2; gravity still present as specific force at rest. |
| `angular_rate_body` | x,y,z in rad/s, body frame. |
| `attitude_quat_onboard` | x,y,z,w unit quaternion, documented body-to-world convention; fused/derived. |
| `position_onboard` | x,y,z in world metres; fused/derived. Independent external position, if available, uses a separate documented event type. |
| `flow_pixels` | x,y image increments; `flow_dt_s` required for quantitative fusion, with source axes/quality in metadata or retained original columns. |
| `range` | x = sensor-to-surface distance in metres; not world Z. |
| `pressure` | x = pressure in Pa; optional for this starter, which has no barometer-update callback. |

A companion `recording.json` records dataset ID, source files/hashes, firmware,
decks, sensor availability, geometry/frame definitions, exact source-channel to
event mapping, units/conversions, timestamp kind and units per block, expected
rates, known [latency](glossary.md#latency-jitter-and-dropout)/integration times, invalid rules, existing onboard processing,
and any offboard corrections. Include reference-condition intervals, external
survey/reference uncertainties, calibration/development/held-out split and IDs.
Unknown timing/calibration facts are explicitly unknown, not estimated silently.
Keep raw observations, normalized data and calibration output separate.

Standard cfclient CSV `Timestamp` is the firmware log timestamp in milliseconds,
not host callback time and not necessarily the physical sensor-acquisition instant.
It wraps after 2^24 ms; unwrap within a continuous connection and split restarts
into new sessions. Convert to seconds relative to a common documented session
origin. Do not independently zero each block and destroy their offsets. Preserve
duplicates/gaps in raw files and define treatment in the converter. A reconnect
does not automatically prove a continuous clock. Independently timestamped external
reference data need a documented clock alignment and its uncertainty.

Prefer true acquisition times when available. If only firmware log times exist,
label them **logging-time proxies**, document unknown sensor age and limitations,
and use sufficiently slow motions or supplied acquisition-timestamped recordings.
Do not assert exact acquisition-time or delay compensation from these CSVs alone.

## Live E6 session and replay fallback

### Live capture using the supplied adapter

The [E6 instructions](kalman_filter_practical.md#e6-compare-filters-on-live-drone-data)
use `python -m day2.live` with the assigned URI and motors disabled.
The helper reads three separate 50 ms log blocks: acceleration plus onboard
position (six floats, 24 bytes), [quaternion](glossary.md#quaternion-and-so3)
(16 bytes), and motor outputs
(16 bytes). No gyro, range, flow or raw Lighthouse-angle fusion is claimed.
Close cfclient; only one client should use the radio connection.

`raw.csv` preserves received values, original firmware timestamps and host receive
times. The helper unwraps 24-bit firmware log milliseconds and buffers arrivals
for 150 ms to reduce callback reordering. It skips duplicates/late events with
a recorded count; it does not claim arbitrary-delay tolerance. Each accepted
acceleration/position packet uses the latest causal quaternion and zero-motor
snapshot, each at most 150 ms old. Stale dependencies or a sample gap over
300 ms after startup stop capture. Total telemetry loss also stops capture.

Body specific force in g, optionally minus a justified additional body-axis bias,
is rotated with the onboard body-to-world quaternion (x,y,z,w), multiplied by
9.80665 and combined with world gravity `[0,0,-9.80665]`. Quaternion norms are
checked and normalized. Current acceleration is a held-input approximation for
the preceding interval; packet timestamps are logging-time proxies, not exact
independent sensor times. No hidden interpolation or sensor-delay compensation
is performed. Verify stationary acceleration at level and tilt before accepting
data. Saved `prepared.csv` and `recording.json` use the replay contract below;
both configurations therefore reproduce the live numerical results offline.
Metadata includes input hashes, channels, conversion/correction, skipped-event
count and completion/failure status. Partial data survive ordinary errors or
Ctrl+C; an abrupt process/power loss may leave incomplete metadata.

The default extra accelerometer correction is zero. E2 values are not silently
loaded; `--acc-bias-g` requires a separately justified correction in g. Do not
subtract existing onboard compensation twice. Neither mode writes parameters,
selects an estimator/controller, arms or commands motors. Motor-output monitoring
does not replace disabled motors and removed propellers or a secure restraint
preventing propeller contact.

### Prepared-data fallback

Before E3, have `data/e6-prepared.csv` and `data/e6-recording.json`
relative to the package root, plus original data or stable hashed locations and
a preparation/calibration map. Include labelled rest and slow motion (and a
position gap if available), using real E2/instructor data. Preparation happens
before the slot: frame/gravity conversion, clock normalization/alignment and any
resampling are **not** participant core tasks. Test the helper on those exact
files and retain known-good outputs. A 30-second rest/slow-motion recording can
serve E3/E4/E5 as well as E6. Label its position source explicitly.

The helper's CSV columns are:

```text
time_s,ax_world_m_s2,ay_world_m_s2,az_world_m_s2,px_m,py_m,pz_m
```

Each row supplies world-frame acceleration, gravity already removed, for the
interval **ending** at `time_s`, and an optional position observation at that
time. The first row establishes time; no propagation precedes it. Times must be
finite and strictly increasing in one documented clock. All three position fields
are present together or all blank; blanks mean no update, not zero position.
All other numeric inputs must be finite. Invalid/reordered/wrapped original data
require documented preparation, not silent acceptance. This restricted format
is intentionally different from the asynchronous raw-event format.

For E3-E5 comparisons, the first available position initializes the estimate
once, with zero initial velocity and the configured prior variance. Leading
rows without position are omitted from analysis; their count and initialization
time are recorded in metadata. The original file and time origin are preserved.
At least one position observation is needed to anchor these comparisons.

Metadata retains dataset identity, source hashes, channel/calibration map,
frame/time/shared-information caveats, preparation/resampling method and labelled
rest/motion/gap intervals. It additionally contains this object (within a JSON document):

```json
{
  "prepared_replay": {
    "acceleration_frame": "world",
    "gravity_removed": true,
    "acceleration_interval": "preceding",
    "position_source": "onboard"
  }
}
```

`position_source` is `onboard` for fused positions or `external` for a documented
independent observation, not automatically noise-free truth. Onboard attitude/
position makes the replay cascaded. The helper uses the existing six-state filter;
it neither calibrates raw sensors nor fuses raw flow/range/Lighthouse angles.
Its uncertainty bands are model predictions, not validated accuracy claims.
Core deliverables are two replay output sets and interpretation, not converter
code or timing-robustness tests.

For an installation/format smoke test only, run from an activated environment:

```text
cd code
python -m day2.replay ../examples/replay.synthetic.csv --metadata ../examples/replay.synthetic.json --output ../E6/format-check
cd ..
```

These tiny synthetic values check the file format. For a complete controlled
motion comparison without hardware or shared files, run from the package root:

```text
cd code
python -m day2.experiments --study single --export-prepared --output ../E6/cooked
python -m day2.replay ../E6/cooked/prepared.csv --metadata ../E6/cooked/recording.json --output ../E6/cooked-baseline
python -m day2.replay ../E6/cooked/prepared.csv --metadata ../E6/cooked/recording.json --position-std 0.12 --output ../E6/cooked-comparison
cd ..
```

The exported metadata identifies these data as synthetic. This fallback supports
the filtering concepts, with real-data calibration/validation left outstanding.
It does not turn simulated results into evidence about the team's hardware.

## Minimum E6 replay and callback mapping (optional adapter extension)

A supported raw-input adapter recording contains body specific force, compatible onboard
orientation and onboard position over rest and slow hand motion, plus metadata.
Gyro is required for the E2 IMU calibration study; range/flow/pressure studies use
their actual installed sensors or supplied data. E6 may omit unsupported flow,
raw Lighthouse or barometer fusion and must identify that limitation.

| Pipeline entry | Adapter responsibility |
| --- | --- |
| `on_imu(time_s, acceleration_world)` | Apply justified calibration, rotate specific force with compatible orientation, then add world gravity `[0,0,-9.80665]` to obtain translational acceleration. Do not subtract body Z alone while tilted. |
| `on_position(position_m)` | Supply explicitly labelled world position; using `position_onboard` makes this cascaded estimation. |
| `on_flow_velocity(velocity_xy_m_s)` | Only after a justified pixel-duration/height/attitude/gyro conversion to world velocity. Otherwise omit this update. |
| `on_range(height_m)` | Only after converting range to world Z using known surface elevation, sensor offset and attitude assumptions; otherwise omit or extend the model. |
| `on_stationary()` | Conditional [ZUPT](glossary.md#zupt) from validated stationarity logic; never inject continuously during motion. |

`pipeline.py` supplies the update callbacks. The adapter schedules them: before
each update, propagate to its timestamp with the appropriate held process input;
do not double-propagate when the next IMU sample arrives. Define an ordering for
equal timestamps and buffer/reject late observations explicitly. The adapter
extension tests should cover duplicate/out-of-order times, missing blocks, invalid returns,
clock wrap and rest-to-motion transitions without sending any motor commands.

**Additional extension deliverables:** converter/replay command, original and normalized data,
`recording.json`, calibration reference, callback mapping/timing tests, state and
sensor-specific innovation plots, and a statement of raw/derived dependencies
and unidentifiable quantities. Replay never qualifies a hardware controller.
