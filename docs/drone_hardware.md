# Meet the Crazyflie: hardware, sensors and positioning

During the Day 1 introduction, find these components on your drone with power
disconnected, before the [cfclient checks](day_1_baseline_checks.md). We use
Crazyflie 2.1-class drones, but boards and decks may differ. Note what is fitted
and confirm the sensors/decks in the cfclient Console at startup.

## The flying platform

| Component | Role in the experiment |
| --- | --- |
| Four motors and propellers | Produce collective thrust and roll/pitch/yaw torques. Tilting redirects thrust to accelerate horizontally. |
| Battery and power electronics | Supply the motors and electronics. Available propulsion depends on battery condition and the complete hardware configuration. |
| Flight processor | Runs sensor acquisition, state estimation, stabilization and motor-command generation onboard. |
| Radio/power-management processor | Handles communication and power-management tasks separately from the flight processor. |
| Crazyradio USB dongle | Links the laptop to the drone for commands, parameters and logs. |
| Expansion decks | Add sensors through the board's expansion connectors. Lighthouse is normally above the drone; Flow normally faces the ground below it. |
| Safety cage | Reduces exposure to propellers, but adds mass and can alter balance, inertia, airflow and optical visibility. It does not make flight harmless. |

The original Crazyflie 2.1 uses an STM32F405 flight processor and nRF51822
radio/power processor. Its stock motors are brushed; do not assume brushless
hardware or RPM telemetry. See the [Bitcraze hardware datasheet](https://www.bitcraze.io/documentation/hardware/crazyflie_2_1/crazyflie_2_1-datasheet.pdf).
Weigh the actual battery/decks/cage assembly rather than using a catalogue mass;
the [mass preflight procedure](practical_workflow.md#hardware-preflight-mass-and-controller-assumptions)
connects that measurement to the selected controller.

## Sensors on the main board

The Crazyflie 2.1 sensor combination is BMI088 plus BMP388. Check the Console for
your board rather than treating those chip names as universal across revisions.
Bitcraze describes this combination in its [Crazyflie 2.1 introduction](https://www.bitcraze.io/2019/02/the-crazyflie-2-1-is-here/).

| Sensor | Measurement | Interpretation and calibration |
| --- | --- | --- |
| Three-axis gyroscope, part of the [IMU](glossary.md#imu) | Body-axis angular velocity. Typical `gyro.x/y/z` log units are degrees/s. | Integrating angular rates propagates orientation; bias accumulates as drift. |
| Three-axis accelerometer, part of the IMU | Body-axis [specific force](glossary.md#specific-force): acceleration relative to free fall. Typical `acc.x/y/z` logs use g. | Translational acceleration follows after rotation to world coordinates and gravity handling. |
| Barometer | Air pressure, with a pressure-based altitude proxy and temperature information. | Altitude depends on the pressure reference, temperature and airflow. |
| Battery-voltage monitoring | Electrical supply voltage, for example `pm.vbat` in volts. | Voltage under load helps assess battery condition alongside charge history and propulsion demand. |

**IMU** means inertial measurement unit: here a three-axis gyro plus a three-axis
accelerometer. A stationary, level drone normally reads approximately +1 g on
its upward accelerometer axis. When tilted, that signal projects onto
other axes. Gyros should be close to zero at rest, but offsets and vibration
matter. Transform body measurements to the world frame before removing gravity;
subtracting 1 g from body Z alone is only appropriate when aligned with vertical.

IMU integration accumulates position drift, while gravity constrains roll and
pitch. Absolute yaw needs another reference. Pressure changes, propeller wash
and sensor offsets also limit barometric altitude. Complementary measurements
help constrain these uncertainties. The
[equations of motion](quadrotor_equations_of_motion.md) give the frame/gravity math.

## Flow deck V2: motion relative to the surface below

Two downward-facing sensors work together:

- **PMW3901 optical-flow sensor:** observes image displacement from ground texture,
  rather like an optical mouse. Converting this displacement to world velocity
  requires timing, distance, attitude and rotational-motion information.
- **VL53L1x time-of-flight (ToF) sensor:** uses light to estimate distance to the
  surface in its view. Relating this range to world altitude requires the sensor
  direction and the surface elevation.

The estimator combines flow, distance, attitude and rotational-motion information
to constrain horizontal velocity. Position can then be propagated from velocity,
but Flow alone does not provide a surveyed room origin or drift-free global XY.
It works without Lighthouse infrastructure. [Flow deck V2 hardware](https://www.bitcraze.io/products/flow-deck-v2/).

Important limitations for our experiments:

- Surface texture, illumination, reflectivity, distance and tilt affect readings.
  Poor texture or a moving surface can invalidate the assumed motion relationship.
- A raised gate floor changes the ToF return even at constant room altitude.
  A flat-floor measurement model may interpret this as drone motion. Range and
  optical-flow errors can consequently affect more than the estimated Z coordinate.
- Combining Flow with Lighthouse does not automatically resolve contradictory
  observations; fusion, uncertainty and outlier handling matter.

Those consequences follow from the sensor geometry and the
[firmware measurement models](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/functional-areas/sensor-to-control/state_estimators/#measurement-models).
The surface model determines how a range reading constrains global height.

## Lighthouse deck: observations referenced to the room

The deck has **four infrared receivers** and an FPGA for optical signal processing.
External base stations sweep infrared laser planes across the volume. Receiver
timing provides angular observations. Known station poses and calibration connect
these observations to the room's coordinate frame. The deck processes the optical
signals, and the onboard estimator combines the observations with other sensors.

The receiver geometry provides orientation constraints as well as positioning
information. Actual fused outputs depend on firmware and visibility. Lighthouse
requires valid geometry/calibration and clear optical paths; the cage, gates or
hands can occlude receivers. Repeated positions can look precise even when the
room geometry has a systematic offset. See Bitcraze's
[Lighthouse system description](https://www.bitcraze.io/documentation/system/positioning/ligthouse-positioning-system/)
and [deck hardware](https://www.bitcraze.io/products/lighthouse-positioning-deck/).

Flow references the **nearby surface**; Lighthouse references the **configured room
geometry**. Their different references provide complementary constraints.
Check visibility over the entire qualified course.

## Measurements, estimates and control are different layers

```text
IMU + enabled deck observations
              |
              v
      Onboard state estimator -----> position, velocity, attitude estimates
              |                                  |
              |                    reference ---> controller
              |                                  |
              |                                  v
              +<------ physical motion <------ motor commands

Laptop/cfclient: sends references and settings; receives selected telemetry.
```

The firmware can use different estimators/controllers. In our setup the Kalman
estimator is `stabilizer.estimator=2`; controller `1` is PID and `2` is Mellinger.
A script may change selection, so verify the running configuration. Mounting a
deck is not evidence that its observations are being fused successfully.

| Logged quantity | Interpretation |
| --- | --- |
| `gyro.*`, `acc.*`, pressure, flow, ToF or Lighthouse observations | Sensor-level channels, potentially already calibrated/filtered by firmware; inspect units and processing. |
| `stateEstimate.x/y/z`, velocities, roll/pitch/yaw | Fused onboard state estimates, with shared sensor inputs and filtering. |
| `ctrltarget.z`, host trajectory references | Requested altitude and motion references. |
| `stabilizer.thrust`, `motor.m1/m2/m3/m4` | Controller and motor command signals; conversion to force requires calibration. |

The [estimator overview](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/functional-areas/sensor-to-control/state_estimators/)
explains the onboard fusion. Feeding onboard position estimates into a second
Kalman filter is possible, but those estimates are already filtered and can be
correlated with IMU/deck observations. Treating all of them as independent raw
measurements can double-count information. Day 2 examines that distinction.

## Identify your team's configuration

During the introduction, locate the IMU/main board, motors, battery, radio,
Lighthouse receivers and downward Flow sensors where fitted. Explain which
observation constrains each state, what fails under occlusion or a changed floor,
and which channels are estimates rather than measurements. Record the fitted
hardware and total mass in the [Day 1 baseline sheet](day_1_baseline_checks.md).
No motor operation or base-station reconfiguration is needed for this introduction.
