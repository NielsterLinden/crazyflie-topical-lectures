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

The deck has **four infrared receivers** (photodiodes) and an FPGA for optical
signal processing. Each base station contains a rotating laser-scanning assembly,
called a **rotor**, which sweeps laser planes across the room. Lighthouse V1 uses
two rotating drums; V2 uses one drum producing two slanted laser planes. The
timing of a plane passing a receiver determines its sweep angle. Multiple sweeps,
receivers and stations provide different geometric constraints on the drone.

The configured **system geometry** gives each station's position and orientation
in room coordinates. Its **factory calibration** describes imperfections in the
laser sweeps. The known receiver offsets on the deck connect those observations
to the drone's position and orientation. Position estimation runs onboard, so the
radio link carries commands and telemetry while the drone processes Lighthouse
observations locally.

### From a sweep angle to a Kalman-filter correction

The onboard extended Kalman filter (EKF) propagates position, velocity and
orientation using the IMU. An incoming sweep then supplies an angular measurement
to correct that prediction. For one receiver and one sweep:

1. **Predict the receiver position.** With drone position $\mathbf p$, body-to-world
   rotation $R$ and receiver mounting offset $\mathbf s$, its world position is
   $\mathbf p_s=\mathbf p+R\mathbf s$.
2. **Express it relative to the scanner.** If the rotor's world position is
   $\mathbf p_r$ and its rotor-to-world rotation is $R_r$, the receiver coordinates
   in that frame are $\mathbf u=R_r^T(\mathbf p_s-\mathbf p_r)$.
   This frame describes the scanner's fixed mounting; the sweep angle describes
   the rotating laser plane.
3. **Predict the angle.** A geometric model, including factory calibration,
   converts $\mathbf u$ into an expected sweep angle $h(\hat{\mathbf x}^{-})$.
   The predicted residual is measured minus predicted angle:
   $r=\theta_{\rm measured}-h(\hat{\mathbf x}^{-})$.
4. **Update the estimate.** The firmware checks for outliers and unsuitable
   geometry, then applies a scalar EKF update for an accepted angle.

Writing $H$ for the angle's sensitivity to the state, $C^{-}$ for the predicted
state-error covariance and $V=\sigma_\theta^2$ for the angular measurement
variance, the update uses the lecture's [C/R/V notation](glossary.md#filter-covariances):

$$
R=HC^{-}H^T+V,\qquad
K=C^{-}H^T/R,\qquad
\hat{\mathbf x}^{+}=\hat{\mathbf x}^{-}+Kr.
$$

Here $R$ is the predicted-residual variance and $K$ the Kalman gain. The covariance
is updated too. Thus each angle corrects the state according to its measurement
uncertainty and the current state uncertainty. This path fuses individual angles
directly, with successive sweeps constraining different directions.

For implementation details, see
[`kalmanCoreUpdateWithSweepAngles()`](https://github.com/bitcraze/crazyflie-firmware/blob/b48aba18a675fe950a88d96586fd09e49d82a38b/src/modules/src/kalman_core/mm_sweep_angles.c#L29).
In that revision the angle prediction includes calibration, while $H$ uses an
ideal geometric derivative with nonzero entries only for position. The rotated
receiver offset is held fixed during this linearization; velocity and attitude
can still be corrected through state cross-covariances. This is an approximation
to differentiating the complete position-and-orientation measurement model.

### Visibility and accuracy

Lighthouse requires valid geometry/calibration and clear optical paths; the cage,
gates or hands can occlude receivers. During a gap, IMU propagation continues,
but uncertainty can grow. Repeated positions can look precise even when the room
geometry has a systematic offset. See Bitcraze's
[Lighthouse system description](https://www.bitcraze.io/documentation/system/positioning/ligthouse-positioning-system/)
and [deck hardware](https://www.bitcraze.io/products/lighthouse-positioning-deck/).

Flow references the **nearby surface**; Lighthouse references the **configured room
geometry**. Their different references provide complementary constraints.
Check visibility over the entire qualified course.

## Residuals for the other sensors

This section gives background for interpreting the Day 2 measurements. Residuals
use the same sign convention as Lighthouse: **measured minus predicted**. In a
conventional EKF correction, the prediction and covariance describe the state
immediately before that correction. For a measurement vector $\mathbf y$ with
measurement-noise covariance $V$,

$$
\mathbf r=\mathbf y-h(\hat{\mathbf x}^{-}),\qquad
R=HC^{-}H^T+V.
$$

The residual has the measurement's units. A scalar residual can be normalized as
$r/\sqrt R$; the corresponding chi-square is $r^2/R$. A vector uses
$\chi^2=\mathbf r^T R^{-1}\mathbf r$. These assess agreement with both the
measurement uncertainty and the predicted state uncertainty. The sequential
Flow implementation below has a timing approximation to account for when
interpreting these quantities.

### Flow deck: image-motion residuals

The flow sensor measures image displacement accumulated over an interval
$\Delta t$. The firmware predicts that displacement from body-frame velocity,
height, orientation and gyro readings. In the
[`mm_flow.c` model](https://github.com/bitcraze/crazyflie-firmware/blob/54f31e243a0b28b67efef5ba20dbb6d9890a5478/src/modules/src/kalman_core/mm_flow.c),
the predicted displacements are

$$
\hat n_x=c\left(\frac{v_{c,x}R_{33}}{z_*}-\omega_y\right),\qquad
\hat n_y=c\left(\frac{v_{c,y}R_{33}}{z_*}+\omega_x\right),\qquad
c=\frac{\Delta t\,N_{\rm pix}}{\theta_{\rm pix}}.
$$

Here $R_{33}$ is the vertical projection of body Z (`R[2][2]`),
$z_*=\max(\hat z,0.1\,\mathrm m)$, and the camera scale constants are
$N_{\rm pix}=35$ and $\theta_{\rm pix}=0.71674$. The camera velocity includes its
mounting offset $\boldsymbol\ell$:
$\mathbf v_c=\mathbf v_{\rm body}+\boldsymbol\omega\times\boldsymbol\ell$.
The explicit gyro terms account for image motion due to rotation.

After converting the sensor's raw displacement counts by the firmware factor
0.1, the residuals are $r_x=n_x-\hat n_x$ and $r_y=n_y-\hat n_y$, in pixels.
The firmware applies two sequential scalar corrections, X then Y, each directly
constraining body velocity and height. Both displacement predictions
use camera velocities and height cached before the X correction. The Y correction
therefore uses a prediction from before X, but a covariance already updated by X.
When reconstructing the Y residual, use those cached inputs rather than the
post-X state. Normalizing it with the post-X covariance gives an approximate
diagnostic; interpreting its square as a calibrated chi-square requires accounting
for this mismatch and the correlations between the two components.

The model's use of world Z as height assumes a floor at the configured zero
elevation. A raised surface can therefore produce a flow residual even when the
drone follows its intended path.

The logs `kalman_pred.measNX/NY` and `kalman_pred.predNX/NY` expose the measured
and predicted displacements. Pair values from the same Flow sample when forming
residuals; slower telemetry may miss updates.

### Flow deck: ToF range residual

For a level drone above the reference floor, the predicted range is approximately
$\hat d=\hat z$. The
[`mm_tof.c` model](https://github.com/bitcraze/crazyflie-firmware/blob/54f31e243a0b28b67efef5ba20dbb6d9890a5478/src/modules/src/kalman_core/mm_tof.c)
includes tilt and the sensor's measurement cone:

$$
\alpha=\max\!\left(\arccos R_{33}-7.5^\circ,0\right),\qquad
\hat d=\frac{\hat z}{\cos\alpha},\qquad r_d=d_{\rm measured}-\hat d.
$$

The residual is in metres, with position sensitivity $H_z=1/\cos\alpha$.
The update requires $R_{33}>0.1$. Over a raised gate floor, the measured range
becomes shorter, producing a negative residual under this flat-floor model.
The correction then pulls the height estimate down; its coupling to flow can
also affect horizontal velocity. Outlier handling or a different surface model
is needed to distinguish that surface change from drone motion.

### IMU: propagation and stationary calibration residuals

Gyro and accelerometer samples enter
[`kalmanCorePredict()`](https://github.com/bitcraze/crazyflie-firmware/blob/54f31e243a0b28b67efef5ba20dbb6d9890a5478/src/modules/src/kalman_core/kalman_core.c)
as **process inputs**. Gyro integration advances orientation; accelerometer input
advances motion with rotation and gravity handling. In the normal flight mode,
the predictor uses body-Z specific force under a thrust-along-body-Z model;
when the firmware reports the drone as not flying, it uses all three components.
Their assumed noise contributes to the propagated state-error covariance.

During a stationary calibration, the physical reference is zero angular rate and
specific force opposing gravity. With upward world axis $\mathbf e_3$, useful
diagnostic residuals are

$$
\mathbf r_\omega=\boldsymbol\omega_{\rm measured},\qquad
\mathbf r_f=\mathbf f_{\rm measured}-R^T g\mathbf e_3.
$$

Use rad/s and m/s squared after converting the raw logs. The means reveal gyro
offsets and accelerometer/attitude inconsistencies; the scatter characterizes
stationary noise. Separating accelerometer bias from a tilt error requires
additional information, such as measurements in several known orientations.
These are **stationary calibration diagnostics**: this onboard EKF consumes the
IMU as inputs, so it forms no separate gyro or accelerometer measurement-update
residual. Comparing IMU data with motion derived from the same fused state also
introduces correlations that must be included in any uncertainty interpretation.

### Barometer and battery monitoring

A pressure-derived altitude can supply a residual of the form
$r_b=(h_{\rm pressure}-h_{\rm reference})-\hat z$, in metres, after aligning the
altitude reference. Pressure changes and propeller wash affect this comparison.
In the [estimator revision used here](https://github.com/bitcraze/crazyflie-firmware/blob/54f31e243a0b28b67efef5ba20dbb6d9890a5478/src/modules/src/estimator/estimator_kalman.c),
barometer fusion is disabled by default through the commented-out
`KALMAN_USE_BARO_UPDATE` definition. Battery voltage is an operational diagnostic
for power and propulsion; it supplies no navigation-state measurement correction.

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
