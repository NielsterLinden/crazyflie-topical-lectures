# Practical glossary

Short definitions for the control, estimation and flight terms used in the
[day guides](README.md). Links in the guides point here when a term first appears.

## Actuator

A device that applies a control action to the physical system. The drone's
motors and propellers generate thrust and torque. Motor PWM (pulse-width
modulation) sets an electrical command; relating it to force requires calibration.

## Anti-windup

A controller mechanism that limits or suspends integral accumulation when a
command reaches its allowed range. It helps the controller recover when the
requested motion becomes achievable again. See [saturation](#saturation).

## Adapter and runner

A runner is the program that carries out an experiment: it reads settings,
processes data or sends commands, and saves results. An adapter is code that
connects two interfaces, for example by converting sensor logs into the filter's
input format or translating a controller's output into drone commands. A flight
adapter must also handle units, coordinate frames, timing, limits and stopping.

## Attitude and bank

Attitude is the drone's orientation relative to the world frame. Roll, pitch
and yaw describe one parameterization of that orientation. Banking tilts the
thrust direction, allowing a horizontal component of acceleration.

## Bandwidth and loop margins

Bandwidth describes the frequency range over which a feedback loop responds
effectively. Gain and phase margins quantify how much gain or phase change a
linear loop can tolerate before instability; delays consume phase margin.
See the [feedback-loop discussion](gravitational_wave_control_analogies.md#one-feedback-loop-language).

## Bias and calibration

Bias is a systematic offset in a measurement or model prediction. Calibration
estimates a sensor's relationship to a known reference, including offsets,
scales and their uncertainties. Independent validation checks the correction
on different data. See the [calibration exercise](sensor_bias_calibration_exercise.md).

## Bumpless handover

A transfer between controllers that keeps the outgoing command continuous at
the switching instant. The incoming controller's reference and internal state,
including any integral term, are initialized to match the current command.
Tracking and stability still need checking after the switch.

## Cascade

Nested feedback loops: an outer loop supplies the reference for an inner loop.
For example, the position loop requests a velocity, and the velocity loop requests
a tilt. Each loop has its own command units, dynamics and bandwidth. The
[estimation chain](#cascaded-estimation) uses the related term "cascaded estimation."

## Covariance

A matrix describing uncertainty: diagonal entries are variances, and
off-diagonal entries describe correlations through covariances between components.
Its units depend on the quantities represented. See [filter covariances](#filter-covariances).

## Cascaded estimation

An estimation chain in which one estimator's output becomes another estimator's
input. In E6, onboard position and attitude contribute to the laptop filter.
Shared sensor inputs and filtering can correlate these quantities; their
uncertainties and dependencies belong in the model and interpretation. See
[filtering and smoothing](#filtering-and-smoothing) for the time ordering of measurements.

## Clearance and gate normal

The gate normal is a unit vector perpendicular to its plane. Its sign defines
the requested crossing direction. Clearance is the space remaining after
accounting for the opening, drone envelope, crossing offset and chosen margins.
Uncertainty projected into the gate plane helps set those margins.
See the [survey guide](hoop_survey_challenge.md#constraints-and-interface).

## Controller gain

A coefficient relating an error or state to a command. Its units depend on the
input and output: a position-to-velocity proportional gain has units 1/s,
while a position-to-acceleration gain has units 1/s2. Gains are interpreted
together with their command layer and plant model.

## Drift

A slowly changing error, such as apparent motion of a stationary drone or
accumulated orientation error from integrating a gyro offset. Its cause may lie
in a sensor, reference frame or propagation model. A random-walk model represents
an evolving quantity through successive random increments.

## EKF and error-state EKF

An extended Kalman filter (EKF) propagates nonlinear state and measurement models
while linearizing their uncertainty relationships about the current estimate.
An error-state EKF estimates small deviations from a separately propagated nominal
state, useful when orientation is represented by a rotation matrix or quaternion.
LHCb track fitting uses a closely related reference-trajectory linearization: pattern
recognition supplies an initial trajectory, and the linearized fit is expressed in
deviations from reference states along it; later iterations can use the preceding fit
as the new reference. This is important because charged-particle transport through
LHCb's inhomogeneous dipole field is nonlinear. The tracking measurements are mostly
made in detector stations before and after the strong central field, so the reference
trajectory connects measurements across the magnet rather than sampling the path
continuously inside it. See Sections 6.2--6.3 of Edwin Bos's VU/Nikhef PhD thesis,
[*Reconstruction of charged particles in the LHCb experiment*](https://research.vu.nl/ws/portalfiles/portal/42192031/complete%20dissertation.pdf),
and Sections 6.1.3--6.1.5 of Jeroen van Tilburg's earlier VU/Nikhef PhD thesis,
[*Track simulation and reconstruction in LHCb*](https://repository.cern/records/8axnk-w9f84).

## Feed-forward

A command calculated from a known reference or measured disturbance, alongside
feedback based on tracking error. Reference acceleration feed-forward supplies
the acceleration needed by the planned motion. See the
[control examples](gravitational_wave_control_analogies.md#feedback-trajectory-feed-forward-and-noise-feed-forward).

## Filter covariances

In the Kalman filter, **Q** describes uncertainty added during process-model
propagation, **R** describes measurement noise, and **P** describes uncertainty
in the state estimate. The innovation covariance **S** combines predicted state
uncertainty and measurement uncertainty for the current update.

## Filtering and smoothing

Filtering estimates the state using measurements available up to the current
time. This is causal: it can run during the experiment. Smoothing also uses
later measurements to refine an earlier state, making it useful for offline
reconstruction. See the [tracking analogy](collider_tracking_kalman_analogies.md#predicted-filtered-and-smoothed-states).

## Frames

A coordinate frame specifies an origin and axes. The Lighthouse world frame
is fixed to the configured room geometry; the body frame moves with the drone.
This course uses world Z upward. A body-to-world rotation maps vector components
between these frames. See [frame conventions](quadrotor_equations_of_motion.md#frames-and-conventions).

## Held-out data

Data reserved for evaluating a chosen calibration, model or controller after
tuning. Keeping development and validation data separate lets us assess how
well the result generalizes.

## Ground truth and in-loop measurements

Ground truth is the known state used to generate a simulation. In hardware,
an independent reference provides a separate measurement with its own uncertainty.
An in-loop measurement supplies the active controller's feedback; its residual
shows behaviour within that loop. Independent observations help distinguish
physical tracking accuracy from sensor noise followed by the controller.

## IMU

Inertial measurement unit: the Crazyflie's three-axis gyroscope and three-axis
accelerometer. They provide body-axis angular rates and [specific force](#specific-force).

## Innovation and NIS

The innovation is the measurement minus its prediction, evaluated before adding
that measurement to the estimate: \(y=z-H\hat x^-\) for a linear measurement.
Normalized innovation squared (NIS) is \(y^TS^{-1}y\), using the
[innovation covariance](#filter-covariances) \(S\). Under a consistent unbiased
Gaussian model it follows a chi-square distribution with degrees of freedom
equal to the measurement dimension. See [gating](#innovation-gating-and-tukey-weighting)
for selection based on this statistic and [whitening](#whitening-and-cholesky-factorization)
for correlated innovations.

## Innovation gating and Tukey weighting

Innovation gating compares a measurement's NIS with a chosen threshold to decide
whether to include the update. The threshold depends on measurement dimension
and the intended false-rejection probability. Tukey weighting progressively
downweights large residuals and gives zero weight beyond a cutoff. Both select
the information entering the filter; retain pre-selection diagnostics.

## Jacobian

A matrix of partial derivatives describing how a vector-valued function changes
near a chosen point. An EKF uses Jacobians of its process and measurement models
to propagate covariance and calculate measurement updates.

## Jerk and snap

Jerk is the third time derivative of position; snap is the fourth.
Continuous jerk gives matching left/right jerk values where trajectory segments
join. Minimum-snap planning minimizes an integrated squared-snap objective.
See the [trajectory extension](trajectory_construction.md#stretch-goal-continuous-and-limited-jerk).

## Kalman filter

A recursive estimator alternating model-based prediction with measurement
updates. State and measurement covariances determine the weighting. For linear
models with Gaussian noise, it gives the conditional state mean and covariance.
See the [estimation practical](kalman_filter_practical.md).

## LQR and LQG

A linear quadratic regulator (LQR) chooses feedback gains to minimize a quadratic
cost in state deviations and control effort for a linear model. Linear quadratic
Gaussian control (LQG) combines LQR feedback with a Kalman state estimator under
the corresponding linear-Gaussian assumptions.

## Latency, jitter and dropout

Latency is the elapsed time between an event, such as measurement acquisition,
and its availability or use elsewhere. Jitter is variation in timing or intervals;
a dropout is a period of missing observations. A deadline specifies the latest
acceptable time for an update or command. State the clocks and events used when
measuring any of these quantities.

## Mellinger controller

The Crazyflie firmware's geometric trajectory-tracking controller. It relates
desired translational motion to thrust and attitude, then controls orientation.
Its inputs depend on the selected command interface. See
[onboard and laptop responsibilities](day_3_race.md#what-runs-on-the-drone-and-on-the-laptop).

## MIMO

Multi-input, multi-output: a model or controller with several coupled inputs
and outputs. For the drone, tilt, altitude, yaw and thrust can interact.

## MPC

Model predictive control: repeatedly predict motion over a finite horizon,
optimize commands subject to a model and constraints, apply the first command,
then repeat using the updated state estimate.

## Observability

The ability to distinguish state components from a sequence of measurements
and known inputs. It depends on the model, available sensors and motion.
Weakly constrained states need appropriate uncertainty and, where possible,
additional reference measurements.

## PD and PID

Proportional-derivative (PD) feedback combines a term proportional to error
with a derivative term for damping. Proportional-integral-derivative (PID)
adds accumulated error to address persistent offsets. In the Day 1 position
controller, the derivative term uses measured velocity. Gains have units set
by the loop's input and output. See the [controller equations](day_1_software.md#controller-layers-and-units).

## Plant

The physical system seen by a controller, including any inner loops treated as
part of its response. For example, a position controller requesting velocity
sees the drone and its onboard velocity/attitude loops as its plant.

## Process input and process noise

A process input is a supplied quantity driving state prediction, such as
world-frame acceleration in the six-state filter. Process noise represents
uncertainty in that propagation. A measurement instead supplies information
used to correct the predicted state.

## Pull and coverage

A scalar pull is an error divided by its predicted standard deviation. Vector
innovations can be [whitened](#whitening-and-cholesky-factorization) using their full covariance. Coverage is the fraction
of reference values contained in stated uncertainty intervals over repeated
trials. Both require a specified reference and covariance definition.

## Qualification and fallback

Qualification is the team's documented check that a specific hardware, software
and course configuration meets the agreed flight requirements, including a
different member's cross-check and a low-speed test. All group members agree
before moving to the next stage. A fallback is an already
checked configuration available when a candidate is unfinished or fails a check.
See the [team flight checks](race_practical.md#r5-qualify-and-freeze-the-race-configuration).

## Quaternion and SO(3)

A unit quaternion uses four components constrained to unit norm to represent
orientation. The rotation group \(SO(3)\) consists of three-dimensional rotation
matrices satisfying \(R^TR=I\) and \(\det R=1\). Both representations support
propagating orientation from angular velocity. See
[rotations, complex numbers and quaternions](rotations_and_quaternions.md) for
the algebra and group structure, and
[orientation kinematics and references](quadrotor_equations_of_motion.md#orientation-kinematics).

## Radio URI

The connection string identifying the radio link to a Crazyflie. In
`radio://0/80/2M/E7E7E7E701`, the fields select radio dongle index 0, channel 80,
2 Mbit/s data rate and the drone's radio address. Use the team's assigned values
and match the drone's stored settings. See [radio setup](software_setup.md#team-radio-configuration).

## Reference and setpoint

A reference specifies desired motion over time; a setpoint gives a requested
value at a particular instant. Position, velocity, attitude and motor-command
interfaces each expect different quantities and units.

## Residual

A difference between an observation and a corresponding model prediction or
estimate. Its sign and the state used to compute it are part of its definition.
The [innovation](#innovation-and-nis) uses the state before the current update.
Post-update residuals have different uncertainty because that measurement has
already influenced the estimate.

## Response metrics

For a step response, rise time measures travel between stated fractions of the
requested change, here 10-90%. Overshoot measures how far the response passes
the target. Settling time is the time after which the response remains within
a stated tolerance for the rest of the observation interval. Damping describes
how oscillations decay. Record the reference step, tolerance and interval used.

## RMS and RMSE

Root mean square (RMS) is \(\sqrt{N^{-1}\sum_k x_k^2}\). Applied to errors it
is root mean square error (RMSE), in the same units as the error. State the
components and time interval included; a vector-position RMSE combines axes,
while a per-axis RMSE describes one component.

## Run card and configuration freeze

A run card is a short record of how a flight will be carried out: the exact
command, code and settings, drone and course, operating limits, pilot and
emergency-stop operator. A configuration freeze is the agreed time to stop
changing the estimator or controller before racing. Save the chosen version
and settings, then complete the remaining checks. Any later safety-related
change needs the relevant checks repeated; a deadline never overrides a failed
safety check. See the [race preparation steps](race_practical.md#freeze-points-and-fallback).

## Saturation

A command reaching an allowed actuator or software limit. Once saturated, the
system's response differs from the unlimited linear model. [Anti-windup](#anti-windup)
helps manage integral accumulation in this condition.

## Shadow estimator

An estimator running alongside the active flight system for comparison and
diagnostics. The active controller continues using its configured feedback source;
using the shadow output for control requires an explicit, qualified integration.

## Specific force

The accelerometer's acceleration relative to free fall, expressed in its sensor
axes. A stationary, level drone measures approximately +1 g along its upward
axis. Translational acceleration follows by rotating to the world frame and
adding world gravity. See the [IMU measurement equations](quadrotor_equations_of_motion.md#relation-to-imu-measurements).

## State and measurement models

The state collects the quantities being estimated. A process model predicts how
they evolve, using known inputs and process noise. A measurement model predicts
the observations from that state. In the six-state exercise, position and
velocity form the state, acceleration drives propagation, and position, flow
velocity or height observations provide updates.

## Sweep

A parameter sweep repeats an experiment over a chosen range of parameter values
to compare their effects. For example, a gain sweep varies the derivative gain
\(K_d\) while keeping the other gains, reference motion and test conditions
fixed, then compares overshoot and settling time. Use the same recorded data or
simulation noise seed for matched comparisons. A frequency sweep instead varies
the frequency of a test input to measure the system's response.

## System identification

Estimating a plant's dynamics or parameters from known inputs and observed
responses, such as actuator lag or drag. A useful result includes the excitation,
units, timing, fitted model and validation on held-out data.

## Telemetry and Log TOC

Telemetry is data sent from the drone, such as sensor readings, state estimates
and motor commands. The logging table of contents (Log TOC) lists available
firmware log variables and their types. Logging periods specify how often values
are transmitted; sensor-acquisition timing needs its own interpretation.

## Trajectory continuity

A trajectory is C2 when position and its first two derivatives are continuous;
C3 adds continuous jerk. A quintic polynomial has degree five; a septic has
degree seven. Matching segment endpoint derivatives determines continuity at joins.
See [trajectory construction](trajectory_construction.md#model-and-checks).

## Whitening and Cholesky factorization

Whitening rescales correlated residuals using their covariance. For positive
definite innovation covariance \(S=LL^T\), Cholesky factorization gives a lower
triangular matrix \(L\); solve \(Lw=y\) for the whitened innovation \(w\).
Under the filter's unbiased Gaussian assumptions, its components have unit
covariance, and \(w^Tw\) is the NIS. See [uncertainty diagnostics](kalman_filter_practical.md#e3-tune-uncertainty-not-just-smoothness).

## ZUPT

Zero-velocity update: a filter update using a zero-velocity constraint during
verified stationary intervals. Its validity depends on the stationarity checks.
See [stationary constraints](kalman_filter_practical.md#e5-missing-measurements-and-stationary-constraints).
