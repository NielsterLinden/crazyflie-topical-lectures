# Shared outer-loop control backend

`controller.py` implements an acceleration-command PD controller with reference
acceleration feed-forward, acceleration/bank limits and effort diagnostics.
`trajectory.py` supplies the teaching reference position, velocity and acceleration
used by control and estimation examples. Both are reusable computations; the
interactive comparison lives in `day1/feedforward.py`.

From `code/`:

```text
python -m day1.feedforward
python -m unittest flight_control.test_controller
```

The simulated state contains position, velocity and achieved acceleration. A
first-order actuator approximates the inner-loop response. Commands are
gravity-excluded world acceleration; attitude and motor control remain onboard
in a corresponding hardware interface. The simulation closes feedback using
truth, independently of the participant Kalman filter.

Day 1's core [Andreas controllers](../andreas/README.md) use position-to-velocity
and velocity-to-tilt interfaces. Their gain units and plant assumptions differ
from this acceleration-command extension. Keep limits, reference and metrics
matched when comparing P, PD and feed-forward; neither toy model qualifies flight.

The [equations of motion](../../docs/quadrotor_equations_of_motion.md) derive the
frame conventions and reduced models. The [controller stretches](../../docs/controller_practical.md#stretch-exercises)
cover integral action/anti-windup, LQR/LQG, loop analysis, scheduling and MPC.
The [gravitational-wave analogies](../../docs/gravitational_wave_control_analogies.md)
connect these choices to feedback, actuators, lock acquisition and noise budgets.
