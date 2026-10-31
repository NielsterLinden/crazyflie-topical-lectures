# Shared outer-loop control backend

`controller.py` implements an acceleration-command PD controller with reference
acceleration feed-forward, acceleration/bank limits and effort diagnostics.
`trajectory.py` supplies the teaching reference position, velocity and acceleration
used by control and estimation examples. Both are reusable computations.
For a notebook experiment with smooth references and feed-forward, see
[Day 1 trajectories](../day1/extra_2Dsim_03_trajectories.ipynb), which uses the
Day 1 2D simulator and its own controller implementation.

From `code/`:

```text
python -m unittest flight_control.test_controller
```

The simulated state contains position, velocity and achieved acceleration. A
first-order actuator approximates the inner-loop response. Commands are
gravity-excluded world acceleration; attitude and motor control remain onboard
in a corresponding hardware interface. The simulation closes feedback using
truth, independently of the participant Kalman filter.

The [Day 1 notebooks](../day1/README.md) develop nested loops from position
through velocity and tilt to motor commands. Their gain units and plant assumptions differ
from this acceleration-command extension. Keep limits, reference and metrics
matched when comparing P, PD and feed-forward; neither toy model qualifies flight.

The [equations of motion](../../docs/quadrotor_equations_of_motion.md) derive the
frame conventions and reduced models. The [controller stretches](../../docs/controller_practical.md#stretch-exercises)
cover integral action/anti-windup, LQR/LQG, loop analysis, scheduling and MPC.
The [gravitational-wave analogies](../../docs/gravitational_wave_control_analogies.md)
connect these choices to feedback, actuators, lock acquisition and noise budgets.
