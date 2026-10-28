# Shared Kalman backend

Reusable six-state position/velocity estimation, with world-frame acceleration
as process input. This package handles numerical prediction, updates and prepared
recording validation. Day-specific commands, plots and participant decisions live
in [day2](../day2/README.md).

- `filter.py`: `FilterConfig`, `PositionVelocityKalmanFilter`, Q/R/P, innovations,
  NIS, rejection and Joseph-form covariance updates.
- `pipeline.py`: IMU/position/flow/range/stationary callback interface.
- `recording.py`: prepared CSV/metadata validation, `ReplayFilter` and replay.
- `simulation.py`: reproducible baseline sensor/filter computation, without
  gate geometry, strategy or plotting. The reference motion comes from
  [flight_control](../flight_control/README.md).

From `code/`, run `python -m unittest kalman.test_filter`.
For the learner-facing baseline, run `python -m day2.simulate`.
The [worksheet](../../docs/kalman_filter_practical.md) explains conventions,
calibration, live measurements, prepared recordings and uncertainty interpretation.
