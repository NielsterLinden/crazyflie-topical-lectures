# Day 1: controller experiments

This front end runs Andreas's supplied position, velocity and cascade experiments.
The controllers, toy plants and guided notebook are in [andreas](../andreas/README.md).
Follow the [Day 1 software guide](../../docs/day_1_software.md) and
[C1-C7 worksheet](../../docs/controller_practical.md).

From the package root, with the environment activated:

```text
cd code
python -m day1 --output ../day1/baseline --no-show
python -m unittest andreas.test_practical flight_control.test_controller
```

`python -m day1.feedforward` is the optional acceleration-command comparison,
using the shared [flight_control](../flight_control/README.md) backend. Its gains
and interface differ from the core velocity/tilt loops. These entry points run
offline simulations.
