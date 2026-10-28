"""Checks for the reusable outer-loop trajectory controller."""

import unittest
import numpy as np

from .controller import ControllerConfig, OuterLoopController, TrajectoryReference


class ControllerTests(unittest.TestCase):
    def test_position_error_commands_correction_towards_reference(self) -> None:
        controller = OuterLoopController(ControllerConfig(feedforward_gain=0.0))
        reference = TrajectoryReference(np.array([1.0, 0.0, 0.0]), np.zeros(3), np.zeros(3))
        command = controller.command(reference, np.zeros(3), np.zeros(3))
        self.assertGreater(command.acceleration_world[0], 0.0)

    def test_acceleration_and_bank_are_limited(self) -> None:
        config = ControllerConfig(maximum_acceleration_m_s2=2.0, maximum_bank_deg=10.0)
        controller = OuterLoopController(config)
        reference = TrajectoryReference(np.array([100.0, 100.0, 0.0]), np.zeros(3), np.zeros(3))
        command = controller.command(reference, np.zeros(3), np.zeros(3))
        self.assertLessEqual(np.linalg.norm(command.acceleration_world), 2.0 + 1e-12)
        self.assertLessEqual(command.required_bank_rad, np.deg2rad(10.0) + 1e-12)
        self.assertTrue(command.saturated)

    def test_bank_limit_accounts_for_vertical_acceleration(self) -> None:
        config = ControllerConfig(maximum_acceleration_m_s2=20.0, maximum_bank_deg=10.0)
        controller = OuterLoopController(config)
        reference = TrajectoryReference(
            np.zeros(3), np.zeros(3), np.array([10.0, 0.0, -5.0])
        )
        command = controller.command(reference, np.zeros(3), np.zeros(3))
        self.assertLessEqual(command.required_bank_rad, np.deg2rad(10.0) + 1e-12)
        self.assertTrue(command.saturated)



if __name__ == "__main__":
    unittest.main()
