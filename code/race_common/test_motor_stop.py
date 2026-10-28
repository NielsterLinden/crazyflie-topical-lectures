"""Radio-free verification of the hand-held firmware-stop gate."""

from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from .motor_stop import MOTOR_CHANNELS, engage_motor_stop


class Callbacks:
    def add_callback(self, callback):
        self.callback = callback


class StopTests(unittest.TestCase):
    def setup_connection(self, packets, readback="1", missing=False):
        config = SimpleNamespace(data_received_cb=Callbacks(), error_cb=Callbacks(),
                                 stop=Mock(), delete=Mock())

        def start():
            for info, motors in packets:
                config.data_received_cb.callback(100, {"supervisor.info": info,
                                                       **dict(zip(MOTOR_CHANNELS, motors))}, config)

        config.start = start
        cf = SimpleNamespace(
            param=SimpleNamespace(set_value=Mock(), get_value=Mock(return_value=readback)),
            log=SimpleNamespace(add_config=Mock(), toc=SimpleNamespace(
                get_element_by_complete_name=lambda name: None if missing else object())))
        return cf, config

    def test_sets_stop_and_requires_lock_zero_outputs_and_readback(self):
        # Idle alone, locked-but-armed, and nonzero output cannot pass.
        packets = [(0, [0]*4), (66, [0]*4), (64, [1, 0, 0, 0]), (64, [0]*4)]
        cf, config = self.setup_connection(packets)
        with patch("race_common.motor_stop._status_log", return_value=config):
            result = engage_motor_stop(cf)
        self.assertEqual(result["supervisor_info"], 64)
        self.assertEqual(result["motor_outputs"], [0]*4)
        cf.param.set_value.assert_called_once_with("stabilizer.stop", "1")
        config.stop.assert_called_once()
        config.delete.assert_called_once()

    def test_timeout_or_bad_readback_never_clears_the_stop(self):
        for packets, readback in (([], "1"), ([(64, [0]*4)], "0")):
            cf, config = self.setup_connection(packets, readback)
            with patch("race_common.motor_stop._status_log", return_value=config):
                with self.assertRaises(TimeoutError):
                    engage_motor_stop(cf, timeout_s=0.01)
            cf.param.set_value.assert_called_once_with("stabilizer.stop", "1")
            config.stop.assert_called_once()
            config.delete.assert_called_once()

    def test_missing_channels_fail_closed_after_requesting_stop(self):
        cf, config = self.setup_connection([], missing=True)
        with self.assertRaisesRegex(ValueError, "missing telemetry"):
            engage_motor_stop(cf)
        cf.param.set_value.assert_called_once_with("stabilizer.stop", "1")
        cf.log.add_config.assert_not_called()

    def test_invalid_telemetry_and_interruption_leave_stop_engaged(self):
        for failure in (ValueError, KeyboardInterrupt):
            cf, config = self.setup_connection([(64, [float("nan"), 0, 0, 0])])
            if failure is KeyboardInterrupt:
                config.start = Mock(side_effect=KeyboardInterrupt)
            with patch("race_common.motor_stop._status_log", return_value=config):
                with self.assertRaises(failure):
                    engage_motor_stop(cf)
            cf.param.set_value.assert_called_once_with("stabilizer.stop", "1")
            config.stop.assert_called_once()
            config.delete.assert_called_once()


if __name__ == "__main__":
    unittest.main()
