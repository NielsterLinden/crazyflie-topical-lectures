"""Engage and verify the firmware stop before hand-held measurements."""

import math
import queue
import time


MOTOR_CHANNELS = tuple(f"motor.m{index}" for index in range(1, 5))
LOCKED_BIT = 1 << 6
ACTIVE_BITS = (1 << 1) | (1 << 3) | (1 << 4)  # Armed, can fly, flying.


def _status_log():
    # Keep offline imports and geometry tests independent of the radio library.
    from cflib.crazyflie.log import LogConfig

    config = LogConfig(name="Hand-held stop check", period_in_ms=50)
    config.add_variable("supervisor.info", "uint16_t")
    for name in MOTOR_CHANNELS:
        config.add_variable(name, "float")
    return config


def engage_motor_stop(cf, timeout_s=5):
    """Set stop=1 and require readback plus fresh locked/zero-output telemetry.

    The caller must have downloaded parameters. No exit path clears the stop:
    timeout, interruption and connection loss leave any engaged lock in place.
    Recovery for flight is an explicit power cycle followed by preflight checks.
    """
    if not math.isfinite(timeout_s) or timeout_s <= 0:
        raise ValueError("Motor-stop verification needs a finite positive timeout.")
    cf.param.set_value("stabilizer.stop", "1")
    required = ("supervisor.info", *MOTOR_CHANNELS)
    missing = [name for name in required if cf.log.toc.get_element_by_complete_name(name) is None]
    if missing:
        raise ValueError("Cannot verify firmware stop; missing telemetry: " + ", ".join(missing))

    events = queue.Queue(maxsize=20)

    def receive(_timestamp, data, _config):
        try:
            events.put_nowait((dict(data), None))
        except queue.Full:
            pass  # The finite verification queue needs only a recent status packet.

    def error(_config, message):
        try:
            events.put_nowait((None, message))
        except queue.Full:
            pass

    config = _status_log()
    config.data_received_cb.add_callback(receive)
    config.error_cb.add_callback(error)
    cf.log.add_config(config)
    deadline = time.monotonic() + timeout_s
    try:
        config.start()
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError("Firmware stop was not verified; leave the drone on the surface "
                                   "and use a checked recording or simulation.")
            try:
                data, message = events.get(timeout=remaining)
            except queue.Empty as cause:
                raise TimeoutError("No verified firmware-stop telemetry; leave the drone on the "
                                   "surface and use a checked recording or simulation.") from cause
            if message is not None:
                raise RuntimeError("Firmware-stop logging failed: " + str(message))
            values = [float(data[name]) for name in required]
            if not all(math.isfinite(value) for value in values):
                raise ValueError("Invalid firmware-stop telemetry; do not handle the drone.")
            if not values[0].is_integer() or not 0 <= values[0] <= 65535:
                raise ValueError("Invalid supervisor status; do not handle the drone.")
            info = int(values[0])
            stopped = int(cf.param.get_value("stabilizer.stop")) == 1
            if stopped and info & LOCKED_BIT and not info & ACTIVE_BITS and all(value == 0 for value in values[1:]):
                return {"parameter": "stabilizer.stop", "value": 1,
                        "supervisor_info": info, "motor_outputs": values[1:]}
    finally:
        for cleanup in (config.stop, config.delete):
            try:
                cleanup()
            except Exception:
                pass  # Lost radio contact must never trigger a command to clear the lock.
