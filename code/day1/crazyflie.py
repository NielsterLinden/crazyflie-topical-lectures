"""Real Crazyflie helpers for the Day 1 flight notebooks.

The simulation notebooks keep all code in the notebook. For the real drone the
radio link, logging, arming, take-off, landing and the safety checks are the
same for every group and are not the point of the exercise, so they live here.
The controller stays in the notebook: both flight loops take a function

    controller(x, y, vx, vy, x_target, y_target) -> (vx_command, vy_command)

that turns the measured horizontal position and velocity (metres, m/s, from the
onboard estimator) into a horizontal velocity command. The height is held by
the onboard controller. This is the position loop of the velocity cascade from
2Dsim_04_velocity_loop, with the velocity and attitude loops running in the firmware.

The chain of the afternoon, each step feeding the next:

  identify_sensors(...)      motors off. A guided protocol: the drone still on
                             the desk, then moved by hand forward and to the
                             left. Gives the direction of the axes and, from
                             the still phase, the noise and the drift of the
                             position estimate, saved to data/drone_id.csv.
  identify_response(...)     a flight under the onboard controllers with a
                             velocity command step; fit_tau() gives the
                             response time of the velocity loop.
  design_gains(...)          position loop gains from that response time and
                             the chosen speed and damping; model_step_response()
                             predicts the step.
  fly(controller, ...)       take off under the onboard controller, hand the
                             horizontal axes to the controller, hold, step
                             the target, hold, land. Logs everything. Refuses
                             to start without data/drone_id.csv.
  handtest(controller, ...)  motors off, the controller's command against the
                             hand-moved position; kept for the sign check.

Plus save(), load_run() and step_numbers() for the analysis, and the status
and recovery helpers. Logs are csv files written with the stdlib csv module;
the analysis needs only numpy and matplotlib.

Terminal use, with no notebook holding the radio link (the drone accepts one
connection at a time):

    python crazyflie.py find                # list the drones in range, by label
    python crazyflie.py status --label 7    # battery, flow deck, supervisor flags
    python crazyflie.py flow --label 7      # raw flow sensor check, move the drone by hand
    python crazyflie.py recover --label 7   # clear a crashed state, then print status
    python crazyflie.py reboot --label 7    # soft reboot over the radio, for a locked drone
    python crazyflie.py charge --label 7    # while charging over USB, a line every 30 s
    python crazyflie.py config --label E7   # the address stored in the drone
    python crazyflie.py relabel --label E7 --new 2   # store label 2 in the drone, then power cycle

Every drone answers to its own radio address. The drones of the course keep
the factory channel and data rate and get the label on their sticker, a hex
digit from 1 to F, as the last byte of the five-byte address, so drone_uri(2)
is radio://0/80/2M/E7E7E7E702. The address is always given explicitly, by
label or as a URI; there is no default, because a wrong address commands
another group's drone.
"""
import csv
import logging
import math
import os
import re
import threading
import time
from contextlib import contextmanager
from threading import Event

import numpy as np

# The radio library is only needed to talk to a drone. Without it the file
# still imports, so the analysis (load_run, step_numbers) works in the plain
# simulation environment; connect(), handtest() and fly() then raise.
try:
    import cflib.crtp
    from cflib.crazyflie import Crazyflie
    from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
    from cflib.crazyflie.log import LogConfig
    from cflib.positioning.motion_commander import MotionCommander
    HAVE_CFLIB = True
except ImportError:
    HAVE_CFLIB = False


# Only surface errors from the cflib logging framework.
logging.basicConfig(level=logging.ERROR)

_drivers_ready = False


class DroneError(Exception):
    """A problem the student can fix: wrong label, drone off, dongle missing.

    In a notebook only the message is shown, not a traceback: IPython calls
    _render_traceback_ when an exception defines it.
    """

    def _render_traceback_(self):
        return ['\x1b[1;31m%s\x1b[0m: %s' % (type(self).__name__, self)]


def _need_cflib():
    if not HAVE_CFLIB:
        raise ImportError('cflib is not installed in this Python environment. Install the '
                          'hardware requirements (cfclient brings cflib) to talk to a drone.')


ADDRESS_PREFIX = 'E7E7E7E7'      # the first four bytes of every drone address in the course
FACTORY_ADDRESS = 'E7E7E7E7E7'   # a drone that has not been given a label yet
DEFAULT_CHANNEL = 80
DEFAULT_RATE = '2M'


def drone_uri(label, channel=DEFAULT_CHANNEL, rate=DEFAULT_RATE):
    """The URI of the drone with this label, the hex digit on its sticker (1 to F).

    The drones keep the factory channel (80) and data rate (2M), and the label
    is the last byte of the five-byte address: label 2 gives
    radio://0/80/2M/E7E7E7E702, label A gives radio://0/80/2M/E7E7E7E70A.
    Letters are passed as strings, LABEL = 'A'. A drone that still has the
    factory address E7E7E7E7E7 has the label 'E7'.
    """
    if label is None:
        raise DroneError("Enter the label of your drone, the hex digit on its sticker: "
                         "LABEL = 2, or LABEL = 'A' for a letter.")
    if isinstance(label, int):
        byte = label
    else:
        text = str(label).strip()
        try:
            byte = int(text, 16) if 1 <= len(text) <= 2 else -1
        except ValueError:
            byte = -1
    if not 1 <= byte <= 0xFF:
        raise DroneError("The label is the last byte of the address in hex digits: LABEL = 2, LABEL = 'A' "
                         "for a letter, or LABEL = 'E7' for a drone that still has the factory address.")
    return 'radio://0/%d/%s/%s%02X' % (channel, rate, ADDRESS_PREFIX, byte)


def resolve_uri(uri=None):
    """Check that a radio address was given. There is no default on purpose."""
    if not uri or not str(uri).startswith('radio://'):
        raise DroneError('No drone address. Enter the label of your drone at the top of the '
                         "notebook (LABEL = 2 gives URI = 'radio://0/80/2M/E7E7E7E702'); "
                         'drone_01_connect lists the drones in range.')
    return uri


def init():
    """Initialise the low level radio drivers once per process."""
    global _drivers_ready
    _need_cflib()
    if not _drivers_ready:
        cflib.crtp.init_drivers()
        _drivers_ready = True


_URI_PATTERN = re.compile(r'^radio://(\d+)/(\d+)/(250K|1M|2M)/([0-9A-Fa-f]{10})$')


def drone_label(uri):
    """The label of a drone from its URI: the last address byte in hex, '2', 'A', or 'E7' for the factory address."""
    m = _URI_PATTERN.match(uri or '')
    if not m:
        raise DroneError('not a radio URI: %r' % (uri,))
    return '%X' % int(m.group(4)[-2:], 16)


def _open_radio():
    """The shared Crazyradio of cflib, for pings outside a connection."""
    from cflib.crtp.radiodriver import RadioManager
    try:
        return RadioManager.open(0)
    except Exception as exc:
        raise DroneError('No Crazyradio dongle found, plug it in (%s)' % exc)


def ping(uri, tries=5):
    """True if the drone at this URI acknowledges a radio packet. Takes milliseconds."""
    from cflib.drivers.crazyradio import Crazyradio
    m = _URI_PATTERN.match(uri)
    if not m:
        raise DroneError('Not a radio address: %r (expected radio://0/80/2M/E7E7E7E7xx)' % uri)
    _devid, channel, rate, address = m.groups()
    rates = {'250K': Crazyradio.DR_250KPS, '1M': Crazyradio.DR_1MPS, '2M': Crazyradio.DR_2MPS}
    init()
    radio = _open_radio()
    try:
        radio.set_channel(int(channel))
        radio.set_data_rate(rates[rate])
        radio.set_address(list(bytes.fromhex(address)))
        for _ in range(tries):
            ack = radio.send_packet([0xFF])
            if ack is not None and ack.ack:
                return True
    finally:
        radio.close()
    return False


def require_drone(uri):
    """Raise a one-line DroneError if no drone answers at this URI."""
    if not ping(uri):
        raise DroneError('No drone answers at %s. Is the drone switched on (blue LEDs), is the '
                              'label right? The drones in range: crazyflie.find_drones() in a '
                              'notebook, python crazyflie.py find in the terminal.' % uri)


def find_drones(labels=range(1, 16), channel=DEFAULT_CHANNEL, rate=DEFAULT_RATE, factory=True, tries=3):
    """List the drones in range: ping the address of every label on the course channel.

    The labels are the hex digits 1 to F (see drone_uri); the factory address
    is tried as well and appears as label E7. A ping takes a few milliseconds,
    so the whole list takes about a second. Prints the drones that answer,
    each with the line to enter in the next cell, and returns them as
    {label byte: uri}. Other groups' drones answer too.
    """
    init()
    from cflib.drivers.crazyradio import Crazyradio
    rates = {'250K': Crazyradio.DR_250KPS, '1M': Crazyradio.DR_1MPS, '2M': Crazyradio.DR_2MPS}
    bytes_to_try = ([0xE7] if factory else []) + [int(n) for n in labels]
    candidates = [(b, '%s%02X' % (ADDRESS_PREFIX, b)) for b in bytes_to_try]
    radio = _open_radio()
    found = {}
    try:
        radio.set_channel(channel)
        radio.set_data_rate(rates[rate])
        for label, address in candidates:
            radio.set_address(list(bytes.fromhex(address)))
            for _ in range(tries):
                ack = radio.send_packet([0xFF])
                if ack is not None and ack.ack:
                    found[label] = 'radio://0/%d/%s/%s' % (channel, rate, address)
                    break
    finally:
        radio.close()
    if not found:
        print('no drone answers on channel %d: is the drone switched on (blue LEDs), is the '
              'dongle plugged in?' % channel)
    else:
        print('drones in range (label, URI, what to enter in the next cell):')
        for label in sorted(found):
            note = '   (factory address, no sticker yet)' if label == 0xE7 else ''
            entry = '%X' % label if label < 10 else "'%X'" % label
            print('  label %-2X   %s   LABEL = %s%s' % (label, found[label], entry, note))
    return found


def find_drone(label=None, address=None):
    """Search all radio channels for one drone and return its address as a URI.

    For a drone that does not show up in find_drones(), for example because its
    channel was changed. The drone is identified by the number on its label or,
    for a drone that still has the factory address, by address='E7E7E7E7E7'.
    All 126 channels and the three data rates are tried, which takes a few
    seconds. Raises DroneError when nothing answers.
    """
    init()
    if address is None:
        address = drone_uri(label).rsplit('/', 1)[1]
    found = [uri for uri, _comment in cflib.crtp.scan_interfaces(int(address, 16))
             if str(uri).startswith('radio://')]
    if not found:
        raise DroneError('No drone with address %s answers. Is the drone switched on (blue LEDs), '
                          'is the Crazyradio plugged in, is the label right? A drone that was never '
                          "given a label answers to find_drone(address='E7E7E7E7E7')." % address)
    if len(found) > 1:
        print('several answers, taking the first: %s' % ', '.join(found))
    print('found the drone with address %s at %s' % (address, found[0]))
    return found[0]


@contextmanager
def connect(uri=None, cache='./cache', estimator=2, controller=1):
    """Open the radio link to the assigned drone and close it cleanly afterwards.

    estimator 2 selects the Kalman filter, which the flow deck needs for the
    velocity and height estimates. controller 1 selects the onboard PID
    controller. On exit a stop setpoint is sent as a best-effort cleanup.
    """
    uri = resolve_uri(uri)
    init()
    require_drone(uri)
    cf = Crazyflie(rw_cache=cache)
    with SyncCrazyflie(uri, cf=cf) as scf:
        scf.cf.param.set_value('stabilizer.estimator', estimator)
        scf.cf.param.set_value('stabilizer.controller', controller)
        time.sleep(0.2)
        try:
            yield scf
        finally:
            scf.cf.commander.send_stop_setpoint()
            time.sleep(0.1)


# ---------------------------------------------------------------------------
# Checks on the ground
# ---------------------------------------------------------------------------

def _toc_has(scf, complete_name):
    """True if a log variable exists in the connected drone's log table."""
    try:
        return scf.cf.log.toc.get_element_by_complete_name(complete_name) is not None
    except Exception:
        return False


def _param_exists(scf, complete_name):
    """True if a parameter exists in the connected drone's parameter table."""
    try:
        return scf.cf.param.toc.get_element_by_complete_name(complete_name) is not None
    except Exception:
        return False


def flow_deck_attached(scf, timeout=5.0):
    """True if a Flow deck v2 is present (deck.bcFlow2 parameter)."""
    try:
        value = scf.cf.param.get_value('deck.bcFlow2', timeout=timeout)
        if value is not None:
            return bool(int(float(value)))
    except Exception:
        pass

    attached = Event()

    def _cb(_name, value_str):
        if int(float(value_str)):
            attached.set()

    scf.cf.param.add_update_callback(group='deck', name='bcFlow2', cb=_cb)
    try:
        scf.cf.param.request_param_update('deck.bcFlow2')
    except Exception:
        pass
    return attached.wait(timeout=timeout)


def reset_estimator(scf, settle=1.5):
    """Reset the Kalman estimator and let it settle. Run with the drone still and level."""
    scf.cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    scf.cf.param.set_value('kalman.resetEstimation', '0')
    time.sleep(settle)


# Power manager state codes reported by the pm.state log variable.
_PM_STATE = {0: 'on battery', 1: 'charging', 2: 'charged',
             3: 'low power', 4: 'shutdown'}

# Bit meanings of the supervisor.info log variable (recent firmware).
_SUPERVISOR_BITS = [
    (0, 'can be armed'), (1, 'is armed'), (2, 'auto arm'), (3, 'can fly'),
    (4, 'is flying'), (5, 'is tumbled'), (6, 'is locked'),
]


def _battery_percent(vbat):
    """Rough 1S LiPo state of charge from resting voltage, 3.0 V to 4.2 V."""
    return int(max(0, min(100, round((vbat - 3.0) / (4.2 - 3.0) * 100))))


def status(scf, timeout=5.0):
    """Print battery, flow deck and supervisor flags. Call on the ground.

    Reads one log sample with a bounded wait. If nothing arrives within
    `timeout` seconds the radio link has probably gone stale; a hint is printed
    instead of blocking forever.
    """
    candidates = [
        ('pm.vbat', 'float'), ('pm.state', 'uint8_t'),
        ('supervisor.info', 'uint16_t'), ('stateEstimate.z', 'float'),
    ]
    conf = LogConfig(name='status', period_in_ms=100)
    present = []
    for name, ctype in candidates:
        if _toc_has(scf, name):
            conf.add_variable(name, ctype)
            present.append(name)

    data = {}
    got = Event()

    def _cb(_timestamp, entry, _logconf):
        data.update(entry)
        got.set()

    received = False
    if present:
        conf.data_received_cb.add_callback(_cb)
        try:
            scf.cf.log.add_config(conf)
            conf.start()
            received = got.wait(timeout=timeout)
        except Exception as exc:
            print('status: could not start logging (%s)' % exc)
        finally:
            try:
                conf.stop()
                conf.delete()
            except Exception:
                pass

    if not received:
        print('Drone status: no response within %.0f s.' % timeout)
        print('  The radio link is probably stale: the drone may have powered off,')
        print('  gone to sleep, discharged, or moved out of range. Re-run the')
        print('  connection cell to reconnect, and check the drone is on and charged.')
        return

    print('Drone status of %s' % getattr(scf.cf, 'link_uri', 'unknown address'))
    if 'pm.vbat' in data:
        vbat = data['pm.vbat']
        state = _PM_STATE.get(int(data.get('pm.state', -1)), 'unknown')
        print('  battery      : %.2f V  (~%d%%)  %s' % (vbat, _battery_percent(vbat), state))
        if vbat < 3.2:
            print('  WARNING      : battery low, do not fly, recharge first')
    else:
        print('  battery      : unavailable')

    print('  flow deck    : %s' % ('attached' if flow_deck_attached(scf, timeout=1.0) else 'NOT attached'))

    if 'supervisor.info' in data:
        info = int(data['supervisor.info'])
        flags = {label: bool(info & (1 << bit)) for bit, label in _SUPERVISOR_BITS}
        print('  can be armed : %s' % ('yes' if flags['can be armed'] else 'no'))
        print('  is armed     : %s' % ('yes' if flags['is armed'] else 'no'))
        print('  is tumbled   : %s' % ('yes' if flags['is tumbled'] else 'no'))
        print('  is locked    : %s' % ('yes' if flags['is locked'] else 'no'))
        if flags['is locked']:
            print('  -> locked: python crazyflie.py recover, if that fails: reboot')
    else:
        print('  supervisor   : info not available on this firmware')

    if 'stateEstimate.z' in data:
        print('  height (est) : %.3f m' % data['stateEstimate.z'])


def identify(scf, blinks=3, period=0.6):
    """Blink the four LEDs of the connected drone, so a group sees which one it is.

    Uses the firmware parameter led.bitmask (firmware 2022.01 or newer): bit 7
    takes the LEDs away from the firmware, bits 0 to 5 switch them on. At the
    end the firmware gets its LEDs back. Returns False if the parameter is
    missing.
    """
    if not _param_exists(scf, 'led.bitmask'):
        print('this firmware has no led.bitmask parameter, so no blink; check the label instead')
        return False
    print('the four LEDs of your drone blink %d times' % blinks)
    try:
        for _ in range(blinks):
            scf.cf.param.set_value('led.bitmask', 0xBF)   # bit 7 plus all LEDs on
            time.sleep(period / 2)
            scf.cf.param.set_value('led.bitmask', 0x80)   # bit 7, all LEDs off
            time.sleep(period / 2)
    finally:
        scf.cf.param.set_value('led.bitmask', 0)          # back to the firmware
    return True


def watch_height(scf, duration=8.0, period=0.5):
    """Print the height measured by the flow deck for a while.

    Lift the drone slowly by hand and put it back: the numbers follow. The
    downward range sensor gives the raw height, the estimator's z is what the
    flight notebooks hold constant.
    """
    if not _toc_has(scf, 'range.zrange'):
        print('no height sensor found: the flow deck is not detected')
        return
    rows = []

    def _cb(_timestamp, entry, _logconf):
        rows.append(entry)

    conf = LogConfig(name='height', period_in_ms=100)
    conf.add_variable('range.zrange', 'uint16_t')
    conf.add_variable('stateEstimate.z', 'float')
    conf.data_received_cb.add_callback(_cb)
    scf.cf.log.add_config(conf)
    print('lift the drone slowly and put it back, %.0f s' % duration)
    conf.start()
    t0 = time.time()
    try:
        while time.time() - t0 < duration:
            time.sleep(period)
            if rows:
                r = rows[-1]
                print('  t = %4.1f s   height sensor %5.2f m   estimated z %5.2f m'
                      % (time.time() - t0, r['range.zrange'] / 1000.0, r['stateEstimate.z']))
    finally:
        conf.stop()
        try:
            conf.delete()
        except Exception:
            pass
    if not rows:
        print('no samples arrived, the link is stale; reconnect and rerun')


def charge(scf, period=30.0, timeout=5.0):
    """Print battery voltage and charge state every `period` s until charged.

    For charging over USB with the drone powered on. The power manager state
    ('charging' / 'charged') is the reliable full indicator; the voltage reads
    high during charging. Ctrl-C stops it early.
    """
    data = {}
    got = Event()

    def _cb(_timestamp, entry, _logconf):
        data.update(entry)
        data['wall'] = time.monotonic()
        got.set()

    conf = LogConfig(name='charge', period_in_ms=1000)
    conf.add_variable('pm.vbat', 'float')
    conf.add_variable('pm.state', 'uint8_t')
    conf.data_received_cb.add_callback(_cb)
    scf.cf.log.add_config(conf)
    conf.start()
    try:
        if not got.wait(timeout=timeout):
            print('charge: no response within %.0f s, is the drone on?' % timeout)
            return
        t0 = time.monotonic()
        while True:
            age = time.monotonic() - data['wall']
            if age > 10.0:
                print('charge: no samples for %.0f s, the radio link went stale.' % age)
                print('  Rerun the command to reconnect, or check the back left blue LED:')
                print('  steady means charged.')
                return
            state = int(data.get('pm.state', -1))
            print('%6.1f min   %.2f V   %s' % ((time.monotonic() - t0) / 60.0, data['pm.vbat'],
                                              _PM_STATE.get(state, 'unknown')))
            if state == 2:
                print('battery charged')
                return
            time.sleep(period)
    except KeyboardInterrupt:
        print('stopped')
    finally:
        try:
            conf.stop()
            conf.delete()
        except Exception:
            pass


def flow_check(scf, duration=6.0):
    """Report raw optical flow counts while the drone is moved by hand.

    Move the drone 20 to 30 cm above a textured surface during the test. A
    healthy flow sensor gives clearly nonzero motion.deltaX/Y counts while the
    drone translates. A dead or blocked one reads zero throughout, while the
    height sensor and the deck detection can still be fine; the horizontal
    position estimate is then unusable and flights turn erratic. Run this
    after any handling of the deck, after a crash, or when flights look odd.
    """
    rows = []

    def _cb(_timestamp, entry, _logconf):
        rows.append(entry)

    conf = LogConfig(name='flow', period_in_ms=100)
    conf.add_variable('motion.deltaX', 'int16_t')
    conf.add_variable('motion.deltaY', 'int16_t')
    conf.add_variable('range.zrange', 'uint16_t')
    conf.add_variable('motion.squal', 'uint8_t')     # surface quality: features the flow sensor sees
    conf.data_received_cb.add_callback(_cb)
    scf.cf.log.add_config(conf)
    disabled = _param_exists(scf, 'motion.disable') and int(float(scf.cf.param.get_value('motion.disable'))) != 0
    print('move the drone by hand over the desk, 20 to 30 cm up, for %.0f s' % duration)
    conf.start()
    time.sleep(duration)
    conf.stop()
    try:
        conf.delete()
    except Exception:
        pass

    if not rows:
        print('no samples arrived, the link is stale; reconnect and rerun')
        return
    deltas = [abs(r['motion.deltaX']) + abs(r['motion.deltaY']) for r in rows]
    nonzero = sum(1 for v in deltas if v > 0)
    heights = [r['range.zrange'] / 1000.0 for r in rows]
    squal = [r.get('motion.squal', 0) for r in rows]
    print('%d samples: %d with flow (%.0f%%), max |delta| %d counts, surface quality %d..%d, height %.2f..%.2f m'
          % (len(deltas), nonzero, 100.0 * nonzero / len(deltas), max(deltas), min(squal), max(squal),
             min(heights), max(heights)))
    if disabled:
        print('the parameter motion.disable is set: the firmware ignores the flow sensor.')
        print('Set it to 0 (cfclient, parameters tab, group motion) and rerun this check.')
    if nonzero > 0.3 * len(deltas) and max(deltas) > 5:
        print('flow ALIVE')
    elif max(squal) < 10:
        print('flow sees NO texture (surface quality near zero): the window is blocked or dirty, or the')
        print('surface is plain. Try over a newspaper or a patterned cloth; else power off, reseat the')
        print('deck, clean the two small windows on its underside, power cycle, rerun this check')
    else:
        print('flow DEAD or blocked: power off, reseat the deck, clean the two')
        print('small windows on its underside, power cycle, rerun this check')


def recover(scf):
    """Clear the supervisor 'locked' latch after a crash so the drone can arm again.

    The drone must be upright and stationary on the ground. Run status()
    afterwards; if it is still locked, reboot() or power cycle the drone.
    """
    scf.cf.supervisor.send_crash_recovery_request()
    time.sleep(0.5)


def reboot(uri=None, settle=3.0):
    """Soft reboot the drone over the radio. Run with NO other connection open."""
    uri = resolve_uri(uri)
    init()
    require_drone(uri)
    from cflib.utils.power_switch import PowerSwitch
    switch = PowerSwitch(uri)
    try:
        switch.stm_power_cycle()
    finally:
        switch.close()
    time.sleep(settle)


# ---------------------------------------------------------------------------
# System identification with the motors off: what the drone measures
# ---------------------------------------------------------------------------

# (phase name, seconds, instruction). %d is replaced by the distance in cm.
SENSOR_ID_PHASES = [
    ('still',   10.0, 'leave the drone on the desk, do not touch it'),
    ('lift',     5.0, 'LIFT the drone 20 to 30 cm and hold it'),
    ('forward',  6.0, 'move it about 30 cm FORWARD and hold it'),
    ('left',     6.0, 'move it about 30 cm to the LEFT and hold it'),
    ('down',     3.0, 'put the drone DOWN'),
]
SENSOR_ID_COLUMNS = ['t_ms', 'phase', 'x', 'y', 'z', 'vx', 'vy',          # the estimate
                     'flow_dx', 'flow_dy', 'squal', 'zrange_m']           # the flow deck's own readings
DRONE_ID_COMMENTS = {          # one sentence per value in data/drone_id.csv, ignored when reading
    'uri': 'radio address of the drone as the notebooks use it',
    'time': 'when the protocol was recorded',
    'rate_hz': 'samples per second in the recording',
    'still_s': 'length of the still phase used for noise and drift in s',
    'std_x_m': 'noise of the position estimate x while the drone lay still (standard deviation) in m',
    'std_y_m': 'the same for y',
    'std_vx_mps': 'noise of the velocity estimate vx while the drone lay still (standard deviation) in m/s',
    'std_vy_mps': 'the same for vy',
    'drift_x_m': 'change of x during the still phase from its first to its last second in m',
    'drift_y_m': 'the same for y',
    'x_sign': '+1 if moving the drone forward increased x and -1 if it decreased it',
    'y_sign': '+1 if moving the drone to the left increased y and -1 if it decreased it',
    'flow_counts_max': 'largest optical flow reading (|dx|+|dy| in counts) during the forward and left moves; 0 means the flow sensor saw no motion and the position estimate is not usable',
    'squal_median': 'surface quality reported by the flow sensor while the drone was held up (median); near 0 means it sees no texture',
}
CURRENT_PHASE = {'phase': ''}     # the phase identify_sensors() is in, readable from outside


def identify_sensors(uri=None, result='data/drone_id.csv', out=None, period_ms=50, phases=None):
    """Motors off: what the position estimate does when the drone is still and when it moves.

    The printout talks you through the phases of SENSOR_ID_PHASES (30 s): the
    drone still on the desk, lifted and held, moved forward and held, moved
    to the left and held, put down. The still phase gives the noise and the
    drift of the estimate, the two moves the direction of the axes. How far
    the hand moved is not used: it is not accurate enough to calibrate the
    estimate. Nothing is sent to the drone.

    Writes every sample to `out`, by default
    data/sensor_id_<drone label>_<HHMM>.csv (columns SENSOR_ID_COLUMNS), a
    file to compare runs and drones later that no notebook requires
    (out=False writes none). Analyses the samples and writes the results,
    twelve values, to `result` as a csv table of name, value and comment,
    which load_drone_id() reads back (None writes nothing). Returns
    (samples, results): the samples as a dict of arrays, the results as the
    dict that load_drone_id() returns.
    """
    _need_cflib()
    uri = resolve_uri(uri)
    if out is None:
        out = os.path.join(os.path.dirname(result) if result else 'data',
                           'sensor_id_%s_%s.csv' % (drone_label(uri), time.strftime('%H%M')))
    phases = phases or SENSOR_ID_PHASES
    rows = []
    current = CURRENT_PHASE
    current['phase'] = 'reset'

    flow = {}          # the newest reading of the flow deck, attached to every state row

    def _cb(ts, data, _conf):
        rows.append((ts, current['phase'], data['stateEstimate.x'], data['stateEstimate.y'],
                     data['stateEstimate.z'], data['stateEstimate.vx'], data['stateEstimate.vy'],
                     flow.get('motion.deltaX', 0), flow.get('motion.deltaY', 0),
                     flow.get('motion.squal', 0), flow.get('range.zrange', 0) / 1000.0))

    def _cb_flow(_ts, data, _conf):
        flow.update(data)

    conf = LogConfig(name='sensor_id', period_in_ms=period_ms)
    for name in ('stateEstimate.x', 'stateEstimate.y', 'stateEstimate.z',
                 'stateEstimate.vx', 'stateEstimate.vy'):
        conf.add_variable(name, 'float')
    conf.data_received_cb.add_callback(_cb)
    conf_flow = LogConfig(name='sensor_id_flow', period_in_ms=period_ms)   # a second block, the first is full
    for name, ctype in (('motion.deltaX', 'int16_t'), ('motion.deltaY', 'int16_t'),
                        ('motion.squal', 'uint8_t'), ('range.zrange', 'uint16_t')):
        conf_flow.add_variable(name, ctype)
    conf_flow.data_received_cb.add_callback(_cb_flow)

    total = sum(seconds for _name, seconds, _text in phases)
    with connect(uri) as scf:
        if not flow_deck_attached(scf):
            raise DroneError('No flow deck detected: without it there is no position estimate to measure.')
        print('resetting the estimator, the drone lies still and level on the start mark...')
        reset_estimator(scf)
        scf.cf.log.add_config(conf_flow)
        scf.cf.log.add_config(conf)
        conf_flow.start()
        conf.start()
        elapsed = 0.0
        for name, seconds, text in phases:
            current['phase'] = name
            print('[%2.0f / %.0f s] %s   ' % (elapsed, total, text), end='', flush=True)
            t_end = time.monotonic() + seconds
            remaining = int(round(seconds))
            while time.monotonic() < t_end:
                left = t_end - time.monotonic()
                if left <= remaining - 1 and remaining > 1:
                    remaining -= 1
                    print('%d ' % remaining, end='', flush=True)
                time.sleep(0.05)
            print()
            elapsed += seconds
        current['phase'] = 'done'
        conf.stop()
        conf_flow.stop()
        for c in (conf, conf_flow):
            try:
                c.delete()
            except Exception:
                pass

    if not rows:
        raise DroneError('no state samples arrived; is the drone on and the link alive?')
    samples = _samples_from_rows(rows)
    if out:
        os.makedirs(os.path.dirname(out) or '.', exist_ok=True)
        with open(out, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(SENSOR_ID_COLUMNS)
            w.writerows(rows)
        print('%d samples saved to %s' % (len(rows), out))
    results = analyse_sensor_id(samples, uri=uri)
    if result:
        save_drone_id(results, result)
        print('%d samples analysed, results written to %s' % (len(rows), result))
    print_drone_id(results)
    return samples, results


def _samples_from_rows(rows):
    """The logged rows as a dict of arrays, one per entry of SENSOR_ID_COLUMNS."""
    return {name: (np.asarray(col) if name == 'phase' else np.asarray(col, dtype=float))
            for name, col in zip(SENSOR_ID_COLUMNS, zip(*rows))}


def load_sensor_id(path='data/sensor_id.csv'):
    """Load the samples of identify_sensors(): arrays per column, the phase as strings."""
    with open(path, newline='', encoding='utf-8-sig') as f:
        reader = csv.reader(f)
        header = next(reader)
        cols = {name: [] for name in header}
        for row in reader:
            for name, cell in zip(header, row):
                cols[name].append(cell if name == 'phase' else float(cell))
    return {name: np.asarray(vals) for name, vals in cols.items()}


def analyse_sensor_id(samples, uri=None):
    """Noise and drift from the still phase, the direction of the axes from the two moves."""
    t = (samples['t_ms'] - samples['t_ms'][0]) / 1000.0
    phase = samples['phase']

    def held(name):
        """The last part of a phase, where the drone is held still."""
        m = phase == name
        if not m.any():
            raise DroneError('no samples in the phase %r of the sensor identification' % name)
        tt = t[m]
        window = min(2.0, 0.5 * (tt[-1] - tt[0]))
        return m & (t >= tt[-1] - window)

    def med(col, m):
        return float(np.median(samples[col][m]))

    still = phase == 'still'
    if not still.any():
        raise DroneError('no samples in the still phase of the sensor identification')
    ts = t[still]
    core = still & (t >= ts[0] + min(1.0, 0.25 * (ts[-1] - ts[0])))   # skip the first second
    first = core & (t <= t[core][0] + 1.0)
    last = core & (t >= t[core][-1] - 1.0)
    lift, forward, left = held('lift'), held('forward'), held('left')
    r = {
        'uri': uri or '',
        'time': time.strftime('%Y-%m-%d %H:%M'),
        'rate_hz': len(t) / (t[-1] - t[0]) if t[-1] > t[0] else 0.0,
        'still_s': float(t[core][-1] - t[core][0]),
        'std_x_m': float(np.std(samples['x'][core])),
        'std_y_m': float(np.std(samples['y'][core])),
        'std_vx_mps': float(np.std(samples['vx'][core])),
        'std_vy_mps': float(np.std(samples['vy'][core])),
        'drift_x_m': med('x', last) - med('x', first),
        'drift_y_m': med('y', last) - med('y', first),
        'x_sign': 1 if med('x', forward) > med('x', lift) else -1,
        'y_sign': 1 if med('y', left) > med('y', forward) else -1,
    }
    if 'flow_dx' in samples:                      # the flow deck's own readings
        moves = (phase == 'forward') | (phase == 'left')
        aloft = moves | (phase == 'lift')
        r['flow_counts_max'] = int(np.max(np.abs(samples['flow_dx'][moves]) + np.abs(samples['flow_dy'][moves])))
        r['squal_median'] = int(np.median(samples['squal'][aloft]))
    return {k: (round(v, 4) if isinstance(v, float) else v) for k, v in r.items()}


def save_drone_id(results, path='data/drone_id.csv'):
    """Write the results of identify_sensors() as a csv table: name, value and a comment.

    The comment is one sentence per value from DRONE_ID_COMMENTS; load_drone_id() ignores it.
    """
    os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['name', 'value', 'comment'])
        w.writerows((k, v, DRONE_ID_COMMENTS.get(k, '')) for k, v in results.items())


def _value(text):
    """A csv cell as int, float or string, whichever it reads as."""
    for kind in (int, float):
        try:
            return kind(text)
        except ValueError:
            pass
    return text


def load_drone_id(path='data/drone_id.csv'):
    """The results of identify_sensors() from its csv, or a DroneError telling to run part 2 first."""
    if not os.path.exists(path):
        raise DroneError('No sensor identification found (%s). Run drone_02_handtest first: the '
                         'flight notebooks need the axes and the noise of your drone.' % path)
    with open(path, newline='', encoding='utf-8-sig') as f:
        reader = csv.reader(f)
        next(reader, None)
        return {row[0]: _value(row[1]) for row in reader if len(row) >= 2}


def print_drone_id(r):
    """Print the results of identify_sensors() in words."""
    print('sensor identification of %s, %s, %.0f samples per second'
          % (r['uri'] or 'the drone', r['time'], r['rate_hz']))
    print('  still on the desk for %.0f s: noise std x %.3f m, y %.3f m, vx %.3f m/s, vy %.3f m/s'
          % (r['still_s'], r['std_x_m'], r['std_y_m'], r['std_vx_mps'], r['std_vy_mps']))
    print('                             drift x %+.3f m, y %+.3f m' % (r['drift_x_m'], r['drift_y_m']))
    word = {1: '+', -1: '-  REVERSED'}
    print('  moved by hand: forward gives x %s, left gives y %s' % (word[r['x_sign']], word[r['y_sign']]))
    if r['x_sign'] > 0 and r['y_sign'] > 0:
        print('  +x forward and +y left, as the controller of part 3 assumes')
    if 'flow_counts_max' in r:
        if r['flow_counts_max'] > 0:
            print('  flow sensor: up to %d counts during the moves, surface quality %d'
                  % (r['flow_counts_max'], r['squal_median']))
        else:
            print('  flow sensor: NO MOTION SEEN during the moves, surface quality %d. The position estimate'
                  % r['squal_median'])
            print('  above is not usable. Check the deck: python crazyflie.py flow --label <your label>')


def gain_limits(r, jitter=0.05):
    """Upper bounds for |Kp| and |Kd| from the noise of the estimate.

    The command jitter caused by the noise, |Kp| times the position noise and
    |Kd| times the velocity noise, is kept below `jitter` m/s. Returns a dict
    with Kp_max and Kd_max (magnitudes).
    """
    std_x = max(r['std_x_m'], r['std_y_m'], 1e-6)
    std_v = max(r['std_vx_mps'], r['std_vy_mps'], 1e-6)
    return {'Kp_max': jitter / std_x, 'Kd_max': jitter / std_v, 'jitter_mps': jitter}


# ---------------------------------------------------------------------------
# The radio configuration stored in the drone (for the instructors)
# ---------------------------------------------------------------------------

_RATE_CODES = {'250K': 0, '1M': 1, '2M': 2}


def read_config(scf, timeout=5.0):
    """Read the radio configuration block of the connected drone.

    The block lives in the EEPROM of the drone and sets the address the drone
    answers to after power-up. Returns the memory element; its `elements`
    dict holds 'radio_channel', 'radio_speed' (0, 1, 2 for 250K, 1M, 2M) and
    'radio_address'.
    """
    from cflib.crazyflie.mem import MemoryElement
    mems = scf.cf.mem.get_mems(MemoryElement.TYPE_I2C)
    if not mems:
        raise DroneError('the drone reports no configuration memory')
    done = Event()
    mems[0].update(lambda _mem: done.set())
    if not done.wait(timeout):
        raise DroneError('reading the configuration block timed out')
    return mems[0]


def config_uri(mem):
    """The URI stored in a configuration block read with read_config()."""
    e = mem.elements
    rate = {0: '250K', 1: '1M', 2: '2M'}.get(e.get('radio_speed'), '?')
    address = e.get('radio_address', int(FACTORY_ADDRESS, 16)) if e.get('version', 0) >= 1 \
        else int(FACTORY_ADDRESS, 16)
    return 'radio://0/%s/%s/%010X' % (e.get('radio_channel', '?'), rate, address)


def set_label(scf, label, channel=DEFAULT_CHANNEL, rate=DEFAULT_RATE, timeout=5.0):
    """Store a new label in the connected drone: address, channel and rate go into its EEPROM.

    The drone keeps answering at its old address until it is switched off and
    on again. Returns the new URI.
    """
    new_uri = drone_uri(label, channel, rate)
    mem = read_config(scf, timeout)
    old_uri = config_uri(mem)
    mem.elements['version'] = 1
    mem.elements['radio_channel'] = int(channel)
    mem.elements['radio_speed'] = _RATE_CODES[rate]
    mem.elements['radio_address'] = int(new_uri.rsplit('/', 1)[1], 16)
    mem.elements.setdefault('pitch_trim', 0.0)
    mem.elements.setdefault('roll_trim', 0.0)
    done = Event()
    mem.write_data(lambda _mem, _addr: done.set())
    if not done.wait(timeout):
        raise DroneError('writing the configuration block timed out')
    print('configuration written: %s -> %s' % (old_uri, new_uri))
    print('RESTART THE DRONE NOW: switch it off and on with its power button.')
    print('The new address takes effect at boot; until then the drone answers at the old one.')
    print('A reboot over the radio is not enough. Afterwards check with: python crazyflie.py find')
    return new_uri


# ---------------------------------------------------------------------------
# Logging during a flight
# ---------------------------------------------------------------------------

# Named groups of log variables. Each group becomes one LogConfig and must fit
# in a single 26 byte radio packet (at most six floats).
STATE_VARS = [
    ('stateEstimate.x', 'float'), ('stateEstimate.y', 'float'), ('stateEstimate.z', 'float'),
    ('stabilizer.roll', 'float'), ('stabilizer.pitch', 'float'), ('stabilizer.yaw', 'float'),
]
SENSOR_VARS = [
    ('range.zrange', 'uint16_t'),      # down facing height sensor, mm
    ('motion.deltaX', 'int16_t'),      # raw optical flow, counts
    ('motion.deltaY', 'int16_t'),
    ('stateEstimate.vx', 'float'), ('stateEstimate.vy', 'float'), ('stateEstimate.vz', 'float'),
]
POWER_VARS = [
    ('pm.vbat', 'float'),              # battery voltage, V
    ('motor.m1', 'uint16_t'),          # motor PWM outputs, 0..65535
    ('motor.m2', 'uint16_t'),
    ('motor.m3', 'uint16_t'),
    ('motor.m4', 'uint16_t'),
]
LAND_VARS = [
    ('supervisor.info', 'uint16_t'),   # supervisor bitfield, bit 6 is 'locked'
    ('acc.x', 'float'), ('acc.y', 'float'), ('acc.z', 'float'),  # acceleration in g
]

# Each group maps to (period in ms, variables). Battery and motor outputs change
# slowly, so the power group is logged at a lower rate to save radio bandwidth.
LOG_GROUPS = {
    'state': (20, STATE_VARS),
    'sensor': (20, SENSOR_VARS),
    'power': (50, POWER_VARS),
    'land': (20, LAND_VARS),
}


class FlightLog:
    """Collect the log groups during a flight, one csv per group on save."""

    def __init__(self, scf, groups=None):
        self.scf = scf
        self.groups = groups if groups is not None else LOG_GROUPS
        self.rows = {name: [] for name in self.groups}
        self._configs = {}
        for name, (period_ms, variables) in self.groups.items():
            conf = LogConfig(name=name, period_in_ms=period_ms)
            for var, ctype in variables:
                conf.add_variable(var, ctype)
            conf.data_received_cb.add_callback(self._make_cb(name))
            self._configs[name] = conf

    def _make_cb(self, group):
        def _cb(timestamp, data, _logconf):
            self.rows[group].append({'t_ms': timestamp, **data})
        return _cb

    def start(self):
        for conf in self._configs.values():
            self.scf.cf.log.add_config(conf)
            conf.start()

    def stop(self):
        for conf in self._configs.values():
            conf.stop()

    @property
    def n_rows(self):
        return {name: len(rows) for name, rows in self.rows.items()}

    def save_csv(self, prefix):
        """Write one csv per group as '<prefix>_<group>.csv'. Returns the paths."""
        written = []
        for name, (_period_ms, variables) in self.groups.items():
            path = '%s_%s.csv' % (prefix, name)
            fields = ['t_ms'] + [var for var, _ctype in variables]
            with open(path, 'w', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=fields)
                writer.writeheader()
                writer.writerows(self.rows[name])
            written.append(path)
        return written


# ---------------------------------------------------------------------------
# Abort by pressing Enter
# ---------------------------------------------------------------------------

def start_abort_listener():
    """Abort on Enter: returns an Event the flight loop checks every iteration.

    input() from a background thread makes the notebook show an input box (in
    VS Code at the top of the window); pressing Enter there sets the event.
    Headless runs have no stdin, then the listener simply stays inactive.
    """
    stop = threading.Event()

    def _listen():
        try:
            input('press Enter to ABORT and land: ')
        except Exception:
            return
        stop.set()

    threading.Thread(target=_listen, daemon=True).start()
    return stop


def bench_test_abort(duration=10.0):
    """Dummy loop with the same abort listener the flight uses, no drone.

    Run once per session before the first flight: press Enter in the input box
    and the loop must report the abort.
    """
    stop = start_abort_listener()
    t0 = time.monotonic()
    while time.monotonic() - t0 < duration and not stop.is_set():
        time.sleep(0.05)
    if stop.is_set():
        print('abort seen after %.1f s, the listener works in this frontend' % (time.monotonic() - t0))
    else:
        print('no abort within %.0f s; press Enter once now to close the input box' % duration)


# ---------------------------------------------------------------------------
# The two loops
# ---------------------------------------------------------------------------

def _clip(value, limit):
    return max(-limit, min(value, limit))


def _call_controller(controller, x, y, vx, vy, target, v_max):
    """Run the notebook controller once and clamp its output to +/- v_max."""
    cmd_x, cmd_y = controller(x, y, vx, vy, target[0], target[1])
    cmd_x, cmd_y = float(cmd_x), float(cmd_y)
    if not (math.isfinite(cmd_x) and math.isfinite(cmd_y)):
        raise ValueError('controller returned a non-finite command (%r, %r)' % (cmd_x, cmd_y))
    return _clip(cmd_x, v_max), _clip(cmd_y, v_max)


def handtest(controller, duration=30.0, target=(0.0, 0.0), v_max=0.3,
             out='data/handtest.csv', uri=None, period_ms=50):
    """Motors off: read the state while the drone is moved by hand, compute the command, log it.

    Nothing is ever sent to the drone: no arming, no setpoints. The estimator
    is reset at the start (hold the drone still and level), so the origin is
    where the drone is at that moment and `target` (0, 0) means "back to the
    start". Then for `duration` seconds the loop reads x, y, vx, vy at
    1000 / period_ms Hz, calls the controller, prints every tenth sample and
    stores all of them. The command must oppose the displacement: push the
    drone toward +x and the vx command must go negative.

    Writes `out` (columns t_ms, x, y, vx, vy, cmd_x, cmd_y) and returns the
    same data as a dict of numpy arrays.
    """
    _need_cflib()
    rows = []
    errors = []

    def _cb(ts, data, _conf):
        if errors:
            return
        try:
            x = data['stateEstimate.x']
            y = data['stateEstimate.y']
            vx = data['stateEstimate.vx']
            vy = data['stateEstimate.vy']
            cmd_x, cmd_y = _call_controller(controller, x, y, vx, vy, target, v_max)
        except Exception as exc:          # keep the exception, the loop below reports it
            errors.append(exc)
            return
        rows.append((ts, x, y, vx, vy, cmd_x, cmd_y))
        if len(rows) % 10 == 0:           # throttle the print to about 2 Hz
            def arrow(cmd):
                return '-> +' if cmd > 0.02 else '-> -' if cmd < -0.02 else '   0'
            elapsed = (ts - rows[0][0]) / 1000.0
            print('%4.1f / %.0f s   x %+.2f m  vx %+.2f %s     y %+.2f m  vy %+.2f %s'
                  % (elapsed, duration, x, cmd_x, arrow(cmd_x), y, cmd_y, arrow(cmd_y)))

    conf = LogConfig(name='handtest', period_in_ms=period_ms)
    for name in ('stateEstimate.x', 'stateEstimate.y', 'stateEstimate.vx', 'stateEstimate.vy'):
        conf.add_variable(name, 'float')
    conf.data_received_cb.add_callback(_cb)

    with connect(uri) as scf:
        if not flow_deck_attached(scf):
            print('WARNING: flow deck not detected, the state estimate will be meaningless')
        print('resetting estimator, hold the drone still and level...')
        reset_estimator(scf)
        print('GO: move the drone by hand. Push toward +x and the vx command should read -.')
        scf.cf.log.add_config(conf)
        conf.start()
        t0 = time.monotonic()
        while time.monotonic() - t0 < duration and not errors:
            time.sleep(0.1)
        conf.stop()
        try:
            conf.delete()
        except Exception:
            pass

    if errors:
        raise errors[0]
    if not rows:
        raise DroneError('no state samples arrived; is the drone on and the link alive?')

    os.makedirs(os.path.dirname(out) or '.', exist_ok=True)
    with open(out, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t_ms', 'x', 'y', 'vx', 'vy', 'cmd_x', 'cmd_y'])
        w.writerows(rows)
    print('%d samples saved to %s' % (len(rows), out))
    return load_csv(out)


def fly(controller, axes='x', step=0.5, z_hold=0.5, hold_s=4.0, step_s=8.0,
        loop_dt=0.02, v_max=0.3, fence=1.0, reset=None, uri=None, stop_event=None,
        drone_id='data/drone_id.csv'):
    """One flight: take off, hand the horizontal axes to `controller`, hold, step, hold, land.

    Sequence: flow deck check, estimator reset, arm, logging on, take off to
    z_hold under the onboard controller, 2 s to settle. The position at that
    moment becomes the target, so the handover itself commands nothing. The
    controller then holds for hold_s, the target steps by `step` metres along
    the active axes, and the loop runs another step_s. Then the command goes
    to zero and the drone lands.

    axes is 'x', 'y' or 'xy': the axes the controller commands, the other one
    is held at zero velocity. reset is an optional function called right
    before the handover, use it to zero the integrators of your controllers.

    The loop runs at 1 / loop_dt Hz on a fixed schedule, so a controller with
    a fixed time step (module.PIDController with deltaT=loop_dt) integrates
    correctly. Every command is clamped to +/- v_max here, whatever the
    controller returns.

    The loop aborts and lands by itself when Enter is pressed in the input box
    (or stop_event is set), when the drone leaves the `fence` box around the
    start, when state samples stop arriving for 0.5 s, when the battery sags
    below 3.1 V under load, or when the estimator's position variance blows up
    (a dead flow sensor). Interrupting the kernel once also lands the drone.

    The flight refuses to start without the sensor identification of part 2
    (`drone_id`, the file written by identify_sensors; None skips the check).

    Returns a dict for save(): loop rows, the FlightLog, the abort reason.
    """
    _need_cflib()
    if drone_id is not None:
        ident = load_drone_id(drone_id)
        if ident.get('flow_counts_max', 1) == 0:
            raise DroneError('The sensor identification saw no optical flow: this drone cannot hold its position. '
                             'Check the deck with python crazyflie.py flow, then run drone_02_handtest again.')
        if ident['x_sign'] < 0 or ident['y_sign'] < 0:
            print('WARNING: the sensor identification found a reversed axis (x sign %+d, y sign %+d); '
                  'the controller signs below assume +x forward and +y left'
                  % (ident['x_sign'], ident['y_sign']))
    uri = resolve_uri(uri)
    started = time.strftime('%H%M')          # goes into the names of the saved files
    own_listener = stop_event is None
    latest = {}   # newest state sample, written by the log callback
    rows = []     # one entry per loop iteration

    def _state_cb(ts, data, _conf):
        latest.update(data)
        latest['t_ms'] = ts
        latest['wall'] = time.monotonic()

    conf = LogConfig(name='pid_in', period_in_ms=20)
    for name in ('stateEstimate.x', 'stateEstimate.y', 'stateEstimate.vx', 'stateEstimate.vy',
                 'kalman.varPX', 'kalman.varPY'):
        conf.add_variable(name, 'float')   # six floats fill the 26 byte packet
    conf.data_received_cb.add_callback(_state_cb)

    aborted = None
    with connect(uri) as scf:
        if not flow_deck_attached(scf):
            raise DroneError('No flow deck detected, aborting before take-off.')

        reset_estimator(scf)
        scf.cf.supervisor.send_arming_request(True)
        time.sleep(1.0)

        flight = FlightLog(scf)
        flight.start()
        scf.cf.log.add_config(conf)
        conf.start()

        if own_listener:
            stop_event = start_abort_listener()   # Enter in the box = abort and land

        with MotionCommander(scf, default_height=z_hold) as mc:
            time.sleep(2.0)   # settle at hover under the onboard controller
            if 't_ms' not in latest:
                raise DroneError('no state samples arriving, aborting before handover')
            x0 = latest['stateEstimate.x']
            y0 = latest['stateEstimate.y']
            target = [x0, y0]
            if reset is not None:
                reset()
            print('handover at x %.2f m, y %.2f m, controller on %s' % (x0, y0, axes))

            try:
                t_start = time.monotonic()
                next_tick = t_start
                for phase, dur in (('hold', hold_s), ('step', step_s)):
                    if phase == 'step':
                        if 'x' in axes:
                            target[0] = x0 + step
                        if 'y' in axes:
                            target[1] = y0 + step
                        print('step to x %.2f m, y %.2f m' % (target[0], target[1]))
                    t_end = time.monotonic() + dur
                    while time.monotonic() < t_end:
                        now = time.monotonic()
                        if stop_event.is_set():
                            aborted = 'stopped by abort'
                            break
                        x = latest['stateEstimate.x']
                        y = latest['stateEstimate.y']
                        vx = latest['stateEstimate.vx']
                        vy = latest['stateEstimate.vy']
                        if abs(x - x0) > fence or abs(y - y0) > fence:
                            aborted = 'fence at x %.2f m, y %.2f m from start' % (x - x0, y - y0)
                            break
                        if now - latest['wall'] > 0.5:
                            aborted = 'state samples stale for %.2f s' % (now - latest['wall'])
                            break
                        power_rows = flight.rows['power']
                        if power_rows and power_rows[-1]['pm.vbat'] < 3.1:
                            aborted = 'battery %.2f V under load' % power_rows[-1]['pm.vbat']
                            break
                        var_px = latest.get('kalman.varPX', 0.0)
                        var_py = latest.get('kalman.varPY', 0.0)
                        if var_px > 0.05 or var_py > 0.05:
                            # healthy flights show 0.001 to 0.002 throughout
                            aborted = ('position variance %.3f, %.3f, estimator unhealthy '
                                       '(flow sensor?)' % (var_px, var_py))
                            break
                        try:
                            cmd_x, cmd_y = _call_controller(controller, x, y, vx, vy, target, v_max)
                        except Exception as exc:
                            aborted = 'controller raised %s: %s' % (type(exc).__name__, exc)
                            break
                        if 'x' not in axes:
                            cmd_x = 0.0
                        if 'y' not in axes:
                            cmd_y = 0.0
                        mc.start_linear_motion(cmd_x, cmd_y, 0.0)
                        rows.append((now - t_start, latest['t_ms'], 1 if phase == 'step' else 0,
                                     x, y, vx, vy, target[0], target[1], cmd_x, cmd_y,
                                     var_px, var_py))
                        next_tick += loop_dt            # fixed schedule, no drift
                        time.sleep(max(0.0, next_tick - time.monotonic()))
                    if aborted:
                        break
            except KeyboardInterrupt:
                # A kernel interrupt is the fallback abort: land below. A second
                # interrupt would cut the landing short, so press once.
                aborted = 'interrupted by user'
            finally:
                mc.start_linear_motion(0.0, 0.0, 0.0)
                time.sleep(0.5)
                land_gently(mc, from_height=z_hold)

        time.sleep(1.5)
        conf.stop()
        flight.stop()

    print('ABORTED: %s' % aborted if aborted else 'run completed')
    print('loop iterations:', len(rows), ' logged rows:', flight.n_rows)
    if own_listener and not stop_event.is_set():
        print('press Enter once to close the abort box')
    return {'rows': rows, 'flight': flight, 'aborted': aborted, 'axes': axes,
            'step': step, 'v_max': v_max, 'uri': uri, 'time': started}


def land_gently(mc, from_height, low=0.1, approach_velocity=0.3, final_velocity=0.2):
    """Land with a decisive final descent, not a slow crawl.

    Below about 5 cm the flow deck cannot sense height, so the estimate floors
    while the drone is already on the ground. A slow final descent then keeps
    the motors at hover thrust for a second or two and shoves the grounded
    drone around until the supervisor locks it. This descends quickly to `low`
    metres and then lands at final_velocity. Call it inside the
    MotionCommander block; the automatic land on block exit is then a no-op.
    """
    drop = from_height - low
    if drop > 0:
        mc.down(drop, velocity=approach_velocity)
    mc.land(velocity=final_velocity)


# ---------------------------------------------------------------------------
# System identification in flight: the response to a velocity command
# ---------------------------------------------------------------------------

def identify_response(step=0.2, step_s=1.5, settle_s=2.0, axis='x', z_hold=0.5, uri=None,
                      stop_event=None, drone_id='data/drone_id.csv'):
    """A flight under the onboard controllers with one velocity command step, no loop of ours.

    Take-off and hover as in fly(); then the velocity command along `axis`
    is 0 for 1.5 s, `step` m/s for `step_s` seconds and 0 again for
    `settle_s` seconds; then the drone lands. The drone moves about
    step * step_s metres. fit_tau() on the result gives the response time of
    the firmware's velocity loop. Returns the run dict of fly(), with
    run['identify'] holding step and step_s; save it with save(run, 'identify').
    """
    clock = {}

    def open_loop(_x, _y, _vx, _vy, x_target, y_target):
        target = x_target if axis == 'x' else y_target
        if 'start_target' not in clock:
            clock['start_target'] = target
        if abs(target - clock['start_target']) < 1e-9:
            return 0.0, 0.0                       # hold phase: zero velocity command
        if 't_step' not in clock:
            clock['t_step'] = time.monotonic()
        command = step if time.monotonic() - clock['t_step'] < step_s else 0.0
        return (command, 0.0) if axis == 'x' else (0.0, command)

    run = fly(open_loop, axes=axis, step=step * step_s, z_hold=z_hold, hold_s=1.5,
              step_s=step_s + settle_s, v_max=max(abs(step), 0.3), fence=1.0, uri=uri,
              stop_event=stop_event, drone_id=drone_id)
    run['identify'] = {'step': step, 'step_s': step_s, 'axis': axis}
    return run


def _pid_arrays(run):
    """The loop records of a run as arrays, from load_run() or straight from fly()."""
    if 'pid' in run:
        return run['pid']
    rows = np.asarray(run['rows'], dtype=float)
    return {name: rows[:, i] for i, name in enumerate(PID_COLUMNS)}


def fit_tau(run, axis='x', tau_range=(0.05, 2.0), delay_range=(0.0, 0.6)):
    """Response time of the velocity loop from an identify_response() run.

    The measured velocity after the command step is fitted with a first order
    response with a dead time, v(t) = v_cmd (1 - exp(-(t - delay) / tau)) for
    t > delay, by a grid search over tau and delay. Returns a dict with tau_s,
    delay_s, the time axis since the step and the measured, fitted and
    commanded velocity, for plotting.
    """
    pid = _pid_arrays(run)
    cmd = pid['cmd_%s' % axis]
    v = pid['v%s' % axis]
    t = pid['t_s']
    on = np.abs(cmd) > 1e-6
    if not on.any():
        raise DroneError('no velocity command in this run, nothing to fit')
    i0 = int(np.argmax(on))
    v_cmd = float(cmd[i0])
    i1 = i0 + int(np.argmax(~on[i0:])) if (~on[i0:]).any() else len(t)
    tt = t[i0:i1] - t[i0]
    vv = v[i0:i1] - v[i0]
    best = (np.inf, None, None)
    for delay in np.arange(delay_range[0], delay_range[1] + 1e-9, 0.02):
        for tau in np.arange(tau_range[0], tau_range[1] + 1e-9, 0.01):
            model = np.where(tt > delay, v_cmd * (1.0 - np.exp(-(tt - delay) / tau)), 0.0)
            sse = float(np.sum((vv - model) ** 2))
            if sse < best[0]:
                best = (sse, tau, delay)
    _sse, tau, delay = best
    fit = np.where(tt > delay, v_cmd * (1.0 - np.exp(-(tt - delay) / tau)), 0.0)
    print('velocity loop: tau %.2f s, dead time %.2f s (command %.2f m/s, %d samples)'
          % (tau, delay, v_cmd, len(tt)))
    return {'tau_s': float(tau), 'delay_s': float(delay), 'v_cmd': v_cmd, 't': tt,
            'v_meas': vv, 'v_fit': fit}


# ---------------------------------------------------------------------------
# From the response time to the gains, and the prediction
# ---------------------------------------------------------------------------

def closed_loop_parameters(Kp, Kd, tau):
    """Natural frequency and damping of the position loop closed around a first order velocity loop.

    With v_dot = (v_cmd - v) / tau and v_cmd = Kp (x - x0) + Kd v (our sign
    convention, Kp < 0), the position obeys
        tau x_ddot + (1 - Kd) x_dot - Kp (x - x0) = 0,
    so omega0 = sqrt(-Kp / tau) and zeta = (1 - Kd) / (2 omega0 tau).
    Returns (omega0 in rad/s, zeta).
    """
    if Kp >= 0:
        raise DroneError('Kp must be negative in this sign convention (error = value - setpoint)')
    omega0 = math.sqrt(-Kp / tau)
    zeta = (1.0 - Kd) / (2.0 * omega0 * tau)
    return omega0, zeta


def design_gains(tau, omega0, zeta=0.7):
    """Kp and Kd for a chosen natural frequency (rad/s) and damping, given the velocity loop's tau.

    Inverts closed_loop_parameters(): Kp = -omega0^2 tau, Kd = 1 - 2 zeta omega0 tau.
    A positive Kd means that the velocity loop alone damps more than asked
    for; then keep Kd at 0 and accept the larger damping, or raise omega0.
    """
    Kp = -omega0 ** 2 * tau
    Kd = 1.0 - 2.0 * zeta * omega0 * tau
    return Kp, Kd


def model_step_response(Kp, Kd, tau, Ki=0.0, delay=0.0, step=0.5, v_max=0.3, hold_s=4.0,
                        step_s=8.0, dt=0.02):
    """The step response of the position loop around a first order velocity loop with dead time.

    The same sequence as the flight: hold at 0 for hold_s, then the target
    steps to `step`. The command is clamped to +/- v_max as in the flight.
    Returns t (seconds since the step), x, v and the command.
    """
    n = int(round((hold_s + step_s) / dt))
    n_delay = int(round(delay / dt))
    t = np.arange(n) * dt - hold_s
    x = np.zeros(n)
    v = np.zeros(n)
    cmd = np.zeros(n)
    integral = 0.0
    for i in range(1, n):
        target = step if t[i] >= 0 else 0.0
        err = x[i - 1] - target
        integral += err * dt
        c = Kp * err + Ki * integral + Kd * v[i - 1]
        cmd[i] = max(-v_max, min(c, v_max))
        c_eff = cmd[i - n_delay] if i - n_delay >= 0 else 0.0
        v[i] = v[i - 1] + (c_eff - v[i - 1]) * dt / tau
        x[i] = x[i - 1] + v[i] * dt
    return t, x, v, cmd


# ---------------------------------------------------------------------------
# Saving and loading runs
# ---------------------------------------------------------------------------

PID_COLUMNS = ['t_s', 't_ms', 'step', 'x', 'y', 'vx', 'vy', 'target_x', 'target_y',
               'cmd_x', 'cmd_y', 'var_px', 'var_py']


def run_name(run, label):
    """The name of a saved run: drone label, time of the flight and the run label, e.g. 'A_1541_w20_z07'."""
    return '%s_%s_%s' % (drone_label(run['uri']), run['time'], label)


def save(run, label, data_dir='data'):
    """Save one run from fly() as data/flight_<name>_*.csv, print the settle check, return the name.

    The name is the drone label, the time of the flight and `label`, for
    example A_1541_w20_z07, so that files of several drones and flights can
    share a folder. The log groups save as
    flight_<name>_{state,sensor,power,land}.csv and the control loop as
    flight_<name>_pid.csv (state in, target, command out per iteration).
    The settle check is the largest position error over the last 3 s of the
    step; below 0.15 m counts as settled. The name is what load_run() takes.
    """
    name = run_name(run, label)
    prefix = os.path.join(data_dir, 'flight_%s' % name)
    pid_csv = '%s_pid.csv' % prefix
    os.makedirs(data_dir, exist_ok=True)
    if os.path.exists(pid_csv):
        print('WARNING: %s exists, overwriting it' % pid_csv)

    for path in run['flight'].save_csv(prefix):
        print('saved', path)
    with open(pid_csv, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(PID_COLUMNS)
        w.writerows(run['rows'])
    print('saved', pid_csv)

    if run['aborted']:
        print('this run was aborted (%s), numbers may be incomplete' % run['aborted'])
    step_numbers(load_run(name, data_dir), v_max=run['v_max'])
    return name


def load_csv(path):
    """Load one csv written here into a dict of numpy arrays keyed by column name.

    The stdlib csv reader keeps dotted column names (stateEstimate.x) intact.
    Blank cells become NaN.
    """
    with open(path, newline='', encoding='utf-8-sig') as f:
        reader = csv.reader(f)
        header = next(reader)
        cols = {name: [] for name in header}
        for row in reader:
            for name, cell in zip(header, row):
                try:
                    cols[name].append(float(cell))
                except ValueError:
                    cols[name].append(np.nan)
    return {name: np.asarray(vals) for name, vals in cols.items()}


def load_run(name, data_dir='data'):
    """Load a saved run by its name (what save() returned and saved_runs() lists).

    Returns a dict with 'label' (the name), 'pid' (dict of arrays, see
    PID_COLUMNS), the groups 'state', 'sensor', 'power', 'land' where their
    files exist, 'axes' (the commanded axes, found from the target columns)
    and 't_step' (loop time of the step, None for a run aborted before the
    step).
    """
    prefix = os.path.join(data_dir, 'flight_%s' % name)
    run = {'label': name, 'pid': load_csv('%s_pid.csv' % prefix)}
    for group in LOG_GROUPS:
        path = '%s_%s.csv' % (prefix, group)
        if os.path.exists(path):
            run[group] = load_csv(path)
    pid = run['pid']
    run['axes'] = ''.join(a for a in 'xy' if np.ptp(pid['target_%s' % a]) > 1e-6)
    in_step = pid['step'] > 0.5
    run['t_step'] = float(pid['t_s'][in_step][0]) if in_step.any() else None
    return run


def saved_runs(data_dir='data', match=None):
    """Names of the runs saved in data_dir, oldest first; `match` keeps only names containing it."""
    import glob
    paths = sorted(glob.glob(os.path.join(data_dir, 'flight_*_pid.csv')), key=os.path.getmtime)
    names = [os.path.basename(p)[len('flight_'):-len('_pid.csv')] for p in paths]
    return [n for n in names if match is None or match in n]


def step_numbers(run, v_max=0.3, crit=0.15, quiet=False):
    """Rise time, overshoot, settling error and clamp use of one run, per commanded axis.

    Rise time is the time after the step at which the position first passes
    90 percent of the step. Overshoot is the largest excursion beyond the new
    target. The settling error is the largest |target - position| over the
    last 3 s of the step phase; below `crit` counts as settled. Clamp use is
    the fraction of the step phase in which the command sat at +/- v_max.
    Returns a dict per axis and prints a summary unless quiet.
    """
    pid = run['pid']
    numbers = {}
    if run['t_step'] is None:
        if not quiet:
            print('%s: no step phase in this run (aborted early), nothing to judge' % run['label'])
        return numbers
    t = pid['t_s']
    in_step = pid['step'] > 0.5
    tail = in_step & (t >= t[-1] - 3.0)
    for a in run['axes']:
        target = pid['target_%s' % a]
        d = target[-1] - target[0]
        pos = (pid[a][in_step] - target[0]) * np.sign(d)
        ts = t[in_step] - run['t_step']
        hit = pos > 0.9 * abs(d)
        rise = float(ts[np.argmax(hit)]) if hit.any() else None
        overshoot = float(max(0.0, pos.max() - abs(d)))
        err = target[tail] - pid[a][tail]
        clamped = np.abs(pid['cmd_%s' % a][in_step]) >= v_max - 1e-3
        numbers[a] = {'rise_s': rise, 'overshoot_m': overshoot,
                      'settle_error_m': float(np.abs(err).max()),
                      'mean_error_m': float(err.mean()),
                      'clamp_fraction': float(clamped.mean()),
                      'settled': bool(np.abs(err).max() < crit)}
    if not quiet:
        print(run['label'])
        for a, n in numbers.items():
            rise = '%.2f s' % n['rise_s'] if n['rise_s'] is not None else 'not reached'
            print('  %s: rise to 90%% %s, overshoot %.3f m, last 3 s max |error| %.3f m '
                  '(mean %+.3f m), clamp %.0f%% of step'
                  % (a, rise, n['overshoot_m'], n['settle_error_m'], n['mean_error_m'],
                     100.0 * n['clamp_fraction']))
        print('  settle check (< %.2f m on %s): %s'
              % (crit, run['axes'], 'PASS' if all(n['settled'] for n in numbers.values()) else 'not yet'))
    return numbers


# ---------------------------------------------------------------------------
# Terminal use
# ---------------------------------------------------------------------------

if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description='Drone status and crash recovery from the terminal.')
    parser.add_argument('command', choices=['find', 'status', 'recover', 'reboot', 'charge', 'flow',
                                            'config', 'relabel'],
                        help='find: list the drones in range by label (all channels for one label with --label); '
                             'status: battery and readiness; recover: clear a crashed state, '
                             'then status; reboot: soft reboot over the radio (for a locked '
                             'drone); charge: report every 30 s while charging until charged; '
                             'flow: raw flow sensor check, move the drone by hand; '
                             'config: the address stored in the drone; '
                             'relabel: store the label given with --new in the drone')
    parser.add_argument('--label', default=None, help='the label of the drone, a hex digit 1 to F')
    parser.add_argument('--uri', default=None, help='the radio address, if not built from the label')
    parser.add_argument('--new', default=None, help='relabel: the new label to store in the drone')
    args = parser.parse_args()
    if args.command == 'relabel' and args.new is None:
        parser.error('relabel needs --new LABEL')

    try:
        uri = args.uri or (drone_uri(args.label) if args.label is not None else None)
        if args.command == 'find':
            if args.label is None:
                find_drones()
            else:
                find_drone(args.label)
        elif args.command == 'reboot':
            reboot(uri)
            print('rebooted, wait a few seconds before reconnecting')
        else:
            with connect(uri) as scf:
                if args.command == 'config':
                    print('stored in the drone: %s' % config_uri(read_config(scf)))
                elif args.command == 'relabel':
                    set_label(scf, args.new)
                elif args.command == 'charge':
                    charge(scf)
                elif args.command == 'flow':
                    flow_check(scf)
                else:
                    if args.command == 'recover':
                        recover(scf)
                    status(scf)
    except DroneError as exc:
        print('%s: %s' % (type(exc).__name__, exc))
        raise SystemExit(1)
