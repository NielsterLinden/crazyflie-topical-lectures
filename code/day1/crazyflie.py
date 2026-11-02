"""Real Crazyflie helpers for the Day 1 flight notebooks.

The simulation notebooks keep all code in the notebook. For the real drone the
radio link, logging, arming, take-off, landing and the safety checks are the
same for every group and are not the point of the exercise, so they live here.
The controller stays in the notebook: both flight loops take a function

    controller(x, y, vx, vy, x_target, y_target) -> (vx_fb, vy_fb)

that turns the measured horizontal position and velocity (metres, m/s, from the
onboard estimator) into a velocity, the feedback signal that sets the velocity
setpoint of the firmware. The height is held by the onboard controller. This
is the position loop of the velocity cascade of the simulator notebooks, with
the velocity and attitude loops running in the firmware.

The chain of the afternoon, each step feeding the next:

  identify_sensors(...)      motors off. A guided protocol: the drone still on
                             the desk, then moved by hand forward and to the
                             left. Gives the direction of the axes and, from
                             the still phase, the noise and the drift of the
                             position estimate. The numbers are recorded by
                             hand; a file is written only on request.
  handtest(controller, ...)  motors off, the controller's feedback against the
                             hand-moved position: the sign of the gains,
                             judged by sign_check().
  fly_path(moves, ...)       a path under the onboard controllers, forward,
                             left, turn, hold: the first flights, logged like
                             every other run.
  identify_response(...)     a flight under the onboard controllers with a
                             velocity setpoint step; fit_tau() gives the
                             response time and the dead time of the velocity loop.
  fly(controller, ...)       take off under the onboard controller, hand the
                             horizontal axes to the controller, hold, step
                             the target, hold, land. Logs everything.
  fly_model(controller, ...) the same flight on a first order model of the
                             velocity loop with the measured response time and
                             dead time: the prediction before a real flight,
                             saved like a real run.

Plus save(), load_run(), step_numbers(), run_table() and predict_step() for the analysis
(every run is saved with its settings, the gains among them), and the status
and recovery helpers. Logs are csv files written with the stdlib csv module;
the analysis needs only numpy and matplotlib.

Terminal use, with no notebook holding the radio link (the drone accepts one
connection at a time):

    python crazyflie.py find                # list the drones in range, on all channels, a few seconds
    python crazyflie.py find --channel 80   # one channel only, a second: the drones still on 80
    python crazyflie.py status --label 7    # battery, flow deck, supervisor flags
    python crazyflie.py flow --label 7      # raw flow sensor check, move the drone by hand
    python crazyflie.py recover --label 7   # clear a crashed state, then print status
    python crazyflie.py reboot --label 7    # soft reboot over the radio, for a locked drone
    python crazyflie.py charge --label 7    # while charging over USB, a line every 30 s
    python crazyflie.py config --label E7   # the address stored in the drone
    python crazyflie.py relabel --label E7 --new 2   # store label 2 (and channel 16) in the drone, then power cycle
    python crazyflie.py channel --label 2   # move drone 2 to the channel of its label, 16, then power cycle
    python crazyflie.py params --label A    # every parameter of the drone, the stored ones listed, csv in data/
    python crazyflie.py compare --files data/params_9.csv data/params_A.csv   # what differs between two drones

Every drone answers to its own radio address on its own channel. The label on
the sticker, a hex digit from 1 to B, is the last byte of the five-byte
address, and the channel is 2 + 7 times the label, so drone_uri(2) is
radio://0/16/2M/E7E7E7E702. A drone that still has the factory address
E7E7E7E7E7 sits on the factory channel 80; the data rate is the factory value
2M throughout. The address is always given explicitly, by label or as a URI;
there is no default, because a wrong address commands another group's drone.
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
FACTORY_CHANNEL = 80             # the channel of a drone that has not been given a label yet
CHANNEL_BASE, CHANNEL_STEP = 2, 7   # the channel of label n is 2 + 7 n
MAX_LABEL = 0xB                  # labels 1 to B give channels 9 to 79, inside the licence-free band
DEFAULT_RATE = '2M'


def _label_byte(label):
    """The last address byte of a label: 2, '2', 'A', or 'E7' for the factory address."""
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
    return byte


def drone_channel(label):
    """The radio channel of the drone with this label: 2 + 7 times the label, 80 for the factory address.

    Each drone of the course has a channel of its own, so that the groups in
    one room do not share a frequency. Channel n lies at 2400 + n MHz; the
    labels 1 to B give the channels 9 to 79, inside the licence-free band,
    which ends at channel 83.
    """
    byte = _label_byte(label)
    if byte == int(FACTORY_ADDRESS[-2:], 16):
        return FACTORY_CHANNEL
    if byte > MAX_LABEL:
        raise DroneError('The labels of the course run from 1 to B; label %X would need channel %d, '
                         'outside the band. A drone elsewhere is reached as drone_uri(label, channel=...).'
                         % (byte, CHANNEL_BASE + CHANNEL_STEP * byte))
    return CHANNEL_BASE + CHANNEL_STEP * byte


def drone_uri(label, channel=None, rate=DEFAULT_RATE):
    """The URI of the drone with this label, the hex digit on its sticker (1 to B).

    The label is the last byte of the five-byte address and sets the channel,
    2 + 7 times the label (see drone_channel): label 2 gives
    radio://0/16/2M/E7E7E7E702, label A gives radio://0/72/2M/E7E7E7E70A.
    Letters are passed as strings, LABEL = 'A'. A drone that still has the
    factory address E7E7E7E7E7 has the label 'E7' and sits on channel 80.
    An explicit `channel` reaches a drone that is not on the channel of its
    label, drone_uri(2, channel=80) for one that has not been moved yet.
    """
    byte = _label_byte(label)
    if channel is None:
        channel = drone_channel(byte)
    return 'radio://0/%d/%s/%s%02X' % (int(channel), rate, ADDRESS_PREFIX, byte)


def resolve_uri(uri=None):
    """Check that a radio address was given. There is no default on purpose."""
    if not uri or not str(uri).startswith('radio://'):
        raise DroneError('No drone address. Enter the label of your drone at the top of the '
                         "notebook (LABEL = 2 gives URI = 'radio://0/16/2M/E7E7E7E702'); "
                         'python crazyflie.py find lists the drones in range.')
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
        raise DroneError('Not a radio address: %r (expected radio://0/<channel>/2M/E7E7E7E7xx)' % uri)
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


def find_drones(labels=range(1, MAX_LABEL + 1), rate=DEFAULT_RATE, factory=True, channel=None):
    """List the drones in range: every label is looked for on all 126 radio channels.

    The dongle scans the channels for one address in a single request, so the
    twelve addresses (the labels 1 to B, see drone_uri, and the factory address,
    which appears as label E7) take two to three seconds at the course rate 2M.
    Prints each drone that answers with the channel it is on, and marks a drone
    that is not on the channel of its label (see drone_channel): it has not
    been moved yet, and python crazyflie.py channel --label N does that.
    Returns {label byte: uri}, the URI the drone answers to now. A single label,
    find_drones('A'), looks for that drone alone, in a fraction of a second.
    With `channel` given, the labels are pinged on that one channel only, which
    takes a second. Other groups' drones answer too.
    """
    init()
    from cflib.drivers.crazyradio import Crazyradio
    rates = {'250K': Crazyradio.DR_250KPS, '1M': Crazyradio.DR_1MPS, '2M': Crazyradio.DR_2MPS}
    if isinstance(labels, (int, str)):
        labels, factory = [labels], False
    bytes_to_try = ([int(FACTORY_ADDRESS[-2:], 16)] if factory else []) + [_label_byte(n) for n in labels]
    bytes_to_try = sorted(set(bytes_to_try), key=bytes_to_try.index)
    radio = _open_radio()
    found, notes, warnings = {}, {}, []
    try:
        radio.set_data_rate(rates[rate])
        for label in bytes_to_try:
            address = '%s%02X' % (ADDRESS_PREFIX, label)
            radio.set_address(list(bytes.fromhex(address)))
            if channel is None:
                channels = [int(c) for c in radio.scan_channels(0, 125, (0xFF,))]
            else:
                radio.set_channel(int(channel))
                acks = (radio.send_packet([0xFF]) for _ in range(3))
                channels = [int(channel)] if any(a is not None and a.ack for a in acks) else []
            if not channels:
                continue
            found[label] = 'radio://0/%d/%s/%s' % (channels[0], rate, address)
            if len(channels) > 1:
                notes[label] = 'answers on channels %s, taking the first' % ', '.join(map(str, channels))
            elif label == int(FACTORY_ADDRESS[-2:], 16):
                notes[label] = 'factory address, no sticker yet'
            elif label > MAX_LABEL:
                notes[label] = 'label outside the course scheme'
            elif channels[0] != drone_channel(label):
                notes[label] = 'NOT ON ITS CHANNEL, see below'
                warnings.append('WARNING: drone %X answers on channel %d, not on the channel of its label, %d: '
                                'drone_uri(%X) does not reach it.\n         python crazyflie.py channel --label %X '
                                'moves it there; switch the drone off and on afterwards.'
                                % (label, channels[0], drone_channel(label), label, label))
    finally:
        radio.close()
    if not found:
        where = 'on channel %d' % channel if channel is not None else 'on any channel'
        print('no drone answers %s: is the drone switched on (blue LEDs), is the '
              'dongle plugged in?' % where)
    else:
        print('drones in range (label, URI, what to enter in the next cell):')
        for label in sorted(found):
            entry = '%X' % label if label < 10 else "'%X'" % label
            note = '   (%s)' % notes[label] if label in notes else ''
            print('  label %-2X   %s   LABEL = %s%s' % (label, found[label], entry, note))
        for line in warnings:
            print(line)
    return found


def find_drone(label=None, address=None):
    """Search all radio channels for one drone and return its address as a URI.

    For a drone that does not show up in find_drones(), for example because it
    is not on the channel of its label. The drone is identified by the number on its label or,
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
        ('range.zrange', 'uint16_t'),
        ('stateEstimate.x', 'float'), ('stateEstimate.y', 'float'),
    ]
    conf = LogConfig(name='status', period_in_ms=100)
    present = []
    for name, ctype in candidates:
        if _toc_has(scf, name):
            conf.add_variable(name, ctype)
            present.append(name)

    data = {}
    track = []          # x, y over the watch period: the estimate must stay put on the ground
    got = Event()

    def _cb(_timestamp, entry, _logconf):
        data.update(entry)
        if 'stateEstimate.x' in entry:
            track.append((entry['stateEstimate.x'], entry['stateEstimate.y']))
        got.set()

    received = False
    if present:
        conf.data_received_cb.add_callback(_cb)
        try:
            scf.cf.log.add_config(conf)
            conf.start()
            received = got.wait(timeout=timeout)
            if received:
                time.sleep(2.0)             # watch the estimate: on the ground it must stay put
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
            print('  -> locked (tumble, crash or hard landing): crazyflie.recover(scf) in a notebook, '
                  'python crazyflie.py recover --label N in the terminal; if it stays locked, power cycle')
    else:
        print('  supervisor   : info not available on this firmware')

    if 'stateEstimate.z' in data:
        line = '  height (est) : %.3f m' % data['stateEstimate.z']
        if 'range.zrange' in data:
            z_range = data['range.zrange'] / 1000.0
            line += '   height sensor %.3f m' % z_range
            if abs(data['stateEstimate.z'] - z_range) > 0.10:
                line += ('\n  WARNING      : the estimate does not follow the height sensor, the flow deck is not\n'
                         '                 delivering. Switch the drone off and on, connect again, check again.')
        print(line)
    if len(track) > 5:
        moved = math.hypot(track[-1][0] - track[0][0], track[-1][1] - track[0][1])
        print('  estimate     : moved %.3f m in 2 s on the ground' % moved)
        if moved > 0.05:
            print('  WARNING      : the position estimate runs away while the drone lies still: the flow deck')
            print('                 is not delivering. Switch the drone off and on, connect again, check again.')


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
        return
    readings = [r['range.zrange'] for r in rows]
    apart = sum(abs(r['stateEstimate.z'] - r['range.zrange'] / 1000.0) > 0.10 for r in rows)
    if max(readings) == min(readings):
        print('the height sensor reading never changed. If you did lift the drone, the deck is not delivering:')
        print('switch the drone off and on, connect again')
    elif apart > len(rows) / 2:
        print('the estimate does not follow the height sensor: switch the drone off and on, connect again')
    else:
        print('the estimate follows the height sensor')


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
DRONE_ID_COMMENTS = {          # one sentence per value in the results csv of identify_sensors(), ignored when reading
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
    'zrange_lift_m': 'height from the range sensor while the drone was held up in m; below 0.05 means the sensor did not see the lift and the deck is not delivering',
    'z_lift_m': 'height estimate of the firmware while the drone was held up in m; it must agree with the range sensor',
}
CURRENT_PHASE = {'phase': ''}     # the phase identify_sensors() is in, readable from outside


def identify_sensors(uri=None, result=None, out=None, period_ms=50, phases=None):
    """Motors off: what the position estimate does when the drone is still and when it moves.

    The printout talks you through the phases of SENSOR_ID_PHASES (30 s): the
    drone still on the desk, lifted and held, moved forward and held, moved
    to the left and held, put down. The still phase gives the noise and the
    drift of the estimate, the two moves the direction of the axes. How far
    the hand moved is not used: it is not accurate enough to calibrate the
    estimate. Nothing is sent to the drone.

    Writes nothing unless asked: `out` is a path for every sample (columns
    SENSOR_ID_COLUMNS), `result` a path for the analysed values as a csv
    table of name, value and comment, which load_drone_id() reads back.
    Returns (samples, results): the samples as a dict of arrays, the results
    as the dict that load_drone_id() returns.
    """
    _need_cflib()
    uri = resolve_uri(uri)
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
    if 'zrange_m' in samples:
        r['zrange_lift_m'] = med('zrange_m', lift)
        r['z_lift_m'] = med('z', lift)
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
    """The results of identify_sensors() from its results csv, or a DroneError when there is none."""
    if not os.path.exists(path):
        raise DroneError('No sensor identification found (%s). Run identify_sensors() with a result path '
                         'first: this needs the axes and the noise of your drone.' % path)
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
    if abs(r['drift_x_m']) > 0.05 or abs(r['drift_y_m']) > 0.05:
        print('  the estimate moved %.2f m while the drone lay still: the flow deck is not delivering.'
              % math.hypot(r['drift_x_m'], r['drift_y_m']))
    word = {1: '+', -1: '-  REVERSED'}
    print('  moved by hand: forward gives x %s, left gives y %s' % (word[r['x_sign']], word[r['y_sign']]))
    if r['x_sign'] > 0 and r['y_sign'] > 0:
        print('  +x forward and +y left, as the code of parts 3 and 4 assumes')
    if 'flow_counts_max' in r:
        if r['flow_counts_max'] > 0:
            print('  flow sensor: up to %d counts during the moves, surface quality %d'
                  % (r['flow_counts_max'], r['squal_median']))
        else:
            print('  flow sensor: NO MOTION SEEN during the moves, surface quality %d. The position estimate'
                  % r['squal_median'])
            print('  above is not usable. Check the deck: python crazyflie.py flow --label <your label>')
    if 'zrange_lift_m' in r:
        print('  held up: height sensor %.2f m, estimate z %.2f m' % (r['zrange_lift_m'], r['z_lift_m']))
        if r['zrange_lift_m'] < 0.05:
            print('  the height sensor did NOT see the lift: the deck is not delivering. Switch the drone off')
            print('  and on, connect again and run the protocol again.')
        elif abs(r['z_lift_m'] - r['zrange_lift_m']) > 0.10:
            print('  the estimate does not follow the height sensor. Switch the drone off and on, connect')
            print('  again and run the protocol again.')


def gain_limits(r, jitter=0.05):
    """Upper bounds for |Kp| and |Kd| from the noise of the estimate.

    The jitter of the feedback caused by the noise, |Kp| times the position noise and
    |Kd| times the velocity noise, is kept below `jitter` m/s. Returns a dict
    with Kp_max and Kd_max (magnitudes).
    """
    std_x = max(r['std_x_m'], r['std_y_m'], 1e-6)
    std_v = max(r['std_vx_mps'], r['std_vy_mps'], 1e-6)
    return {'Kp_max': jitter / std_x, 'Kd_max': jitter / std_v, 'jitter_mps': jitter}


def hover_noise(run):
    """The spread of the estimate during the hover before the step of a flight, for gain_limits().

    The standard deviation of x, y, vx and vy over the loop records of the
    hold phase, the drone hovering at the start before the target moves. In
    an identify_response() flight the velocity setpoint is zero there, so the
    spread is the noise of the estimate plus a little drift. Returns a dict
    with 'std_x_m', 'std_y_m', 'std_vx_mps', 'std_vy_mps' and 'n', the number
    of records; gain_limits() takes it.
    """
    pid = _pid_arrays(run)
    hold = pid['step'] < 0.5
    if hold.sum() < 10:
        raise DroneError('only %d loop records in the hold phase of this run, no noise estimate' % hold.sum())
    return {'std_x_m': float(np.std(pid['x'][hold])), 'std_y_m': float(np.std(pid['y'][hold])),
            'std_vx_mps': float(np.std(pid['vx'][hold])), 'std_vy_mps': float(np.std(pid['vy'][hold])),
            'n': int(hold.sum())}


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


def set_label(scf, label, channel=None, rate=DEFAULT_RATE, timeout=5.0):
    """Store a new label in the connected drone: address, channel and rate go into its EEPROM.

    The channel is the one of the label (see drone_channel) unless given. The
    drone keeps answering at its old address and channel until it is switched
    off and on again. Returns the new URI.
    """
    if channel is None:
        channel = drone_channel(label)
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
    print('A reboot over the radio is not enough. Afterwards check with: python crazyflie.py find --label %s'
          % drone_label(new_uri))
    return new_uri


def set_channel(label, channel=None, timeout=5.0):
    """Move the drone with this label to the channel of its label (or to `channel`).

    The drone is looked for on its own channel first (then there is nothing to
    do), then on the factory channel 80, then on all channels. Its
    configuration block is written with the new channel, and the drone must be
    switched off and on afterwards. Returns the URI the drone will answer to.
    """
    new_uri = drone_uri(label, channel)
    if ping(new_uri):
        print('the drone with label %s already answers at %s: nothing to do' % (drone_label(new_uri), new_uri))
        return new_uri
    old_uri = drone_uri(label, FACTORY_CHANNEL)
    if old_uri != new_uri and ping(old_uri):
        print('found the drone with label %s on the factory channel: %s' % (drone_label(old_uri), old_uri))
    else:
        old_uri = find_drone(label)
    with connect(old_uri) as scf:
        return set_label(scf, label, channel=channel, timeout=timeout)


def dump_params(scf, out=None, timeout=5.0, quiet=False):
    """Read every parameter of the connected drone; for the persistent ones also the value stored in flash.

    A persistent parameter with a stored value is a setting somebody changed
    and saved in this drone; the list of those is printed. Returns a list of
    dicts (name, type, value, persistent, stored, default, stored_value),
    sorted by name, and writes them as csv to `out` when given. Takes a few
    seconds for the few hundred parameters. Note that connect() sets
    stabilizer.estimator and stabilizer.controller for the session; they show
    as current values, not as stored ones.
    """
    param = scf.cf.param
    rows = []
    for group in sorted(param.toc.toc):
        for name in sorted(param.toc.toc[group]):
            element = param.toc.toc[group][name]
            full = '%s.%s' % (group, name)
            row = {'name': full, 'type': element.ctype, 'value': param.get_value(full, timeout=timeout),
                   'persistent': bool(element.is_persistent()), 'stored': '', 'default': '', 'stored_value': ''}
            if row['persistent']:
                done, got = Event(), {}

                def cb(_name, state, got=got, done=done):
                    got['state'] = state
                    done.set()
                param.persistent_get_state(full, cb)
                state = got.get('state') if done.wait(timeout) else None
                if state is not None:
                    row['stored'] = bool(state.is_stored)
                    row['default'] = state.default_value
                    row['stored_value'] = state.stored_value if state.is_stored else ''
            rows.append(row)
    if out:
        os.makedirs(os.path.dirname(out) or '.', exist_ok=True)
        with open(out, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0]))
            w.writeheader()
            w.writerows(rows)
    if not quiet:
        stored = [r for r in rows if r['stored'] is True]
        print('%d parameters read, %d persistent' % (len(rows), sum(r['persistent'] for r in rows)))
        if stored:
            print('settings stored in this drone (persistent parameters with a saved value):')
            for r in stored:
                print('  %-28s stored %s   default %s   current %s' % (r['name'], r['stored_value'], r['default'], r['value']))
        else:
            print('no settings stored in this drone: every persistent parameter is at its default')
        if out:
            print('saved', out)
    return rows


def compare_params(file_a, file_b):
    """Print the parameters whose current or stored values differ between two dump_params() files."""
    def read(path):
        with open(path, newline='') as f:
            return {r['name']: r for r in csv.DictReader(f)}
    a, b = read(file_a), read(file_b)
    names = sorted(set(a) | set(b))
    diff = 0
    for n in names:
        if n not in a or n not in b:
            print('  %-28s only in %s' % (n, file_a if n in a else file_b)); diff += 1
            continue
        if a[n]['value'] != b[n]['value'] or a[n]['stored_value'] != b[n]['stored_value']:
            print('  %-28s %s: %s (stored %s)   %s: %s (stored %s)'
                  % (n, file_a, a[n]['value'], a[n]['stored_value'] or '-', file_b, b[n]['value'], b[n]['stored_value'] or '-'))
            diff += 1
    print('%d parameters compared, %d differ' % (len(names), diff))
    return diff


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
    try:
        fb_x, fb_y = controller(x, y, vx, vy, target[0], target[1])
    except TypeError as exc:
        raise DroneError('the controller failed: %s. Are Kp, Kd and Ki all numbers? None is only a placeholder.' % exc)
    fb_x, fb_y = float(fb_x), float(fb_y)
    if not (math.isfinite(fb_x) and math.isfinite(fb_y)):
        raise ValueError('controller returned a non-finite feedback (%r, %r)' % (fb_x, fb_y))
    return _clip(fb_x, v_max), _clip(fb_y, v_max)


def handtest(controller, duration=30.0, target=(0.0, 0.0), v_max=0.3,
             out=None, uri=None, period_ms=50):
    """Motors off: read the state while the drone is moved by hand, compute the feedback, log it.

    Nothing is ever sent to the drone: no arming, no setpoints. The estimator
    is reset at the start (hold the drone still and level), so the origin is
    where the drone is at that moment and `target` (0, 0) means "back to the
    start". Then for `duration` seconds the loop reads x, y, vx, vy at
    1000 / period_ms Hz, calls the controller, prints every tenth sample and
    stores all of them. The feedback must oppose the displacement: push the
    drone toward +x and the vx feedback must go negative.

    Returns the data as a dict of numpy arrays (t_ms, x, y, vx, vy, fb_x,
    fb_y) and writes them to `out` when a path is given. sign_check() on the
    result prints the verdict per axis.
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
            fb_x, fb_y = _call_controller(controller, x, y, vx, vy, target, v_max)
        except Exception as exc:          # keep the exception, the loop below reports it
            errors.append(exc)
            return
        rows.append((ts, x, y, vx, vy, fb_x, fb_y))
        if len(rows) % 10 == 0:           # throttle the print to about 2 Hz
            def arrow(fb):
                return '-> +' if fb > 0.02 else '-> -' if fb < -0.02 else '   0'
            elapsed = (ts - rows[0][0]) / 1000.0
            print('%4.1f / %.0f s   x %+.2f m  vx %+.2f %s     y %+.2f m  vy %+.2f %s'
                  % (elapsed, duration, x, fb_x, arrow(fb_x), y, fb_y, arrow(fb_y)))

    conf = LogConfig(name='handtest', period_in_ms=period_ms)
    for name in ('stateEstimate.x', 'stateEstimate.y', 'stateEstimate.vx', 'stateEstimate.vy'):
        conf.add_variable(name, 'float')
    conf.data_received_cb.add_callback(_cb)

    with connect(uri) as scf:
        if not flow_deck_attached(scf):
            print('WARNING: flow deck not detected, the state estimate will be meaningless')
        print('resetting estimator, hold the drone still and level...')
        reset_estimator(scf)
        print('GO: move the drone by hand. Push toward +x and the vx feedback should read -.')
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

    names = ['t_ms', 'x', 'y', 'vx', 'vy', 'fb_x', 'fb_y']
    d = {name: np.asarray(col, dtype=float) for name, col in zip(names, zip(*rows))}
    if out:
        os.makedirs(os.path.dirname(out) or '.', exist_ok=True)
        with open(out, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(names)
            w.writerows(rows)
        print('%d samples saved to %s' % (len(rows), out))
    else:
        print('%d samples' % len(rows))
    sign_check(d)
    return d


def sign_check(d, min_move=0.05, dead=0.02, pass_fraction=0.8, quiet=False):
    """Did the feedback oppose the displacement? The verdict of a handtest() record, per axis.

    Over the samples in which the drone was more than `min_move` from the
    start, the fraction in which the feedback pointed back toward the start
    (and was larger than `dead`). PASS from `pass_fraction`, FAIL below,
    'not tested' when the drone never got that far on the axis. Returns
    {axis: {'fraction', 'n', 'verdict'}} and prints one line per axis.
    """
    result = {}
    for a in 'xy':
        pos, fb = np.asarray(d[a], dtype=float), np.asarray(d['fb_%s' % a], dtype=float)
        away = np.abs(pos) > min_move
        n = int(away.sum())
        if n == 0:
            result[a] = {'fraction': None, 'n': 0, 'verdict': 'not tested'}
            line = 'the drone never moved more than %.0f cm along %s: not tested' % (100 * min_move, a)
        else:
            back = (np.sign(fb[away]) == -np.sign(pos[away])) & (np.abs(fb[away]) > dead)
            frac = float(back.mean())
            verdict = 'PASS' if frac >= pass_fraction else 'FAIL'
            result[a] = {'fraction': frac, 'n': n, 'verdict': verdict}
            line = ('the feedback pointed back to the start in %.0f%% of the %d samples beyond %.0f cm: %s'
                    % (100 * frac, n, 100 * min_move, verdict))
        if not quiet:
            print('  %s: %s' % (a, line))
    if not quiet and any(r['verdict'] == 'FAIL' for r in result.values()):
        print('  FAIL: the feedback follows the displacement; in flight this loop would push the drone away '
              'from its target. Flip the sign of the gain and test again.')
    return result


def _check_deck_on_ground(flight, latest, timeout=2.0, tolerance=0.10, still_s=1.5, still_m=0.05, still_v=0.05):
    """Before arming: the height estimate must agree with the range sensor, and the position estimate
    must stay put for still_s seconds with the motors off (moved less than still_m, velocity below
    still_v); otherwise the deck is not delivering. The window doubles as the settle time after the
    estimator reset."""
    t_end = time.monotonic() + timeout
    while time.monotonic() < t_end and not (flight.rows['sensor'] and flight.rows['state']):
        time.sleep(0.05)
    if not (flight.rows['sensor'] and flight.rows['state']):
        raise DroneError('no log samples arriving, aborting before arming; is the link alive?')
    z_range = flight.rows['sensor'][-1]['range.zrange'] / 1000.0
    z_est = flight.rows['state'][-1]['stateEstimate.z']
    if abs(z_est - z_range) > tolerance:
        raise DroneError('On the ground the height estimate (%.2f m) does not follow the height sensor (%.2f m): '
                         'the flow deck is not delivering. Switch the drone off and on, then try again.'
                         % (z_est, z_range))
    t_end = time.monotonic() + timeout
    while time.monotonic() < t_end and 'stateEstimate.x' not in latest:
        time.sleep(0.05)
    if 'stateEstimate.x' not in latest:
        raise DroneError('no state samples arriving, aborting before arming; is the link alive?')
    x0, y0 = latest['stateEstimate.x'], latest['stateEstimate.y']
    time.sleep(still_s)
    moved = math.hypot(latest['stateEstimate.x'] - x0, latest['stateEstimate.y'] - y0)
    speed = math.hypot(latest['stateEstimate.vx'], latest['stateEstimate.vy'])
    if moved > still_m or speed > still_v:
        raise DroneError('On the ground the position estimate moved %.2f m in %.1f s and reports %.2f m/s with '
                         'the motors off: the flow deck is not delivering. Switch the drone off and on, then '
                         'try again.' % (moved, still_s, speed))


def fly(controller, axes='x', step=0.5, z_hold=0.5, hold_s=4.0, step_s=8.0,
        loop_dt=0.02, v_max=0.3, fence=1.0, reset=None, uri=None, stop_event=None,
        drone_id=None, hover_s=2.0, info=None):
    """One flight: take off, hand the horizontal axes to `controller`, hold, step, hold, land.

    Sequence: flow deck check, logging on, estimator reset with 1.5 s in which
    the estimate must stay put, arm (waits for the armed flag, at most 1 s),
    take off to z_hold under the onboard controller, hover_s to settle. The
    printout gives the elapsed time at each stage. The position at that
    moment becomes the target, so the handover itself sends no feedback. The
    controller then holds for hold_s, the target steps by `step` metres along
    the active axes, and the loop runs another step_s. Then the feedback goes
    to zero and the drone lands.

    axes is 'x', 'y' or 'xy': the axes the controller acts on, the other one
    is held at zero velocity. reset is an optional function called right
    before the handover, use it to zero the integrators of your controllers.

    The loop runs at 1 / loop_dt Hz on a fixed schedule, so a controller with
    a fixed time step (module.PIDController with deltaT=loop_dt) integrates
    correctly. Every feedback value is clamped to +/- v_max here, whatever the
    controller returns.

    The loop aborts and lands by itself when Enter is pressed in the input box
    (or stop_event is set), when the drone leaves the `fence` box around the
    start, when state samples stop arriving for 0.5 s, when the battery sags
    below 3.1 V under load, when the estimator's position variance blows up
    (a dead flow sensor), or when the height sensor reading freezes for 1 s
    (the deck stopped delivering). Before arming, the height estimate must
    agree with the height sensor. Interrupting the kernel once also lands the
    drone.

    With `drone_id`, the path of a file written by identify_sensors(), the
    flight refuses a drone whose flow deck did not deliver there and warns
    about a reversed axis; by default (None) there is no such check.

    `info` is a dict of settings of this run that save() writes next to the
    data, for example the gains of the controller, which fly() cannot know.

    Returns a dict for save(): loop rows, the FlightLog, the abort reason,
    the settings.
    """
    _need_cflib()
    if drone_id is not None:
        ident = load_drone_id(drone_id)
        if ident.get('flow_counts_max', 1) == 0 or ident.get('zrange_lift_m', 1.0) < 0.05:
            raise DroneError('The sensor identification shows a flow deck that was not delivering (no optical flow '
                             'or no height): the drone cannot hold its position. Switch it off and on, check with '
                             'python crazyflie.py flow, then repeat the sensor identification.')
        if ident['x_sign'] < 0 or ident['y_sign'] < 0:
            print('WARNING: the sensor identification found a reversed axis (x sign %+d, y sign %+d); '
                  'the controller signs below assume +x forward and +y left'
                  % (ident['x_sign'], ident['y_sign']))
    uri = resolve_uri(uri)
    started = time.strftime('%H%M')          # goes into the names of the saved files
    t_cell = time.monotonic()

    def stage(text):
        print('[%4.1f s] %s' % (time.monotonic() - t_cell, text))
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

        flight = FlightLog(scf)
        flight.start()
        scf.cf.log.add_config(conf)
        conf.start()
        stage('connected, estimator reset, 1.5 s watching the estimate on the ground')
        reset_estimator(scf, settle=0.0)
        _check_deck_on_ground(flight, latest)   # settle time and deck check in one window
        scf.cf.supervisor.send_arming_request(True)
        t_arm = time.monotonic()
        while time.monotonic() - t_arm < 1.0:   # wait for the armed flag instead of a fixed second
            land_rows = flight.rows['land']
            if land_rows and int(land_rows[-1]['supervisor.info']) & (1 << 1):
                break
            time.sleep(0.05)
        stage('deck ok, armed, take-off to %.2f m' % z_hold)

        if own_listener:
            stop_event = start_abort_listener()   # Enter in the box = abort and land

        with MotionCommander(scf, default_height=z_hold) as mc:
            stage('in the air, %.1f s to settle at hover' % hover_s)
            time.sleep(hover_s)   # settle at hover under the onboard controller
            if 't_ms' not in latest:
                raise DroneError('no state samples arriving, aborting before handover')
            x0 = latest['stateEstimate.x']
            y0 = latest['stateEstimate.y']
            target = [x0, y0]
            if reset is not None:
                reset()
            stage('handover at x %.2f m, y %.2f m, controller on %s' % (x0, y0, axes))
            sensor_rows = flight.rows['sensor']
            if sensor_rows and sensor_rows[-1]['range.zrange'] / 1000.0 < 0.5 * z_hold:
                aborted = ('height sensor reads %.2f m after the take-off to %.2f m: the deck is not delivering'
                           % (sensor_rows[-1]['range.zrange'] / 1000.0, z_hold))

            try:
                t_start = time.monotonic()
                next_tick = t_start
                for phase, dur in (('hold', hold_s), ('step', step_s)):
                    if aborted:
                        break
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
                        sensor_rows = flight.rows['sensor']
                        if len(sensor_rows) >= 50:
                            recent = [s['range.zrange'] for s in sensor_rows[-50:]]   # the last second
                            if max(recent) == min(recent):
                                aborted = ('height sensor frozen at %d mm for 1 s, the deck stopped delivering'
                                           % recent[-1])
                                break
                        var_px = latest.get('kalman.varPX', 0.0)
                        var_py = latest.get('kalman.varPY', 0.0)
                        if var_px > 0.05 or var_py > 0.05:
                            # healthy flights show 0.001 to 0.002 throughout
                            aborted = ('position variance %.3f, %.3f, estimator unhealthy '
                                       '(flow sensor?)' % (var_px, var_py))
                            break
                        try:
                            fb_x, fb_y = _call_controller(controller, x, y, vx, vy, target, v_max)
                        except Exception as exc:
                            aborted = 'controller raised %s: %s' % (type(exc).__name__, exc)
                            break
                        if 'x' not in axes:
                            fb_x = 0.0
                        if 'y' not in axes:
                            fb_y = 0.0
                        mc.start_linear_motion(fb_x, fb_y, 0.0)
                        rows.append((now - t_start, latest['t_ms'], 1 if phase == 'step' else 0,
                                     x, y, vx, vy, target[0], target[1], fb_x, fb_y,
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
        print('landed. Press Enter once to close the abort box; the flight is over, Enter does nothing else now')
    return {'rows': rows, 'flight': flight, 'aborted': aborted, 'axes': axes,
            'step': step, 'v_max': v_max, 'uri': uri, 'time': started, 'drone': drone_label(uri),
            'info': dict(info or {})}


def fly_path(moves, uri=None, z=0.5, speed=0.3, turn_rate=45.0, hold_s=2.0, loop_dt=0.02, fence=1.5,
             stop_event=None, hover_s=2.0, info=None):
    """Fly a simple path under the drone's own controllers, logged like a flight of fly().

    `moves` is a list of (what, how much): ('forward', 0.5), ('back', 0.5),
    ('left', 0.5), ('right', 0.5) in metres in the frame of the drone,
    ('up', 0.2) and ('down', 0.2) in metres, ('turn', 90) in degrees, positive
    to the left, ('hold', 2.0) in seconds. The drone takes off to `z`, hovers
    hover_s, flies the moves at `speed` m/s and turn_rate deg/s, holds hold_s
    and lands. All control is on board: the MotionCommander sends velocity
    setpoints, the position loop of the firmware follows them.

    The same checks as in fly(): the deck before arming, the fence around the
    start, stale samples, battery, estimator variance, a frozen height sensor;
    Enter in the input box (or stop_event) lands the drone. Returns the run
    dict of fly(): the planned position in the target columns, the velocity
    setpoint sent, in the frame of the room, in the fb columns, the 'step'
    column 0 during the first second of hover and 1 from the first move on,
    so that save(), load_run() and the plots work unchanged.
    """
    _need_cflib()
    uri = resolve_uri(uri)
    started = time.strftime('%H%M')
    t_cell = time.monotonic()

    def stage(text):
        print('[%4.1f s] %s' % (time.monotonic() - t_cell, text))

    for what, _amount in moves:
        if what not in ('forward', 'back', 'left', 'right', 'up', 'down', 'turn', 'hold'):
            raise DroneError('unknown move %r; use forward, back, left, right, up, down, turn or hold' % (what,))
    own_listener = stop_event is None
    latest = {}
    rows = []

    def _state_cb(ts, data, _conf):
        latest.update(data)
        latest['t_ms'] = ts
        latest['wall'] = time.monotonic()

    conf = LogConfig(name='path_in', period_in_ms=20)
    for name in ('stateEstimate.x', 'stateEstimate.y', 'stateEstimate.vx', 'stateEstimate.vy',
                 'kalman.varPX', 'kalman.varPY'):
        conf.add_variable(name, 'float')
    conf.data_received_cb.add_callback(_state_cb)

    aborted = None
    with connect(uri) as scf:
        if not flow_deck_attached(scf):
            raise DroneError('No flow deck detected, aborting before take-off.')
        flight = FlightLog(scf)
        flight.start()
        scf.cf.log.add_config(conf)
        conf.start()
        stage('connected, estimator reset, 1.5 s watching the estimate on the ground')
        reset_estimator(scf, settle=0.0)
        _check_deck_on_ground(flight, latest)
        scf.cf.supervisor.send_arming_request(True)
        t_arm = time.monotonic()
        while time.monotonic() - t_arm < 1.0:
            land_rows = flight.rows['land']
            if land_rows and int(land_rows[-1]['supervisor.info']) & (1 << 1):
                break
            time.sleep(0.05)
        stage('deck ok, armed, take-off to %.2f m' % z)
        if own_listener:
            stop_event = start_abort_listener()

        with MotionCommander(scf, default_height=z) as mc:
            stage('in the air, %.1f s to settle at hover' % hover_s)
            time.sleep(hover_s)
            if 't_ms' not in latest:
                raise DroneError('no state samples arriving, aborting before the path')
            x0, y0 = latest['stateEstimate.x'], latest['stateEstimate.y']
            sensor_rows = flight.rows['sensor']
            if sensor_rows and sensor_rows[-1]['range.zrange'] / 1000.0 < 0.5 * z:
                aborted = ('height sensor reads %.2f m after the take-off to %.2f m: the deck is not delivering'
                           % (sensor_rows[-1]['range.zrange'] / 1000.0, z))
            heading = 0.0                       # the drone's forward at take-off is +x of the room
            planned = [x0, y0]
            segments = [('hold', 1.0, 0)] + [(what, amount, 1) for what, amount in moves] + [('hold', hold_s, 1)]
            stage('path starts at x %.2f m, y %.2f m: %s' % (x0, y0, ', '.join('%s %g' % m for m in moves)))
            try:
                t_start = time.monotonic()
                next_tick = t_start
                for what, amount, flag in segments:
                    if aborted:
                        break
                    body_v = (0.0, 0.0)
                    v_z = 0.0
                    yaw_rate = 0.0
                    if what in ('forward', 'back', 'left', 'right'):
                        sign = 1.0 if what in ('forward', 'left') else -1.0
                        body_v = (sign * speed, 0.0) if what in ('forward', 'back') else (0.0, sign * speed)
                        duration = abs(amount) / speed
                    elif what in ('up', 'down'):
                        v_z = (1.0 if what == 'up' else -1.0) * min(speed, 0.2)
                        duration = abs(amount) / abs(v_z)
                    elif what == 'turn':
                        yaw_rate = turn_rate if amount >= 0 else -turn_rate
                        duration = abs(amount) / turn_rate
                    else:
                        duration = float(amount)
                    if what == 'turn':
                        (mc.start_turn_left if amount >= 0 else mc.start_turn_right)(turn_rate)
                    else:
                        mc.start_linear_motion(body_v[0], body_v[1], v_z)
                    t_end = time.monotonic() + duration
                    while time.monotonic() < t_end:
                        now = time.monotonic()
                        if stop_event.is_set():
                            aborted = 'stopped by abort'
                            break
                        x, y = latest['stateEstimate.x'], latest['stateEstimate.y']
                        vx, vy = latest['stateEstimate.vx'], latest['stateEstimate.vy']
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
                        sensor_rows = flight.rows['sensor']
                        if len(sensor_rows) >= 50:
                            recent = [s['range.zrange'] for s in sensor_rows[-50:]]
                            if max(recent) == min(recent):
                                aborted = 'height sensor frozen at %d mm for 1 s, the deck stopped delivering' % recent[-1]
                                break
                        var_px = latest.get('kalman.varPX', 0.0)
                        var_py = latest.get('kalman.varPY', 0.0)
                        if var_px > 0.05 or var_py > 0.05:
                            aborted = 'position variance %.3f, %.3f, estimator unhealthy (flow sensor?)' % (var_px, var_py)
                            break
                        c, s_ = math.cos(heading), math.sin(heading)
                        fb_x = c * body_v[0] - s_ * body_v[1]       # the setpoint in the frame of the room
                        fb_y = s_ * body_v[0] + c * body_v[1]
                        rows.append((now - t_start, latest['t_ms'], flag, x, y, vx, vy, planned[0], planned[1],
                                     fb_x, fb_y, var_px, var_py))
                        planned[0] += fb_x * loop_dt
                        planned[1] += fb_y * loop_dt
                        heading += math.radians(yaw_rate) * loop_dt
                        next_tick += loop_dt
                        time.sleep(max(0.0, next_tick - time.monotonic()))
                    if aborted:
                        break
            except KeyboardInterrupt:
                aborted = 'interrupted by user'
            finally:
                mc.start_linear_motion(0.0, 0.0, 0.0)
                time.sleep(0.5)
                land_gently(mc, from_height=z)

        time.sleep(1.5)
        conf.stop()
        flight.stop()

    print('ABORTED: %s' % aborted if aborted else 'path completed')
    print('loop iterations:', len(rows), ' logged rows:', flight.n_rows)
    if own_listener and not stop_event.is_set():
        print('landed. Press Enter once to close the abort box; the flight is over, Enter does nothing else now')
    return {'rows': rows, 'flight': flight, 'aborted': aborted, 'axes': 'xy', 'step': 0.0, 'v_max': speed,
            'uri': uri, 'time': started, 'drone': drone_label(uri), 'path': list(moves),
            'info': dict(info or {}, speed=speed, z=z, moves='; '.join('%s %s' % m for m in moves))}


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
# System identification in flight: the response to a velocity setpoint
# ---------------------------------------------------------------------------

def identify_response(step=0.2, step_s=1.5, settle_s=2.0, axis='x', z_hold=0.5, uri=None,
                      stop_event=None, drone_id=None, hover_s=2.0, info=None):
    """A flight under the onboard controllers with one velocity setpoint step, no loop of ours.

    Take-off and hover as in fly(); then the velocity setpoint along `axis`
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
            return 0.0, 0.0                       # hold phase: zero velocity setpoint
        if 't_step' not in clock:
            clock['t_step'] = time.monotonic()
        setpoint = step if time.monotonic() - clock['t_step'] < step_s else 0.0
        return (setpoint, 0.0) if axis == 'x' else (0.0, setpoint)

    run = fly(open_loop, axes=axis, step=step * step_s, z_hold=z_hold, hold_s=1.5,
              step_s=step_s + settle_s, v_max=max(abs(step), 0.3), fence=1.0, uri=uri,
              stop_event=stop_event, drone_id=drone_id, hover_s=hover_s,
              info=dict(info or {}, v_step=step, v_step_s=step_s, axis=axis))
    run['identify'] = {'step': step, 'step_s': step_s, 'axis': axis}
    return run


def _pid_arrays(run):
    """The loop records of a run as arrays, from load_run() or straight from fly()."""
    if 'pid' in run:
        return run['pid']
    rows = np.asarray(run['rows'], dtype=float)
    return {name: rows[:, i] for i, name in enumerate(PID_COLUMNS)}


def velocity_step(run, axis='x'):
    """The measured velocity after the setpoint step of an identify_response() run.

    Returns a dict with 't' (seconds since the setpoint step), 'v_meas' (the
    measured velocity relative to its value at the step) and 'v_set' (the
    setpoint step): the raw curve, to be read by eye before fit_tau().
    """
    pid = _pid_arrays(run)
    fb = pid['fb_%s' % axis]
    v = pid['v%s' % axis]
    t = pid['t_s']
    on = np.abs(fb) > 1e-6
    if not on.any():
        raise DroneError('no velocity setpoint step in this run, nothing to fit')
    i0 = int(np.argmax(on))
    i1 = i0 + int(np.argmax(~on[i0:])) if (~on[i0:]).any() else len(t)
    return {'t': t[i0:i1] - t[i0], 'v_meas': v[i0:i1] - v[i0], 'v_set': float(fb[i0])}


def fit_tau(run, axis='x', tau_range=(0.05, 2.0), delay_range=(0.0, 0.6)):
    """Response time of the velocity loop from an identify_response() run.

    The measured velocity after the setpoint step is fitted with a first order
    response with a dead time, v(t) = v_set (1 - exp(-(t - delay) / tau)) for
    t > delay, by a grid search over tau and delay. Returns a dict with tau_s,
    delay_s, the time axis since the step and the measured, fitted and
    setpoint, for plotting.
    """
    vs = velocity_step(run, axis)
    tt, vv, v_set = vs['t'], vs['v_meas'], vs['v_set']
    best = (np.inf, None, None)
    for delay in np.arange(delay_range[0], delay_range[1] + 1e-9, 0.02):
        for tau in np.arange(tau_range[0], tau_range[1] + 1e-9, 0.01):
            model = np.where(tt > delay, v_set * (1.0 - np.exp(-(tt - delay) / tau)), 0.0)
            sse = float(np.sum((vv - model) ** 2))
            if sse < best[0]:
                best = (sse, tau, delay)
    _sse, tau, delay = best
    fit = np.where(tt > delay, v_set * (1.0 - np.exp(-(tt - delay) / tau)), 0.0)
    print('velocity loop: tau %.2f s, dead time %.2f s (setpoint %.2f m/s, %d samples)'
          % (tau, delay, v_set, len(tt)))
    return {'tau_s': float(tau), 'delay_s': float(delay), 'v_set': v_set, 't': tt,
            'v_meas': vv, 'v_fit': fit}


# ---------------------------------------------------------------------------
# From the response time to the gains, and the prediction
# ---------------------------------------------------------------------------

def closed_loop_parameters(Kp, Kd, tau):
    """Natural frequency and damping of the position loop closed around a first order velocity loop.

    With v_dot = (v_set - v) / tau and v_set = Kp (x - x0) + Kd v (our sign
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
    steps to `step`. The feedback is clamped to +/- v_max as in the flight.
    Returns t (seconds since the step), x, v and the feedback.
    """
    n = int(round((hold_s + step_s) / dt))
    n_delay = int(round(delay / dt))
    t = np.arange(n) * dt - hold_s
    x = np.zeros(n)
    v = np.zeros(n)
    fb = np.zeros(n)
    integral = 0.0
    for i in range(1, n):
        target = step if t[i] >= 0 else 0.0
        err = x[i - 1] - target
        integral += err * dt
        c = Kp * err + Ki * integral + Kd * v[i - 1]
        fb[i] = max(-v_max, min(c, v_max))
        c_eff = fb[i - n_delay] if i - n_delay >= 0 else 0.0
        v[i] = v[i - 1] + (c_eff - v[i - 1]) * dt / tau
        x[i] = x[i - 1] + v[i] * dt
    return t, x, v, fb


def model_numbers(Kp, Kd, tau, delay=0.0, step=0.5, v_max=0.3, Ki=0.0, hold_s=4.0, step_s=8.0):
    """Rise time, overshoot, settling error and clamp use of the model step response.

    The same numbers that step_numbers() gives for a flight, from
    model_step_response() with the same arguments: the prediction to write
    down before the flight.
    """
    t, x, v, fb = model_step_response(Kp, Kd, tau, Ki=Ki, delay=delay, step=step, v_max=v_max,
                                       hold_s=hold_s, step_s=step_s)
    in_step = t >= 0
    pid = {'t_s': t + hold_s, 'step': in_step.astype(float), 'x': x, 'vx': v, 'fb_x': fb,
           'target_x': np.where(in_step, step, 0.0)}
    return step_numbers({'label': 'model', 'pid': pid, 'axes': 'x', 't_step': hold_s}, v_max=v_max, quiet=True)['x']


def predict_step(Kp, Kd, Ki, tau, delay, step=0.5, v_max=0.3, limits=None, quiet=False):
    """What the model says these gains do: printed, and the numbers returned.

    Prints the natural frequency and the damping of the closed loop, the
    feedback at the moment of the step against v_max, the gains against the
    noise bounds from the sensor identification (`limits` from gain_limits(), optional), and the
    rise time, overshoot, largest and mean error of the model step response
    with and without the dead time. Returns the numbers with the dead time,
    the dict of model_numbers(): the prediction to write down before the
    flight.
    """
    omega0, zeta = closed_loop_parameters(Kp, Kd, tau)
    with_delay = model_numbers(Kp, Kd, tau, delay=delay, step=step, v_max=v_max, Ki=Ki)
    without = model_numbers(Kp, Kd, tau, delay=0.0, step=step, v_max=v_max, Ki=Ki)
    if not quiet:
        print('Kp = %+.2f 1/s, Kd = %+.2f, Ki = %+.2f 1/s^2 with tau = %.2f s: omega0 = %.2f rad/s, zeta = %.2f%s'
              % (Kp, Kd, Ki, tau, omega0, zeta, '' if Kd <= 0 else '  (a positive Kd removes damping)'))
        at_step = abs(Kp) * step
        print('feedback at the step %.2f m/s against v_max = %.2f m/s: %s'
              % (at_step, v_max, 'at the limit, the first part of the response is a flight at v_max'
                 if at_step > v_max else 'below the limit'))
        if limits is not None:
            exceeded = abs(Kp) > limits['Kp_max'] or abs(Kd) > limits['Kd_max']
            print('noise bounds from the sensor identification: |Kp| %.2f of %.1f, |Kd| %.2f of %.1f%s'
                  % (abs(Kp), limits['Kp_max'], abs(Kd), limits['Kd_max'],
                     '  EXCEEDED, the noise of the estimate will show in the feedback' if exceeded else ''))
        show = lambda v: '--' if v is None else '%.2f' % v
        print('%-28s %18s %14s' % ('model', 'dead time %.2f s' % delay, 'no dead time'))
        for key, text in (('rise_s', 'rise to 90 % [s]'), ('overshoot_m', 'overshoot [m]'),
                          ('settle_error_m', 'max error, last 3 s [m]'),
                          ('mean_error_m', 'mean error, last 3 s [m]')):
            print('%-28s %18s %14s' % (text, show(with_delay[key]), show(without[key])))
    return with_delay


def run_table(runs, quiet=False):
    """One line per run and axis: the gains the run was flown with and the numbers of its step.

    The gains come from the settings saved with the run (see save()); runs
    without them show '--'. The numbers are those of step_numbers(): rise
    time to 90 percent, overshoot, largest and mean error over the last 3 s,
    fraction of the step spent at the limit of the feedback. Returns the rows as a
    list of dicts.
    """
    rows = []
    for r in runs:
        info = r.get('info', {})
        for a, n in step_numbers(r, v_max=info.get('v_max', 0.3), quiet=True).items():
            rows.append(dict(run=r['label'], axis=a, Kp=info.get('Kp'), Kd=info.get('Kd'), Ki=info.get('Ki'), **n))
    if not quiet:
        gain = lambda v: '    --' if v is None else '%+6.2f' % v
        print('%-26s %-4s %6s %6s %6s %8s %9s %12s %9s %6s'
              % ('run', 'axis', 'Kp', 'Kd', 'Ki', 'rise [s]', 'over [m]', 'max err [m]', 'mean [m]', 'limit'))
        for row in rows:
            rise = '      --' if row['rise_s'] is None else '%8.2f' % row['rise_s']
            print('%-26s %-4s %s %s %s %s %9.3f %12.3f %+9.3f %5.0f%%'
                  % (row['run'], row['axis'], gain(row['Kp']), gain(row['Kd']), gain(row['Ki']), rise,
                     row['overshoot_m'], row['settle_error_m'], row['mean_error_m'], 100 * row['clamp_fraction']))
    return rows


# ---------------------------------------------------------------------------
# A model drone: the flight of fly() without a drone
# ---------------------------------------------------------------------------

MODEL_NOISE = {'std_x_m': 0.002, 'std_y_m': 0.002, 'std_vx_mps': 0.01, 'std_vy_mps': 0.01,
               'drift_x_m': 0.0, 'drift_y_m': 0.0, 'still_s': 10.0}   # used when no sensor identification file is given


class _ModelLog:
    """Stands in for the FlightLog of a real flight: the model has no sensors to log."""
    rows = {'state': [], 'sensor': [], 'power': [], 'land': []}
    n_rows = 0

    def save_csv(self, prefix):
        return []


def fly_model(controller, axes='x', step=0.5, hold_s=4.0, step_s=8.0, loop_dt=0.02, v_max=0.3,
              fence=1.0, reset=None, tau=0.3, delay=0.0, drone_id=None,
              noise_scale=1.0, seed=None, quiet=False, info=None):
    """The flight of fly() on a model of the drone: no radio, no drone, no waiting.

    The model is a first order velocity loop: the velocity loop of the firmware
    follows the feedback as a first order system with the response time `tau` and the
    dead time `delay`, the feedback is clamped to +/- v_max, and the loop runs
    at 1 / loop_dt Hz through the same sequence as fly(): hold for hold_s,
    step the target by `step` along `axes`, run for step_s. The controller
    sees the true position and velocity plus noise and drift: the values of
    a sensor identification when `drone_id` names a results file of
    identify_sensors(), MODEL_NOISE otherwise, scaled by `noise_scale`.
    The fence and a failing controller abort the flight as in fly().

    Returns the run dict of fly(), so save(), load_run() and the plots work
    unchanged; the files are named flight_model_<HHMM>_<label>_*.csv.
    """
    rng = np.random.default_rng(seed)
    noise = dict(MODEL_NOISE)
    source = 'typical values'
    if drone_id is not None:
        ident = load_drone_id(drone_id)
        noise.update({k: ident[k] for k in noise if k in ident})
        source = drone_id
    std_x = noise_scale * np.array([noise['std_x_m'], noise['std_y_m']])
    std_v = noise_scale * np.array([noise['std_vx_mps'], noise['std_vy_mps']])
    drift = noise_scale * np.array([noise['drift_x_m'], noise['drift_y_m']]) / max(noise['still_s'], 1e-6)
    if not quiet:
        print('model drone: tau %.2f s, dead time %.2f s, noise x %.1f mm, vx %.0f mm/s, drift %.1f mm/s (%s)'
              % (tau, delay, 1000 * std_x[0], 1000 * std_v[0], 1000 * drift[0], source))
    n_delay = int(round(delay / loop_dt))
    pos = np.zeros(2)                        # the true state of the model
    vel = np.zeros(2)
    pending = [np.zeros(2)] * (n_delay + 1)   # feedback values on their way through the dead time
    t = 0.0

    def measured():
        return (pos + drift * t + rng.normal(0.0, 1.0, 2) * std_x,
                vel + rng.normal(0.0, 1.0, 2) * std_v)

    if reset is not None:
        reset()
    rows = []
    aborted = None
    meas_pos, meas_vel = measured()
    x0, y0 = meas_pos
    target = [x0, y0]
    for phase, dur in (('hold', hold_s), ('step', step_s)):
        if phase == 'step':
            if 'x' in axes:
                target[0] = x0 + step
            if 'y' in axes:
                target[1] = y0 + step
        for _ in range(int(round(dur / loop_dt))):
            x, y = meas_pos
            vx, vy = meas_vel
            if abs(x - x0) > fence or abs(y - y0) > fence:
                aborted = 'fence at x %.2f m, y %.2f m from start' % (x - x0, y - y0)
                break
            try:
                fb_x, fb_y = _call_controller(controller, x, y, vx, vy, target, v_max)
            except Exception as exc:
                aborted = 'controller raised %s: %s' % (type(exc).__name__, exc)
                break
            if 'x' not in axes:
                fb_x = 0.0
            if 'y' not in axes:
                fb_y = 0.0
            rows.append((t, int(round(1000 * t)), 1 if phase == 'step' else 0, x, y, vx, vy,
                         target[0], target[1], fb_x, fb_y, 0.001, 0.001))
            pending.append(np.array([fb_x, fb_y]))
            effective = pending[-1 - n_delay]
            vel = vel + (effective - vel) * loop_dt / tau
            pos = pos + vel * loop_dt
            t += loop_dt
            meas_pos, meas_vel = measured()
        if aborted:
            break
    if not quiet:
        print('%s (%d loop iterations)' % ('ABORTED: %s' % aborted if aborted else 'model flight completed', len(rows)))
    return {'rows': rows, 'flight': _ModelLog(), 'aborted': aborted, 'axes': axes, 'step': step,
            'v_max': v_max, 'uri': 'model', 'time': time.strftime('%H%M'), 'drone': 'model',
            'model': {'tau': tau, 'delay': delay, 'noise_scale': noise_scale},
            'info': dict(info or {}, tau=tau, delay=delay, noise_scale=noise_scale)}


# ---------------------------------------------------------------------------
# Saving and loading runs
# ---------------------------------------------------------------------------

PID_COLUMNS = ['t_s', 't_ms', 'step', 'x', 'y', 'vx', 'vy', 'target_x', 'target_y',
               'fb_x', 'fb_y', 'var_px', 'var_py']


def run_name(run, label):
    """The name of a saved run: drone label (or 'model'), time of the flight and the run label, e.g. 'A_1541_kp05'."""
    drone = run.get('drone') or drone_label(run['uri'])
    return '%s_%s_%s' % (drone, run['time'], label)


def save(run, label, data_dir='data'):
    """Save one run from fly() as data/flight_<name>_*.csv, print the settle check, return the name.

    The name is the drone label, the time of the flight and `label`, for
    example A_1541_kp05, so that files of several drones and flights can
    share a folder. The log groups save as
    flight_<name>_{state,sensor,power,land}.csv and the control loop as
    flight_<name>_pid.csv (state in, target, feedback out per iteration).
    The settings of the run go to flight_<name>_info.csv as name,value
    rows: drone, time, label, axes, step, v_max, the abort reason, and
    what the flight was given as `info`, for example the gains.
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
    settings = [('drone', run.get('drone') or drone_label(run['uri'])), ('time', run['time']),
                ('run_label', label), ('axes', run['axes']), ('step', run['step']), ('v_max', run['v_max']),
                ('aborted', run['aborted'] or '')] + list(run.get('info', {}).items())
    info_csv = '%s_info.csv' % prefix
    with open(info_csv, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['name', 'value'])
        w.writerows(settings)
    print('saved', info_csv)

    if run['aborted']:
        print('this run was aborted (%s), numbers may be incomplete' % run['aborted'])
    if not run['rows']:
        print('no loop records: the flight ended before the handover; the logs are saved for a look at what happened')
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

    Returns a dict with 'label' (the name), 'drone', 'when' and 'run_label'
    (the three parts of the name, None when the name has another form),
    'pid' (dict of arrays, see PID_COLUMNS), the groups 'state', 'sensor',
    'power', 'land' where their files exist, 'info' (the settings saved
    with the run, see save(); empty for a run saved without them), 'axes'
    (the axes the controller acted on, found from the target columns) and 't_step' (loop
    time of the step, None for a run aborted before the step).
    """
    prefix = os.path.join(data_dir, 'flight_%s' % name)
    run = {'label': name, 'pid': load_csv('%s_pid.csv' % prefix)}
    for old_name, new_name in (('cmd_x', 'fb_x'), ('cmd_y', 'fb_y')):     # runs saved before the rename
        if old_name in run['pid']:
            run['pid'][new_name] = run['pid'].pop(old_name)
    parts = name.split('_', 2)
    if len(parts) == 3 and len(parts[1]) == 4 and parts[1].isdigit():
        run['drone'], run['when'], run['run_label'] = parts
    else:
        run['drone'], run['when'], run['run_label'] = None, None, name
    for group in LOG_GROUPS:
        path = '%s_%s.csv' % (prefix, group)
        if os.path.exists(path):
            run[group] = load_csv(path)
    run['info'] = {}
    info_csv = '%s_info.csv' % prefix
    if os.path.exists(info_csv):
        with open(info_csv, newline='', encoding='utf-8-sig') as f:
            reader = csv.reader(f)
            next(reader)
            run['info'] = {name: _value(value) for name, value in reader}
    pid = run['pid']
    if len(pid['t_s']) == 0:                 # aborted before the handover: the logs exist, no loop records
        run['axes'], run['t_step'] = '', None
        return run
    run['axes'] = ''.join(a for a in 'xy' if np.ptp(pid['target_%s' % a]) > 1e-6)
    in_step = pid['step'] > 0.5
    run['t_step'] = float(pid['t_s'][in_step][0]) if in_step.any() else None
    return run


def saved_runs(data_dir='data', match=None, drone=None):
    """Names of the runs saved in data_dir, oldest first.

    `match` keeps only names containing it, `drone` only the runs of that
    drone label ('A', 2, or 'model' for the model flights).
    """
    import glob
    paths = sorted(glob.glob(os.path.join(data_dir, 'flight_*_pid.csv')), key=os.path.getmtime)
    names = [os.path.basename(p)[len('flight_'):-len('_pid.csv')] for p in paths]
    return [n for n in names if (match is None or match in n)
            and (drone is None or n.startswith('%s_' % drone))]


def load_runs(data_dir='data', match=None, drone=None):
    """All runs saved in data_dir as a list of load_run() dicts, oldest first; filters as in saved_runs()."""
    return [load_run(name, data_dir) for name in saved_runs(data_dir, match, drone)]


def step_numbers(run, v_max=0.3, crit=0.15, quiet=False):
    """Rise time, overshoot, settling error and clamp use of one run, per axis the controller acted on.

    Rise time is the time after the step at which the position first passes
    90 percent of the step. Overshoot is the largest excursion beyond the new
    target. The settling error is the largest |target - position| over the
    last 3 s of the step phase; below `crit` counts as settled. Clamp use is
    the fraction of the step phase in which the feedback sat at +/- v_max.
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
        clamped = np.abs(pid['fb_%s' % a][in_step]) >= v_max - 1e-3
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
                                            'config', 'relabel', 'channel', 'params', 'compare'],
                        help='find: list the drones in range by label (all channels for one label with --label); '
                             'status: battery and readiness; recover: clear a crashed state, '
                             'then status; reboot: soft reboot over the radio (for a locked '
                             'drone); charge: report every 30 s while charging until charged; '
                             'flow: raw flow sensor check, move the drone by hand; '
                             'config: the address stored in the drone; '
                             'relabel: store the label given with --new in the drone; '
                             'channel: move the drone to the channel of its label, 2 + 7 times the '
                             'label, looking for it on channel 80 first; '
                             'params: read every parameter, list the stored settings, save them as csv; '
                             'compare: the parameters that differ between two params files (--files A B)')
    parser.add_argument('--label', default=None, help='the label of the drone, a hex digit 1 to B')
    parser.add_argument('--uri', default=None, help='the radio address, if not built from the label')
    parser.add_argument('--new', default=None, help='relabel: the new label to store in the drone')
    parser.add_argument('--files', nargs=2, default=None, metavar='CSV', help='compare: two files of params')
    parser.add_argument('--out', default=None, help='params: the csv to write, default data/params_<label>.csv')
    parser.add_argument('--channel', type=int, default=None,
                        help='find: ping all labels on this one channel only (quick), instead of scanning all channels')
    args = parser.parse_args()
    if args.command == 'relabel' and args.new is None:
        parser.error('relabel needs --new LABEL')
    if args.command == 'channel' and args.label is None:
        parser.error('channel needs --label LABEL')
    if args.command == 'compare' and args.files is None:
        parser.error('compare needs --files A.csv B.csv')

    try:
        uri = args.uri or (drone_uri(args.label) if args.label is not None else None)
        if args.command == 'find':
            if args.label is None:
                find_drones(channel=args.channel)
            else:
                find_drone(args.label)
        elif args.command == 'channel':
            set_channel(args.label)
        elif args.command == 'compare':
            compare_params(*args.files)
        elif args.command == 'reboot':
            reboot(uri)
            print('rebooted, wait a few seconds before reconnecting')
        else:
            with connect(uri) as scf:
                if args.command == 'config':
                    print('stored in the drone: %s' % config_uri(read_config(scf)))
                elif args.command == 'params':
                    dump_params(scf, out=args.out or 'data/params_%s.csv' % drone_label(uri))
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
