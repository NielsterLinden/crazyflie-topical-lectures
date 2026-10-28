"""Reproduce Andreas's controller experiments; no mode sends flight commands."""

import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import time

import numpy as np

from . import hover_pid, velocity_pid


def load_recording(path):
    """Read synchronized horizontal state, including the original Stage 3 PID CSV."""
    data = hover_pid.load_log(path)
    required = ('t_s', 'x', 'y', 'vx', 'vy')
    if any(name not in data for name in required):
        raise ValueError('recording requires t_s,x,y,vx,vy; see the Day 1 guide')
    if len(data['t_s']) < 2 or any(not np.isfinite(data[name]).all() for name in required):
        raise ValueError('recording requires at least two finite state samples')
    if any(len(data[name]) != len(data['t_s']) for name in required):
        raise ValueError('every recording row must contain all required state components')
    if np.any(np.diff(data['t_s']) <= 0):
        raise ValueError('recording times must be strictly increasing; unwrap/split sessions first')
    return {name: data[name] for name in required}


def simulate(args):
    """Reuse the original three plant helpers, rather than another PID implementation."""
    count = int(args.duration / args.dt)
    target = np.where(np.arange(count) * args.dt < 4.0, 0.0, args.step)
    position = hover_pid.AxisPID(args.kp, args.kd, args.ki, args.v_max)
    velocity = velocity_pid.AxisVelocityPID(args.kv, args.kvi, args.tilt_max)
    # Each original helper resets its controller; states are not shared between runs.
    return {
        'position': hover_pid.simulate_axis(position, target, args.dt, count,
                                           tau=args.tau, v_bias=args.bias,
                                           position_noise_std=args.position_noise_std,
                                           velocity_noise_std=args.velocity_noise_std, seed=args.seed),
        'velocity': velocity_pid.simulate_velocity_axis(
            velocity, np.where(np.arange(count) * args.dt < 4, 0, args.velocity_step), args.dt, count,
            tau_att=args.tau_att, drag=args.drag,
            velocity_noise_std=args.velocity_noise_std, seed=args.seed),
        'cascade': velocity_pid.simulate_cascade_axis(
            position, velocity, target, args.dt, count,
            tau_att=args.tau_att, drag=args.drag, position_noise_std=args.position_noise_std,
            velocity_noise_std=args.velocity_noise_std, seed=args.seed),
    }


def replay(data, args):
    """Compute commands from a fixed recording; do not claim a counterfactual flight."""
    result = {}
    for axis in ('x', 'y'):
        target = np.full(len(data['t_s']), data[axis][0])
        position = hover_pid.AxisPID(args.kp, args.kd, args.ki, args.v_max)
        terms = hover_pid.replay_axis(position, target, data[axis], data['v' + axis], data['t_s'])
        # Keep independent controller histories: damping motion toward zero velocity
        # is a different experiment from restoring the initial position.
        velocity_target = np.zeros(len(data['t_s']))
        velocity = velocity_pid.AxisVelocityPID(args.kv, args.kvi, args.tilt_max)
        standalone = velocity_pid.replay_velocity_axis(
            velocity, velocity_target, data['v' + axis], data['t_s'])
        cascade_velocity = velocity_pid.AxisVelocityPID(args.kv, args.kvi, args.tilt_max)
        cascade = velocity_pid.replay_velocity_axis(
            cascade_velocity, terms['cmd'], data['v' + axis], data['t_s'])
        result[axis] = dict(t=data['t_s'], x=data[axis], v=data['v' + axis],
                            target=target, **terms, velocity_target=velocity_target,
                            velocity_tilt_cmd=standalone['tilt_cmd'],
                            cascade_tilt_cmd=cascade['tilt_cmd'])
    return result


def record_handcheck(uri, duration, samples=None):
    """Log only: no arming, estimator reset, commander or setpoint calls."""
    import cflib.crtp
    from cflib.crazyflie import Crazyflie
    from cflib.crazyflie.log import LogConfig
    from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

    if not uri or not uri.startswith('radio://'):
        raise ValueError('handcheck requires the assigned --uri or CF_URI')
    print(f'Recording only from {uri}. Motors disabled; propellers removed or approved restraint.')
    if input('Type RECORD ONLY to confirm: ') != 'RECORD ONLY':
        raise ValueError('Recording not confirmed')
    cflib.crtp.init_drivers()
    samples = [] if samples is None else samples
    motors, errors = {}, []
    first_timestamp = None

    def motor_callback(timestamp, data, config):
        values = [data.get(f'motor.m{number}', float('nan')) for number in range(1, 5)]
        motors.update(values=values, received=time.monotonic())
        if any(not np.isfinite(value) or value != 0 for value in values):
            errors.append('Nonzero/invalid motor command; stop recording and review hardware')

    def state_callback(timestamp, data, config):
        nonlocal first_timestamp
        if not motors or time.monotonic() - motors['received'] > 0.25:
            return
        if errors:
            return
        values = [data[name] for name in ('stateEstimate.x', 'stateEstimate.y',
                                         'stateEstimate.vx', 'stateEstimate.vy')]
        if not np.isfinite(values).all():
            errors.append('Invalid state sample')
            return
        if first_timestamp is None:
            first_timestamp = timestamp
        # Firmware log timestamps wrap at 24 bits, not at the host clock origin.
        elapsed = ((timestamp - first_timestamp) % (1 << 24)) / 1000.0
        if samples and elapsed <= samples[-1][0]:
            errors.append('Duplicate/out-of-order state time; retain and inspect the source recording')
            return
        samples.append([elapsed, *values])

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        state = LogConfig(name='day1_state', period_in_ms=50)
        for name in ('stateEstimate.x', 'stateEstimate.y', 'stateEstimate.vx', 'stateEstimate.vy'):
            state.add_variable(name, 'float')
        motor = LogConfig(name='day1_motors', period_in_ms=50)
        for number in range(1, 5):
            motor.add_variable(f'motor.m{number}', 'float')
        state.data_received_cb.add_callback(state_callback)
        motor.data_received_cb.add_callback(motor_callback)
        for block in (motor, state):
            block.error_cb.add_callback(lambda config, message: errors.append(str(message)))
            scf.cf.log.add_config(block)
        try:
            motor.start()
            state.start()
            started = time.monotonic()
            while time.monotonic() - started < duration:
                if errors:
                    raise ValueError(errors[0])
                if motors and time.monotonic() - motors['received'] > 0.25:
                    raise ValueError('Motor-status telemetry is stale')
                time.sleep(0.05)
        finally:
            state.stop()
            motor.stop()
    if errors or len(samples) < 2:
        raise ValueError(errors[0] if errors else 'No usable state/motor telemetry')
    return dict(zip(('t_s', 'x', 'y', 'vx', 'vy'), np.asarray(samples).T))


def save_results(results, args, output):
    import matplotlib.pyplot as plt

    output.mkdir(parents=True, exist_ok=False)
    metadata = vars(args).copy()
    metadata['output'] = str(output)
    metadata['recording'] = str(args.recording) if args.recording else None
    if args.recording:
        metadata['recording_sha256'] = hashlib.sha256(args.recording.read_bytes()).hexdigest()
    metadata['data_kind'] = 'simulation' if args.mode == 'simulate' else 'onboard-state replay'
    metadata['command_semantics'] = 'velocity m/s; tilt toward positive world axis in degrees, NOT firmware roll/pitch'
    metadata['metrics'] = {}
    figure, axes = plt.subplots(len(results), 2, figsize=(11, 3 * len(results)), squeeze=False)
    for row, (name, data) in enumerate(results.items()):
        with (output / f'{name}.csv').open('x', newline='', encoding='utf-8') as stream:
            writer = csv.writer(stream)
            writer.writerow(data)
            writer.writerows(zip(*data.values()))
        # Velocity-only studies have no position target; keep their metrics in m/s.
        observed = data['x'] if 'x' in data else data['v']
        error = observed - data['target']
        metadata['metrics'][name] = {
            'error_units': 'm' if 'x' in data else 'm/s',
            'rms_error': float(np.sqrt(np.mean(error**2))),
            'max_error': float(np.max(np.abs(error))),
            'tail_max_error': float(np.max(np.abs(error[data['t'] >= data['t'][-1] - 3]))),
        }
        axes[row, 0].plot(data['t'], data['target'], '--', label='target')
        axes[row, 0].plot(data['t'], observed, label='simulated state' if args.mode == 'simulate' else 'onboard estimate')
        axes[row, 0].set(title=name, xlabel='Time [s]', ylabel='Position [m]' if 'x' in data else 'Velocity [m/s]')
        if name == 'velocity':
            axes[row, 1].plot(data['t'], data['target'], '--', label='velocity target')
        if 'velocity_target' in data:
            axes[row, 1].plot(data['t'], data['velocity_target'], '--', label='standalone velocity target')
        for key in ('cmd', 'v_set'):
            if key in data:
                axes[row, 1].plot(data['t'], data[key], label='position-loop velocity command')
        if any(key in data for key in ('tilt_cmd', 'velocity_tilt_cmd', 'cascade_tilt_cmd')):
            tilt_axis = axes[row, 1].twinx()
            for key, label, color, style in (
                    ('tilt_cmd', 'tilt command', 'C2', '-'),
                    ('velocity_tilt_cmd', 'standalone velocity-loop tilt', 'C2', '--'),
                    ('cascade_tilt_cmd', 'cascade tilt', 'C3', '-')):
                if key in data:
                    tilt_axis.plot(data['t'], data[key], style, color=color, label=label)
            if 'tilt' in data:
                tilt_axis.plot(data['t'], data['tilt'], ':', color='C2', label='simulated tilt')
            tilt_axis.set_ylabel('Tilt toward positive axis [deg]')
            tilt_axis.legend(loc='lower right')
        axes[row, 1].set(xlabel='Time [s]', ylabel='Velocity command [m/s]')
        for axis in axes[row]:
            axis.grid(alpha=0.3)
            if axis.lines:
                axis.legend()
    figure.suptitle('Closed-loop toy plants' if args.mode == 'simulate' else 'Computed commands on fixed data: not a closed-loop flight')
    figure.tight_layout()
    figure.savefig(output / 'comparison.png')
    (output / 'settings.json').write_text(json.dumps(metadata, indent=2), encoding='utf-8')
    if not args.no_show:
        plt.show()
    plt.close(figure)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=('simulate', 'replay', 'handcheck'), default='simulate')
    parser.add_argument('--recording', type=Path)
    parser.add_argument('--output', type=Path, required=True, help='New output directory; never overwritten')
    parser.add_argument('--no-show', action='store_true')
    parser.add_argument('--uri', default=os.environ.get('CF_URI'))
    parser.add_argument('--seed', type=int, default=0)
    for name, default in (('kp', 1.2), ('kd', 0.6), ('ki', 0.0), ('kv', 12.0), ('kvi', 0.0),
                          ('v-max', 0.3), ('tilt-max', 6.0), ('tau', 0.3), ('tau-att', 0.2),
                          ('drag', 0.25), ('bias', 0.0), ('duration', 12.0), ('dt', 0.02),
                          ('step', 0.5), ('velocity-step', 0.3),
                          ('position-noise-std', 0.0), ('velocity-noise-std', 0.0)):
        parser.add_argument('--' + name, type=float, default=default)
    args = parser.parse_args(argv)
    try:
        numeric = [value for value in vars(args).values() if isinstance(value, float)]
        if not np.isfinite(numeric).all():
            raise ValueError('numeric settings must be finite')
        if min(args.dt, args.tau, args.tau_att, args.v_max, args.tilt_max) <= 0 or args.duration <= 4:
            raise ValueError('positive time constants/limits and duration > 4 s are required')
        if args.duration > 120 or args.dt > min(args.tau, args.tau_att) / 2:
            raise ValueError('use duration <= 120 s and dt <= half the smallest plant time constant')
        if args.duration / args.dt < 2:
            raise ValueError('at least two samples are required')
        if min(args.position_noise_std, args.velocity_noise_std, args.drag) < 0:
            raise ValueError('noise standard deviations and drag must be nonnegative')
        if args.output.exists():
            raise ValueError('output directory already exists; choose a new run ID')
        if args.no_show:
            import matplotlib
            matplotlib.use('Agg')
        if args.mode == 'simulate':
            results = simulate(args)
        else:
            if args.mode == 'handcheck':
                args.output.mkdir(parents=True)
                args.recording = args.output / 'state.csv'
                samples = []
                try:
                    data = record_handcheck(args.uri, args.duration, samples)
                finally:
                    # Preserve partial acquisition on errors, without pretending
                    # that it completed the hand check or qualifies hardware.
                    with args.recording.open('x', newline='', encoding='utf-8') as stream:
                        writer = csv.writer(stream)
                        writer.writerow(('t_s', 'x', 'y', 'vx', 'vy'))
                        writer.writerows(samples)
                args.output = args.output / 'analysis'
            else:
                if not args.recording:
                    raise ValueError('--mode replay requires --recording')
                data = load_recording(args.recording)
            results = replay(data, args)
        save_results(results, args, args.output)
        print(f'Results written to {args.output}; no flight commands sent')
    except (OSError, ValueError) as error:
        parser.error(str(error))


if __name__ == '__main__':
    main()
