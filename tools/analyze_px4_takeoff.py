#!/usr/bin/env python3
"""Offline PX4 ULog takeoff/kill analysis. No hardware or network access.

Requires numpy and pyulog; matplotlib is optional with --plot.
All positions retain native PX4 NED axes. A reported position is not ground truth.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np
from pyulog import ULog


def clean(value):
    if isinstance(value, dict):
        return {k: clean(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [clean(v) for v in value]
    if isinstance(value, np.generic):
        return clean(value.item())
    if isinstance(value, float) and not np.isfinite(value):
        return None
    return value


def stats(values):
    values = np.asarray(values)
    values = values[np.isfinite(values)]
    if not len(values):
        return None
    return dict(min=float(values.min()), max=float(values.max()),
                mean=float(values.mean()), std=float(values.std()),
                first=float(values[0]), last=float(values[-1]))


def euler(data, prefix):
    w, x, y, z = [data[f'{prefix}[{i}]'].astype(float) for i in range(4)]
    return np.rad2deg(np.array([
        np.arctan2(2*(w*x+y*z), 1-2*(x*x+y*y)),
        np.arcsin(np.clip(2*(w*y-z*x), -1, 1)),
        np.arctan2(2*(w*z+x*y), 1-2*(y*y+z*z))]))


def analyze(path, output, plot=False):
    ulog = ULog(str(path))
    start = ulog.start_timestamp

    def get(name, instance=0):
        return next((d.data for d in ulog.data_list
                     if d.name == name and d.multi_id == instance), None)

    def time(data):
        return (data['timestamp'].astype(float)-start)/1e6

    def transitions(data, keys):
        result, previous = [], None
        if data is None:
            return result
        for i, stamp in enumerate(time(data)):
            values = tuple(data[k][i].item() for k in keys)
            if values != previous:
                result.append(dict(time_s=stamp, **dict(zip(keys, values))))
                previous = values
        return result

    def window(data, low, high):
        return (time(data) >= low) & (time(data) < high)

    messages = [dict(time_s=(m.timestamp-start)/1e6, message=m.message)
                for m in ulog.logged_messages]
    takeoff = next((x['time_s'] for x in messages if 'Takeoff detected' in x['message']), 0)
    kill = next((x['time_s'] for x in messages if 'Kill-switch engaged' in x['message']), None)
    duration = (ulog.last_timestamp-start)/1e6
    cutoff = kill if kill is not None else duration
    att, desired = get('vehicle_attitude'), get('vehicle_attitude_setpoint')
    lp, ps = get('vehicle_local_position'), get('vehicle_local_position_setpoint')
    if any(d is None for d in (att, desired, lp, ps)):
        raise ValueError('Required attitude/position feedback or setpoint topic is missing')
    angles, requested = euler(att, 'q'), euler(desired, 'q_d')
    desired_on_att = np.array([np.interp(time(att), time(desired), v) for v in requested])
    errors = (angles-desired_on_att+180) % 360-180
    airborne = window(att, takeoff, cutoff)
    result = dict(file=path.name, sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                  duration_s=duration, takeoff_s=takeoff, kill_switch_s=kill,
                  pre_stop_window_s=[takeoff, cutoff], positions='PX4 NED, estimated; not ground truth',
                  dropouts=[dict(timestamp=d.timestamp, duration_ms=d.duration) for d in ulog.dropouts],
                  messages=messages,
                  modes=transitions(get('vehicle_status'), ['nav_state', 'arming_state', 'failsafe', 'rc_signal_lost']),
                  controls=transitions(get('vehicle_control_mode'),
                                       ['flag_control_auto_enabled', 'flag_control_offboard_enabled', 'flag_armed']),
                  estimator_selection=transitions(get('estimator_selector_status'),
                                                  ['primary_instance', 'instance_changed_count']),
                  attitude_before_stop_deg={}, position_before_stop={}, commands=[], external_vision=[], imu=[])
    for i, name in enumerate(['roll', 'pitch', 'yaw']):
        result['attitude_before_stop_deg'][name] = dict(
            actual=stats(angles[i, airborne]), error_rms=float(np.sqrt(np.mean(errors[i, airborne]**2))),
            error_abs_max=float(np.abs(errors[i, airborne]).max()))
    for key in ['x', 'y', 'z', 'vx', 'vy', 'vz']:
        result['position_before_stop'][key] = stats(lp[key][window(lp, takeoff, cutoff)])
    command = get('vehicle_command')
    if command is not None:
        for i, stamp in enumerate(time(command)):
            result['commands'].append(dict(time_s=stamp, **{
                k: command[k][i].item() for k in command if k != 'timestamp'}))
    for d in ulog.data_list:
        data = d.data
        if d.name == 'estimator_visual_odometry_aligned':
            times = time(data)
            gap_indices = np.flatnonzero(np.diff(times) > .4)
            before = times[times < cutoff]
            result['external_vision'].append(dict(instance=d.multi_id,
                note='Aligned odometry logged at reduced rate; gaps are corroborated with EKF flags',
                gaps_over_400ms=[dict(start_s=times[i], end_s=times[i+1], gap_s=times[i+1]-times[i])
                                 for i in gap_indices],
                age_since_last_logged_update_at_stop_s=float(cutoff-before[-1]) if len(before) else None))
        if d.name == 'estimator_status_flags':
            result.setdefault('estimator_flags', []).append(dict(instance=d.multi_id,
                events=transitions(data, ['cs_ev_pos', 'cs_ev_yaw', 'cs_ev_hgt',
                                         'cs_inertial_dead_reckoning', 'reject_yaw', 'reject_ver_pos'])))
        if d.name == 'vehicle_imu_status':
            keys = ['accel_clipping[0]', 'accel_clipping[1]', 'accel_clipping[2]',
                    'accel_vibration_metric', 'gyro_vibration_metric', 'accel_error_count', 'gyro_error_count']
            mask = window(data, takeoff, cutoff)
            result['imu'].append(dict(instance=d.multi_id,
                pre_stop={k: stats(data[k][mask]) for k in keys}))
    sensor = get('sensor_combined')
    if sensor is not None:
        result['accelerometer_before_stop_ms2'] = {
            k: stats(sensor[k][window(sensor, max(2, takeoff), cutoff)])
            for k in ['accelerometer_m_s2[0]', 'accelerometer_m_s2[1]', 'accelerometer_m_s2[2]']}
    output.mkdir(parents=True, exist_ok=True)
    (output/(path.stem+'_analysis.json')).write_text(
        json.dumps(clean(result), ensure_ascii=False, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    with (output/(path.stem+'_trace.csv')).open('w', encoding='utf-8', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['time_s', 'north_m', 'east_m', 'down_m', 'north_sp_m', 'east_sp_m', 'down_sp_m',
                         'roll_deg', 'pitch_deg', 'yaw_deg', 'roll_sp_deg', 'pitch_sp_deg', 'yaw_sp_deg'])
        for i, stamp in enumerate(time(lp)):
            row = [stamp]+[lp[k][i] for k in ['x', 'y', 'z']]
            row += [np.interp(stamp, time(ps), ps[k]) for k in ['x', 'y', 'z']]
            row += [np.interp(stamp, time(att), v) for v in angles]
            row += [np.interp(stamp, time(desired), v) for v in requested]
            writer.writerow(clean(row))
    if plot:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(3, 2, figsize=(12, 9), sharex=True)
        for i, name in enumerate(['roll', 'pitch']):
            ax = axes[0, i]
            ax.plot(time(att), angles[i], label='actual')
            ax.plot(time(desired), requested[i], label='requested')
            ax.set_ylabel(name+' (deg)'); ax.set_ylim(-8, 8); ax.legend()
        ev_instance = int(get('estimator_selector_status')['primary_instance'][0])
        ev = get('estimator_visual_odometry_aligned', ev_instance)
        for i, name in enumerate(['x', 'y']):
            ax = axes[1, i]
            ax.plot(time(lp), lp[name], label='EKF')
            ax.plot(time(ps), ps[name], label='requested')
            if ev is not None:
                ax.plot(time(ev), ev[name], '.', label='external vision')
            ax.set_ylabel(('NED North' if i == 0 else 'NED East')+' (m)'); ax.legend()
        axes[2, 0].plot(time(lp), -lp['z'], label='EKF height')
        axes[2, 0].plot(time(ps), -ps['z'], label='requested')
        axes[2, 0].set_ylim(-.5, 2); axes[2, 0].set_ylabel('Height (m)'); axes[2, 0].legend()
        axes[2, 1].plot(time(att), angles[2], label='yaw')
        axes[2, 1].plot(time(desired), requested[2], label='requested')
        axes[2, 1].set_ylim(80, 115); axes[2, 1].set_ylabel('Yaw (deg)'); axes[2, 1].legend()
        for ax in axes.flat:
            ax.set_xlim(0, duration); ax.grid(alpha=.25)
            if kill is not None:
                ax.axvline(kill, color='red', linestyle='--')
                ax.axvspan(kill, duration, color='red', alpha=.07)
        for ax in axes[-1]:
            ax.set_xlabel('Seconds since log start; red = after kill switch')
        fig.suptitle(path.name+' | pre-stop attitude panels clipped to +/-8 deg; full data in CSV')
        fig.tight_layout(); fig.savefig(output/(path.stem+'_overview.png'), dpi=145)
        plt.close(fig)
    print(json.dumps(clean({k: result[k] for k in
                           ['file', 'duration_s', 'takeoff_s', 'kill_switch_s', 'modes', 'attitude_before_stop_deg']})))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('ulog', type=Path, nargs='+')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--plot', action='store_true')
    args = parser.parse_args()
    for path in args.ulog:
        analyze(path, args.output, args.plot)


if __name__ == '__main__':
    main()
