#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Ground-only lightweight recorder A/B check. Never sends control commands."""
from __future__ import print_function
import argparse
from collections import defaultdict
import json
import os
import signal
import subprocess
import sys
import threading
import time
from uav_perf import Sampler, steady
import uav_log_lite as lite


def percentile(values, p):
    return sorted(values)[int((len(values)-1)*p)] if values else None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output'); parser.add_argument('--seconds', type=int, default=90)
    parser.add_argument('--logging', action='store_true')
    args = parser.parse_args()
    directory = os.path.abspath(args.output); os.makedirs(directory)
    import rosgraph
    master = rosgraph.Master('/lite_bench_check')
    try:
        nodes = set(n for g in master.getSystemState() for t,ns in g for n in ns)
        if nodes-{'/rosout'}:
            raise RuntimeError('Existing ROS graph; do not overlap a real task')
    except socket_error_types():
        pass
    os.environ['UAV_LOG_DISABLED'] = '0' if args.logging else '1'
    os.environ['PYTHONUNBUFFERED'] = '1'
    stack_log = open(directory+'/stack.stdout.log', 'wb')
    stack = subprocess.Popen([os.path.expanduser('~/.local/bin/uav'), 'check', '--target', 'flammable'],
        stdout=stack_log, stderr=subprocess.STDOUT, preexec_fn=lite.child_session)
    top_log = open(directory+'/top.txt', 'wb'); vm_log = open(directory+'/vmstat.txt', 'wb')
    top = subprocess.Popen(['top', '-b', '-d', '2'], stdout=top_log, preexec_fn=lite.child_session)
    vm = subprocess.Popen(['vmstat', '1'], stdout=vm_log, preexec_fn=lite.child_session)
    import rospy
    from std_msgs.msg import String
    from mavros_msgs.msg import State
    phase = ['warmup']; state = [None]; values = {}; forbidden = []; health = []
    received = defaultdict(list); lock = threading.Lock()
    gates = ('model_loaded', 'frame_fresh', 'inference_ready', 'pose_fresh', 'fcu_fresh', 'tf_fresh')

    def topic_callback(topic):
        def cb(msg):
            if phase[0] == 'measure':
                with lock:
                    received[topic].append(steady())
            if topic in ('/mavros/setpoint_raw/local', '/gpio_cmd'):
                forbidden.append(topic)
        return cb

    def readiness(msg):
        value = json.loads(msg.data); values['readiness'] = (steady(), value)
        if phase[0] == 'measure':
            health.append(value); received['/mission/readiness'].append(steady())
        if value.get('actuation_enabled') or value.get('flight_authority'):
            forbidden.append('Actuation or flight authority enabled')

    def fcu(msg):
        state[0] = msg
        if msg.armed:
            forbidden.append('FCU armed')

    report = dict(logging=args.logging, seconds=args.seconds, passed=False, ground_only=True)
    try:
        deadline = steady()+20
        while steady() < deadline:
            try:
                master.getPid(); break
            except Exception:
                time.sleep(.2)
        rospy.init_node('lite_logging_benchmark', disable_signals=True)
        subscribers = [rospy.Subscriber(t, rospy.AnyMsg, topic_callback(t), queue_size=4)
            for t in ['/mavros/imu/data', '/scan', '/mavros/local_position/pose',
                      '/mavros/vision_pose/pose', '/mavros/setpoint_raw/local', '/gpio_cmd']]
        subscribers += [rospy.Subscriber('/mission/readiness', String, readiness, queue_size=2),
                        rospy.Subscriber('/mavros/state', State, fcu, queue_size=1)]
        deadline = steady()+100; good_since = None
        while steady() < deadline:
            if stack.poll() is not None or forbidden:
                raise RuntimeError('Stack stopped or forbidden output: '+str(forbidden))
            row = values.get('readiness')
            healthy = row and steady()-row[0] < 1 and all(row[1].get(k) for k in gates)
            if healthy:
                if good_since is None:
                    good_since = steady()
                if steady()-good_since >= 15:
                    break
            else:
                good_since = None
            time.sleep(.2)
        else:
            raise RuntimeError('No stable readiness')
        report['session'] = lite.active()
        if args.logging and not report['session']:
            raise RuntimeError('Recorder not running')
        phase[0] = 'measure'; began = steady(); report['start_wall'] = time.time()
        sampler = Sampler(); samples = []
        with open(directory+'/performance.jsonl', 'w') as stream:
            while steady()-began < args.seconds:
                if forbidden or stack.poll() is not None:
                    raise RuntimeError('Forbidden output or ground stack exit: '+str(forbidden))
                if args.logging and not lite.active():
                    raise RuntimeError('Recorder stopped during measurement')
                row = values.get('readiness')
                if not row or steady()-row[0] > 1:
                    raise RuntimeError('Readiness stopped')
                sample = sampler.sample(); samples.append(sample)
                stream.write(json.dumps(sample, sort_keys=True)+'\n'); stream.flush(); time.sleep(1)
        phase[0] = 'finished'; report['end_wall'] = time.time()
        report['topics'] = {}
        for topic,times in received.items():
            gaps = [b-a for a,b in zip(times,times[1:])]
            report['topics'][topic] = dict(count=len(times),
                hz=(len(times)-1)/(times[-1]-times[0]) if len(times)>1 else None,
                max_gap_s=max(gaps) if gaps else None, p99_gap_s=percentile(gaps, .99))
        cpu = [s['cpu_busy_pct'] for s in samples if s['cpu_busy_pct'] is not None]
        report['cpu_busy_mean'] = sum(cpu)/len(cpu)
        report['iowait_p99'] = percentile([s['iowait_pct'] for s in samples if s['iowait_pct'] is not None], .99)
        report['min_available_mib'] = min(s['mem_available_kib'] for s in samples)/1024.
        report['readiness_healthy_fraction'] = sum(all(r.get(k) for k in gates) for r in health)/float(len(health))
        report['forbidden_outputs'] = forbidden
        report['passed'] = report['readiness_healthy_fraction'] >= .99
        for topic,minimum in [('/mavros/imu/data',35),('/scan',7),('/mavros/local_position/pose',20),
                              ('/mavros/vision_pose/pose',14),('/mission/readiness',7)]:
            if (report['topics'].get(topic,{}).get('hz') or 0) < minimum:
                report['passed'] = False
    except Exception as error:
        report['error'] = str(error)
    finally:
        if state[0] is None or state[0].armed:
            report['left_stack_running'] = 'FCU state unknown or armed; requires manual intervention'
        else:
            if stack.poll() is None:
                os.killpg(stack.pid, signal.SIGINT)
            deadline = steady()+25
            while stack.poll() is None and steady() < deadline:
                time.sleep(.1)
            if stack.poll() is None:
                report['cleanup_error'] = 'Stack did not close'; report['passed'] = False
            if args.logging:
                try:
                    lite.stop_active()
                except Exception as error:
                    report['cleanup_error'] = str(error); report['passed'] = False
        for proc in (top, vm):
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGTERM)
            proc.wait()
        lite.atomic_json(directory+'/result.json', report)
        print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report['passed'] else 1


def socket_error_types():
    import socket
    return (socket.error,)


if __name__ == '__main__':
    sys.exit(main())
