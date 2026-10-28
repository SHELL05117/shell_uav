#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Explicit normal arming for an already selected OFFBOARD mission.

Never sends NAV_TAKEOFF, set-mode, forced arming or disarm. The task's existing
OFFBOARD state machine owns ascent. --check performs the same gates without
calling any flight service. Hardware execution requires a human `uav arm`.
"""
from __future__ import print_function
import argparse
from collections import deque
import json
import os
import socket
import sys
import threading
import time

GATES = ('model_loaded', 'frame_fresh', 'inference_ready', 'pose_fresh', 'fcu_fresh', 'tf_fresh')


def steady():
    with open('/proc/uptime') as stream:
        return float(stream.read().split()[0])


def reasons(values, now, ros_now, parameters):
    pending = []
    def fresh(key, timeout):
        row = values.get(key)
        if not row or not (0 <= now-row[0] <= timeout):
            pending.append(key+' missing/stale'); return None
        return row[1]
    state = fresh('state', 1.5)
    if state:
        if not state.connected: pending.append('FCU disconnected')
        if state.armed: pending.append('FCU already armed; do not repeat arming')
        if state.mode != 'OFFBOARD': pending.append('select OFFBOARD; do not click QGC Takeoff')
    readiness = fresh('readiness', .5)
    if readiness:
        if not all(readiness.get(k) is True for k in GATES): pending.append('mission inputs not ready')
        if readiness.get('actuation_enabled') is not True: pending.append('mission actuation disabled')
        if readiness.get('mission_aborted') is not False: pending.append('mission aborted/unknown')
        if readiness.get('phase') != 0: pending.append('mission is not waiting for initial takeoff')
    for key,timeout in [('pose',.3), ('vision',.3)]:
        message = fresh(key, timeout)
        if message and not (0 <= ros_now-message.header.stamp.to_sec() <= timeout):
            pending.append(key+' source timestamp stale/future')
    if values.get('location', (None,None))[1] != 'READY': pending.append('location not READY')
    stamps = [t for t in values.get('setpoints', []) if now-1.5 <= t <= now]
    if len(stamps) < 30 or now-stamps[-1] > .15:
        pending.append('setpoint stream missing/stale')
    elif max(b-a for a,b in zip(stamps, stamps[1:])) > .15:
        pending.append('setpoint stream has gaps')
    if parameters.get('enable_actuation') is not True: pending.append('wrong mission enable_actuation')
    if parameters.get('auto_start') is not False: pending.append('wrong/unknown auto_start')
    if parameters.get('target_hazard') not in ('explosive','flammable','toxic'):
        pending.append('hazard target missing')
    return pending


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true', help='Read-only: never arm')
    args = parser.parse_args()
    import rosgraph
    socket.setdefaulttimeout(2)
    rosgraph.Master('/uav_arm_gate').getPid()
    import rospy
    from geometry_msgs.msg import PoseStamped
    from mavros_msgs.msg import State, PositionTarget
    from mavros_msgs.srv import CommandBool, CommandBoolRequest
    from std_msgs.msg import String
    rospy.init_node('uav_arm_gate', anonymous=True, disable_signals=True)
    lock = threading.Lock(); values = {'setpoints': deque(maxlen=100)}

    def receive(key):
        def cb(message):
            with lock:
                if key == 'setpoints': values[key].append(steady())
                elif key == 'readiness':
                    try: values[key] = (steady(), json.loads(message.data))
                    except ValueError: values.pop(key, None)
                elif key == 'location': values[key] = (steady(), message.data)
                else: values[key] = (steady(), message)
        return cb

    subscriptions = [rospy.Subscriber(topic, kind, receive(key), queue_size=2)
        for key,topic,kind in [('state','/mavros/state',State),
            ('readiness','/mission/readiness',String), ('location','/location_node/status',String),
            ('pose','/mavros/local_position/pose',PoseStamped), ('vision','/mavros/vision_pose/pose',PoseStamped),
            ('setpoints','/mavros/setpoint_raw/local',PositionTarget)]]
    prefix = '/shoot_with_move_base_node/'
    def params():
        return dict((k, rospy.get_param(prefix+k, None))
                    for k in ('enable_actuation','auto_start','target_hazard'))
    parameters = params(); deadline = steady()+10; good_since = None; pending = ['No samples']
    while steady() < deadline and not rospy.is_shutdown():
        with lock:
            pending = reasons(values, steady(), rospy.Time.now().to_sec(), parameters)
        if not pending:
            if good_since is None: good_since = steady()
            if steady()-good_since >= 2: break
        else: good_since = None
        time.sleep(.05)
    else:
        raise RuntimeError('Arming refused: '+', '.join(pending))
    # Re-read parameters and all streams immediately before creating the request.
    parameters = params()
    with lock:
        pending = reasons(values, steady(), rospy.Time.now().to_sec(), parameters)
    if pending: raise RuntimeError('Arming refused: '+', '.join(pending))
    if args.check:
        print('CHECK ONLY: OFFBOARD, disarmed, mission/position/vision ready and live setpoints; no arming sent')
        return 0
    rospy.wait_for_service('/mavros/cmd/arming', timeout=2)
    with lock:
        pending = reasons(values, steady(), rospy.Time.now().to_sec(), parameters)
    if pending: raise RuntimeError('Arming refused: '+', '.join(pending))
    print('Normal arming in OFFBOARD; mission ascent starts immediately. No NAV_TAKEOFF command is sent.')
    sys.stdout.flush()
    directory = None
    try:
        import uav_log_lite
        directory = uav_log_lite.attach('arm', dict(request='normal arming only', mode='OFFBOARD'))
    except Exception as error:
        print('Arming log unavailable: '+str(error), file=sys.stderr)
    # This service has only bool value; there is no force parameter or mode call.
    response = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)(CommandBoolRequest(value=True))
    if directory:
        try:
            uav_log_lite.atomic_json(directory+'/arm-result-%d.json' % int(time.time()*1000),
                dict(wall=time.time(), steady=steady(), success=response.success, result=response.result))
        except Exception as error:
            print('Arming ACK log unavailable: '+str(error), file=sys.stderr)
    if not response.success:
        raise RuntimeError('PX4 rejected normal arming (result=%s); inspect statustext; no retry' % response.result)
    deadline = steady()+3
    while steady() < deadline:
        with lock:
            state = values.get('state')
            if state and steady()-state[0] < 1.5 and state[1].armed:
                if state[1].mode != 'OFFBOARD':
                    raise RuntimeError('Armed but mode is '+state[1].mode+'; no automatic mode override')
                print('FCU confirms armed + OFFBOARD. Mission owns takeoff; monitor TAKEOFF status.')
                return 0
        time.sleep(.05)
    raise RuntimeError('Arming ACK received but fresh armed+OFFBOARD not confirmed; no retry')


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (KeyboardInterrupt, Exception) as error:
        print('UAV ARM: '+str(error), file=sys.stderr)
        sys.exit(2)
