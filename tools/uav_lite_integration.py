#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Private localhost ROS tests of lite recording and normal arming.

All FCU states, poses, setpoints and services are synthetic on port 11340.
Never starts MAVROS, sensors, servos, motors or a real flight command endpoint.
"""
from __future__ import print_function
import argparse
import json
import os
import signal
import socket
import subprocess
import sys
import threading
import time
import uav_log_lite as lite


def until(predicate, seconds, description):
    deadline = lite.steady()+seconds
    while lite.steady() < deadline:
        if predicate(): return
        time.sleep(.1)
    raise RuntimeError('Timed out: '+description)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output')
    args = parser.parse_args(); output = os.path.abspath(args.output)
    os.makedirs(output)
    if lite.active(): raise RuntimeError('An existing recorder is active; test refused')
    reservation = socket.socket(); reservation.bind(('127.0.0.1',11340)); reservation.close()
    os.environ.update(ROS_MASTER_URI='http://127.0.0.1:11340', ROS_IP='127.0.0.1',
                      ROS_LOG_DIR=output+'/ros-logs', UAV_LOG_ROOT=output+'/sessions')
    os.environ.pop('ROS_HOSTNAME', None)
    master = subprocess.Popen(['roscore','-p','11340'],
        stdout=open(output+'/master.log','w'), stderr=subprocess.STDOUT, preexec_fn=lite.child_session)
    report = dict(all_fcu_messages_synthetic=True, real_mavros=False, real_sensors=False,
                  master='127.0.0.1:11340', cases=[], passed=False)
    running = [True]; thread = None
    try:
        import rosgraph
        graph = rosgraph.Master('/lite_private_wait')
        def ready():
            try: graph.getPid(); return True
            except Exception: return False
        until(ready, 15, 'private master')
        import rospy
        from geometry_msgs.msg import PoseStamped
        from mavros_msgs.msg import State, PositionTarget
        from mavros_msgs.srv import CommandBool, CommandBoolResponse, SetMode, SetModeResponse, CommandTOL, CommandTOLResponse
        from std_msgs.msg import String
        rospy.init_node('lite_synthetic_source', disable_signals=True)
        state = dict(armed=False, mode='OFFBOARD', phase=0, ticks=0, arm_calls=[], mode_calls=[], takeoff_calls=[])
        pose_pub = rospy.Publisher('/mavros/local_position/pose', PoseStamped, queue_size=2)
        vision_pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=2)
        setpoint_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=2)
        state_pub = rospy.Publisher('/mavros/state', State, queue_size=2)
        health_pub = rospy.Publisher('/mission/readiness', String, queue_size=2)
        location_pub = rospy.Publisher('/location_node/status', String, queue_size=1, latch=True)
        location_pub.publish(String(data='READY'))
        for key,value in dict(enable_actuation=True, auto_start=False, target_hazard='flammable').items():
            rospy.set_param('/shoot_with_move_base_node/'+key,value)
        def arm(request):
            state['arm_calls'].append(request.value); state['armed'] = request.value; state['phase'] = 1
            return CommandBoolResponse(success=True, result=0)
        def mode(request):
            state['mode_calls'].append(request.custom_mode)
            return SetModeResponse(mode_sent=False)
        def takeoff(request):
            state['takeoff_calls'].append(True)
            return CommandTOLResponse(success=False, result=2)
        services = [rospy.Service('/mavros/cmd/arming', CommandBool, arm),
                    rospy.Service('/mavros/set_mode', SetMode, mode),
                    rospy.Service('/mavros/cmd/takeoff', CommandTOL, takeoff)]
        def publish():
            while running[0] and not rospy.is_shutdown():
                stamp = rospy.Time.now(); pose = PoseStamped(); pose.header.stamp = stamp; pose.pose.orientation.w = 1
                pose_pub.publish(pose); vision_pub.publish(pose)
                target = PositionTarget(); target.header.stamp = stamp; target.coordinate_frame = 1
                target.type_mask = 2552; setpoint_pub.publish(target)
                fcu = State(); fcu.header.stamp = stamp; fcu.connected = True
                fcu.armed = state['armed']; fcu.mode = state['mode']; state_pub.publish(fcu)
                health_pub.publish(String(data=json.dumps(dict(model_loaded=True, frame_fresh=True,
                    inference_ready=True, pose_fresh=True, fcu_fresh=True, tf_fresh=True,
                    actuation_enabled=True, flight_authority=state['armed'] and state['mode']=='OFFBOARD',
                    mission_aborted=False, phase=state['phase']))))
                state['ticks'] += 1; time.sleep(.02)
        thread = threading.Thread(target=publish); thread.daemon=True; thread.start()
        directory = lite.prepare('isolated-test'); lite.launch(directory)
        before = state['ticks']; time.sleep(5); lite.stop_active()
        summary = lite.report(directory)
        count = summary['topics'].get('/mavros/setpoint_raw/local',{}).get('count',0)
        if count < 100 or not summary['recording_closed_without_writer_errors']:
            raise RuntimeError('Synthetic control recording incomplete')
        if state['ticks'] <= before or not ready(): raise RuntimeError('Recorder stopped the source or master')
        report['cases'].append(dict(case='record_close_report', passed=True, control_messages=count, session=directory))
        directory = lite.prepare('isolated-fault'); lite.launch(directory)
        until(lambda: os.path.isfile(directory+'/status.json'), 6, 'writer identity')
        writer = json.loads(lite.read(directory+'/status.json'))['writer']
        if lite.identity(writer['pid']) != writer: raise RuntimeError('Writer identity changed')
        before = state['ticks']; os.kill(writer['pid'], signal.SIGTERM)
        until(lambda: os.path.isfile(directory+'/closed.json') and not lite.active(), 8, 'writer fault closure')
        closed = json.loads(lite.read(directory+'/closed.json'))
        if not closed['errors'] or state['ticks'] <= before or not ready():
            raise RuntimeError('Writer failure was not independently contained')
        report['cases'].append(dict(case='writer_failure_isolation', passed=True,
                                   errors=closed['errors'], source_still_running=True))
        helper = os.path.expanduser('~/.local/share/uav/uav_arm.py')
        def run_arm(name, extra, expected):
            with open(output+'/'+name+'.stdout.log','w') as stream:
                child = subprocess.Popen([sys.executable, helper]+extra,
                    stdout=stream, stderr=subprocess.STDOUT, preexec_fn=lite.child_session)
            until(lambda: child.poll() is not None, 16, name)
            if child.returncode != expected: raise RuntimeError(name+' unexpected code '+str(child.returncode))
        run_arm('check_only', ['--check'], 0)
        if state['arm_calls']: raise RuntimeError('--check sent arming')
        run_arm('normal_offboard_arm', [], 0)
        if state['arm_calls'] != [True] or state['mode_calls'] or state['takeoff_calls']:
            raise RuntimeError('Unexpected arming/mode/takeoff service calls')
        report['cases'].append(dict(case='offboard_normal_arm_only', passed=True,
            arm_calls=state['arm_calls'][:], mode_calls=state['mode_calls'][:], takeoff_calls=state['takeoff_calls'][:]))
        run_arm('already_armed_refused', [], 2)
        state.update(armed=False, mode='AUTO.TAKEOFF', phase=0)
        run_arm('auto_takeoff_refused', [], 2)
        if state['arm_calls'] != [True]: raise RuntimeError('Refused cases still sent arming')
        report['cases'].append(dict(case='wrong_mode_and_repeated_arm_refused', passed=True))
        report['passed'] = True
    except Exception as error:
        report['error'] = str(error)
    finally:
        running[0] = False
        if thread: thread.join(2)
        if lite.active(): lite.stop_active()
        if master.poll() is None:
            os.killpg(master.pid, signal.SIGINT)
            try: until(lambda: master.poll() is not None, 10, 'private master cleanup')
            except RuntimeError:
                os.killpg(master.pid, signal.SIGTERM); master.wait()
        lite.atomic_json(output+'/result.json', report)
        print(json.dumps(report, sort_keys=True, indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
