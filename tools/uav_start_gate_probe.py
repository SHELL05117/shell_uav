#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Private ROS startup-gate lifetime/CPU regression; no hardware or services.

Compare two uav_start.py files against identical 1280x720 RGB, 30 Hz traffic.
Owns localhost:11341 only. All FCU/pose/TF inputs are synthetic and disarmed.
"""
from __future__ import print_function
import argparse
import imp
import json
import os
import signal
import socket
import subprocess
import sys
import time


def ticks(pid):
    with open('/proc/%d/stat' % pid) as stream:
        fields = stream.read().split(') ', 1)[1].split()
    return int(fields[11])+int(fields[12])


def stop(process):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=8)
    except TypeError:  # Python 2 Popen.wait has no timeout.
        deadline = time.time()+8
        while process.poll() is None and time.time()<deadline:
            time.sleep(.1)
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGKILL)
        process.wait()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output')
    parser.add_argument('--before')
    parser.add_argument('--after')
    parser.add_argument('--worker')
    parser.add_argument('--failure', action='store_true')
    args = parser.parse_args()
    directory = os.path.abspath(args.output)
    if not os.path.isdir(directory): os.makedirs(directory)
    os.environ.update(ROS_MASTER_URI='http://127.0.0.1:11341', ROS_IP='127.0.0.1',
                      ROS_LOG_DIR=directory+'/ros-logs')
    os.environ.pop('ROS_HOSTNAME', None)
    if args.worker:
        module = imp.load_source('gate_under_test', args.worker)
        try:
            module.wait_ready(2 if args.failure else 10)
            result = 'ready'
        except RuntimeError as error:
            if not args.failure: raise
            result = str(error)
        with open(directory+'/worker-ready.json', 'w') as stream:
            json.dump(dict(result=result), stream)
        time.sleep(60)
        return
    reserve = socket.socket()
    reserve.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    reserve.bind(('127.0.0.1', 11341)); reserve.close()
    streams = []; owned = []
    def start(name, command):
        stream = open(directory+'/'+name+'.log', 'w'); streams.append(stream)
        process = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT,
                                   preexec_fn=os.setsid)
        owned.append(process)
        return process
    master = start('master', ['roscore', '-p', '11341'])
    results = []
    try:
        time.sleep(3)
        import rospy
        import tf2_ros
        from geometry_msgs.msg import PoseStamped, TransformStamped
        from sensor_msgs.msg import Image, Imu, LaserScan
        from nav_msgs.msg import OccupancyGrid
        from mavros_msgs.msg import State
        from std_msgs.msg import String
        rospy.init_node('synthetic_gate_inputs', disable_signals=True)
        messages = [('state', '/mavros/state', State(connected=True, armed=False, mode='POSCTL')),
                    ('imu', '/mavros/imu/data', Imu()), ('scan', '/scan', LaserScan()),
                    ('map', '/map', OccupancyGrid()), ('location', '/location_node/status', String('READY')),
                    ('vision', '/mavros/vision_pose/pose', PoseStamped()),
                    ('camera', '/camera/image_raw', Image(height=720, width=1280, encoding='rgb8',
                                                        step=3840, data=b'\0'*(1280*720*3)))]
        messages[3][2].info.width = messages[3][2].info.height = 1
        publishers = [(key, rospy.Publisher(topic, type(message), queue_size=1), message)
                      for key, topic, message in messages]
        broadcaster = tf2_ros.TransformBroadcaster()
        transform = TransformStamped(); transform.header.frame_id = 'map'
        transform.child_frame_id = 'base_link'; transform.transform.rotation.w = 1
        omit_vision = [False]
        def publish(event):
            stamp = rospy.Time.now()
            for key, publisher, message in publishers:
                if key == 'vision' and omit_vision[0]: continue
                if hasattr(message, 'header'): message.header.stamp = stamp
                publisher.publish(message)
            transform.header.stamp = stamp; broadcaster.sendTransform(transform)
        timer = rospy.Timer(rospy.Duration(1./30), publish)
        camera = publishers[-1][1]
        for label, path, failure in [('before', args.before, False), ('after', args.after, False),
                                     ('after_timeout', args.after, True)]:
            marker = directory+'/worker-ready.json'
            if os.path.exists(marker): os.unlink(marker)
            omit_vision[0] = failure
            command = [sys.executable, '-u', __file__, directory, '--worker', path]
            if failure: command.append('--failure')
            process = start(label, command)
            deadline = time.time()+16
            while not os.path.exists(marker) and time.time()<deadline and process.poll() is None:
                time.sleep(.1)
            if not os.path.exists(marker): raise RuntimeError(label+' gate did not finish')
            time.sleep(1)
            start_ticks = ticks(process.pid); start_time = time.time()
            connections = camera.get_num_connections()
            time.sleep(10)
            cpu = 100.*(ticks(process.pid)-start_ticks)/os.sysconf('SC_CLK_TCK')/(time.time()-start_time)
            with open(marker) as stream: outcome = json.load(stream)
            result = dict(case=label, cpu_one_core_percent=cpu,
                          camera_connections_after_gate=connections, outcome=outcome)
            results.append(result); print(json.dumps(result)); sys.stdout.flush()
            stop(process)
        timer.shutdown()
        assert results[0]['camera_connections_after_gate'] == 1
        assert all(row['camera_connections_after_gate'] == 0 for row in results[1:])
        assert results[1]['outcome']['result'] == 'ready'
        assert 'vision' in results[2]['outcome']['result']
        with open(directory+'/results.json', 'w') as stream:
            json.dump(dict(private_master=os.environ['ROS_MASTER_URI'], cases=results,
                           hardware=False, flight_services=False, passed=True), stream, indent=2)
    finally:
        for process in reversed(owned):
            if process.poll() is None: stop(process)
        for stream in streams: stream.close()


if __name__ == '__main__':
    main()
