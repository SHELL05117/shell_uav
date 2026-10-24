#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Observe real camera, lidar and MAVROS telemetry on an isolated ROS master.

Does not publish control topics or invoke flight services. GPS reception and
GPS position fix are separate outcomes. Requires sourced catkin environment.
"""
from __future__ import print_function
import json
import math
import os
import signal
import socket
import subprocess
import sys
import time
from rk_camera_lifecycle_check import stop_process


def main():
    directory = os.path.abspath(sys.argv[1])
    os.environ.update(ROS_MASTER_URI='http://127.0.0.1:11326', ROS_IP='127.0.0.1',
                      ROS_HOME=directory+'/ros-home', ROS_LOG_DIR=directory+'/ros-logs')
    os.environ.pop('ROS_HOSTNAME', None)
    os.environ.pop('ROS_NAMESPACE', None)
    for path in (directory, directory+'/ros-home', directory+'/ros-logs'):
        if not os.path.isdir(path): os.makedirs(path)
    reservation = socket.socket()
    reservation.bind(('127.0.0.1', 11326))
    reservation.close()
    processes = []
    logs = []
    def start(name, args):
        output = open(directory+'/'+name+'.log', 'w')
        process = subprocess.Popen(args, stdout=output, stderr=subprocess.STDOUT, preexec_fn=os.setsid)
        processes.append((name, process))
        logs.append(output)
    result = dict(streams={}, samples={}, exits={})
    times = {name: [] for name in ('image', 'scan', 'state', 'imu', 'gps')}
    armed_seen = [False]
    try:
        start('master', ['roscore', '-p', '11326'])
        deadline = time.time()+15
        while time.time() < deadline:
            sock = socket.socket()
            try:
                sock.connect(('127.0.0.1', 11326))
                break
            except socket.error: time.sleep(.1)
            finally: sock.close()
        else: raise RuntimeError('Master startup failed')
        import rospy
        from sensor_msgs.msg import Image, LaserScan, Imu, NavSatFix
        from mavros_msgs.msg import State
        rospy.init_node('peripheral_bench_observer', disable_signals=True)
        def image(m):
            times['image'].append(time.time())
            result['samples']['image'] = dict(width=m.width, height=m.height, encoding=m.encoding, size_consistent=len(m.data)==m.height*m.step)
        def scan(m):
            times['scan'].append(time.time())
            result['samples']['scan'] = dict(points=len(m.ranges), valid=sum(1 for x in m.ranges if not math.isnan(x) and not math.isinf(x) and m.range_min<=x<=m.range_max))
        def state(m):
            times['state'].append(time.time())
            armed_seen[0] = armed_seen[0] or m.armed
            result['samples']['state'] = dict(connected=m.connected, armed=m.armed, mode=m.mode)
        def imu(m):
            times['imu'].append(time.time())
            q=m.orientation
            result['samples']['imu'] = dict(frame=m.header.frame_id, quaternion_norm=math.sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w), acceleration=[m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z])
        def gps(m):
            times['gps'].append(time.time())
            result['samples']['gps'] = dict(status=m.status.status, latitude=m.latitude, longitude=m.longitude, altitude=m.altitude, position_fix=m.status.status>=0)
        subscribers = [rospy.Subscriber('/camera/image_raw', Image, image, queue_size=3, buff_size=8000000), rospy.Subscriber('/scan', LaserScan, scan, queue_size=10), rospy.Subscriber('/mavros/state', State, state), rospy.Subscriber('/mavros/imu/data', Imu, imu, queue_size=50), rospy.Subscriber('/mavros/global_position/raw/fix', NavSatFix, gps, queue_size=20)]
        start('mavros', ['roslaunch', 'location', 'rk_mavros_bench.launch'])
        start('camera', ['roslaunch', 'usb_cam', 'rk_usb_camera.launch'])
        start('lidar', ['rosrun','rplidar_ros','rplidarNode','_serial_port:=/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_909394b23877ed11907beefafdf7b791-if00-port0','_serial_baudrate:=1000000','_scan_frequency:=10.0','_angle_compensate:=true'])
        deadline=time.time()+25
        while time.time()<deadline and not rospy.is_shutdown():
            if armed_seen[0]: raise RuntimeError('Armed state observed; stopping telemetry check')
            for name, process in processes:
                if process.poll() is not None: raise RuntimeError(name+' exited during observation')
            time.sleep(.1)
        result['receive_checks'] = dict(image=len(times['image'])>=120 and result['samples'].get('image',{}).get('size_consistent',False), scan=len(times['scan'])>=50 and result['samples'].get('scan',{}).get('valid',0)>0, connected_unarmed=result['samples'].get('state',{}).get('connected',False) and not armed_seen[0], imu=len(times['imu'])>=200 and abs(result['samples'].get('imu',{}).get('quaternion_norm',0)-1)<.01, gps=len(times['gps'])>=20)
        result['gps_position_fix_passed']=result['samples'].get('gps',{}).get('position_fix',False)
        result['observed_topics']=rospy.get_published_topics()
        rospy.signal_shutdown('Bounded telemetry observation complete')
    finally:
        for name, process in reversed(processes): result['exits'][name]=stop_process(process)
        for output in logs: output.close()
        for name, values in times.items():
            result['streams'][name] = dict(count=len(values), hz=(len(values)-1)/(values[-1]-values[0]) if len(values)>1 else None)
        result['passed_receive_and_exit'] = bool(result.get('receive_checks')) and all(result.get('receive_checks',{}).values()) and all(code==0 for code in result['exits'].values())
        with open(directory+'/bench-results.json','w') as output: json.dump(result,output,indent=2,sort_keys=True)
        print(json.dumps(result,indent=2,sort_keys=True))
        subprocess.call(['v4l2-ctl','-d','/dev/video10','--set-fmt-video=width=1280,height=720,pixelformat=MJPG','--set-parm=60'])
    if not result['passed_receive_and_exit']: raise SystemExit(1)


if __name__ == '__main__':
    main()
