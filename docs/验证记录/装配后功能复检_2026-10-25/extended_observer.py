#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Bounded subscription-only supplement to the existing peripheral bench."""
from __future__ import print_function
import json
import math
import os
import socket
import sys
import time

directory = sys.argv[1]
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:11326', ROS_IP='127.0.0.1',
                  ROS_HOME=directory+'/observer-home', ROS_LOG_DIR=directory+'/observer-logs')
os.environ.pop('ROS_HOSTNAME', None)
os.environ.pop('ROS_NAMESPACE', None)
deadline = time.time()+20
while time.time() < deadline:
    sock = socket.socket()
    try:
        sock.connect(('127.0.0.1', 11326))
        break
    except socket.error:
        time.sleep(.1)
    finally:
        sock.close()
else:
    raise RuntimeError('Bench master unavailable')

import rospy
from sensor_msgs.msg import Image, LaserScan, BatteryState, Imu, FluidPressure
from std_msgs.msg import UInt32
from mavros_msgs.msg import State, StatusText
from diagnostic_msgs.msg import DiagnosticArray

rospy.init_node('assembly_subscription_observer', disable_signals=True)
result = dict(streams={}, samples={}, diagnostics={}, statustext=[], armed_seen=False)
arrivals = {}
stamps = {}
last_image = [None]
battery_voltages = []

def track(name, message):
    arrivals.setdefault(name, []).append(time.time())
    if hasattr(message, 'header'):
        stamps.setdefault(name, []).append(message.header.stamp.to_sec())

def image(message):
    track('image', message)
    last_image[0] = message
    result['samples']['image'] = dict(width=message.width, height=message.height,
        encoding=message.encoding, step=message.step, bytes=len(message.data))

def scan(message):
    track('scan', message)
    values = [v for v in message.ranges if not math.isnan(v) and not math.isinf(v)
              and message.range_min <= v <= message.range_max]
    result['samples']['scan'] = dict(points=len(message.ranges), valid=len(values),
        range_min=message.range_min, range_max=message.range_max,
        closest=min(values) if values else None, farthest=max(values) if values else None,
        angle_min=message.angle_min, angle_max=message.angle_max,
        scan_time=message.scan_time, frame=message.header.frame_id)

def state(message):
    track('state', message)
    result['armed_seen'] = result['armed_seen'] or message.armed
    result['samples']['state'] = dict(armed=message.armed, connected=message.connected,
        mode=message.mode, system_status=message.system_status)

def battery(message):
    track('battery', message)
    if not math.isnan(message.voltage):
        battery_voltages.append(message.voltage)
    result['samples']['battery'] = dict(voltage=message.voltage, current=message.current,
        percentage=message.percentage, present=message.present,
        cell_voltage=list(message.cell_voltage))

def imu(message):
    track('imu', message)
    result['samples']['imu'] = dict(acceleration=[message.linear_acceleration.x,
        message.linear_acceleration.y, message.linear_acceleration.z],
        angular_velocity=[message.angular_velocity.x, message.angular_velocity.y,
                          message.angular_velocity.z])

def diagnostics(message):
    track('diagnostics', message)
    for item in message.status:
        result['diagnostics'][item.name] = dict(level=item.level, message=item.message,
            hardware_id=item.hardware_id, values={v.key:v.value for v in item.values})

def text(message):
    track('statustext', message)
    result['statustext'].append(dict(severity=message.severity, text=message.text))

def satellites(message):
    track('satellites', message)
    result['samples']['satellites'] = message.data

def pressure(message):
    track('pressure', message)
    result['samples']['pressure'] = message.fluid_pressure

subscriptions = [
    rospy.Subscriber('/camera/image_raw', Image, image, queue_size=1, buff_size=8000000),
    rospy.Subscriber('/scan', LaserScan, scan, queue_size=5),
    rospy.Subscriber('/mavros/state', State, state),
    rospy.Subscriber('/mavros/battery', BatteryState, battery),
    rospy.Subscriber('/mavros/imu/data', Imu, imu),
    rospy.Subscriber('/diagnostics', DiagnosticArray, diagnostics),
    rospy.Subscriber('/mavros/statustext/recv', StatusText, text),
    rospy.Subscriber('/mavros/global_position/raw/satellites', UInt32, satellites),
    rospy.Subscriber('/mavros/imu/static_pressure', FluidPressure, pressure),
]
deadline = time.time()+28
while time.time() < deadline and not rospy.is_shutdown():
    time.sleep(.1)

for name, values in arrivals.items():
    differences = [b-a for a,b in zip(values,values[1:])]
    stream = dict(count=len(values), hz=(len(values)-1)/(values[-1]-values[0])
        if len(values)>1 else None, max_receive_gap=max(differences) if differences else None)
    if name in stamps:
        differences = [b-a for a,b in zip(stamps[name],stamps[name][1:])]
        stream.update(nonincreasing_stamp_pairs=sum(d<=0 for d in differences),
            first_stamp=stamps[name][0], last_stamp=stamps[name][-1])
    result['streams'][name] = stream
if battery_voltages:
    result['battery_observed_voltage_range'] = [min(battery_voltages),max(battery_voltages)]
message = last_image[0]
if message is not None:
    try:
        import cv2
        import numpy
        data = numpy.frombuffer(message.data, dtype=numpy.uint8).reshape(message.height, message.step)
        data = data[:, :message.width*3].reshape(message.height, message.width, 3)
        if message.encoding == 'rgb8':
            data = cv2.cvtColor(data, cv2.COLOR_RGB2BGR)
        elif message.encoding != 'bgr8':
            raise RuntimeError('Unexpected image encoding')
        if not cv2.imwrite(directory+'/camera-last-frame.jpg', data):
            raise RuntimeError('Image save failed')
        result['saved_frame'] = 'camera-last-frame.jpg'
    except Exception as error:
        result['frame_save_error'] = repr(error)
with open(directory+'/extended-results.json','w') as output:
    json.dump(result, output, indent=2, sort_keys=True)
print(json.dumps(result, indent=2, sort_keys=True))
rospy.signal_shutdown('Bounded observation completed')
