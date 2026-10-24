#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Bounded real-camera checks on an isolated master; no flight/servo nodes.

Source catkin devel/setup.bash first. All logs go to the specified temporary
directory. The caller retains evidence locally, then removes board test files.
"""
from __future__ import print_function
import json
import os
import signal
import socket
import subprocess
import sys
import time


def stop_process(process):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
    deadline = time.time() + 6
    while process.poll() is None and time.time() < deadline:
        time.sleep(.05)
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGTERM)
    return process.wait()


def main():
    directory = os.path.abspath(sys.argv[1])
    os.environ.update(ROS_MASTER_URI='http://127.0.0.1:11325',
                      ROS_IP='127.0.0.1', ROS_HOME=directory+'/ros-home',
                      ROS_LOG_DIR=directory+'/ros-logs')
    os.environ.pop('ROS_HOSTNAME', None)
    os.environ.pop('ROS_NAMESPACE', None)
    for path in (directory, directory+'/ros-home', directory+'/ros-logs'):
        if not os.path.isdir(path):
            os.makedirs(path)
    port = socket.socket()
    port.bind(('127.0.0.1', 11325))
    port.close()
    master_log = open(directory+'/master.log', 'w')
    master = subprocess.Popen(['roscore', '-p', '11325'], stdout=master_log,
                              stderr=subprocess.STDOUT, preexec_fn=os.setsid)
    camera = None
    results = []
    try:
        deadline = time.time()+12
        while time.time() < deadline:
            sock = socket.socket()
            try:
                sock.connect(('127.0.0.1', 11325))
                break
            except socket.error:
                time.sleep(.1)
            finally:
                sock.close()
        else:
            raise RuntimeError('Master startup failed')
        import rospy
        from sensor_msgs.msg import Image
        rospy.init_node('camera_lifecycle_observer', disable_signals=True)
        for index, fmt in enumerate(('mjpeg', 'yuyv', 'mjpeg', 'yuyv')):
            size = (1280, 720) if fmt == 'mjpeg' else (640, 480)
            seen = []
            def receive(message):
                seen.append((time.time(), message.width, message.height,
                             message.encoding, len(message.data), message.step))
            sub = rospy.Subscriber('/camera/image_raw', Image, receive,
                                   queue_size=3, buff_size=8000000)
            log = open(directory+'/camera-%s-%s.log' % (index, fmt), 'w')
            args = ['rosrun', 'usb_cam', 'usb_cam_node', '__name:=camera',
                    '_video_device:=/dev/video10', '_pixel_format:='+fmt,
                    '_image_width:='+str(size[0]), '_image_height:='+str(size[1]),
                    '_framerate:=30', '_io_method:=mmap']
            camera = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT,
                                      preexec_fn=os.setsid)
            deadline = time.time()+10
            while time.time() < deadline and camera.poll() is None and len(seen)<60:
                time.sleep(.05)
            count = len(seen)
            sample = seen[-1] if seen else None
            exit_code = stop_process(camera)
            camera = None
            sub.unregister()
            log.close()
            result = dict(format=fmt, size=size, frames=count, exit_code=exit_code,
                          sample=sample, passed=count>=60 and exit_code==0 and
                          sample[1:3]==size and sample[4]==sample[5]*sample[2])
            results.append(result)
            sys.stdout.write(json.dumps(result)+'\n')
            sys.stdout.flush()
            if not result['passed']:
                raise RuntimeError('Camera data/exit check failed')
        rospy.signal_shutdown('Camera checks complete')
    finally:
        if camera is not None:
            stop_process(camera)
        master_exit = stop_process(master)
        master_log.close()
        subprocess.call(['v4l2-ctl', '-d', '/dev/video10',
                         '--set-fmt-video=width=1280,height=720,pixelformat=MJPG',
                         '--set-parm=60'])
        output = dict(cases=results, master_exit=master_exit,
                      passed=len(results)==4 and all(r['passed'] for r in results) and master_exit==0)
        with open(directory+'/camera-lifecycle.json','w') as stream:
            json.dump(output, stream, indent=2)
        print(json.dumps(output, indent=2))


if __name__ == '__main__':
    main()
