#!/usr/bin/env python2
"""Bounded bridge-only and /proc occupancy integration check, without hardware IO."""
from __future__ import print_function
import json
import os
import signal
import socket
import subprocess
import tempfile
import time

os.environ.update(ROS_MASTER_URI='http://127.0.0.1:11335', ROS_IP='127.0.0.1')
os.environ.pop('ROS_HOSTNAME', None)
children = []
report = dict(checks={}, errors=[], hardware_opened=False)


def probe(arguments):
    child = subprocess.Popen(arguments, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    output, error = child.communicate()
    return child.returncode, output, error


try:
    # The caller must source ROS first. All children are created and owned here.
    for port in (11335, 8765):
        connection = socket.socket()
        connection.settimeout(.2)
        occupied = connection.connect_ex(('127.0.0.1', port)) == 0
        connection.close()
        if occupied:
            raise RuntimeError('Port occupied: '+str(port))
    for name, command in [('master', ['roscore', '-p', '11335']),
                           ('bridge', ['bash', '/home/jcfh/start_lichtblick_monitor.sh', 'bridge'])]:
        logfile = open('/home/jcfh/lichtblick-check-'+name+'.log', 'w')
        child = subprocess.Popen(command, stdout=logfile, stderr=subprocess.STDOUT, preexec_fn=os.setsid)
        children.append((name, child, logfile))
        deadline = time.time()+15
        while time.time() < deadline:
            if child.poll() is not None:
                raise RuntimeError(name+' exited during startup')
            connection = socket.socket()
            connection.settimeout(.2)
            ready = connection.connect_ex(('127.0.0.1', 11335 if name=='master' else 8765)) == 0
            connection.close()
            if ready:
                break
            time.sleep(.1)
        else:
            raise RuntimeError(name+' startup timeout')
    import rospy
    rospy.init_node('lichtblick_bridge_check', disable_signals=True)
    time.sleep(1)
    report['nodes'] = probe(['rosnode', 'list'])[1].splitlines()
    report['parameters'] = {name:rospy.get_param('/foxglove_bridge/'+name) for name in
                           ['capabilities','client_topic_whitelist','param_whitelist','service_whitelist']}
    report['checks']['bridge_relay_attached'] = '/monitor_jpeg' in report['nodes']
    report['checks']['no_hardware_nodes'] = not any(name in report['nodes'] for name in
        ['/mavros','/camera','/detectnet','/rplidarNode','/cartographer_node','/shoot_with_move_base_node'])
    report['checks']['readonly_parameters'] = all(value==[] for value in report['parameters'].values())
    report['checks']['duplicate_ground_refused'] = probe(['bash','/home/jcfh/start_lichtblick_monitor.sh','ground'])[0] == 2
    helper = ['sudo','-n','python2','/home/jcfh/monitor_device_handles.py']
    devices = ['/dev/ttyACM0','/dev/ttyUSB0','/dev/v4l/by-id/usb-RYS_USB_Camera_200901010001-video-index0']
    code, output, error = probe(helper+devices)
    report['device_idle'] = json.loads(output)
    report['checks']['real_devices_idle'] = code==0 and report['device_idle']['passed']
    fd, name = tempfile.mkstemp(prefix='lichtblick-fd-check-')
    try:
        code, output, error = probe(helper+[name])
        report['dummy_busy'] = json.loads(output)
        report['checks']['open_regular_file_detected'] = code==2 and any(
            item['pid']==os.getpid() for item in report['dummy_busy']['busy'])
        report['checks']['missing_path_refused'] = probe(helper+[name+'.missing'])[0]==2
    finally:
        os.close(fd)
        os.unlink(name)
except BaseException as error:
    report['errors'].append(str(error))
finally:
    for name, child, logfile in reversed(children):
        if child.poll() is None:
            os.killpg(child.pid,signal.SIGINT)
        deadline=time.time()+15
        while child.poll() is None and time.time()<deadline:
            time.sleep(.1)
        if child.poll() is None:
            os.killpg(child.pid,signal.SIGTERM)
            report['errors'].append(name+' needed SIGTERM')
        child.wait()
        logfile.close()
    report['passed'] = bool(report['checks']) and all(report['checks'].values()) and not report['errors']
    with open('/home/jcfh/lichtblick-runtime-check.json','w') as target:
        json.dump(report,target,indent=2,sort_keys=True)
    print(json.dumps(report,indent=2))
raise SystemExit(0 if report['passed'] else 1)
