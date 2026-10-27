#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""SHELL two-terminal startup. Ground modes never enable mission actuation."""
from __future__ import print_function
import argparse
import glob
import json
import os
import signal
import socket
import subprocess
import sys
import time
import threading

children = []
log_directory = None
log_streams = []


def mirror_log(path):
    # Mirroring can block on a disconnected terminal; child processes write
    # directly to files and never depend on this expendable daemon thread.
    with open(path,'rb') as stream:
        while True:
            data=stream.read(4096)
            if data:
                try: os.write(sys.stdout.fileno(),data)
                except OSError: return
            else: time.sleep(.1)


def start(args):
    print('+ '+ ' '.join(args)); sys.stdout.flush()
    if log_directory:
        path=os.path.join(log_directory,'launch-%02d-%d.stdout.log'%(len(children),os.getpid()))
        stream=open(path,'ab'); log_streams.append(stream)
        p = subprocess.Popen(args, preexec_fn=os.setsid,stdout=stream,stderr=subprocess.STDOUT)
        thread=threading.Thread(target=mirror_log,args=(path,)); thread.daemon=True; thread.start()
    else:
        p = subprocess.Popen(args, preexec_fn=os.setsid)
    children.append(p)
    return p


def stop():
    for p in reversed(children):
        if p.poll() is None:
            os.killpg(p.pid, signal.SIGINT)
    deadline = time.time()+12
    while time.time()<deadline and any(p.poll() is None for p in children):
        time.sleep(.1)
    for p in reversed(children):
        if p.poll() is None:
            os.killpg(p.pid, signal.SIGTERM)
    deadline = time.time()+3
    while time.time()<deadline and any(p.poll() is None for p in children):
        time.sleep(.1)
    for p in reversed(children):
        if p.poll() is None:
            os.killpg(p.pid, signal.SIGKILL)
        p.wait()


def device(variable, pattern):
    value = os.environ.get(variable)
    if value:
        if not os.path.exists(value):
            raise RuntimeError(variable+' does not exist: '+value)
        return value
    values = glob.glob(pattern)
    if len(values)!=1:
        raise RuntimeError('Expected one device for '+variable+'; found '+str(values)+
                           '. Set it in ~/.config/uav/env.sh')
    return values[0]


def active_nodes():
    import rosgraph
    try:
        state = rosgraph.Master('/uav_start_check').getSystemState()
        return sorted(set(n for group in state for topic,nodes in group for n in nodes))
    except socket.error:
        return []


def wait_ready(timeout=65):
    import rospy
    import tf2_ros
    from mavros_msgs.msg import State
    from sensor_msgs.msg import Imu, LaserScan
    from nav_msgs.msg import OccupancyGrid
    from std_msgs.msg import String
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import Image
    if not rospy.core.is_initialized():
        rospy.init_node('uav_start_gate', anonymous=True, disable_signals=True)
    values = {}
    def receive(key):
        def cb(msg):
            values[key] = (time.time(),msg)
        return cb
    subs = [rospy.Subscriber(topic, kind, receive(key), queue_size=1) for key,topic,kind in [
        ('state','/mavros/state',State),('imu','/mavros/imu/data',Imu),
        ('scan','/scan',LaserScan),('map','/map',OccupancyGrid),
        ('location','/location_node/status',String),
        ('vision','/mavros/vision_pose/pose',PoseStamped),('camera','/camera/image_raw',Image)]]
    buffer=tf2_ros.Buffer(); listener=tf2_ros.TransformListener(buffer)
    deadline=time.time()+timeout; pending=[]
    while time.time()<deadline:
        if any(p.poll() is not None for p in children):
            raise RuntimeError('A launched process exited before readiness')
        now=time.time()
        pending=[key for key in ('state','imu','scan','vision','camera') if key not in values or now-values[key][0]>2]
        state=values.get('state',(None,None))[1]
        if state and state.armed:
            raise RuntimeError('Ground startup refused: FCU armed')
        if not state or not state.connected: pending.append('FCU connection')
        loc=values.get('location',(None,None))[1]
        if not loc or loc.data!='READY': pending.append('location READY')
        m=values.get('map',(None,None))[1]
        if not m or not m.info.width or not m.info.height: pending.append('map')
        if not buffer.can_transform('map','base_link',rospy.Time(0),rospy.Duration(.1)):
            pending.append('map -> base_link')
        if not pending:
            print('READY: unarmed FCU, live IMU/scan, map, location and TF.'); return
        time.sleep(.2)
    raise RuntimeError('Startup timed out; missing '+', '.join(pending))


def navigation(args, flight=False):
    if flight and args.target not in ('explosive','flammable','toxic'):
        raise RuntimeError('mission requires --target explosive|flammable|toxic')
    start(['roslaunch','navigation_2d','navigation_2d.launch','detect_backend:=rknn',
           'target_hazard:='+args.target,'stage:='+str(args.stage),
           'field_yaw_deg:='+str(args.yaw),'auto_start:=false',
           'enable_actuation:='+str(flight).lower(),'use_real_servos:='+str(flight).lower()])


def main():
    global log_directory
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode',choices=['a','b','c','check','mission','status','bridge'])
    parser.add_argument('--target',choices=['explosive','flammable','toxic'],default='')
    parser.add_argument('--stage',type=int,choices=[1,2],default=1)
    parser.add_argument('--yaw',type=float,default=-90)
    args=parser.parse_args()
    print('ROS_MASTER_URI='+os.environ['ROS_MASTER_URI']+' ROS_IP='+os.environ['ROS_IP'])
    import rospkg
    root=rospkg.RosPack().get_path('location')
    nodes=active_nodes()
    if args.mode=='status':
        if not nodes: raise RuntimeError('No running ROS graph')
        wait_ready(8)
        print('Nodes: '+', '.join(nodes))
        if '/shoot_with_move_base_node' in nodes:
            for topic in ['/mission/readiness','/mission/status']:
                if subprocess.call(['timeout','4','rostopic','echo','-n','1',topic])!=0:
                    raise RuntimeError('Mission feedback unavailable: '+topic)
        else:
            print('Localization/camera ready. Mission is not running.')
        return
    if args.mode in ('a','check'):
        if [n for n in nodes if n!='/rosout']:
            raise RuntimeError('Existing ROS nodes found; do not overlap A/check: '+str(nodes))
        if time.localtime().tm_year<2026:
            raise RuntimeError('Set system clock before ROS startup')
        fcu=device('UAV_FCU','/dev/serial/by-id/*PX4*')
        lidar=device('UAV_LIDAR','/dev/serial/by-id/*CP2102*')
        camera=device('UAV_CAMERA','/dev/v4l/by-id/*Camera*video-index0')
        profile=os.environ.get('UAV_HEIGHT_PROFILE',root+'/param/rk_height_reference_20261027.yaml')
        if os.environ.get('UAV_LOG_DISABLED','0')!='1':
            try:
                import uav_log
                if uav_log.active(): raise RuntimeError('Previous recorder still active')
                log_directory=uav_log.prepare(args.mode)
                os.environ['ROS_LOG_DIR']=log_directory+'/ros-logs'
                print('UAV logs: '+log_directory)
            except Exception as error:
                print('UAV LOGGING UNAVAILABLE: '+str(error),file=sys.stderr)
        start(['roslaunch','location','location.launch','open_rviz:=false',
               'fcu_url:='+fcu+':115200','lidar_serial_port:='+lidar,'height_profile:='+profile])
        deadline=time.time()+12
        while time.time()<deadline and not active_nodes(): time.sleep(.2)
        if not active_nodes(): raise RuntimeError('ROS master startup timed out')
        if log_directory:
            try: uav_log.launch(log_directory)
            except Exception as error:
                print('UAV LOGGING UNAVAILABLE: '+str(error),file=sys.stderr)
        start(['roslaunch','usb_cam','rk_usb_camera.launch','video_device:='+camera,
               'mjpeg_backend:='+os.environ.get('UAV_CAMERA_BACKEND','mpp_rga')])
        start(['roslaunch','location','lichtblick_bridge_readonly.launch','jpeg_relay:=true',
               'start_bridge:='+os.environ.get('UAV_MONITOR_BRIDGE','false')])
        wait_ready()
        print('Foxglove native ROS 1: '+os.environ['ROS_MASTER_URI']+
              '; set client ROS_HOSTNAME to the computer LAN IP; map fixed/display frame=map')
        if args.mode=='check':
            navigation(args)
            if log_directory:
                try: uav_log.attach('check',vars(args))
                except Exception as error: print('UAV LOG SNAPSHOT FAILED: '+str(error),file=sys.stderr)
    elif args.mode in ('c','mission'):
        if not nodes: raise RuntimeError('Start uav a first')
        if '/shoot_with_move_base_node' in nodes or '/move_base' in nodes:
            raise RuntimeError('Navigation already running')
        wait_ready(12)
        try:
            import uav_log
            log_directory=uav_log.attach(args.mode,vars(args))
            if log_directory: os.environ['ROS_LOG_DIR']=log_directory+'/ros-logs'
            else: print('UAV WARNING: no active flight recorder',file=sys.stderr)
        except Exception as error:
            print('UAV LOG ATTACH FAILED: '+str(error),file=sys.stderr)
        navigation(args,args.mode=='mission')
    elif args.mode=='bridge':
        if not nodes: raise RuntimeError('Start data sources first')
        if '/foxglove_bridge' in nodes: raise RuntimeError('Bridge already running')
        start(['roslaunch','location','lichtblick_bridge_readonly.launch',
               'jpeg_relay:='+str('/monitor_jpeg' not in nodes).lower()])
    elif args.mode=='b':
        if '/camera' in nodes: raise RuntimeError('Camera already running (included in A/check)')
        start(['roslaunch','usb_cam','rk_usb_camera.launch',
               'mjpeg_backend:='+os.environ.get('UAV_CAMERA_BACKEND','mpp_rga')])
    warned = set()
    while True:
        exited = [p for p in children if p.poll() is not None]
        if exited and args.mode != 'a':
            raise RuntimeError('A child exited; stopping this command group')
        # A also supplies the flight position source: a display/camera failure
        # must not automatically remove the remaining localization processes.
        for p in exited:
            if p.pid not in warned:
                print('UAV: child %d exited (%d); inspect logs, remaining A sources kept running' % (p.pid,p.returncode),file=sys.stderr)
                warned.add(p.pid)
        time.sleep(.3)


if __name__=='__main__':
    signal.signal(signal.SIGTERM,lambda signum,frame: sys.exit(0))
    try:
        main()
    except KeyboardInterrupt:
        pass
    except Exception as error:
        print('UAV: '+str(error),file=sys.stderr)
        sys.exit(2)
    finally:
        stop()
