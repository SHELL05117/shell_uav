#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Isolated synthetic bridge check; never opens hardware or flight services.

Run with sourced catkin devel/setup.bash: python2 this_script.py OUTPUT_DIR.
The test owns only its child groups and ROS master at localhost:11329.
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


def stop(process):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
    deadline=time.time()+6
    while process.poll() is None and time.time()<deadline:
        time.sleep(.05)
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGTERM)
    return process.wait()


def main():
    directory=os.path.abspath(sys.argv[1])
    if not os.path.isdir(directory): os.makedirs(directory)
    os.environ.update(ROS_MASTER_URI='http://127.0.0.1:11329',ROS_IP='127.0.0.1',
                      ROS_HOME=directory+'/ros-home',ROS_LOG_DIR=directory+'/ros-logs')
    os.environ.pop('ROS_HOSTNAME',None)
    reservation=socket.socket(); reservation.bind(('127.0.0.1',11329)); reservation.close()
    processes=[]; logs=[]
    def start(name,args):
        stream=open(directory+'/'+name+'.log','w'); logs.append(stream)
        process=subprocess.Popen(args,stdout=stream,stderr=subprocess.STDOUT,preexec_fn=os.setsid)
        processes.append((name,process))
        return process
    master=start('master',['roscore','-p','11329'])
    result=dict(cases=[],exits={})
    try:
        deadline=time.time()+15
        while time.time()<deadline:
            sock=socket.socket()
            try: sock.connect(('127.0.0.1',11329)); break
            except socket.error: time.sleep(.1)
            finally: sock.close()
        else: raise RuntimeError('Isolated master unavailable')
        import rospy
        import tf2_ros
        from geometry_msgs.msg import TransformStamped,PoseStamped
        from sensor_msgs.msg import Imu,Range
        from std_msgs.msg import String
        rospy.init_node('localization_guard_observer',disable_signals=True)
        received=[]; statuses=[]
        def pose(message): received.append((time.time(),message))
        def status(message): statuses.append((time.time(),message.data))
        subscribers=[rospy.Subscriber('/location_probe/pose',PoseStamped,pose,queue_size=20),
                     rospy.Subscriber('/location_guard_probe/status',String,status)]
        imu_pub=rospy.Publisher('/location_probe/imu',Imu,queue_size=5)
        range_pub=rospy.Publisher('/location_probe/range',Range,queue_size=5)
        broadcaster=tf2_ros.TransformBroadcaster()
        args=['rosrun','location','location_node','__name:=location_guard_probe',
              '_tf_mini_type:=topic','_height_reference_calibrated:=true',
              '_range_topic:=/location_probe/range','_imu_topic:=/location_probe/imu',
              '_tf_timeout:=0.2','_range_timeout:=0.2','_imu_timeout:=0.2',
              '/mavros/vision_pose/pose:=/location_probe/pose']
        bridge=start('bridge',args)

        def phase(name,duration=0.9,tf_enabled=True,range_enabled=True,imu_enabled=True,
                  range_value=.59,pitch=0.0,freeze=None,expected_status=None,expected_z=None):
            started=time.time(); frozen=rospy.Time.now()
            while time.time()-started<duration:
                if bridge.poll() is not None: raise RuntimeError('Bridge exited during '+name)
                stamp=rospy.Time.now()
                if tf_enabled:
                    transform=TransformStamped(); transform.header.frame_id='odom'
                    transform.child_frame_id='base_link'; transform.header.stamp=frozen if freeze=='tf' else stamp
                    transform.transform.rotation.w=1.0
                    broadcaster.sendTransform(transform)
                if imu_enabled:
                    message=Imu(); message.header.frame_id='base_link'
                    message.header.stamp=frozen if freeze=='imu' else stamp
                    message.orientation.y=math.sin(pitch/2); message.orientation.w=math.cos(pitch/2)
                    imu_pub.publish(message)
                if range_enabled:
                    message=Range(); message.header.frame_id='tfmini_link'
                    message.header.stamp=frozen if freeze=='range' else stamp
                    message.min_range=.05; message.max_range=6.0; message.range=range_value
                    range_pub.publish(message)
                time.sleep(.025)
            cutoff=started+.4
            observed=[m for when,m in received if when>=cutoff]
            final_status=statuses[-1][1] if statuses else None
            passed=True
            if expected_status is not None: passed=passed and final_status==expected_status and not observed
            if expected_z is not None:
                passed=passed and len(observed)>=4 and all(abs(m.pose.position.z-expected_z)<.001 for m in observed)
            # Published time may be old (the real sample time), never future.
            passed=passed and all(m.header.stamp.to_sec()<=rospy.Time.now().to_sec()+.02 for m in observed)
            case=dict(name=name,passed=passed,poses=len(observed),status=final_status,
                      z=observed[-1].pose.position.z if observed else None)
            result['cases'].append(case); print(json.dumps(case)); sys.stdout.flush()
            if not passed: raise RuntimeError('Synthetic case failed: '+name)

        phase('missing_tf',duration=1.2,tf_enabled=False,range_enabled=False,imu_enabled=False,expected_status='WAIT_TF')
        phase('missing_range',range_enabled=False,expected_status='WAIT_RANGE')
        phase('level_height',expected_z=.5)
        phase('bad_range_expires',range_value=.002,expected_status='RANGE_STALE')
        phase('range_recovery',expected_z=.5)
        phase('cached_range_expires',freeze='range',expected_status='RANGE_STALE')
        phase('cached_range_recovery',expected_z=.5)
        phase('cached_tf_expires',freeze='tf',expected_status='TF_STALE')
        phase('tf_recovery',expected_z=.5)
        phase('cached_imu_expires',freeze='imu',expected_status='IMU_STALE')
        phase('imu_recovery',expected_z=.5)
        angle=20*math.pi/180
        phase('tilt_projection',range_value=.59/math.cos(angle),pitch=angle,expected_z=.5)
        phase('excess_tilt_blocks',range_value=1.18,pitch=math.pi/3,expected_status='HEIGHT_GEOMETRY_OR_LIMIT')
        result['exits']['bridge']=stop(bridge)
        args=[x if x!='_height_reference_calibrated:=true' else '_height_reference_calibrated:=false' for x in args]
        bridge=start('uncalibrated-bridge',args)
        phase('uncalibrated_reference_blocks',duration=1.4,expected_status='HEIGHT_REFERENCE_UNCALIBRATED')
        result['exits']['uncalibrated-bridge']=stop(bridge)
        bridge=start('height-disabled-bridge',args+['_use_tf_mini:=false'])
        phase('disabled_height_blocks_slam_z',duration=1.4,expected_status='HEIGHT_DISABLED')
        rospy.signal_shutdown('Synthetic checks completed')
    finally:
        for name,process in reversed(processes):
            if name not in result['exits']: result['exits'][name]=stop(process)
        for stream in logs: stream.close()
        result['passed']=len(result['cases'])==15 and all(c['passed'] for c in result['cases']) and all(v==0 for v in result['exits'].values())
        with open(directory+'/localization-results.json','w') as output: json.dump(result,output,indent=2,sort_keys=True)
        print(json.dumps(result,indent=2,sort_keys=True))
    if not result['passed']: raise SystemExit(1)


if __name__=='__main__':
    main()
