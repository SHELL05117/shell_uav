#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Private-master recorder failure probe. No real MAVROS, sensors or actuators."""
from __future__ import print_function
import argparse
import glob
import json
import os
import signal
import socket
import subprocess
import time
from uav_log import prepare, launch, active, atomic_json
from uav_perf import steady


def main():
    parser=argparse.ArgumentParser(description=__doc__); parser.add_argument('directory')
    args=parser.parse_args(); directory=os.path.abspath(args.directory); os.makedirs(directory)
    os.environ.update(ROS_MASTER_URI='http://127.0.0.1:11339',ROS_IP='127.0.0.1',
                      UAV_LOG_ROOT=directory+'/sessions',ROS_LOG_DIR=directory+'/ros-logs')
    os.environ.pop('ROS_HOSTNAME',None)
    reservation=socket.socket(); reservation.bind(('127.0.0.1',11339)); reservation.close()
    if active(): raise RuntimeError('Stop production recorder before isolated probe')
    output=open(directory+'/master.log','w')
    master=subprocess.Popen(['roscore','-p','11339'],stdout=output,stderr=subprocess.STDOUT,preexec_fn=os.setsid)
    marker=subprocess.Popen(['sleep','120'],preexec_fn=os.setsid)
    result=dict(private_master='127.0.0.1:11339',synthetic_setpoints_only=True,passed=False)
    try:
        import rospy
        import rosgraph
        from mavros_msgs.msg import PositionTarget, State
        rospy.init_node('uav_logging_isolated_probe',disable_signals=True)
        state_pub=rospy.Publisher('/mavros/state',State,queue_size=1)
        pub=rospy.Publisher('/mavros/setpoint_raw/local',PositionTarget,queue_size=2)
        session=prepare('isolated_fault_probe'); launch(session); result['session']=session
        state=State(connected=True,armed=False,mode='SIMULATION_ONLY')
        begin=steady(); sent=0
        while steady()-begin<4:
            state.header.stamp=rospy.Time.now(); state_pub.publish(state)
            msg=PositionTarget(); msg.header.stamp=rospy.Time.now(); msg.position.z=123.456
            pub.publish(msg); sent+=1; time.sleep(.02)
        graph=rosgraph.Master('/probe_audit').getSystemState()
        consumers=[nodes for topic,nodes in graph[1] if topic=='/mavros/setpoint_raw/local']
        if consumers!=[['/uav_core_recorder']]: raise RuntimeError('Unexpected synthetic setpoint consumer '+repr(consumers))
        subprocess.check_call(['rosnode','kill','/uav_media_recorder'])
        time.sleep(3)
        if not active(): raise RuntimeError('Media failure killed core recorder')
        graph=rosgraph.Master('/probe_audit').getSystemState()
        if not any('/uav_core_recorder' in nodes for topic,nodes in graph[1]): raise RuntimeError('Core missing after media failure')
        result['media_failure_kept_core']=True
        for index in range(50):
            msg=PositionTarget(); msg.header.stamp=rospy.Time.now(); msg.position.z=123.456
            pub.publish(msg); time.sleep(.02)
        subprocess.check_call(['rosnode','kill','/uav_core_recorder'])
        deadline=steady()+22
        while active() and steady()<deadline: time.sleep(.1)
        if active(): raise RuntimeError('Recorder failed to close after core failure')
        closed=json.load(open(session+'/closed.json'))
        result['core_failure_recorded']=any('core_recorder_exit' in text for text in closed['errors'])
        result['unrelated_process_kept_running']=marker.poll() is None
        result['master_kept_running']=master.poll() is None
        result['bags_closed']=not glob.glob(session+'/*.active')
        import rosbag
        messages=0
        for name in glob.glob(session+'/core*.bag'):
            with rosbag.Bag(name) as bag: messages+=bag.get_message_count('/mavros/setpoint_raw/local')
        result['synthetic_setpoints_recorded']=messages
        result['passed']=all(result[k] for k in ('media_failure_kept_core','core_failure_recorded',
            'unrelated_process_kept_running','master_kept_running','bags_closed')) and messages>=150
    finally:
        import rospy
        rospy.signal_shutdown('Isolated recorder probe complete')
        for child in (marker,master):
            if child.poll() is None: os.killpg(child.pid,signal.SIGINT)
        deadline=steady()+10
        while any(c.poll() is None for c in (marker,master)) and steady()<deadline: time.sleep(.1)
        output.close(); atomic_json(directory+'/result.json',result)
        print(json.dumps(result,sort_keys=True,indent=2))
    return 0 if result['passed'] else 1


if __name__=='__main__':
    import sys
    sys.exit(main())
