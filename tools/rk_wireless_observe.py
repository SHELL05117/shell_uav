#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Bounded subscription-only observation of an existing disarmed ROS graph."""
from __future__ import print_function
import argparse
import collections
import json
import os
import time
import threading
import rospy
from sensor_msgs.msg import Image, CompressedImage, Imu, LaserScan, BatteryState, NavSatFix
from mavros_msgs.msg import State, RCIn, StatusText, EstimatorStatus
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray


def run(path, seconds):
    rospy.init_node('shell_wireless_observer',anonymous=True,disable_signals=True)
    lock=threading.Lock(); data={}; counts=collections.Counter(); ages={}; states=[]
    began=time.time(); violations=[]
    def receive(key):
        def callback(msg):
            now=time.time()
            with lock:
                counts[key]+=1
                if hasattr(msg,'header') and msg.header.stamp.to_sec():
                    ages.setdefault(key,[]).append(now-msg.header.stamp.to_sec())
                if key=='state':
                    value=dict(connected=msg.connected,armed=msg.armed,mode=msg.mode,system_status=msg.system_status)
                    states.append(value); data[key]=value
                    if msg.armed: violations.append('FCU armed')
                elif key in ('raw','annotated'):
                    data[key]=dict(width=msg.width,height=msg.height,encoding=msg.encoding,step=msg.step)
                elif key=='map':
                    data[key]=dict(width=msg.info.width,height=msg.info.height,resolution=msg.info.resolution,
                                   frame=msg.header.frame_id,known_cells=sum(v>=0 for v in msg.data))
                elif key in ('mission','readiness','location'):
                    try: data[key]=json.loads(msg.data)
                    except ValueError: data[key]=msg.data
                elif key=='battery':
                    data[key]=dict(voltage=msg.voltage,percentage=msg.percentage,present=msg.present,
                                   current=msg.current,power_supply_status=msg.power_supply_status)
                elif key=='rc': data[key]=dict(channels=list(msg.channels),rssi=msg.rssi)
                elif key=='gps': data[key]=dict(status=msg.status.status,latitude=msg.latitude,longitude=msg.longitude)
                elif key=='statustext':
                    data.setdefault(key,[]).append(dict(severity=msg.severity,text=msg.text))
                elif key=='diagnostics':
                    data[key]=[dict(level=s.level,name=s.name,message=s.message,
                                    values={p.key:p.value for p in s.values}) for s in msg.status]
                elif key=='estimator':
                    data[key]=dict(local_xy=msg.pos_horiz_rel_status_flag,vertical=msg.pos_vert_abs_status_flag,
                                   constant_position=msg.const_pos_mode_status_flag)
                elif key in ('setpoint','gpio'): violations.append(key+' message')
        return callback
    topics=[('state','/mavros/state',State),('imu','/mavros/imu/data',Imu),('scan','/scan',LaserScan),
        ('raw','/camera/image_raw',Image),('annotated','/detect_result',Image),
        ('raw_jpeg','/monitor/camera/raw/compressed',CompressedImage),
        ('annotated_jpeg','/monitor/camera/image/compressed',CompressedImage),
        ('map','/map',OccupancyGrid),('vision','/mavros/vision_pose/pose',PoseStamped),
        ('pose','/mavros/local_position/pose',PoseStamped),('location','/location_node/status',String),
        ('mission','/mission/status',String),('readiness','/mission/readiness',String),
        ('battery','/mavros/battery',BatteryState),('rc','/mavros/rc/in',RCIn),
        ('gps','/mavros/global_position/raw/fix',NavSatFix),('statustext','/mavros/statustext/recv',StatusText),
        ('diagnostics','/diagnostics',DiagnosticArray),('estimator','/mavros/estimator_status',EstimatorStatus),
        ('setpoint','/mavros/setpoint_raw/local',rospy.AnyMsg),('gpio','/gpio_cmd',rospy.AnyMsg)]
    subs=[rospy.Subscriber(topic,kind,receive(key),queue_size=1,buff_size=16777216) for key,topic,kind in topics]
    rospy.sleep(seconds)
    with lock:
        report=dict(seconds=time.time()-began,counts=dict(counts),latest=data,violations=violations,
                    all_states_disarmed=bool(states) and all(not s['armed'] for s in states),
                    all_states_connected=bool(states) and all(s['connected'] for s in states))
        report['hz']={k:v/report['seconds'] for k,v in counts.items() if k not in ('location','diagnostics')}
        report['age_seconds']={k:dict(median=sorted(v)[len(v)//2],max=max(v)) for k,v in ages.items()}
        names=['enable_actuation','auto_start','backend','model_file']
        report['parameters']={n:rospy.get_param('/shoot_with_move_base_node/'+n,None) for n in names}
        report['passed']=report['all_states_disarmed'] and report['all_states_connected'] and not violations and\
            all(counts[k]>0 for k in ['map','imu','scan','raw','raw_jpeg','pose','vision','readiness']) and\
            data.get('location')=='READY' and data.get('readiness',{}).get('inference_ready') is True and\
            report['parameters']['enable_actuation'] is False
    with open(path,'w') as f: json.dump(report,f,indent=2,sort_keys=True)
    print(json.dumps({k:report[k] for k in ('passed','hz','violations')},indent=2,sort_keys=True))
    return 0 if report['passed'] else 1


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output'); parser.add_argument('--seconds',type=float,default=20)
    args=parser.parse_args()
    raise SystemExit(run(args.output,args.seconds))
