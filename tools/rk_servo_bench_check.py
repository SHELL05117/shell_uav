#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Bounded servo bench exercise on a private ROS master.

Requires grounded OE, removed propellers, empty drop bays and --allow-motion.
Default unlinked mode needs explicit small angles and --mechanism-free. Source
mode reads the actual navigation launch angles and requires confirmed physical
clearance. SHELL confirmed both servos may stay connected for that mode.
ACK/register checks do
not verify shaft movement, voltage under load, or calibrated release angles.
There are no MAVROS, navigation or flight nodes in this test. Always full-off
all PCA channels after the controller has stopped; rk_pca_disable.py must be
next to this script. Absolute angles must never be guessed with a coupled mount:
the first 2026-10-27 90-degree trial hit its physical stop.
"""
from __future__ import print_function
import argparse
import json
import math
import os
import socket
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

from rk_fourclass_camera_check import Children, steady


def read_register(reg):
    raw = subprocess.check_output(['i2cget','-y','6','0x40','0x%02x'%reg,'b'])
    return int(raw.strip(), 16)


def run(args):
    if not args.allow_motion:
        raise RuntimeError('Physical motion requires --allow-motion')
    if args.profile=='source':
        if not args.clearance_confirmed:
            raise RuntimeError('Source-angle test requires confirmed physical clearance')
        package=subprocess.check_output(['rospack','find','navigation_2d']).strip()
        root=ET.parse(os.path.join(package,'launch','navigation_2d.launch')).getroot()
        angles={name:float(root.find("arg[@name='%s']"%name).get('default')) for name in
            ('front_angle','down_angle','drop_safe_angle','fire_drop_angle','rescue_drop_angle')}
        neutral,step=angles['front_angle'],angles['down_angle']
    else:
        if not args.mechanism_free:
            raise RuntimeError('Disconnect the mechanical linkage before this bench exercise')
        neutral,step=args.neutral_angle,args.step_angle
        if neutral is None or step is None or not 0 < abs(step-neutral) <= 10:
            raise RuntimeError('Explicit unlinked bench angles with a 0..10 degree step required')
        angles=dict(front_angle=neutral,down_angle=step,drop_safe_angle=neutral,
                    fire_drop_angle=step,rescue_drop_angle=2*neutral-step)
    if not all(not math.isnan(v) and not math.isinf(v) and 0<=v<=180 for v in angles.values()):
        raise RuntimeError('Servo angles must be finite and inside 0..180')
    directory = os.path.abspath(args.output_dir)
    if os.path.exists(directory):
        raise RuntimeError('Refusing to overwrite a previous bench run')
    os.makedirs(directory)
    os.environ.update(ROS_MASTER_URI='http://127.0.0.1:11336', ROS_IP='127.0.0.1',
        ROS_HOME=directory+'/ros-home', ROS_LOG_DIR=directory+'/ros-logs')
    os.environ.pop('ROS_HOSTNAME', None)
    os.environ.pop('ROS_NAMESPACE', None)
    reservation=socket.socket(); reservation.bind(('127.0.0.1',11336)); reservation.close()
    children=Children(directory)
    report=dict(software_passed=False, mechanical_acceptance=False,
        start_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),
        load_voltage_acceptance=False, flight_acceptance=False,
        selected_servo=args.servo, commands=[], errors=[], temporary_angles=args.profile!='source',
        profile=args.profile,linkage_disconnected_declared=args.mechanism_free,
        clearance_confirmed=args.clearance_confirmed,angles=angles)
    rospy=None
    try:
        master=children.start('master',['roscore','-p','11336'])
        deadline=steady()+15
        while steady()<deadline:
            children.require_alive([master])
            connection=socket.socket()
            try:
                connection.connect(('127.0.0.1',11336)); break
            except socket.error:
                time.sleep(.1)
            finally:
                connection.close()
        else:
            raise RuntimeError('Private master startup failed')
        import rospy
        from std_msgs.msg import String, Bool
        rospy.init_node('shell_single_servo_bench',disable_signals=True)
        acknowledgements=[]; ready=[]
        ack_sub=rospy.Subscriber('/gpio_ack',String,
            lambda m: acknowledgements.append((steady(),m.data)),queue_size=10)
        ready_sub=rospy.Subscriber('/servo_ready',Bool,
            lambda m: ready.append(bool(m.data)),queue_size=10)
        publisher=rospy.Publisher('/gpio_cmd',String,queue_size=1)
        servo=children.start('servo',['roslaunch','servo_control','servo_bench.launch',
            'restore_after_drop:=false']+[name+':='+str(value) for name,value in sorted(angles.items())])
        deadline=steady()+12
        while steady()<deadline and not (any(ready) and publisher.get_num_connections()):
            children.require_alive([master,servo]); time.sleep(.05)
        if not any(ready) or not publisher.get_num_connections():
            raise RuntimeError('Servo readiness/command subscription deadline')
        report['prescale']=read_register(0xfe)
        if report['prescale']!=121:
            raise RuntimeError('PCA prescale does not match production 50 Hz setting')
        sequence=([('CAMERA_FRONT',neutral),('CAMERA_DOWN',step),('CAMERA_FRONT',neutral)]
            if args.servo=='camera' else [('DROP_FIRE',angles['fire_drop_angle']),
                                        ('DROP_RESCUE',angles['rescue_drop_angle'])])
        channel=0 if args.servo=='camera' else 1
        for command,angle in sequence:
            time.sleep(.5)
            begin=steady(); publisher.publish(String(data=command))
            deadline=begin+5
            while steady()<deadline and not any(t>=begin and v==command+'_DONE'
                                                for t,v in acknowledgements):
                children.require_alive([master,servo]); time.sleep(.02)
            ack=next(((t,v) for t,v in acknowledgements if t>=begin and v==command+'_DONE'),None)
            if ack is None:
                raise RuntimeError(command+' acknowledgement deadline')
            registers=[read_register(6+channel*4+i) for i in range(4)]
            ticks=int(math.floor((750+1500*angle/180.0)*50*4096/1000000.0+.5))
            expected=[0,0,ticks&255,ticks>>8]
            report['commands'].append(dict(command=command,angle=angle,channel=channel,
                acknowledgement=ack[1],ack_delay=ack[0]-begin,registers=registers,
                expected_registers=expected,registers_match=registers==expected))
            if registers!=expected:
                raise RuntimeError(command+' PWM register readback differs')
        report['software_passed']=True
    except BaseException as error:
        report['errors'].append(type(error).__name__+': '+str(error))
    finally:
        report['errors'].extend(children.finish_all())
        if rospy is not None:
            rospy.signal_shutdown('Single servo bounded bench complete')
        report['processes']=children.report()
        try:
            raw=subprocess.check_output(['python3',os.path.join(os.path.dirname(__file__),
                'rk_pca_disable.py')])
            report['outputs_disabled_after']=json.loads(raw)
        except Exception as error:
            report['errors'].append('Final output disable failed: '+str(error))
        report['software_passed'] = report['software_passed'] and not report['errors'] and all(
            child['exit_code']==0 and child['process_group_empty'] for child in report['processes'])
        with open(directory+'/servo-bench-results.json','w') as output:
            json.dump(report,output,indent=2,sort_keys=True)
        print(json.dumps(report,indent=2,sort_keys=True))
    return 0 if report['software_passed'] else 1


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output_dir')
    parser.add_argument('--servo',choices=['camera','drop'],default='camera')
    parser.add_argument('--allow-motion',action='store_true')
    parser.add_argument('--mechanism-free',action='store_true')
    parser.add_argument('--profile',choices=['unlinked','source'],default='unlinked')
    parser.add_argument('--clearance-confirmed',action='store_true')
    parser.add_argument('--neutral-angle',type=float)
    parser.add_argument('--step-angle',type=float)
    sys.exit(run(parser.parse_args()))
