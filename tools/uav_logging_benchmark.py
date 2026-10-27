#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""USB SSH ground-only A/B run: no arming, mode, goal, PWM or FCU parameter writes."""
from __future__ import print_function
import argparse
import json
import os
import signal
import subprocess
import threading
import time
from uav_perf import Sampler, steady


def percentile(values,p):
    return sorted(values)[min(len(values)-1,int((len(values)-1)*p))] if values else None


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output'); parser.add_argument('--seconds',type=int,default=90)
    parser.add_argument('--logging',action='store_true')
    parser.add_argument('--profile',choices=['full','core'],default='full')
    args=parser.parse_args(); directory=os.path.abspath(args.output); os.makedirs(directory)
    os.environ['UAV_LOG_DISABLED']='0' if args.logging else '1'
    os.environ['UAV_LOG_PROFILE']=args.profile
    os.environ['PYTHONUNBUFFERED']='1'
    log=open(directory+'/stack.stdout.log','wb')
    stack=subprocess.Popen([os.path.expanduser('~/.local/bin/uav'),'check','--target','flammable'],
                           stdout=log,stderr=subprocess.STDOUT,preexec_fn=os.setsid)
    top_log=open(directory+'/top.txt','wb'); vm_log=open(directory+'/vmstat.txt','wb')
    top=subprocess.Popen(['top','-b','-d','2'],stdout=top_log,preexec_fn=os.setsid)
    vm=subprocess.Popen(['vmstat','1'],stdout=vm_log,preexec_fn=os.setsid)
    import rospy
    from std_msgs.msg import String
    from mavros_msgs.msg import State
    from uav_log import atomic_json, active, stop_active
    from collections import defaultdict
    metrics=defaultdict(lambda:dict(times=[],ages=[]))
    lock=threading.Lock(); values={}; forbidden=[]; state=[None]
    report=dict(logging=args.logging,profile=args.profile,seconds=args.seconds,passed=False,ground_only=True)
    phase=['warmup']
    audit_values=[]; inference_values=[]; readiness_values=[]
    def receive(topic):
        def cb(msg):
            now=steady()
            if phase[0]=='measure':
                with lock:
                    metrics[topic]['times'].append(now)
            if topic in ('/mavros/setpoint_raw/local','/gpio_cmd'):
                forbidden.append(topic)
        return cb
    def readiness(msg):
        receive('/mission/readiness')(msg)
        try:
            val=json.loads(msg.data); values['readiness']=(steady(),val)
            if phase[0]=='measure': readiness_values.append(val)
            if any(val.get(k,False) for k in ('actuation_enabled','flight_authority')):
                forbidden.append('actuation/flight authority enabled')
        except ValueError: pass
    def diagnostic(topic, destination):
        def cb(msg):
            receive(topic)(msg)
            if phase[0]=='measure':
                try: destination.append(json.loads(msg.data))
                except ValueError: pass
        return cb
    def fcu(msg):
        state[0]=msg; values['state']=(steady(),msg)
        if msg.armed: forbidden.append('FCU ARMED')
    try:
        deadline=steady()+20
        import rosgraph
        master=rosgraph.Master('/benchmark_wait')
        while steady()<deadline:
            try: master.getPid(); break
            except Exception: time.sleep(.2)
        rospy.init_node('logging_benchmark',disable_signals=True)
        topics=['/mavros/imu/data','/scan','/mavros/local_position/pose','/mavros/vision_pose/pose',
                '/camera/image_raw',
                '/mavros/setpoint_raw/local','/gpio_cmd']
        subscribers=[rospy.Subscriber(t,rospy.AnyMsg,receive(t),queue_size=4) for t in topics]
        subscribers += [rospy.Subscriber('/mission/readiness',String,readiness,queue_size=1),
                        rospy.Subscriber('/mavros/state',State,fcu,queue_size=1),
                        rospy.Subscriber('/mission/audit',String,diagnostic('/mission/audit',audit_values),queue_size=4),
                        rospy.Subscriber('/mission/inference',String,diagnostic('/mission/inference',inference_values),queue_size=4)]
        # No image decoding in this observer. Same observer/top/vmstat in both arms.
        deadline=steady()+90; good_since=None
        gates=('model_loaded','frame_fresh','inference_ready','pose_fresh','fcu_fresh','tf_fresh')
        while steady()<deadline:
            if stack.poll() is not None: raise RuntimeError('Ground stack exited')
            if forbidden: raise RuntimeError(str(forbidden))
            row=values.get('readiness')
            healthy=row and steady()-row[0]<1 and all(row[1].get(k) for k in gates)
            if healthy:
                if good_since is None: good_since=steady()
                if steady()-good_since>15: break
            else: good_since=None
            time.sleep(.2)
        else: raise RuntimeError('No stable ground readiness')
        session=active(); report['session']=session
        if args.logging and not session: raise RuntimeError('Recorder not active')
        phase[0]='measure'; began=steady(); report['start_wall']=time.time()
        samples=[]; sampler=Sampler(); last_check=0
        with open(directory+'/performance.jsonl','w') as stream:
            while steady()-began<args.seconds:
                if forbidden: raise RuntimeError(str(forbidden))
                if stack.poll() is not None: raise RuntimeError('Ground stack exited')
                sample=sampler.sample(); samples.append(sample)
                stream.write(json.dumps(sample,sort_keys=True)+'\n'); stream.flush()
                if args.logging and not active(): raise RuntimeError('Recorder stopped during measurement')
                row=values.get('readiness')
                if not row or steady()-row[0]>1: raise RuntimeError('Readiness stream stopped')
                time.sleep(1)
        phase[0]='finished'; elapsed=steady()-began
        with lock:
            for topic,data in metrics.items():
                times=data['times']; gaps=[b-a for a,b in zip(times,times[1:])]
                report.setdefault('topics',{})[topic]=dict(count=len(times),hz=(len(times)-1)/(times[-1]-times[0])
                    if len(times)>1 else 0,max_gap=max(gaps) if gaps else None,p99_gap=percentile(gaps,.99))
        for topic,minimum in [('/mavros/imu/data',35),('/scan',7),('/mavros/local_position/pose',20),
                               ('/mavros/vision_pose/pose',14),('/mission/readiness',7)]:
            if report['topics'].get(topic,{}).get('hz',0)<minimum:
                raise RuntimeError('Insufficient topic rate '+topic)
        report['cpu_busy_mean']=sum(s['cpu_busy_pct'] for s in samples if s['cpu_busy_pct'] is not None)/(len(samples)-1)
        report['iowait_p99']=percentile([s['iowait_pct'] for s in samples if s['iowait_pct'] is not None],.99)
        report['min_available_mib']=min(s['mem_available_kib'] for s in samples)/1024.0
        report['max_temperature_c']=max(v for s in samples for v in s['temperatures_c'].values())
        report['passed']=True; report['end_wall']=time.time()
        report['readiness_healthy_fraction']=sum(all(row.get(k) for k in gates) for row in readiness_values)/float(len(readiness_values))
        report['control_max_gap_ms']=max([row.get('control_max_gap_ms',0) for row in audit_values] or [0])
        report['control_p99_gap_ms']=percentile([row.get('control_max_gap_ms',0) for row in audit_values],.99)
        report['inference_p99_s']=percentile([row['inference_s'] for row in inference_values],.99)
        report['inference_mean_s']=sum(row['inference_s'] for row in inference_values)/len(inference_values) if inference_values else None
        report['inference_rejected']=sum(not row['accepted'] for row in inference_values)
        report['forbidden_outputs']=forbidden
    except Exception as error:
        report['error']=str(error)
    finally:
        # Unexpected arming: leave position sources running for human landing.
        if state[0] is not None and state[0].armed:
            report['left_stack_running']='Unexpected FCU armed; manual intervention required'
        else:
            if stack.poll() is None: os.killpg(stack.pid,signal.SIGINT)
            deadline=steady()+25
            while stack.poll() is None and steady()<deadline: time.sleep(.1)
            if stack.poll() is None:
                report['cleanup_error']='Stack failed to exit normally'
                report['passed']=False
            # Recorder is independent of SSH/stack. Explicitly close before reading bags.
            if args.logging: stop_active()
        for proc in (top,vm):
            if proc.poll() is None: os.killpg(proc.pid,signal.SIGTERM)
            proc.wait()
        top_log.close(); vm_log.close(); log.close()
        atomic_json(directory+'/result.json',report)
        print(json.dumps(report,indent=2,sort_keys=True))
    return 0 if report['passed'] else 1


if __name__=='__main__':
    import sys
    sys.exit(main())
