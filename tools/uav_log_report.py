#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Offline bag/performance audit. Run after closing the session, away from flight."""
from __future__ import print_function
import argparse
import csv
import glob
import json
import math
import os
import sys
from collections import defaultdict
from uav_log import atomic_json


def stats(values):
    values=sorted(v for v in values if v is not None)
    if not values: return dict(count=0)
    return dict(count=len(values),mean=sum(values)/len(values),maximum=values[-1],
        p95=values[int((len(values)-1)*.95)],p99=values[int((len(values)-1)*.99)])


def generate(directory):
    import rosbag
    directory=os.path.abspath(directory)
    if glob.glob(directory+'/*.active'): raise RuntimeError('Active/unclosed bags; inspect before report')
    topics=defaultdict(lambda:dict(times=[],ages=[])); timeline=[]; inference=[]; audit=[]; path=[]
    source_images=[]; range_rows=[]; errors=[]; healthy=[]; states=[]; setpoints=[]
    for filename in sorted(glob.glob(directory+'/*.bag')):
        try:
            with rosbag.Bag(filename) as bag:
                for topic,msg,stamp in bag.read_messages():
                    now=stamp.to_sec(); topics[topic]['times'].append(now)
                    header=getattr(msg,'header',None)
                    if header and header.stamp.to_sec()>0:
                        topics[topic]['ages'].append(now-header.stamp.to_sec())
                    if topic in ('/mission/status','/location_node/status','/gpio_cmd','/gpio_ack'):
                        timeline.append(dict(t=now,topic=topic,event=msg.data))
                    elif topic in ('/mission/inference','/mission/audit','/mission/readiness','/location_node/range_audit'):
                        try:
                            row=json.loads(msg.data); row['bag_time']=now
                            if topic=='/mission/inference': inference.append(row)
                            elif topic=='/mission/audit': audit.append(row)
                            elif topic=='/mission/readiness': healthy.append(row)
                            else: range_rows.append(row)
                        except ValueError: errors.append('Invalid JSON on '+topic)
                    elif topic=='/mission/inference_frame/compressed':
                        source_images.append(msg.header.stamp.to_nsec())
                    elif topic=='/mavros/local_position/pose':
                        p=msg.pose.position; path.append([now,p.x,p.y,p.z])
                    elif topic=='/mavros/setpoint_raw/local':
                        p=msg.position; v=msg.velocity
                        setpoints.append([now,msg.coordinate_frame,msg.type_mask,p.x,p.y,p.z,v.x,v.y,v.z,msg.yaw])
                    elif topic=='/mavros/state': states.append([now,msg.armed,msg.mode,msg.connected])
        except Exception as error: errors.append(os.path.basename(filename)+': '+str(error))
    for topic,item in topics.items():
        times=sorted(item.pop('times')); ages=item.pop('ages')
        item.update(count=len(times),hz=(len(times)-1)/(times[-1]-times[0]) if len(times)>1 and times[-1]>times[0] else 0,
            receive_gap_s=stats([b-a for a,b in zip(times,times[1:])]),source_age_s=stats(ages))
    samples=[]
    with open(directory+'/performance.jsonl') as stream:
        for line in stream:
            try: samples.append(json.loads(line))
            except ValueError: errors.append('Truncated performance sample')
    source_keys=set(int(round(row['source_stamp']*1e9)) for row in inference)
    # JSON floating point seconds lose sub-microsecond precision. Pair with 2us tolerance.
    unmatched=sum(not any(abs(key-stamp)<=2000 for key in source_keys) for stamp in source_images)
    missing_images=sum(not any(abs(key-stamp)<=2000 for stamp in source_images) for key in source_keys)
    summary=dict(schema=1,session=os.path.basename(directory),topics=dict(topics),errors=errors,
        cpu_busy_pct=stats([s.get('cpu_busy_pct') for s in samples]),
        iowait_pct=stats([s.get('iowait_pct') for s in samples]),
        available_mib_min=min([s['mem_available_kib']/1024.0 for s in samples] or [0]),
        inference_s=stats([row['inference_s'] for row in inference]),
        inference_rejected=sum(not row['accepted'] for row in inference),
        inference_audit_overwritten=max([row.get('audit_overwritten',0) for row in inference] or [0]),
        inference_frames=len(source_images),frames_without_result=unmatched,results_without_frame=missing_images,
        control_interval_max_per_tick_ms=stats([r.get('control_max_gap_ms') for r in audit]),
        healthy_fraction=sum(all(r.get(k) for k in ('model_loaded','frame_fresh','inference_ready','pose_fresh','fcu_fresh','tf_fresh'))
                             for r in healthy)/float(len(healthy)) if healthy else None,
        range_rejected=sum(not r['accepted'] for r in range_rows),
        bag_bytes=sum(os.path.getsize(p) for p in glob.glob(directory+'/*.bag')),
        actual_setpoint_messages=len(setpoints),
        limitations=['JPEG is a lossy record of the selected source frame, not pixel-identical model tensor',
                     'Bag receive timestamps do not prove cross-machine latency',
                     'Control timer intervals in a ground dry run do not validate airborne setpoint delivery',
                     'GPIO ACK is software completion, not physical payload release',
                     'No PX4 ULog is imported automatically'])
    for filename in glob.glob(directory+'/*stdout.log'):
        with open(filename) as stream:
            for line in stream:
                if 'buffer exceeded' in line.lower() or 'dropping oldest' in line.lower():
                    errors.append(os.path.basename(filename)+': recorder buffer overflow')
    required=['/mavros/state','/mavros/local_position/pose','/mavros/imu/data','/scan',
              '/mavros/vision_pose/pose','/mission/readiness','/mission/inference','/mission/audit']
    summary['missing_required_topics']=[t for t in required if not topics.get(t,{}).get('count')]
    session=json.load(open(directory+'/session.json'))
    summary['profile']=session.get('profile','full')
    summary['data_complete']=not errors and not summary['missing_required_topics'] and unmatched==0 and (missing_images==0 or summary['profile']=='core') and summary['inference_audit_overwritten']==0
    atomic_json(directory+'/summary.json',summary)
    for name,header,rows in [
        ('trajectory.csv',['time','x','y','z'],path),
        ('setpoints.csv',['time','coordinate_frame','type_mask','x','y','z','vx','vy','vz','yaw'],setpoints),
        ('flight_modes.csv',['time','armed','mode','connected'],states),
        ('events.csv',['time','topic','event'],[[r['t'],r['topic'],r['event']] for r in sorted(timeline,key=lambda x:x['t'])])]:
        with open(directory+'/'+name,'w') as stream:
            writer=csv.writer(stream); writer.writerow(header); writer.writerows(rows)
    with open(directory+'/diagnostics.jsonl','w') as stream:
        for row in inference+audit+range_rows: stream.write(json.dumps(row,sort_keys=True)+'\n')
    with open(directory+'/report.md','w') as stream:
        stream.write('# Flight evidence report\n\nSession: '+summary['session']+'\n\n')
        stream.write('Data complete: '+str(summary['data_complete'])+'\n\n')
        stream.write('Inference source frames: %d; missing frame/result pairs: %d/%d.\n\n' %
                     (len(source_images),missing_images,unmatched))
        stream.write('| Topic | Messages | Hz | Max receive gap (s) |\n|---|---:|---:|---:|\n')
        for topic,row in sorted(topics.items()):
            stream.write('| %s | %d | %.3f | %s |\n' % (topic,row['count'],row['hz'],row['receive_gap_s'].get('maximum')))
        stream.write('\n## Evidence limits\n\n')
        for entry in summary['limitations']+errors: stream.write('- '+entry+'\n')
    return summary


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__); parser.add_argument('directory')
    args=parser.parse_args(); print(json.dumps(generate(args.directory),sort_keys=True,indent=2))
