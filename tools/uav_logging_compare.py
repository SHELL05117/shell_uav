#!/usr/bin/env python3
"""Apply the predeclared ground acceptance budget, not a flight certification."""
import argparse
import json
from pathlib import Path


def compare(root,on_labels=('on-optimized-1','on-optimized-2')):
    root=Path(root)
    labels=['off-1']+list(on_labels)+['off-2']
    rows={label:json.loads((root/label/'result.json').read_text()) for label in labels}
    off=[rows[k] for k in ('off-1','off-2')]; on=[rows[k] for k in on_labels]
    average=lambda group,key:sum(r[key] for r in group)/len(group)
    delta=average(on,'cpu_busy_mean')-average(off,'cpu_busy_mean')
    memory=average(off,'min_available_mib')-average(on,'min_available_mib')
    p99delta=max(r['control_p99_gap_ms'] for r in on)-max(r['control_p99_gap_ms'] for r in off)
    reasons=[]
    if not all(row.get('passed') for row in rows.values()): reasons.append('A ground run failed')
    if delta>5: reasons.append('Whole-system mean CPU increase exceeds 5 percentage points')
    if memory>150: reasons.append('Available-memory decrease exceeds 150 MiB')
    if max(r['iowait_p99'] for r in on)>5: reasons.append('IOwait P99 exceeds 5%')
    if p99delta>5: reasons.append('Control-timer P99 gap increase exceeds 5 ms')
    if max(r['control_max_gap_ms'] for r in on)>100: reasons.append('Control-timer gap exceeds 100 ms')
    if min(r['readiness_healthy_fraction'] for r in on)<.99: reasons.append('Readiness fraction below 99%')
    if sum(r['inference_rejected'] for r in on)>sum(r['inference_rejected'] for r in off): reasons.append('Additional inference rejections')
    for topic in ['/mavros/imu/data','/scan','/mavros/local_position/pose','/mavros/vision_pose/pose','/mission/readiness']:
        ref=sum(r['topics'][topic]['hz'] for r in off)/len(off)
        if min(r['topics'][topic]['hz'] for r in on)<.95*ref: reasons.append('Rate decrease exceeds 5%: '+topic)
    return dict(retain_onboard=not reasons,reasons=reasons,rows=rows,
        off_cpu_mean=average(off,'cpu_busy_mean'),on_cpu_mean=average(on,'cpu_busy_mean'),
        cpu_delta_percentage_points=delta,available_memory_decrease_mib=memory,
        control_p99_gap_delta_ms=p99delta,
        limitations=['Top/vmstat and ROS observer were present in both arms',
                     'Static disarmed ground scene; no airborne or active-navigation load certification',
                     'High load is a screening rejection; no airborne failure was induced'])


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__); parser.add_argument('root'); parser.add_argument('--output')
    parser.add_argument('--core',action='store_true')
    args=parser.parse_args(); result=compare(args.root,('core-1','core-2') if args.core else ('on-optimized-1','on-optimized-2')); value=json.dumps(result,ensure_ascii=False,indent=2)
    if args.output: Path(args.output).write_text(value+'\n',encoding='utf-8')
    print(value)
