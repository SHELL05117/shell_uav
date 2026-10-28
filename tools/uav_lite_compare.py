#!/usr/bin/env python3
"""Compare predeclared ground-only off/on/on/off lightweight logging windows."""
import argparse
import json
from pathlib import Path


def compare(root):
    reports = {name: json.loads((root/name/'result.json').read_text())
               for name in ['off-1-valid','on-1','on-2','off-2']}
    off = [reports[k] for k in ['off-1-valid','off-2']]
    on = [reports[k] for k in ['on-1','on-2']]
    mean = lambda rows,key: sum(r[key] for r in rows)/len(rows)
    baseline = mean(off,'cpu_busy_mean'); enabled = mean(on,'cpu_busy_mean')
    memory_delta = mean(off,'min_available_mib')-mean(on,'min_available_mib')
    reasons = []
    if not all(r.get('passed') and r.get('ground_only') and not r.get('forbidden_outputs')
               and not r.get('cleanup_error') for r in reports.values()):
        reasons.append('One or more windows invalid or unsafe')
    if enabled-baseline > 5: reasons.append('CPU increase exceeds 5 percentage points')
    if memory_delta > 150: reasons.append('Available-memory reduction exceeds 150 MiB')
    if any(r['iowait_p99'] > 5 for r in on): reasons.append('Iowait P99 exceeds 5 percent')
    rates = {}
    recording_checks = {}
    for name in ['on-1','on-2']:
        path = root/name/'recording-summary.json'
        if not path.is_file():
            reasons.append('Missing closed recording audit: '+name)
            continue
        summary = json.loads(path.read_text())
        missing = [t for t in ['/mavros/state','/mavros/vision_pose/pose',
                    '/mavros/local_position/pose','/mission/readiness']
                   if not summary.get('topics',{}).get(t,{}).get('count')]
        valid = summary.get('recording_closed_without_writer_errors') and not missing
        recording_checks[name] = dict(valid=valid, missing_required=missing,
                                       errors=summary.get('errors',[]))
        if not valid: reasons.append('Recording integrity/overflow check failed: '+name)
    for topic in ['/mavros/imu/data','/scan','/mavros/local_position/pose',
                  '/mavros/vision_pose/pose','/mission/readiness']:
        off_hz = sum(r['topics'][topic]['hz'] for r in off)/len(off)
        on_hz = sum(r['topics'][topic]['hz'] for r in on)/len(on)
        rates[topic] = dict(off_hz=off_hz, on_hz=on_hz, change_pct=100*(on_hz/off_hz-1))
        if on_hz < .95*off_hz: reasons.append('Frequency decrease exceeds 5 percent: '+topic)
    return dict(retain_onboard=not reasons, reasons=reasons,
        off_cpu_mean=baseline, on_cpu_mean=enabled, cpu_delta_percentage_points=enabled-baseline,
        available_memory_reduction_mib=memory_delta,
        recording_checks=recording_checks,
        max_on_iowait_p99=max(r['iowait_p99'] for r in on), rates=rates, windows=reports,
        boundary='Ground dry-run only; no actual flight setpoint timing or dynamic planner load validated')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root',type=Path); parser.add_argument('output',type=Path)
    args = parser.parse_args()
    result = compare(args.root)
    args.output.write_bytes((json.dumps(result,ensure_ascii=False,indent=2)+'\n').encode('utf-8'))
    print(json.dumps({k:v for k,v in result.items() if k not in ['windows']},ensure_ascii=False,indent=2))
