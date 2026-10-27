#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Read-only Linux /proc sampler. CPU 100% for a process means one full core."""
from __future__ import print_function
import argparse
import glob
import json
import os
import time


def read(path):
    with open(path) as stream:
        return stream.read()


def steady():
    return float(read('/proc/uptime').split()[0])


class Sampler(object):
    def __init__(self):
        self.previous = None
        self.hz = os.sysconf('SC_CLK_TCK')
        self.cores = os.sysconf('SC_NPROCESSORS_ONLN')

    def sample(self):
        cpus = [int(v) for v in read('/proc/stat').splitlines()[0].split()[1:9]]
        mem = dict((line.split(':')[0], int(line.split()[1]))
                   for line in read('/proc/meminfo').splitlines())
        processes = {}
        for entry in glob.glob('/proc/[0-9]*/stat'):
            try:
                pid = int(entry.split('/')[2])
                data = read(entry); fields = data[data.rfind(')')+2:].split()
                command = read('/proc/%d/cmdline' % pid).replace('\0', ' ').strip()
                if not command:
                    continue
                io = {}
                try:
                    io = dict((k.rstrip(':'), int(v)) for k,v in
                              (line.split() for line in read('/proc/%d/io' % pid).splitlines()))
                except IOError:
                    pass
                processes[str(pid)] = dict(start=fields[19], ticks=int(fields[11])+int(fields[12]),
                    rss_kib=int(fields[21])*os.sysconf('SC_PAGE_SIZE')//1024,
                    write_bytes=io.get('write_bytes'), command=command[:240])
            except (IOError, OSError, ValueError, IndexError):
                continue
        disks = {}
        for line in read('/proc/diskstats').splitlines():
            fields = line.split()
            # Only whole devices; partitions would count the same IO twice.
            if len(fields) >= 14 and (fields[2].startswith('mmcblk') and fields[2][-1].isdigit()
                                      and 'p' not in fields[2] or fields[2] in ('sda', 'nvme0n1')):
                disks[fields[2]] = dict(read_sectors=int(fields[5]), write_sectors=int(fields[9]),
                                        io_ms=int(fields[12]))
        stamp = steady()
        result = dict(wall=time.time(), steady=stamp, cores=self.cores,
            mem_available_kib=mem.get('MemAvailable', mem['MemFree']),
            swap_used_kib=mem['SwapTotal']-mem['SwapFree'], dirty_kib=mem.get('Dirty',0),
            loadavg=read('/proc/loadavg').split()[:3], cpu_busy_pct=None, iowait_pct=None,
            processes=processes, disks=disks, temperatures_c={})
        for path in glob.glob('/sys/class/thermal/thermal_zone*/temp'):
            try:
                result['temperatures_c'][path.split('/')[-2]] = float(read(path))/1000.0
            except (IOError, ValueError):
                pass
        if self.previous:
            elapsed = stamp-self.previous['steady']
            delta = [a-b for a,b in zip(cpus, self.previous['cpus'])]
            total = sum(delta)
            if total > 0:
                result['cpu_busy_pct'] = 100.0*(total-delta[3]-delta[4])/total
                result['iowait_pct'] = 100.0*delta[4]/total
            for pid,item in processes.items():
                old = self.previous['processes'].get(pid)
                if old and old['start']==item['start'] and elapsed>0:
                    item['cpu_one_core_pct'] = 100.0*(item['ticks']-old['ticks'])/self.hz/elapsed
            for name,item in disks.items():
                old = self.previous['disks'].get(name)
                if old and elapsed>0:
                    item['write_kib_s'] = (item['write_sectors']-old['write_sectors'])*.5/elapsed
                    item['busy_pct'] = (item['io_ms']-old['io_ms'])/10.0/elapsed
        self.previous = dict(steady=stamp, cpus=cpus, processes=processes, disks=disks)
        return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output'); parser.add_argument('--seconds',type=int,default=60)
    args=parser.parse_args(); sampler=Sampler(); deadline=steady()+args.seconds
    with open(args.output,'a') as stream:
        while steady()<deadline:
            began=steady(); stream.write(json.dumps(sampler.sample(),sort_keys=True)+'\n'); stream.flush()
            time.sleep(max(.05,1-(steady()-began)))
