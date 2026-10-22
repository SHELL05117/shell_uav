#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Print the last-modified time of every launch/param file in a catkin src
tree, newest first - so "is the flight computer running the same launch files
as the dev machine" stops being a guess.

WHY THIS EXISTS
---------------
Half of the "it works on the bench but not on the aircraft" failures here have
turned out to be a stale file on the Jetson: the launch that was synced is an
older revision than the one being read while reasoning about a log, and the
difference is exactly the parameter that is missing at runtime.  An mtime is
not a checksum - it only tells you whether a copy happened, not whether it was
the right copy - but it is enough to answer the one question that matters in
the field: "when did this file last change, and does that match the fix I just
made on the dev machine?".

WHAT IT PRINTS
--------------
One line per file, newest first:

    2026-10-07 17:31  navigation_2d/launch/navigation_2d.launch
    2026-10-07 17:28  servo_control/launch/servo_bench.launch
    ...

Run it against the Jetson's catkin src (the 6-package tree), then against the
dev machine's src/ and compare the first lines.  A row whose timestamp is older
than the fix it is supposed to contain is the file to re-sync.

Usage
-----
    python3 tools/list_launch_mtimes.py /home/jcfh/catkin_ws/src

    # same on the dev machine, to compare
    python3 tools/list_launch_mtimes.py src

Needs nothing beyond the standard library, so it runs on the Jetson with no
pip installs.
"""

import os
import sys
import time

WANTED_SUFFIXES = (".launch", ".yaml", ".lua")


def collect(root):
    found = []
    for dirpath, _dirnames, filenames in os.walk(root):
        for name in filenames:
            if not name.endswith(WANTED_SUFFIXES):
                continue
            full = os.path.join(dirpath, name)
            try:
                mtime = os.path.getmtime(full)
            except OSError:
                continue
            found.append((mtime, full))
    found.sort(key=lambda item: item[0], reverse=True)
    return found


def main(argv):
    if len(argv) != 2:
        print("usage: python3 list_launch_mtimes.py <src-root>", file=sys.stderr)
        print(__doc__, file=sys.stderr)
        return 2
    root = argv[1]
    if not os.path.isdir(root):
        print("not a directory: %s" % root, file=sys.stderr)
        return 2
    rows = collect(root)
    if not rows:
        print("no .launch/.yaml/.lua under %s" % root, file=sys.stderr)
        return 1
    for mtime, full in rows:
        stamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(mtime))
        rel = os.path.relpath(full, root)
        print("%s  %s" % (stamp, rel))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
