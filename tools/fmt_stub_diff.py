#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Verify that the format-string stub file mirrors the real sources verbatim.

Why this exists
---------------
tools/fmt_check_2026_09_25.cpp re-creates each ROS_* call site by hand so that
g++ can apply format(gnu_printf) without any ROS headers.  That only proves
something if the string in the stub is byte-identical to the string in the
source: a stub that has drifted is a test that validates itself.  This script
extracts every adjacent-string-literal sequence from both sides, concatenates
it the way the compiler does, and reports any stub message that does not appear
verbatim in the real sources.

How the concatenation rule works, and why it is not just "join every literal"
------------------------------------------------------------------------------
C++ joins two literals only when nothing but whitespace separates them.  A comma
does not join:  f("a", "b")  is one format string plus one argument.  That
distinction is the whole point of the script, so a line is walked group by
group, where one group is a maximal run of literals separated by whitespace
only.  A group also continues onto the next physical line when that next line
begins with a literal (nothing but indentation before it) and holds exactly one
group - which is the shape every multi-line format string in these sources has.

That "begins with a literal" rule, rather than "consists of nothing but a
literal", is what makes the LAST line of a multi-line format string count.  It
always ends in a comma and the argument list - `"speed=%.3f [odom]",` - so a
stricter rule truncated the run one line early and left the tail of every
message unchecked.  That is not hypothetical: the stub still carried
`cmd=(%.3f, %.3f) [%s]` for a message the source had already changed to `[odom]`
and this script reported OK, because both sides dropped that line.

Not wired into check_config.py on purpose: the stub is a development aid, not a
build prerequisite, and it must never be able to fail a real run.
Usage:  python3 tools/fmt_stub_diff.py [--verbose]
Exit 1 only if a stub message is missing from the sources.
"""

import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
STUB = os.path.join(ROOT, "tools", "fmt_check_2026_09_25.cpp")

# Scan the whole tree rather than a hand-written list: the stub mirrors calls in
# ego_planner_2d_node, ego_trajectory_controller, bspline_optimizer,
# rescue_mission, mission_vision and servo_controller_node, and a hand-kept list
# goes stale the first time a log line moves file.
SRC_DIR = os.path.join(ROOT, "src")

LITERAL = re.compile(r'(?:u8|L|u|U)?"((?:[^"\\\n]|\\.)*)"')

# One maximal group: a literal, then any number of (whitespace + literal).
ADJACENT_GROUP = re.compile(
    r'(?:u8|L|u|U)?"(?:[^"\\\n]|\\.)*"'
    r'(?:\s*(?:u8|L|u|U)?"(?:[^"\\\n]|\\.)*")*'
)


def groups_in_line(raw):
    """Return [(start, end, joined_text), ...] for one line."""
    out = []
    for m in ADJACENT_GROUP.finditer(raw):
        out.append((m.start(), m.end(), "".join(LITERAL.findall(m.group(0)))))
    return out


def concat_runs(path):
    """Return [(start_line, concatenated_string), ...] for one file."""
    with open(path, "r", encoding="utf-8", errors="replace") as fh:
        lines = fh.read().splitlines()

    runs = []
    cur = None
    cur_start = 0
    prev_lineno = None
    prev_tail_clean = False

    for idx, raw in enumerate(lines):
        lineno = idx + 1
        groups = groups_in_line(raw)
        if not groups:
            prev_lineno = None       # nothing here can be adjacent to it
            prev_tail_clean = False
            continue

        # "the line begins with its literal group" - the multi-line format
        # style. Anything after the group is the argument list of the enclosing
        # call, which does not stop the literal from being a continuation.
        starts_with_literal = (
            len(groups) == 1
            and raw[:groups[0][0]].strip() == ""
        )
        # ...but the PREVIOUS line has to end on its literal with nothing after
        # it. A trailing comma ends the string and starts the argument list, so
        # `"...%.2fs",` followed by `"true", static_cast<double>(x)` must not
        # join - "true" is an argument, not a continuation of the message.
        prev_ends_on_literal = prev_tail_clean
        adjacent = prev_lineno is not None and lineno == prev_lineno + 1

        for start, _end, text in groups:
            if (starts_with_literal and prev_ends_on_literal and adjacent
                    and cur is not None and len(groups) == 1):
                cur += text
            else:
                if cur is not None:
                    runs.append((cur_start, cur))
                cur = text
                cur_start = lineno
                adjacent = False

        prev_lineno = lineno
        prev_tail_clean = raw[groups[-1][1]:].strip() == ""

    if cur is not None:
        runs.append((cur_start, cur))
    return runs


def iter_sources():
    for dirpath, _dirnames, filenames in os.walk(SRC_DIR):
        for name in sorted(filenames):
            if name.endswith((".cpp", ".h", ".hpp")):
                yield os.path.join(dirpath, name)


def main():
    verbose = "--verbose" in sys.argv or "-v" in sys.argv

    if not os.path.exists(STUB):
        print("FAIL: stub file not found: %s" % STUB)
        return 1
    stub_msgs = concat_runs(STUB)

    source_msgs = []
    scanned = 0
    for path in iter_sources():
        scanned += 1
        rel = os.path.relpath(path, ROOT).replace(os.sep, "/")
        for start, text in concat_runs(path):
            source_msgs.append((rel, start, text))

    haystack = "\n".join(t for _, _, t in source_msgs)

    checked = 0
    missing = []
    for start, text in stub_msgs:
        # Only messages long enough to be a log line and carrying a conversion.
        # Short tokens like "odom" match everywhere and prove nothing.
        if len(text) < 40 or "%" not in text:
            continue
        checked += 1
        if text not in haystack:
            missing.append((start, text))

    print("stub messages carrying a format specifier and >= 40 chars: %d"
          % checked)
    print("source files scanned: %d" % scanned)

    if verbose:
        for start, text in stub_msgs:
            if len(text) >= 40 and "%" in text:
                print("  stub:%4d  %s..." % (start, text[:70]))

    if missing:
        print("")
        print("FAIL: %d stub message(s) do not appear verbatim in any source:"
              % len(missing))
        for start, text in missing:
            print("")
            print("  tools/fmt_check_2026_09_25.cpp:%d" % start)
            print("    %s" % text)
        return 1

    print("")
    print("OK: every stub message exists verbatim in the sources, so the "
          "format check is exercising the real strings.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
