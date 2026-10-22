#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Type an angle, watch the servo move - the interactive front end of
/servo_debug_cmd.

WHY THIS EXISTS
---------------
Calibrating fire_drop_angle and rescue_drop_angle means sitting next to the
mechanism and answering one question over and over: "at this angle, which
container opens?".  Through raw ROS that loop looks like

    rostopic pub -1 /servo_debug_cmd std_msgs/String "data: 'SET_DROP 90'"
    rostopic echo /servo_debug_ack            # in a second terminal

which is a 60-character line and a second window per angle.  That friction is
what makes a sweep get abandoned halfway, so the last few angles - the ones
nobody measured - are the ones that end up in the launch file.

This script is that loop with nothing else in it: type an angle, press enter,
read the reply.

WHAT IT TALKS TO
----------------
std_msgs/String on /servo_debug_cmd, answered on /servo_debug_ack.  Neither
topic exists unless servo_controller_node was started with angle_debug:=true,
and the only place that is true is the bench launch:

    roslaunch servo_control servo_bench.launch

Do NOT try to pass angle_debug:=true to navigation_2d.launch.  That launch does
not declare it as an <arg>, so roslaunch discards the command-line value
without a word, the node reads the default false, and the symptom is "the servo
does nothing and reports no error".  servo_bench.launch keeps that flag out of
the flight chain entirely.

The flag is deliberately off in the flight launch, so if this script cannot
connect the diagnosis is almost always "the bench launch is not the one running"
- which is why the failure message below names the launch file instead of just
timing out.

Nothing here drives the PCA9685 directly.  servo_controller_node does, so its
cached drop/camera angles stay correct, the aircraft is not being driven by a
second writer, and the servo moves to exactly the angle it would move to in
flight (same ticks, same clamp).  tools/servo_angle_calib.py is the fallback
for when the node itself will not start.

SAFETY
------
* A sweep WILL release a loaded container if it passes a release angle.  Test
  with the mechanism unloaded, or with the containers mechanically isolated,
  until you know where the two release angles are.
* Never run servo_bench.launch and navigation_2d.launch at the same time: one
  PCA9685, one set of topics, two writers.  Stop the flight launch first.
* servo_bench.launch is not the flight chain, so there is nothing to switch off
  when calibration is done.  tools/check_config.py fails if angle_debug:=true
  ever appears in navigation_2d.launch.
* The angle is clamped to 0..180 by the node, and the reply says when it was
  clamped.  There is no way to command a servo past its mechanical stop from
  here - if the mechanism needs one, the mechanism is wrong.

Usage
-----
    # one angle
    python3 tools/servo_debug.py 90

    # what would be sent, without ROS - safe to run anywhere
    python3 tools/servo_debug.py --print-commands --sweep 60 150 10

    # bracket a release angle: press enter after each step to look and record
    python3 tools/servo_debug.py --sweep 60 150 10

    # same, on a timer instead of on enter (e.g. while filming)
    python3 tools/servo_debug.py --sweep 60 150 10 --settle 1.5

    # hand-driven: type an angle, c <deg> for the camera, p park, s status
    python3 tools/servo_debug.py --interactive

    # camera channel, and a readout of what the node is using right now
    python3 tools/servo_debug.py --camera 55
    python3 tools/servo_debug.py --status
"""

import argparse
import math
import sys
import threading
import time

CMD_TOPIC = "/servo_debug_cmd"
ACK_TOPIC = "/servo_debug_ack"
NODE_NAME = "/servo_controller_node"

# When input is not a terminal - the sweep output is being logged to a file, or
# a script is driving this - there is nobody to press enter.  Firing the whole
# sweep at the mechanism with no pause is the one behaviour that could damage
# it, so fall back to a fixed delay instead of a zero one.
SETTLE_WITHOUT_TERMINAL = 1.0

HELP = """\
  <deg>       point the drop servo at <deg>        (SET_DROP <deg>)
  d <deg>     same thing
  c <deg>     point the camera servo at <deg>      (SET_CAMERA <deg>)
  p           park the drop servo                  (PARK)
  s           what the node is using right now     (STATUS)
  ?           this help
  q           quit
"""


# ---------------------------------------------------------------------------
# Payload construction.  Kept free of rospy so --print-commands works on any
# machine, and so the list of things this script can send is one function.
# ---------------------------------------------------------------------------
def fmt_angle(value):
    """90.0 -> '90', 87.5 -> '87.5'.  The node parses with std::stod, which
    accepts a trailing '.' or '90.0' equally well - this is for the human
    reading the reply, not for the parser."""
    text = ("%.4f" % float(value)).rstrip("0").rstrip(".")
    if text in ("", "-0"):
        return "0"
    return text


def sweep_angles(start, end, step):
    """Inclusive START..END.  The endpoint is appended even when STEP does not
    divide the span, because the angle you care about is usually the end."""
    if step <= 0.0:
        raise ValueError("--sweep step must be greater than zero")
    if end < start:
        raise ValueError("--sweep end must not be below start")
    count = int(math.floor((end - start) / step + 1e-9)) + 1
    angles = [start + i * step for i in range(count)]
    if abs(angles[-1] - end) > 1e-9:
        angles.append(end)
    return angles


def parse_interactive(line):
    """One typed line -> a /servo_debug_cmd payload, or a local verb.

    A bare number means the drop servo, because that is what calibration
    actually types.  Everything else is left to the node to validate: a
    non-numeric angle is echoed back as 'not a number' by the same code that
    will reject it in the air, which is one validator instead of two."""
    text = line.strip()
    if not text:
        return None
    fields = text.replace(":", " ").replace("=", " ").split()
    head = fields[0].lower()
    if head in ("q", "quit", "exit"):
        return "QUIT"
    if head in ("?", "h", "help"):
        return "HELP"
    if head in ("p", "park"):
        return "PARK"
    if head in ("s", "status"):
        return "STATUS"
    if head in ("d", "drop", "set_drop"):
        if len(fields) < 2:
            return "ERR DROP needs an angle"
        return "SET_DROP " + fields[1]
    if head in ("c", "camera", "set_camera"):
        if len(fields) < 2:
            return "ERR CAMERA needs an angle"
        return "SET_CAMERA " + fields[1]
    if len(fields) == 1:
        return "SET_DROP " + fields[0]
    return "ERR cannot read '%s'; type ? for help" % text


def plan_commands(args):
    """The payloads this invocation will send, in order."""
    out = []
    if args.status:
        out.append("STATUS")
    if args.park:
        out.append("PARK")
    if args.camera is not None:
        out.append("SET_CAMERA " + fmt_angle(args.camera))
    if args.sweep:
        for angle in sweep_angles(*args.sweep):
            out.append("SET_DROP " + fmt_angle(angle))
    if args.angle is not None:
        out.append("SET_DROP " + fmt_angle(args.angle))
    return out


# ---------------------------------------------------------------------------
# The ROS link.
# ---------------------------------------------------------------------------
class DebugLink(object):
    """Publishes on /servo_debug_cmd and collects the matching reply.

    The reply is a plain std_msgs/String with no request id, so "the matching
    reply" is defined the only way it can be: this class sends one command at
    a time and waits for the next String on /servo_debug_ack.  Every send()
    therefore has a timeout, and a timeout is reported rather than ignored -
    a silent servo and a rejected command have to look different."""

    def __init__(self):
        import rospy
        from std_msgs.msg import String

        self._rospy = rospy
        self._String = String
        self._ack = None
        self._ack_event = threading.Event()
        self._lock = threading.Lock()
        rospy.init_node("servo_debug", anonymous=True)
        self.pub = rospy.Publisher(CMD_TOPIC, String, queue_size=8)
        self.sub = rospy.Subscriber(ACK_TOPIC, String, self._on_ack)

    def _on_ack(self, message):
        self._ack = message.data
        self._ack_event.set()

    def is_shutdown(self):
        return self._rospy.is_shutdown()

    def wait_for_link(self, timeout):
        """True once the node is both reading our commands and writing acks."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self._rospy.is_shutdown():
                return False
            if self.pub.get_num_connections() > 0 and self.sub.get_num_connections() > 0:
                return True
            time.sleep(0.05)
        return False

    def send(self, text, timeout):
        """Return the reply string, or None if none arrived in time."""
        with self._lock:
            self._ack = None
            self._ack_event.clear()
            self.pub.publish(self._String(text))
            if not self._ack_event.wait(timeout):
                return None
            return self._ack


def running_node_names():
    """Node names from the master, or None when the master cannot be reached."""
    try:
        from rosgraph import rosnode

        names = rosnode.get_node_names()
        return list(names) if names is not None else []
    except Exception:
        return None


def explain_no_link(link):
    """Turn "publisher has no subscriber" into the two things it can mean."""
    lines = []
    lines.append("no link to servo_controller_node: %s has no subscriber and %s "
                 "has no publisher." % (CMD_TOPIC, ACK_TOPIC))
    names = running_node_names()
    if names is None:
        lines.append("  could not list nodes, so roscore may not be reachable from here.")
    elif NODE_NAME not in names:
        lines.append("  %s is NOT running. Start it on the bench:" % NODE_NAME)
        lines.append("    roslaunch servo_control servo_bench.launch")
    else:
        lines.append("  %s IS running but does not subscribe %s, so angle_debug "
                     "is false in its namespace - almost certainly because it was "
                     "started from navigation_2d.launch, which keeps it false on "
                     "purpose." % (NODE_NAME, CMD_TOPIC))
        lines.append("  Stop that launch, then start the bench one:")
        lines.append("    roslaunch servo_control servo_bench.launch")
        lines.append("  Passing angle_debug:=true to navigation_2d.launch does NOT "
                     "work: that launch never declares it as an <arg>, so roslaunch "
                     "drops the value silently.")
        lines.append("  If you cannot restart it, tools/servo_angle_calib.py drives "
                     "the PCA9685 directly instead - but stop the node first.")
    lines.append("  Check the node's own startup line: it prints 'angle_debug is ON' "
                 "or 'angle_debug is off' at construction.")
    return "\n".join(lines)


def print_reply(text):
    if text is None:
        print("  !! no reply within the timeout - the node did not answer on %s"
              % ACK_TOPIC)
        return
    if text.startswith("ERR"):
        print("  !! " + text)
    else:
        print("  -> " + text)


def pause_between_steps(args, index, total):
    """Return False to stop the sweep."""
    if args.settle is not None:
        time.sleep(max(0.0, args.settle))
        return True
    if not sys.stdin.isatty():
        time.sleep(SETTLE_WITHOUT_TERMINAL)
        return True
    try:
        reply = input("     [enter] %d/%d, 'q' to stop > " % (index + 1, total))
    except EOFError:
        return False
    return reply.strip().lower() not in ("q", "quit", "exit")


def run_script(link, args, commands):
    """--angle / --camera / --sweep / --status / --park: send, then wait."""
    print("connected: %s -> %s (angle_debug is on)" % (CMD_TOPIC, ACK_TOPIC))
    sweeping = bool(args.sweep)
    for index, text in enumerate(commands):
        print("%s" % text)
        reply = link.send(text, args.timeout)
        print_reply(reply)
        if reply is None:
            # One lost ack is worth reporting, but the next command may well
            # come back fine; only the sweep has a reason to stop here.
            if sweeping:
                print("     stopping the sweep so the last angle you saw stays put")
                return 0
            continue
        if sweeping and index + 1 < len(commands):
            if not pause_between_steps(args, index, len(commands)):
                print("     sweep stopped at %s" % text)
                break
    return 0


def run_interactive(link, args):
    print("connected: %s -> %s (angle_debug is on)" % (CMD_TOPIC, ACK_TOPIC))
    print("type ? for help, q to quit")
    while not link.is_shutdown():
        try:
            line = input("> ")
        except (EOFError, KeyboardInterrupt):
            print()
            break
        text = parse_interactive(line)
        if text is None:
            continue
        if text == "QUIT":
            break
        if text == "HELP":
            sys.stdout.write(HELP)
            continue
        if text.startswith("ERR"):
            print("  !! " + text)
            continue
        reply = link.send(text, args.timeout)
        print_reply(reply)
    # Park on the way out so the next person does not find a drop servo
    # sitting on a release angle.  Skipped when rospy itself is shutting down -
    # there is no link left to park over.
    if args.park_on_exit and not link.is_shutdown():
        reply = link.send("PARK", args.timeout)
        if reply is not None:
            print("-> parked; " + reply)
        else:
            print("  !! could not confirm PARK on the way out; check the servo "
                  "position by hand")
    return 0


def build_parser():
    parser = argparse.ArgumentParser(
        description="Type an angle and watch the drop servo move "
                    "(front end for /servo_debug_cmd).",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=HELP)
    parser.add_argument("angle", nargs="?", type=float,
                        help="drop-servo angle in degrees; same as SET_DROP")
    parser.add_argument("-c", "--camera", type=float, metavar="DEG",
                        help="set the camera servo instead (SET_CAMERA)")
    parser.add_argument("-s", "--sweep", nargs=3, type=float,
                        metavar=("START", "END", "STEP"),
                        help="step the drop servo from START to END; press enter "
                             "after each step unless --settle is given")
    parser.add_argument("--status", action="store_true",
                        help="print the angles the node is using right now")
    parser.add_argument("--park", action="store_true",
                        help="return the drop servo to drop_safe_angle and exit")
    parser.add_argument("-i", "--interactive", action="store_true",
                        help="hand-driven prompt: type an angle, c <deg>, p, s, q")
    parser.add_argument("--settle", type=float, default=None, metavar="SECONDS",
                        help="wait this long between sweep steps instead of "
                             "waiting for enter")
    parser.add_argument("--timeout", type=float, default=2.0, metavar="SECONDS",
                        help="how long to wait for each reply (default 2.0)")
    parser.add_argument("--connect-timeout", type=float, default=6.0,
                        metavar="SECONDS",
                        help="how long to wait for the node to appear "
                             "(default 6.0)")
    parser.add_argument("--no-park-on-exit", dest="park_on_exit",
                        action="store_false",
                        help="interactive mode: do not PARK when quitting")
    parser.add_argument("--print-commands", action="store_true",
                        help="print the payloads that would be sent and exit; "
                             "needs no ROS and no hardware")
    parser.set_defaults(park_on_exit=True)
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)

    if args.interactive and (args.sweep or args.angle is not None):
        sys.stderr.write("--interactive cannot be combined with an angle or "
                         "--sweep; run them one at a time.\n")
        return 2
    if args.angle is not None and args.sweep:
        sys.stderr.write("give either an angle or --sweep, not both.\n")
        return 2

    # A bad --sweep is a typing mistake, not a crash: report it the way
    # argparse reports a bad option instead of letting the traceback out.
    try:
        commands = plan_commands(args)
    except ValueError as exc:
        sys.stderr.write("%s\n" % exc)
        return 2

    if args.print_commands:
        for text in commands:
            print(text)
        return 0

    if not args.interactive and not commands:
        sys.stderr.write("nothing to do; pass an angle, --sweep, --camera, "
                         "--status, --park or --interactive.\n")
        return 2

    try:
        link = DebugLink()
    except ImportError:
        sys.stderr.write(
            "rospy is not importable, so there is no ROS to talk to.\n"
            "Source the workspace first:\n"
            "  source /opt/ros/noetic/setup.bash && source devel/setup.bash\n"
            "To check the payloads without ROS, use --print-commands.\n")
        return 2

    if not link.wait_for_link(args.connect_timeout):
        sys.stderr.write(explain_no_link(link) + "\n")
        return 3

    for value in (args.angle, args.camera):
        if value is not None and not (0.0 <= value <= 180.0):
            print("note: %s deg is outside 0..180; the node clamps it and says so"
                  % fmt_angle(value))

    if args.interactive:
        return run_interactive(link, args)
    return run_script(link, args, commands)


if __name__ == "__main__":
    sys.exit(main())
