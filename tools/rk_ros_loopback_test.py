#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Check ROS message transport on a dedicated, local validation master.

Python 2.7 and Python 3 are supported. Start roscore separately, for example:
  ROS_MASTER_URI=http://127.0.0.1:11321 ROS_IP=127.0.0.1 roscore -p 11321
Then run this script in the same ROS environment. It publishes only synthetic
messages under a fresh /rk_validation/<uuid>/ namespace. It does not start
drivers, call ROS services, connect to hardware, or validate a real FCU.
"""
from __future__ import print_function

import argparse
import json
import math
import os
import signal
import socket
import sys
import threading
import time
import uuid

try:
    from urllib.parse import urlparse
    import xmlrpc.client as xmlrpc_client
except ImportError:
    from urlparse import urlparse
    import xmlrpclib as xmlrpc_client


LOOPBACK_HOSTS = frozenset(("127.0.0.1", "localhost", "::1"))
WALL_CLOCK = getattr(time, "monotonic", time.time)


def checked_environment(environ):
    """Refuse the default master and every non-loopback ROS address."""
    uri = environ.get("ROS_MASTER_URI", "")
    parsed = urlparse(uri)
    try:
        port = parsed.port
    except ValueError:
        raise ValueError("ROS_MASTER_URI has an invalid port")
    if (parsed.scheme != "http" or parsed.hostname not in LOOPBACK_HOSTS or
            port is None or not 1 <= port <= 65535 or port == 11311 or
            parsed.username is not None or parsed.password is not None or
            parsed.path not in ("", "/") or parsed.params or parsed.query or
            parsed.fragment):
        raise ValueError("ROS_MASTER_URI must explicitly use a loopback host "
                         "and a dedicated port other than 11311")
    ros_ip = environ.get("ROS_IP", "")
    if ros_ip not in ("127.0.0.1", "::1"):
        raise ValueError("ROS_IP must be explicitly set to 127.0.0.1 or ::1")
    ros_hostname = environ.get("ROS_HOSTNAME", "")
    if ros_hostname and ros_hostname not in LOOPBACK_HOSTS:
        raise ValueError("ROS_HOSTNAME must be unset or a loopback host")
    # Check localhost resolution too: no external XML-RPC target may be used.
    for host in (parsed.hostname, ros_hostname):
        if host == "localhost":
            addresses = socket.getaddrinfo(host, None, 0, socket.SOCK_STREAM)
            if not addresses or any(item[4][0] not in ("127.0.0.1", "::1")
                                    for item in addresses):
                raise ValueError("localhost must resolve only to loopback")
    return uri


class TimeoutTransport(xmlrpc_client.Transport):
    def __init__(self, timeout):
        xmlrpc_client.Transport.__init__(self)
        self.timeout = timeout

    def make_connection(self, host):
        connection = xmlrpc_client.Transport.make_connection(self, host)
        connection.timeout = self.timeout
        return connection


def require_dedicated_master(uri, caller, timeout):
    """Read the master registry; never join a running application graph."""
    proxy = xmlrpc_client.ServerProxy(uri, transport=TimeoutTransport(timeout))
    response = proxy.getSystemState(caller)
    if response[0] != 1:
        raise RuntimeError("ROS master getSystemState failed: %s" % response[1])
    # A newly started roscore can have /rosout. Existing application nodes would
    # make a local master an unsuitable validation sandbox, even on another port.
    existing = set()
    for group in response[2]:
        for unused_name, nodes in group:
            existing.update(nodes)
    unexpected = sorted(node for node in existing if node != "/rosout")
    if unexpected:
        raise RuntimeError("Dedicated master already has application nodes: %s" %
                           ", ".join(unexpected))


def set_fields(message, fields):
    """Require actual message fields, including custom-message versions."""
    slots = set(getattr(message, "__slots__", ()))
    for name, value in fields.items():
        if name not in slots:
            raise RuntimeError("%s is missing required field %s (actual: %s)" %
                               (message._type, name, ", ".join(sorted(slots))))
        setattr(message, name, value)
    return message


def header_fields(message, rospy):
    if "header" not in getattr(message, "__slots__", ()):
        raise RuntimeError("%s is missing its header" % message._type)
    message.header.stamp = rospy.Time(123, 456000000)
    message.header.frame_id = "rk_validation_synthetic"
    return {"stamp": {"secs": 123, "nsecs": 456000000},
            "frame_id": "rk_validation_synthetic"}


def specifications(rospy, with_project_messages, token):
    from std_msgs.msg import String
    from geometry_msgs.msg import PoseStamped
    from mavros_msgs.msg import State, PositionTarget

    result = []
    message = set_fields(String(), {"data": "rk_validation:" + token})
    result.append(("string", String, message, {"data": message.data}))

    message = PoseStamped()
    header = header_fields(message, rospy)
    set_fields(message.pose.position, {"x": 1.25, "y": -2.5, "z": 0.75})
    set_fields(message.pose.orientation, {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0})
    result.append(("pose", PoseStamped, message,
                   {"header": header, "pose": {
                       "position": {"x": 1.25, "y": -2.5, "z": 0.75},
                       "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}}}))

    message = State()
    header = header_fields(message, rospy)
    fields = {"connected": True, "armed": False, "guided": False,
              "mode": "RK_VALIDATION_SYNTHETIC", "system_status": 3}
    set_fields(message, fields)
    expected = dict(fields)
    expected["header"] = header
    result.append(("state", State, message, expected))

    message = PositionTarget()
    header = header_fields(message, rospy)
    fields = {"coordinate_frame": PositionTarget.FRAME_LOCAL_NED,
              "type_mask": (PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY |
                            PositionTarget.IGNORE_AFZ | PositionTarget.IGNORE_YAW_RATE),
              "yaw": 0.25, "yaw_rate": 0.0}
    set_fields(message, fields)
    set_fields(message.position, {"x": 1.25, "y": -2.5, "z": 0.75})
    set_fields(message.velocity, {"x": 0.1, "y": -0.2, "z": 0.0})
    expected = dict(fields)
    expected.update({"header": header,
                     "position": {"x": 1.25, "y": -2.5, "z": 0.75},
                     "velocity": {"x": 0.1, "y": -0.2, "z": 0.0}})
    result.append(("position_target", PositionTarget, message, expected))

    if with_project_messages:
        from ssd_detection.msg import VisionDetection, VisionDetectionArray
        from navigation_2d.msg import TargetDetection

        detection_fields = {"class_name": "hazard_flammable", "zone": "hazard",
                            "confidence": 0.875, "xmin": 0.2, "ymin": 0.3,
                            "xmax": 0.6, "ymax": 0.7}
        detection = set_fields(VisionDetection(), detection_fields)
        result.append(("vision_detection", VisionDetection, detection,
                       detection_fields))

        message = VisionDetectionArray()
        header = header_fields(message, rospy)
        set_fields(message, {"image_width": 640, "image_height": 480,
                             "detections": [detection]})
        result.append(("vision_detection_array", VisionDetectionArray, message,
                       {"header": header, "image_width": 640, "image_height": 480,
                        "detections": [detection_fields]}))

        message = TargetDetection()
        header = header_fields(message, rospy)
        fields = {"mode": "hazard", "found": True, "stable": False,
                  "center_x": 256.0, "center_y": 240.0, "width": 256.0,
                  "height": 192.0, "pixel_dx": -64.0, "pixel_dy": 0.0,
                  "confidence": 0.875, "image_width": 640, "image_height": 480}
        set_fields(message, fields)
        expected = dict(fields)
        expected["header"] = header
        result.append(("target_detection", TargetDetection, message, expected))
    return result


def compare_fields(actual, expected, path=""):
    errors = []
    if isinstance(expected, dict):
        for name, value in expected.items():
            location = path + "." + name if path else name
            if not hasattr(actual, name):
                errors.append(location + " is missing")
            else:
                errors.extend(compare_fields(getattr(actual, name), value, location))
    elif isinstance(expected, list):
        if len(actual) != len(expected):
            errors.append(path + " has an unexpected length")
        else:
            for index, value in enumerate(expected):
                errors.extend(compare_fields(actual[index], value,
                                              path + "[%d]" % index))
    elif isinstance(expected, float):
        value = float(actual)
        if math.isnan(value) or math.isinf(value) or abs(value - expected) > 1e-5:
            errors.append("%s: expected %r, received %r" % (path, expected, actual))
    elif actual != expected:
        errors.append("%s: expected %r, received %r" % (path, expected, actual))
    return errors


def check_transport(rospy, namespace, specification, timeout):
    name, message_class, outgoing, expected = specification
    topic = namespace + "/" + name
    if rospy.resolve_name(topic) != topic:
        raise RuntimeError("Unexpected topic remapping: " + topic)
    lock = threading.Lock()
    received = []

    def callback(message):
        with lock:
            received.append(message)

    subscriber = rospy.Subscriber(topic, message_class, callback, queue_size=1)
    publisher = rospy.Publisher(topic, message_class, queue_size=1, latch=False)
    started = WALL_CLOCK()
    deadline = started + timeout
    try:
        while publisher.get_num_connections() == 0:
            if WALL_CLOCK() >= deadline or rospy.is_shutdown():
                raise RuntimeError("Timed out waiting for TCPROS connection: " + topic)
            time.sleep(0.02)
        while WALL_CLOCK() < deadline and not rospy.is_shutdown():
            publisher.publish(outgoing)
            time.sleep(0.05)
            with lock:
                message = received[-1] if received else None
            if message is not None:
                errors = compare_fields(message, expected)
                if errors:
                    raise RuntimeError("; ".join(errors))
                return {"topic": topic, "type": message_class._type,
                        "md5": message_class._md5sum,
                        "fields": list(message_class.__slots__),
                        "passed": True,
                        "elapsed_seconds": round(WALL_CLOCK() - started, 3)}
        raise RuntimeError("Timed out waiting for synthetic message: " + topic)
    finally:
        publisher.unregister()
        subscriber.unregister()


def _main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--with-project-messages", action="store_true",
                        help="also require and validate generated project messages")
    parser.add_argument("--timeout", type=float, default=10.0,
                        help="wall-clock timeout per message, 1 to 60 seconds")
    args = parser.parse_args(argv)
    token = uuid.uuid4().hex
    namespace = "/rk_validation/" + token
    report = {"passed": False, "namespace": namespace, "checks": [],
              "scope": "synthetic TCPROS message transport only; no hardware or FCU"}
    rospy = None
    try:
        if not 1.0 <= args.timeout <= 60.0:
            raise ValueError("--timeout must be between 1 and 60 seconds")
        uri = checked_environment(os.environ)
        report["master_uri"] = uri
        require_dedicated_master(uri, namespace + "/probe", args.timeout)
        # Melodic rospy caches its namespace at first import; argv alone does
        # not update that cached value on every installed rospy version.
        os.environ["ROS_NAMESPACE"] = namespace
        import rospy as rospy_module
        rospy = rospy_module
        # Ignore inherited command-line ROS remappings; use our absolute sandbox.
        rospy.init_node("probe", argv=[sys.argv[0], "__ns:=" + namespace],
                        anonymous=False, disable_signals=True, disable_rosout=True)
        if rospy.get_name() != namespace + "/probe":
            raise RuntimeError("Unexpected ROS node namespace: " + rospy.get_name())
        specs = specifications(rospy, args.with_project_messages, token)
        for spec in specs:
            try:
                report["checks"].append(check_transport(rospy, namespace, spec,
                                                       args.timeout))
            except Exception as error:
                report["checks"].append({"topic": namespace + "/" + spec[0],
                                         "type": spec[1]._type, "passed": False,
                                         "error": str(error)})
                raise
        report["passed"] = True
    except (Exception, KeyboardInterrupt) as error:
        report["error"] = str(error) or "Interrupted"
    finally:
        # A repeated SIGTERM must not interrupt ROS connection cleanup.
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        if rospy is not None:
            rospy.signal_shutdown("RK loopback validation finished")
    print(json.dumps(report, ensure_ascii=True, sort_keys=True))
    return 0 if report["passed"] else 1


def main(argv=None):
    previous_handler = signal.getsignal(signal.SIGTERM)

    def interrupt_on_sigterm(unused_signum, unused_frame):
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        raise KeyboardInterrupt("Terminated by SIGTERM")

    try:
        signal.signal(signal.SIGTERM, interrupt_on_sigterm)
        return _main(argv)
    finally:
        signal.signal(signal.SIGTERM, previous_handler)


if __name__ == "__main__":
    sys.exit(main())
