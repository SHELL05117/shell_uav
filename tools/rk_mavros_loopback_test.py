#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Validate MAVROS using synthetic MAVLink 1 on IPv4 loopback only.

Python 2.7/3; no pymavlink or hardware. Start a dedicated roscore separately:
  ROS_MASTER_URI=http://127.0.0.1:11321 ROS_IP=127.0.0.1 roscore -p 11321
Then, with the same environment and sourced Melodic installation:
  python rk_mavros_loopback_test.py --log-dir /path/to/evidence
Only the real mavros_node's sys_status, imu and local_position plugins run.
No arming, mode, mission, parameter or setpoint ROS services/topics are used.
"""
from __future__ import print_function

import argparse
import errno
import json
import math
import os
import signal
import socket
import struct
import subprocess
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


WALL_CLOCK = getattr(time, "monotonic", time.time)
# Official generated MAVLink 1 headers (wire order and CRC_EXTRA):
# https://github.com/mavlink/c_library_v1/blob/master/minimal/mavlink_msg_heartbeat.h
# https://github.com/mavlink/c_library_v1/blob/master/common/mavlink_msg_attitude.h
# https://github.com/mavlink/c_library_v1/blob/master/common/mavlink_msg_local_position_ned.h
# CRC accumulation: https://github.com/mavlink/c_library_v1/blob/master/checksum.h
CRC_EXTRA = {0: 50, 30: 39, 32: 185}
PAYLOAD_LENGTH = {0: 9, 30: 28, 32: 28}
POSITION_NED = (1.25, -2.5, -0.75)
POSITION_ENU = (-2.5, 1.25, 0.75)
GYRO_FRD = (0.1, 0.2, 0.3)
GYRO_FLU = (0.1, -0.2, -0.3)


def checked_environment(environ):
    uri = environ.get("ROS_MASTER_URI", "")
    parsed = urlparse(uri)
    try:
        port = parsed.port
    except ValueError:
        raise ValueError("Invalid ROS_MASTER_URI port")
    if (parsed.scheme != "http" or parsed.hostname != "127.0.0.1" or
            port is None or not 1 <= port <= 65535 or port == 11311 or
            parsed.username is not None or parsed.password is not None or
            parsed.path not in ("", "/") or parsed.params or parsed.query or
            parsed.fragment):
        raise ValueError("ROS_MASTER_URI must use http://127.0.0.1:<port>, "
                         "with an explicit non-11311 port")
    if environ.get("ROS_IP") != "127.0.0.1":
        raise ValueError("ROS_IP must explicitly be 127.0.0.1")
    if environ.get("ROS_HOSTNAME") not in (None, "", "127.0.0.1"):
        raise ValueError("ROS_HOSTNAME must be unset or 127.0.0.1")
    if environ.get("ROS_NAMESPACE"):
        raise ValueError("Unset inherited ROS_NAMESPACE before validation")
    if os.name != "posix":
        raise ValueError("Run on the Linux RK board; POSIX process groups required")
    return uri


class TimeoutTransport(xmlrpc_client.Transport):
    def make_connection(self, host):
        connection = xmlrpc_client.Transport.make_connection(self, host)
        connection.timeout = 3.0
        return connection


def require_dedicated_master(uri, caller):
    proxy = xmlrpc_client.ServerProxy(uri, transport=TimeoutTransport())
    response = proxy.getSystemState(caller)
    if response[0] != 1:
        raise RuntimeError("Cannot inspect validation master: " + response[1])
    existing = set()
    for group in response[2]:
        for unused_topic, nodes in group:
            existing.update(nodes)
    unexpected = sorted(existing.difference(("/rosout",)))
    if unexpected:
        raise RuntimeError("Validation master is not empty: " + ", ".join(unexpected))
    simulated_time = proxy.getParam(caller, "/use_sim_time")
    if simulated_time[0] == 1 and simulated_time[2]:
        raise RuntimeError("Dedicated master must not enable /use_sim_time")


def mavlink_crc(data):
    crc = 0xffff
    for value in bytearray(data):
        temporary = value ^ (crc & 255)
        temporary = (temporary ^ (temporary << 4)) & 255
        crc = ((crc >> 8) ^ (temporary << 8) ^
               (temporary << 3) ^ (temporary >> 4)) & 0xffff
    return crc


def mavlink1_packet(message_id, payload, sequence):
    if message_id not in CRC_EXTRA or len(payload) != PAYLOAD_LENGTH[message_id]:
        raise ValueError("Only the three fixed telemetry payloads are permitted")
    # MAVLink 1: length, sequence, system=1, component=1, message ID.
    header = struct.pack("<BBBBB", len(payload), sequence & 255, 1, 1, message_id)
    crc = mavlink_crc(header + payload + struct.pack("B", CRC_EXTRA[message_id]))
    return b"\xfe" + header + payload + struct.pack("<H", crc)


def valid_downlink_heartbeats(datagram):
    """Count only complete, CRC-valid MAVLink 1 HEARTBEATs from MAVROS 255.240."""
    octets = bytearray(datagram)
    offset, valid = 0, 0
    while offset + 8 <= len(octets):
        if octets[offset] != 254:
            offset += 1
            continue
        payload_length = octets[offset + 1]
        end = offset + payload_length + 8
        if end > len(octets):
            break
        if (payload_length == 9 and octets[offset + 3] == 255 and
                octets[offset + 4] == 240 and octets[offset + 5] == 0):
            expected_crc = mavlink_crc(datagram[offset + 1:end - 2] +
                                      struct.pack("B", CRC_EXTRA[0]))
            actual_crc = struct.unpack("<H", datagram[end - 2:end])[0]
            if actual_crc == expected_crc:
                valid += 1
        offset = end
    return valid


def telemetry_payloads(milliseconds):
    # minimal.xml: QUADROTOR=2, PX4=12, base_mode=0 (never armed),
    # STANDBY=3; the HEARTBEAT mavlink_version field is always 3.
    heartbeat = struct.pack("<IBBBBB", 0, 2, 12, 0, 3, 3)
    attitude = struct.pack("<Iffffff", milliseconds, 0.0, 0.0, 0.0, *GYRO_FRD)
    position = struct.pack("<Iffffff", milliseconds,
                           *(POSITION_NED + (0.0, 0.0, 0.0)))
    return ((0, heartbeat), (30, attitude), (32, position))


def vector_values(vector):
    return [vector.x, vector.y, vector.z]


def close_vector(actual, expected):
    return all(not math.isnan(value) and not math.isinf(value) and
               abs(value - target) < 1e-5
               for value, target in zip(actual, expected))


def unit_quaternion(quaternion):
    values = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
    return (all(not math.isnan(value) and not math.isinf(value) for value in values)
            and abs(sum(value * value for value in values) - 1.0) < 1e-4)


def stop_child(process):
    """Signal only our own new process group, then reap it with bounded waits."""
    actions = []
    for signum, seconds in ((signal.SIGINT, 3.0), (signal.SIGTERM, 2.0),
                            (signal.SIGKILL, 2.0)):
        if process.poll() is not None:
            break
        try:
            os.killpg(process.pid, signum)
            actions.append(signum)
        except OSError:
            pass
        deadline = WALL_CLOCK() + seconds
        while process.poll() is None and WALL_CLOCK() < deadline:
            time.sleep(0.05)
    return {"returncode": process.poll(), "signals": actions,
            "stopped": process.poll() is not None}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--log-dir", required=True,
                        help="directory for complete MAVROS stdout and ROS logs")
    parser.add_argument("--timeout", type=float, default=25.0,
                        help="wall-clock timeout for startup and reception, 5 to 60s")
    args = parser.parse_args(argv)
    token = uuid.uuid4().hex
    namespace = "/rk_validation/mavros_" + token
    probe_name = "/rk_validation/probe_" + token
    report = {"passed": False, "namespace": namespace,
              "checks": {"bidirectional_heartbeat": False},
              "scope": "real MAVROS with synthetic loopback MAVLink; no real FCU",
              "plugins": ["sys_status", "imu", "local_position"],
              "synthetic_system_id": 1, "synthetic_component_id": 1}
    rospy = process = udp = reserved = log_handle = None
    subscribers = []
    alarm_installed = False
    previous_alarm = None
    previous_term = None
    started = WALL_CLOCK()
    try:
        if not 5.0 <= args.timeout <= 60.0:
            raise ValueError("--timeout must be between 5 and 60 seconds")
        uri = checked_environment(os.environ)
        report["master_uri"] = uri
        def termination_handler(unused_signal, unused_frame):
            raise KeyboardInterrupt("Received SIGTERM")
        previous_term = signal.signal(signal.SIGTERM, termination_handler)
        require_dedicated_master(uri, probe_name)
        # Bound all subsequent startup/XML-RPC operations as well as the loop.
        def alarm_handler(unused_signal, unused_frame):
            raise RuntimeError("Validation wall-clock deadline exceeded")
        previous_alarm = signal.signal(signal.SIGALRM, alarm_handler)
        signal.setitimer(signal.ITIMER_REAL, args.timeout)
        alarm_installed = True

        # Melodic rospy can cache its namespace during import, before init_node
        # parses argv. Set the controlled namespace before importing it.
        os.environ["ROS_NAMESPACE"] = "/rk_validation"
        import rospy as rospy_module
        from roslib.packages import find_node
        from mavros_msgs.msg import State
        from geometry_msgs.msg import PoseStamped
        from sensor_msgs.msg import Imu
        rospy = rospy_module
        rospy.init_node("probe_" + token,
                        argv=[sys.argv[0], "__ns:=/rk_validation"],
                        disable_signals=True, disable_rosout=True)
        if rospy.get_name() != probe_name:
            raise RuntimeError("Unexpected ROS remapping of the probe node")

        received, lock = {}, threading.Lock()
        def callback(name):
            def store(message):
                with lock:
                    received[name] = message
            return store
        for suffix, message_class in (("state", State),
                                      ("local_position/pose", PoseStamped),
                                      ("imu/data", Imu)):
            topic = namespace + "/" + suffix
            if rospy.resolve_name(topic) != topic:
                raise RuntimeError("Unexpected ROS topic remapping: " + topic)
            subscribers.append(rospy.Subscriber(topic, message_class,
                                                callback(suffix), queue_size=2))

        udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        udp.bind(("127.0.0.1", 0))
        simulated_port = udp.getsockname()[1]
        reserved = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        reserved.bind(("127.0.0.1", 0))
        mavros_port = reserved.getsockname()[1]
        udp.setblocking(False)
        fcu_url = "udp://127.0.0.1:%d@127.0.0.1:%d" % (mavros_port, simulated_port)
        report["fcu_url"] = fcu_url
        parameters = {"fcu_url": fcu_url, "gcs_url": "", "fcu_protocol": "v1.0",
                      "system_id": 255, "component_id": 240,
                      "target_system_id": 1, "target_component_id": 1,
                      "startup_px4_usb_quirk": False,
                      "plugin_blacklist": ["*"],
                      "plugin_whitelist": report["plugins"],
                      "local_position/tf/send": False, "conn/timeout": 5.0,
                      "conn/heartbeat_rate": 1.0}
        for key, value in parameters.items():
            rospy.set_param(namespace + "/" + key, value)

        log_dir = os.path.abspath(os.path.join(args.log_dir, "mavros_" + token))
        os.makedirs(log_dir)
        report["log_directory"] = log_dir
        report["mavros_log"] = os.path.join(log_dir, "mavros.stdout.log")
        child_environment = os.environ.copy()
        child_environment.pop("ROS_NAMESPACE", None)
        child_environment.pop("ROS_HOSTNAME", None)
        child_environment["ROS_LOG_DIR"] = log_dir
        candidates = find_node("mavros", "mavros_node")
        if not candidates:
            raise RuntimeError("Cannot find the installed mavros_node executable")
        command = [candidates[0], "__name:=mavros_" + token,
                   "__ns:=/rk_validation"]
        report["command"] = command
        log_handle = open(report["mavros_log"], "wb")
        reserved.close()
        reserved = None
        with open(os.devnull, "rb") as null_input:
            process = subprocess.Popen(command, stdout=log_handle,
                                       stderr=subprocess.STDOUT, stdin=null_input,
                                       env=child_environment, preexec_fn=os.setsid,
                                       close_fds=True)
        sequence = packets_sent = packets_received = heartbeats_received = 0
        messages = {}
        deadline = started + args.timeout
        while WALL_CLOCK() < deadline and not rospy.is_shutdown():
            if process.poll() is not None:
                raise RuntimeError("mavros_node exited early; see complete MAVROS log")
            milliseconds = int((WALL_CLOCK() - started) * 1000) & 0xffffffff
            for message_id, payload in telemetry_payloads(milliseconds):
                udp.sendto(mavlink1_packet(message_id, payload, sequence),
                           ("127.0.0.1", mavros_port))
                sequence += 1
                packets_sent += 1
            while True:
                try:
                    packet, peer = udp.recvfrom(65535)
                except socket.error as error:
                    if error.errno in (errno.EAGAIN, errno.EWOULDBLOCK):
                        break
                    raise
                if peer != ("127.0.0.1", mavros_port):
                    raise RuntimeError("Unexpected sender on synthetic UDP endpoint")
                packets_received += 1
                heartbeats_received += valid_downlink_heartbeats(packet)
            report["packets_sent"] = packets_sent
            report["mavros_datagrams_received"] = packets_received
            report["valid_mavros_heartbeats_received"] = heartbeats_received
            report["checks"]["bidirectional_heartbeat"] = heartbeats_received > 0
            with lock:
                messages = dict(received)
            state = messages.get("state")
            pose = messages.get("local_position/pose")
            imu = messages.get("imu/data")
            if state is not None and state.armed:
                raise RuntimeError("Unexpected armed state in synthetic validation")
            if (state is not None and state.connected and pose is not None and
                    imu is not None and heartbeats_received > 0):
                position = vector_values(pose.pose.position)
                gyro = vector_values(imu.angular_velocity)
                report["checks"] = {
                    "connected_unarmed": bool(state.connected and not state.armed),
                    "bidirectional_heartbeat": bool(heartbeats_received > 0),
                    "ned_to_enu_position": close_vector(position, POSITION_ENU),
                    "frd_to_flu_angular_velocity": close_vector(gyro, GYRO_FLU),
                    "imu_unit_quaternion": unit_quaternion(imu.orientation)}
                report["received"] = {"position_enu": position, "gyro_flu": gyro,
                                      "connected": state.connected, "armed": state.armed,
                                      "imu_frame_id": imu.header.frame_id,
                                      "pose_frame_id": pose.header.frame_id}
                if not all(report["checks"].values()):
                    raise RuntimeError("Synthetic telemetry field validation failed")
                report["passed"] = True
                break
            time.sleep(0.1)
        if not report["passed"]:
            report["topics_received"] = sorted(messages)
            raise RuntimeError("Timed out waiting for connected state, pose, IMU "
                               "and a CRC-valid MAVROS downlink HEARTBEAT")
    except (Exception, KeyboardInterrupt) as error:
        report["error"] = str(error) or "Interrupted"
    finally:
        if alarm_installed:
            signal.setitimer(signal.ITIMER_REAL, 0)
            signal.signal(signal.SIGALRM, previous_alarm)
        if process is not None:
            report["child_cleanup"] = stop_child(process)
            if not report["child_cleanup"]["stopped"]:
                report["passed"] = False
                report["error"] = "Failed to stop validation mavros_node"
        for subscriber in subscribers:
            subscriber.unregister()
        for handle in (udp, reserved, log_handle):
            if handle is not None:
                handle.close()
        if rospy is not None:
            rospy.signal_shutdown("MAVROS loopback validation finished")
        if previous_term is not None:
            signal.signal(signal.SIGTERM, previous_term)
        report["elapsed_seconds"] = round(WALL_CLOCK() - started, 3)
    print(json.dumps(report, ensure_ascii=True, sort_keys=True))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
