#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Run two built ROS applications with synthetic, isolated loopback inputs.

Start a dedicated roscore separately and source the workspace's devel/setup.bash.
Set ROS_MASTER_URI=http://127.0.0.1:11321 and ROS_IP=127.0.0.1, then run:
  python tools/rk_application_loopback_test.py --workspace /path/to/catkin_ws \
      --log-dir /path/to/validation-logs

Keep rk_ros_loopback_test.py beside this file; it supplies the shared environment
and empty-master checks. Python 2.7 and Python 3 are supported. No launch file,
mission node, servo node, real MAVROS topic, I2C device, or UART is opened here.
"""
from __future__ import print_function

import argparse
import json
import math
import os
import signal
import subprocess
import sys
import threading
import time
import uuid

from rk_ros_loopback_test import (WALL_CLOCK, checked_environment, compare_fields,
                                  require_dedicated_master)


class Samples(object):
    def __init__(self):
        self.lock = threading.Lock()
        self.values = []

    def callback(self, message):
        with self.lock:
            self.values.append((WALL_CLOCK(), message))
            self.values = self.values[-500:]

    def snapshot(self):
        with self.lock:
            return list(self.values)


def wait_until(rospy, process, timeout, description, pump, condition):
    deadline = WALL_CLOCK() + timeout
    while WALL_CLOCK() < deadline:
        if rospy.is_shutdown():
            raise RuntimeError("ROS shut down while " + description)
        if process.poll() is not None:
            raise RuntimeError("Application exited with %s while %s" %
                               (process.returncode, description))
        pump()
        result = condition()
        if result is not None:
            return result
        time.sleep(0.02)
    raise RuntimeError("Timed out while " + description)


def stop_process(process):
    if process.poll() is not None:
        if process.returncode != 0:
            raise RuntimeError("Validation child PID %s exited with %s; expected 0" %
                               (process.pid, process.returncode))
        return
    # Each application has its own process group. Never signal the caller,
    # roscore, SSH session, or any process selected by name.
    for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
        try:
            os.killpg(process.pid, signum)
        except OSError:
            if process.poll() is not None:
                break
            raise
        deadline = WALL_CLOCK() + (3.0 if signum == signal.SIGINT else 1.0)
        while process.poll() is None and WALL_CLOCK() < deadline:
            time.sleep(0.05)
        if process.poll() is not None:
            process.wait()
            if process.returncode != 0:
                raise RuntimeError("Validation child PID %s exited with %s after %s; "
                                   "expected 0" %
                                   (process.pid, process.returncode, signum))
            return
    if process.poll() is not None:
        if process.returncode != 0:
            raise RuntimeError("Validation child PID %s exited with %s; expected 0" %
                               (process.pid, process.returncode))
        return
    raise RuntimeError("Could not stop validation child PID %s" % process.pid)


def start_application(workspace, relative_binary, name, arguments, namespace,
                      log_dir, children, log_handles):
    binary = os.path.join(workspace, "devel", "lib", *relative_binary)
    if not os.path.isfile(binary) or not os.access(binary, os.X_OK):
        raise RuntimeError("Missing executable: " + binary)
    log_path = os.path.join(log_dir, name + ".stdout.log")
    log_handle = open(log_path, "wb")
    log_handles.append(log_handle)
    argv = [binary, "__name:=" + name, "__ns:=" + namespace,
            "/tf:=" + namespace + "/tf",
            "/tf_static:=" + namespace + "/tf_static",
            "/rosout:=" + namespace + "/rosout"] + arguments
    environment = os.environ.copy()
    environment["ROS_NAMESPACE"] = namespace
    environment["ROS_LOG_DIR"] = log_dir
    process = subprocess.Popen(argv, cwd=workspace, env=environment,
                               stdout=log_handle, stderr=subprocess.STDOUT,
                               preexec_fn=os.setsid, close_fds=True)
    children.append(process)
    return process, {"binary": binary, "arguments": argv[1:],
                     "stdout_log": log_path, "pid": process.pid}


def publish_transform(rospy, publisher, tf_message_class, transform_class,
                      position):
    transform = transform_class()
    transform.header.stamp = rospy.Time.now()
    transform.header.frame_id = "odom"
    transform.child_frame_id = "base_link"
    transform.transform.translation.x = position[0]
    transform.transform.translation.y = position[1]
    transform.transform.translation.z = position[2]
    transform.transform.rotation.w = 1.0
    message = tf_message_class()
    message.transforms = [transform]
    publisher.publish(message)


def location_check(rospy, workspace, namespace, log_dir, timeout, children,
                   log_handles, tf_publisher, TFMessage, TransformStamped):
    from geometry_msgs.msg import PoseStamped

    output_topic = namespace + "/location/vision_pose/pose"
    samples = Samples()
    subscriber = rospy.Subscriber(output_topic, PoseStamped, samples.callback,
                                  queue_size=10)
    arguments = ["_is_simulation:=true", "_use_tf_mini:=false",
                 "_rplidar_location:=true", "_target_frame_id:=odom",
                 "_source_frame_id:=base_link", "_publish_rate:=20.0",
                 "/mavros/vision_pose/pose:=" + output_topic]
    process = None
    try:
        process, details = start_application(
            workspace, ("location", "location_node"), "location_probe", arguments,
            namespace, log_dir, children, log_handles)
        observations = []
        for position in ((1.25, -2.5, 0.75), (1.35, -2.4, 0.85)):
            sent_after = WALL_CLOCK()

            def pump():
                publish_transform(rospy, tf_publisher, TFMessage, TransformStamped,
                                  position)

            def matching_pose():
                expected = {"header": {"frame_id": "odom"}, "pose": {
                    "position": dict(zip(("x", "y", "z"), position)),
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}}}
                for received_at, message in samples.snapshot():
                    if received_at >= sent_after and not compare_fields(message, expected):
                        if message.header.stamp.to_sec() <= 0.0:
                            raise RuntimeError("Location output has no timestamp")
                        return {"x": message.pose.position.x,
                                "y": message.pose.position.y,
                                "z": message.pose.position.z,
                                "frame_id": message.header.frame_id}
                return None

            observations.append(wait_until(
                rospy, process, timeout, "checking location TF-to-pose output",
                pump, matching_pose))
        details.update({"name": "location_tf_to_pose", "passed": True,
                        "output_topic": output_topic, "observations": observations,
                        "hardware_gate": "is_simulation=true and use_tf_mini=false"})
        return details
    finally:
        if process is not None:
            stop_process(process)
        subscriber.unregister()


def twist_values(message):
    return [message.linear.x, message.linear.y, message.linear.z,
            message.angular.x, message.angular.y, message.angular.z]


def is_forward(message):
    values = twist_values(message)
    if any(math.isnan(value) or math.isinf(value) for value in values):
        raise RuntimeError("Follower emitted a non-finite velocity")
    return 0.01 < values[0] <= 0.35001 and all(abs(value) <= 1e-6
                                              for value in values[1:])


def is_zero(message):
    values = twist_values(message)
    return all(not math.isnan(value) and not math.isinf(value) and abs(value) <= 1e-6
               for value in values)


def follower_check(rospy, workspace, namespace, log_dir, timeout, children,
                   log_handles, tf_publisher, TFMessage, TransformStamped):
    from geometry_msgs.msg import PoseStamped, Twist
    from nav_msgs.msg import Path
    from std_msgs.msg import Bool

    path_topic = namespace + "/follower/path"
    trigger_topic = namespace + "/follower/trigger"
    output_topic = namespace + "/follower/cmd_vel"
    samples = Samples()
    subscriber = rospy.Subscriber(output_topic, Twist, samples.callback, queue_size=20)
    path_publisher = rospy.Publisher(path_topic, Path, queue_size=1, latch=False)
    trigger_publisher = rospy.Publisher(trigger_topic, Bool, queue_size=1, latch=True)
    command_timeout = 0.4
    arguments = ["_global_frame:=odom", "_base_frame:=base_link",
                 "_trajectory_topic:=" + path_topic, "_trigger_topic:=" + trigger_topic,
                 "_cmd_vel_topic:=" + output_topic, "_command_timeout:=0.4",
                 "_control_rate:=20.0", "_max_velocity:=0.35",
                 "_max_acceleration:=0.50", "_lookahead_distance:=0.40",
                 "/cmd_vel:=" + output_topic,
                 "/trigger_plan:=" + trigger_topic,
                 "/visual_local_trajectory:=" + path_topic]
    process = None
    try:
        process, details = start_application(
            workspace, ("navigation_2d", "ego_trajectory_controller_node"),
            "follower_probe", arguments, namespace, log_dir, children, log_handles)

        def tf_only():
            publish_transform(rospy, tf_publisher, TFMessage, TransformStamped,
                              (0.0, 0.0, 0.0))

        def connected():
            if (path_publisher.get_num_connections() > 0 and
                    trigger_publisher.get_num_connections() > 0 and
                    tf_publisher.get_num_connections() > 0 and
                    subscriber.get_num_connections() > 0):
                return True
            return None

        wait_until(rospy, process, timeout, "waiting for follower connections",
                   tf_only, connected)

        def fresh_path():
            tf_only()
            trajectory = Path()
            trajectory.header.stamp = rospy.Time.now()
            trajectory.header.frame_id = "odom"
            for x in (0.0, 1.0, 2.0):
                pose = PoseStamped()
                pose.header = trajectory.header
                pose.pose.position.x = x
                pose.pose.orientation.w = 1.0
                trajectory.poses.append(pose)
            path_publisher.publish(trajectory)

        trigger_publisher.publish(Bool(data=True))
        phase_start = WALL_CLOCK()

        def forward_command():
            for received_at, message in reversed(samples.snapshot()):
                if received_at >= phase_start and is_forward(message):
                    return {"linear_x": message.linear.x, "linear_y": message.linear.y,
                            "linear_z": message.linear.z}
            return None

        first_forward = wait_until(rospy, process, timeout,
                                   "checking positive-X follower output",
                                   fresh_path, forward_command)
        stopped_publishing_at = WALL_CLOCK()

        def three_zero_samples(cutoff):
            recent = [(received_at, message) for received_at, message in samples.snapshot()
                      if received_at >= cutoff]
            if len(recent) >= 3 and all(is_zero(item[1]) for item in recent[-3:]):
                return {"consecutive_zero_samples": 3,
                        "last_zero_at": recent[-1][0]}
            return None

        stale_result = wait_until(
            rospy, process, timeout, "checking stale-path zero velocity", tf_only,
            lambda: three_zero_samples(stopped_publishing_at + command_timeout + 0.1))
        stale_result["elapsed_since_path_stop"] = round(
            stale_result.pop("last_zero_at") - stopped_publishing_at, 3)

        # Show positive output again before disabling the trigger, so a stale
        # path cannot accidentally satisfy the trigger=false assertion.
        phase_start = WALL_CLOCK()
        resumed_forward = wait_until(rospy, process, timeout,
                                     "checking resumed follower output",
                                     fresh_path, forward_command)
        disabled_at = WALL_CLOCK()
        trigger_publisher.publish(Bool(data=False))

        def disabled_with_fresh_path():
            fresh_path()
            trigger_publisher.publish(Bool(data=False))

        disabled_result = wait_until(
            rospy, process, timeout, "checking trigger=false with fresh paths",
            disabled_with_fresh_path,
            lambda: three_zero_samples(disabled_at + 0.15))
        disabled_result["elapsed_since_disable"] = round(
            disabled_result.pop("last_zero_at") - disabled_at, 3)
        details.update({"name": "ego_follower_path_to_velocity", "passed": True,
                        "output_topic": output_topic,
                        "positive_x": first_forward, "stale_path_zero": stale_result,
                        "resumed_positive_x": resumed_forward,
                        "trigger_false_zero_with_fresh_paths": disabled_result,
                        "command_timeout_seconds": command_timeout})
        return details
    finally:
        if process is not None:
            stop_process(process)
        path_publisher.unregister()
        trigger_publisher.unregister()
        subscriber.unregister()


def _main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", required=True,
                        help="built catkin workspace containing devel/lib")
    parser.add_argument("--log-dir", required=True,
                        help="directory for child stdout and ROS diagnostic logs")
    parser.add_argument("--timeout", type=float, default=10.0,
                        help="wall-clock timeout per assertion, 2 to 60 seconds")
    args = parser.parse_args(argv)
    namespace = "/rk_validation/" + uuid.uuid4().hex
    report = {"passed": False, "namespace": namespace, "checks": [],
              "scope": "synthetic location and follower application checks; no hardware"}
    children = []
    log_handles = []
    tf_publisher = None
    rospy = None
    active_check = "preflight"
    try:
        if os.name != "posix":
            raise RuntimeError("Run against a built Linux catkin workspace")
        if not 2.0 <= args.timeout <= 60.0:
            raise ValueError("--timeout must be between 2 and 60 seconds")
        uri = checked_environment(os.environ)
        report["master_uri"] = uri
        require_dedicated_master(uri, namespace + "/application_probe", args.timeout)
        workspace = os.path.abspath(args.workspace)
        if not os.path.isdir(os.path.join(workspace, "devel", "lib")):
            raise RuntimeError("Workspace has no devel/lib: " + workspace)
        log_dir = os.path.abspath(args.log_dir)
        if not os.path.isdir(log_dir):
            os.makedirs(log_dir)
        os.environ["ROS_LOG_DIR"] = log_dir
        report["log_dir"] = log_dir
        # Set this before importing rospy or any ROS message module: Melodic
        # can cache the namespace before init_node processes __ns arguments.
        os.environ["ROS_NAMESPACE"] = namespace
        import rospy as rospy_module
        from geometry_msgs.msg import TransformStamped
        from tf2_msgs.msg import TFMessage
        rospy = rospy_module
        rospy.init_node("application_probe",
                        argv=[sys.argv[0], "__ns:=" + namespace],
                        anonymous=False, disable_signals=True, disable_rosout=True)
        if rospy.get_name() != namespace + "/application_probe":
            raise RuntimeError("Unexpected application probe namespace")
        if rospy.get_param("/use_sim_time", False):
            raise RuntimeError("Dedicated master must use wall time, not /use_sim_time")
        tf_topic = namespace + "/tf"
        if rospy.resolve_name(tf_topic) != tf_topic:
            raise RuntimeError("Unexpected TF topic remapping")
        tf_publisher = rospy.Publisher(tf_topic, TFMessage, queue_size=10, latch=False)
        active_check = "location_tf_to_pose"
        report["checks"].append(location_check(
            rospy, workspace, namespace, log_dir, args.timeout, children, log_handles,
            tf_publisher, TFMessage, TransformStamped))
        active_check = "ego_follower_path_to_velocity"
        report["checks"].append(follower_check(
            rospy, workspace, namespace, log_dir, args.timeout, children, log_handles,
            tf_publisher, TFMessage, TransformStamped))
        report["passed"] = True
    except (Exception, KeyboardInterrupt) as error:
        report["failed_check"] = active_check
        report["error"] = str(error) or "Interrupted"
    finally:
        # Complete process-group cleanup even if another SIGTERM arrives.
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        cleanup_errors = []
        for process in reversed(children):
            try:
                stop_process(process)
            except Exception as error:
                cleanup_errors.append(str(error))
        report["child_processes"] = [{"pid": child.pid, "returncode": child.poll()}
                                      for child in children]
        for handle in log_handles:
            handle.close()
        if tf_publisher is not None:
            tf_publisher.unregister()
        if rospy is not None:
            rospy.signal_shutdown("Application loopback validation finished")
        if cleanup_errors:
            report["passed"] = False
            report["cleanup_errors"] = cleanup_errors
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
