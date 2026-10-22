#!/usr/bin/env python3
"""Observe a running, disarmed SITL without moving it or resetting its clock."""

import argparse
from collections import Counter
import math
import threading
import time

import rospy
from gazebo_msgs.msg import ModelStates
from mavros_msgs.msg import State
from nav_msgs.msg import OccupancyGrid
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CompressedImage, Imu, LaserScan
import tf


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=45)
    args = parser.parse_args()
    rospy.init_node("check_px4_slam", anonymous=True)
    counts = Counter()
    latest = {}
    errors = set()
    lock = threading.Lock()

    def observe(msg, key):
        with lock:
            counts[key] += 1
            if key == "clock" and key in latest:
                if msg.clock < latest[key].clock:
                    errors.add("Simulation clock moved backwards")
            latest[key] = msg

    # The annotated feed depends on the vision backend the SITL was started
    # with: hsv (default) publishes /mission/video/compressed from the mission
    # node, ssd publishes /ssd_detection/image/compressed from the detector
    # node. Subscribe to both and require one of them to flow, so this check
    # works for either backend instead of only the default one.
    video_topics = {
        "video_hsv": "/mission/video/compressed",
        "video_ssd": "/ssd_detection/image/compressed",
    }

    subscribers = [
        rospy.Subscriber(topic, kind, observe, key, queue_size=10)
        for key, topic, kind in (
            ("clock", "/clock", Clock),
            ("models", "/gazebo/model_states", ModelStates),
            ("state", "/mavros/state", State),
            ("scan", "/scan", LaserScan),
            ("imu", "/mavros/imu/data", Imu),
            ("map", "/map", OccupancyGrid),
        )
    ]
    for key, topic in video_topics.items():
        subscribers.append(
            rospy.Subscriber(topic, CompressedImage, observe, key,
                             queue_size=10))
    listener = tf.TransformListener()
    deadline = time.monotonic() + args.duration
    connected_once = False
    tf_ok = False
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        time.sleep(0.1)  # Wall time: a stalled /clock must not stall the test.
        with lock:
            state = latest.get("state")
            if state:
                if connected_once and not state.connected:
                    errors.add("PX4 disconnected during observation")
                connected_once = connected_once or state.connected
                if state.armed:
                    errors.add("Test expects a disarmed aircraft")
            models = latest.get("models")
            if models and "iris" in models.name:
                pose = models.pose[models.name.index("iris")]
                if not (-0.1 <= pose.position.z <= 0.6):
                    errors.add("Disarmed Iris left the ground/spawn height range")
                if not (-0.65 < pose.position.x < 3.35 and
                        -3.35 < pose.position.y < 0.65):
                    errors.add("Iris is outside the competition floor")
        try:
            listener.lookupTransform("map", "laser_link", rospy.Time(0))
            tf_ok = True
        except (tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException):
            pass

    with lock:
        for key, minimum in (("clock", 50), ("models", 20), ("state", 3),
                             ("scan", 20), ("imu", 20), ("map", 2)):
            print("{}: {} messages".format(key, counts[key]))
            if counts[key] < minimum:
                errors.add("{} has too few messages".format(key))
        video_total = sum(counts[key] for key in video_topics)
        print("annotated feed: {} messages (hsv {}, ssd {})".format(
            video_total, counts["video_hsv"], counts["video_ssd"]))
        if video_total < 5:
            errors.add(
                "annotated feed has too few messages (hsv {}, ssd {}) - "
                "neither /mission/video/compressed nor "
                "/ssd_detection/image/compressed is flowing".format(
                    counts["video_hsv"], counts["video_ssd"]))
        if not connected_once:
            errors.add("PX4 never connected")
        if not tf_ok:
            errors.add("Missing map -> laser_link TF chain")
        grid = latest.get("map")
        if grid:
            occupied = sum(value >= 65 for value in grid.data)
            free = sum(0 <= value < 50 for value in grid.data)
            print("map: {}x{}, occupied={}, free={}".format(
                grid.info.width, grid.info.height, occupied, free))
            if not occupied or not free:
                errors.add("Map lacks measured obstacles or free space")
        scan = latest.get("scan")
        if scan:
            finite = [value for value in scan.ranges if math.isfinite(value)]
            hits = sum(math.isfinite(value) and
                       scan.range_min < value < scan.range_max
                       for value in scan.ranges)
            # Gazebo's ray sensor does not drop a beam that hit nothing and does
            # not return inf: it returns range_max exactly. cartographer_ros
            # counts echo <= max_range as a HIT, so a scan that is all range_max
            # is not "empty", it is a scan of phantom obstacles at 3.50 m, and
            # the front end will happily match those phantoms every frame. That
            # is the failure mode this check exists for: on 2026-09-24 the lidar
            # was flying 0.15 m above the top of every wall in
            # gc_2026_field.world, so ALL 360 beams returned range_max, the map
            # grew to 5.55 x 7.85 m for a 4.05 x 4.05 m field and map->odom
            # drifted until the safety field aborted the mission.
            phantoms = sum(math.isfinite(value) and
                           value >= scan.range_max - 1e-6
                           for value in scan.ranges)
            total = len(scan.ranges)
            if finite:
                print("scan: {} finite obstacle returns of {} "
                      "({:.1f}% phantom at range_max {:.2f} m), "
                      "min {:.2f} max {:.2f}".format(
                          hits, total, 100.0 * phantoms / total,
                          scan.range_max, min(finite), max(finite)))
            else:
                print("scan: {} finite obstacle returns of {}".format(
                    hits, total))
            if hits < 5:
                errors.add("Laser is not observing field obstacles")
            if total and phantoms > 0.5 * total:
                errors.add("Laser is mostly (>{:.0f}%) returning range_max "
                           "phantoms - it is probably flying above or below "
                           "every wall and obstacle in gc_2026_field.world"
                           .format(100.0 * phantoms / total))
        models = latest.get("models")
        if models:
            for name in ("competition_floor", "fixed_obstacle_1",
                         "fixed_obstacle_2", "iris", "px4_sensor_rig"):
                if name not in models.name:
                    errors.add("Missing Gazebo model: " + name)
    for subscriber in subscribers:
        subscriber.unregister()
    for error in sorted(errors):
        print("FAIL: " + error)
    print("SITL_SLAM_CHECK_" + ("FAILED" if errors else "PASSED"))
    return bool(errors)


if __name__ == "__main__":
    raise SystemExit(main())
