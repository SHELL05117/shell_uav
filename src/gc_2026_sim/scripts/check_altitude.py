#!/usr/bin/env python3
"""Audit where the lidar actually flies, in one table, while the sim runs.

Run this while the vehicle is hovering at the mission altitude (fly_height,
default 1.5 m). It answers one question with numbers instead of reasoning:

    the mission commands 1.5 m, so why does the lidar end up at ~2.1 m?

It prints every altitude in the chain side by side, because the whole failure
mode on 2026-09-24/25 was that only ONE of them was ever visible in the log
("Rig check: iris=(0.17, 0.08, 1.92)") and the rest had to be guessed:

    iris_z          Gazebo world z of the 'iris' model origin
    rig_z           Gazebo world z of the 'px4_sensor_rig' model origin
    lidar_z         rig_z + lidar mount height (param, default 0.25, read from
                    px4_sensor_rig/model.sdf <sensor><pose>0 0 0.25</pose>)
    mavros_z        /mavros/local_position/pose -> z, i.e. what PX4 believes
    gt_odom_z       /simulation/ground_truth/odom -> z, what the rig publishes
    tf_odom_laser_z TF odom -> laser_link z, what Cartographer actually consumes

The decisive number is

    lidar_z - mavros_z

A faithful rig has the lidar mount height (0.25 m) as this difference: the
mission flies 1.5 m, the sensor sits 1.75 m. If this comes out near 0.67 m,
then 0.42 m of the sensor height is a simulation artifact, not mission geometry,
and every wall in gc_2026_field.world has to be 0.42 m taller than the real
field to compensate - which is exactly the wrong way round, because it makes
the simulator MORE forgiving than the real venue.

The second decisive number is

    gt_odom_z - mavros_z

px4_sensor_rig_node::mavrosPoseCallback captures odom_offset_* ONCE, from the
first /mavros/local_position/pose and whatever Gazebo pose happened to be
cached at that instant, with the comment "the vehicle is stationary before
arming". If the iris is still settling from its spawn drop when that fires, the
z offset is captured mid-fall and the published odom disagrees with PX4 in z by
a constant for the rest of the run - which is the exact thing that function
exists to prevent. A few cm is fine; 0.30 m is not.
"""

import argparse
import math
import sys
import time

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import tf2_ros


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=10.0,
                        help="seconds to sample (default 10)")
    parser.add_argument("--rate", type=float, default=2.0,
                        help="samples per second (default 2)")
    parser.add_argument("--vehicle-model", default="iris")
    parser.add_argument("--rig-model", default="px4_sensor_rig")
    parser.add_argument("--lidar-mount", type=float, default=0.25,
                        help="lidar z in px4_sensor_rig/model.sdf (default 0.25)")
    parser.add_argument("--fly-height", type=float, default=1.5,
                        help="navigation_2d fly_height arg (default 1.5)")
    args = parser.parse_args()

    rospy.init_node("check_altitude", anonymous=True)

    latest = {}

    def keep_model_states(msg):
        latest["models"] = msg

    def keep_mavros(msg):
        latest["mavros"] = msg

    def keep_gt(msg):
        latest["gt"] = msg

    rospy.Subscriber("/gazebo/model_states", ModelStates, keep_model_states,
                     queue_size=5)
    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, keep_mavros,
                     queue_size=5)
    rospy.Subscriber("/simulation/ground_truth/odom", Odometry, keep_gt,
                     queue_size=5)

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    samples = []
    deadline = time.monotonic() + max(1.0, args.duration)
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        time.sleep(1.0 / max(0.1, args.rate))
        row = {}

        models = latest.get("models")
        if models is not None:
            for model, key in ((args.vehicle_model, "iris_z"),
                               (args.rig_model, "rig_z")):
                if model in models.name:
                    row[key] = models.pose[models.name.index(model)].position.z

        mavros = latest.get("mavros")
        if mavros is not None:
            row["mavros_z"] = mavros.pose.position.z

        gt = latest.get("gt")
        if gt is not None:
            row["gt_odom_z"] = gt.pose.pose.position.z

        try:
            transform = tf_buffer.lookup_transform("odom", "laser_link",
                                                   rospy.Time(0),
                                                   rospy.Duration(0.1))
            row["tf_odom_laser_z"] = transform.transform.translation.z
        except Exception:
            pass

        if row:
            samples.append(row)

    if not samples:
        print("FAIL: no samples. Is the simulation running?")
        print("      needed: /gazebo/model_states, /mavros/local_position/pose")
        return 1

    fields = ["iris_z", "rig_z", "mavros_z", "gt_odom_z", "tf_odom_laser_z",
              "lidar_z", "lidar_z-mavros_z", "gt_odom_z-mavros_z"]

    def collect(name):
        values = [s[name] for s in samples if name in s]
        return values

    def stat(values):
        if not values:
            return None
        return (sum(values) / len(values), min(values), max(values))

    def derive(sample):
        out = dict(sample)
        if "rig_z" in out:
            out["lidar_z"] = out["rig_z"] + args.lidar_mount
        if "lidar_z" in out and "mavros_z" in out:
            out["lidar_z-mavros_z"] = out["lidar_z"] - out["mavros_z"]
        if "gt_odom_z" in out and "mavros_z" in out:
            out["gt_odom_z-mavros_z"] = out["gt_odom_z"] - out["mavros_z"]
        return out

    derived = [derive(s) for s in samples]

    print("")
    print("altitude audit, {} samples over {:.1f} s".format(
        len(derived), args.duration))
    print("lidar mount assumed {:.2f} m; fly_height assumed {:.2f} m".format(
        args.lidar_mount, args.fly_height))
    print("")
    print("{:<22} {:>9} {:>9} {:>9}   {}".format(
        "quantity", "mean", "min", "max", "meaning"))
    meanings = {
        "iris_z": "Gazebo world z, iris model origin",
        "rig_z": "Gazebo world z, sensor rig origin",
        "lidar_z": "rig_z + mount = where beams start",
        "mavros_z": "PX4's own altitude estimate",
        "gt_odom_z": "rig-published odom z",
        "tf_odom_laser_z": "what Cartographer consumes",
        "lidar_z-mavros_z": "SENSOR ABOVE COMMAND (want ~mount)",
        "gt_odom_z-mavros_z": "odom vs PX4 (want ~0)",
    }
    for name in fields:
        row = stat(collect(name))
        if row is None:
            print("{:<22} {:>9} {:>9} {:>9}   {}".format(
                name, "-", "-", "-", meanings[name]))
            continue
        print("{:<22} {:>9.3f} {:>9.3f} {:>9.3f}   {}".format(
            name, row[0], row[1], row[2], meanings[name]))

    errors = []

    above = stat(collect("lidar_z-mavros_z"))
    if above is None:
        errors.append("could not measure lidar_z - mavros_z")
    else:
        excess = above[0] - args.lidar_mount
        print("")
        if abs(excess) <= 0.05:
            print("OK   sensor flies {:.3f} m above the commanded altitude, "
                  "which is the mount height. Sim is faithful.".format(above[0]))
        else:
            print("FAIL sensor flies {:.3f} m above the commanded altitude, "
                  "but the mount is only {:.2f} m. Extra {:.3f} m of sensor "
                  "height is a simulation artifact.".format(
                      above[0], args.lidar_mount, excess))
            print("     With the real field's 2.00 m wall/obstacle tops and a "
                  "1.50 m flight altitude the real lidar sits at {:.2f} m, so "
                  "this sim is seeing geometry the real venue will not "
                  "present. Fix the rig, do not raise the world.".format(
                      args.fly_height + args.lidar_mount))
            errors.append("lidar flies {:.3f} m too high".format(excess))

    odom_bias = stat(collect("gt_odom_z-mavros_z"))
    if odom_bias is not None and abs(odom_bias[0]) > 0.05:
        print("FAIL published odom disagrees with PX4 in z by {:.3f} m. "
              "auto_odom_origin most likely captured odom_offset_z before the "
              "iris settled; see mavrosPoseCallback.".format(odom_bias[0]))
        errors.append("odom z disagrees with PX4 by {:.3f} m".format(
            odom_bias[0]))

    print("")
    for error in errors:
        print("FAIL: " + error)
    print("ALTITUDE_CHECK_" + ("FAILED" if errors else "PASSED"))
    return bool(errors)


if __name__ == "__main__":
    sys.exit(main())
