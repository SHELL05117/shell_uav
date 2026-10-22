#!/usr/bin/env bash
# Verify the Cartographer / TF wiring for the PX4 SITL mission.
#
# Usage: bash src/gc_2026_sim/scripts/check_slam_topics.sh
#
# The PX4 SITL launch (px4_gazebo_slam.launch -> px4_sitl_field.launch ->
# simulation_slam.launch) has these invariants:
#   * px4_sensor_rig publishes odom -> base_link from the Gazebo ground truth,
#     and is the ONLY publisher of that edge
#   * cartographer owns map -> odom only (published_frame=odom,
#     provide_odom_frame=false)
#   * base_to_laser (static_transform_publisher) owns base_link -> laser_link
#   * use_odometry = false, so cartographer must NOT subscribe to odom
#   * cartographer is started AFTER the rig, with a node_start_delay margin,
#     so its first TF lookup of odom -> base_link can succeed
#
# Pass criteria:
#   1. cartographer is up and subscribes to /scan
#   2. cartographer does NOT subscribe to an odom topic
#   3. exactly one TF tree: map -> odom -> base_link all resolvable
#   4. base_link -> laser_link exists (the scan frame)
#   5. no "tf_bridge" extrapolation warnings in the cartographer log

set -u

GRN='\033[0;32m'; RED='\033[0;31m'; YEL='\033[0;33m'; NC='\033[0m'
ok()   { echo -e "${GRN}[ OK ]${NC} $*"; }
bad()  { echo -e "${RED}[FAIL]${NC} $*"; }
warn() { echo -e "${YEL}[WARN]${NC} $*"; }

echo "===== 1. cartographer_node subscriptions ====="
SUBS="$(rosnode info /cartographer_node 2>/dev/null | sed -n '/Subscriptions/,/Services/p')"
echo "${SUBS}"
if echo "${SUBS}" | grep -q "/scan"; then
  ok "subscribes to /scan"
else
  bad "no /scan subscription -- check the scan remap"
fi
if echo "${SUBS}" | grep -q "/mavros/local_position/odom"; then
  bad "subscribes to odom, but cartographer_sim.lua sets use_odometry=false."
  echo "     This mismatch crashes OptimizationProblem2D on duplicate stamps."
  echo "     Remove the <remap from=\"odom\" .../> in simulation_slam.launch."
else
  ok "does NOT subscribe to odom (matches use_odometry=false)"
fi

echo
echo "===== 2. single connected TF tree ====="
for pair in "map odom" "odom base_link" "base_link laser_link"; do
  set -- ${pair}
  if timeout 3 rosrun tf tf_echo "$1" "$2" >/dev/null 2>&1; then
    ok "$1 -> $2 resolvable"
  else
    bad "$1 -> $2 MISSING (two or more unconnected TF trees)"
  fi
done

echo
echo "===== 3. odom topic (informational) ====="
if timeout 3 rostopic hz /mavros/local_position/odom >/dev/null 2>&1; then
  warn "/mavros/local_position/odom is publishing but intentionally unused by SLAM"
else
  warn "/mavros/local_position/odom has no data (mavros not up?)"
fi

echo
echo "Done. cartographer must print 'scan rate: 10.00 Hz' and no 'tf_bridge'"
echo "warnings. After editing lua/launch files restart the whole launch:"
echo "  bash src/gc_2026_sim/scripts/start_px4_slam.sh gui:=true auto_start:=true"
