#!/usr/bin/env bash
# Run on the Nano, after an existing ROS Master is available.
# Only the display bridge is launched. No camera, localization, mission, or servo.
set -e

usage() {
  printf 'Usage: bash start_monitor_bridge.sh [preview|mission|ssd]\n' >&2
}

if (( $# > 1 )); then
  usage
  exit 2
fi
jpeg_source="${1:-preview}"
case "$jpeg_source" in
  preview|mission|ssd) ;;
  *) usage; exit 2 ;;
esac

monitor_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
ros_setup="${MONITOR_ROS_SETUP:-/opt/ros/melodic/setup.bash}"
workspace_setup="${MONITOR_WORKSPACE_SETUP:-/home/jcfh/catkin_ws/devel/setup.bash}"
if [[ ! -r "$ros_setup" || ! -r "$workspace_setup" ]]; then
  printf 'Missing readable ROS/workspace setup: %s or %s\n' "$ros_setup" "$workspace_setup" >&2
  exit 3
fi
# Do not use nounset while sourcing generated ROS environment hooks.
source "$ros_setup"
source "$workspace_setup"
set -u

for tool in rospack rosversion rosnode roslaunch ss; do
  if ! command -v "$tool" >/dev/null 2>&1; then
    printf 'Required existing tool is unavailable: %s\n' "$tool" >&2
    exit 3
  fi
done
bridge_version="$(rosversion foxglove_bridge)"
if [[ "$bridge_version" != '0.6.3' ]]; then
  printf 'This configuration was checked for bridge 0.6.3; installed version is %s. Review parameters first.\n' "$bridge_version" >&2
  exit 3
fi
bridge_package="$(rospack find foxglove_bridge)"
if [[ ! -r "$bridge_package/launch/foxglove_bridge.launch" ]]; then
  printf 'Existing bridge launch is missing: %s\n' "$bridge_package/launch/foxglove_bridge.launch" >&2
  exit 3
fi
if ! node_names="$(rosnode list 2>/dev/null)"; then
  printf 'No reachable existing ROS Master at %s. This script does not start data sources or a new master.\n' "${ROS_MASTER_URI:-http://localhost:11311}" >&2
  exit 4
fi
while IFS= read -r node_name; do
  case "$node_name" in
    /foxglove_bridge|/foxglove_nodelet_manager)
      printf 'Bridge/nodelet manager already exists: %s. Do not run two display bridges; inspect the existing session.\n' "$node_name" >&2
      exit 4
      ;;
  esac
done <<< "$node_names"
if ! listeners="$(ss -H -ltn 'sport = :8765')"; then
  printf 'Unable to inspect TCP listeners; bridge startup was not attempted.\n' >&2
  exit 4
fi
if [[ -n "$listeners" ]]; then
  printf 'TCP port 8765 is already listening. Inspect the existing process; no process was stopped.\n' >&2
  exit 4
fi

printf 'Starting display-only bridge on 0.0.0.0:8765; JPEG selection: %s\n' "$jpeg_source"
printf 'ROS Master: %s\n' "${ROS_MASTER_URI:-http://localhost:11311}"
exec roslaunch "$monitor_dir/bridge_readonly.launch" address:=0.0.0.0 port:=8765 "jpeg_source:=$jpeg_source"
