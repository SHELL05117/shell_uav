#!/usr/bin/env bash
# Run on RK. Foreground process: Ctrl+C stops only this roslaunch and its children.
set -eo pipefail
mode="${1:-bridge}"
source /opt/ros/melodic/setup.bash
source /home/jcfh/catkin_ws/devel/setup.bash
set -u
if ss -H -ltn 'sport = :8765' | grep -q .; then
  echo '8765 already listening. Inspect existing bridge; no process was stopped.' >&2
  exit 2
fi
case "$mode" in
  ground)
    # Refuse overlap with any existing ROS system or occupied hardware.
    if pgrep -f '[r]osmaster|[r]oslaunch|[r]oscore|[m]avros_node|[u]sb_cam_node|[r]plidarNode|[c]artographer_node|[s]hoot_with_move_base|[s]ervo_controller_node' >/dev/null; then
      echo 'Existing ROS/hardware process found; ground preview refused.' >&2
      exit 2
    fi
    if ss -H -ltn 'sport = :11333' | grep -q .; then
      echo '11333 is occupied; ground preview refused.' >&2
      exit 2
    fi
    script_dir="$(cd -- "$(dirname -- "$0")" && pwd)"
    # This BSP has no fuser. Missing helper/privilege or unreadable FD state
    # must fail closed, rather than treating a failed occupancy check as idle.
    if ! sudo -n python2 "$script_dir/monitor_device_handles.py" \
        /dev/ttyACM0 /dev/ttyUSB0 \
        /dev/v4l/by-id/usb-RYS_USB_Camera_200901010001-video-index0; then
      echo 'Device missing/busy or FD inspection unavailable; ground preview refused.' >&2
      exit 2
    fi
    export ROS_MASTER_URI=http://127.0.0.1:11333
    export ROS_IP=127.0.0.1
    unset ROS_HOSTNAME ROS_NAMESPACE
    exec roslaunch --port 11333 location lichtblick_ground_preview.launch
    ;;
  bridge)
    # Attach to the selected running ROS master; never manufacture sensor data.
    if ! timeout 8 rosnode list >/dev/null 2>&1; then
      echo 'ROS master unavailable. Start the separately approved data sources first.' >&2
      exit 2
    fi
    exec roslaunch location lichtblick_bridge_readonly.launch jpeg_relay:=true
    ;;
  *) echo 'Usage: bash start_lichtblick_monitor.sh {ground|bridge}' >&2; exit 2 ;;
esac
