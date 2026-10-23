#!/usr/bin/env bash
# Read-only Ubuntu 18.04 / Bash 4 audit. Run as the normal SSH account.
# No package changes, downloads, device opens/writes, or ROS nodes are started.
# Missing commands/packages are reported without stopping later sections.
set -o pipefail
exec 2>&1
export LC_ALL=C
export PYTHONDONTWRITEBYTECODE=1

section() {
    printf '\n===== %s =====\n' "$1"
}

run() {
    printf '\n$'
    printf ' %q' "$@"
    printf '\n'
    if command -v "$1" >/dev/null 2>&1; then
        "$@" 2>&1
        local result=$?
        printf '[exit=%s]\n' "$result"
    else
        printf '[unavailable: %s]\n' "$1"
    fi
    return 0
}

section 'Audit scope and identity'
printf 'Read-only dependency inventory; no hardware protocol test.\n'
run date --iso-8601=seconds
run date -u --iso-8601=seconds
run id
run uname -a
run dpkg --print-architecture
if [ -r /etc/os-release ]; then
    run cat /etc/os-release
fi
if [ -r /proc/device-tree/model ]; then
    printf '\nDevice-tree model: '
    tr -d '\000' </proc/device-tree/model
    printf '\n'
fi

section 'Clock, storage, memory, and swap'
# Ubuntu 18.04's systemd 237 does not support timedatectl show -p.
run timedatectl status
run uptime
run df -hT / /boot "$HOME"
run free -h
run swapon --show

section 'dpkg audit (empty output normally means no incomplete packages)'
run dpkg --audit

section 'Declared deployment dependencies: status, version, architecture'
packages=(
    build-essential cmake pkg-config git rsync
    libeigen3-dev libopencv-dev libi2c-dev
    i2c-tools v4l-utils usbutils openssh-server python-rosdep
    geographiclib-tools python-numpy python3-numpy
    ros-melodic-ros-base ros-melodic-catkin
    ros-melodic-mavros ros-melodic-mavros-extras ros-melodic-mavros-msgs
    ros-melodic-mavlink ros-melodic-cmake-modules
    ros-melodic-message-generation ros-melodic-message-runtime
    ros-melodic-tf ros-melodic-tf2 ros-melodic-tf2-ros
    ros-melodic-tf2-geometry-msgs ros-melodic-cv-bridge
    ros-melodic-cartographer-ros ros-melodic-gazebo-msgs
    ros-melodic-usb-cam ros-melodic-foxglove-bridge
    ros-melodic-roscpp ros-melodic-rospy ros-melodic-rosconsole
    ros-melodic-std-msgs ros-melodic-std-srvs
    ros-melodic-geometry-msgs ros-melodic-nav-msgs
    ros-melodic-sensor-msgs ros-melodic-visualization-msgs
    ros-melodic-actionlib-msgs
)
printf 'PACKAGE\tSTATUS\tVERSION\tARCHITECTURE\n'
if command -v dpkg-query >/dev/null 2>&1; then
    for package in "${packages[@]}"; do
        if ! dpkg-query -W -f='${binary:Package}\t${Status}\t${Version}\t${Architecture}\n' "$package" 2>/dev/null; then
            printf '%s\tNOT_REGISTERED\t-\t-\n' "$package"
        fi
    done
else
    printf '[unavailable: dpkg-query]\n'
fi

section 'ROS Melodic environment and package resolution'
if [ -r /opt/ros/melodic/setup.bash ]; then
    # This changes only this script process; do not source workspace overlays.
    source /opt/ros/melodic/setup.bash
    printf '[Melodic setup source exit=%s]\n' "$?"
else
    printf '[missing: /opt/ros/melodic/setup.bash]\n'
fi
# Keep ROS tooling from creating ~/.ros caches/log directories, including any
# rospack subprocess spawned by rosversion. This is a process-only override.
export ROS_HOME=/dev/null
run rosversion -d
run rosversion roslaunch
run rosversion catkin
run rosversion mavros
run rosversion cv_bridge
ros_packages=(
    roscpp rospy rosconsole roslaunch catkin cmake_modules
    std_msgs std_srvs geometry_msgs nav_msgs sensor_msgs visualization_msgs
    actionlib_msgs tf tf2 tf2_ros tf2_geometry_msgs
    message_generation message_runtime mavros mavros_msgs mavlink
    cv_bridge cartographer_ros gazebo_msgs usb_cam foxglove_bridge
)
# rospack normally maintains ~/.ros/rospack_cache. /dev/null cannot be a
# directory, so this inventory cannot create a cache or overwrite real files.
# Package resolution still uses the environment loaded from Melodic above.
if command -v rospack >/dev/null 2>&1; then
    for package in "${ros_packages[@]}"; do
        printf '\nrospack find %s\n' "$package"
        ROS_HOME=/dev/null rospack find "$package" 2>&1
        printf '[exit=%s]\n' "$?"
    done
else
    printf '[unavailable: rospack]\n'
fi

section 'Python 2 and Python 3 imports after Melodic setup'
for interpreter in python2 python3; do
    if ! command -v "$interpreter" >/dev/null 2>&1; then
        printf '\n[unavailable: %s]\n' "$interpreter"
        continue
    fi
    run "$interpreter" --version
    "$interpreter" - <<'PYTHON'
from __future__ import print_function
import importlib
import sys
import traceback

print('executable={}'.format(sys.executable))
for name in ('rospy', 'cv_bridge', 'numpy'):
    try:
        module = importlib.import_module(name)
        print('{}: OK file={} version={}'.format(
            name, getattr(module, '__file__', '?'),
            getattr(module, '__version__', '(not exposed)')))
        if name == 'cv_bridge':
            # Importing the top-level package alone can miss the compiled
            # Python/OpenCV bridge ABI; this imports it without a camera.
            importlib.import_module('cv_bridge.boost.cv_bridge_boost')
            print('cv_bridge.boost.cv_bridge_boost: OK')
    except Exception:
        print('{}: IMPORT ERROR'.format(name))
        traceback.print_exc(file=sys.stdout)
PYTHON
    printf '[%s import-check exit=%s]\n' "$interpreter" "$?"
done

section 'GeographicLib dataset files (metadata only)'
for directory in /usr/share/GeographicLib /usr/local/share/GeographicLib; do
    if [ -d "$directory" ]; then
        run find "$directory" -maxdepth 2 -type f -printf '%M %u %g %s %p\n'
    else
        printf '[missing directory: %s]\n' "$directory"
    fi
done
for geoid in /usr/share/GeographicLib/geoids/egm96-5.pgm /usr/local/share/GeographicLib/geoids/egm96-5.pgm; do
    if [ -s "$geoid" ]; then
        printf '[nonempty geoid: %s]\n' "$geoid"
    else
        printf '[missing or empty geoid: %s]\n' "$geoid"
    fi
done

section 'IP addresses and routes (no network authentication files)'
run ip -br address
run ip route show
run ip -6 route show

section 'USB enumeration and SSH / USB-network service states'
run lsusb
if [ -d /sys/class/udc ]; then
    run ls -l /sys/class/udc
fi
for service in ssh.service rk-usb-ssh.service; do
    run systemctl is-enabled "$service"
    run systemctl is-active "$service"
done

section 'Serial, I2C and camera node permissions (metadata only)'
shopt -s nullglob
if [ -d /dev/serial/by-id ]; then
    run ls -ld /dev/serial/by-id
    serial_links=(/dev/serial/by-id/*)
    if [ "${#serial_links[@]}" -gt 0 ]; then
        run ls -ld "${serial_links[@]}"
        run stat -L -c '%A %a %U %G major_minor=%t:%T %n' "${serial_links[@]}"
    else
        printf '[no /dev/serial/by-id entries]\n'
    fi
else
    printf '[missing: /dev/serial/by-id]\n'
fi
for family in ttyACM ttyUSB i2c- video; do
    device_nodes=(/dev/"$family"*)
    if [ "${#device_nodes[@]}" -gt 0 ]; then
        run stat -c '%A %a %U %G major_minor=%t:%T %n' "${device_nodes[@]}"
    else
        printf '[no /dev/%s* nodes]\n' "$family"
    fi
done
run i2cdetect -l
printf 'Only adapter listing was requested; no I2C bus was scanned.\n'

section 'Selected running processes (PID and comm only; no arguments)'
if command -v ps >/dev/null 2>&1; then
    ps -eo pid=,comm= | awk '
        $2 ~ /^(roscore|rosmaster|rosout|roslaunch|mavros_node|cartographer.*|rplidar.*|usb_cam.*|foxglove_bridge|location.*|navigation_2d|ssd_detection.*|ego_planner.*|sshd|rk_usb_ssh\.sh|i2cdetect|apt|apt-get|dpkg|catkin_make|cmake|make|cc1plus)$/ { print }
    '
else
    printf '[unavailable: ps]\n'
fi

section 'Existing home catkin workspace (top level only)'
workspace="$HOME/catkin_ws"
if [ -d "$workspace" ]; then
    run find "$workspace" -mindepth 1 -maxdepth 1 -printf '%M %u %g %f\n'
    for item in src build devel logs .catkin_tools; do
        if [ -e "$workspace/$item" ] || [ -L "$workspace/$item" ]; then
            run ls -ld "$workspace/$item"
        else
            printf '[absent: %s/%s]\n' "$workspace" "$item"
        fi
    done
else
    printf '[missing directory: %s]\n' "$workspace"
fi

section 'Audit complete'
printf 'Inventory complete. Individual failures above are findings, not a system acceptance result.\n'
