#!/usr/bin/env bash
set -eo pipefail

PX4_ROOT="${PX4_ROOT:-$HOME/PX4-Autopilot}"
ROS_DISTRO="${ROS_DISTRO:-noetic}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd "${SCRIPT_DIR}/../../.." && pwd)"

source "/opt/ros/${ROS_DISTRO}/setup.bash"
for setup in \
    "${CARTOGRAPHER_SETUP:-$HOME/cartographer_ws/install_isolated/setup.bash}" \
    "${FOXGLOVE_SETUP:-$HOME/foxglove_bridge_ws/devel/setup.bash}"; do
  if [[ -f "${setup}" ]]; then
    source "${setup}" --extend
  fi
done
if [[ -f "${WORKSPACE}/devel/setup.bash" ]]; then
  source "${WORKSPACE}/devel/setup.bash" --extend
fi
if [[ ! -x "${PX4_ROOT}/build/px4_sitl_default/bin/px4" ]]; then
  echo "Build PX4 first: cd \"${PX4_ROOT}\" && make px4_sitl_default" >&2
  exit 1
fi
# This legacy PX4 checkout needs both its ROS package and Gazebo model paths.
source "${PX4_ROOT}/Tools/setup_gazebo.bash" \
  "${PX4_ROOT}" "${PX4_ROOT}/build/px4_sitl_default"
export ROS_PACKAGE_PATH="${WORKSPACE}/src:${PX4_ROOT}:${PX4_ROOT}/Tools/sitl_gazebo:${ROS_PACKAGE_PATH}"
export GAZEBO_MODEL_DATABASE_URI=""
for package in gazebo_ros mavros cartographer_ros foxglove_bridge; do
  if ! rospack find "${package}" >/dev/null; then
    echo "Missing ROS package: ${package}" >&2
    exit 1
  fi
done
# Do not kill an existing simulation, or attach a second PX4 to its ports.
if pgrep -x px4 >/dev/null || pgrep -x gzserver >/dev/null; then
  echo "Close the previous PX4/Gazebo run with Ctrl+C before starting this launch." >&2
  exit 1
fi

# 端口闸：11311 被占时 roslaunch 先报 "Unable to start XML-RPC server"，
# 再抛 "RLException: could not contact master" —— 读起来像编译/环境坏了，其实
# 只是有东西占着端口。这里先分开这两种情形，把真正的原因和 pid 打出来。
MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"
MASTER_PORT="${MASTER_URI##*:}"
MASTER_PORT="${MASTER_PORT%%/*}"
if [[ ! "${MASTER_PORT}" =~ ^[0-9]+$ ]]; then
  # 端口解析不出来就别往下走，否则后面 $(( )) 会在 set -e 下抛算术错误。
  echo "Cannot parse a port out of ROS_MASTER_URI=\"${MASTER_URI}\"." >&2
  exit 1
fi

master_port_holders() {
  # 只为了在报错里指出"谁占的"；拿不到也不影响下面的判定。
  if command -v ss >/dev/null 2>&1; then
    ss -ltnp 2>/dev/null | grep -F ":${MASTER_PORT} " | grep -o 'users:.*' | head -n 1 || true
  elif command -v lsof >/dev/null 2>&1; then
    lsof -nP -iTCP:"${MASTER_PORT}" -sTCP:LISTEN 2>/dev/null \
      | awk 'NR==2 {print "users:((" $1 ",pid=" $2 "))"}' || true
  fi
}

port_busy() {
  # 精简镜像里可能没有 ss/lsof，最后一档用 bash 自带 TCP 探针兜底：
  # 能连上就说明有监听者。
  if command -v ss >/dev/null 2>&1; then
    ss -ltn 2>/dev/null | grep -qF ":${MASTER_PORT} " && return 0
    return 1
  fi
  if command -v lsof >/dev/null 2>&1; then
    lsof -nP -iTCP:"${MASTER_PORT}" -sTCP:LISTEN >/dev/null 2>&1 && return 0
    return 1
  fi
  (exec 3<>"/dev/tcp/127.0.0.1/${MASTER_PORT}") >/dev/null 2>&1
}

if port_busy; then
  holders="$(master_port_holders)"
  if timeout 3 rosparam list >/dev/null 2>&1; then
    echo "A ROS master is already running on ${MASTER_URI}." >&2
    echo "  ${holders:-holder unknown}" >&2
    echo "If a run is still alive, Ctrl+C it in its own terminal. If it is not, this is a" >&2
    echo "leftover master from a killed run (it has no terminal left to stop it):" >&2
    echo "  pkill -x rosout; pkill -x rosmaster" >&2
    echo "Or start the whole tree on another port:" >&2
    echo "  ROS_MASTER_URI=http://localhost:$((MASTER_PORT + 1)) bash $0 $*" >&2
    exit 1
  fi
  echo "Port ${MASTER_PORT} is held by a process that does not answer as a ROS master." >&2
  echo "  ${holders:-holder unknown (install iproute2 or lsof to see it)}" >&2
  echo "roslaunch reports this as \"could not contact master\", which reads like a" >&2
  echo "build problem. Clear the port and start again:" >&2
  echo "  ss -ltnp | grep ':${MASTER_PORT} '" >&2
  echo "  pkill -x rosout; pkill -x rosmaster; pkill -f roslaunch" >&2
  exit 1
fi

exec roslaunch gc_2026_sim px4_gazebo_slam.launch px4_root:="${PX4_ROOT}" "$@"
