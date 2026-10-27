#!/usr/bin/env bash
# Source this from .bashrc; one ROS environment for launch and observation.
if [[ -r "$HOME/.config/uav/env.sh" ]]; then source "$HOME/.config/uav/env.sh"; fi
export UAV_WS="${UAV_WS:-$HOME/catkin_ws}"
export UAV_ROS_DISTRO="${UAV_ROS_DISTRO:-melodic}"
if [[ ! -r "/opt/ros/$UAV_ROS_DISTRO/setup.bash" || ! -r "$UAV_WS/devel/setup.bash" ]]; then
  echo 'UAV: ROS/workspace setup missing; check ~/.config/uav/env.sh' >&2
  return 2
fi
source "/opt/ros/$UAV_ROS_DISTRO/setup.bash"
source "$UAV_WS/devel/setup.bash"
export PYTHONPATH="/opt/ros/$UAV_ROS_DISTRO/lib/python2.7/dist-packages:$UAV_WS/devel/lib/python2.7/dist-packages:${PYTHONPATH:-}"
# Prefer the route to the current SSH peer; no saved wireless or USB address.
uav_peer="${SSH_CONNECTION%% *}"
uav_route="$(ip -4 route get "${uav_peer:-1.1.1.1}" 2>/dev/null)"
uav_ip="$(awk '{for(i=1;i<=NF;i++) if($i=="src") {print $(i+1); exit}}' <<< "$uav_route")"
export ROS_IP="${UAV_ROS_IP:-${uav_ip:-127.0.0.1}}"
export ROS_MASTER_URI="${UAV_MASTER_URI:-http://$ROS_IP:11311}"
unset ROS_HOSTNAME ROS_NAMESPACE
export PATH="$HOME/.local/bin:$PATH"
unset uav_peer uav_route uav_ip
