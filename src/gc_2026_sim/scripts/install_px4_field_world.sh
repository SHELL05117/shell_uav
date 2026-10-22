#!/usr/bin/env bash
set -euo pipefail

PX4_ROOT="${PX4_ROOT:-$HOME/PX4-Autopilot}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORLD_SOURCE="$(cd "${SCRIPT_DIR}/../gazebo/worlds" && pwd)/gc_2026_field.world"

if [[ ! -d "${PX4_ROOT}" ]]; then
  echo "PX4 root does not exist: ${PX4_ROOT}" >&2
  exit 1
fi

if [[ -d "${PX4_ROOT}/Tools/simulation/gazebo-classic/sitl_gazebo/worlds" ]]; then
  WORLD_DEST="${PX4_ROOT}/Tools/simulation/gazebo-classic/sitl_gazebo/worlds/gc_2026_field.world"
elif [[ -d "${PX4_ROOT}/Tools/sitl_gazebo/worlds" ]]; then
  WORLD_DEST="${PX4_ROOT}/Tools/sitl_gazebo/worlds/gc_2026_field.world"
else
  echo "Cannot find a PX4 Gazebo Classic worlds directory under ${PX4_ROOT}" >&2
  exit 1
fi

cp "${WORLD_SOURCE}" "${WORLD_DEST}"
echo "Installed competition world:"
echo "  ${WORLD_DEST}"

# The world references model://gc_task_markers, so the model has to land
# somewhere Gazebo searches.  PX4's own setup_gazebo.bash puts
# <sitl_gazebo>/models on GAZEBO_MODEL_PATH, so copy it next to the world.
# Without this, gazebo offers the URI to the online model database and world
# loading stalls at "Preparing your world" with no error printed.
MODEL_SOURCE="$(cd "${SCRIPT_DIR}/../gazebo/models" && pwd)"
MODEL_DEST="$(dirname "${WORLD_DEST}")/../models"
mkdir -p "${MODEL_DEST}"
echo
echo "Installed model resources:"
for model in "${MODEL_SOURCE}"/*/; do
  model="${model%/}"
  [[ -d "${model}" ]] || continue
  name="$(basename "${model}")"
  rm -rf "${MODEL_DEST:?}/${name:?}"
  cp -r "${model}" "${MODEL_DEST}/"
  echo "  ${MODEL_DEST}/${name}"
done

echo
echo "Start the ROS1 PX4 + Cartographer simulation with:"
echo "  bash \"${SCRIPT_DIR}/start_px4_slam.sh\""
echo "Do not use the legacy make gazebo spawn: it is outside this field."
echo "Do not Reset Time while PX4 is connected; restart the launch instead."
