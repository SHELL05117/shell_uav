#!/usr/bin/env bash
# Offline native build on the RK3399Pro Ubuntu 18.04 BSP. No kernel/ROS/NPU update.
# Inputs are unpacked pinned sources; see the wireless check evidence README.
set -eo pipefail
if [[ $# != 3 ]]; then echo 'Usage: build_rk_media_runtime.sh MPP_SOURCE RGA_SOURCE BUILD_DIRECTORY' >&2; exit 2; fi
mpp_source="$(realpath "$1")"
rga_source="$(realpath "$2")"
build_root="$(realpath -m "$3")"
test -f "$mpp_source/CMakeLists.txt"
test -f "$rga_source/CMakeLists.txt"
mkdir -p "$build_root/mpp" "$build_root/rga"
# MPP 1.0.0 requires DRM allocation on this BSP. VP9/H265/AV1 parser sources
# are absent in this upstream snapshot; they are not used by the MJPEG camera.
cd "$build_root/mpp"
cmake "$mpp_source" -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/shell-media \
  -DHAVE_DRM=ON -DCMAKE_C_FLAGS=-I/usr/include/libdrm \
  -DENABLE_H265D=OFF -DENABLE_VP9D=OFF -DENABLE_AV1D=OFF -DBUILD_TEST=ON
make -j2
sudo -n make install
# Toybrick source uses CMake >=3.12 only for object-library link syntax.
# On 3.10, the Linux COMMON_LIBS interface is empty; remove that object-only
# link and disable samples, then use the same upstream C++ implementation.
python3 - "$rga_source/CMakeLists.txt" <<'PY'
import sys
path=sys.argv[1]
with open(path) as f: text=f.read()
text=text.replace('cmake_minimum_required(VERSION 3.12)','cmake_minimum_required(VERSION 3.10)')
text=text.replace('target_link_libraries(${OBJECT_NAME} PRIVATE COMMON_LIBS)','')
text=text.replace('set(RGA_SAMPLES_ENABLE true)','set(RGA_SAMPLES_ENABLE false)')
with open(path,'w') as f: f.write(text)
PY
cd "$build_root/rga"
cmake "$rga_source" -DCMAKE_BUILD_TARGET=cmake_linux -DCMAKE_C_COMPILER=gcc \
  -DCMAKE_CXX_COMPILER=g++ -DCMAKE_INSTALL_PREFIX=/opt/shell-media
make -j2
sudo -n make install
