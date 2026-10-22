#!/bin/sh
set -e

# Only used by the local PX4 process in px4_gazebo_slam.launch.
. etc/init.d-posix/rcS

if [ "${GC_SITL_NO_RC:-0}" = "1" ]; then
    # Preserve other RC exceptions. Offboard-stream loss protection stays enabled.
    rc_exceptions=$(param show -q COM_RCL_EXCEPT)
    param set COM_RCL_EXCEPT "$((rc_exceptions | 4))"
    echo "Competition SITL: enabled RC-loss exception for OFFBOARD (no simulated RC)."
fi
