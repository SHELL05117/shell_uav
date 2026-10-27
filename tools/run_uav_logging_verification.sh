#!/usr/bin/env bash
# Post-benchmark, ground-only regressions. Every ROS probe owns a private master.
set -o pipefail
source "$HOME/.local/share/uav/uav_env.sh"
uav_verify_root="$HOME/logging-benchmark"
while [[ ! -f "$uav_verify_root/core-2/result.json" ]]; do sleep 1; done
if pgrep -f '^python2 .*/uav_start.py (a|check|mission)' >/dev/null; then
  echo 'Ground stack still running; refusing post-benchmark tests' >&2
  exit 1
fi
python3 "$UAV_WS/tools/test_uav_logging.py" >"$uav_verify_root/unit.log" 2>&1
python2 "$UAV_WS/tools/uav_logging_fault_probe.py" "$uav_verify_root/fault-probe" >"$uav_verify_root/fault-probe.log" 2>&1
for uav_session in "$HOME"/uav_flight_logs/20261028-*; do
  if [[ -f "$uav_session/closed.json" ]]; then
    nice -n 15 python2 "$UAV_WS/tools/uav_log_report.py" "$uav_session" >"$uav_session/report.stdout.log" 2>&1
  fi
done
cd "$UAV_WS"
catkin_make run_tests_location run_tests_navigation_2d -j2 -l3 >"$uav_verify_root/catkin-tests.log" 2>&1
catkin_test_results build/test_results >"$uav_verify_root/catkin-test-results.txt" 2>&1
python2 tools/dwa_mission_probe.py "$uav_verify_root/mission-regression" --image "$uav_verify_root/images/IMG_2766.jpg" --person-image "$uav_verify_root/images/IMG_2706.jpg" >"$uav_verify_root/mission-regression.log" 2>&1
python2 tools/rk_px4_serial_shell.py --mode logging >"$uav_verify_root/px4-logging-readonly.txt" 2>&1
date -Is >"$uav_verify_root/verification-finished.txt"
