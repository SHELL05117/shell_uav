#!/usr/bin/env bash
# Verify the restored, disarmed production ground entry; only owns its own group.
set -eo pipefail
source "$HOME/.local/share/uav/uav_env.sh"
uav_verify_root="$HOME/logging-benchmark"
setsid "$HOME/.local/bin/uav" check --target flammable >"$uav_verify_root/restored-stack.log" 2>&1 &
uav_verify_pid=$!
uav_verify_stop() {
  # Bash background jobs can inherit SIGINT ignored. The launcher explicitly
  # handles SIGTERM and then closes only its own child process groups.
  if kill -0 "$uav_verify_pid" 2>/dev/null; then kill -TERM -- "-$uav_verify_pid"; fi
  wait "$uav_verify_pid" || true
}
trap uav_verify_stop EXIT
for uav_verify_attempt in {1..70}; do
  if rosnode list 2>/dev/null | grep -qx '/shoot_with_move_base_node'; then break; fi
  if ! kill -0 "$uav_verify_pid" 2>/dev/null; then echo 'Restored stack exited' >&2; exit 1; fi
  sleep 1
done
sleep 5
python2 "$UAV_WS/tools/rk_wireless_observe.py" "$uav_verify_root/restored-observation.json" --seconds 20 >"$uav_verify_root/restored-observation.stdout.log" 2>&1
"$HOME/.local/bin/uav" status >"$uav_verify_root/restored-status.txt" 2>&1
