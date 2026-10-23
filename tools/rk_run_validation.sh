#!/usr/bin/env bash
# Run synthetic tests on three fresh local ROS masters. No hardware is used.
# Usage: bash rk_run_validation.sh WORKSPACE LOG_DIRECTORY [all|messages|mavros|application]
set -eo pipefail
workspace=${1:?catkin workspace required}
log_directory=${2:?test log directory required}
selected_case=${3:-all}
case "$selected_case" in
    all|messages|mavros|application) ;;
    *) printf 'Unknown validation case: %s\n' "$selected_case" >&2; exit 2 ;;
esac
tool_directory=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
source "$workspace/devel/setup.bash"
set -u
export ROS_IP=127.0.0.1
unset ROS_HOSTNAME ROS_NAMESPACE
export PYTHONDONTWRITEBYTECODE=1
mkdir -p "$log_directory"
export ROS_HOME="$log_directory/ros-home"
export ROS_LOG_DIR="$log_directory/ros-logs"
mkdir -p "$ROS_HOME" "$ROS_LOG_DIR"
master_pid=
cleanup() {
    if [[ -n "$master_pid" ]]; then
        kill -INT -- "-$master_pid" 2>/dev/null || true
        for unused in {1..50}; do
            if ! kill -0 "$master_pid" 2>/dev/null; then break; fi
            sleep 0.1
        done
        if kill -0 "$master_pid" 2>/dev/null; then
            kill -TERM -- "-$master_pid" 2>/dev/null || true
        fi
        wait "$master_pid" 2>/dev/null || true
        master_pid=
    fi
}
trap cleanup EXIT
trap 'exit 130' INT TERM
run_case() {
    local label=$1 port=$2 script=$3
    shift 3
    export ROS_MASTER_URI="http://127.0.0.1:$port"
    python2 - "$port" <<'PY'
import socket, sys
sock = socket.socket()
sock.bind(('127.0.0.1', int(sys.argv[1])))
sock.close()
PY
    setsid roscore -p "$port" >"$log_directory/$label.roscore.log" 2>&1 &
    master_pid=$!
    python2 - "$port" <<'PY'
import socket, sys, time
deadline = time.time() + 15
while True:
    sock = socket.socket()
    sock.settimeout(0.3)
    try:
        sock.connect(('127.0.0.1', int(sys.argv[1])))
        break
    except socket.error:
        if time.time() > deadline:
            raise
        time.sleep(0.1)
    finally:
        sock.close()
PY
    if python2 "$tool_directory/$script" "$@" >"$log_directory/$label.json" 2>"$log_directory/$label.stderr.log"; then
        cat "$log_directory/$label.json"
    else
        local result=$?
        cat "$log_directory/$label.json"
        cat "$log_directory/$label.stderr.log" >&2
        return "$result"
    fi
    cleanup
}
if [[ "$selected_case" == all || "$selected_case" == messages ]]; then
    run_case messages 11321 rk_ros_loopback_test.py --with-project-messages
fi
if [[ "$selected_case" == all || "$selected_case" == mavros ]]; then
    run_case mavros 11322 rk_mavros_loopback_test.py --log-dir "$log_directory/mavros" --timeout 25
fi
if [[ "$selected_case" == all || "$selected_case" == application ]]; then
    run_case application 11323 rk_application_loopback_test.py --workspace "$workspace" --log-dir "$log_directory/application" --timeout 12
fi
printf 'Selected synthetic loopback cases passed: %s\n' "$selected_case"
