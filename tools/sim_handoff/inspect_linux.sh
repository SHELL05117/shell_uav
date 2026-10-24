#!/usr/bin/env bash
# Read-only inventory in an already running WSL distro or container.
set -u
date -Is
uname -m
if test -r /etc/os-release; then cat /etc/os-release; fi
if command -v lscpu >/dev/null; then lscpu; fi
if command -v free >/dev/null; then free -h; fi
df -h . /tmp
printf '\nROS installations\n'
if test -d /opt/ros; then find /opt/ros -maxdepth 2 -name setup.bash; fi
if command -v rosversion >/dev/null; then rosversion -d; fi
for cmd in gcc cmake python3 gazebo; do
    if command -v "$cmd" >/dev/null; then "$cmd" --version 2>&1 | head -n 2; fi
done
printf '\nActive display variables\n'
printf 'DISPLAY=%s\nWAYLAND_DISPLAY=%s\n' "${DISPLAY:-}" "${WAYLAND_DISPLAY:-}"
if command -v glxinfo >/dev/null; then
    if command -v timeout >/dev/null; then timeout 15 glxinfo -B; fi
fi
if command -v docker >/dev/null; then
    docker version
    docker info --format '{{.OSType}} {{.Architecture}} CPUs={{.NCPU}} Memory={{.MemTotal}}'
fi
