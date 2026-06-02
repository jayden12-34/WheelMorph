#!/usr/bin/env bash
set -e

# Source ROS if available
if [ -f "/opt/ros/humble/setup.bash" ]; then
  source /opt/ros/humble/setup.bash
fi

# Prepare Xauthority for GUI if DISPLAY is set
if [ -n "${DISPLAY:-}" ] && command -v xauth >/dev/null 2>&1; then
  mkdir -p /tmp 2>/dev/null || true
  touch /tmp/.docker-xauth 2>/dev/null || true
  chmod 666 /tmp/.docker-xauth 2>/dev/null || true
  
  if xauth nlist "${DISPLAY}" >/dev/null 2>&1; then
    xauth_list=$(xauth nlist "${DISPLAY}" 2>/dev/null || true)
    if [ -n "$xauth_list" ]; then
      echo "$xauth_list" | sed -e 's/^..../ffff/' | xauth -f /tmp/.docker-xauth nmerge - || true
    fi
  fi
fi

# Execute the command passed to the entrypoint
exec "$@"
