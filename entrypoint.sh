#!/usr/bin/env bash
set -e

# Source ROS if available
if [ -f "/opt/ros/humble/setup.bash" ]; then
  source /opt/ros/humble/setup.bash
fi

# Prepare Xauthority for GUI if DISPLAY is set.
# /tmp/.host-xauth is the host's real Xauthority file, mounted read-only.
# We rewrite its entries with FamilyWild (ffff) so they work from inside
# the container regardless of hostname, then store them in /tmp/.docker-xauth.
if [ -n "${DISPLAY:-}" ] && command -v xauth >/dev/null 2>&1; then
  touch /tmp/.docker-xauth 2>/dev/null || true
  chmod 666 /tmp/.docker-xauth 2>/dev/null || true

  if [ -s /tmp/.host-xauth ]; then
    xauth -f /tmp/.host-xauth nlist "${DISPLAY}" 2>/dev/null \
      | sed -e 's/^..../ffff/' \
      | xauth -f /tmp/.docker-xauth nmerge - 2>/dev/null || true
  fi
fi

# Execute the command passed to the entrypoint
exec "$@"
