#!/bin/bash
# Wheel Teleop launcher — add this file to Steam as a Non-Steam Game.
#
# Set ROBOT_IP to your robot's IP address before use.
# Leave as 127.0.0.1 if running on the same machine as the receiver.

ROBOT_IP="127.0.0.1"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SENDER="$SCRIPT_DIR/pygame_sender.py"

# Prefer the system python3; fall back to common Steam Deck locations
PYTHON=$(command -v python3 \
  || ls /usr/bin/python3 2>/dev/null \
  || ls /home/deck/.local/bin/python3 2>/dev/null \
  || echo "")

if [ -z "$PYTHON" ]; then
    echo "ERROR: python3 not found"
    read -r -p "Press enter to close..."
    exit 1
fi

if ! "$PYTHON" -c "import pygame" 2>/dev/null; then
    echo "pygame not found — installing..."
    "$PYTHON" -m pip install --user pygame
fi

exec "$PYTHON" "$SENDER" --host "$ROBOT_IP"
