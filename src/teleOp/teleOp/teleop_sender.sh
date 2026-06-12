#!/bin/bash
# Wheel Teleop sender — add this file to Steam as a Non-Steam Game.
# In Steam game properties → Compatibility: ensure "Force Proton" is OFF.
# Set the robot IP in Launch Options: --host 192.168.1.XXX

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SENDER="$SCRIPT_DIR/pygame_sender.py"

PYTHON=$(command -v python3 2>/dev/null \
  || ls /usr/bin/python3 2>/dev/null \
  || ls /home/deck/.local/bin/python3 2>/dev/null \
  || echo "")

if [ -z "$PYTHON" ]; then
    echo "ERROR: python3 not found"
    read -r -p "Press enter to close..."
    exit 1
fi

if ! "$PYTHON" -c "import pygame" 2>/dev/null; then
    "$PYTHON" -m pip install --user pygame
fi

exec "$PYTHON" "$SENDER" "$@"
