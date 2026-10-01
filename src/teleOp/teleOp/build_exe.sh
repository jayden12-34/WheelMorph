#!/bin/bash
# Run this script ONCE on your Steam Deck (or any x86_64 Linux machine with Steam).
# It builds a standalone executable from pygame_sender.py.
#
# After it finishes, add the resulting file to Steam:
#   Steam → Games → Add a Non-Steam Game → Browse → select teleop_sender
#
# Normal use:
#   No Launch Options needed.
#   The sender automatically discovers the robot receiver on the local network.
#
# Manual override, if discovery does not work:
#   Steam → Properties → Launch Options:
#       --host 192.168.1.XXX

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT_DIR="$SCRIPT_DIR/dist"
VENV="$SCRIPT_DIR/.buildenv"

PYTHON=$(command -v python3 \
  || ls /usr/bin/python3 2>/dev/null \
  || ls /home/deck/.local/bin/python3 2>/dev/null \
  || echo "")

if [ -z "$PYTHON" ]; then
    echo "ERROR: python3 not found"
    exit 1
fi

# Use a venv to avoid "externally managed environment" errors (PEP 668)
echo "Creating build environment..."
"$PYTHON" -m venv "$VENV"

PYTHON="$VENV/bin/python"
PIP="$VENV/bin/pip"

echo "Installing dependencies..."
"$PIP" install --quiet pygame opencv-python numpy pyinstaller

echo "Building executable..."
"$VENV/bin/pyinstaller" \
    --onefile \
    --name teleop_sender \
    --distpath "$OUT_DIR" \
    --workpath "$SCRIPT_DIR/.pyibuild" \
    --specpath "$SCRIPT_DIR/.pyibuild" \
    --collect-all cv2 \
    --clean \
    --noconfirm \
    "$SCRIPT_DIR/pygame_sender.py"

rm -rf "$SCRIPT_DIR/.pyibuild" "$VENV"

echo ""
echo "Done! Add this to Steam as a Non-Steam Game:"
echo "  $OUT_DIR/teleop_sender"
echo ""
echo "Normal use:"
echo "  No Launch Options required."
echo "  The sender will automatically discover the robot."
echo ""
echo "Manual override:"
echo "  --host 192.168.1.XXX"