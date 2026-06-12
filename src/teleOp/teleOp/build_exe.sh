#!/bin/bash
# Run this script ONCE on your Steam Deck (or any x86_64 Linux machine with Steam).
# It builds a standalone executable from pygame_sender.py.
#
# After it finishes, add the resulting file to Steam:
#   Steam → Games → Add a Non-Steam Game → Browse → select teleop_sender
#   In Properties → Launch Options: --host 192.168.1.XXX

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT_DIR="$SCRIPT_DIR/dist"

PYTHON=$(command -v python3 \
  || ls /usr/bin/python3 2>/dev/null \
  || ls /home/deck/.local/bin/python3 2>/dev/null \
  || echo "")

if [ -z "$PYTHON" ]; then
    echo "ERROR: python3 not found"
    exit 1
fi

for pkg in pygame pyinstaller; do
    if ! "$PYTHON" -c "import $pkg" 2>/dev/null; then
        echo "Installing $pkg..."
        "$PYTHON" -m pip install --user "$pkg"
    fi
done

PYINSTALLER="$PYTHON -m PyInstaller"

echo "Building executable..."
$PYINSTALLER \
    --onefile \
    --name teleop_sender \
    --distpath "$OUT_DIR" \
    --workpath "$SCRIPT_DIR/.pyibuild" \
    --specpath "$SCRIPT_DIR/.pyibuild" \
    --clean \
    --noconfirm \
    "$SCRIPT_DIR/pygame_sender.py"

rm -rf "$SCRIPT_DIR/.pyibuild"

echo ""
echo "Done! Add this to Steam as a Non-Steam Game:"
echo "  $OUT_DIR/teleop_sender"
