#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
WS_DIR="$REPO_ROOT/dofbot_ros2_ws"
cd "$WS_DIR"

source /opt/ros/jazzy/setup.bash
if [ -f "$WS_DIR/install/setup.bash" ]; then
  source "$WS_DIR/install/setup.bash"
fi

# Force X11/GLX path for RViz/OGRE to avoid Wayland parent window errors.
unset WAYLAND_DISPLAY
export QT_QPA_PLATFORM=xcb
export XDG_SESSION_TYPE=x11
export LIBGL_ALWAYS_SOFTWARE=1
export QT_OPENGL=software

exec ros2 launch moveit_setup_assistant setup_assistant.launch.py
