#!/usr/bin/env bash
# MoveIt (move_group + optional RViz), FOREGROUND.
# All node output streams to this terminal; Ctrl-C stops it.
# Start scripts/driver.sh first (in another container shell).
#
# Usage (inside the container):
#   scripts/moveit.sh              # with RViz
#   scripts/moveit.sh --no-rviz    # headless
set -u
source "$(dirname -- "${BASH_SOURCE[0]}")/container_lib.sh"

USE_RVIZ=true
for arg in "$@"; do
    [ "$arg" = "--no-rviz" ] && USE_RVIZ=false
done

exec ros2 launch dofbot_bringup moveit.launch.py use_rviz:="$USE_RVIZ"
