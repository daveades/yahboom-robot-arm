#!/usr/bin/env bash
# SIMULATION stack, FOREGROUND — no hardware, no serial, no driver.
# ros2_control with mock hardware + controllers + MoveIt + RViz: plan and
# execute in RViz and the virtual arm follows. Ctrl-C stops everything.
#
# Do NOT run driver.sh/moveit.sh at the same time (same node/action names).
#
# Usage (inside the container):
#   scripts/sim.sh
#   scripts/sim.sh --no-rviz
set -u
source "$(dirname -- "${BASH_SOURCE[0]}")/container_lib.sh"

USE_RVIZ=true
for arg in "$@"; do
    [ "$arg" = "--no-rviz" ] && USE_RVIZ=false
done

exec ros2 launch dofbot_bringup demo.launch.py use_rviz:="$USE_RVIZ"
