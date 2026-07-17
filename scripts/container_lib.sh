# Shared preamble for the container-side scripts. Not runnable on its own.
if [ ! -f /.dockerenv ]; then
    echo "This script must run INSIDE the dofbot container:" >&2
    echo "    docker exec -it dofbot bash" >&2
    echo "(host side: scripts/container.sh then scripts/usb.sh)" >&2
    exit 1
fi

WS=/root/yahboom-robot-arm
# ROS setup files reference unset vars, so relax -u while sourcing.
# Single workspace: dofbot_ros2_ws (the old top-level tree is gone —
# rebuild with ros-build / scripts/one_time/setup_container.sh if
# setup.bash is missing).
set +u
source /opt/ros/humble/setup.bash
source "$WS/dofbot_ros2_ws/install/setup.bash"
set -u
