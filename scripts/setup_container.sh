#!/usr/bin/env bash
# Container provisioning + workspace migration. Run INSIDE the container,
# once after cloning / pulling structural changes. Idempotent.
#
#   docker exec -it dofbot bash
#   /root/yahboom-robot-arm/scripts/setup_container.sh
#
# Does three things:
#   1. removes the legacy top-level build/install/log trees (the project
#      uses ONE workspace: dofbot_ros2_ws)
#   2. makes ~/.bashrc source ROS + the workspace and define ros-build
#   3. clean-rebuilds dofbot_ros2_ws
set -eu

if [ ! -f /.dockerenv ]; then
    echo "Run this inside the dofbot container: docker exec -it dofbot bash" >&2
    exit 1
fi
WS=/root/yahboom-robot-arm

echo "==> 1/3 Removing legacy top-level build trees"
rm -rf "$WS/build" "$WS/install" "$WS/log"

echo "==> 2/3 Setting up ~/.bashrc"
# drop any line sourcing the old top-level install tree
sed -i '\|yahboom-robot-arm/install/setup.bash|d' /root/.bashrc
grep -q '^source /opt/ros/humble/setup.bash' /root/.bashrc \
    || echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc
grep -q 'dofbot_ros2_ws/install/setup.bash' /root/.bashrc \
    || echo "[ -f $WS/dofbot_ros2_ws/install/setup.bash ] && source $WS/dofbot_ros2_ws/install/setup.bash" >> /root/.bashrc
grep -q 'alias ros-build=' /root/.bashrc \
    || echo "alias ros-build='colcon build --symlink-install'" >> /root/.bashrc

echo "==> 3/3 Clean rebuild of dofbot_ros2_ws"
set +u  # ROS setup files reference unset vars (AMENT_TRACE_SETUP_FILES)
source /opt/ros/humble/setup.bash
set -u
cd "$WS/dofbot_ros2_ws"
rm -rf build install log
colcon build --symlink-install

echo
echo "Done. Open a fresh shell (or 'source ~/.bashrc') and use:"
echo "    scripts/driver.sh | scripts/moveit.sh | scripts/camera.sh | scripts/status.sh"
