# Yahboom DOFBOT ROS 2 Workspace

This repo contains the ROS 2 workspace plus helper scripts and utilities used to develop and test the Yahboom DOFBOT arm.

**Layout**
- `dofbot_ros2_ws/` ROS 2 workspace (packages live in `dofbot_ros2_ws/src`)
- `scripts/` Host-side scripts (deploy, setup helpers)
- `tools/` Small utilities (e.g., checkerboard generation)
- `experiments/` One-off test scripts for hardware bring-up

**Key ROS packages**
- `dofbot_description` URDF/xacro + meshes + RViz configs
- `dofbot_driver` Hardware driver node
- `dofbot_ros2_control` ros2_control hardware interface
- `dofbot_moveit_config` MoveIt configuration (now thin wrapper launches)
- `dofbot_bringup` System launch files (main entrypoints)
- `dofbot_vision` Vision nodes

## Quick Start

### 1) Build
```bash
source /opt/ros/jazzy/setup.bash
cd dofbot_ros2_ws
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
```

### 2) Hardware bringup (Pi)
```bash
cd dofbot_ros2_ws
ros2 launch dofbot_bringup robot.launch.py port:=/dev/ttyUSB0 use_rviz:=false
```

### 3) ros2_control on Pi (required for MoveIt execution)
```bash
cd dofbot_ros2_ws
ros2 launch dofbot_bringup pi_control.launch.py
```

### 4) MoveIt on PC
```bash
cd dofbot_ros2_ws
ros2 launch dofbot_bringup moveit_pc.launch.py
```

### 5) Simulation-only (no hardware)
```bash
cd dofbot_ros2_ws
ros2 launch dofbot_moveit_config demo.launch.py
```

## Notes
- `dofbot_moveit_config` launch files are wrappers now. Prefer `dofbot_bringup` for actual use.
- If you rename or move packages, always rebuild from a clean `build/ install/ log/`.

## Utilities
Sync packages to the Pi:
```bash
scripts/sync_to_pi.sh
```

Run MoveIt Setup Assistant:
```bash
scripts/run_moveit_setup.sh
```

## Vision (PC)
Install YOLO dependencies in a venv (PC):
```bash
source /opt/ros/jazzy/setup.bash
python3 -m venv --system-site-packages ~/venvs/ros2_yolo
source ~/venvs/ros2_yolo/bin/activate
pip install -U pip
pip install ultralytics
```

Run detector:
```bash
cd dofbot_ros2_ws
colcon build --symlink-install --packages-select dofbot_vision
source install/setup.bash
ros2 launch dofbot_vision yolo.launch.py image_topic:=/image_raw model:=yolov8n.pt device:=cpu
```
