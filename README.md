# Yahboom DOFBOT ROS 2 Workspace

ROS 2 **Humble** workspace, helper scripts, and utilities for developing and
testing the Yahboom DOFBOT arm: MoveIt motion planning on a PC, hardware
control on a Raspberry Pi, and a vision pipeline for object detection.

## Documentation

- **[Setup Guide](docs/setup_guide.md)** — get running in your own
  environment (WSL2 + Docker, native Ubuntu, or other), from clone to a
  working pick, including platform-specific configuration. Start here.
- [Getting Started](docs/getting_started.md) — build, simulation demo,
  real-hardware bringup (PC + Pi), vision, and troubleshooting.
- [Object picking status](docs/object_picking_status.md) — state of the
  vision-driven picking pipeline (work in progress).

## Repository layout

- `dofbot_ros2_ws/` ROS 2 workspace (packages live in `dofbot_ros2_ws/src`)
- `scripts/` Host-side scripts (deploy to Pi, MoveIt Setup Assistant)
- `tools/` Small utilities (e.g., checkerboard generation)
- `experiments/` One-off test scripts for hardware bring-up
- `docs/` Documentation

## ROS packages

| Package | Role |
|---|---|
| `dofbot_description` | URDF/xacro, meshes, RViz configs |
| `dofbot_driver` | Serial driver node for the arm servos |
| `dofbot_ros2_control` | ros2_control hardware interface (topic bridge to the driver) |
| `dofbot_moveit_config` | MoveIt configuration (launch files are thin wrappers) |
| `dofbot_bringup` | System launch files — the main entrypoints |
| `dofbot_vision` | Camera, YOLO/ArUco detection, picking nodes |

## Quick reference

Everything below is covered in detail in the
[Getting Started guide](docs/getting_started.md); this is just a reminder of
the entrypoints.

| Task | Command |
|---|---|
| Simulate (no hardware) | `ros2 launch dofbot_moveit_config demo.launch.py` |
| Arm control on the Pi | `ros2 launch dofbot_bringup control.launch.py` |
| MoveIt + RViz on the PC | `ros2 launch dofbot_bringup moveit.launch.py` |
| Sync packages to the Pi | `scripts/sync_to_pi.sh` |
