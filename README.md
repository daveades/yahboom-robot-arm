# Yahboom DOFBOT ROS 2 Workspace

ROS 2 **Humble** workspace, helper scripts, and utilities for developing and
testing the Yahboom DOFBOT arm: MoveIt motion planning on a PC, hardware
control on a Raspberry Pi, and a vision pipeline for object detection.

## Documentation

- **[User Manual](docs/user_manual.md)** — install through vision
  pick-and-place, in order, with the explanations. **Start here.**
- [Demo Runbook](docs/demo_runbook.md) — cold-machine checklist for the
  chess demo.

## Repository layout

- `dofbot_ros2_ws/` ROS 2 workspace (packages live in `dofbot_ros2_ws/src`)
- `scripts/` Bringup scripts: container, USB, driver, MoveIt, sim, camera
- `tools/` Standalone Python utilities: teleop, calibration, the chess game
- `config/` Board model (`board.yaml`) — the single source of truth
- `docs/` Documentation

## ROS packages

| Package | Role |
|---|---|
| `dofbot_description` | URDF/xacro, meshes, RViz configs |
| `dofbot_driver` | Serial driver; serves the trajectory actions on hardware |
| `dofbot_moveit_config` | MoveIt configuration (config only — no launch files) |
| `dofbot_bringup` | System launch files — the main entrypoints |
| `dofbot_vision` | Camera, YOLO/ArUco detection, picking nodes |

On real hardware the driver executes trajectories itself; `ros2_control` is
used only in simulation. See [user manual §7.9](docs/user_manual.md).

## Quick reference

| Task | Command |
|---|---|
| Simulate (no hardware) | `scripts/sim.sh` — or `ros2 launch dofbot_bringup demo.launch.py` |
| Arm driver (machine wired to the arm) | `scripts/driver.sh` — or `ros2 launch dofbot_bringup control.launch.py` |
| MoveIt + RViz | `scripts/moveit.sh` — or `ros2 launch dofbot_bringup moveit.launch.py` |
| Keyboard teleop | `python3 tools/teleop_key.py` |
| Health check | `scripts/status.sh` |
