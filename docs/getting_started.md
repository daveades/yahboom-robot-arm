# Getting Started

This guide walks you from a fresh clone to planning motions in RViz, first in
simulation and then on the real Yahboom DOFBOT arm. The workspace targets
**ROS 2 Humble** (Ubuntu 22.04).

## How the pieces fit together

```
MoveIt (move_group + RViz)
        │  FollowJointTrajectory actions
        ▼
ros2_control (joint_trajectory_controller @ 100 Hz)
        │
        ▼
DofbotTopicHardware (dofbot_ros2_control)
        │  publishes /target_joints          subscribes /joint_states
        ▼                                            ▲
dofbot_driver (serial servo driver) ─────────────────┘
        │
        ▼
   DOFBOT servos (/dev/ttyUSB0)
```

| Package | Role |
|---|---|
| `dofbot_description` | URDF/xacro, meshes, RViz configs |
| `dofbot_driver` | Python serial driver: `/target_joints` in, `/joint_states` out |
| `dofbot_ros2_control` | Hardware interface bridging ros2_control to the driver topics |
| `dofbot_moveit_config` | MoveIt configuration (SRDF, kinematics, controllers, planning params) |
| `dofbot_bringup` | The launch files you actually run |
| `dofbot_vision` | Camera, YOLO/ArUco detection, pick-from-detections nodes |

Key detail: when **no driver is running**, the hardware interface feeds its
commands back as state, so the whole stack behaves like a simulator. That is
what makes the demo below work with zero hardware.

## 1. Prerequisites

A ROS 2 Humble environment with MoveIt:

```bash
sudo apt install ros-humble-desktop ros-humble-moveit \
  ros-humble-ros2-control ros-humble-ros2-controllers
```

**Running in Docker on WSL2?** Start the container with display access:

```bash
docker run -it --net=host \
  -e DISPLAY=$DISPLAY -e WAYLAND_DISPLAY=$WAYLAND_DISPLAY \
  -e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
  -v /tmp/.X11-unix:/tmp/.X11-unix -v /mnt/wslg:/mnt/wslg \
  <your-humble-image>
```

Add `-e LIBGL_ALWAYS_SOFTWARE=1` if RViz opens but renders garbage.

## 2. Build

```bash
source /opt/ros/humble/setup.bash
cd dofbot_ros2_ws
rm -rf build install log        # clean build recommended after pulling changes
colcon build --symlink-install
source install/setup.bash
```

## 3. Simulation demo (no hardware)

```bash
ros2 launch dofbot_moveit_config demo.launch.py
```

This single command starts `move_group`, `ros2_control` with the fake-loop
hardware, the `arm_controller` / `gripper_controller`, and RViz with the
MotionPlanning display preloaded.

### Planning your first motion

1. In the **MotionPlanning** panel, open the **Planning** tab.
2. Set **Planning Group** to `arm` (the 5 arm joints; `gripper` is the grip joint).
3. Give it a goal:
   - Drag the orange **interactive marker** at the end effector. Use the
     position arrows — the IK is configured position-only
     (`position_only_ik: true`), so orientation is ignored. This suits the
     5-DOF arm.
   - Or under **Goal State** pick `<random valid>` for a quick test.
4. Click **Plan** to preview, then **Execute** (or **Plan & Execute**).

The default planning pipeline is **OMPL**; **Pilz** (straight-line PTP/LIN)
and **CHOMP** are also configured and selectable from the pipeline dropdown.

### Motion speed

Default velocity/acceleration scaling is **0.5** (half of the 1.0 rad/s joint
limits). Override per-plan with the **Velocity Scaling / Accel Scaling**
dropdowns in the Planning tab, or change the defaults in:

- `dofbot_moveit_config/config/moveit_params.yaml` (`robot_description_planning:` section — used by `demo.launch.py` and the PC launches)
- `dofbot_moveit_config/config/joint_limits.yaml` (used by the wrapper launches)

## 4. Real hardware

The intended split: the **Raspberry Pi** (plugged into the arm over USB
serial) runs the driver and ros2_control; your **PC** runs MoveIt and RViz.
Both machines must share the network and the same `ROS_DOMAIN_ID`.

### Sync code to the Pi

```bash
scripts/sync_to_pi.sh                 # all packages
scripts/sync_to_pi.sh dofbot_driver   # or specific ones
```

Configure via env vars: `PI_HOST` (default `ubuntu@192.168.82.50`), `PI_KEY`
(default `~/.ssh/raspi`), `PI_BASE`. Then build on the Pi the same way as in
step 2.

### Driver (container terminal 1)

```bash
scripts/driver.sh        # or: ros2 launch dofbot_bringup control.launch.py
```

This starts the serial driver and robot_state_publisher; the driver itself
serves the arm/gripper FollowJointTrajectory actions (no ros2_control on
hardware). Useful driver parameters (via `--ros-args -p`):
`max_speed_deg_s` (default 120), `startup_time_ms` startup glide,
`gripper_open_deg` / `gripper_closed_deg`, `min_delta_deg` jitter filter.

### MoveIt (container terminal 2)

```bash
scripts/moveit.sh        # or: ros2 launch dofbot_bringup moveit.launch.py
```

RViz opens; plan and execute exactly as in the simulation section — Execute
now moves the physical servos. Start with low velocity scaling (0.1–0.3)
until you trust a motion; the workspace has no obstacle model beyond the arm
itself.

### Sanity checks before executing

```bash
ros2 control list_controllers        # on Pi: all three should be "active"
ros2 topic echo /joint_states --once # positions should track the real arm
ros2 topic list | grep target_joints # command path to the driver exists
```

## 5. Vision (optional)

Start the camera bridge (Windows streamer + container node — see
setup_guide section 4):

```bash
scripts/camera.sh        # publishes /image_raw
```

Install the YOLO dependencies once, in a venv that can still see
the ROS Python packages:

```bash
source /opt/ros/humble/setup.bash
python3 -m venv --system-site-packages ~/venvs/ros2_yolo
source ~/venvs/ros2_yolo/bin/activate
pip install -U pip
pip install ultralytics
```

Then run the detector (with the venv active):

```bash
cd dofbot_ros2_ws
colcon build --symlink-install --packages-select dofbot_vision
source install/setup.bash
ros2 launch dofbot_vision yolo.launch.py image_topic:=/image_raw model:=yolov8n.pt device:=cpu
```

The object-picking pipeline (`pick.launch.py`, `pick_from_detections`) is
the basis for the chess pick-and-place work.

## 6. Development notes

- Launch files live in `dofbot_bringup`; `dofbot_moveit_config` holds
  configuration only.
- Always build in `dofbot_ros2_ws` (use the `ros-build` alias); if a build
  behaves strangely, rebuild from a clean `build/ install/ log/`.
- To regenerate the MoveIt configuration with the Setup Assistant:
  `scripts/one_time/run_moveit_setup.sh`.

## Troubleshooting

- **RViz shows no robot / errors about robot_description** — you skipped
  `source install/setup.bash`, or the build is stale. Rebuild clean.
- **Controllers stuck "inactive" or spawner times out** — check the
  `ros2_control_node` output for a plugin load error; a mismatched
  build (pre-Humble artifacts in `build/`/`install/`) is the usual cause.
- **`Execute` succeeds in RViz but the arm does not move** — the driver
  is not running or cannot open the serial port. Check
  `ros2 topic echo /target_joints` while executing and the driver log for
  serial errors (permissions → `dialout` group).
- **PC and Pi cannot see each other's topics** — `ROS_DOMAIN_ID` mismatch,
  or multicast blocked on the network. Test with `ros2 topic list` on both.
- **Arm jumps on startup** — expected safeguard: the hardware interface
  initializes commands from the first real joint state, and the driver rate-
  limits the first moves (`startup_time_ms`). If it still jumps, verify
  `/joint_states` reflects reality before spawning controllers.
