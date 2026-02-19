# Object Picking Status (DOFBOT SE)

**Date:** 2026-02-19  
**Owner:** dave  

## Current Status

YOLO detections are publishing, and the picker node starts successfully, but **IK fails** for the target poses. The arm does not move.

Latest error:

```
IK error code: -31 for pose x=0.107 y=0.048 z=0.120
```

`-31` = **NO_IK_SOLUTION**.

## What’s Working

- `/detections` is publishing.
- `pick_from_detections` node runs and receives detections.
- MoveIt runs on PC.
- ros2_control/driver runs on Pi and publishes `/joint_states`.

## Current Blocking Issue

MoveIt IK returns **NO_IK_SOLUTION** for the generated poses. This is likely caused by:

- 5-DOF arm orientation constraints
- IK solver configuration not set to position-only
- Target pose outside reachable workspace
- Dummy homography producing unreachable XY

## Changes Already Made

### Picker Config (safe dummy mode)
`dofbot_ros2_ws/src/dofbot_vision/config/picking.yaml`

- `target_classes: ""`
- `min_confidence: 0.2`
- `approach_z/grasp_z/lift_z: 0.12`
- `pick_once: true`
- `avoid_collisions: false`
- `ik_timeout: 1.0`
- `pick_roll/pitch/yaw = 0`
- Dummy homography:
  ```
  0.00018182, 0, 0.08181818
  0, 0.00028571, 0.02142857
  0, 0, 1
  ```

### MoveIt Kinematics (prepared but requires restart)
`dofbot_ros2_ws/src/dofbot_moveit_config/config/kinematics.yaml`

```
position_only_ik: true
kinematics_solver_attempts: 10
```

## Required Next Steps

### 1. Rebuild + Restart MoveIt (to activate position-only IK)

```bash
cd /home/dave/yahboom-robot-arm/dofbot_ros2_ws
colcon build --packages-select dofbot_moveit_config
source install/setup.bash
ros2 launch dofbot_bringup moveit_pc.launch.py use_rviz:=false
```

Then rerun:
```bash
ros2 launch dofbot_vision pick.launch.py
```

### 2. If IK Still Fails

Try these in order:

1. **Change IK link**
   ```
   ik_link: arm_link4
   ```
2. **Raise Z**
   ```
   approach_z: 0.15
   grasp_z: 0.15
   lift_z: 0.15
   ```
3. **Check `compute_ik` service**
   ```
   ros2 service list | grep compute_ik
   ```

### 3. Proper Homography Calibration (for real picking)

Dummy homography is only for smoke test.  
Real calibration requires 4 pixel points + matching base (x,y) points.

Use:
```bash
python3 tools/compute_homography.py \
  --image "u1,v1;u2,v2;u3,v3;u4,v4" \
  --base "x1,y1;x2,y2;x3,y3;x4,y4"
```

Paste the output into `picking.yaml`.

## Files to Check

- Picker: `dofbot_ros2_ws/src/dofbot_vision/dofbot_vision/pick_from_detections.py`
- Picker config: `dofbot_ros2_ws/src/dofbot_vision/config/picking.yaml`
- MoveIt kinematics: `dofbot_ros2_ws/src/dofbot_moveit_config/config/kinematics.yaml`

