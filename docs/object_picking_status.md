# Object Picking Status (DOFBOT SE)

**Date:** 2026-07-09
**Owner:** dave

## Current Status

**The pipeline works end to end.** Camera → YOLO detections → pixel-to-base
mapping → IK → full pick sequence (open gripper, approach, descend, close,
lift) executes on the real arm with no IK errors.

What remains is calibration and tuning, not debugging:

- [ ] Real homography calibration (currently running the dummy matrix, so
      pick coordinates are placeholder — see procedure below)
- [ ] Lower `grasp_z` stepwise from 0.12 until the gripper actually envelops
      objects (verify hover accuracy first)
- [ ] Tune `gripper_closed` for grip strength
- [ ] Optional: set `place_x`/`place_y` for a pick-and-place demo

## What Was Actually Wrong (resolved 2026-07-08)

Two stacked bugs, found and fixed:

### 1. The IK fix never loaded

The February diagnosis (5-DOF arm needs `position_only_ik: true`) was
correct, but the flag was added to `kinematics.yaml` — **a file that
`pc_moveit_rviz.launch.py` never loads**. The launch passes exactly one
params file to move_group, `moveit_params.yaml`, which has its own
`robot_description_kinematics` section without the flag. So the solver kept
attempting full 6-DOF pose IK and returning `-31 NO_IK_SOLUTION` no matter
how many times MoveIt was restarted.

Fix: add `position_only_ik: true` to `moveit_params.yaml`. Verify against
the *running* process, which is the step that was missing before:

```bash
ros2 param get /move_group robot_description_kinematics.arm.position_only_ik
# must print: Boolean value is: True
```

### 2. The picker crashed on its first detection

`pick_from_detections` runs a `MultiThreadedExecutor`, but the pick worker
thread called `rclpy.spin_until_future_complete()` — spinning a second
executor on the same node and corrupting the wait set
(`RCLError: wait set index ... out of bounds`). This crash sat *in front of*
the IK path, so earlier testing never even reached MoveIt reliably.

Fix: the worker thread now blocks on futures via a done-callback + event
(`_wait_future()`) and never spins. Also fixed printf-style logger calls
that raised `TypeError` on the error paths.

## Verified Working (2026-07-08 run)

```
Picking knife (conf 0.27): pixel (383, 364) -> base (0.152, 0.125)
Pick once complete. Stopping further picks.
```

- `/detections` publishing (YOLO on `/image_raw`)
- `/compute_ik` resolving poses with position-only IK
- Full pick sequence executed on hardware via arm/gripper
  `FollowJointTrajectory` controllers

Note for WSL2 setups: the camera feed comes from the ffmpeg network bridge
(`ros2 run dofbot_vision stream_camera`), not `v4l2_camera` — usbipd cannot stream
webcams. See the [setup guide](setup_guide.md).

## Calibration Procedure (next step)

1. Fix the camera rigidly viewing the workspace. Moving it afterwards
   invalidates the calibration.
2. Place a small object at 4 widely-spread, non-collinear spots. For each,
   record the pixel center from `ros2 topic echo /detections`
   (`u=(x1+x2)/2, v=(y1+y2)/2`) and measure the position from the arm base
   in meters (x forward, y left).
3. ```bash
   python3 tools/compute_homography.py \
     --image "u1,v1;u2,v2;u3,v3;u4,v4" \
     --base  "x1,y1;x2,y2;x3,y3;x4,y4"
   ```
4. Paste the output into `homography:` in
   `dofbot_ros2_ws/src/dofbot_vision/config/picking.yaml`, rebuild
   `dofbot_vision`, relaunch the picker.
5. First run with `grasp_z: 0.12`: gripper should stop directly above the
   object. Only then lower it.

## Known Limitations

- Grasp orientation is uncontrolled (position-only IK on a 5-DOF arm) —
  fine for top-down grasps of small objects.
- Picker sends single-point trajectories straight to the controllers: no
  path planning, no collision checking (`avoid_collisions: false`, and no
  planning-scene obstacles exist). Keep the workspace clear.
- Homography assumes objects on the table plane; tall objects map slightly
  off. Calibration is only valid within the region spanned by the 4 points.
- Driver is open-loop (`/joint_states` echoes commands): pose the arm
  straight up before starting the stack.

## Files

- Picker: `dofbot_ros2_ws/src/dofbot_vision/dofbot_vision/pick_from_detections.py`
- Picker config: `dofbot_ros2_ws/src/dofbot_vision/config/picking.yaml`
- MoveIt params (the file that is actually loaded):
  `dofbot_ros2_ws/src/dofbot_moveit_config/config/moveit_params.yaml`
- Camera bridge: `dofbot_vision/stream_camera.py` (`ros2 run dofbot_vision stream_camera` or `scripts/camera.sh`)
