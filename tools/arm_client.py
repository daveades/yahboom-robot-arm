"""Shared arm-motion client for the calibration and chess tools.

Closed-form IK for the DOFBOT's planar geometry plus the arm/gripper
FollowJointTrajectory actions, behind simple calls:

    client.move_to(x, y, z)      Cartesian target in the base frame
    client.move_joints(pos)      joint-space target (e.g. home pose)
    client.set_gripper(pos)      grip_joint position

Trajectories are densely sampled quintic eases (zero velocity and
acceleration at both ends) so the arm starts and stops smoothly, with the
peak joint speed capped at max_speed.
"""
import math
from typing import List, Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

ARM_JOINTS = [
    "arm_joint1",
    "arm_joint2",
    "arm_joint3",
    "arm_joint4",
    "arm_joint5",
]
HOME_POSE = [0.0, 0.0, 0.0, 0.0, 0.0]  # upright, the driver's K1 pose


class ArmClient(Node):
    def __init__(self, node_name: str = "arm_client",
                 move_time: float = 3.0, max_speed: float = 0.5) -> None:
        super().__init__(node_name)
        self.move_time = move_time
        self.max_speed = max_speed
        # arm_joint4 (wrist pitch) to the grasp point between the open
        # fingertips, along the gripper axis. URDF says 0.17455 to the
        # wrist-roll frame; the fingers grip a few cm short of that.
        # If squares drift radially as the gripper descends, this is the
        # knob: drift outward when lowering -> increase, inward -> decrease.
        self.tool_len = 0.145
        # Don't approach dead-vertical: a slight slant lets the claw come
        # in across the square and wrap the piece instead of poking its
        # top. 18 deg = the gripper meets the board at ~72 deg
        # (hardware-tuned 2026-07-17; 10 was still too upright).
        self.min_tilt_deg = 18.0
        self.joint_state: Optional[JointState] = None
        self.arm_client = ActionClient(
            self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory"
        )
        self.gripper_client = ActionClient(
            self, FollowJointTrajectory, "/gripper_controller/follow_joint_trajectory"
        )
        self.create_subscription(JointState, "/joint_states", self._js_cb, 10)

    def _js_cb(self, msg: JointState) -> None:
        self.joint_state = msg

    def wait_ready(self) -> bool:
        # IK is computed locally now - only the driver's action servers
        # and /joint_states are required; move_group can be down.
        if not self.arm_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Arm controller action not available.")
            return False
        deadline = self.get_clock().now().nanoseconds + int(10e9)
        while self.joint_state is None:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.get_clock().now().nanoseconds > deadline:
                self.get_logger().error("No /joint_states received.")
                return False
        return True

    def settle(self, seconds: float) -> None:
        """Spin in place, e.g. while the driver's startup correction runs."""
        deadline = self.get_clock().now().nanoseconds + int(seconds * 1e9)
        while self.get_clock().now().nanoseconds < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)

    def current_positions(self) -> List[float]:
        start = [0.0] * len(ARM_JOINTS)
        if self.joint_state:
            lookup = {n: i for i, n in enumerate(self.joint_state.name)}
            for i, n in enumerate(ARM_JOINTS):
                if n in lookup:
                    start[i] = float(self.joint_state.position[lookup[n]])
        return start

    def _eased_points(
        self, start: List[float], target: List[float], move_time: float
    ) -> List[JointTrajectoryPoint]:
        # A single-point goal makes the controller interpolate linearly:
        # full velocity instantly at start and stop, which jerks the arm.
        # Sample a quintic ease (zero velocity AND acceleration at both
        # ends) into dense waypoints instead.
        points = []
        n_steps = max(2, int(move_time / 0.1))
        for k in range(1, n_steps + 1):
            t = k / n_steps
            s = t * t * t * (10.0 - 15.0 * t + 6.0 * t * t)  # quintic ease
            point = JointTrajectoryPoint()
            point.positions = [a + s * (b - a) for a, b in zip(start, target)]
            ts = t * move_time
            point.time_from_start = Duration(
                sec=int(ts), nanosec=int((ts - int(ts)) * 1e9)
            )
            points.append(point)
        return points

    def _spin_future(self, future, timeout_sec: float):
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        if not future.done():
            future.cancel()
            return None
        try:
            return future.result()
        except Exception as exc:
            self.get_logger().warn(f"Call failed: {exc}")
            return None

    def set_gripper(self, position: float) -> bool:
        if not self.gripper_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn("Gripper controller not available.")
            return False
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ["grip_joint"]
        point = JointTrajectoryPoint()
        point.positions = [float(position)]
        point.time_from_start = Duration(sec=1)
        goal.trajectory.points = [point]
        handle = self._spin_future(self.gripper_client.send_goal_async(goal), timeout_sec=5.0)
        if handle is None or not handle.accepted:
            return False
        result = self._spin_future(handle.get_result_async(), timeout_sec=10.0)
        return result is not None and result.result.error_code == 0

    # Kinematic constants from dofbot.urdf.xacro. The arm is planar:
    # joint1 yaws the plane, joints 2-4 pitch within it, joint 5 rolls
    # the gripper (kept at 0). Angles follow the URDF zero pose (arm
    # straight up); the gripper's cumulative pitch j2+j3+j4 is -pi when
    # pointing straight down, -pi + tilt when leaning away from the base.
    SHOULDER_Z = 0.06605 + 0.0405   # base_link -> arm_joint2 height
    L2 = 0.0829                     # arm_joint2 -> arm_joint3
    L3 = 0.0829                     # arm_joint3 -> arm_joint4
    JOINT_LIMIT = math.pi / 2       # joints 1-4 are all +-90 deg

    def solve_ik(self, x: float, y: float, z: float,
                 max_tilt_deg: Optional[float] = None,
                 attempts: int = 2) -> Optional[List[float]]:
        """Closed-form IK for the GRASP POINT (between the fingertips).

        Deterministic replacement for MoveIt's randomized position-only
        IK, which (a) positioned the wrist frame rather than the
        fingertips, so the touch point slid radially as the gripper tilt
        changed with height, and (b) returned a different contortion on
        every call. Here the tilt is scanned from vertical outward, so
        the returned pose is always the most vertical grasp available
        and identical for identical targets. `attempts` is unused (kept
        for API compatibility).
        """
        bearing = math.atan2(y, x)
        if abs(bearing) > self.JOINT_LIMIT:
            return None
        rho = math.hypot(x, y)          # radial distance in the arm plane
        zeta = z - self.SHOULDER_Z      # height relative to the shoulder
        max_tilt = 60.0 if max_tilt_deg is None else float(max_tilt_deg)

        # (rho, zeta) direction of a segment whose cumulative joint angle
        # is C: straight up at C=0, leaning toward +rho as C goes negative.
        def direction(c: float):
            return -math.sin(c), math.cos(c)

        # Prefer tilts of at least min_tilt_deg (slanted approach), but
        # fall back toward vertical rather than fail: the close rank-1
        # squares can only be grasped near-vertical.
        start = int(min(self.min_tilt_deg, max_tilt) * 2)
        scan = list(range(start, int(max_tilt * 2) + 1))
        scan += list(range(start - 1, -1, -1))
        for half_deg in scan:
            for lean in (1.0, -1.0):    # away from / toward the base
                t = lean * math.radians(half_deg * 0.5)
                # Wrist (joint4) sits tool_len back from the grasp point
                # along the gripper axis.
                wr = rho - self.tool_len * math.sin(t)
                wz = zeta + self.tool_len * math.cos(t)
                reach = math.hypot(wr, wz)
                if reach < 1e-6 or reach > self.L2 + self.L3:
                    continue
                # Isoceles triangle shoulder->elbow->wrist (L2 == L3).
                cum_wrist = math.atan2(-wr, wz)
                spread = math.acos(min(1.0, reach / (2.0 * self.L2)))
                pitch = -math.pi + t    # required j2+j3+j4
                best = None
                for elbow in (1.0, -1.0):
                    j2 = cum_wrist + elbow * spread
                    j3 = -2.0 * elbow * spread
                    j4 = pitch - j2 - j3
                    j4 -= 2.0 * math.pi * round(j4 / (2.0 * math.pi))
                    if max(abs(j2), abs(j3), abs(j4)) > self.JOINT_LIMIT + 1e-9:
                        continue
                    elbow_height = self.L2 * math.cos(j2)
                    if best is None or elbow_height > best[0]:
                        best = (elbow_height, [bearing, j2, j3, j4, 0.0])
                if best is not None:
                    return best[1]
        return None

    def move_joints(self, target: List[float],
                    move_time: Optional[float] = None) -> bool:
        start = self.current_positions()
        # Cap the peak joint speed: long swings take proportionally longer
        # (quintic ease peaks at 1.875x the average velocity).
        biggest = max(abs(b - a) for a, b in zip(start, target))
        duration = max(
            move_time or self.move_time, 1.875 * biggest / self.max_speed
        )

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM_JOINTS)
        goal.trajectory.points = self._eased_points(start, target, duration)

        handle = self._spin_future(self.arm_client.send_goal_async(goal), timeout_sec=5.0)
        if handle is None or not handle.accepted:
            self.get_logger().warn("Trajectory rejected.")
            return False
        result = self._spin_future(
            handle.get_result_async(), timeout_sec=duration + 10.0
        )
        if result is None:
            self.get_logger().warn("Trajectory result missing.")
            return False
        return result.result.error_code == 0

    def move_to(self, x: float, y: float, z: float) -> bool:
        positions = self.solve_ik(x, y, z)
        if positions is None:
            self.get_logger().warn("No acceptable IK solution. Not moving.")
            return False
        return self.move_joints(positions)
