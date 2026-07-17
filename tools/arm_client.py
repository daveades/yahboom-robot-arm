"""Shared arm-motion client for the calibration and chess tools.

Wraps the same path pick_from_detections uses - /compute_ik (position-only
IK must be active in move_group) + the arm/gripper FollowJointTrajectory
actions - behind simple calls:

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
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.msg import MoveItErrorCodes, RobotState
from moveit_msgs.srv import GetPositionIK

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
        self.joint_state: Optional[JointState] = None
        self.ik_client = self.create_client(GetPositionIK, "/compute_ik")
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
        if not self.ik_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("IK service /compute_ik not available.")
            return False
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

    def solve_ik(self, x: float, y: float, z: float) -> Optional[List[float]]:
        """Position-only IK with sanity filtering; None if unreachable."""
        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)
        pose.pose.orientation.w = 1.0  # ignored: position-only IK

        bearing = math.atan2(y, x)
        current = self.current_positions()

        # Seed IK from the CURRENT elbow pose (base swapped to the new
        # bearing) so consecutive targets get the same arm fold and the
        # transition is a direct sweep. Fall back to a neutral reach pose
        # (also used from upright, where the zero pose is a bad seed).
        seeds = []
        if any(abs(p) > 0.15 for p in current[1:4]):
            seeds.append([bearing] + current[1:4] + [0.0])
        seeds.append([bearing, 0.5, -1.0, -0.8, 0.0])

        # The solver is randomized: a solvable pose can fail one attempt,
        # so run the seed list twice before giving up.
        for seed_pos in seeds + seeds:
            seed = JointState()
            seed.name = list(ARM_JOINTS)
            seed.position = seed_pos

            req = GetPositionIK.Request()
            req.ik_request.group_name = "arm"
            req.ik_request.ik_link_name = "arm_link5"
            req.ik_request.pose_stamped = pose
            req.ik_request.avoid_collisions = False
            req.ik_request.timeout = Duration(sec=1)
            req.ik_request.robot_state = RobotState(joint_state=seed)

            res = self._spin_future(self.ik_client.call_async(req), timeout_sec=5.0)
            if res is None:
                self.get_logger().warn("IK service call failed or timed out.")
                continue
            if res.error_code.val != MoveItErrorCodes.SUCCESS:
                self.get_logger().warn(
                    f"IK error {res.error_code.val} for x={x:.3f} y={y:.3f} "
                    f"z={z:.3f} (target may be out of reach)"
                )
                continue

            lookup = {n: i for i, n in enumerate(res.solution.joint_state.name)}
            try:
                cand = [
                    float(res.solution.joint_state.position[lookup[n]])
                    for n in ARM_JOINTS
                ]
            except KeyError:
                self.get_logger().warn("IK solution missing arm joints.")
                continue

            # Position-only IK has 2 spare DOF and can return folded/rolled
            # poses. A sane top-down reach must swing the base toward the
            # target and keep the wrist roll near zero.
            j1_err = math.degrees(abs(cand[0] - bearing))
            j5_off = math.degrees(abs(cand[4]))
            if j1_err > 8.0 or j5_off > 20.0:
                self.get_logger().warn(
                    f"Rejecting contorted IK solution: base off-bearing by "
                    f"{j1_err:.0f}deg, wrist roll {j5_off:.0f}deg."
                )
                continue
            return cand
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
