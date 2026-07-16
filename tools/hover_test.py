#!/usr/bin/env python3
"""Hover the gripper over chess-board squares to calibrate the board->base
transform.

Board convention (looking down at the table, robot base at origin,
x forward, y left):

  * ``--a1 X Y``     center of square a1 in base frame, meters
  * ``--square S``   square size in meters (e.g. 0.025)
  * ``--yaw DEG``    direction files run (a->h). 0 deg = +x (away from
                     robot), 90 deg = +y (to the robot's left).
  * ``--mirror``     flip the rank direction (1->8) to the other side of
                     the file axis. Use this if the hover lands mirrored.

Ranks (1->8) run 90 deg counterclockwise from the file direction unless
--mirror is given.

The script prints the computed (x, y) for every requested square, asks for
confirmation, then hovers over each square in turn at --z, pausing for
Enter between squares so you can check alignment with a ruler or by eye.

Uses the same path as pick_from_detections: /compute_ik (position-only IK
must be active in move_group) and /arm_controller/follow_joint_trajectory.

Example:
  python3 tools/hover_test.py --a1 0.16 -0.09 --square 0.025 --yaw 90
"""

import argparse
import math
import sys
import time
from typing import List, Optional, Tuple

try:  # ROS only needed for actual motion; --check-only works without it
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
    HAVE_ROS = True
except ImportError:
    HAVE_ROS = False
    Node = object  # allow class definition without rclpy

DEFAULT_SQUARES = ["a1", "h1", "h8", "a8", "d4", "e5"]


def square_to_xy(
    square: str,
    a1: Tuple[float, float],
    size: float,
    yaw_deg: float,
    mirror: bool,
) -> Tuple[float, float]:
    name = square.strip().lower()
    if len(name) != 2 or name[0] not in "abcdefgh" or name[1] not in "12345678":
        raise ValueError(f"Bad square name: {square!r}")
    file_idx = ord(name[0]) - ord("a")  # 0..7 along a->h
    rank_idx = int(name[1]) - 1  # 0..7 along 1->8

    yaw = math.radians(yaw_deg)
    fx, fy = math.cos(yaw), math.sin(yaw)  # file direction (a->h)
    # rank direction: +90 deg CCW from files, or -90 deg with --mirror
    if mirror:
        rx, ry = fy, -fx
    else:
        rx, ry = -fy, fx

    x = a1[0] + size * (file_idx * fx + rank_idx * rx)
    y = a1[1] + size * (file_idx * fy + rank_idx * ry)
    return x, y


class HoverTest(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("hover_test")
        self.args = args
        self.joint_state: Optional[JointState] = None
        self.arm_joints = [
            "arm_joint1",
            "arm_joint2",
            "arm_joint3",
            "arm_joint4",
            "arm_joint5",
        ]
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

    def _current_positions(self) -> List[float]:
        start = [0.0] * len(self.arm_joints)
        if self.joint_state:
            lookup = {n: i for i, n in enumerate(self.joint_state.name)}
            for i, n in enumerate(self.arm_joints):
                if n in lookup:
                    start[i] = float(self.joint_state.position[lookup[n]])
        return start

    def _eased_points(
        self, start: List[float], target: List[float], move_time: float
    ) -> List["JointTrajectoryPoint"]:
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

    def hover(self, x: float, y: float) -> bool:
        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(self.args.z)
        pose.pose.orientation.w = 1.0  # ignored: position-only IK

        bearing = math.atan2(y, x)
        current = self._current_positions()

        # Seed IK from the CURRENT elbow pose (base swapped to the new
        # bearing) so consecutive squares get the same arm fold and the
        # transition is a direct sweep. Fall back to a neutral reach pose
        # (also used from upright, where the zero pose is a bad seed).
        seeds = []
        if any(abs(p) > 0.15 for p in current[1:4]):
            seeds.append([bearing] + current[1:4] + [0.0])
        seeds.append([bearing, 0.5, -1.0, -0.8, 0.0])

        positions = None
        for seed_pos in seeds:
            seed = JointState()
            seed.name = list(self.arm_joints)
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
                    f"z={self.args.z:.3f} (square may be out of reach)"
                )
                continue

            lookup = {n: i for i, n in enumerate(res.solution.joint_state.name)}
            try:
                cand = [
                    float(res.solution.joint_state.position[lookup[n]])
                    for n in self.arm_joints
                ]
            except KeyError:
                self.get_logger().warn("IK solution missing arm joints.")
                continue

            # Position-only IK has 2 spare DOF and can return folded/rolled
            # poses. A sane top-down hover must swing the base toward the
            # target and keep the wrist roll near zero.
            j1_err = math.degrees(abs(cand[0] - bearing))
            j5_off = math.degrees(abs(cand[4]))
            if j1_err > 8.0 or j5_off > 20.0:
                self.get_logger().warn(
                    f"Rejecting contorted IK solution: base off-bearing by "
                    f"{j1_err:.0f}deg, wrist roll {j5_off:.0f}deg."
                )
                continue
            positions = cand
            break

        if positions is None:
            self.get_logger().warn("No acceptable IK solution. Not moving.")
            return False

        start = current
        # Cap the peak joint speed: long swings take proportionally longer
        # (quintic ease peaks at 1.875x the average velocity).
        biggest = max(abs(b - a) for a, b in zip(start, positions))
        move_time = max(
            self.args.move_time, 1.875 * biggest / self.args.max_speed
        )

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.arm_joints
        goal.trajectory.points = self._eased_points(start, positions, move_time)

        handle = self._spin_future(self.arm_client.send_goal_async(goal), timeout_sec=5.0)
        if handle is None or not handle.accepted:
            self.get_logger().warn("Trajectory rejected.")
            return False
        result = self._spin_future(
            handle.get_result_async(), timeout_sec=move_time + 10.0
        )
        if result is None:
            self.get_logger().warn("Trajectory result missing.")
            return False
        return result.result.error_code == 0


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--a1", nargs=2, type=float, required=True,
                        metavar=("X", "Y"),
                        help="center of square a1 in base frame, meters")
    parser.add_argument("--square", type=float, required=True,
                        help="square size in meters, e.g. 0.025")
    parser.add_argument("--yaw", type=float, default=0.0,
                        help="file direction a->h in degrees (0=+x, 90=+y)")
    parser.add_argument("--mirror", action="store_true",
                        help="flip rank direction to the other side")
    parser.add_argument("--z", type=float, default=0.12,
                        help="hover height in meters (default 0.12)")
    parser.add_argument("--move-time", type=float, default=3.0,
                        help="minimum seconds per move (default 3.0)")
    parser.add_argument("--max-speed", type=float, default=0.5,
                        help="peak joint speed in rad/s; long swings are "
                             "slowed to respect this (default 0.5)")
    parser.add_argument("--gripper", type=float, default=None, metavar="POS",
                        help="close gripper to this grip_joint position first "
                             "(e.g. -1.0) so the tips act as a pointer")
    parser.add_argument("--check-only", action="store_true",
                        help="print square coordinates and exit, no motion")
    parser.add_argument("squares", nargs="*", default=None,
                        help=f"squares to visit (default: {' '.join(DEFAULT_SQUARES)})")
    args = parser.parse_args()

    squares = args.squares or DEFAULT_SQUARES
    a1 = (args.a1[0], args.a1[1])

    print(f"\nBoard: a1=({a1[0]:.3f}, {a1[1]:.3f})  square={args.square*1000:.0f}mm  "
          f"yaw={args.yaw:.0f}deg  mirror={args.mirror}  hover z={args.z:.3f}\n")
    targets = []
    for sq in squares:
        x, y = square_to_xy(sq, a1, args.square, args.yaw, args.mirror)
        dist = math.hypot(x, y)
        ang = math.degrees(math.atan2(y, x))
        note = ""
        if abs(ang) > 90.0:
            note = "  <-- BEYOND joint1 limit (+/-90deg)!"
        elif abs(ang) > 80.0:
            note = "  <-- close to joint1 limit"
        print(f"  {sq}: x={x:+.3f}  y={y:+.3f}  "
              f"(dist {dist*100:.1f}cm, angle {ang:+.0f}deg){note}")
        targets.append((sq, x, y))
    print()

    if args.check_only:
        return

    if not HAVE_ROS:
        print("rclpy not available in this environment — run inside the ROS "
              "container for motion, or use --check-only here.")
        sys.exit(1)

    answer = input("Move the arm through these squares? [y/N] ").strip().lower()
    if answer != "y":
        print("Aborted, no motion sent.")
        return

    rclpy.init()
    node = HoverTest(args)
    try:
        if not node.wait_ready():
            sys.exit(1)
        # If the stack was just launched, the driver spends ~3s pulling the
        # arm to its assumed-upright pose. Moving during that correction
        # makes commands and reality fight; wait it out.
        print("Settling 4s (lets any driver startup correction finish) ...")
        settle_end = time.time() + 4.0
        while time.time() < settle_end:
            rclpy.spin_once(node, timeout_sec=0.2)
        if args.gripper is not None:
            print(f"Setting gripper to {args.gripper} ...")
            node.set_gripper(args.gripper)
        for sq, x, y in targets:
            print(f"\n--> Hovering over {sq} (x={x:.3f}, y={y:.3f}) ...")
            ok = node.hover(x, y)
            print(f"    {'OK' if ok else 'FAILED'} - check gripper alignment over {sq}")
            if sq != targets[-1][0]:
                input("    Press Enter for next square (Ctrl-C to stop) ")
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
