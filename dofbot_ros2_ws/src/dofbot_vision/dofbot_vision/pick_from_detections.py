import json
import math
import threading
import time
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.msg import MoveItErrorCodes, RobotState
from moveit_msgs.srv import GetPositionIK


def rpy_to_quat(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


class PickFromDetections(Node):
    def __init__(self) -> None:
        super().__init__("pick_from_detections")

        self.declare_parameter("detections_topic", "/detections")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("target_classes", "")
        self.declare_parameter("min_confidence", 0.5)
        self.declare_parameter("homography", [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("group_name", "arm")
        self.declare_parameter("ik_link", "arm_link5")
        self.declare_parameter(
            "arm_joints",
            ["arm_joint1", "arm_joint2", "arm_joint3", "arm_joint4", "arm_joint5"],
        )
        self.declare_parameter("gripper_joint", "grip_joint")
        self.declare_parameter("ik_service", "/compute_ik")
        self.declare_parameter("ik_timeout", 0.5)
        self.declare_parameter("avoid_collisions", True)
        self.declare_parameter("arm_action", "/arm_controller/follow_joint_trajectory")
        self.declare_parameter("gripper_action", "/gripper_controller/follow_joint_trajectory")
        self.declare_parameter("approach_z", 0.08)
        self.declare_parameter("grasp_z", 0.02)
        self.declare_parameter("lift_z", 0.10)
        self.declare_parameter("pick_roll", math.pi)
        self.declare_parameter("pick_pitch", 0.0)
        self.declare_parameter("pick_yaw", 0.0)
        self.declare_parameter("gripper_open", 0.0)
        self.declare_parameter("gripper_closed", -1.2)
        self.declare_parameter("move_time", 2.0)
        self.declare_parameter("gripper_time", 1.0)
        self.declare_parameter("cooldown", 2.0)
        self.declare_parameter("pick_once", False)
        self.declare_parameter("place_x", float("nan"))
        self.declare_parameter("place_y", float("nan"))
        self.declare_parameter("place_z", 0.10)

        self.detections_topic = self.get_parameter("detections_topic").value
        self.joint_state_topic = self.get_parameter("joint_state_topic").value
        self.target_classes = self._parse_class_list(self.get_parameter("target_classes").value)
        self.min_confidence = float(self.get_parameter("min_confidence").value)
        self.homography = np.array(self.get_parameter("homography").value, dtype=float).reshape((3, 3))
        self.base_frame = self.get_parameter("base_frame").value
        self.group_name = self.get_parameter("group_name").value
        self.ik_link = self.get_parameter("ik_link").value
        self.arm_joints = list(self.get_parameter("arm_joints").value)
        self.gripper_joint = self.get_parameter("gripper_joint").value
        self.ik_service = self.get_parameter("ik_service").value
        self.ik_timeout = float(self.get_parameter("ik_timeout").value)
        self.avoid_collisions = bool(self.get_parameter("avoid_collisions").value)
        self.arm_action = self.get_parameter("arm_action").value
        self.gripper_action = self.get_parameter("gripper_action").value
        self.approach_z = float(self.get_parameter("approach_z").value)
        self.grasp_z = float(self.get_parameter("grasp_z").value)
        self.lift_z = float(self.get_parameter("lift_z").value)
        self.pick_roll = float(self.get_parameter("pick_roll").value)
        self.pick_pitch = float(self.get_parameter("pick_pitch").value)
        self.pick_yaw = float(self.get_parameter("pick_yaw").value)
        self.gripper_open = float(self.get_parameter("gripper_open").value)
        self.gripper_closed = float(self.get_parameter("gripper_closed").value)
        self.move_time = float(self.get_parameter("move_time").value)
        self.gripper_time = float(self.get_parameter("gripper_time").value)
        self.cooldown = float(self.get_parameter("cooldown").value)
        self.pick_once = bool(self.get_parameter("pick_once").value)
        self.place_x = float(self.get_parameter("place_x").value)
        self.place_y = float(self.get_parameter("place_y").value)
        self.place_z = float(self.get_parameter("place_z").value)

        self.joint_state: Optional[JointState] = None
        self.busy = False
        self.last_pick_time = 0.0
        self.stopped = False

        self.ik_client = self.create_client(GetPositionIK, self.ik_service)
        self.arm_client = ActionClient(self, FollowJointTrajectory, self.arm_action)
        self.gripper_client = ActionClient(self, FollowJointTrajectory, self.gripper_action)

        self.create_subscription(JointState, self.joint_state_topic, self._joint_state_cb, 10)
        self.create_subscription(String, self.detections_topic, self._detections_cb, 10)

        self.get_logger().info(
            f"Pick node ready. Waiting for detections on {self.detections_topic}"
        )

    def _joint_state_cb(self, msg: JointState) -> None:
        self.joint_state = msg

    def _detections_cb(self, msg: String) -> None:
        if self.busy or self.stopped:
            return
        now = time.time()
        if now - self.last_pick_time < self.cooldown:
            return

        try:
            detections = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn("Failed to parse detections JSON")
            return

        target = self._select_detection(detections)
        if target is None:
            return

        thread = threading.Thread(target=self._run_pick, args=(target,), daemon=True)
        thread.start()

    def _select_detection(self, detections: List[dict]) -> Optional[dict]:
        best = None
        best_conf = 0.0
        for det in detections:
            conf = float(det.get("confidence", 0.0))
            if conf < self.min_confidence:
                continue
            class_name = str(det.get("class_name", "")).strip()
            if self.target_classes and class_name.lower() not in self.target_classes:
                continue
            if conf > best_conf:
                best_conf = conf
                best = det
        return best

    def _run_pick(self, det: dict) -> None:
        self.busy = True
        try:
            if self.joint_state is None:
                self.get_logger().warn("No joint_state yet. Skipping pick.")
                return

            center = self._bbox_center(det.get("bbox_xyxy", [0, 0, 0, 0]))
            base_xy = self._pixel_to_base(center)
            if base_xy is None:
                self.get_logger().warn("Homography failed for detection.")
                return

            x, y = base_xy
            self.get_logger().info(
                f"Picking {det.get('class_name', '?')} "
                f"(conf {float(det.get('confidence', 0.0)):.2f}): "
                f"pixel ({center[0]:.0f}, {center[1]:.0f}) -> base ({x:.3f}, {y:.3f})"
            )
            qx, qy, qz, qw = rpy_to_quat(self.pick_roll, self.pick_pitch, self.pick_yaw)

            approach_pose = self._make_pose(x, y, self.approach_z, qx, qy, qz, qw)
            grasp_pose = self._make_pose(x, y, self.grasp_z, qx, qy, qz, qw)
            lift_pose = self._make_pose(x, y, self.lift_z, qx, qy, qz, qw)

            self._wait_for_servers()

            self._send_gripper(self.gripper_open)
            if not self._move_arm_pose(approach_pose):
                return
            if not self._move_arm_pose(grasp_pose):
                return
            self._send_gripper(self.gripper_closed)
            self._move_arm_pose(lift_pose)

            if self._has_place_target():
                place_pose = self._make_pose(
                    self.place_x, self.place_y, self.place_z, qx, qy, qz, qw
                )
                self._move_arm_pose(place_pose)
                self._send_gripper(self.gripper_open)

            self.last_pick_time = time.time()
            if self.pick_once:
                self.stopped = True
                self.get_logger().info("Pick once complete. Stopping further picks.")
        finally:
            self.busy = False

    def _has_place_target(self) -> bool:
        return not (math.isnan(self.place_x) or math.isnan(self.place_y))

    def _wait_future(self, future, timeout_sec: float):
        # This runs in the pick worker thread while the node's executor spins
        # in the main thread; block on the future instead of spinning a
        # second executor on the same node (which corrupts the wait set).
        event = threading.Event()
        future.add_done_callback(lambda _: event.set())
        if not event.wait(timeout_sec):
            future.cancel()
            return None
        try:
            return future.result()
        except Exception as exc:  # service/action call raised
            self.get_logger().warn(f"Future failed: {exc}")
            return None

    def _wait_for_servers(self) -> None:
        if not self.ik_client.service_is_ready():
            self.get_logger().info(f"Waiting for IK service {self.ik_service}")
            self.ik_client.wait_for_service(timeout_sec=5.0)
        if not self.arm_client.server_is_ready():
            self.get_logger().info(f"Waiting for arm controller {self.arm_action}")
            self.arm_client.wait_for_server(timeout_sec=5.0)
        if not self.gripper_client.server_is_ready():
            self.get_logger().info(f"Waiting for gripper controller {self.gripper_action}")
            self.gripper_client.wait_for_server(timeout_sec=5.0)

    def _move_arm_pose(self, pose: PoseStamped) -> bool:
        joints = self._ik_for_pose(pose)
        if joints is None:
            self.get_logger().warn("IK failed for target pose.")
            return False
        return self._send_arm(joints)

    def _ik_for_pose(self, pose: PoseStamped) -> Optional[List[float]]:
        req = GetPositionIK.Request()
        req.ik_request.group_name = self.group_name
        req.ik_request.ik_link_name = self.ik_link
        req.ik_request.pose_stamped = pose
        req.ik_request.avoid_collisions = self.avoid_collisions
        req.ik_request.timeout = Duration(
            sec=int(self.ik_timeout),
            nanosec=int((self.ik_timeout - int(self.ik_timeout)) * 1e9),
        )

        if self.joint_state:
            req.ik_request.robot_state = RobotState(joint_state=self.joint_state)

        res = self._wait_future(self.ik_client.call_async(req), timeout_sec=self.ik_timeout + 2.0)
        if res is None:
            self.get_logger().warn("IK service call failed or timed out.")
            return None
        if res.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().warn(
                f"IK error code: {res.error_code.val} for pose x={pose.pose.position.x:.3f} "
                f"y={pose.pose.position.y:.3f} z={pose.pose.position.z:.3f}"
            )
            return None

        return self._extract_joint_positions(res.solution.joint_state, self.arm_joints)

    def _extract_joint_positions(
        self, joint_state: JointState, names: List[str]
    ) -> Optional[List[float]]:
        if not joint_state.name:
            return None
        lookup = {name: idx for idx, name in enumerate(joint_state.name)}
        positions = []
        for name in names:
            if name not in lookup:
                return None
            positions.append(float(joint_state.position[lookup[name]]))
        return positions

    def _send_arm(self, positions: List[float]) -> bool:
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.arm_joints
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(
            sec=int(self.move_time),
            nanosec=int((self.move_time - int(self.move_time)) * 1e9),
        )
        goal.trajectory.points = [point]
        return self._send_trajectory(self.arm_client, goal, "arm")

    def _send_gripper(self, position: float) -> bool:
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [self.gripper_joint]
        point = JointTrajectoryPoint()
        point.positions = [position]
        point.time_from_start = Duration(
            sec=int(self.gripper_time),
            nanosec=int((self.gripper_time - int(self.gripper_time)) * 1e9),
        )
        goal.trajectory.points = [point]
        return self._send_trajectory(self.gripper_client, goal, "gripper")

    def _send_trajectory(
        self,
        client: ActionClient,
        goal: FollowJointTrajectory.Goal,
        label: str,
    ) -> bool:
        goal_handle = self._wait_future(client.send_goal_async(goal), timeout_sec=5.0)
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn(f"Trajectory rejected for {label}.")
            return False
        result = self._wait_future(goal_handle.get_result_async(), timeout_sec=self.move_time + 10.0)
        if result is None:
            self.get_logger().warn(f"Trajectory result missing for {label}.")
            return False
        return result.result.error_code == 0

    def _pixel_to_base(self, pixel: Tuple[float, float]) -> Optional[Tuple[float, float]]:
        u, v = pixel
        vec = np.array([u, v, 1.0], dtype=float)
        out = self.homography @ vec
        if abs(out[2]) < 1e-6:
            return None
        return float(out[0] / out[2]), float(out[1] / out[2])

    def _bbox_center(self, bbox: List[float]) -> Tuple[float, float]:
        x1, y1, x2, y2 = [float(v) for v in bbox]
        return (x1 + x2) * 0.5, (y1 + y2) * 0.5

    def _make_pose(
        self,
        x: float,
        y: float,
        z: float,
        qx: float,
        qy: float,
        qz: float,
        qw: float,
    ) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self.base_frame
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    @staticmethod
    def _parse_class_list(value: str) -> List[str]:
        if not value:
            return []
        return [item.strip().lower() for item in value.split(",") if item.strip()]


def main() -> None:
    rclpy.init()
    node = PickFromDetections()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
