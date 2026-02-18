#!/usr/bin/env python3
import math
from typing import Optional

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np

try:
    from tf2_ros import TransformBroadcaster
    from geometry_msgs.msg import TransformStamped
    HAVE_TF = True
except Exception:
    HAVE_TF = False


DICT_MAP = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
}


class ArucoDetector(Node):
    def __init__(self):
        super().__init__("aruco_detector")

        self.declare_parameter("image_topic", "/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("marker_size", 0.03)  # meters
        self.declare_parameter("marker_id", -1)  # -1 = any
        self.declare_parameter("dictionary", "DICT_4X4_50")
        self.declare_parameter("camera_frame", "camera_frame")
        self.declare_parameter("publish_tf", False)
        self.declare_parameter("tf_prefix", "aruco_")

        self.image_topic = self.get_parameter("image_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value
        self.marker_size = float(self.get_parameter("marker_size").value)
        self.marker_id = int(self.get_parameter("marker_id").value)
        self.dictionary_name = self.get_parameter("dictionary").value
        self.camera_frame = self.get_parameter("camera_frame").value
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.tf_prefix = self.get_parameter("tf_prefix").value

        self.bridge = CvBridge()

        self.camera_matrix: Optional[np.ndarray] = None
        self.dist_coeffs: Optional[np.ndarray] = None

        self.aruco_dict = self._get_dictionary(self.dictionary_name)
        # Support both old and new OpenCV ArUco APIs
        if hasattr(cv2.aruco, "DetectorParameters"):
            self.aruco_params = cv2.aruco.DetectorParameters()
        else:
            self.aruco_params = cv2.aruco.DetectorParameters_create()

        self.pose_pub = self.create_publisher(PoseStamped, "aruco_pose", 10)
        self.image_sub = self.create_subscription(Image, self.image_topic, self.image_cb, 10)
        self.info_sub = self.create_subscription(CameraInfo, self.camera_info_topic, self.info_cb, 10)

        if self.publish_tf and HAVE_TF:
            self.tf_broadcaster = TransformBroadcaster(self)
        else:
            self.tf_broadcaster = None

        self.get_logger().info("Aruco detector started")

    def _get_dictionary(self, name: str):
        if name in DICT_MAP:
            return cv2.aruco.getPredefinedDictionary(DICT_MAP[name])
        self.get_logger().warn(f"Unknown dictionary '{name}', falling back to DICT_4X4_50")
        return cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

    def info_cb(self, msg: CameraInfo):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            d = list(msg.d)
            if len(d) == 0:
                d = [0.0, 0.0, 0.0, 0.0, 0.0]
            self.dist_coeffs = np.array(d, dtype=np.float64).reshape(-1)
            if msg.header.frame_id:
                self.camera_frame = msg.header.frame_id
            self.get_logger().info("Camera info received")

    def image_cb(self, msg: Image):
        if self.camera_matrix is None or self.dist_coeffs is None:
            return

        try:
            image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        if hasattr(cv2.aruco, "ArucoDetector"):
            detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
            corners, ids, _ = detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, self.aruco_dict, parameters=self.aruco_params
            )

        if ids is None or len(ids) == 0:
            return

        ids = ids.flatten()

        # Choose marker
        idx = 0
        if self.marker_id >= 0:
            matches = np.where(ids == self.marker_id)[0]
            if len(matches) == 0:
                return
            idx = int(matches[0])

        # Estimate pose for all markers, then select by index
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, self.marker_size, self.camera_matrix, self.dist_coeffs
        )
        rvec = rvecs[idx][0]
        tvec = tvecs[idx][0]

        # Convert to PoseStamped
        pose = PoseStamped()
        pose.header.stamp = msg.header.stamp
        pose.header.frame_id = self.camera_frame
        pose.pose.position.x = float(tvec[0])
        pose.pose.position.y = float(tvec[1])
        pose.pose.position.z = float(tvec[2])

        # Convert rvec to quaternion
        rot_mat, _ = cv2.Rodrigues(rvec)
        qw = math.sqrt(max(0.0, 1.0 + rot_mat[0, 0] + rot_mat[1, 1] + rot_mat[2, 2])) / 2.0
        qx = math.sqrt(max(0.0, 1.0 + rot_mat[0, 0] - rot_mat[1, 1] - rot_mat[2, 2])) / 2.0
        qy = math.sqrt(max(0.0, 1.0 - rot_mat[0, 0] + rot_mat[1, 1] - rot_mat[2, 2])) / 2.0
        qz = math.sqrt(max(0.0, 1.0 - rot_mat[0, 0] - rot_mat[1, 1] + rot_mat[2, 2])) / 2.0
        qx = math.copysign(qx, rot_mat[2, 1] - rot_mat[1, 2])
        qy = math.copysign(qy, rot_mat[0, 2] - rot_mat[2, 0])
        qz = math.copysign(qz, rot_mat[1, 0] - rot_mat[0, 1])

        pose.pose.orientation.x = float(qx)
        pose.pose.orientation.y = float(qy)
        pose.pose.orientation.z = float(qz)
        pose.pose.orientation.w = float(qw)

        self.pose_pub.publish(pose)

        if self.tf_broadcaster is not None:
            tf_msg = TransformStamped()
            tf_msg.header = pose.header
            tf_msg.child_frame_id = f"{self.tf_prefix}{int(ids[idx])}"
            tf_msg.transform.translation.x = pose.pose.position.x
            tf_msg.transform.translation.y = pose.pose.position.y
            tf_msg.transform.translation.z = pose.pose.position.z
            tf_msg.transform.rotation = pose.pose.orientation
            self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
