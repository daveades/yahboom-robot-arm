#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo


class FakeCameraInfo(Node):
    def __init__(self):
        super().__init__("fake_camera_info")
        self.declare_parameter("frame_id", "camera_frame")
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("fx", 600.0)
        self.declare_parameter("fy", 600.0)
        self.declare_parameter("cx", 320.0)
        self.declare_parameter("cy", 240.0)
        self.declare_parameter("distortion", [0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("publish_topic", "/camera/camera_info")
        self.declare_parameter("rate_hz", 10.0)

        self.frame_id = self.get_parameter("frame_id").value
        self.width = int(self.get_parameter("width").value)
        self.height = int(self.get_parameter("height").value)
        self.fx = float(self.get_parameter("fx").value)
        self.fy = float(self.get_parameter("fy").value)
        self.cx = float(self.get_parameter("cx").value)
        self.cy = float(self.get_parameter("cy").value)
        self.distortion = list(self.get_parameter("distortion").value)
        self.publish_topic = self.get_parameter("publish_topic").value
        self.rate_hz = float(self.get_parameter("rate_hz").value)

        self.pub = self.create_publisher(CameraInfo, self.publish_topic, 10)
        self.timer = self.create_timer(1.0 / self.rate_hz, self.on_timer)

    def on_timer(self):
        msg = CameraInfo()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.width = self.width
        msg.height = self.height
        msg.k = [
            self.fx, 0.0, self.cx,
            0.0, self.fy, self.cy,
            0.0, 0.0, 1.0
        ]
        msg.d = self.distortion
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeCameraInfo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
