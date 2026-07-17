#!/usr/bin/env python3
"""Publish frames from a network video stream as a ROS image topic.

Workaround for WSL2/Docker setups where usbipd cannot stream USB webcams
(isochronous transfers unsupported): run a streamer on the Windows side, e.g.

    ffmpeg -f dshow -video_size 640x480 -framerate 30 -i video="<camera name>" \
        -c:v mjpeg -q:v 6 -f mpjpeg -listen 1 http://0.0.0.0:8090/cam.mjpg

then point this node at it:

    python3 stream_camera_node.py --ros-args -p url:=http://<windows-ip>:8090/cam.mjpg

Publishes sensor_msgs/Image (bgr8) on /image_raw, compatible with
dofbot_vision's yolo_detector and color_sorter.
"""
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class StreamCameraNode(Node):
    def __init__(self):
        super().__init__('stream_camera')
        self.declare_parameter('url', 'http://localhost:8090/cam.mjpg')
        self.declare_parameter('topic', '/image_raw')
        self.declare_parameter('frame_id', 'camera')
        self.declare_parameter('reconnect_delay_s', 2.0)

        self.url = self.get_parameter('url').value
        topic = self.get_parameter('topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.reconnect_delay_s = float(self.get_parameter('reconnect_delay_s').value)

        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, topic, 10)
        self.cap = None
        self.frames = 0
        self.last_report = time.monotonic()

        # Short period; cap.read() blocks until the stream delivers a frame,
        # so the actual publish rate follows the stream's frame rate.
        self.timer = self.create_timer(0.005, self.tick)
        self.get_logger().info(f"Reading {self.url} -> {topic}")

    def connect(self):
        self.cap = cv2.VideoCapture(self.url)
        if self.cap.isOpened():
            self.get_logger().info("Stream connected")
        else:
            self.cap.release()
            self.cap = None
            self.get_logger().warn(
                f"Cannot open {self.url}, retrying in {self.reconnect_delay_s}s "
                "(is the streamer running and the port allowed through the firewall?)"
            )
            time.sleep(self.reconnect_delay_s)

    def tick(self):
        if self.cap is None:
            self.connect()
            return
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warn("Stream ended or dropped, reconnecting")
            self.cap.release()
            self.cap = None
            return
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        self.pub.publish(msg)

        self.frames += 1
        now = time.monotonic()
        if now - self.last_report >= 10.0:
            fps = self.frames / (now - self.last_report)
            self.get_logger().info(f"Publishing at {fps:.1f} fps")
            self.frames = 0
            self.last_report = now


def main():
    rclpy.init()
    node = StreamCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.cap is not None:
            node.cap.release()
        node.destroy_node()


if __name__ == '__main__':
    main()
