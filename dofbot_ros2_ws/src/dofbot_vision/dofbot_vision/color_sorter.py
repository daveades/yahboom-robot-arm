#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

class ColorSorter(Node):
    def __init__(self):
        super().__init__('color_sorter')
        
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        self.pub = self.create_publisher(JointState, '/target_joints', 10)
        
        # State Machine
        self.state = "INIT" 
        self.target_color = None
        self.timer = self.create_timer(0.1, self.loop)
        
        # Poses (Radians relative to Center)
        # Note: Driver expects inputs where 0.0 is center (90 deg).
        # Adjust these based on real world testing!
        self.POSE_OBSERVE = [0.0, -0.5, -0.5, 0.0, 0.0, 0.0] 
        self.POSE_LEFT_BIN = [1.57, 0.0, 0.0, 0.0, 0.0, 0.0] # 90 deg Left
        self.POSE_RIGHT_BIN = [-1.57, 0.0, 0.0, 0.0, 0.0, 0.0] # 90 deg Right
        
        # Color Thresholds (HSV)
        self.lower_red = np.array([0, 100, 100])
        self.upper_red = np.array([10, 255, 255])
        self.lower_blue = np.array([100, 100, 100])
        self.upper_blue = np.array([124, 255, 255])
        
        self.latest_image = None
        self.action_timer = 0
        
        self.get_logger().info("Color Sorter initialized. Waiting for camera...")

    def image_callback(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"CV Bridge Error: {e}")

    def loop(self):
        if self.state == "INIT":
            # Move to Observe Pose
            self.publish_joints(self.POSE_OBSERVE)
            if self.wait_seconds(2):
                self.state = "SCANNING"
                
        elif self.state == "SCANNING":
            if self.latest_image is None: return
            
            # Detect Color
            color = self.detect_color(self.latest_image)
            if color:
                self.get_logger().info(f"detected {color}")
                self.target_color = color
                self.state = "PICKING"
                self.action_timer = time.time()
                
        elif self.state == "PICKING":
            # Simplified: Move Down Blindly (In reality, use coords)
            # 1. Open Gripper
            # 2. Move Down
            # 3. Close Gripper
            
            # For this demo, we just wave to indicate detection
            if self.target_color == "RED":
                self.publish_joints(self.POSE_LEFT_BIN)
            elif self.target_color == "BLUE":
                self.publish_joints(self.POSE_RIGHT_BIN)
                
            if self.wait_seconds(3):
                self.state = "INIT"
                
    def detect_color(self, img):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        
        # Check Red
        mask = cv2.inRange(hsv, self.lower_red, self.upper_red)
        if cv2.countNonZero(mask) > 1000:
            return "RED"
            
        # Check Blue
        mask = cv2.inRange(hsv, self.lower_blue, self.upper_blue)
        if cv2.countNonZero(mask) > 1000:
            return "BLUE"
            
        return None

    def publish_joints(self, pose):
        msg = JointState()
        msg.position = pose
        self.pub.publish(msg)
        
    def wait_seconds(self, seconds):
        if time.time() - self.action_timer > seconds:
            self.action_timer = time.time()
            return True
        return False

def main(args=None):
    rclpy.init(args=args)
    node = ColorSorter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
