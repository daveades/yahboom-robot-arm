#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time
import math

class AutomationDemo(Node):
    def __init__(self):
        super().__init__('automation_demo')
        self.pub = self.create_publisher(JointState, '/target_joints', 10)
        self.timer = self.create_timer(2.0, self.move_sequence)
        self.step = 0
        
        # Define poses (Degrees: 0 is center/up, 90 is max +ve, -90 is max -ve)
        # Note: Our driver maps 0.0 rad to 90 deg physical (Center).
        # Driver Input: Radians relative to center.
        # Driver Node Logic: Angle_Deg = Degrees(Input_Rad) + 90
        # So Input 0.0 -> 90 Deg (Up/Center)
        # Input -1.57 (-90 deg) -> 0 Deg (Right/Flat)
        # Input 1.57 (90 deg) -> 180 Deg (Left/Flat)
        
        self.poses = [
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],       # Home
            [-0.5, 0.5, -0.5, 0.0, 0.0, 0.0],     # Pose A
            [0.5, 0.5, -0.5, 0.0, 0.0, 0.0],      # Pose B
            [0.0, 0.8, 0.0, 0.0, 1.57, 0.0],      # Pose C (Gripper Rotate)
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],       # Home
        ]
        self.get_logger().info("Automation Node Started")

    def move_sequence(self):
        # Loop through poses
        current_pose = self.poses[self.step % len(self.poses)]
        
        msg = JointState()
        msg.position = current_pose
        self.pub.publish(msg)
        
        self.get_logger().info(f"Moving to Pose {self.step % len(self.poses)}")
        self.step += 1

def main(args=None):
    rclpy.init(args=args)
    node = AutomationDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
