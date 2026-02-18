#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import serial
import time
import math

# Protocol Constants
HEADER = 0xFF
DEVICE_ID = 0xFC
CMD_SERVO_WRITE = 0x10
CMD_SERVO_WRITE6 = 0x1D

class ArmUSB:
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200, logger=None):
        self.logger = logger
        try:
            self.ser = serial.Serial(port, baudrate, timeout=1)
            if self.logger: self.logger.info(f"Connected to {port}")
        except Exception as e:
            if self.logger: self.logger.error(f"Error opening serial port {port}: {e}")
            self.ser = None

    def close(self):
        if self.ser:
            self.ser.close()

    def _send_packet(self, data):
        if not self.ser: return
        csum = sum(data) & 0xFF
        packet = [HEADER, DEVICE_ID] + data + [csum]
        try:
            self.ser.write(bytearray(packet))
        except Exception as e:
            if self.logger: self.logger.error(f"Serial write failed: {e}")

    def servo_write_all(self, angles, time_ms):
        """
        angles: list of 6 angles in DEGREES
        """
        if len(angles) != 6: return

        values = []
        for i, angle in enumerate(angles):
            sid = i + 1
            pos = 0
            if sid in [1, 6]:
                pos = int((3100 - 900) * angle / 180 + 900)
            elif sid in [2, 3, 4]:
                pos = int((3100 - 900) * (180 - angle) / 180 + 900)
            elif sid == 5:
                pos = int((3700 - 380) * angle / 270 + 380)
            
            # Constrain to valid range 0-4096 roughly or just strict short
            pos = max(0, min(4096, pos))
            
            values.append((pos >> 8) & 0xFF)
            values.append(pos & 0xFF)

        time_h = (time_ms >> 8) & 0xFF
        time_l = time_ms & 0xFF
        
        # Length = 0x11, Type = 0x1D
        data = [0x11, CMD_SERVO_WRITE6] + values + [time_h, time_l]
        self._send_packet(data)

class DofbotDriver(Node):
    def __init__(self):
        super().__init__('dofbot_driver')
        
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('max_speed_deg_s', 120.0)
        self.declare_parameter('min_time_ms', 50)
        self.declare_parameter('max_time_ms', 1000)
        self.declare_parameter('gripper_range_rad', 1.54)  # URDF grip_joint lower limit magnitude
        self.declare_parameter('gripper_open_deg', 0.0)    # Servo angle at fully open
        self.declare_parameter('gripper_closed_deg', 180.0)  # Servo angle at fully closed
        self.declare_parameter('min_delta_deg', 0.5)  # ignore tiny changes to prevent jitter
        self.declare_parameter('startup_time_ms', 1200)  # slow initial move after controller start
        self.declare_parameter('startup_moves', 1)
        port = self.get_parameter('port').value
        self.max_speed_deg_s = float(self.get_parameter('max_speed_deg_s').value)
        self.min_time_ms = int(self.get_parameter('min_time_ms').value)
        self.max_time_ms = int(self.get_parameter('max_time_ms').value)
        self.gripper_range_rad = float(self.get_parameter('gripper_range_rad').value)
        self.gripper_open_deg = float(self.get_parameter('gripper_open_deg').value)
        self.gripper_closed_deg = float(self.get_parameter('gripper_closed_deg').value)
        self.min_delta_deg = float(self.get_parameter('min_delta_deg').value)
        self.startup_time_ms = int(self.get_parameter('startup_time_ms').value)
        self.startup_moves = int(self.get_parameter('startup_moves').value)
        
        self.arm = ArmUSB(port, logger=self.get_logger())
        
        # Joint names matching the URDF
        self.joint_names = [
            'arm_joint1', 'arm_joint2', 'arm_joint3', 
            'arm_joint4', 'arm_joint5', 'grip_joint' # Note: Gripper is 6th
        ]
        
        # Current state (Degrees)
        self.current_angles = [90.0] * 6
        self.current_angles[5] = self.gripper_closed_deg  # gripper starts closed
        self.last_sent_angles = self.current_angles.copy()
        self.startup_moves_remaining = self.startup_moves
        
        self.sub = self.create_subscription(
            JointState,
            'target_joints',
            self.joint_callback,
            10
        )
        
        self.pub = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.1, self.publish_state)
        
        self.get_logger().info("Dofbot Driver Initialized")

    def joint_callback(self, msg):
        target_indices = []
        target_values = [] # In degrees
        prev_angles = self.current_angles.copy()
        
        # Simple mapping assuming order or names
        # If names provided, map them
        if msg.name:
            for name, pos in zip(msg.name, msg.position):
                if name in self.joint_names:
                    idx = self.joint_names.index(name)
                    deg = math.degrees(pos)
                    # Mapping: 
                    # URDF 0 is center? 
                    # Driver 90 is center.
                    # Need to check URDF limits. 
                    # joint1: -1.57 to 1.57 (Radians). 0 is center.
                    # Arm: 0-180. 90 is center.
                    # So: Driver_Deg = (Rad * 180/PI) + 90
                    
                    if idx == 5:  # gripper: map 0 (open) to -range (closed)
                        if self.gripper_range_rad > 0:
                            t = (pos + self.gripper_range_rad) / self.gripper_range_rad  # 0..1
                            driver_angle = self.gripper_closed_deg + t * (self.gripper_open_deg - self.gripper_closed_deg)
                        else:
                            driver_angle = self.gripper_open_deg
                    else:
                        driver_angle = deg + 90
                    self.current_angles[idx] = driver_angle
        else:
            # Assume ordered 1-6
            for i, pos in enumerate(msg.position):
                if i < 6:
                    if i == 5:  # gripper
                        if self.gripper_range_rad > 0:
                            t = (pos + self.gripper_range_rad) / self.gripper_range_rad  # 0..1
                            driver_angle = self.gripper_closed_deg + t * (self.gripper_open_deg - self.gripper_closed_deg)
                        else:
                            driver_angle = self.gripper_open_deg
                    else:
                        driver_angle = math.degrees(pos) + 90
                    self.current_angles[i] = driver_angle

        # Send to arm (ignore tiny changes to prevent jitter)
        max_delta = max(abs(c - p) for c, p in zip(self.current_angles, self.last_sent_angles))
        if max_delta < self.min_delta_deg:
            return
        if self.max_speed_deg_s > 0:
            time_ms = int((max_delta / self.max_speed_deg_s) * 1000.0)
        else:
            time_ms = self.min_time_ms
        time_ms = max(self.min_time_ms, min(self.max_time_ms, time_ms))
        if self.startup_moves_remaining > 0:
            time_ms = max(time_ms, self.startup_time_ms)
            self.startup_moves_remaining -= 1
        self.arm.servo_write_all(self.current_angles, time_ms)
        self.last_sent_angles = self.current_angles.copy()

    def publish_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        
        # Convert Driver(0-180) back to Rad(-90 to 90), except gripper which is 0..90 (0 = closed)
        rads = []
        for i, angle in enumerate(self.last_sent_angles):
            if i == 5:  # gripper: map servo degrees back to [-range, 0]
                denom = (self.gripper_open_deg - self.gripper_closed_deg)
                if self.gripper_range_rad > 0 and abs(denom) > 1e-6:
                    t = (angle - self.gripper_closed_deg) / denom  # 0..1
                    rad = (t * self.gripper_range_rad) - self.gripper_range_rad
                else:
                    rad = 0.0
            else:
                rad = math.radians(angle - 90)
            rads.append(rad)
            
        msg.position = rads
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DofbotDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.arm.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
