#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
import serial
import time
import math
import os
import glob
import threading

# Protocol Constants
HEADER = 0xFF
DEVICE_ID = 0xFC
CMD_SERVO_WRITE = 0x10
CMD_SERVO_WRITE6 = 0x1D

class ArmUSB:
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200, logger=None):
        self.logger = logger
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.last_open_attempt = 0.0
        self.lock = threading.Lock()
        self._open()

    def _open(self):
        self.last_open_attempt = time.monotonic()
        port = self.port
        if not os.path.exists(port):
            # after a USB drop the device often re-enumerates under a new
            # name (ttyUSB0 -> ttyUSB1); take whatever ttyUSB is present
            candidates = sorted(glob.glob('/dev/ttyUSB*'))
            if candidates:
                port = candidates[0]
        try:
            self.ser = serial.Serial(port, self.baudrate, timeout=1)
            if self.logger: self.logger.info(f"Connected to {port}")
        except Exception as e:
            if self.logger: self.logger.error(f"Error opening serial port {port}: {e}")
            self.ser = None

    def close(self):
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass

    def _send_packet(self, data):
        # The USB device can drop mid-session (cable/brownout + usbipd);
        # keep retrying the port at most once per second so a re-attach
        # brings the arm back without restarting the driver.
        with self.lock:
            if not self.ser:
                if time.monotonic() - self.last_open_attempt < 1.0:
                    return
                self._open()
                if not self.ser:
                    return
            csum = sum(data) & 0xFF
            packet = [HEADER, DEVICE_ID] + data + [csum]
            try:
                self.ser.write(bytearray(packet))
            except Exception as e:
                if self.logger: self.logger.error(f"Serial write failed: {e} — will reconnect")
                self.close()
                self.ser = None

    @staticmethod
    def _servo_pos(sid, angle):
        if sid in [1, 6]:
            pos = int((3100 - 900) * angle / 180 + 900)
        elif sid in [2, 3, 4]:
            pos = int((3100 - 900) * (180 - angle) / 180 + 900)
        elif sid == 5:
            pos = int((3700 - 380) * angle / 270 + 380)
        else:
            pos = 0
        return max(0, min(4096, pos))

    def servo_write_all(self, angles, time_ms):
        """
        angles: list of 6 angles in DEGREES
        """
        if len(angles) != 6: return

        values = []
        for i, angle in enumerate(angles):
            pos = self._servo_pos(i + 1, angle)
            values.append((pos >> 8) & 0xFF)
            values.append(pos & 0xFF)

        time_h = (time_ms >> 8) & 0xFF
        time_l = time_ms & 0xFF

        # Length = 0x11, Type = 0x1D
        data = [0x11, CMD_SERVO_WRITE6] + values + [time_h, time_l]
        self._send_packet(data)

    def servo_write(self, sid, angle, time_ms):
        """Write a single servo (id 1..6, angle in DEGREES)."""
        pos = self._servo_pos(sid, angle)
        data = [
            0x07,
            CMD_SERVO_WRITE + sid,
            (pos >> 8) & 0xFF, pos & 0xFF,
            (time_ms >> 8) & 0xFF, time_ms & 0xFF,
        ]
        self._send_packet(data)

class DofbotDriver(Node):
    """Serial driver that executes MoveIt trajectories natively.

    Exposes the two FollowJointTrajectory action servers MoveIt's simple
    controller manager expects (arm_controller + gripper_controller) and
    plays trajectories back as timed waypoints: one serial command per
    segment, with the segment duration as the servo's move time, so the
    firmware does the interpolation it was designed for. This replaces
    the old path (ros2_control JTC streaming positions at 100 Hz), which
    restarted the servo's accel/decel ramp every packet and made motion
    pulse and lag.
    """

    ARM_JOINTS = ['arm_joint1', 'arm_joint2', 'arm_joint3', 'arm_joint4', 'arm_joint5']
    GRIPPER_JOINTS = ['grip_joint']

    def __init__(self):
        super().__init__('dofbot_driver')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('max_speed_deg_s', 120.0)
        self.declare_parameter('min_time_ms', 10)
        self.declare_parameter('max_time_ms', 1000)
        self.declare_parameter('gripper_range_rad', 1.54)  # URDF grip_joint lower limit magnitude
        self.declare_parameter('gripper_open_deg', 0.0)    # Servo angle at fully open
        self.declare_parameter('gripper_closed_deg', 180.0)  # Servo angle at fully closed
        self.declare_parameter('min_delta_deg', 0.5)  # ignore tiny changes to prevent jitter
        self.declare_parameter('startup_time_ms', 4000)  # duration of the slow startup sync sweep
        self.declare_parameter('min_segment_ms', 100)  # waypoint downsampling floor
        self.declare_parameter('settle_ms', 150)  # pause after the last waypoint
        port = self.get_parameter('port').value
        self.max_speed_deg_s = float(self.get_parameter('max_speed_deg_s').value)
        self.min_time_ms = int(self.get_parameter('min_time_ms').value)
        self.max_time_ms = int(self.get_parameter('max_time_ms').value)
        self.gripper_range_rad = float(self.get_parameter('gripper_range_rad').value)
        self.gripper_open_deg = float(self.get_parameter('gripper_open_deg').value)
        self.gripper_closed_deg = float(self.get_parameter('gripper_closed_deg').value)
        self.min_delta_deg = float(self.get_parameter('min_delta_deg').value)
        self.startup_time_ms = int(self.get_parameter('startup_time_ms').value)
        self.min_segment_ms = int(self.get_parameter('min_segment_ms').value)
        self.settle_ms = int(self.get_parameter('settle_ms').value)

        self.arm = ArmUSB(port, logger=self.get_logger())

        # Joint names matching the URDF
        self.joint_names = self.ARM_JOINTS + self.GRIPPER_JOINTS

        # Current state (Degrees)
        self.current_angles = [90.0] * 6
        self.current_angles[5] = self.gripper_closed_deg  # gripper starts closed
        self.last_sent_angles = self.current_angles.copy()
        self.last_send_time = None
        self.started = False  # startup sync sweep done?

        # One motion at a time: actions serialize on this, and the legacy
        # streaming topic is dropped while a trajectory is executing.
        self.motion_lock = threading.Lock()

        self.arm_action = ActionServer(
            self, FollowJointTrajectory,
            'arm_controller/follow_joint_trajectory',
            execute_callback=self.execute_arm,
            cancel_callback=self._cancel_cb,
            callback_group=ReentrantCallbackGroup(),
        )
        self.gripper_action = ActionServer(
            self, FollowJointTrajectory,
            'gripper_controller/follow_joint_trajectory',
            execute_callback=self.execute_gripper,
            cancel_callback=self._cancel_cb,
            callback_group=ReentrantCallbackGroup(),
        )

        # Legacy streaming interface (kept for tools that publish targets
        # directly; unused by the MoveIt path).
        self.sub = self.create_subscription(
            JointState,
            'target_joints',
            self.joint_callback,
            10
        )

        self.pub = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.05, self.publish_state)

        self.get_logger().info("Dofbot Driver Initialized (native trajectory execution)")

    # ---------- unit conversion ----------

    def _rad_to_driver_deg(self, idx, pos):
        """URDF joint position (rad) -> servo angle (deg) for joint index."""
        if idx == 5:  # gripper: map 0 (open) .. -range (closed)
            if self.gripper_range_rad > 0:
                t = (pos + self.gripper_range_rad) / self.gripper_range_rad  # 0..1
                return self.gripper_closed_deg + t * (self.gripper_open_deg - self.gripper_closed_deg)
            return self.gripper_open_deg
        # URDF 0 rad is center; servo 90 deg is center.
        return math.degrees(pos) + 90.0

    # ---------- trajectory execution ----------

    def _cancel_cb(self, _goal_handle):
        return CancelResponse.ACCEPT

    def execute_arm(self, goal_handle):
        return self._execute_trajectory(goal_handle, self.ARM_JOINTS)

    def execute_gripper(self, goal_handle):
        return self._execute_trajectory(goal_handle, self.GRIPPER_JOINTS)

    def _execute_trajectory(self, goal_handle, allowed_joints):
        result = FollowJointTrajectory.Result()
        traj = goal_handle.request.trajectory

        idx_map = []
        for name in traj.joint_names:
            if name not in allowed_joints:
                result.error_code = FollowJointTrajectory.Result.INVALID_JOINTS
                result.error_string = f'joint {name} not handled by this controller'
                goal_handle.abort()
                return result
            idx_map.append(self.joint_names.index(name))

        if not traj.points:
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            goal_handle.succeed()
            return result

        # Downsample: the firmware interpolates within a segment, so dense
        # waypoints only add serial traffic. Keep points at least
        # min_segment_ms apart, plus always the final point.
        segments = []
        last_kept_t = None
        for k, pt in enumerate(traj.points):
            t = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
            is_last = (k == len(traj.points) - 1)
            if last_kept_t is None or is_last or (t - last_kept_t) * 1000.0 >= self.min_segment_ms:
                segments.append((t, pt))
                last_kept_t = t

        total_s = segments[-1][0]
        self.get_logger().info(
            f"Trajectory goal: {len(traj.joint_names)} joints "
            f"({', '.join(traj.joint_names)}), {len(traj.points)} points over "
            f"{total_s:.2f}s -> {len(segments)} segments")

        with self.motion_lock:
            # Startup sync: on the very first motion, glide ALL servos from
            # wherever the arm physically is to the trajectory start over
            # startup_time_ms, so a pose mismatch never causes a jump.
            if not self.started:
                t0, pt0 = segments[0]
                start_angles = self.current_angles.copy()
                for j, idx in enumerate(idx_map):
                    start_angles[idx] = self._rad_to_driver_deg(idx, pt0.positions[j])
                self.get_logger().info(
                    f"Startup sync: easing arm to trajectory start over {self.startup_time_ms} ms")
                self.arm.servo_write_all(start_angles, self.startup_time_ms)
                self.current_angles = start_angles.copy()
                self.last_sent_angles = start_angles.copy()
                time.sleep(self.startup_time_ms / 1000.0)
                self.started = True

            prev_t = None
            for (t, pt) in segments:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                    self.get_logger().info("Trajectory canceled; arm holds last waypoint")
                    return result

                angles = self.current_angles.copy()
                for j, idx in enumerate(idx_map):
                    angles[idx] = self._rad_to_driver_deg(idx, pt.positions[j])

                dt = (t - prev_t) if prev_t is not None else t
                # Respect the hardware speed cap even if the plan is faster.
                max_delta = max(abs(angles[i] - self.last_sent_angles[i]) for i in range(6))
                if self.max_speed_deg_s > 0:
                    dt = max(dt, max_delta / self.max_speed_deg_s)
                dt_ms = int(max(self.min_time_ms, dt * 1000.0))

                # Use the all-six command so the firmware moves every servo
                # together over the same duration. Per-servo timed writes
                # execute one after another on the board (a new command
                # preempts the previous joint's ramp), which makes joints
                # blitz sequentially instead of moving in coordination.
                # Beliefs for untouched joints are grounded by the startup
                # sync, so re-asserting them here is safe.
                self.arm.servo_write_all(angles, dt_ms)
                self.last_sent_angles = angles.copy()
                self.current_angles = angles
                prev_t = t
                if dt_ms > 0:
                    time.sleep(dt_ms / 1000.0)

            if self.settle_ms > 0:
                time.sleep(self.settle_ms / 1000.0)

        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        goal_handle.succeed()
        return result

    # ---------- legacy streaming interface ----------

    def joint_callback(self, msg):
        # Drop streamed targets while a trajectory action is executing.
        if not self.motion_lock.acquire(blocking=False):
            return
        try:
            self._handle_stream(msg)
        finally:
            self.motion_lock.release()

    def _handle_stream(self, msg):
        if msg.name:
            for name, pos in zip(msg.name, msg.position):
                if name in self.joint_names:
                    idx = self.joint_names.index(name)
                    self.current_angles[idx] = self._rad_to_driver_deg(idx, pos)
        else:
            # Assume ordered 1-6
            for i, pos in enumerate(msg.position):
                if i < 6:
                    self.current_angles[i] = self._rad_to_driver_deg(i, pos)

        # Startup sync (same rationale as the action path).
        now = time.monotonic()
        if not self.started:
            self.arm.servo_write_all(self.current_angles, self.startup_time_ms)
            self.last_sent_angles = self.current_angles.copy()
            self.last_send_time = now
            self.started = True
            self.get_logger().info(
                f"Startup sync: easing arm to commanded pose over "
                f"{self.startup_time_ms} ms"
            )
            time.sleep(self.startup_time_ms / 1000.0)
            return

        # Send ONLY the servos whose target changed.
        changed = [
            i for i, (c, p) in enumerate(zip(self.current_angles, self.last_sent_angles))
            if abs(c - p) >= self.min_delta_deg
        ]
        if not changed:
            return
        max_delta = max(
            abs(self.current_angles[i] - self.last_sent_angles[i]) for i in changed
        )
        # Pace streamed commands by the real time since the previous packet.
        if self.last_send_time is None:
            elapsed_ms = float(self.max_time_ms)
        else:
            elapsed_ms = (now - self.last_send_time) * 1000.0
        if self.max_speed_deg_s > 0:
            speed_floor_ms = (max_delta / self.max_speed_deg_s) * 1000.0
        else:
            speed_floor_ms = float(self.min_time_ms)
        time_ms = int(max(
            self.min_time_ms,
            speed_floor_ms,
            min(elapsed_ms, self.max_time_ms),
        ))
        for i in changed:
            self.arm.servo_write(i + 1, self.current_angles[i], time_ms)
            self.last_sent_angles[i] = self.current_angles[i]
        self.last_send_time = now

    # ---------- state publication ----------

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
    # Trajectory execution sleeps inside the action callback; a multi-threaded
    # executor keeps joint_states publishing and cancel requests responsive.
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.arm.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
