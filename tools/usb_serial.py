#!/usr/bin/env python3
import serial
import time
import struct
import sys

# Protocol Constants
HEADER = 0xFF
DEVICE_ID = 0xFC
CMD_SERVO_WRITE = 0x10
CMD_SERVO_WRITE6 = 0x1D

class ArmUSB:
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200):
        self.ser = None
        try:
            self.ser = serial.Serial(port, baudrate, timeout=1)
            print(f"Connected to {port}")
        except Exception as e:
            print(f"Error opening serial port {port}: {e}")
            sys.exit(1)

    def close(self):
        if self.ser:
            self.ser.close()

    def _send_packet(self, data):
        """Constructs and sends the packet with checksum."""
        # Packet format: Header(FF) ID(FC) Length Type Data Checksum
        # Length = len(data) + 1 (for Type) + 1 (for Data?? No, length includes Type + Params)
        # Actually structure from Arm_Lib:
        # cmd = [0xFF, 0xFC, Length, Type, Params...]
        # Checksum = sum(Length, Type, Params) & 0xFF
        
        # User's tam.py: [0xFF, 0xFC] + data_bytes + [csum]
        # data_bytes = [length, cmd_type] + payload
        # This matches standard packet structure.
        
        csum = sum(data) & 0xFF
        packet = [HEADER, DEVICE_ID] + data + [csum]
        self.ser.write(bytearray(packet))
        # print(f"Sent: {[hex(x) for x in packet]}")

    def servo_write(self, servo_id, angle, time_ms):
        """
        Control a single servo (1-6).
        angle: 0-180
        time_ms: execution time in ms
        """
        if servo_id < 1 or servo_id > 6:
            print(f"Invalid servo ID: {servo_id}")
            return

        # Calibration mapping from Arm_Lib.py
        # ID 1, 6: (3100-900) * angle / 180 + 900
        # ID 2, 3, 4: Inverted. 180-angle. Then same formula.
        # ID 5: (3700-380) * angle / 270 + 380 (Range 0-270)
        
        pos = 0
        if servo_id in [1, 6]:
            pos = int((3100 - 900) * (angle) / 180 + 900)
        elif servo_id in [2, 3, 4]:
            pos = int((3100 - 900) * (180 - angle) / 180 + 900)
        elif servo_id == 5:
            pos = int((3700 - 380) * (angle) / 270 + 380)

        value_h = (pos >> 8) & 0xFF
        value_l = pos & 0xFF
        time_h = (time_ms >> 8) & 0xFF
        time_l = time_ms & 0xFF

        # CMD_SERVO_WRITE + id is the command type for single servo in Arm_Lib logic 
        # But wait, Arm_Lib says:
        # cmd = [0xFF,0xFC,0x07,0x10 + id,value_H,value_L,time_H,time_L]
        # Length = 7
        # Type = 0x10 + id
        
        length = 7
        cmd_type = CMD_SERVO_WRITE + servo_id
        
        data = [length, cmd_type, value_h, value_l, time_h, time_l]
        self._send_packet(data)

    def servo_write_all(self, angles, time_ms):
        """
        Control all 6 servos at once.
        angles: list of 6 angles [s1, s2, s3, s4, s5, s6]
        """
        if len(angles) != 6:
            return

        # Prepare values
        # Logic from Arm_serial_servo_write6 in Arm_Lib.py
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
            
            values.append((pos >> 8) & 0xFF)
            values.append(pos & 0xFF)

        time_h = (time_ms >> 8) & 0xFF
        time_l = time_ms & 0xFF

        # cmd = [0xFF, 0xFC, 0x11, 0x1D, v1H, v1L, ... v6H, v6L, tH, tL]
        # Length = 0x11 (17 decimal)
        # Type = 0x1D
        
        length = 0x11 # 1 + 12 + 2 + 2? No
        # 0x1D (1 byte) + 6*2 (12 bytes) + 2 (time) = 15 bytes payload?
        # Arm_Lib: cmd len is 18 bytes total including header/checksum.
        # Data part: [0x11, 0x1D, vals..., time...] -> 1+1+12+2 = 16 bytes?
        # Checksum logic: sum([0x11, 0x1d...])
        
        data = [0x11, CMD_SERVO_WRITE6] + values + [time_h, time_l]
        self._send_packet(data)

def interactive_loop():
    arm = ArmUSB(port='/dev/ttyUSB0')
    
    # Initial state
    current_angles = [90.0, 90.0, 90.0, 90.0, 90.0, 90.0]
    last_sent_angles = [90, 90, 90, 90, 90, 90]
    # Motion tuning
    tick_s = 0.02  # 50 Hz update
    hold_timeout_s = 0.25
    speed = 15.0  # deg/s steady speed
    
    print("\n--- USB Serial Control (Gamepad Mimic) ---")
    print("Controls:")
    print("  1/2: Base (Servo 1) Left/Right")
    print("  3/4: Shoulder (Servo 2) Fwd/Back")
    print("  5/6: Elbow (Servo 3) Fwd/Back")
    print("  7/8: Wrist (Servo 4) Fwd/Back")
    print("  9/0: Wrist Roll (Servo 5) Left/Right")
    print("  -/=: Claw (Servo 6) Open/Close")
    print("  r: Reset All to 90")
    print("  q: Quit")
    
    # Center all
    arm.servo_write_all(current_angles, 1000)

    import sys, tty, termios, select
    
    def get_char_nonblocking(fd, timeout_s):
        rlist, _, _ = select.select([fd], [], [], timeout_s)
        if rlist:
            return sys.stdin.read(1)
        return None

    try:
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        tty.setraw(fd)

        current_cmd = None  # (servo_id, direction)
        last_key_time = 0.0

        while True:
            char = get_char_nonblocking(fd, tick_s)

            if char:
                if char == 'q':
                    break
                elif char == 'r':
                    current_angles = [90.0] * 6
                    last_sent_angles = [90] * 6
                    arm.servo_write_all([int(a) for a in current_angles], 1000)
                    current_cmd = None
                    hold_start = None
                    print("\rReset to 90", end="")
                    continue

                # Map key to (servo_id, direction)
                key_map = {
                    '1': (1, +1), '2': (1, -1),
                    '3': (2, +1), '4': (2, -1),
                    '5': (3, +1), '6': (3, -1),
                    '7': (4, +1), '8': (4, -1),
                    '9': (5, +1), '0': (5, -1),
                    '-': (6, +1), '=': (6, -1),
                }

                if char in key_map:
                    new_cmd = key_map[char]
                    if new_cmd != current_cmd:
                        current_cmd = new_cmd
                    last_key_time = time.time()

            # Continue motion while key is held (within timeout)
            now = time.time()
            if current_cmd and (now - last_key_time) > hold_timeout_s:
                current_cmd = None

            if current_cmd:
                servo_id, direction = current_cmd
                idx = servo_id - 1
                step = speed * tick_s * direction

                # Clamp by servo limits
                if servo_id == 5:
                    lo, hi = 0.0, 270.0
                else:
                    lo, hi = 0.0, 180.0

                current_angles[idx] = max(lo, min(hi, current_angles[idx] + step))
                send_angle = int(round(current_angles[idx]))

                if send_angle != last_sent_angles[idx]:
                    arm.servo_write(servo_id, send_angle, 50)
                    last_sent_angles[idx] = send_angle
                    print(f"\rServo {servo_id}: {send_angle}   ", end="")
                
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        arm.close()
        print("\nClosed.")

if __name__ == "__main__":
    interactive_loop()
