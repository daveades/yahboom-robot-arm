#!/usr/bin/env python3
"""Raw-serial gripper check, no ROS: sweeps ONLY servo 6 (the claw)
through open/close angles with the single-servo command. The arm joints
are not touched.

STOP THE DRIVER FIRST (Ctrl-C in its terminal) - two writers on one
serial port corrupt each other's packets.

Watch the claw and note (a) whether it moves at all and (b) which angle
is open vs closed - that calibrates grip_open/grip_closed.
"""
import glob
import sys
import time

import serial

ports = sorted(glob.glob("/dev/ttyUSB*"))
if not ports:
    sys.exit("No /dev/ttyUSB* device - USB not attached (usbipd).")
ser = serial.Serial(ports[0], 115200, timeout=1)


def servo_write(sid, angle, time_ms):
    pos = int((3100 - 900) * angle / 180 + 900)
    data = [0x07, 0x10 + sid, (pos >> 8) & 0xFF, pos & 0xFF,
            (time_ms >> 8) & 0xFF, time_ms & 0xFF]
    ser.write(bytearray([0xFF, 0xFC] + data + [sum(data) & 0xFF]))


print(f"Using {ports[0]} - watch the CLAW ONLY (arm should not move).")
for angle in (90, 150, 30, 180, 0, 90):
    print(f"  servo 6 -> {angle} deg")
    servo_write(6, angle, 1000)
    time.sleep(2)
ser.close()
print("Done. If the claw never moved: single-servo path is also dead -> "
      "check the claw servo's cable at the expansion board.")
