#!/usr/bin/env python3
"""Raw-serial hardware check, no ROS: wiggles the base servo
90 -> 110 -> 70 -> 90 over ~6 s. If the arm doesn't move, the problem is
power/cable, not the ROS stack."""
import glob
import sys
import time

import serial

ports = sorted(glob.glob("/dev/ttyUSB*"))
if not ports:
    sys.exit("No /dev/ttyUSB* device — USB not attached (usbipd).")
ser = serial.Serial(ports[0], 115200, timeout=1)


def servo_write(sid, angle, time_ms):
    pos = int((3100 - 900) * angle / 180 + 900)
    data = [0x07, 0x10 + sid, (pos >> 8) & 0xFF, pos & 0xFF,
            (time_ms >> 8) & 0xFF, time_ms & 0xFF]
    ser.write(bytearray([0xFF, 0xFC] + data + [sum(data) & 0xFF]))


print(f"Using {ports[0]} — watch the base: right, left, center...")
for target in (110, 70, 90):
    servo_write(1, target, 1500)
    time.sleep(2)
ser.close()
print("Done. If nothing moved: check DC power + K1.")
