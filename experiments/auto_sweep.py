#!/usr/bin/env python3
import time
import sys
import termios
import tty
import select

from usb_serial import ArmUSB


def read_key_nonblocking(fd, timeout_s):
    rlist, _, _ = select.select([fd], [], [], timeout_s)
    if rlist:
        return sys.stdin.read(1)
    return None


def main():
    arm = ArmUSB(port="/dev/ttyUSB0")

    # Motion tuning: use time-based moves for smooth motion
    move_time_ms = 1200

    # Servo order: top to bottom
    servo_order = [6, 5, 4, 3, 2, 1]

    # Reset all servos to 90 before starting
    print("Resetting all servos to 90...")
    arm.servo_write_all([90, 90, 90, 90, 90, 90], 1000)
    time.sleep(1.0)

    print("Press Enter to start.")
    input()

    print("Running auto sweep. Space = pause/resume, q = quit.")

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    tty.setraw(fd)

    paused = False

    try:
        for servo_id in servo_order:
            if servo_id == 5:
                low, high = 0, 180  # keep within 0-180 for this sweep
            else:
                low, high = 0, 180

            sequence = [90, low, 90, high, 90]
            for target in sequence:
                while True:
                    key = read_key_nonblocking(fd, 0.05)
                    if key == 'q':
                        return
                    if key == ' ':
                        paused = not paused
                        print("\rPaused" if paused else "\rResumed", end="")
                    if not paused:
                        break

                arm.servo_write(servo_id, target, move_time_ms)
                print(f"\rServo {servo_id}: {target}   ", end="")
                time.sleep(move_time_ms / 1000.0)

            print("")  # newline between servos

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        arm.close()
        print("\nClosed.")


if __name__ == "__main__":
    main()
