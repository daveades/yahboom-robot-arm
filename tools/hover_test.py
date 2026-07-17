#!/usr/bin/env python3
"""Hover the gripper over chess-board squares to calibrate the board->base
transform.

Board convention (looking down at the table, robot base at origin,
x forward, y left):

  * ``--a1 X Y``     center of square a1 in base frame, meters
  * ``--square S``   square size in meters (e.g. 0.025)
  * ``--yaw DEG``    direction files run (a->h). 0 deg = +x (away from
                     robot), 90 deg = +y (to the robot's left).
  * ``--mirror``     flip the rank direction (1->8) to the other side of
                     the file axis. Use this if the hover lands mirrored.

Ranks (1->8) run 90 deg counterclockwise from the file direction unless
--mirror is given.

The script prints the computed (x, y) for every requested square, asks for
confirmation, then hovers over each square in turn at --z, pausing for
Enter between squares so you can check alignment with a ruler or by eye.

Uses the same path as pick_from_detections: /compute_ik (position-only IK
must be active in move_group) and /arm_controller/follow_joint_trajectory.

Example:
  python3 tools/hover_test.py --a1 0.16 -0.09 --square 0.025 --yaw 90
"""

import argparse
import math
import sys

from board_config import add_board_args, resolve as resolve_board, square_to_xy

try:  # ROS only needed for actual motion; --check-only works without it
    import rclpy
    from arm_client import ArmClient
    HAVE_ROS = True
except ImportError:
    HAVE_ROS = False

DEFAULT_SQUARES = ["a1", "h1", "h8", "a8", "d4", "e5"]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    add_board_args(parser)
    parser.add_argument("--z", type=float, default=0.12,
                        help="hover height in meters (default 0.12)")
    parser.add_argument("--move-time", type=float, default=3.0,
                        help="minimum seconds per move (default 3.0)")
    parser.add_argument("--max-speed", type=float, default=0.5,
                        help="peak joint speed in rad/s; long swings are "
                             "slowed to respect this (default 0.5)")
    parser.add_argument("--gripper", type=float, default=None, metavar="POS",
                        help="close gripper to this grip_joint position first "
                             "(e.g. -1.0) so the tips act as a pointer")
    parser.add_argument("--check-only", action="store_true",
                        help="print square coordinates and exit, no motion")
    parser.add_argument("squares", nargs="*", default=None,
                        help=f"squares to visit (default: {' '.join(DEFAULT_SQUARES)})")
    args = parser.parse_args()

    squares = args.squares or DEFAULT_SQUARES
    a1, square, yaw, mirror = resolve_board(args)

    print(f"\nBoard: a1=({a1[0]:.3f}, {a1[1]:.3f})  square={square*1000:.0f}mm  "
          f"yaw={yaw:.0f}deg  mirror={mirror}  hover z={args.z:.3f}\n")
    targets = []
    for sq in squares:
        x, y = square_to_xy(sq, a1, square, yaw, mirror)
        dist = math.hypot(x, y)
        ang = math.degrees(math.atan2(y, x))
        note = ""
        if abs(ang) > 90.0:
            note = "  <-- BEYOND joint1 limit (+/-90deg)!"
        elif abs(ang) > 80.0:
            note = "  <-- close to joint1 limit"
        print(f"  {sq}: x={x:+.3f}  y={y:+.3f}  "
              f"(dist {dist*100:.1f}cm, angle {ang:+.0f}deg){note}")
        targets.append((sq, x, y))
    print()

    if args.check_only:
        return

    if not HAVE_ROS:
        print("rclpy not available in this environment — run inside the ROS "
              "container for motion, or use --check-only here.")
        sys.exit(1)

    answer = input("Move the arm through these squares? [y/N] ").strip().lower()
    if answer != "y":
        print("Aborted, no motion sent.")
        return

    rclpy.init()
    node = ArmClient("hover_test", move_time=args.move_time,
                     max_speed=args.max_speed)
    try:
        if not node.wait_ready():
            sys.exit(1)
        # If the stack was just launched, the driver spends ~3s pulling the
        # arm to its assumed-upright pose. Moving during that correction
        # makes commands and reality fight; wait it out.
        print("Settling 4s (lets any driver startup correction finish) ...")
        node.settle(4.0)
        if args.gripper is not None:
            print(f"Setting gripper to {args.gripper} ...")
            node.set_gripper(args.gripper)
        for sq, x, y in targets:
            print(f"\n--> Hovering over {sq} (x={x:.3f}, y={y:.3f}) ...")
            ok = node.move_to(x, y, args.z)
            print(f"    {'OK' if ok else 'FAILED'} - check gripper alignment over {sq}")
            if sq != targets[-1][0]:
                input("    Press Enter for next square (Ctrl-C to stop) ")
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
