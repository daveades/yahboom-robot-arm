#!/usr/bin/env python3
"""IK-check every square of the board model without moving the arm.

For each of the 64 squares, asks /compute_ik whether the arm can reach it
at --hover-z (travel height) and --grasp-z (pick height), and prints a
reachability map. Use it to size and place the board before printing or
taping anything (run against the simulation stack: scripts/sim.sh).

    #  = reachable at hover AND grasp height
    o  = hover only        ^  = grasp only
    .  = unreachable

Example - test a candidate 22 mm board placed 12 cm out:
    python3 tools/reach_check.py --square 0.022 --a1 0.12 0.077
"""
import argparse
import math
import sys

from board_config import add_board_args, resolve as resolve_board, square_to_xy

import rclpy
from rclpy.logging import LoggingSeverity
from arm_client import ArmClient


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    add_board_args(parser)
    parser.add_argument("--hover-z", type=float, default=0.12,
                        help="travel height in meters (default 0.12)")
    parser.add_argument("--grasp-z", type=float, default=0.06,
                        help="grasp height in meters (default 0.06)")
    parser.add_argument("--max-tilt", type=float, default=35.0,
                        help="max gripper tilt from vertical in degrees "
                             "(default 35; pass 999 to disable)")
    args = parser.parse_args()

    a1, square, yaw, mirror = resolve_board(args)
    print(f"Board: a1=({a1[0]:.3f}, {a1[1]:.3f})  square={square*1000:.1f}mm  "
          f"yaw={yaw:.0f}deg  mirror={mirror}")
    print(f"Heights: hover={args.hover_z:.3f}  grasp={args.grasp_z:.3f}\n")

    rclpy.init()
    node = ArmClient("reach_check")
    node.get_logger().set_level(LoggingSeverity.ERROR)  # silence per-square IK warns
    if not node.ik_client.wait_for_service(timeout_sec=10.0):
        print("IK service /compute_ik not available - is the stack running "
              "(scripts/sim.sh or moveit.sh)?", file=sys.stderr)
        return 1

    grid = {}
    n_full = 0
    dists = []
    for rank in range(8, 0, -1):
        for file in "abcdefgh":
            sq = f"{file}{rank}"
            x, y = square_to_xy(sq, a1, square, yaw, mirror)
            dists.append((math.hypot(x, y), sq))
            hover_ok = node.solve_ik(x, y, args.hover_z,
                                     max_tilt_deg=args.max_tilt) is not None
            grasp_ok = node.solve_ik(x, y, args.grasp_z,
                                     max_tilt_deg=args.max_tilt) is not None
            grid[sq] = ("#" if hover_ok and grasp_ok else
                        "o" if hover_ok else
                        "^" if grasp_ok else ".")
            if hover_ok and grasp_ok:
                n_full += 1

    for rank in range(8, 0, -1):
        row = " ".join(grid[f"{f}{rank}"] for f in "abcdefgh")
        print(f"  {rank}  {row}")
    print("\n     a b c d e f g h\n")
    near = min(dists)
    far = max(dists)
    print(f"{n_full}/64 squares fully reachable "
          f"(#=hover+grasp, o=hover only, ^=grasp only, .=neither)")
    print(f"Nearest square: {near[1]} at {near[0]*100:.1f}cm, "
          f"farthest: {far[1]} at {far[0]*100:.1f}cm from base")

    node.destroy_node()
    rclpy.shutdown()
    return 0 if n_full == 64 else 2


if __name__ == "__main__":
    sys.exit(main())
