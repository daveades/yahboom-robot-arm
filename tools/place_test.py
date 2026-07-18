#!/usr/bin/env python3
"""Measure the arm's net place offset: pick a piece and set it back on
the SAME square. The claw drags the piece to its own center on pick, so
where the piece lands relative to where you put it is exactly the
systematic error to calibrate out.

For each square: center a piece on it precisely, press Enter, let the
arm do its round trip, then measure the landing offset with a ruler
(millimeters, sign convention: +forward = toward rank 8 / away from the
robot, +left = toward the a-file).

Test a middle square plus one on each wing to see whether the offset is
constant in board coordinates (fix: shift a1 in board.yaml) or rotates
with the arm's bearing (fix: claw-frame offsets in arm_client):

    python3 tools/place_test.py d3 a2 g2
"""
import argparse
import sys

import rclpy

from arm_client import ArmClient
from board_config import add_board_args, resolve as resolve_board
from chess_game import BoardMotion


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    add_board_args(parser)
    # Motion defaults mirror chess_game.py - keep them in sync.
    parser.add_argument("--hover-z", type=float, default=0.10)
    parser.add_argument("--grasp-z", type=float, default=0.053)
    parser.add_argument("--carry-z", type=float, default=0.16)
    parser.add_argument("--max-tilt", type=float, default=45.0)
    parser.add_argument("--pick-tilt", type=float, default=32.0)
    parser.add_argument("--pick-forward", type=float, default=0.002)
    parser.add_argument("--grip-open", type=float, default=-1.1)
    parser.add_argument("--grip-closed", type=float, default=-1.42)
    parser.add_argument("--move-time", type=float, default=2.0)
    parser.add_argument("--max-speed", type=float, default=0.5)
    parser.add_argument("squares", nargs="+", help="squares to round-trip")
    args = parser.parse_args()
    args.discard = None

    geom = resolve_board(args)
    rclpy.init()
    node = ArmClient("place_test", move_time=args.move_time,
                     max_speed=args.max_speed)
    try:
        if not node.wait_ready():
            return 1
        print("Settling 4s (lets any driver startup correction finish) ...")
        node.settle(4.0)
        motion = BoardMotion(node, geom, args)
        for sq in args.squares:
            input(f"\nCenter a piece EXACTLY on {sq}, then press Enter ")
            if not motion._transfer(motion.xy(sq), motion.xy(sq),
                                    f"{sq} round trip",
                                    from_sq=sq, to_sq=sq):
                print(f"  !! round trip on {sq} failed")
                continue
            print(f"  Measure {sq}: mm forward (+toward rank 8) and "
                  f"mm left (+toward a-file) of the square's center.")
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
