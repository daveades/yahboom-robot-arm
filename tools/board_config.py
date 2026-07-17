"""Board model loader and square math shared by the calibration tools.

The chess board's pose in the robot base frame lives in config/board.yaml
(one source of truth). Tools read it for defaults; their CLI flags
override individual values.
"""
import math
from pathlib import Path
from typing import Tuple

import yaml

DEFAULT_PATH = Path(__file__).resolve().parents[1] / "config" / "board.yaml"


def load_board(path: str | None = None) -> dict:
    p = Path(path) if path else DEFAULT_PATH
    with open(p) as f:
        d = yaml.safe_load(f)
    return {
        "a1": (float(d["a1"][0]), float(d["a1"][1])),
        "square": float(d["square"]),
        "yaw_deg": float(d["yaw_deg"]),
        "mirror": bool(d.get("mirror", False)),
    }


def resolve(args) -> tuple:
    """Merge argparse values over board.yaml. Returns (a1, square, yaw, mirror)."""
    board = None

    def fallback(key):
        nonlocal board
        if board is None:
            try:
                board = load_board(getattr(args, "board", None))
            except FileNotFoundError:
                raise SystemExit(
                    f"No board model: pass --a1/--square or create {DEFAULT_PATH}"
                )
        return board[key]

    a1 = tuple(args.a1) if args.a1 else fallback("a1")
    square = args.square if args.square is not None else fallback("square")
    yaw = args.yaw if args.yaw is not None else fallback("yaw_deg")
    mirror = args.mirror if args.mirror is not None else fallback("mirror")
    return a1, square, yaw, mirror


def square_to_xy(
    square: str,
    a1: Tuple[float, float],
    size: float,
    yaw_deg: float,
    mirror: bool,
) -> Tuple[float, float]:
    """Base-frame (x, y) of a square's center, e.g. square_to_xy('e4', ...)."""
    name = square.strip().lower()
    if len(name) != 2 or name[0] not in "abcdefgh" or name[1] not in "12345678":
        raise ValueError(f"Bad square name: {square!r}")
    file_idx = ord(name[0]) - ord("a")  # 0..7 along a->h
    rank_idx = int(name[1]) - 1  # 0..7 along 1->8

    yaw = math.radians(yaw_deg)
    fx, fy = math.cos(yaw), math.sin(yaw)  # file direction (a->h)
    # rank direction: +90 deg CCW from files, or -90 deg with --mirror
    if mirror:
        rx, ry = fy, -fx
    else:
        rx, ry = -fy, fx

    x = a1[0] + size * (file_idx * fx + rank_idx * rx)
    y = a1[1] + size * (file_idx * fy + rank_idx * ry)
    return x, y


def add_board_args(parser) -> None:
    """Attach the standard board-model override flags to an ArgumentParser."""
    import argparse

    parser.add_argument("--a1", nargs=2, type=float, default=None,
                        metavar=("X", "Y"),
                        help="center of square a1 in base frame, meters "
                             "(default: config/board.yaml)")
    parser.add_argument("--square", type=float, default=None,
                        help="square size in meters (default: config/board.yaml)")
    parser.add_argument("--yaw", type=float, default=None,
                        help="file direction a->h in degrees (0=+x, 90=+y)")
    parser.add_argument("--mirror", action=argparse.BooleanOptionalAction,
                        default=None,
                        help="flip rank direction to the other side")
    parser.add_argument("--board", default=None,
                        help="board model yaml (default: config/board.yaml)")
