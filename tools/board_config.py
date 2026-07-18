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

# Calibration offsets measured with tools/place_test.py: board.yaml may
# carry an `offsets:` map of square -> [dx, dy] (meters, base frame) to
# ADD to the commanded position so the claw lands centered there.
# Between anchors the offset is inverse-distance-weighted: each measured
# square pins its own neighborhood, corrections fade smoothly toward
# anchors measured as centered, and extrapolation past the outermost
# anchor stays bounded at that anchor's value. Add anchors wherever the
# claw is off; a zero anchor is information too ("this square is good").
_OFFSET_ANCHORS: dict = {}

# Optional `pitch:` map of square -> claw pitch (deg from vertical) to
# PIN that square's pick/place column to a fixed angle. Squares whose
# geometry leaves the pitch free to slide with the target are not
# calibratable (the anchor moves the pitch, the pitch moves the tip),
# so pin them here. Squares already clamped at the preferred pitch don't
# need an entry.
_PITCH_PINS: dict = {}


def load_board(path: str | None = None) -> dict:
    global _OFFSET_ANCHORS, _PITCH_PINS
    p = Path(path) if path else DEFAULT_PATH
    with open(p) as f:
        d = yaml.safe_load(f)
    _OFFSET_ANCHORS = {
        str(k).strip().lower(): (float(v[0]), float(v[1]))
        for k, v in (d.get("offsets") or {}).items()
    }
    _PITCH_PINS = {
        str(k).strip().lower(): float(v)
        for k, v in (d.get("pitch") or {}).items()
    }
    return {
        "a1": (float(d["a1"][0]), float(d["a1"][1])),
        "square": float(d["square"]),
        "yaw_deg": float(d["yaw_deg"]),
        "mirror": bool(d.get("mirror", False)),
    }


def pitch_pin(square: str | None):
    """Fixed claw pitch (deg) for a square, or None if it may float."""
    if not square:
        return None
    return _PITCH_PINS.get(square.strip().lower())


def _offset_at(file_idx: int, rank_idx: int) -> Tuple[float, float]:
    if not _OFFSET_ANCHORS:
        return 0.0, 0.0
    num_x = num_y = den = 0.0
    for name, (dx, dy) in _OFFSET_ANCHORS.items():
        df = file_idx - (ord(name[0]) - ord("a"))
        dr = rank_idx - (int(name[1]) - 1)
        d2 = df * df + dr * dr
        if d2 == 0:
            return dx, dy
        w = 1.0 / d2
        num_x += w * dx
        num_y += w * dy
        den += w
    return num_x / den, num_y / den


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
    apply_offset: bool = True,
) -> Tuple[float, float]:
    """Base-frame (x, y) of a square's center, e.g. square_to_xy('e4', ...).

    apply_offset adds the calibration anchor (default). Pass False for the
    NOMINAL center - used to choose the claw pitch, which must not depend
    on the anchor or the anchor could never be calibrated (adjusting it
    would move the pitch, which moves the tip).
    """
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
    if apply_offset:
        dx, dy = _offset_at(file_idx, rank_idx)
        x, y = x + dx, y + dy
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
