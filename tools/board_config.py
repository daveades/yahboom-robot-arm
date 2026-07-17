"""Board model loader shared by the calibration tools.

The chess board's pose in the robot base frame lives in config/board.yaml
(one source of truth). Tools read it for defaults; their CLI flags
override individual values.
"""
from pathlib import Path

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
