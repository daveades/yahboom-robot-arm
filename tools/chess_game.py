#!/usr/bin/env python3
"""Play chess with the DOFBOT arm.

The game loop: python-chess tracks the position and legality, the human
types their move (vision input comes later), Stockfish picks the robot's
reply, and the reply is executed as pick/place motions over the board
model in config/board.yaml.

Works identically in simulation (scripts/sim.sh) and on hardware
(scripts/driver.sh + scripts/moveit.sh) - same IK service, same actions.

The arm cannot reach a full-size board (far squares force a horizontal
gripper that cannot grasp), so at startup the reachable squares are
mapped (cached in runs/reach_cache.json) and the engine is restricted to
physically playable moves; if none exists the robot asks for a hand.
Place the board with the ROBOT'S OWN back rank nearest the base - e.g.
robot plays White and rank 1 is the near edge - so its pieces and the
center fall inside the reachable band.

Usage:
    python3 tools/chess_game.py                    # human White, robot Black
    python3 tools/chess_game.py --robot-color white
    python3 tools/chess_game.py --self-play        # engine vs itself (demo)
    python3 tools/chess_game.py --no-arm           # game logic only

Type moves as SAN (Nf3, e4, O-O) or UCI (g1f3, e2e4); 'quit' resigns.

Dependencies (inside the container):
    pip install chess
    apt-get install -y stockfish
"""
import argparse
import json
import shutil
import sys
from pathlib import Path

try:
    import chess
    import chess.engine
except ImportError:
    sys.exit("python-chess is not installed: pip install chess")

from board_config import add_board_args, resolve as resolve_board, square_to_xy

try:  # ROS only needed for actual motion (--no-arm works without it)
    import rclpy
    from arm_client import ArmClient, HOME_POSE
    HAVE_ROS = True
except ImportError:
    HAVE_ROS = False


class BoardMotion:
    """Turns chess moves into pick/place sequences on the physical board."""

    def __init__(self, client: "ArmClient", geom: tuple, args) -> None:
        self.client = client
        self.a1, self.square, self.yaw, self.mirror = geom
        self.hover_z = args.hover_z
        self.grasp_z = args.grasp_z
        self.grip_open = args.grip_open
        self.grip_closed = args.grip_closed
        # Captured pieces go to a fixed graveyard point past the h-file.
        if args.discard:
            self.discard_xy = tuple(args.discard)
        else:
            h1 = square_to_xy("h1", self.a1, self.square, self.yaw, self.mirror)
            a1 = square_to_xy("a1", self.a1, self.square, self.yaw, self.mirror)
            fx = (h1[0] - a1[0]) / 7.0, (h1[1] - a1[1]) / 7.0  # one square a->h
            self.discard_xy = (h1[0] + 2.0 * fx[0], h1[1] + 2.0 * fx[1])

    def xy(self, square_name: str) -> tuple:
        return square_to_xy(square_name, self.a1, self.square, self.yaw,
                            self.mirror)

    def _transfer(self, from_xy: tuple, to_xy: tuple, what: str) -> bool:
        """Pick at from_xy, place at to_xy, via hover height."""
        fx, fy = from_xy
        tx, ty = to_xy
        steps = [
            ("open gripper", lambda: self.client.set_gripper(self.grip_open)),
            ("hover source", lambda: self.client.move_to(fx, fy, self.hover_z)),
            ("descend", lambda: self.client.move_to(fx, fy, self.grasp_z)),
            ("grip", lambda: self.client.set_gripper(self.grip_closed)),
            ("lift", lambda: self.client.move_to(fx, fy, self.hover_z)),
            ("hover target", lambda: self.client.move_to(tx, ty, self.hover_z)),
            ("lower", lambda: self.client.move_to(tx, ty, self.grasp_z)),
            ("release", lambda: self.client.set_gripper(self.grip_open)),
            ("retreat", lambda: self.client.move_to(tx, ty, self.hover_z)),
        ]
        for name, action in steps:
            if not action():
                print(f"    !! motion failed at '{name}' while moving {what}")
                return False
        return True

    def go_home(self) -> bool:
        return self.client.move_joints(list(HOME_POSE))

    def execute(self, board: chess.Board, move: chess.Move) -> bool:
        """Physically perform `move` (not yet pushed to `board`)."""
        # 1. A captured piece leaves the board first.
        if board.is_capture(move):
            cap_square = move.to_square
            if board.is_en_passant(move):
                # The captured pawn is behind the arrival square.
                offset = -8 if board.turn == chess.WHITE else 8
                cap_square = move.to_square + offset
            cap_name = chess.square_name(cap_square)
            print(f"    capturing {cap_name} -> discard pile")
            if not self._transfer(self.xy(cap_name), self.discard_xy,
                                  f"captured piece from {cap_name}"):
                return False

        # 2. The moving piece.
        from_name = chess.square_name(move.from_square)
        to_name = chess.square_name(move.to_square)
        print(f"    moving {from_name} -> {to_name}")
        if not self._transfer(self.xy(from_name), self.xy(to_name),
                              f"piece {from_name}->{to_name}"):
            return False

        # 3. Castling also moves the rook.
        if board.is_castling(move):
            rank = "1" if board.turn == chess.WHITE else "8"
            if board.is_kingside_castling(move):
                r_from, r_to = "h" + rank, "f" + rank
            else:
                r_from, r_to = "a" + rank, "d" + rank
            print(f"    castling: rook {r_from} -> {r_to}")
            if not self._transfer(self.xy(r_from), self.xy(r_to),
                                  f"rook {r_from}->{r_to}"):
                return False

        # 4. Promotion needs a human hand (no spare queens in the gripper).
        if move.promotion:
            piece = chess.piece_name(move.promotion)
            input(f"    PROMOTION: replace the pawn on {to_name} with a "
                  f"{piece}, then press Enter ")
        return True


REACH_CACHE = Path(__file__).resolve().parents[1] / "runs" / "reach_cache.json"


def map_reachable_squares(node, motion: BoardMotion, args) -> set:
    """Squares where the arm can both hover and grasp; cached per geometry.

    The arm cannot cover a full-size board (far ranks need a stretched,
    horizontal gripper that cannot pick pieces), so the robot's move
    choice is restricted to this set. Play with the robot's own back rank
    nearest the base.
    """
    key = (f"a1={motion.a1[0]:.3f},{motion.a1[1]:.3f} sq={motion.square:.4f} "
           f"yaw={motion.yaw:.1f} mir={motion.mirror} "
           f"hover={motion.hover_z:.3f} grasp={motion.grasp_z:.3f} "
           f"tilt={args.max_tilt:.0f}")
    cache = {}
    if REACH_CACHE.exists():
        try:
            cache = json.loads(REACH_CACHE.read_text())
        except (json.JSONDecodeError, OSError):
            cache = {}
    if key in cache:
        return set(cache[key])

    print("Mapping reachable squares (one-time per geometry, ~1-2 min) ...")
    from rclpy.logging import LoggingSeverity
    node.get_logger().set_level(LoggingSeverity.ERROR)
    reach = set()
    for sq in chess.SQUARE_NAMES:
        x, y = motion.xy(sq)
        if (node.solve_ik(x, y, motion.hover_z, max_tilt_deg=args.max_tilt,
                          attempts=1) is not None
                and node.solve_ik(x, y, motion.grasp_z,
                                  max_tilt_deg=args.max_tilt,
                                  attempts=1) is not None):
            reach.add(sq)
    node.get_logger().set_level(LoggingSeverity.INFO)

    cache[key] = sorted(reach)
    try:
        REACH_CACHE.parent.mkdir(parents=True, exist_ok=True)
        REACH_CACHE.write_text(json.dumps(cache, indent=1))
    except OSError as exc:
        print(f"(could not cache reach map: {exc})")
    return reach


def move_is_reachable(board: chess.Board, move: chess.Move, reach: set,
                      discard_ok: bool) -> bool:
    """Can the arm physically execute this move?"""
    squares = [move.from_square, move.to_square]
    if board.is_capture(move):
        if not discard_ok:
            return False
        cap = move.to_square
        if board.is_en_passant(move):
            cap += -8 if board.turn == chess.WHITE else 8
        squares.append(cap)
    if board.is_castling(move):
        rank = 0 if board.turn == chess.WHITE else 7
        if board.is_kingside_castling(move):
            squares += [chess.square(7, rank), chess.square(5, rank)]
        else:
            squares += [chess.square(0, rank), chess.square(3, rank)]
    return all(chess.square_name(s) in reach for s in squares)


def parse_human_move(board: chess.Board, text: str):
    """Accept SAN or UCI; return a legal Move or None."""
    text = text.strip()
    try:
        return board.parse_san(text)
    except ValueError:
        pass
    try:
        move = chess.Move.from_uci(text.lower())
        if move in board.legal_moves:
            return move
    except ValueError:
        pass
    return None


def print_board(board: chess.Board) -> None:
    print()
    ranks = str(board).split("\n")
    for i, row in enumerate(ranks):
        print(f"  {8 - i}  {row}")
    print("\n     a b c d e f g h\n")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    add_board_args(parser)
    parser.add_argument("--robot-color", choices=["white", "black"],
                        default="black", help="side the robot plays (default black)")
    parser.add_argument("--fen", default=None,
                        help="start from this position instead of the "
                             "initial one (the physical board must match)")
    parser.add_argument("--self-play", action="store_true",
                        help="engine plays both sides, arm executes every move")
    parser.add_argument("--moves", type=int, default=0,
                        help="stop self-play after this many half-moves (0 = play out)")
    parser.add_argument("--no-arm", action="store_true",
                        help="skip all arm motion (logic/engine test)")
    parser.add_argument("--engine", default=None,
                        help="UCI engine binary (default: stockfish on PATH)")
    parser.add_argument("--skill", type=int, default=5,
                        help="Stockfish skill level 0-20 (default 5)")
    parser.add_argument("--think-time", type=float, default=0.3,
                        help="engine seconds per move (default 0.3)")
    parser.add_argument("--hover-z", type=float, default=0.12,
                        help="travel height in meters (default 0.12)")
    parser.add_argument("--grasp-z", type=float, default=0.06,
                        help="grasp height in meters (default 0.06)")
    parser.add_argument("--grip-open", type=float, default=0.0,
                        help="grip_joint position for open (default 0.0)")
    parser.add_argument("--grip-closed", type=float, default=-1.0,
                        help="grip_joint position for closed (default -1.0)")
    parser.add_argument("--max-tilt", type=float, default=45.0,
                        help="max gripper tilt from vertical in degrees for "
                             "reachability (default 45)")
    parser.add_argument("--discard", nargs=2, type=float, default=None,
                        metavar=("X", "Y"),
                        help="captured-piece drop point in base frame "
                             "(default: two squares past h-file, rank 1)")
    parser.add_argument("--move-time", type=float, default=2.0,
                        help="minimum seconds per arm motion (default 2.0)")
    parser.add_argument("--max-speed", type=float, default=0.5,
                        help="peak joint speed in rad/s (default 0.5)")
    args = parser.parse_args()

    geom = resolve_board(args)
    a1, square, yaw, mirror = geom
    print(f"Board: a1=({a1[0]:.3f}, {a1[1]:.3f})  square={square*1000:.0f}mm  "
          f"yaw={yaw:.0f}deg  mirror={mirror}")

    # Debian installs stockfish into /usr/games, often not on PATH.
    engine_path = (args.engine or shutil.which("stockfish")
                   or shutil.which("stockfish", path="/usr/games"))
    if engine_path is None:
        sys.exit("Stockfish not found: apt-get install -y stockfish "
                 "(or pass --engine /path/to/uci-engine)")
    engine = chess.engine.SimpleEngine.popen_uci(engine_path)
    engine.configure({"Skill Level": max(0, min(20, args.skill))})
    print(f"Engine: {engine_path} (skill {args.skill}, "
          f"{args.think_time}s per move)")

    motion = None
    node = None
    reach = None
    discard_ok = True
    if not args.no_arm:
        if not HAVE_ROS:
            engine.quit()
            sys.exit("rclpy not available - run inside the ROS container, "
                     "or pass --no-arm for a logic-only game.")
        rclpy.init()
        node = ArmClient("chess_game", move_time=args.move_time,
                         max_speed=args.max_speed)
        if not node.wait_ready():
            engine.quit()
            sys.exit(1)
        print("Settling 4s (lets any driver startup correction finish) ...")
        node.settle(4.0)
        motion = BoardMotion(node, geom, args)
        reach = map_reachable_squares(node, motion, args)
        dx, dy = motion.discard_xy
        discard_ok = (node.solve_ik(dx, dy, motion.hover_z,
                                    max_tilt_deg=args.max_tilt) is not None
                      and node.solve_ik(dx, dy, motion.grasp_z,
                                        max_tilt_deg=args.max_tilt) is not None)
        by_rank = {r: sum(1 for s in reach if s[1] == r) for r in "12345678"}
        discard_note = ("OK" if discard_ok else
                        "UNREACHABLE - captures disabled, consider --discard X Y")
        print(f"Arm can play on {len(reach)}/64 squares "
              f"(per rank 1..8: {[by_rank[r] for r in '12345678']}); "
              f"discard point {discard_note}")
        if not reach:
            engine.quit()
            sys.exit("No reachable squares at all - check board placement "
                     "(board.yaml) against reality with tools/reach_check.py.")

    board = chess.Board(args.fen) if args.fen else chess.Board()
    robot_color = chess.WHITE if args.robot_color == "white" else chess.BLACK
    resigned = False

    try:
        while not board.is_game_over():
            if args.moves and len(board.move_stack) >= args.moves:
                print(f"Reached --moves {args.moves}, stopping.")
                break
            print_board(board)
            side = "White" if board.turn == chess.WHITE else "Black"

            robot_turn = args.self_play or board.turn == robot_color
            if robot_turn:
                limit = chess.engine.Limit(time=args.think_time)
                allowed = None
                if reach is not None:
                    allowed = [m for m in board.legal_moves
                               if move_is_reachable(board, m, reach, discard_ok)]
                if allowed is not None and not allowed:
                    # Nothing physically playable: let the engine pick freely
                    # and ask the human to move the pieces for the robot.
                    move = engine.play(board, limit).move
                    print(f"{side} (robot): {board.san(move)} - OUT OF REACH")
                    input(f"    Please make this move for me ("
                          f"{move.uci()}), then press Enter ")
                else:
                    move = engine.play(board, limit, root_moves=allowed).move
                    print(f"{side} (robot): {board.san(move)}")
                    if motion is not None:
                        if not motion.execute(board, move):
                            print("Arm failed to execute the move - stopping "
                                  "so the physical board stays in sync.")
                            break
                        motion.go_home()
            else:
                while True:
                    text = input(f"{side} (you), your move: ").strip()
                    if text.lower() in ("quit", "resign"):
                        resigned = True
                        break
                    move = parse_human_move(board, text)
                    if move is not None:
                        break
                    print("  Not a legal move (SAN like Nf3 or UCI like g1f3).")
                if resigned:
                    print(f"{side} resigns.")
                    break
            board.push(move)

        if board.is_game_over():
            print_board(board)
            outcome = board.outcome()
            print(f"Game over: {outcome.result()} ({outcome.termination.name})")
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        engine.quit()
        if node is not None:
            node.destroy_node()
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
