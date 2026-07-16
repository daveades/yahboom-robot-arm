#!/usr/bin/env python3
"""Calibrate the pixel->base homography using the chessboard itself.

Detects all 49 inner corners of the 8x8 board with OpenCV, pairs each with
its known base-frame position (from the same --a1 / --square / --yaw /
--mirror board model used by hover_test.py), and fits the homography in one
least-squares shot. Far more accurate than the 4-object manual procedure.

Requirements:
  * Camera fixed rigidly, viewing the whole board. Moving it afterwards
    invalidates the calibration.
  * Board EMPTY (no pieces) and evenly lit for corner detection.

Frame source: --image FILE, or a live grab from --topic (default
/image_raw, needs the ffmpeg bridge + stream_camera_node on WSL2).

Orientation check: the tool writes an annotated image (--annotate) with
a1/h1/a8/h8 labelled. If the labels sit on the wrong corners, re-run with
--rotate 90/180/270 until they match your physical board.

Example:
  python3 tools/calibrate_camera.py --a1 0.085 0.155 --square 0.04375 \\
      --yaw -90 --annotate check.png
Then paste the printed homography into
dofbot_ros2_ws/src/dofbot_vision/config/picking.yaml and rebuild.
"""

import argparse
import math
import sys

import cv2
import numpy as np

PATTERN = (7, 7)  # inner corners of an 8x8 board


def board_dirs(yaw_deg: float, mirror: bool):
    yaw = math.radians(yaw_deg)
    fx, fy = math.cos(yaw), math.sin(yaw)  # file direction (a->h)
    if mirror:
        rx, ry = fy, -fx
    else:
        rx, ry = -fy, fx
    return (fx, fy), (rx, ry)


def indices_to_xy(file_idx: float, rank_idx: float, a1, size, yaw_deg, mirror):
    """Fractional square indices (a1 center = 0,0) -> base-frame meters."""
    (fx, fy), (rx, ry) = board_dirs(yaw_deg, mirror)
    x = a1[0] + size * (file_idx * fx + rank_idx * rx)
    y = a1[1] + size * (file_idx * fy + rank_idx * ry)
    return x, y


def square_center_xy(square: str, a1, size, yaw_deg, mirror):
    file_idx = ord(square[0].lower()) - ord("a")
    rank_idx = int(square[1]) - 1
    return indices_to_xy(file_idx, rank_idx, a1, size, yaw_deg, mirror)


def grab_frame(topic: str, timeout: float) -> "np.ndarray":
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image

    rclpy.init()
    node = Node("camera_calib_grab")
    frames = []

    def cb(msg: Image) -> None:
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        img = arr.reshape(msg.height, msg.width, -1)
        if msg.encoding == "rgb8":
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        frames.append(img.copy())

    node.create_subscription(Image, topic, cb, 10)
    deadline = node.get_clock().now().nanoseconds + int(timeout * 1e9)
    while not frames and node.get_clock().now().nanoseconds < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()
    if not frames:
        print(f"No image received on {topic} within {timeout}s.", file=sys.stderr)
        sys.exit(1)
    return frames[-1]


def corner_indices(i: int, j: int, rotate: int):
    """Map detected-grid indices to board corner indices for a given
    orientation. Corner (0,0) of the board sits between a1/a2/b1/b2 ...
    i.e. board corner (i,j) = fractional square indices (i+0.5, j+0.5)."""
    n = PATTERN[0] - 1  # 6
    if rotate == 0:
        return i, j
    if rotate == 90:
        return j, n - i
    if rotate == 180:
        return n - i, n - j
    if rotate == 270:
        return n - j, i
    raise ValueError("rotate must be 0/90/180/270")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--a1", nargs=2, type=float, required=True, metavar=("X", "Y"))
    parser.add_argument("--square", type=float, required=True)
    parser.add_argument("--yaw", type=float, default=0.0)
    parser.add_argument("--mirror", action="store_true")
    parser.add_argument("--rotate", type=int, default=0, choices=[0, 90, 180, 270],
                        help="fix pattern orientation if labels are wrong")
    parser.add_argument("--image", help="use an image file instead of the live topic")
    parser.add_argument("--topic", default="/image_raw")
    parser.add_argument("--timeout", type=float, default=10.0)
    parser.add_argument("--annotate", default="calib_check.png",
                        help="write labelled image here for verification")
    args = parser.parse_args()

    if args.image:
        frame = cv2.imread(args.image)
        if frame is None:
            print(f"Cannot read {args.image}", file=sys.stderr)
            sys.exit(1)
    else:
        frame = grab_frame(args.topic, args.timeout)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    found, corners = cv2.findChessboardCorners(
        gray, PATTERN,
        flags=cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE,
    )
    if not found:
        print("Chessboard corners NOT found. Check: board empty, fully in "
              "frame, even lighting, camera in focus.", file=sys.stderr)
        cv2.imwrite(args.annotate, frame)
        print(f"Raw frame saved to {args.annotate} for inspection.", file=sys.stderr)
        sys.exit(1)

    corners = cv2.cornerSubPix(
        gray, corners, (11, 11), (-1, -1),
        (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001),
    )
    pts = corners.reshape(-1, 2)  # row-major (PATTERN[1] rows x PATTERN[0] cols)

    a1 = (args.a1[0], args.a1[1])
    img_pts, base_pts = [], []
    for j in range(PATTERN[1]):
        for i in range(PATTERN[0]):
            bi, bj = corner_indices(i, j, args.rotate)
            bx, by = indices_to_xy(bi + 0.5, bj + 0.5, a1, args.square,
                                   args.yaw, args.mirror)
            img_pts.append(pts[j * PATTERN[0] + i])
            base_pts.append((bx, by))
    img_pts = np.array(img_pts, dtype=np.float64)
    base_pts = np.array(base_pts, dtype=np.float64)

    H, _ = cv2.findHomography(img_pts, base_pts, method=0)  # least squares
    if H is None:
        print("Homography fit failed.", file=sys.stderr)
        sys.exit(1)

    # Reprojection error in board space (mm)
    ones = np.hstack([img_pts, np.ones((len(img_pts), 1))])
    proj = (H @ ones.T).T
    proj = proj[:, :2] / proj[:, 2:3]
    err_mm = np.linalg.norm(proj - base_pts, axis=1) * 1000.0
    print(f"\nCorners: {len(img_pts)}   reprojection error: "
          f"mean {err_mm.mean():.1f} mm, max {err_mm.max():.1f} mm")
    if err_mm.max() > 5.0:
        print("WARNING: max error > 5 mm — lens distortion or a wrong board "
              "measurement. Consider intrinsic calibration + undistortion.")

    # Annotate square centers for orientation verification
    Hinv = np.linalg.inv(H)
    out = frame.copy()
    cv2.drawChessboardCorners(out, PATTERN, corners, found)
    for sq in ["a1", "h1", "a8", "h8", "d4"]:
        bx, by = square_center_xy(sq, a1, args.square, args.yaw, args.mirror)
        p = Hinv @ np.array([bx, by, 1.0])
        u, v = int(p[0] / p[2]), int(p[1] / p[2])
        cv2.circle(out, (u, v), 6, (0, 0, 255), -1)
        cv2.putText(out, sq, (u + 8, v - 8), cv2.FONT_HERSHEY_SIMPLEX,
                    0.8, (0, 0, 255), 2)
    cv2.imwrite(args.annotate, out)
    print(f"Verification image: {args.annotate} — labels MUST sit on the "
          f"right squares; if not, re-run with --rotate 90/180/270.")

    print("\nPaste into picking.yaml (homography:):")
    for row in H:
        for v in row:
            print(f"      - {v:.8f}")


if __name__ == "__main__":
    main()
