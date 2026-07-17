#!/usr/bin/env python3
"""Manual 4-point pixel->base homography (generic fallback).

Prefer tools/calibrate_camera.py (scripts/homography.sh): it auto-detects
49 chessboard corners and is far more accurate. Use this only for scenes
without a chessboard, entering the point correspondences by hand.
"""
import argparse
import sys

import cv2
import numpy as np


def parse_points(value: str):
    points = []
    for pair in value.split(";"):
        pair = pair.strip()
        if not pair:
            continue
        x_str, y_str = pair.split(",")
        points.append([float(x_str), float(y_str)])
    return np.array(points, dtype=float)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Compute a homography matrix from image pixels to base XY."
    )
    parser.add_argument(
        "--image",
        required=True,
        help="Image points as 'u,v;u,v;u,v;u,v' (pixels).",
    )
    parser.add_argument(
        "--base",
        required=True,
        help="Base points as 'x,y;x,y;x,y;x,y' (meters).",
    )
    args = parser.parse_args()

    img_pts = parse_points(args.image)
    base_pts = parse_points(args.base)

    if img_pts.shape[0] < 4 or base_pts.shape[0] < 4:
        print("Need at least 4 points for image and base.", file=sys.stderr)
        return 1
    if img_pts.shape[0] != base_pts.shape[0]:
        print("Image and base points count mismatch.", file=sys.stderr)
        return 1

    H, status = cv2.findHomography(img_pts, base_pts, method=0)
    if H is None:
        print("Homography failed.", file=sys.stderr)
        return 1

    print("Homography (row-major):")
    flat = H.flatten().tolist()
    print(", ".join(f"{v:.8f}" for v in flat))
    print("\nYAML snippet:")
    print("homography:")
    for v in flat:
        print(f"  - {v:.8f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
