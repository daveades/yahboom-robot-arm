#!/usr/bin/env python3
"""Generate a print-ready sheet of ArUco corner markers for the chess board.

Creates four DICT_4X4_50 markers (ids 0-3), one per board corner, laid out
on an A4 page at 300 DPI with labels and cut lines. Print at 100% scale
("Actual size" - NOT "fit to page") and verify with a ruler that a marker
measures the requested size before sticking them down.

Corner assignment (looking at the board from the arm):
    id 0 -> a1 corner    id 1 -> h1 corner
    id 2 -> h8 corner    id 3 -> a8 corner

Usage (inside the container):
    python3 tools/gen_aruco_markers.py                  # 30 mm markers
    python3 tools/gen_aruco_markers.py --size-mm 25
    python3 tools/gen_aruco_markers.py --out my_sheet.png
"""
import argparse
import os

import cv2
import numpy as np

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

DPI = 300
MM_PER_INCH = 25.4
A4_MM = (210, 297)
CORNER_LABELS = ["a1 corner", "h1 corner", "h8 corner", "a8 corner"]


def mm_to_px(mm: float) -> int:
    return round(mm / MM_PER_INCH * DPI)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--size-mm", type=float, default=30.0,
                    help="marker side length in mm (default 30)")
    ap.add_argument("--out", default=os.path.join(REPO_ROOT, "aruco_markers.png"),
                    help="output path (default: repo root, so it is visible "
                         "on the host through the bind mount)")
    args = ap.parse_args()

    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    page = np.full((mm_to_px(A4_MM[1]), mm_to_px(A4_MM[0])), 255, np.uint8)

    marker_px = mm_to_px(args.size_mm)
    quiet_px = mm_to_px(args.size_mm / 4)  # white quiet zone around marker
    cell = marker_px + 2 * quiet_px
    margin = mm_to_px(20)
    gap = mm_to_px(15)

    for i in range(4):
        row, col = divmod(i, 2)
        x = margin + col * (cell + gap)
        y = margin + row * (cell + gap + mm_to_px(10))
        marker = cv2.aruco.generateImageMarker(dictionary, i, marker_px)
        page[y + quiet_px:y + quiet_px + marker_px,
             x + quiet_px:x + quiet_px + marker_px] = marker
        # cut line around the quiet zone
        cv2.rectangle(page, (x, y), (x + cell, y + cell), 128, 2)
        cv2.putText(page, f"id {i}  ({CORNER_LABELS[i]})",
                    (x, y + cell + mm_to_px(7)),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, 0, 3)

    cv2.putText(page,
                f"DICT_4X4_50  markers {args.size_mm:g} mm  print at 100% "
                f"scale ({DPI} DPI)",
                (margin, mm_to_px(A4_MM[1]) - mm_to_px(10)),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, 0, 2)

    cv2.imwrite(args.out, page)
    print(f"Wrote {args.out} ({args.size_mm:g} mm markers, DICT_4X4_50, "
          f"ids 0-3).")

    # PDF embeds the physical page size, so printers can't rescale it.
    try:
        from PIL import Image
        pdf_path = os.path.splitext(args.out)[0] + ".pdf"
        Image.fromarray(page).save(pdf_path, resolution=DPI)
        print(f"Wrote {pdf_path} (prefer this for printing - exact A4 size).")
    except ImportError:
        print("Pillow not installed, skipped PDF (pip install pillow).")

    print("Print at 100% / 'Actual size', then measure a marker with a "
          "ruler to confirm.")
    return 0


if __name__ == "__main__":
    main()
