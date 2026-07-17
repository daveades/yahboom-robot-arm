#!/usr/bin/env python3
"""Generate a print-ready chess board with integrated ArUco corner markers.

The four DICT_4X4_50 markers (same ids/corners as gen_aruco_markers.py:
id 0 = a1, 1 = h1, 2 = h8, 3 = a8) are printed on the same sheet as the
board, so board->marker geometry is exact by construction; the script
prints each marker center's offset from the a1 square center for the
vision configuration.

Paper: an 8x26mm board is wider than A4, so --paper chooses:
    a3   one portrait A3 sheet
    a4   two landscape A4 sheets (ranks 5-8 and 1-4); cut sheet 2 along
         its seam line, butt it against sheet 1's seam line, tape.

Print at 100% / "Actual size", then verify a square measures --square-mm
with a ruler. Place the board with rank 1 nearest the robot (the sheet
says ARM SIDE) - the robot plays White.

Usage (inside the container):
    python3 tools/gen_board.py                    # 26 mm squares, A4 pair
    python3 tools/gen_board.py --paper a3
    python3 tools/gen_board.py --square-mm 30
"""
import argparse
import os

import cv2
import numpy as np

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DPI = 300
MM_PER_INCH = 25.4
PAPER_MM = {"a4": (297, 210), "a3": (297, 420)}  # a4 landscape, a3 portrait
DARK, LIGHT, INK = 165, 255, 0
CORNER_IDS = {0: "a1", 1: "h1", 2: "h8", 3: "a8"}


def mm_to_px(v: float) -> int:
    return round(v / MM_PER_INCH * DPI)


def build_canvas(square_mm: float, marker_mm: float, quiet_mm: float):
    """Full board + markers as one image; returns (canvas, seam_y_px)."""
    pad = 4.0
    cell = marker_mm + 2 * quiet_mm
    board = 8 * square_mm
    w = 2 * pad + 2 * cell + board
    h = 2 * pad + 2 * cell + board
    canvas = np.full((mm_to_px(h), mm_to_px(w)), 255, np.uint8)

    x0, y0 = pad + cell, pad + cell  # top-left of the a8 square
    s = square_mm

    # Squares: a1 is dark; ranks 8..1 top to bottom, files a..h left-right.
    for row in range(8):          # row 0 = rank 8
        for col in range(8):      # col 0 = file a
            rank = 8 - row
            dark = (col + rank) % 2 == 1  # a1 (col 0, rank 1) is dark
            if dark:
                x1, y1 = mm_to_px(x0 + col * s), mm_to_px(y0 + row * s)
                x2, y2 = mm_to_px(x0 + (col + 1) * s), mm_to_px(y0 + (row + 1) * s)
                canvas[y1:y2, x1:x2] = DARK
    # Thin grid so every square edge is visible (also marks the cut seam
    # where it crosses light squares).
    for k in range(9):
        cv2.line(canvas, (mm_to_px(x0 + k * s), mm_to_px(y0)),
                 (mm_to_px(x0 + k * s), mm_to_px(y0 + board)), INK, 1)
        cv2.line(canvas, (mm_to_px(x0), mm_to_px(y0 + k * s)),
                 (mm_to_px(x0 + board), mm_to_px(y0 + k * s)), INK, 1)
    cv2.rectangle(canvas, (mm_to_px(x0), mm_to_px(y0)),
                  (mm_to_px(x0 + board), mm_to_px(y0 + board)), INK, 2)

    # Markers at the four board corners, touching diagonally.
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    centers = {
        0: (x0 - cell / 2, y0 + board + cell / 2),           # a1 corner
        1: (x0 + board + cell / 2, y0 + board + cell / 2),   # h1
        2: (x0 + board + cell / 2, y0 - cell / 2),           # h8
        3: (x0 - cell / 2, y0 - cell / 2),                   # a8
    }
    mpx = mm_to_px(marker_mm)
    marker_offsets = {}
    for mid, (cx, cy) in centers.items():
        img = cv2.aruco.generateImageMarker(dictionary, mid, mpx)
        x1 = mm_to_px(cx) - mpx // 2
        y1 = mm_to_px(cy) - mpx // 2
        canvas[y1:y1 + mpx, x1:x1 + mpx] = img
        # Offset from the a1 square center, +file toward h, +rank toward 8.
        a1cx, a1cy = x0 + s / 2, y0 + board - s / 2
        marker_offsets[mid] = (cx - a1cx, a1cy - cy)

    # Coordinate labels in the margin bands (clear of the corner markers).
    for col in range(8):
        cv2.putText(canvas, "abcdefgh"[col],
                    (mm_to_px(x0 + col * s + s / 2 - 1.5),
                     mm_to_px(y0 + board + 6.5)),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, INK, 2)
    for row in range(8):
        cv2.putText(canvas, str(8 - row),
                    (mm_to_px(x0 - 7.0),
                     mm_to_px(y0 + row * s + s / 2 + 2.0)),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, INK, 2)
    cv2.putText(canvas, "ARM SIDE - rank 1 faces the robot",
                (mm_to_px(x0 + board / 2 - 45), mm_to_px(h - 3)),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, INK, 2)

    seam_y = mm_to_px(y0 + 4 * s)  # boundary between ranks 5 and 4
    return canvas, seam_y, marker_offsets


def paste_on_page(part: np.ndarray, paper: str) -> np.ndarray:
    pw, ph = PAPER_MM[paper]
    page = np.full((mm_to_px(ph), mm_to_px(pw)), 255, np.uint8)
    y = (page.shape[0] - part.shape[0]) // 2
    x = (page.shape[1] - part.shape[1]) // 2
    if y < 0 or x < 0:
        raise SystemExit(
            f"Content ({part.shape[1]}x{part.shape[0]}px) does not fit "
            f"{paper.upper()} - reduce --square-mm or --marker-mm.")
    page[y:y + part.shape[0], x:x + part.shape[1]] = part
    return page


def save(pages, out_base: str) -> None:
    paths = []
    for i, page in enumerate(pages, 1):
        suffix = f"_sheet{i}" if len(pages) > 1 else ""
        p = f"{out_base}{suffix}.png"
        cv2.imwrite(p, page)
        paths.append(p)
    print("Wrote " + ", ".join(paths))
    try:
        from PIL import Image
        imgs = [Image.fromarray(p) for p in pages]
        pdf = f"{out_base}.pdf"
        imgs[0].save(pdf, resolution=DPI, save_all=True,
                     append_images=imgs[1:])
        print(f"Wrote {pdf} (prefer this for printing - exact scale)")
    except ImportError:
        print("Pillow not installed, skipped PDF (pip install pillow).")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--square-mm", type=float, default=26.0,
                    help="square size in mm (default 26)")
    ap.add_argument("--marker-mm", type=float, default=25.0,
                    help="ArUco marker side in mm (default 25)")
    ap.add_argument("--quiet-mm", type=float, default=5.0,
                    help="white quiet zone around each marker (default 5)")
    ap.add_argument("--paper", choices=["a4", "a3"], default="a4",
                    help="a4 = two landscape sheets, a3 = one sheet (default a4)")
    ap.add_argument("--out", default=os.path.join(REPO_ROOT, "board"),
                    help="output basename (default: <repo>/board)")
    args = ap.parse_args()

    canvas, seam_y, offsets = build_canvas(args.square_mm, args.marker_mm,
                                           args.quiet_mm)
    if args.paper == "a3":
        pages = [paste_on_page(canvas, "a3")]
    else:
        top, bottom = canvas[:seam_y, :], canvas[seam_y:, :]
        # Seam guidance printed just outside the board content.
        top = np.vstack([top, np.full((mm_to_px(8), top.shape[1]), 255, np.uint8)])
        cv2.putText(top, "SHEET 1 (ranks 5-8): cut EXACTLY on this bottom edge",
                    (mm_to_px(10), top.shape[0] - mm_to_px(2)),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, INK, 2)
        bottom = np.vstack([np.full((mm_to_px(8), bottom.shape[1]), 255, np.uint8), bottom])
        cv2.putText(bottom, "SHEET 2 (ranks 1-4): butt sheet 1 against this top edge, tape",
                    (mm_to_px(10), mm_to_px(6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, INK, 2)
        pages = [paste_on_page(top, "a4"), paste_on_page(bottom, "a4")]

    save(pages, args.out)

    print(f"\nBoard: {args.square_mm:g} mm squares "
          f"({8 * args.square_mm / 10:g} cm), markers {args.marker_mm:g} mm "
          f"DICT_4X4_50.")
    print("Marker centers relative to the a1 SQUARE CENTER "
          "(+file toward h, +rank toward 8), for the vision config:")
    for mid in sorted(offsets):
        f_mm, r_mm = offsets[mid]
        print(f"  id {mid} ({CORNER_IDS[mid]} corner): "
              f"file {f_mm:+.1f} mm, rank {r_mm:+.1f} mm")
    print("\nPrint at 100% / 'Actual size'; verify a square measures "
          f"{args.square_mm:g} mm before use.")
    return 0


if __name__ == "__main__":
    main()
