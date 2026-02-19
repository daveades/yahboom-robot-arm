#!/usr/bin/env python3
import argparse
import json
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Convert LabelMe JSON to YOLO txt")
    parser.add_argument("--base", required=True, help="Base folder to scan for .json files")
    parser.add_argument("--classes", required=True, help="Path to classes.txt")
    return parser.parse_args()


def load_classes(path: Path) -> dict:
    names = [line.strip() for line in path.read_text().splitlines() if line.strip()]
    return {name: idx for idx, name in enumerate(names)}


def clamp(val: float, low: float, high: float) -> float:
    return max(low, min(high, val))


def main() -> int:
    args = parse_args()
    base = Path(args.base).expanduser().resolve()
    classes_path = Path(args.classes).expanduser().resolve()

    if not classes_path.exists():
        raise SystemExit(f"Classes file not found: {classes_path}")

    class_map = load_classes(classes_path)
    json_files = list(base.rglob("*.json"))
    if not json_files:
        raise SystemExit(f"No LabelMe JSON files found under: {base}")

    converted = 0
    skipped = 0

    for jf in json_files:
        try:
            data = json.loads(jf.read_text())
        except Exception:
            skipped += 1
            continue

        img_w = data.get("imageWidth")
        img_h = data.get("imageHeight")
        shapes = data.get("shapes", [])

        if not img_w or not img_h or not shapes:
            skipped += 1
            continue

        lines = []
        for shape in shapes:
            label = shape.get("label")
            points = shape.get("points", [])
            shape_type = shape.get("shape_type", "rectangle")

            if label not in class_map:
                continue
            if len(points) < 2:
                continue

            if shape_type == "rectangle":
                (x1, y1), (x2, y2) = points[0], points[1]
            else:
                # Fallback: use min/max over all points
                xs = [p[0] for p in points]
                ys = [p[1] for p in points]
                x1, x2 = min(xs), max(xs)
                y1, y2 = min(ys), max(ys)

            x1 = clamp(float(x1), 0.0, float(img_w))
            x2 = clamp(float(x2), 0.0, float(img_w))
            y1 = clamp(float(y1), 0.0, float(img_h))
            y2 = clamp(float(y2), 0.0, float(img_h))

            if x2 <= x1 or y2 <= y1:
                continue

            cx = (x1 + x2) / 2.0 / img_w
            cy = (y1 + y2) / 2.0 / img_h
            bw = (x2 - x1) / img_w
            bh = (y2 - y1) / img_h

            cls_id = class_map[label]
            lines.append(f"{cls_id} {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f}")

        if not lines:
            skipped += 1
            continue

        out_txt = jf.with_suffix(".txt")
        out_txt.write_text("\n".join(lines) + "\n")
        converted += 1

    print(f"Converted: {converted}, Skipped: {skipped}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
