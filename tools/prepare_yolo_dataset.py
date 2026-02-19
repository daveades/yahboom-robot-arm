#!/usr/bin/env python3
import argparse
import random
import shutil
from pathlib import Path


IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp"}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Prepare YOLO dataset from color folders")
    parser.add_argument("--base", required=True, help="Base folder with color subfolders")
    parser.add_argument("--out", required=True, help="Output dataset folder")
    parser.add_argument("--val", type=float, default=0.2, help="Validation split ratio")
    parser.add_argument("--seed", type=int, default=42, help="Random seed")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    base = Path(args.base).expanduser().resolve()
    out = Path(args.out).expanduser().resolve()

    class_names = ["red_cube", "blue_cube", "green_cube", "yellow_cube"]
    folder_map = {
        "red": 0,
        "blue": 1,
        "green": 2,
        "yellow": 3,
    }

    samples = []

    for subdir in base.iterdir():
        if not subdir.is_dir():
            continue
        key = subdir.name.lower()
        if key not in folder_map:
            continue
        for img in subdir.iterdir():
            if img.suffix.lower() not in IMAGE_EXTS:
                continue
            label = img.with_suffix(".txt")
            if not label.exists():
                print(f"Skipping (no label): {img}")
                continue
            samples.append((img, label))

    if not samples:
        raise SystemExit("No labeled images found. Make sure LabelImg saved .txt files.")

    random.seed(args.seed)
    random.shuffle(samples)

    val_count = int(len(samples) * args.val)
    val_samples = samples[:val_count]
    train_samples = samples[val_count:]

    def ensure_dirs(split: str) -> None:
        (out / "images" / split).mkdir(parents=True, exist_ok=True)
        (out / "labels" / split).mkdir(parents=True, exist_ok=True)

    for split, split_samples in ("train", train_samples), ("val", val_samples):
        ensure_dirs(split)
        for img, label in split_samples:
            shutil.copy2(img, out / "images" / split / img.name)
            shutil.copy2(label, out / "labels" / split / label.name)

    data_yaml = out / "data.yaml"
    data_yaml.write_text(
        "\n".join(
            [
                f"path: {out}",
                "train: images/train",
                "val: images/val",
                "names:",
            ]
            + [f"  {i}: {name}" for i, name in enumerate(class_names)]
        )
        + "\n"
    )

    print(f"Prepared dataset at {out}")
    print(f"Train: {len(train_samples)} images, Val: {len(val_samples)} images")
    print(f"data.yaml: {data_yaml}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
