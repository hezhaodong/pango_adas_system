#!/usr/bin/env python3
"""Create a deterministic CCPD2019 random-sample manifest without reading images."""

from __future__ import annotations

import argparse
import random
from pathlib import Path


SUBSETS = (
    "ccpd_blur",
    "ccpd_challenge",
    "ccpd_db",
    "ccpd_fn",
    "ccpd_rotate",
    "ccpd_tilt",
    "ccpd_weather",
)

PROVINCES = (
    "皖", "沪", "津", "渝", "冀", "晋", "蒙", "辽", "吉", "黑", "苏",
    "浙", "京", "闽", "赣", "鲁", "豫", "鄂", "湘", "粤", "桂", "琼",
    "川", "贵", "云", "藏", "陕", "甘", "青", "宁", "新",
)
ALPHABETS = tuple("ABCDEFGH") + tuple("JKLMN") + tuple("PQRSTUVWXYZ")
ADS = ALPHABETS + tuple("0123456789")


def parse_label(filename: str) -> str:
    parts = Path(filename).stem.split("-")
    if len(parts) < 5:
        raise ValueError("not enough '-' fields")
    codes = [int(value) for value in parts[4].split("_")]
    if len(codes) != 7:
        raise ValueError("plate field must contain 7 indices")
    return (
        PROVINCES[codes[0]]
        + ALPHABETS[codes[1]]
        + "".join(ADS[index] for index in codes[2:])
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--per-subset", type=int, default=1000)
    parser.add_argument("--seed", type=int, default=20260802)
    args = parser.parse_args()

    rng = random.Random(args.seed)
    lines = [
        "# CCPD2019 end-to-end sample manifest",
        f"# seed={args.seed}",
        f"# per_subset={args.per_subset}",
        "# labels are intentionally omitted; the board program parses each filename",
    ]
    total = 0
    for subset in SUBSETS:
        subset_dir = args.dataset / subset
        images = sorted(
            path
            for path in subset_dir.iterdir()
            if path.is_file() and path.suffix.lower() in {".jpg", ".jpeg", ".png"}
        )
        if len(images) < args.per_subset:
            raise RuntimeError(
                f"{subset} has {len(images)} images, fewer than {args.per_subset}"
            )
        selected = rng.sample(images, args.per_subset)
        selected.sort(key=lambda path: path.name)
        for path in selected:
            parse_label(path.name)
            lines.append(f"{subset}/{path.name}")
        total += len(selected)
        print(f"{subset}: available={len(images)}, selected={len(selected)}")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text("\n".join(lines) + "\n", encoding="utf-8", newline="\n")
    print(f"total={total}")
    print(f"manifest={args.output.resolve()}")
    for line in lines[4:9]:
        print(f"sample: {line} -> {parse_label(Path(line).name)}")


if __name__ == "__main__":
    main()
