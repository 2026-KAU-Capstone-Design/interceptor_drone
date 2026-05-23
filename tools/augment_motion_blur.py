#!/usr/bin/env python3

from pathlib import Path
import random
import shutil

import cv2
import numpy as np


src = Path.home() / "datasets/balloon_yolo_final"
dst = Path.home() / "datasets/balloon_yolo_blur"

for split in ["train", "val"]:
    (dst / "images" / split).mkdir(parents=True, exist_ok=True)
    (dst / "labels" / split).mkdir(parents=True, exist_ok=True)

counter = 0

def motion_blur(image, ksize=15, angle=0):
    kernel = np.zeros((ksize, ksize), dtype=np.float32)
    kernel[ksize // 2, :] = 1.0
    kernel /= ksize

    center = (ksize // 2, ksize // 2)
    matrix = cv2.getRotationMatrix2D(center, angle, 1.0)
    kernel = cv2.warpAffine(kernel, matrix, (ksize, ksize))

    return cv2.filter2D(image, -1, kernel)

for split in ["train", "val"]:
    img_dir = src / "images" / split
    lbl_dir = src / "labels" / split

    for img_path in sorted(img_dir.glob("*.jpg")):
        if random.random() > 0.25:
            continue

        label_path = lbl_dir / f"{img_path.stem}.txt"
        if not label_path.exists():
            continue

        image = cv2.imread(str(img_path))
        if image is None:
            continue

        ksize = random.choice([7, 9, 11, 15])
        angle = random.choice([0, 30, 60, 90, 120, 150])
        blurred = motion_blur(image, ksize=ksize, angle=angle)

        new_name = f"blur_{counter:06d}"
        cv2.imwrite(str(dst / "images" / split / f"{new_name}.jpg"), blurred)
        shutil.copy2(label_path, dst / "labels" / split / f"{new_name}.txt")

        counter += 1

print(f"created {counter} motion-blur images in {dst}")

