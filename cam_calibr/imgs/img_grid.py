import cv2
import os
import numpy as np
import sys
from pathlib import Path


def make_image_grid(folder_path, rows=4, cols=5, prefix="calibr_img_", ext=".jpg"):
    images = []

    for i in range(1, rows * cols + 1):
        img_path = os.path.join(folder_path, f"{prefix}{i}{ext}")
        img = cv2.imread(img_path)

        if img is None:
            raise FileNotFoundError(f"Image not found: {img_path}")

        images.append(img)

    # 统一尺寸
    h, w = images[0].shape[:2]
    images = [cv2.resize(img, (w, h)) for img in images]

    # 拼接
    grid_rows = []
    for r in range(rows):
        row_imgs = images[r * cols:(r + 1) * cols]
        row = np.hstack(row_imgs)
        grid_rows.append(row)

    grid_img = np.vstack(grid_rows)

    return grid_img


def main():

    if len(sys.argv) < 2:
        print("Usage: python img_grid.py [left|right]")
        return

    side = sys.argv[1]

    base_path = Path(__file__).resolve().parent
    folder_path = base_path / side

    grid = make_image_grid(str(folder_path), rows=4, cols=5)

    save_path = base_path / f"20_grid_{side}.jpg"
    cv2.imwrite(str(save_path), grid)

    print(f"Saved grid image to: {save_path}")


if __name__ == "__main__":
    main()
