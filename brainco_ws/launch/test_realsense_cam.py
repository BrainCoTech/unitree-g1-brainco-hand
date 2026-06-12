#!/usr/bin/env python
"""快速测试脚本：找到 RealSense D435 的 RGB 通道"""

import cv2
from pathlib import Path


TEST_DEVICES = [f"/dev/video{i}" for i in range(8)]
OUTPUT_DIR = Path("_test_cams")
GREEN = "\033[32m"
RESET = "\033[0m"
GRAY  = "\033[90m"

def test_device(device_path: str) -> None:
    print(f"Test device: {device_path}", end="")

    cap = cv2.VideoCapture(device_path)
    if not cap.isOpened():
        print(f" -> 无法打开 {device_path}")
        return

    ret, frame = cap.read()
    cap.release()

    if not ret or frame is None:
        print(f"  ❌ 无法从 {device_path} 读取帧")
        return

    print("  ✅ 成功读取帧")
    print(f"    {frame.shape}, {frame.dtype}, range: [{frame.min()}, {frame.max()}]")

    if frame.ndim == 3 and frame.shape[2] == 3 and not (frame[:,:,0] == frame[:,:,1]).all():
        print(f"    {GREEN}彩色图{RESET}", end="")
    else:
        print(f"    {GRAY}深度图{RESET}", end="")

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    output_path = OUTPUT_DIR / f"test_{device_path.replace('/', '_')}.png"
    cv2.imwrite(str(output_path), frame)
    print(f"已保存图像到: {output_path}")


def main():
    print("测试 RealSense D435 相机设备...")
    print("=" * 30)

    for device in TEST_DEVICES:
        try:
            test_device(device)
        except Exception as e:
            print(f"  ❌ 错误: {e}")

    print("\n" + "=" * 30)
    print("测试完成！\n")

if __name__ == "__main__":
    main()
