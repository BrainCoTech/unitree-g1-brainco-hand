import pyrealsense2 as rs
import cv2
from pathlib import Path
import numpy as np

OUTPUT_DIR = Path("_test_cams")

pipeline = rs.pipeline()
config = rs.config()
x, y, fps = 1920, 1080, 30
config.enable_stream(rs.stream.color, x, y, rs.format.bgr8, fps)
pipeline.start(config)

try:
    # 等待相机稳定
    for _ in range(10):
        pipeline.wait_for_frames()

    # 获取一帧
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()

    if not color_frame:
        print("Failed to capture image")
    else:
        image = np.asanyarray(color_frame.get_data())

        # 保存图片
        OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
        filename = OUTPUT_DIR / "capture.jpg"
        cv2.imwrite(filename, image)

        print(f"Image saved as {filename}")

finally:
    pipeline.stop()