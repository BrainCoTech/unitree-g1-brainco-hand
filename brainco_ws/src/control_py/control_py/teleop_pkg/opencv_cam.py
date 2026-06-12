import pyrealsense2 as rs
import numpy as np
from dataclasses import dataclass

from loguru import logger
from control_py.utils.loguru_settings import setup_loguru, logger_with_params
setup_loguru(log_folder_path="log", show_on_terminal=True, terminal_level='DEBUG') # 设置日志



@dataclass
class CameraConfig:
    device: str  # 可以是串号，也可以用 "unknown-device"
    width: int
    height: int
    fps: int

class OpenCVCamera:
    """
    只读取 RGB 流，替代 OpenCV VideoCapture
    """
    def __init__(self, config: CameraConfig):
        self.config = config
        self.pipeline = None

    def connect(self):
        self.pipeline = rs.pipeline()
        cfg = rs.config()

        # 如果指定了串号
        if self.config.device != "unknown-device":
            cfg.enable_device(self.config.device)

        # 启用 color 流
        cfg.enable_stream(rs.stream.color,
                          self.config.width,
                          self.config.height,
                          rs.format.bgr8,
                          self.config.fps)
        try:
            self.pipeline.start(cfg)
        except RuntimeError as e:
            raise RuntimeError(f"[RealSenseCamera] Cannot start device {self.config.device}: {e}")

        logger.info(f"[RealSenseCamera] Connected {self.config.device}")

    def read(self):
        """
        返回 RGB 图像 (BGR np.array)
        如果读取失败返回 None
        """
        if self.pipeline is None:
            return None

        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=2000)
            color_frame = frames.get_color_frame()
            if not color_frame:
                return None
            color_np = np.asanyarray(color_frame.get_data())
            return color_np
        except Exception as e:
            logger.warning(f"[RealSenseCamera] Read failed: {e}")
            return None

    def disconnect(self):
        if self.pipeline:
            self.pipeline.stop()
            self.pipeline = None
            logger.info(f"[RealSenseCamera] Disconnected {self.config.device}")



# import cv2
# from dataclasses import dataclass

# from loguru import logger
# from control_py.utils.loguru_settings import setup_loguru, logger_with_params
# setup_loguru(log_folder_path="log", show_on_terminal=True, terminal_level='DEBUG') # 设置日志

# @dataclass
# class CameraConfig:
#     device: str
#     width: int
#     height: int
#     fps: int


# class OpenCVCamera:
#     def __init__(self, config: CameraConfig):
#         self.config = config
#         self.device = config.device
#         self.cap = None

#     def connect(self):
#         self.cap = cv2.VideoCapture(self.config.device)

#         if not self.cap.isOpened():
#             raise RuntimeError(f"Cannot open camera {self.config.device}")

#         self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.config.width)
#         self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config.height)
#         self.cap.set(cv2.CAP_PROP_FPS, self.config.fps)

#     def read(self):
#         if self.cap is None:
#             return None

#         ret, frame = self.cap.read()
#         if not ret:
#             return None
#         return frame

#     def close(self):
#         if self.cap is not None:
#             self.cap.release()
#             self.cap = None




def _get_cam_device(cam) -> str:
    """
    Return camera device path if available.
    """
    for attr in (
        "device",
        "device_path",
        "path",
    ):
        if hasattr(cam, attr):
            val = getattr(cam, attr)
            if isinstance(val, str):
                return val

    return "unknown-device"


def shutdown_dict(cam_dict, name: str):
    if not cam_dict:
        logger_with_params(
            "[Camera] '{}' exists but empty, nothing to shutdown",
            name,
            level="INFO"
        )
        return

    for cam_key, cam in list(cam_dict.items()):
        dev = _get_cam_device(cam)

        logger.info(f"[Camera] shutting down {name}.{cam_key} ({dev})")

        try:
            if hasattr(cam, "disconnect"):
                cam.disconnect()
        except Exception as e:
            logger.warning(
                f"[Camera] {name}.{cam_key} ({dev}) disconnect failed: {e}"
            )

        for attr in ("cap", "_cap", "video", "_video"):
            if hasattr(cam, attr):
                try:
                    getattr(cam, attr).release()
                except Exception:
                    pass

    cam_dict.clear()
    logger.info(f"[Camera] '{name}' shutdown successfully")


