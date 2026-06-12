#!/usr/bin/env python3
from __future__ import annotations

import sys
from pathlib import Path

THIS_FILE = Path(__file__).resolve()
PACKAGE_ROOT = THIS_FILE.parents[3]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

import cv2
import numpy as np
import pyrealsense2 as rs
import yaml
from loguru import logger
from ultralytics import YOLO

from control_py.utils.loguru_settings import setup_loguru

setup_loguru(log_folder_path="log", show_on_terminal=True, terminal_level="DEBUG")


MODEL_DIR = THIS_FILE.parent / "model" / "yolo"
CLASSES_CONFIG_PATH = THIS_FILE.parent / "yolo11_classes.yaml"
DEFAULT_MODEL_NAME = "yolo11m-seg"
OUTPUT_PATH = THIS_FILE.parent / "test_model.jpg"


def resolve_model_path() -> Path:
    preferred_model = MODEL_DIR / f"{DEFAULT_MODEL_NAME}.pt"
    if preferred_model.exists():
        return preferred_model

    candidates = sorted(MODEL_DIR.glob("*.pt"))
    if not candidates:
        raise FileNotFoundError(f"No model file found in {MODEL_DIR}")

    logger.warning(
        f"Preferred model {preferred_model.name} not found, fallback to {candidates[0].name}"
    )
    return candidates[0]


def load_class_config() -> tuple[list[str], list[int]]:
    if not CLASSES_CONFIG_PATH.exists():
        raise FileNotFoundError(f"Class config not found: {CLASSES_CONFIG_PATH}")

    with CLASSES_CONFIG_PATH.open("r", encoding="utf-8") as file:
        config = yaml.safe_load(file) or {}

    classes_map = config.get("classes")
    if not isinstance(classes_map, dict) or not classes_map:
        raise ValueError(f"Invalid class config in {CLASSES_CONFIG_PATH}")

    sorted_items = sorted((int(class_id), str(name)) for class_id, name in classes_map.items())
    class_ids = [class_id for class_id, _ in sorted_items]
    class_names = [name for _, name in sorted_items]
    return class_names, class_ids


def get_valid_class_filter(
    model: YOLO, requested_classes: list[str], requested_class_ids: list[int]
) -> tuple[list[str], list[int] | None]:
    model_names = getattr(model, "names", {}) or {}
    available_items = [
        (class_name, class_id)
        for class_name, class_id in zip(requested_classes, requested_class_ids)
        if class_id in model_names
    ]

    if not available_items:
        logger.warning(
            f"No configured classes from {CLASSES_CONFIG_PATH.name} are available in this model, fallback to full-model detection"
        )
        return requested_classes, None

    available_classes = [class_name for class_name, _ in available_items]
    available_class_ids = [class_id for _, class_id in available_items]
    missing_count = len(requested_class_ids) - len(available_class_ids)
    if missing_count > 0:
        logger.warning(
            f"{missing_count} configured classes are unavailable in this model and will be skipped"
        )

    return available_classes, available_class_ids


def predict_once(model: YOLO, image: np.ndarray, classes_id: list[int] | None):
    predict_kwargs = {
        "conf": 0.7,
        "verbose": False,
    }
    if classes_id is not None:
        predict_kwargs["classes"] = classes_id

    last_error = None
    for device in (0, "cpu"):
        try:
            return model.predict(image, device=device, **predict_kwargs)[0]
        except Exception as exc:  # noqa: BLE001
            last_error = exc
            logger.warning(f"Predict failed on device={device}: {exc}")

    raise RuntimeError(f"YOLO prediction failed: {last_error}")


def draw_summary(image: np.ndarray, result, requested_classes: list[str]) -> np.ndarray:
    annotated = result.plot()
    boxes = getattr(result, "boxes", None)
    summary_lines: list[str] = []

    if boxes is not None and len(boxes) > 0:
        counts: dict[str, int] = {}
        for class_idx in boxes.cls.tolist():
            class_name = result.names.get(int(class_idx), str(int(class_idx)))
            counts[class_name] = counts.get(class_name, 0) + 1
        summary_lines = [f"{name}: {count}" for name, count in sorted(counts.items())]
    else:
        summary_lines = [f"No detections ({', '.join(requested_classes)})"]

    for idx, line in enumerate(summary_lines):
        y = 30 + idx * 30
        cv2.putText(
            annotated,
            line,
            (10, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 0, 0),
            4,
            cv2.LINE_AA,
        )
        cv2.putText(
            annotated,
            line,
            (10, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )

    return annotated


def brighten_if_needed(image: np.ndarray, min_mean_brightness: float = 90.0) -> np.ndarray:
    mean_brightness = float(image.mean())
    if mean_brightness >= min_mean_brightness:
        return image

    gain = min_mean_brightness / max(mean_brightness, 1.0)
    gain = min(gain, 2.5)
    logger.warning(
        f"Captured image is dark (mean={mean_brightness:.1f}), applying brightness gain {gain:.2f}"
    )
    return cv2.convertScaleAbs(image, alpha=gain, beta=12)


class CalibrCameraRunner:
    def __init__(self):
        self.cam_pipeline = None
        self.started = False

    def calibr_camera_init(self):
        self.cam_pipeline = rs.pipeline()
        config = rs.config()
        x, y, fps = 640, 360, 30
        config.enable_stream(rs.stream.color, x, y, rs.format.bgr8, fps)
        try:
            profile = self.cam_pipeline.start(config)
            self.started = True
            self._configure_color_sensor(profile)
        except RuntimeError as exc:
            raise RuntimeError(
                "Failed to start RealSense camera: No device connected or the device is busy"
            ) from exc

    def _configure_color_sensor(self, profile):
        try:
            device = profile.get_device()
            for sensor in device.query_sensors():
                sensor_name = sensor.get_info(rs.camera_info.name).lower()
                if "color" not in sensor_name:
                    continue
                if sensor.supports(rs.option.enable_auto_exposure):
                    sensor.set_option(rs.option.enable_auto_exposure, 1)
                if sensor.supports(rs.option.enable_auto_white_balance):
                    sensor.set_option(rs.option.enable_auto_white_balance, 1)
                if sensor.supports(rs.option.brightness):
                    current_brightness = sensor.get_option(rs.option.brightness)
                    sensor.set_option(rs.option.brightness, current_brightness)
                logger.info(f"Configured color sensor: {sensor_name}")
                break
        except Exception as exc:  # noqa: BLE001
            logger.warning(f"Failed to configure camera exposure settings: {exc}")

    def capture_image(self, warmup_frames: int = 30) -> np.ndarray:
        if self.cam_pipeline is None:
            raise RuntimeError("Camera pipeline is not initialized")

        image = None
        for _ in range(max(1, warmup_frames)):
            frames = self.cam_pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue
            image = np.asanyarray(color_frame.get_data())

        if image is None:
            raise RuntimeError("Failed to capture RealSense color frame")

        return image

    def shutdown_calibr_cameras(self):
        if self.cam_pipeline is not None and self.started:
            try:
                self.cam_pipeline.stop()
            except Exception as exc:  # noqa: BLE001
                logger.warning(f"Failed to stop camera cleanly: {exc}")
            finally:
                self.started = False
        self.cam_pipeline = None


def main() -> int:
    camera = None
    try:
        configured_classes, configured_class_ids = load_class_config()
        model_path = resolve_model_path()
        logger.info(f"Loading model: {model_path}")
        model = YOLO(str(model_path))
        requested_classes, classes_id = get_valid_class_filter(
            model, configured_classes, configured_class_ids
        )

        logger.info("Warming up model")
        predict_once(model, np.ones((32, 32, 3), dtype=np.uint8), classes_id)

        logger.info("Starting RealSense camera")
        camera = CalibrCameraRunner()
        camera.calibr_camera_init()
        image = camera.capture_image()
        image = brighten_if_needed(image)

        logger.info("Running single-shot detection")
        result = predict_once(model, image, classes_id)
        output_image = draw_summary(image, result, requested_classes)

        if not cv2.imwrite(str(OUTPUT_PATH), output_image):
            raise RuntimeError(f"Failed to save image to {OUTPUT_PATH}")

        logger.info(f"Saved detection result to {OUTPUT_PATH}")
        return 0
    except Exception as exc:  # noqa: BLE001
        logger.exception(f"test_model failed: {exc}")
        return 1
    finally:
        if camera is not None:
            camera.shutdown_calibr_cameras()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    raise SystemExit(main())
