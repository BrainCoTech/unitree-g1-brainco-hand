import os
import threading

import numpy as np
from loguru import logger


class VisionStatesHandler:
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self._shared_yolo_seg_model = None
        self._shared_yolo_seg_model_path = "src/control_py/control_py/state_manager/calibrate/model/yolo/"
        self._shared_yolo_seg_model_name = "yolo11m-seg"
        self._shared_yolo_seg_model_warmed = False
        self.eye = None
        self.hand_eye = None
        self._cam_lock = threading.Lock()
        self._eye_lock = threading.Lock()
        self._vision_thread = None
        self._vision_running = False
        self._vision_done = False
        self._vision_finalize_done = False
        self._parallel_grasp_ready_vision = False
        self._grasp_parallel_vision_started = False
        self._parallel_pour_ready_vision = False
        self._pour_parallel_vision_started = False
        self._vision_task_mode = None
        self._grasp_motion_done = False
        self._release_motion_done = False
        self._hand_gesture_done = False
        self._pour_motion_done = False
        self.left_target_pos = None
        self.right_target_pos = None
        self.left_hand_target_pos = None
        self.right_hand_target_pos = None
        self.bottle_pos = None
        self.cup_pos = None
        self.grasp_handside = "both"

    def _has_active_vision_actions(self):
        vision_actions = {"Vision-Apple", "Vision-Bottle-Cup"}
        return any(action in vision_actions for action in self.action_name.values())

    def _get_shared_yolo_seg_model(self, warmup=True):
        if self._shared_yolo_seg_model is None:
            from control_py.state_manager.calibrate.vision import YOLO

            yolo_path = os.path.join(
                self._shared_yolo_seg_model_path,
                f"{self._shared_yolo_seg_model_name}.pt",
            )
            if not os.path.exists(yolo_path):
                logger.error(f"Model file not found: {yolo_path}")
                raise FileNotFoundError(yolo_path)

            self._shared_yolo_seg_model = YOLO(yolo_path)
            logger.info(
                f"Shared model {self._shared_yolo_seg_model_name} loaded successfully"
            )

        if warmup and not self._shared_yolo_seg_model_warmed:
            self._shared_yolo_seg_model.predict(np.ones((1, 1, 3)))
            self._shared_yolo_seg_model_warmed = True
            logger.info(
                f"Shared model {self._shared_yolo_seg_model_name} warmup finished"
            )

        return self._shared_yolo_seg_model

    # 关闭 视觉相关线程与相机资源，避免状态切换时残留后台任务。
    def _shutdown_vision_resources(self):
        if hasattr(self, "shutdown_vision_cameras"):
            self.shutdown_vision_cameras()

    # 启动后台视觉线程执行检测和图像发布。
    def _start_vision_thread(self):
        self._vision_running = True
        self._vision_thread = threading.Thread(
            target=self._vision_loop,
            daemon=True,
        )
        self._vision_thread.start()
