from control_py.utils.utils import *
import numpy as np

from loguru import logger
from control_py.utils.loguru_settings import setup_loguru
setup_loguru(log_folder_path="log", show_on_terminal=True, terminal_level='DEBUG') # 设置日志

from control_py.state_manager.basic_states import BasicStatesHandler
from control_py.state_manager.common import ACTION, ReplayStatesHandler, VisionStatesHandler
from control_py.state_manager.calibrate.vision import Eye
from control_py.state_manager.eeg.vision_hand import VisionHand

class EEGTasksHandler(BasicStatesHandler, ReplayStatesHandler, VisionStatesHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    # 从状态机参数中解析抓取使用的手侧，默认允许双手模式。
    def _resolve_grasp_handside(self):
        if self.param.handside in ("left", "right"):
            return self.param.handside
        return "both"

    # 根据手侧选择抓取任务需要检测的类别和类别 ID。
    def _get_grasp_detection_classes(self, handside: str):
        classes = ["apple", "orange"]
        classes_id = [47, 49]
        if handside == "left":
            classes = ["orange"]
            classes_id = [49]
        elif handside == "right":
            classes = ["apple"]
            classes_id = [47]
        return classes, classes_id

    # 重置抓苹果流程中的视觉、动作和目标点缓存。
    def _reset_grasp_sequence_state(self):
        self._vision_done = False
        self._grasp_motion_done = False
        self._release_motion_done = False
        self._hand_gesture_done = False
        self._vision_finalize_done = False
        self._parallel_grasp_ready_vision = False
        self._grasp_parallel_vision_started = False
        self._vision_task_mode = "grasp"
        self.left_target_pos = None
        self.right_target_pos = None
        self.left_hand_target_pos = None
        self.right_hand_target_pos = None

    # 初始化抓苹果流程的视觉对象，并按需附带手势识别模块。
    def _setup_grasp_eye(self, handside: str, with_hand_eye: bool = False):
        model_path = "src/control_py/control_py/state_manager/calibrate/model/yolo/"
        model_name = "yolo11m-seg"
        classes, classes_id = self._get_grasp_detection_classes(handside)
        self.eye = Eye(
            model_path,
            model_name,
            classes,
            classes_id,
            yolo_model=self._get_shared_yolo_seg_model(),
        )
        if with_hand_eye:
            self.hand_eye = VisionHand()
        self._vision_task_mode = "grasp"
        return classes

    # 重置倒水流程中的视觉、动作和目标点缓存。
    def _reset_pour_sequence_state(self):
        self._vision_done = False
        self._grasp_motion_done = False
        self._release_motion_done = False
        self._hand_gesture_done = False
        self._vision_finalize_done = False
        self._parallel_pour_ready_vision = False
        self._pour_parallel_vision_started = False
        self._pour_motion_done = False
        self._vision_task_mode = "pour"
        self.bottle_pos = None
        self.cup_pos = None

    # 初始化倒水流程的视觉对象。
    def _setup_pour_eye(self):
        model_path = "src/control_py/control_py/state_manager/calibrate/model/yolo/"
        model_name = "yolo11m-seg"
        classes = ["bottle", "cup"]
        classes_id = [39, 41]
        self.eye = Eye(
            model_path,
            model_name,
            classes,
            classes_id,
            yolo_model=self._get_shared_yolo_seg_model(),
        )
        self._vision_task_mode = "pour"
        return classes

    # 在抓取序列中提前启动目标检测，供并行行为树复用。
    def _start_grasp_object_vision(self, handside: str):
        if self._grasp_parallel_vision_started:
            return

        self._setup_grasp_eye(handside, with_hand_eye=False)
        self.eye.initialize_obj_coordinate(self.eye.classes)
        self.eye.obj_detect = True
        self._start_vision_thread()
        self._grasp_parallel_vision_started = True

    # 在倒水序列中提前启动 bottle/cup 检测，供并行行为树复用。
    def _start_pour_object_vision(self):
        if self._pour_parallel_vision_started:
            return

        self._setup_pour_eye()
        self.eye.initialize_obj_coordinate(self.eye.classes)
        self.eye.obj_detect = True
        self._start_vision_thread()
        self._pour_parallel_vision_started = True

    # 统一封装打招呼动作的进入逻辑。
    def _on_active_hello_handler(self, action_idx: int, handside: str):
        logger.info(f"Enter state {self.sm.get_state()} '{self.action_name[action_idx]}'")
        self.clear_timer()
        self._shutdown_vision_resources()
        self.update_cmd_buffer("both")
        self._timer = self.create_timer(0.01, lambda: self.timer_hello(handside=handside))
        self._track_active_action(
            action_idx,
            expected_duration=self._get_expected_duration(action_idx)
        )
        self.get_logger().info("New timer created.")

    # 统一封装点赞动作的进入逻辑。
    def _on_active_like_handler(self, action_idx: int, handside: str):
        logger.info(f"Enter state {self.sm.get_state()} '{self.action_name[action_idx]}'")
        self.clear_timer()
        self._shutdown_vision_resources()
        self.update_cmd_buffer("both")
        self._timer = self.create_timer(0.01, lambda: self.timer_like(handside=handside))
        self._track_active_action(
            action_idx,
            expected_duration=self._get_expected_duration(action_idx)
        )
        self.get_logger().info("New timer created.")

    # 统一封装石头剪刀布动作的进入逻辑和公共参数初始化。
    def _on_active_rps_handler(self, action_idx: int, handside: str):
        logger.info(f"Enter state {self.sm.get_state()} '{self.action_name[action_idx]}'")
        self.clear_timer()
        self._shutdown_vision_resources()
        self.update_cmd_buffer("both")

        self.curr_note_left = 0
        self.curr_note_right = 0
        self.gesture_list = ["石头", "剪刀", "布"]
        self.loop = 100
        self.interval_move = 0.5
        self.interval_stop = 2.0
        self.prepare = 0.0
        self.end_ts = (
            self.prepare + 1.0 +
            (self.interval_move * 6 + self.interval_stop) * self.loop -
            self.interval_move
        )

        self._timer = self.create_timer(0.01, lambda: self.timer_rps(handside=handside))
        self._track_active_action(
            action_idx,
            expected_duration=self._get_expected_duration(action_idx, fallback=self.end_ts)
        )
        self.get_logger().info("New timer created.")

    # 统一封装握手动作的进入逻辑。
    def _on_active_handshake_handler(self, action_idx: int, handside: str):
        logger.info(f"Enter state {self.sm.get_state()} '{self.action_name[action_idx]}'")
        self.clear_timer()
        self._shutdown_vision_resources()
        self.update_cmd_buffer("both")
        self._timer = self.create_timer(0.01, lambda: self.timer_hand_shake(handside=handside))
        self._track_active_action(
            action_idx,
            expected_duration=self._get_expected_duration(action_idx)
        )
        self.get_logger().info("New timer created.")


    # Active 1 手臂放下
    def on_active_1_handler(self):
        logger.info(f"Enter state {self.sm.get_state()} '{self.action_name[1]}'")
        self.clear_timer()
        self._shutdown_vision_resources()
        self.update_cmd_buffer("both")
        self._timer = self.create_timer(0.01, self.timer_back)
        self._track_active_action(
            1,
            expected_duration=self._get_expected_duration(1)
        )
        self.get_logger().info(f"New timer created.")

    # Active 10 手臂放下
    def on_active_10_handler(self):
        logger.info(f"Enter state {self.sm.get_state()} '{self.action_name[10]}'")
        self.clear_timer()
        self._shutdown_vision_resources()
        self.update_cmd_buffer("both")
        self._timer = self.create_timer(0.01, self.timer_back)
        self._track_active_action(
            10,
            expected_duration=self._get_expected_duration(10)
        )
        self.get_logger().info(f"New timer created.")

    # Active 11 打招呼（left）
    def on_active_11_handler(self):
        self._on_active_hello_handler(11, "left")

    # Active 12 打招呼（right）
    def on_active_12_handler(self):
        self._on_active_hello_handler(12, "right")

    # Active 13 点赞（left）
    def on_active_13_handler(self):
        self._on_active_like_handler(13, "left")

    # Active 14 点赞（right）
    def on_active_14_handler(self):
        self._on_active_like_handler(14, "right")

    # Active 15 石头剪刀布（left）
    def on_active_15_handler(self):
        self._on_active_rps_handler(15, "left")

    # Active 16 石头剪刀布（right）
    def on_active_16_handler(self):
        self._on_active_rps_handler(16, "right")

    # Active 17 握手（left）
    def on_active_17_handler(self):
        self._on_active_handshake_handler(17, "left")

    # Active 18 握手（right）
    def on_active_18_handler(self):
        self._on_active_handshake_handler(18, "right")
    
    # Active 19 比心
    def on_active_19_handler(self):
        logger.info(f"Enter state {self.sm.get_state()} '{self.action_name[19]}'")
        self.clear_timer()
        self._shutdown_vision_resources()
        self.update_cmd_buffer("both")
        self._timer = self.create_timer(0.01, self.timer_showheart)
        self._track_active_action(
            19,
            expected_duration=self._get_expected_duration(19)
        )
        self.get_logger().info(f"New timer created.")

    # Active 41 设置抓取手并进入下一步
    def on_active_41_handler(self):
        logger.info(f"Enter state {self.sm.get_state()} '{self.action_name[41]}'")
        self.clear_timer()
        self._shutdown_vision_resources()
        self.update_cmd_buffer("both")
        self.param.handside = "right"
        self._timer = self.create_timer(0.01, self.timer_wait_for_expected_duration)
        self._track_active_action(
            41,
            expected_duration=self._get_expected_duration(41, fallback=0.0)
        )

    # Active 42 原地左转 2 秒
    def on_active_42_handler(self):
        self._start_turn_action(42, "0.5 0 0 1", fallback_duration=3.2)

    # Active 31 原地左转 4 秒
    def on_active_31_handler(self):
        self._start_turn_action(31, "0 0 0.5 4", fallback_duration=5.2)

    # Active 32 原地右转 + 
    def on_active_32_handler(self):
        self._start_turn_action(32, "-0.1 0 -0.5 4", fallback_duration=6.2)

    # Active 34 前进 1 秒
    def on_active_34_handler(self):
        self._start_turn_action(34, "0.3 0 -0.45 1.5", fallback_duration=2.2)

    # Active 36 前进同时左转 2 秒
    def on_active_36_handler(self):
        self._start_turn_action(36, "0.25 0 0.5 2", fallback_duration=3.2)

    # Active 37 前进同时右转 2 秒
    def on_active_37_handler(self):
        self._start_turn_action(37, "0.25 0 -0.5 2", fallback_duration=3.2)

    def on_active_60_handler(self):
        self._start_turn_action(60, "-0.1 0 1.0 2", fallback_duration=3.2)

    def on_active_61_handler(self):
        self._start_turn_action(61, "-0.25 0.1 0 2", fallback_duration=3.2)


    # Active 71 左转 2 秒
    def on_active_71_handler(self):
        self._start_turn_action(71, "0. 0 1.0 2", fallback_duration=3.2)
        # Active 71 左转 2 秒
    def on_active_72_handler(self):
        self._start_turn_action(72, "0. 0 -1.0 2", fallback_duration=3.2)


    # Active 2 安全放下手臂（独立动作）
    def on_active_2_handler(self):
        logger.info(f"Enter state {self.sm.get_state()} '{self.action_name[2]}'")
        self.clear_timer()
        self._shutdown_vision_resources()
        self.update_cmd_buffer("both")

        lr_height = [self.eecmd_fk_buffer.left.pos[2], self.eecmd_fk_buffer.right.pos[2]]
        self._timer = self.create_timer(0.01, lambda: self.timer_safe_arm_down(lr_height))
        self._track_active_action(
            2,
            expected_duration=self._get_expected_duration(2, fallback=3.2)
        )
        self.get_logger().info("New timer created.")

    # Active 40 安全放下手臂
    def on_active_40_handler(self):
        logger.info(f"Enter state {self.sm.get_state()} '{self.action_name[40]}'")
        self.clear_timer()
        self._shutdown_vision_resources()
        self.update_cmd_buffer("both")

        lr_height = [self.eecmd_fk_buffer.left.pos[2], self.eecmd_fk_buffer.right.pos[2]]
        self._timer = self.create_timer(0.01, lambda: self.timer_safe_arm_down(lr_height))
        self._track_active_action(
            40,
            expected_duration=self._get_expected_duration(40, fallback=3.2)
        )
        self.get_logger().info("New timer created.")

    # Active 51 倒水准备姿态
    def on_active_51_handler(self):
        logger.info(f"Enter state {self.sm.get_state()} '{self.action_name[51]}'")
        self.clear_timer()
        self._shutdown_vision_resources()
        self.update_cmd_buffer("both")

        lr_height = [self.eecmd_fk_buffer.left.pos[2], self.eecmd_fk_buffer.right.pos[2]]
        self._reset_pour_sequence_state()
        self._parallel_pour_ready_vision = self._is_parallel_group_entry(51)
        if self._parallel_pour_ready_vision:
            # In pour_water_sequence, start bottle/cup detection while moving to the ready pose.
            self._start_pour_object_vision()
        self._timer = self.create_timer(0.01, lambda: self.timer_get_ready_pour_sequence(lr_height))
        self._track_active_action(
            51,
            expected_duration=self._get_expected_duration(51)
        )
        self.get_logger().info("New timer created.")

    # Active 52 视觉识别 bottle 和 cup
    def on_active_52_handler(self):
        logger.info(f"Enter state {self.sm.get_state()} '{self.action_name[52]}'")
        self.clear_timer()
        self.update_cmd_buffer("both")

        if self._is_parallel_group_finalizer(52) and (
            self._pour_parallel_vision_started or self._vision_done
        ):
            logger.info("Active 52 reuses the vision task started in Active 51.")
            self._timer = self.create_timer(0.01, self.timer_detect_pour_object)
            self._track_active_action(
                52,
                expected_duration=self._get_expected_duration(52)
            )
            self.get_logger().info("New timer created.")
            return

        self._shutdown_vision_resources()
        self._reset_pour_sequence_state()
        self._setup_pour_eye()
        self.eye.initialize_obj_coordinate(self.eye.classes)
        self.eye.obj_detect = True
        self._start_vision_thread()

        self._timer = self.create_timer(0.01, self.timer_detect_pour_object)
        self._track_active_action(
            52,
            expected_duration=self._get_expected_duration(52)
        )
        self.get_logger().info("New timer created.")

    # Active 53 倒水
    def on_active_53_handler(self):
        logger.info(f"Enter state {self.sm.get_state()} '{self.action_name[53]}'")
        self.clear_timer()
        self._shutdown_vision_resources()
        self.update_cmd_buffer("both")

        self._pour_motion_done = False
        self._timer = self.create_timer(0.01, self.timer_pour_water_motion)
        self._track_active_action(
            53,
            expected_duration=self._get_expected_duration(53)
        )
        self.get_logger().info("New timer created.")

    # Active 20 Biubiubiu
    def on_active_20_handler(self):
        logger.info(f"Enter state {self.sm.get_state()} '{self.action_name[20]}'")
        self.clear_timer()
        self._shutdown_vision_resources()
        self.update_cmd_buffer("both")

        # Replay Config
        self._replay_config(dataset_idx=0)

        # Replay Init
        self._replay_initialization()

        # ACTION mapping
        action_names = self.replay_dataset.features[ACTION]["names"]
        self._arm_action_mapping(action_names)
        self._hand_action_mapping(action_names)

        self._timer = self.create_timer(0.01, lambda: self.timer_replay(speed=0.33))
        replay_duration = self.replay_num_frames * self.control_dt_ / 0.33
        self._track_active_action(
            20,
            expected_duration=self._get_expected_duration(20, fallback=replay_duration)
        )
        self.get_logger().info(f"New timer created.")

    # Active 43 准备姿态
    def on_active_43_handler(self):
        logger.info(f"Enter state {self.sm.get_state()} '{self.action_name[43]}'")
        self.clear_timer()
        self._shutdown_vision_resources()
        self.update_cmd_buffer("both")

        self.grasp_handside = self._resolve_grasp_handside()
        self.grasp_handside = "right"
        lr_height = [self.eecmd_fk_buffer.left.pos[2], self.eecmd_fk_buffer.right.pos[2]]
        self._reset_grasp_sequence_state()
        self._parallel_grasp_ready_vision = self._is_parallel_group_entry(43)
        if self._parallel_grasp_ready_vision:
            # In grasp_apple_sequence, let object detection start while the arm moves to ready pose.
            self._start_grasp_object_vision(self.grasp_handside)
        self._timer = self.create_timer(0.01, lambda: self.timer_get_ready_sequence(lr_height, self.grasp_handside))
        self._track_active_action(
            43,
            expected_duration=self._get_expected_duration(43)
        )
        self.get_logger().info("New timer created.")

    # Active 44 视觉识别苹果/橙子
    def on_active_44_handler(self):
        logger.info(f"Enter state {self.sm.get_state()} '{self.action_name[44]}'")
        self.clear_timer()
        self.update_cmd_buffer("both")
        self.param.handside = "right"

        if self._is_parallel_group_finalizer(44) and (
            self._grasp_parallel_vision_started or self._vision_done
        ):
            logger.info("Active 44 reuses the vision task started in Active 43.")
            self._timer = self.create_timer(0.01, self.timer_detect_grasp_object)
            self._track_active_action(
                44,
                expected_duration=self._get_expected_duration(44)
            )
            self.get_logger().info("New timer created.")
            return

        self._shutdown_vision_resources()

        handside = getattr(self, "grasp_handside", self._resolve_grasp_handside())
        self.grasp_handside = handside
        self._vision_done = False
        self._grasp_motion_done = False
        self._hand_gesture_done = False
        self._vision_finalize_done = False
        self.left_target_pos = None
        self.right_target_pos = None
        self._setup_grasp_eye(handside, with_hand_eye=False)
        self.eye.initialize_obj_coordinate(self.eye.classes)
        self.eye.obj_detect = True
        self._start_vision_thread()

        self._timer = self.create_timer(0.01, self.timer_detect_grasp_object)
        self._track_active_action(
            44,
            expected_duration=self._get_expected_duration(44)
        )
        self.get_logger().info("New timer created.")

    # Active 45 抓取动作
    def on_active_45_handler(self):
        logger.info(f"Enter state {self.sm.get_state()} '{self.action_name[45]}'")
        self.clear_timer()
        self._shutdown_vision_resources()
        self.update_cmd_buffer("both")
        self.param.handside = "right"
        handside = getattr(self, "grasp_handside", self._resolve_grasp_handside())
        self._grasp_motion_done = False
        lr_height = [self.eecmd_fk_buffer.left.pos[2], self.eecmd_fk_buffer.right.pos[2]]
        handside = "right"
        self._timer = self.create_timer(0.01, lambda: self.timer_grasp_sequence(lr_height, handside))
        self._track_active_action(
            45,
            expected_duration=self._get_expected_duration(45)
        )
        self.get_logger().info("New timer created.")

    # Active 47 手势视觉
    def on_active_47_handler(self):
        logger.info(f"Enter state {self.sm.get_state()} '{self.action_name[47]}'")
        self.clear_timer()
        self._shutdown_vision_resources()
        self.update_cmd_buffer("both")

        handside = getattr(self, "grasp_handside", self._resolve_grasp_handside())
        handside = "right"
        self.grasp_handside = handside
        self._hand_gesture_done = False
        self._vision_finalize_done = False
        self.left_hand_target_pos = None
        self.right_hand_target_pos = None
        self._setup_grasp_eye(handside, with_hand_eye=True)
        self.eye.initialize_obj_coordinate(self.eye.classes)
        self.eye.obj_detect = False
        self._vision_done = True
        self._grasp_motion_done = True
        self._start_vision_thread()

        self._timer = self.create_timer(0.01, lambda: self.timer_detect_hand_gesture(handside))
        self._track_active_action(
            47,
            expected_duration=self._get_expected_duration(47)
        )
        self.get_logger().info("New timer created.")

    # Active 48 放苹果
    def on_active_48_handler(self):
        logger.info(f"Enter state {self.sm.get_state()} '{self.action_name[48]}'")
        self.clear_timer()
        self._shutdown_vision_resources()
        self.update_cmd_buffer("both")

        handside = getattr(self, "grasp_handside", self._resolve_grasp_handside())
        handside = "right"
        self._grasp_motion_done = False
        lr_height = [self.eecmd_fk_buffer.left.pos[2], self.eecmd_fk_buffer.right.pos[2]]
        self._timer = self.create_timer(0.01, lambda: self.timer_release_sequence(lr_height, handside))
        self._track_active_action(
            48,
            expected_duration=self._get_expected_duration(48)
        )
        self.get_logger().info("New timer created.")



    # ========== Timer Callbacks ==========

    def timer_back(self):
        self.time_ += self.control_dt_
        self.arm_back_zero(0., 2, "both")
        self.publish_all()


    def timer_hello(self, handside: str):
        self.time_ += self.control_dt_
        if handside != "left" and handside != "right":
            self.show_hello(0., 600, "both", speed=1.5)
        else:
            arm_side_opp = "left" if handside == "right" else "right"
            self.show_hello(0., 600, handside, speed=1.5)
            self.arm_back_zero(0., 2, arm_side_opp)
        self.publish_all()


    def timer_like(self, handside: str):
        self.time_ += self.control_dt_
        if handside != "left" and handside != "right":
            self.show_like(0., 1, "both")
        else:
            arm_side_opp = "left" if handside == "right" else "right"
            self.show_like(0., 1, handside)
            self.arm_back_zero(0., 2, arm_side_opp)
        self.publish_all()


    def timer_hand_shake(self, handside: str):
        self.time_ += self.control_dt_
        if handside != "left" and handside != "right":
            self.hand_shake(0., 2, pause=3., armside="right")
        else:
            arm_side_opp = "left" if handside == "right" else "right"
            self.hand_shake(0., 2, pause=5., armside=handside)
            self.arm_back_zero(0., 2, arm_side_opp)
        self.publish_all()


    def timer_rps(self, handside: str):
        self.time_ += self.control_dt_
        if handside != "left" and handside != "right":
            self.play_rps(0., 2, self.end_ts, self.interval_stop, self.interval_move, "both")
        else:
            arm_side_opp = "left" if handside == "right" else "right"
            self.play_rps(0., 2, self.end_ts, self.interval_stop, self.interval_move, handside)
            self.arm_back_zero(0., 2, arm_side_opp)
        self.publish_all()  

    
    def timer_showheart(self):
        self.time_ += self.control_dt_
        self.teleop_showheart(0., 1, "both")
        self.publish_all()

    def timer_turn_command(self):
        self.time_ += self.control_dt_

        if self._turn_action_future is not None and self._turn_action_future.done() and not self._turn_action_result_logged:
            self._turn_action_result_logged = True
            try:
                result = self._turn_action_future.result()
                if result.success:
                    logger.info(f"{self._turn_action_name} high command finished: {result.message}")
                else:
                    logger.warning(f"{self._turn_action_name} high command failed: {result.message}")
            except Exception as exc:
                logger.error(f"{self._turn_action_name} high command exception: {exc}")

        self.publish_all()

    def timer_safe_arm_down(self, lr_height):
        self.time_ += self.control_dt_
        self.arm_hand_back_zero_table_safe(lr_height, handside="both")
        self.publish_all()

    def timer_wait_for_expected_duration(self):
        self.time_ += self.control_dt_
        if self._tracked_active_deadline is not None and self.time_ >= self._tracked_active_deadline:
            if hasattr(self, "mark_active_action_done"):
                self.mark_active_action_done()
        self.publish_all()

    def timer_get_ready_sequence(self, lr_height, handside):
        self.time_ += self.control_dt_
        if handside != "left" and handside != "right":
            self.get_ready_safe("both", lr_height)
        else:
            arm_side_opp = "left" if handside == "right" else "right"
            self.get_ready_safe(handside, lr_height)
            self.arm_hand_back_zero_table_safe(lr_height, handside=arm_side_opp)

        ready_done = self.time_ >= 3.5
        if (
            self._parallel_grasp_ready_vision
            and ready_done
            and self._vision_done
            and hasattr(self, "mark_active_action_done")
        ):
            self.mark_active_action_done()
        elif ready_done and hasattr(self, "mark_active_action_done"):
            self.mark_active_action_done()

        self.publish_all()

    def timer_get_ready_pour_sequence(self, lr_height):
        self.time_ += self.control_dt_
        self.get_ready_safe("both", lr_height)

        if self.time_ >= 3.5 and hasattr(self, "mark_active_action_done"):
            self.mark_active_action_done()

        self.publish_all()

    def timer_detect_grasp_object(self):
        self.time_ += self.control_dt_
        self.param.handside = "right"
        if self._vision_done and not self._vision_finalize_done:
            self._vision_finalize_done = True
            self._shutdown_vision_resources()
            if hasattr(self, "mark_active_action_done"):
                self.mark_active_action_done()

        self.publish_all()

    def timer_detect_pour_object(self):
        self.time_ += self.control_dt_

        if self._vision_done and not self._vision_finalize_done:
            self._vision_finalize_done = True
            self._shutdown_vision_resources()
            if hasattr(self, "mark_active_action_done"):
                self.mark_active_action_done()

        self.publish_all()

    def timer_pour_water_motion(self):
        self.time_ += self.control_dt_
        self.pour_water_seq(0.)

        if self._pour_motion_done and hasattr(self, "mark_active_action_done"):
            self.mark_active_action_done()

        self.publish_all()

    def timer_grasp_sequence(self, lr_height, handside):
        self.time_ += self.control_dt_
        if handside != "left" and handside != "right":
            self.grasp_apple_seq(0., handside="both")
        else:
            arm_side_opp = "left" if handside == "right" else "right"
            self.grasp_apple_seq(0., handside=handside)
            self.arm_hand_back_zero_table_safe(lr_height, handside=arm_side_opp)

        if self._release_motion_done and hasattr(self, "mark_active_action_done"):
            self.mark_active_action_done()

        self.publish_all()

    def timer_detect_hand_gesture(self, handside):
        self.time_ += self.control_dt_

        if self._hand_gesture_done and not self._vision_finalize_done:
            self._vision_finalize_done = True
            self._shutdown_vision_resources()
            if hasattr(self, "mark_active_action_done"):
                self.mark_active_action_done()

        self.publish_all()

    def timer_release_sequence(self, lr_height, handside):
        self.time_ += self.control_dt_
        if handside != "left" and handside != "right":
            self.release_apple_seq(0., handside="both")
        else:
            arm_side_opp = "left" if handside == "right" else "right"
            self.release_apple_seq(0., handside=handside)
            self.arm_hand_back_zero_table_safe(lr_height, handside=arm_side_opp)

        if self._grasp_motion_done and hasattr(self, "mark_active_action_done"):
            self.mark_active_action_done()

        self.publish_all()

    def timer_replay(self, speed=1.):
        self.time_ += self.control_dt_
        self.dataset_replay(speed=speed)
        self.publish_all()
