from control_py.utils.utils import *
from control_py.state_manager.basic_states import BasicStatesHandler

from pathlib import Path
import yaml

import threading

from loguru import logger
from control_py.utils.loguru_settings import setup_loguru
setup_loguru(log_folder_path="log", show_on_terminal=True, terminal_level='DEBUG') # 设置日志

config_file = 'src/control_py/control_py/state_manager/calibrate/calibrate_config.yaml'
with open(config_file, 'r') as f:
    calibrate_cfg = yaml.safe_load(f) or {}

CAM_CALIBR_DIR = Path(calibrate_cfg["paths"]["cam_calibr_dir"])

class CalibrTasksHandler(BasicStatesHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.cam_pipeline = None
        self.eye = None
        self.cam_fps = 30
        # self.yolo_skip = 0   # 控制YOLO频率
        self._last_yolo_time = 0
        self._yolo_interval = 0.1   # 10Hz

        self._cam_running = False
        self._cam_lock = threading.Lock()
        self._eye_lock = threading.Lock()

        self._vision_thread = None
        self._vision_running = False
        self.calibr_armside = "right"
        self._active_finalize_done = False

    def _get_expected_duration(self, action_idx: int, fallback=None):
        if hasattr(self, "get_active_expected_duration"):
            return self.get_active_expected_duration(action_idx, fallback=fallback)
        return fallback

    def _track_active_action(self, action_idx: int, expected_duration=None):
        if hasattr(self, "start_active_action_tracking"):
            self.start_active_action_tracking(action_idx, expected_duration=expected_duration)

    def _complete_active_action(self, cleanup=False):
        if cleanup and not self._active_finalize_done:
            self.shutdown_calibr_cameras()
            self._active_finalize_done = True
        if hasattr(self, "mark_active_action_done"):
            self.mark_active_action_done()
        

    # Active 1 准备
    def on_active_1_handler(self):
        logger.info(f"Enter state -> {self.sm.get_state()} '{self.action_name[1]}'")
        self.clear_timer()
        self.update_cmd_buffer("both")
        self.shutdown_calibr_cameras()
        self._active_finalize_done = False
        lr_height = [self.eecmd_fk_buffer.left.pos[2], self.eecmd_fk_buffer.right.pos[2]]
        self._timer = self.create_timer(0.01, lambda: self.timer_ready(lr_height))
        self._track_active_action(1, expected_duration=self._get_expected_duration(1))


    # Active 2 校准点测试
    def on_active_2_handler(self):
        logger.info(f"Enter state -> {self.sm.get_state()} '{self.action_name[2]}'")
        self.clear_timer()
        self.update_cmd_buffer("both")
        self.shutdown_calibr_cameras()
        self._active_finalize_done = False
        # 相机采集初始化
        self.camera_start, self.image_captured = False, False
        self._timer = self.create_timer(0.01, self.timer_calibr_test)
        self._track_active_action(2, expected_duration=self._get_expected_duration(2))


    def _start_calibr_collection(self, armside):
        self.calibr_armside = armside
        self.clear_timer()
        self.update_cmd_buffer("both")
        self.shutdown_calibr_cameras()
        self._active_finalize_done = False
        # 相机采集初始化
        self.calibr_camera_init()
        self.image_count = 0
        self.capture_image_flag = False
        self.random_reset = True
        self._calibr_collection_done = False
        # 初始化末端位置
        self.random_pose_left, self.random_pose_right = None, None
        # 启动两个定时器
        self._timer = self.create_timer(0.01, self.timer_calibr_robot_start)
        self._cam_timer = self.create_timer(0.1, self.timer_calibr_cam_start)


    # Active 3 左手采集校准数据
    def on_active_3_handler(self):
        logger.info(f"Enter state -> {self.sm.get_state()} '{self.action_name[3]}'")
        self._start_calibr_collection("left")
        self._track_active_action(3, expected_duration=self._get_expected_duration(3))

    
    # Active 4 右手采集校准数据
    def on_active_4_handler(self):
        logger.info(f"Enter state -> {self.sm.get_state()} '{self.action_name[4]}'")
        self._start_calibr_collection("right")
        self._track_active_action(4, expected_duration=self._get_expected_duration(4))


    def timer_ready(self, lr_height):
        self.time_ += self.control_dt_
        if self.param.handside != "left" and self.param.handside != "right":
            self.get_ready("both", lr_height)
        else:
            arm_side_opp = "left" if self.param.handside == "right" else "right"
            self.get_ready(self.param.handside, lr_height)
            # self.arm_back_zero(0., 2, arm_side_opp)
            self.arm_hand_back_zero_table_safe(lr_height, handside=arm_side_opp)
        if self.time_ >= 3.0:
            self._complete_active_action()
        self.publish_all()


    def timer_calibr_test(self):
        self.time_ += self.control_dt_
        self.calibr_test(0., 1, self.param.handside)
        if self.image_captured:
            self._complete_active_action(cleanup=True)
        self.publish_all()


    def timer_calibr_robot_start(self):
        self.time_ += self.control_dt_
        self.calibr_start(start=0., t_move=1., t_pause=2., repeat=20, armside=self.calibr_armside)
        if self._calibr_collection_done:
            self._complete_active_action(cleanup=True)
        self.publish_all()
    
    def timer_calibr_cam_start(self):
        if self.capture_image_flag:
            output_dir = CAM_CALIBR_DIR / "imgs" / self.calibr_armside
            filename = f"calibr_img_{self.image_count}.jpg"
            self.capture_image(str(output_dir), filename)
            self.capture_image_flag = False


    
    # ----- 修改 BasicStatesHandler -----
    # 准备阶段
    def on_configure_handler(self):
        if self.ready_to_start:
            logger.info(f"Enter state -> {self.sm.get_state()}")
            self.clear_timer()
            self.update_cmd_buffer("both")
            self.shutdown_calibr_cameras()

            # safe reset
            # lr_height = [self.eecmd_fk_buffer.left.pos[2], self.eecmd_fk_buffer.right.pos[2]]
            # self._timer = self.create_timer(0.01, lambda: self.timer_get_ready_table_safe(lr_height))
            self._timer = self.create_timer(0.01, self.timer_get_ready)
        else:
            logger.info("Waiting ...")

    # 停止程序
    def on_shutdown_handler(self):
        super().on_shutdown_handler()
        self.shutdown_calibr_cameras()

    # Active 0 静止
    def on_active_0_handler(self):
        super().on_active_0_handler()
        self.shutdown_calibr_cameras()

    
    


    
