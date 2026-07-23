import numpy as np

from control_py.state_manager.basic_states import BasicStates

import pyrealsense2 as rs
import cv2
from pathlib import Path
import yaml

from control_py.utils.math_process import generate_random_pose

from loguru import logger
from control_py.utils.loguru_settings import setup_loguru
setup_loguru(log_folder_path="log", show_on_terminal=True, terminal_level='DEBUG') 

config_file = 'src/control_py/control_py/state_manager/calibrate/calibrate_config.yaml'
with open(config_file, 'r') as f:
    calibrate_cfg = yaml.safe_load(f) or {}

paths_cfg = calibrate_cfg["paths"]
collection_cfg = calibrate_cfg["collection"]

CAM_CALIBR_DIR = Path(paths_cfg["cam_calibr_dir"])
ARM_POSE_DIR = CAM_CALIBR_DIR / "arm_pose"


ResetPoseLeft = collection_cfg["reset_pose"]["left"]
ResetPoseRight = collection_cfg["reset_pose"]["right"]

InitPoseLeft = collection_cfg["init_pose"]["left"]
InitPoseRight = collection_cfg["init_pose"]["right"]

move_range = collection_cfg["move_range"]

class CalibrTasks(BasicStates):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        
        self.ready_to_start = True

        # 修改零位防止板子与腿碰撞
        self.zero_arm_q = self._arm_target_for_current_dof([
            0.4, 0.15, 0., 0.8, -0.5, 0., 0.,
            0.4, -0.15, 0., 0.8, 0.5, 0., 0.,
        ])
        
        self._calibr_collection_done = False
        

                    
    # 准备
    def get_ready(self, armside, lr_height):
        hand_q = [0., 0., 0., 0., 0., 0., 
                  0., 0., 0., 0., 0., 0.]
        if 0. <= self.time_ < 1.:
            arm_q = self._arm_target_for_current_dof([
                0., 0.8, 1.4, 1.2, 0., 0., 0.,
                0., -0.8, -1.4, 1.2, 0., 0., 0.,
            ])
            
            if lr_height[0] > self.safe_height_threshold:
                arm_q = self._replace_arm_side_values(
                    arm_q,
                    "left",
                    [0.142, 0.556, 0.501, -0.147, -0.218, 0.052, -0.525],
                )
            if lr_height[1] > self.safe_height_threshold:
                arm_q = self._replace_arm_side_values(
                    arm_q,
                    "right",
                    [0.142, -0.556, -0.501, -0.147, 0.218, 0.052, 0.525],
                )
            
            self.arm_hand_joint_control(0., 1., hand_q, arm_q, armside=armside)
            self.waist_joint_control(0., 1., waist_q=0.)
        
        elif 1. <= self.time_ < 2.:
            self.waist_joint_fix(waist_q=0.)
            arm_q = self._arm_target_for_current_dof([
                0., 0.8, 1.4, 0.5, 0., 0., 0.,
                0., -0.8, -1.4, 0.5, 0., 0., 0.,
            ])
            if armside != "right" and lr_height[0] <= self.safe_height_threshold:
                self.arm_hand_joint_control(1., 2., hand_q, arm_q, armside="left")
            if armside != "left" and lr_height[1] <= self.safe_height_threshold:
                self.arm_hand_joint_control(1., 2., hand_q, arm_q, armside="right")

        elif 2. <= self.time_ < 3.:
            self.waist_joint_fix(waist_q=0.)
            target_ee_left = [0.2, 0.32, 0.18, 0.4, 0., -0.1]
            target_ee_right = [0.2, -0.32, 0.18, -0.4, 0., 0.1]
            sol_q, sol_tauff = self.solve_ee_ik(target_ee_left, target_ee_right, euler_order="xyz")
            self.arm_hand_joint_control(2., 3., hand_q, sol_q, arm_tau=sol_tauff, armside=armside)

    # 校准测试
    def calibr_test(self, start, end, armside):
        if start <= self.time_ < end:
            hand_q = [0.] * 12

            # Calculate IK
            if armside == "left":
                sol_q, sol_tauff = self.solve_ee_ik(InitPoseLeft, ResetPoseRight, quat_order="xyzw")
            elif armside == "right":
                sol_q, sol_tauff = self.solve_ee_ik(ResetPoseLeft, InitPoseRight, quat_order="xyzw")
            else:
                sol_q, sol_tauff = self.solve_ee_ik(ResetPoseLeft, ResetPoseRight, quat_order="xyzw")
            
            self.arm_hand_joint_control(start, end, hand_q, sol_q, arm_tau=sol_tauff, armside="both")
            self.waist_joint_control(start, end, waist_q=0.)
        
        elif end < self.time_ < end  + 1. and not self.camera_start:
            self.waist_joint_fix(waist_q=0.)
            logger.info(f"Camera Start...")
            # 提前初始化等待相机稳定
            self.calibr_camera_init()
            self.camera_start = True
        
        elif self.time_ > end + 1. and not self.image_captured:
            self.waist_joint_fix(waist_q=0.)
            logger.info(f"Taking image...")
            img_id = {"left": 1, "right": 2}.get(armside, 0)
            output_dir = "/home/unitree/unitree-g1-brainco-hand/scripts/_test_cams"
            
            filename = f"calibr_test_{img_id}.jpg"
            self.image_captured = self.capture_image(output_dir, filename)
            self.shutdown_calibr_cameras()
            if self.image_captured:
                logger.info(f"Image captured.")
            else:
                logger.warning(f"Fail to capture image.")


    def calibr_start(self, start, t_move, t_pause, repeat, armside):
        armside = "right" if armside != "left" else "left"
        hand_q = [0.] * 12
        final_reset_end = start + 2. + t_pause + (t_move + t_pause) * repeat + t_move

        if start <= self.time_ < start + 2:
            sol_q, sol_tauff = self.solve_ee_ik(ResetPoseLeft, ResetPoseRight)
            self.arm_hand_joint_control(start, start + 2, hand_q, sol_q, arm_tau=sol_tauff, armside="both")
            self.waist_joint_control(start, start + 2, waist_q=0.)
        elif self.image_count < repeat:
            self.waist_joint_fix(waist_q=0.)

            _move_start = start + 2. + t_pause + (t_move + t_pause) * self.image_count
            _move_end = _move_start + t_move
            _pause_end = _move_end + t_pause
        
            if _move_start <= self.time_ < _move_end:

                if self.random_reset:
                    if armside == "left":
                        self.random_pose_left = generate_random_pose(InitPoseLeft, move_range)
                        self.random_pose_right = ResetPoseRight.copy()
                        logger.info(f"{armside}-{self.image_count + 1}: {np.round(self.random_pose_left, 3)}")
                    elif armside == "right":
                        self.random_pose_left = ResetPoseLeft.copy()
                        self.random_pose_right = generate_random_pose(InitPoseRight, move_range)
                        logger.info(f"{armside}-{self.image_count + 1}: {np.round(self.random_pose_right, 3)}")
                    self.random_reset = False
                
                sol_q, sol_tauff = self.solve_ee_ik(self.random_pose_left, self.random_pose_right)
                self.arm_hand_joint_control(_move_start, _move_end, hand_q, sol_q, arm_tau=sol_tauff, armside="both")

            elif _move_end + t_pause/2 <= self.time_ < _pause_end:

                if not self.random_reset:
                    self.capture_image_flag = True
                    self.image_count += 1

                    # save_pose
                    save_id=f"pose{self.image_count}"
                    if armside == "left":
                        output_path = ARM_POSE_DIR / "left.json"
                        self.eecmd_buffer.left.save_json(save_id, str(output_path))
                    elif armside == "right":
                        output_path = ARM_POSE_DIR / "right.json"
                        self.eecmd_buffer.right.save_json(save_id, str(output_path))

                    self.random_reset = True
                    logger.info(f"image done: {self.image_count}")

        elif self.image_count >= repeat:
            self.waist_joint_fix(waist_q=0.)
            _move_start = start + 2. + t_pause + (t_move + t_pause) * self.image_count
            _move_end = _move_start + t_move
            if _move_start <= self.time_ < _move_end:
                sol_q, sol_tauff = self.solve_ee_ik(ResetPoseLeft, ResetPoseRight)
                self.arm_hand_joint_control(_move_start, _move_end, hand_q, sol_q, arm_tau=sol_tauff, armside="both")
            elif self.time_ >= final_reset_end:
                self._calibr_collection_done = True
        else:
            self.waist_joint_fix(waist_q=0.)
            sol_q, sol_tauff = None, None
    

    def calibr_camera_init(self):       
        self.cam_pipeline = rs.pipeline()
        config = rs.config()
        x, y, fps = 640, 360, 30 # 1280, 720, 30
        config.enable_stream(rs.stream.color, x, y, rs.format.bgr8, fps)
        self.cam_pipeline.start(config)

    def head_camera_init(self):       
        self.cam_pipeline = rs.pipeline()
        config = rs.config()
        x, y, fps = 1280, 720, 30
        config.enable_stream(rs.stream.color, x, y, rs.format.bgr8, fps)
        self.cam_pipeline.start(config)


    def capture_image(self, output_dir, filename):
        if self.cam_pipeline is not None:
            # 获取一帧
            frames = self.cam_pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                logger.warning("Failed to capture image")
                return False
            else:
                image = np.asanyarray(color_frame.get_data())
                # 保存图片
                output_dir = Path(output_dir)
                output_dir.mkdir(parents=True, exist_ok=True)
                savefile = output_dir / filename
                cv2.imwrite(savefile, image)
                logger.info(f"Image saved as {savefile}")
                return True
        else:
            logger.warning(f"self.cam_pipeline is NoneType")
    def shutdown_calibr_cameras(self):
        if self.cam_pipeline is not None:
            self.cam_pipeline.stop()
            self.cam_pipeline = None

        # 关闭线程
        self._vision_running = False
        if self._vision_thread is not None:
            self._vision_thread.join(timeout=1.0)

    
