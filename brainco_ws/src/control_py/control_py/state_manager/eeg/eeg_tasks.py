import numpy as np
import cv2
import yaml

# from control_py.state_manager.robot_control import *
from control_py.state_manager.basic_states import BasicStates
from control_py.teleop_pkg.data_process import *
from sensor_msgs.msg import CompressedImage
from control_py.state_manager.calibrate.motion import calculate_obj_coordinate_hand, build_pose

from control_py.state_manager.common.lerobot_dataset import (
    ACTION, 
    action_to_arm_cmd,
    action_to_hand_cmd
    )

from loguru import logger
from control_py.utils.loguru_settings import setup_loguru, logger_with_params
setup_loguru(log_folder_path="log", show_on_terminal=True, terminal_level='DEBUG')

config_file = 'src/control_py/control_py/state_manager/eeg/vision_config.yaml'
with open(config_file, 'r') as f:
    vision_cfg = yaml.safe_load(f) or {}

task_pose_cfg = vision_cfg["task_pose"]
pour_water_cfg = task_pose_cfg["pour_water"]
grasp_apple_cfg = task_pose_cfg["grasp_apple"]
release_apple_cfg = task_pose_cfg["release_apple"]

left_cup_offset = pour_water_cfg["left_cup_offset"]
right_bottle_offset = pour_water_cfg["right_bottle_offset"]
left_cup_offer_pose = pour_water_cfg["left_cup_offer_pose"]
left_cup_offer_offset = pour_water_cfg["left_cup_offer_offset"]

left_orange_offset_approach = grasp_apple_cfg["left_orange_offset_approach"]
right_apple_offset_approach = grasp_apple_cfg["right_apple_offset_approach"]
left_orange_offset = grasp_apple_cfg["left_orange_offset"]
right_apple_offset = grasp_apple_cfg["right_apple_offset"]

left_hand_offset_approach = release_apple_cfg["left_hand_offset_approach"]
right_hand_offset_approach = release_apple_cfg["right_hand_offset_approach"]
left_hand_offset = release_apple_cfg["left_hand_offset"]
right_hand_offset = release_apple_cfg["right_hand_offset"]



class EEGTasks(BasicStates):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        
        self.ready_to_start = True
        self.left_target_pos = None
        self.right_target_pos = None
        self.left_hand_target_pos = None
        self.right_hand_target_pos = None
        self.bottle_pos = None
        self.cup_pos = None
        self.grasp_handside = "both"
        self.hand_eye = None
        self._grasp_motion_done = False
        self._release_motion_done = False
        self._hand_gesture_done = False
        self._pour_motion_done = False
        self._vision_task_mode = None

    def get_ready(self, handside, lr_height):
        hand_q = [0., 0., 0., 0., 0., 0.,
                  0., 0., 0., 0., 0., 0.]
        if 0. <= self.time_ < 1.:
            self.waist_joint_control(0., 1., waist_q=0.)
            target_ee_left = [0.2, 0.32, 0.18, 0.4, 0., -0.1]
            target_ee_right = [0.2, -0.32, 0.18, -0.4, 0., 0.1]
            sol_q, sol_tauff = self.solve_ee_ik(target_ee_left, target_ee_right, euler_order="xyz")
            self.arm_hand_joint_control(2., 3., hand_q, sol_q, arm_tau=sol_tauff, armside=handside)
    
    
    def get_ready_safe(self, handside, lr_height):
        hand_q = [0., 0., 0., 0., 0., 0.,
                  0., 0., 0., 0., 0., 0.]
        if 0. <= self.time_ < 1.:
            self.waist_joint_control(0., 1., waist_q=0.)
            arm_q = self._table_safe_arm_target()
            if handside != "right" and lr_height[0] <= self.safe_height_threshold:
                self.arm_hand_joint_control(0., 1., hand_q, arm_q, armside="left")
            if handside != "left" and lr_height[1] <= self.safe_height_threshold:
                self.arm_hand_joint_control(0., 1., hand_q, arm_q, armside="right")

        elif 1. <= self.time_ < 2.:
            self.waist_joint_fix(waist_q=0.)
            target_ee_left = [0.2, 0.32, 0.18, 0.4, 0., -0.1]
            target_ee_right = [0.2, -0.32, 0.18, -0.4, 0., 0.1]
            sol_q, sol_tauff = self.solve_ee_ik(target_ee_left, target_ee_right, euler_order="xyz")
            self.arm_hand_joint_control(1., 2., hand_q, sol_q, arm_tau=sol_tauff, armside=handside)
    
    # 挥手
    def show_hello(self, start, end, armside, speed=1.):
        if start <= self.time_ < end:
            duration = round(1/speed, 3)
            hand_q = [0., 0., 0., 0., 0., 0.] * 2

            target_q_1 = [-0.721,  0.727,  0.193, -0.099,  0.911,  0.,  0.656,
                          -0.721, -0.727, -0.193, -0.099, -0.911,  0., -0.656]
            target_q_2 = [-0.711,  1.029,  0.275,  0.42,   0.641, 0.,  0.873,
                          -0.711, -1.029, -0.275,  0.42,  -0.641, 0., -0.873]
            target_q_avg = [(x + y) / 2 for x, y in zip(target_q_1, target_q_2)]

            if self.time_ < start + 1.:
                self.arm_hand_joint_control(start, start + 1., 
                                            hand_q, target_q_avg, armside=armside)
                
            elif self.time_ >= start + 1.:
                t = (self.time_ - start - 1.) % duration
                if 0. <= t < duration/2:
                    self.arm_hand_joint_control(self.time_ - t, self.time_ - t + duration/2, 
                                                hand_q, target_q_1, armside=armside)
                    
                elif duration/2 <= t < duration:
                    self.arm_hand_joint_control(self.time_ - t + duration/2, self.time_ - t + duration, 
                                                hand_q, target_q_2, armside=armside)
                    
            self.waist_joint_control(start, end, waist_q=0.)

    # 点赞
    def show_like(self, start, end, armside):
        if start <= self.time_ < end:
            hand_q = [0., 0., 1.4, 1.4, 1.4, 1.4] * 2
            target_q = [-0.835,  0.315,  0.226,  1.027, -0.127, -0.116, -0.182, 
                        -0.835, -0.315, -0.226,  1.027,  0.127, -0.116,  0.182]
            self.arm_hand_joint_control(start, end, hand_q, target_q, armside=armside)

    # 握手
    def hand_shake(self, start, end, pause=5., armside="right"):
        # 默认右手握手，不存在双手握手
        armside = "right" if armside != "left" else "left"
        target_q = [-0.926,  0.029,  0.152, 1.205, 0.2, 0., 0.,
                    -0.926, -0.029, -0.152, 1.205,  0.2, 0., 0.]
        if start <= self.time_ < end:
            hand_q = [0.5, 1.0, 0.1, 0.1, 0.1, 0.1] * 2
            self.arm_hand_joint_control(start, end, hand_q, target_q, armside=armside)
        if end <= self.time_ < end + pause:
            hand_q = [0.5, 1.0, 0.1, 0.1, 0.1, 0.1] * 2
            self.arm_hand_joint_control(end, end + pause, hand_q, target_q, armside=armside)
        if end + pause <= self.time_ < end + pause + 2.:
            self.arm_back_zero(end + pause, end + pause + 2., armside)
        self.waist_joint_control(start, end, waist_q=0.)


    # 石头剪刀布，抬手
    def play_rps_move(self, show, direction):
        # show: 是否出拳
        # direction: "up" or "down" 手臂向上或向下
        if direction == "up":
            target_q = [-0.374,  0.466,  0.531,  0.104, -0.311, -0.088, -0.061, 
                        -0.384, -0.495, -0.548,  0.158,  0.33,  -0.109,  0.089]
        else:
            target_q = [-0.382,  0.461,  0.374,  0.493, -0.114, -0.02,   0.019, 
                        -0.398, -0.492, -0.383,  0.553,  0.128, -0.05,   0.006]
            
        rps_gesture = [[1.1, 0.4, 1.4, 1.4, 1.4, 1.4], # 石头
                        [1.1, 0., 0., 0., 1.4, 1.4],    # 剪刀
                        [0., 0., 0., 0., 0., 0.]]        # 布


        hand_q = (rps_gesture[self.curr_note_left] + rps_gesture[self.curr_note_right]
                  if show else rps_gesture[0] * 2)
        
        return target_q, hand_q

    
    
    # 随机石头剪刀布
    def play_rps_shuffle(self, arm, shuffle=True):
        if arm != "left":
            self.curr_note_right = np.random.randint(3) if shuffle else 0
        if arm != "right":
            self.curr_note_left = np.random.randint(3) if shuffle else 0

    # 石头剪刀布
    def play_rps(self, prepare, start, end, inter_stop, inter_m, armside):

        if prepare <= self.time_ < start:
            target_q, hand_q = self.play_rps_move(False, "up")
            self.arm_hand_joint_control(prepare, start, hand_q, target_q, armside=armside)
            self.waist_joint_control(prepare, start, waist_q=0.)

        elif start <= self.time_ < end:
            self.waist_joint_fix(waist_q=0.)
            
            t = (self.time_ - start) % (inter_stop + inter_m * 6)

            if 0 <= t < inter_m:
                target_q, hand_q = self.play_rps_move(False, "down")
                _start = self.time_ - t
                _end = self.time_ - t + inter_m
                self.arm_hand_joint_control(_start, _end, hand_q, target_q, armside=armside)

            elif inter_m <= t < inter_m * 2:
                target_q, hand_q = self.play_rps_move(False, "up")
                _start = self.time_ - t + inter_m
                _end = self.time_ - t + inter_m * 2
                self.arm_hand_joint_control(_start, _end, hand_q, target_q, armside=armside)
            
            elif inter_m * 2 <= t < inter_m * 3:
                target_q, hand_q = self.play_rps_move(False, "down")
                _start = self.time_ - t + inter_m * 2
                _end = self.time_ - t + inter_m * 3
                self.arm_hand_joint_control(_start, _end, hand_q, target_q, armside=armside)
                self.play_rps_shuffle(armside)

            elif inter_m * 3 <= t < inter_m * 4:
                target_q, hand_q = self.play_rps_move(False, "up")
                _start = self.time_ - t + inter_m * 3
                _end = self.time_ - t + inter_m * 4
                self.arm_hand_joint_control(_start, _end, hand_q, target_q, armside=armside)

            elif inter_m * 4 <= t < inter_m * 5:
                target_q, hand_q = self.play_rps_move(True, "down")
                _start = self.time_ - t + inter_m * 4
                _end = self.time_ - t + inter_m * 5
                self.arm_hand_joint_control(_start, _end, hand_q, target_q, armside=armside)

            elif inter_m * 5 <= t < inter_m * 5 + inter_stop:
                self.play_rps_shuffle(armside, shuffle=False)

            elif inter_m * 5 + inter_stop <= t < inter_m * 6 + inter_stop:
                target_q, hand_q = self.play_rps_move(False, "up")
                _start = self.time_ - t + inter_m * 5 + inter_stop
                _end = self.time_ - t + inter_m * 6 + inter_stop
                self.arm_hand_joint_control(_start, _end, hand_q, target_q, armside=armside)
            
            
            

    def teleop_showheart(self, start, end, armside):
        target_q = [-0.66,   0.188,  0.038,  0.326,  0.45,  -1.43,   0.094,
                    -0.599, -0.186,  0.029,  0.098, -0.4 ,  -1.271, -0.062]
        hand_q = [0., 0.7, 0.6, 0.6, 0.6, 0.6] * 2
        if start <= self.time_ < end:
            self.arm_hand_joint_control(start, end, hand_q, target_q, armside=armside)

    def _vision_loop(self):
        logger.info("Vision loop started")

        while self._vision_running:
            try:
                with self._eye_lock:
                    self.eye.get_images()

                    if self.eye.obj_detect and not self._vision_done:
                        self.eye.yolo_object_detection()
                    elif (
                        self._grasp_motion_done
                        and not self._hand_gesture_done
                        and self.hand_eye is not None
                    ):
                        stable_gesture = self.hand_eye.recognize_gesture(
                            self.eye.color_image,
                            self.eye.depth_frame,
                            self.eye.depth_intrin,
                        )
                        if (
                            stable_gesture == "五指张开"
                            and self.hand_eye.hand_coordinate_camera_avg is not None
                        ):
                            hand_coordinate_camera = self.hand_eye.hand_coordinate_camera_avg
                            hand_coordinate_dict = {"hand": hand_coordinate_camera}
                            left_hand = calculate_obj_coordinate_hand(hand_coordinate_dict, "left")["hand"]
                            right_hand = calculate_obj_coordinate_hand(hand_coordinate_dict, "right")["hand"]
                            self.left_hand_target_pos = left_hand
                            self.right_hand_target_pos = right_hand
                            logger.info("五指张开")
                            logger.info(
                                f"hand pos camera: {np.round(hand_coordinate_camera, 3)}, "
                                f"left: {np.round(left_hand, 3)}, "
                                f"right: {np.round(right_hand, 3)}"
                            )
                            self._hand_gesture_done = True
                        elif stable_gesture == "五指张开":
                            # logger.debug("五指张开，但手部位置未稳定")
                            pass
                        elif self.hand_eye.last_hand_detected:
                            logger.debug("检测到手，但未识别为五指张开")
                        else:
                            logger.debug("未检测到手势")

                self.publish_head_image()

                if self._vision_task_mode == "pour":
                    required_classes = list(self.eye.classes)
                elif self.grasp_handside == "left":
                    required_classes = ["orange"]
                elif self.grasp_handside == "right":
                    required_classes = ["apple"]
                else:
                    required_classes = list(self.eye.classes)

                get_obj_coordinate = 0
                for class_name in required_classes:
                    if class_name in self.eye.obj_coordinate_camera_avg:
                        if self.eye.obj_coordinate_camera_avg[class_name]:
                            get_obj_coordinate += 1

                if get_obj_coordinate == len(required_classes):
                    left_obj = calculate_obj_coordinate_hand(
                        self.eye.obj_coordinate_camera_avg, "left"
                    )
                    right_obj = calculate_obj_coordinate_hand(
                        self.eye.obj_coordinate_camera_avg, "right"
                    )

                    self.eye.obj_detect = False
                    if not self._vision_done:
                        self._vision_done = True

                        if self._vision_task_mode == "pour":
                            self.bottle_pos = right_obj["bottle"]
                            self.cup_pos = left_obj["cup"]
                            logger.info(
                                f"bottle pos left: {np.round(left_obj['bottle'], 3)}, "
                                f"right: {np.round(right_obj['bottle'], 3)}"
                            )
                            logger.info(
                                f"cup pos left: {np.round(left_obj['cup'], 3)}, "
                                f"right: {np.round(right_obj['cup'], 3)}"
                            )
                        elif "apple" in required_classes:
                            self.right_target_pos = right_obj["apple"]
                            logger.info(
                                f"apple pos left: {np.round(left_obj['apple'], 3)}, "
                                f"right: {np.round(right_obj['apple'], 3)}"
                            )
                        if "orange" in required_classes:
                            self.left_target_pos = left_obj["orange"]
                            logger.info(
                                f"orange pos left: {np.round(left_obj['orange'], 3)}, "
                                f"right: {np.round(right_obj['orange'], 3)}"
                            )
                        logger.info("Vision done!")

            except Exception as e:
                logger.error(f"Vision loop error: {e}")

        logger.info("Vision loop stopped")

    def pour_water_seq(self, start):
        approach_end = start + 1.
        grasp_end = approach_end + 0.5
        take_end = grasp_end + 1.
        pour_end = take_end + 3.
        rotate_start = pour_end + 1.
        rotate_end = rotate_start + 1.
        down_start = rotate_end + 0.5
        down_end = down_start + 1.
        release_end = down_end + 0.5
        away_end = release_end + 1.
        reset_end = away_end + 1.

        if start <= self.time_ < approach_end:
            hand_q = [0., 1.2, 0., 0., 0., 0.,
                      0., 1.2, 0., 0., 0., 0.]

            target_ee_left = build_pose(self.cup_pos, left_cup_offset)
            target_ee_right = build_pose(self.bottle_pos, right_bottle_offset)
            sol_q, sol_tauff = self.solve_ee_ik(target_ee_left, target_ee_right, euler_order="xyz")

            self.arm_hand_joint_control(start, approach_end, hand_q, sol_q, arm_tau=sol_tauff, armside="both")
            self.waist_joint_control(start, approach_end, waist_q=0.)

        elif approach_end <= self.time_ < grasp_end:
            self.waist_joint_fix(waist_q=0.)
            hand_q = [0.5, 1.2, 0.3, 0.5, 0.5, 0.3,
                      0.5, 1.2, 0.3, 0.5, 0.5, 0.3]
            self.hand_cmd_control(hand_q, handside="both", scale_rad_to_1000=True)

        elif grasp_end <= self.time_ < take_end:
            self.waist_joint_fix(waist_q=0.)
            hand_q = [0.5, 1.2, 0.3, 0.5, 0.5, 0.3,
                      0.5, 1.2, 0.3, 0.5, 0.5, 0.3]
            arm_q = [-0.23, 0.14, -0.03, -0.35, -0.22, 0.34, -0.31,
                     -0.23, -0.14, -0.03, -0.35, 0.22, 0.24, 0.31]

            self.arm_hand_joint_control(grasp_end, take_end, hand_q, arm_q, armside="both")

        elif take_end <= self.time_ < pour_end:
            self.waist_joint_fix(waist_q=0.)
            hand_q = [0.5, 1.2, 0.3, 0.5, 0.5, 0.3,
                      0.5, 1.2, 0.3, 0.5, 0.5, 0.3]

            target_ee_left = [0.32, 0.10, 0.2, 0.03, -0.22, -0.49]
            target_ee_right = [0.27, -0.10, 0.35, -1.5, -0.18, 0.04]

            sol_q, sol_tauff = self.solve_ee_ik(target_ee_left, target_ee_right, euler_order="xyz")
            self.arm_hand_joint_control(take_end, pour_end, hand_q, sol_q, arm_tau=sol_tauff, armside="both")

        elif rotate_start <= self.time_ < rotate_end:
            self.waist_joint_fix(waist_q=0.)
            hand_q = [0.5, 1.2, 0.3, 0.5, 0.5, 0.3,
                      0.5, 1.2, 0.3, 0.5, 0.5, 0.3]

            target_ee_left = [0.32, 0.10, 0.2, 0.03, -0.22, -0.49]
            target_ee_right = [0.27, -0.10, 0.35, -0.9, -0.18, 0.04]

            sol_q, sol_tauff = self.solve_ee_ik(target_ee_left, target_ee_right, euler_order="xyz")
            self.arm_hand_joint_control(rotate_start, rotate_end, hand_q, sol_q, arm_tau=sol_tauff, armside="both")

        elif down_start <= self.time_ < down_end:
            self.waist_joint_fix(waist_q=0.)
            hand_q = [0.5, 1.2, 0.3, 0.5, 0.5, 0.3,
                      0.5, 1.2, 0.3, 0.5, 0.5, 0.3]

            target_ee_left = left_cup_offer_pose.copy()
            target_ee_right = build_pose(self.bottle_pos, right_bottle_offset)

            sol_q, sol_tauff = self.solve_ee_ik(target_ee_left, target_ee_right, euler_order="xyz")
            self.arm_hand_joint_control(down_start, down_end, hand_q, sol_q, arm_tau=sol_tauff, armside="both")

        elif down_end <= self.time_ < release_end:
            self.waist_joint_fix(waist_q=0.)
            hand_q = [0., 1.2, 0., 0., 0., 0.,
                      0., 1.2, 0., 0., 0., 0.]
            self.hand_cmd_control(hand_q, handside="both", scale_rad_to_1000=True)

        elif release_end <= self.time_ < away_end:
            self.waist_joint_fix(waist_q=0.)
            hand_q = [0., 1.2, 0., 0., 0., 0.,
                      0., 1.2, 0., 0., 0., 0.]
            target_ee_left = build_pose(left_cup_offer_pose, left_cup_offer_offset)
            target_ee_right = build_pose(self.bottle_pos, right_bottle_offset)
            sol_q, sol_tauff = self.solve_ee_ik(target_ee_left, target_ee_right, euler_order="xyz")
            self.arm_hand_joint_control(release_end, away_end, hand_q, sol_q, arm_tau=sol_tauff, armside="both")

        elif away_end <= self.time_ < reset_end:
            self.waist_joint_fix(waist_q=0.)
            hand_q = [0., 0., 0., 0., 0., 0.,
                      0., 0., 0., 0., 0., 0.]
            target_ee_left = [0.2, 0.32, 0.18, 0.4, 0., -0.1]
            target_ee_right = [0.2, -0.32, 0.18, -0.4, 0., 0.1]
            sol_q, sol_tauff = self.solve_ee_ik(target_ee_left, target_ee_right, euler_order="xyz")
            self.arm_hand_joint_control(away_end, reset_end, hand_q, sol_q, arm_tau=sol_tauff, armside="both")

        elif self.time_ > reset_end:
            self._pour_motion_done = True

    def grasp_apple_seq(self, start, handside="both"):
        approach_end = start + 1.
        down_end = approach_end + 0.5
        grasp_end = down_end + 0.5
        take_end = grasp_end + 1.

        if start <= self.time_ < approach_end:
            hand_q = [0., 1.2, 0., 0., 0., 0.,
                      0., 1.2, 0., 0., 0., 0.]

            if handside == "left":
                target_ee_left = build_pose(self.left_target_pos, left_orange_offset_approach)
                target_ee_right = list(self.eecmd_fk_buffer.right.pos) + list(self.eecmd_fk_buffer.right.euler)
            elif handside == "right":
                target_ee_left = list(self.eecmd_fk_buffer.left.pos) + list(self.eecmd_fk_buffer.left.euler)
                target_ee_right = build_pose(self.right_target_pos, right_apple_offset_approach)
            else:
                target_ee_left = build_pose(self.left_target_pos, left_orange_offset_approach)
                target_ee_right = build_pose(self.right_target_pos, right_apple_offset_approach)

            sol_q, sol_tauff = self.solve_ee_ik(target_ee_left, target_ee_right, euler_order="xyz")

            self.arm_hand_joint_control(start, approach_end, hand_q, sol_q, arm_tau=sol_tauff, armside=handside)
            self.waist_joint_control(start, approach_end, waist_q=0.)

        if approach_end <= self.time_ < down_end:
            self.waist_joint_fix(waist_q=0.)
            hand_q = [0., 1.2, 0., 0., 0., 0.,
                      0., 1.2, 0., 0., 0., 0.]

            if handside == "left":
                target_ee_left = build_pose(self.left_target_pos, left_orange_offset)
                target_ee_right = list(self.eecmd_fk_buffer.right.pos) + list(self.eecmd_fk_buffer.right.euler)
            elif handside == "right":
                target_ee_left = list(self.eecmd_fk_buffer.left.pos) + list(self.eecmd_fk_buffer.left.euler)
                target_ee_right = build_pose(self.right_target_pos, right_apple_offset)
            else:
                target_ee_left = build_pose(self.left_target_pos, left_orange_offset)
                target_ee_right = build_pose(self.right_target_pos, right_apple_offset)

            sol_q, sol_tauff = self.solve_ee_ik(target_ee_left, target_ee_right, euler_order="xyz")

            self.arm_hand_joint_control(approach_end, down_end, hand_q, sol_q, arm_tau=sol_tauff, armside=handside)

        elif down_end <= self.time_ < grasp_end:
            self.waist_joint_fix(waist_q=0.)
            hand_q = [0.3, 1.46, 0.2, 0.3, 0.4, 0.5,
                      0.3, 1.46, 0.2, 0.3, 0.4, 0.5]
            self.hand_cmd_control(hand_q, handside=handside, scale_rad_to_1000=True)

        elif grasp_end <= self.time_ < take_end:
            self.waist_joint_fix(waist_q=0.)
            hand_q = [0.3, 1.46, 0.2, 0.3, 0.4, 0.5,
                      0.3, 1.46, 0.2, 0.3, 0.4, 0.5]
            arm_q = [-0.23, 0.14, -0.03, -0.35, -0.92, 0.0, -0.0,
                     -0.23, -0.14, -0.03, -0.35, 0.92, 0.0, 0.0]

            self.arm_hand_joint_control(grasp_end, take_end, hand_q, arm_q, armside=handside)
        elif self.time_ > take_end:
            self._release_motion_done = True

    def release_apple_seq(self, start, handside="both"):
        approach_end = start + 1.
        down_end = approach_end + 0.5
        release_end = down_end + 0.5
        up_end = release_end + 0.5
        take_end = up_end + 1.

        if start <= self.time_ < approach_end:
            hand_q = [0.3, 1.46, 0.2, 0.3, 0.4, 0.5,
                      0.3, 1.46, 0.2, 0.3, 0.4, 0.5]

            if handside == "left":
                target_ee_left = build_pose(self.left_hand_target_pos, left_hand_offset_approach)
                target_ee_right = list(self.eecmd_fk_buffer.right.pos) + list(self.eecmd_fk_buffer.right.euler)
            elif handside == "right":
                target_ee_left = list(self.eecmd_fk_buffer.left.pos) + list(self.eecmd_fk_buffer.left.euler)
                target_ee_right = build_pose(self.right_hand_target_pos, right_hand_offset_approach)
            else:
                target_ee_left = build_pose(self.left_hand_target_pos, left_hand_offset_approach)
                target_ee_right = build_pose(self.right_hand_target_pos, right_hand_offset_approach)

            sol_q, sol_tauff = self.solve_ee_ik(target_ee_left, target_ee_right, euler_order="xyz")

            self.arm_hand_joint_control(start, approach_end, hand_q, sol_q, arm_tau=sol_tauff, armside=handside)
            self.waist_joint_control(start, approach_end, waist_q=0.)

        if approach_end <= self.time_ < down_end:
            self.waist_joint_fix(waist_q=0.)
            hand_q = [0.3, 1.46, 0.2, 0.3, 0.4, 0.5,
                      0.3, 1.46, 0.2, 0.3, 0.4, 0.5]

            if handside == "left":
                target_ee_left = build_pose(self.left_hand_target_pos, left_hand_offset)
                target_ee_right = list(self.eecmd_fk_buffer.right.pos) + list(self.eecmd_fk_buffer.right.euler)
            elif handside == "right":
                target_ee_left = list(self.eecmd_fk_buffer.left.pos) + list(self.eecmd_fk_buffer.left.euler)
                target_ee_right = build_pose(self.right_hand_target_pos, right_hand_offset)
            else:
                target_ee_left = build_pose(self.left_hand_target_pos, left_hand_offset)
                target_ee_right = build_pose(self.right_hand_target_pos, right_hand_offset)

            sol_q, sol_tauff = self.solve_ee_ik(target_ee_left, target_ee_right, euler_order="xyz")

            self.arm_hand_joint_control(approach_end, down_end, hand_q, sol_q, arm_tau=sol_tauff, armside=handside)

        elif down_end <= self.time_ < release_end:
            self.waist_joint_fix(waist_q=0.)
            hand_q = [0., 1.2, 0., 0., 0., 0.,
                      0., 1.2, 0., 0., 0., 0.]
            self.hand_cmd_control(hand_q, handside=handside, scale_rad_to_1000=True)

        elif release_end <= self.time_ < up_end:
            self.waist_joint_fix(waist_q=0.)
            hand_q = [0., 0., 0., 0., 0., 0.,
                      0., 0., 0., 0., 0., 0.]
            if handside == "left":
                target_ee_left = build_pose(self.left_hand_target_pos, left_hand_offset_approach)
                target_ee_right = list(self.eecmd_fk_buffer.right.pos) + list(self.eecmd_fk_buffer.right.euler)
            elif handside == "right":
                target_ee_left = list(self.eecmd_fk_buffer.left.pos) + list(self.eecmd_fk_buffer.left.euler)
                target_ee_right = build_pose(self.right_hand_target_pos, right_hand_offset_approach)
            else:
                target_ee_left = build_pose(self.left_hand_target_pos, left_hand_offset_approach)
                target_ee_right = build_pose(self.right_hand_target_pos, right_hand_offset_approach)

            sol_q, sol_tauff = self.solve_ee_ik(target_ee_left, target_ee_right, euler_order="xyz")

            self.arm_hand_joint_control(release_end, up_end, hand_q, sol_q, arm_tau=sol_tauff, armside=handside)

        elif up_end <= self.time_ < take_end:
            self.waist_joint_fix(waist_q=0.)
            hand_q = [0., 0., 0., 0., 0., 0.,
                      0., 0., 0., 0., 0., 0.]
            target_ee_left = [0.2, 0.32, 0.18, 0.4, 0., -0.1]
            target_ee_right = [0.2, -0.32, 0.18, -0.4, 0., 0.1]
            sol_q, sol_tauff = self.solve_ee_ik(target_ee_left, target_ee_right, euler_order="xyz")
            self.arm_hand_joint_control(up_end, take_end, hand_q, sol_q, arm_tau=sol_tauff, armside=handside)

        elif self.time_ > take_end:
            self._grasp_motion_done = True

    def shutdown_vision_cameras(self):
        self._vision_running = False
        if self._vision_thread is not None:
            self._vision_thread.join(timeout=1.0)
            self._vision_thread = None

        if self.eye is not None and self.eye.pipeline is not None:
            self.eye.pipeline.stop()
            self.eye.pipeline = None

        if self.hand_eye is not None:
            self.hand_eye.close()
            self.hand_eye = None

    def publish_head_image(self):
        if self.eye is None or self.eye.color_image is None:
            return

        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = "jpeg"

        success, encoded_image = cv2.imencode('.jpg', self.eye.color_image)

        if not success:
            self.get_logger().error("Image encode failed")
            return

        msg.data = encoded_image.tobytes()

        self.camera_publishers["cam_head"].publish(msg)


    def dataset_replay(self, speed=1.0):
        if self.replay_frame_pos >= self.replay_num_frames - 1:
            logger.info("Replay finished")
            self.update_cmd_buffer("both")
            if hasattr(self, "mark_active_action_done"):
                self.mark_active_action_done()
            self._timer.cancel()
            return

        idx0 = int(np.floor(self.replay_frame_pos))
        idx1 = min(idx0 + 1, self.replay_num_frames - 1)
        alpha = self.replay_frame_pos - idx0

        action0 = self.replay_actions[idx0][ACTION]
        action1 = self.replay_actions[idx1][ACTION]

        # === 解算两帧 ===
        arm_l_0 = np.array(action_to_arm_cmd(action0, self.left_arm_mapping, self.convert_deg))
        arm_r_0 = np.array(action_to_arm_cmd(action0, self.right_arm_mapping, self.convert_deg))
        hand_l_0 = np.array(action_to_hand_cmd(action0, self.left_hand_indices, self.use_named_hand, 'left'))
        hand_r_0 = np.array(action_to_hand_cmd(action0, self.right_hand_indices, self.use_named_hand, 'right'))

        arm_l_1 = np.array(action_to_arm_cmd(action1, self.left_arm_mapping, self.convert_deg))
        arm_r_1 = np.array(action_to_arm_cmd(action1, self.right_arm_mapping, self.convert_deg))
        hand_l_1 = np.array(action_to_hand_cmd(action1, self.left_hand_indices, self.use_named_hand, 'left'))
        hand_r_1 = np.array(action_to_hand_cmd(action1, self.right_hand_indices, self.use_named_hand, 'right'))


        # === 线性插值 ===
        arm_l = (1 - alpha) * arm_l_0 + alpha * arm_l_1
        arm_r = (1 - alpha) * arm_r_0 + alpha * arm_r_1
        hand_l = (1 - alpha) * hand_l_0 + alpha * hand_l_1
        hand_r = (1 - alpha) * hand_r_0 + alpha * hand_r_1

        target_arm_q = np.concatenate([arm_l, arm_r])
        target_hand = np.concatenate([hand_l, hand_r])

        if 0 <= self.time_ < 1:
            self.arm_hand_joint_control(0., 1., target_hand, target_arm_q, armside="both")
            self.waist_joint_control(0., 1., waist_q=0.)
        else:
            self.arm_cmd_control(ratio=1., arm_q=target_arm_q, arm_tauff=np.zeros(14), armside="both")
            self.hand_cmd_control(target_hand, handside="both", scale_rad_to_1000=True)
            self.waist_joint_fix(waist_q=0.)

        # === 推进连续帧 ===
        self.replay_frame_pos += speed
        self.replay_frame_idx = int(self.replay_frame_pos)

        logger.info(f"time: {self.replay_frame_idx / self.replay_fps:.2f}/"
                    f"{self.replay_num_frames / self.replay_fps:.2f} s")
