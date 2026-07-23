import numpy as np

from control_py.state_manager.robot_control import *
from control_py.arm_ik_control.arm_joints import G1JointIndex

from loguru import logger
from control_py.utils.loguru_settings import setup_loguru
setup_loguru(log_folder_path="log", show_on_terminal=True, terminal_level='DEBUG') # 设置日志

class BasicStates(RobotControl):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.ready_to_start = True

        

    
    # # 开始阶段（激活宇树SDK）
    # def arm_hand_start(self, st_s, end_s, update_arm_q=None, update_hand=None):
    #     if st_s <= self.time_ < end_s:
    #         # 初始化电机
    #         self.robot_cmd_init()
    #         # 激活SDK 1:Enable arm_sdk, 0:Disable arm_sdk
    #         self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q =  1.0
    #         # 移动到初始位置
    #         self.arm_hand_control(st_s, end_s, "both", update_arm_q=update_arm_q, update_hand=update_hand)

    # 开始阶段（激活宇树SDK）
    def arm_hand_start(self, start, end, update_arm_q=None, update_hand=None):
        if start <= self.time_ < end:
            # 初始化电机
            self.robot_cmd_init()
            # 激活SDK 1:Enable arm_sdk, 0:Disable arm_sdk
            self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q =  1.0
            # 移动到初始位置
            self.arm_hand_joint_control(start, end, update_hand, update_arm_q, armside="both")
            self.waist_joint_control(start, end, waist_q=0.)

    # 手臂回到零位
    def arm_back_zero(self, start, end, armside):
        self.arm_hand_joint_control(start, end, self.zero_hand, self.zero_arm_q, armside=armside)
        self.waist_joint_control(start, end, waist_q=0.)

    # 结束阶段（停止激活宇树SDK，回到初始位置）
    def arm_hand_end(self, start):
        if  self.time_ >= start:
            # 停止激活SDK 1:Enable arm_sdk, 0:Disable arm_sdk
            ratio = np.clip((self.time_ - start) / 4., 0.0, 1.0)
            self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q =  (1 - ratio)
            # 缓慢回归零位
            self.arm_hand_joint_control(start, start + 4., self.zero_hand, self.zero_arm_q, armside="both")
            self.waist_joint_control(start, start + 4., waist_q=0.)

    
    # 开始阶段（激活宇树SDK）(桌子安全防撞)
    def arm_hand_start_table_safe(self, lr_height, handside="both"):
        if 0. <= self.time_ < 1.:
            # 初始化电机
            self.robot_cmd_init()
            # 激活SDK 1:Enable arm_sdk, 0:Disable arm_sdk
            self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q =  1.0
        
        self.arm_hand_back_zero_table_safe(lr_height, handside=handside)


    # 手臂回到零位(桌子安全防撞)
    def arm_hand_back_zero_table_safe(self, lr_height, handside="both"):
        if 0. <= self.time_ < 1.:
            self.waist_joint_control(0., 1., waist_q=0.)
            arm_q = self._table_safe_arm_target()
            if lr_height[0] < self.safe_height_threshold:
                arm_q = self._replace_arm_side_target(arm_q, "left", self.zero_arm_q)
            if lr_height[1] < self.safe_height_threshold:
                arm_q = self._replace_arm_side_target(arm_q, "right", self.zero_arm_q)

            self.arm_hand_joint_control(0., 1., self.zero_hand, arm_q, armside=handside)
        
        elif 1. <= self.time_ < 2.:
            self.waist_joint_fix(waist_q=0.)
            self.arm_hand_joint_control(1., 2., self.zero_hand, self.zero_arm_q, armside=handside)
