import sys, os, copy
import numpy as np

from unitree_hg.msg import LowState
from unitree_hg.msg import LowCmd
from sensor_msgs.msg import JointState

pkgs_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

sys.path.append(pkgs_dir)

arm_urdf_path = '/home/unitree/g1_description/' # 宇树urdf文件路径

from arm_ik_control.cal_arm_ik import Arm, Hand
from arm_ik_control.arm_joints import G1JointIndex

from loguru import logger
from utils.loguru_settings import setup_loguru
setup_loguru(log_folder_path="log", show_on_terminal=True) # 设置日志


class RobotControl:
    def __init__(self, *args, **kwargs):
        
        super().__init__(*args, **kwargs)

        self.ready_to_start = False

        # config 参数
        self.robot_dof = None
        self.log_time, self.log_arm_state, self.log_arm_cmd, self.log_wrist_pose, self.log_hand = True, True, True, True, True

        # Publisher and Subscriber
        self.publisher = self.create_publisher(LowCmd,"arm_sdk",10) 
        self.publisher_left = self.create_publisher(JointState, 'joint_commands_left', 10)
        self.publisher_right = self.create_publisher(JointState, 'joint_commands_right', 10)
        self.subscriber = self.create_subscription(LowState, "lowstate", self.ArmStateHandler, 10)

         # Timer Init
        self._timer = None

        self.time_ = 0.
        self.control_dt_ = 0.01


    def robot_control_initialization(self):
        if self.robot_dof not in (23, 29):
            raise ValueError(f"robot_dof must be 23 or 29, got {self.robot_dof}")
        self.arm_dof = 5 if self.robot_dof == 23 else 7
        logger.info(f"3-robot_dof: {self.robot_dof}")

        # Unitree Arm
        self.arm = Arm(self.robot_dof)
        self.low_cmd = LowCmd()
        self.low_state = None 
        if self.robot_dof == 23:
            self.zero_arm_q = [0.4, 0.2, 0., 0.5, 0.,
                            0.4, -0.2, 0., 0.5, -0.]
            
            self.target_arm_q = [0., 0.3, 0., 1.5, 0.,
                                0., -0.3, 0., 1.5, 0.]
        else:
            self.zero_arm_q = [0.4, 0.2, 0., 0.5, 0., 0., 0.,
                            0.4, -0.2, 0., 0.5, 0., 0., 0.]
            
            self.target_arm_q = [0., 0.3, 0., 1.5, 0., 0., 0.,
                                0., -0.3, 0., 1.5, 0., 0., 0.]
        
        self.target_waist = 0.
        self.arm_state_st = None    # 计算匀速运动的起始关节角(弧度)
        self.arm_pos_st_left, self.arm_pos_st_right = None, None  # 计算匀速运动的起始姿态(xyzabc)

        # BrainCo Hand
        self.hand = Hand()
        self.hand_cmd_left = JointState()
        self.hand_cmd_right = JointState()
        self.hand_cmd_left.name = self.hand.joint_names
        self.hand_cmd_right.name = self.hand.joint_names

        self.zero_hand = [0., 0., 0., 0., 0., 0., 
                          0., 0., 0., 0., 0., 0.]
        
        self.target_hand = [0., 0., 0., 0., 0., 0.,
                            0., 0., 0., 0., 0., 0.]
        
         # IK
        logger.info("Initializing IK...")

        if self.robot_dof == 23:
            self.arm.init_ik(arm_urdf_path + 'g1_23dof.urdf', arm_urdf_path)
            self.arm.ik.curr_lr_q = [0., 0.3, 0., 1.5, 0.,
                                    0., -0.3, 0., 1.5, 0.]
        else:
            self.arm.init_ik(arm_urdf_path + 'g1_29dof.urdf', arm_urdf_path)
            self.arm.ik.curr_lr_q = [0., 0.3, 0., 1.5, 0., 0., 0.,
                                    0., -0.3, 0., 1.5, 0., 0., 0.]
        self.get_logger().info("IK initialization done.\n")
        

    def ArmStateHandler(self, msg: LowState):
        self.low_state = msg
        if self.log_arm_state:
            low_state_q = [msg.motor_state[joint].q for joint in self.arm.arm_joints]
            logger.info("Arm State: %s" % np.round(low_state_q, 3))

    def publish_all(self):
        self.publisher.publish(self.low_cmd)
        self.publisher_left.publish(self.hand_cmd_left)
        self.publisher_right.publish(self.hand_cmd_right)

        if self.log_time:
            logger.info(f"time {round(self.time_, 2)}")
        if self.log_arm_cmd:
            low_cmd = [self.low_cmd.motor_cmd[joint].q for joint in self.arm.arm_joints]
            logger.info(f"arm_pub: {np.round(low_cmd, 3)}")
        if self.log_hand:
            logger.info(f"hand_pub: {list(self.hand_cmd_left.position)} {list(self.hand_cmd_right.position)}")
        if self.log_wrist_pose:
            logger.info(f"wrist_pose: {np.round(self.arm.ik.left_wrist, 2)} {np.round(self.arm.ik.right_wrist, 2)}")

    
    def clear_timer(self):
        # 停用功能：停止定时器
        if self._timer is not None:
            self._timer.cancel()
            self._timer = None # 清除引用
            self.get_logger().info("Timer stopped.")
        # 创建并启动定时器
        self.time_ = 0.


    # 手臂控制 arm=left/right/both
    def arm_control(self, ratio, arm):
        for i, joint in enumerate(self.arm.arm_joints):
            if arm == "left" and i<self.arm_dof or arm == "right" and i>=self.arm_dof or arm == "both":
                self.low_cmd.motor_cmd[joint].tau = self.arm.tau_ff[i]
                self.low_cmd.motor_cmd[joint].dq = self.arm.dq
                self.low_cmd.motor_cmd[joint].kp = self.arm.kp[i]
                self.low_cmd.motor_cmd[joint].kd = self.arm.kd
                self.low_cmd.motor_cmd[joint].q = ratio * self.target_arm_q[i] + (1.0 - ratio) * self.arm_state_st[i]
                self.arm.ik.curr_lr_q[i] = self.low_cmd.motor_cmd[joint].q
        self.waist_control(1.)  # 腰部固定


    def arm_control_split(self, ratio_l, ratio_r, arm):
        for i, joint in enumerate(self.arm.arm_joints):
            if arm == "left" and i<self.arm_dof or arm == "right" and i>=self.arm_dof or arm == "both":
                self.low_cmd.motor_cmd[joint].dq = self.arm.dq
                self.low_cmd.motor_cmd[joint].tau = self.arm.tau_ff[i]
                self.low_cmd.motor_cmd[joint].kp = self.arm.kp[i]
                self.low_cmd.motor_cmd[joint].kd = self.arm.kd
                if i >= self.arm_dof:
                    self.low_cmd.motor_cmd[joint].q = ratio_r * self.target_arm_q[i] + (1.0 - ratio_r) * self.arm_state_st[i]
                else:
                    self.low_cmd.motor_cmd[joint].q = ratio_l * self.target_arm_q[i] + (1.0 - ratio_l) * self.arm_state_st[i]
            # self.curr_lr_q[i] = self.low_cmd.motor_cmd[joint].q
        self.waist_control(1.)

        if ratio_l == 1. or ratio_r == 1.:
            low_cmd = [self.low_cmd.motor_cmd[joint].q for joint in self.arm.arm_joints]
            if ratio_l == 1.:
                self.arm_state_st[:self.arm_dof] =  low_cmd[:self.arm_dof]
            if ratio_r == 1.:
                self.arm_state_st[self.arm_dof:] =  low_cmd[self.arm_dof:]

        
    # 固定腰部
    def waist_control(self, ratio): 
        for i in [12, 13, 14]:
            self.low_cmd.motor_cmd[i].tau = 0.
            self.low_cmd.motor_cmd[i].dq = self.arm.dq
            self.low_cmd.motor_cmd[i].kp = 60. 
            self.low_cmd.motor_cmd[i].kd = self.arm.kd
            self.low_cmd.motor_cmd[i].q = ratio * self.target_waist + (1.0 - ratio) * self.low_state.motor_state[i].q
    
    # 灵巧手控制
    def hand_control(self):
        self.hand_cmd_left.header.stamp = self.get_clock().now().to_msg()
        self.hand_cmd_right.header.stamp = self.get_clock().now().to_msg()
        self.hand_cmd_left.position = self.target_hand[:6]
        self.hand_cmd_right.position = self.target_hand[6:]

    
    # 为下个匀速运动记录起始关节角 为下个抛物运动记录起始姿态
    def store_curr_cmd(self, arm, trans_space=False):
        
        if self.low_cmd is not None and self.arm_state_st is not None:
            curr_arm_q = [self.low_cmd.motor_cmd[joint].q for joint in self.arm.arm_joints]
        else:
            curr_arm_q = [self.low_state.motor_state[joint].q for joint in self.arm.arm_joints]
            self.arm_state_st = [self.low_state.motor_state[joint].q for joint in self.arm.arm_joints]
        
        if trans_space:
            lw, rw = self.arm.ik.cal_fk(curr_arm_q)
        else:
            lw = copy.deepcopy(self.arm.ik.left_wrist)
            rw = copy.deepcopy(self.arm.ik.right_wrist)
        
        if arm != "right":
            self.arm_state_st[:self.arm_dof] =  curr_arm_q[:self.arm_dof]
            self.arm_pos_st_left = lw.copy()
        
        if arm != "left":
            self.arm_state_st[self.arm_dof:] =  curr_arm_q[self.arm_dof:]
            self.arm_pos_st_right = rw.copy()
        

    
    
    # 手臂灵巧手位置控制（匀速运动）
    # 选择 arm 的情况下 update_arm_q, update_hand 只会赋值 arm 侧手臂和手
    def arm_hand_control(self, st_s, end_s, arm, update_arm_q=None, update_hand=None):
        if st_s <= self.time_ < end_s:
            # 使用 update_arm_q (弧度), update_hand (弧度) 更新控制
            # 或使用 self.arm.ik.left_wrist, self.arm.ik.right_wrist, self.target_hand 更新控制
            
            # self.target_arm_q = update_arm_q.copy() if update_arm_q is not None else self.arm.ik.cal_ik()[0]
            # self.target_hand = update_hand.copy() if update_hand is not None else self.target_hand

            if arm != 'left':
                self.target_arm_q[self.arm_dof:] = update_arm_q[self.arm_dof:].copy() if update_arm_q is not None else self.arm.ik.cal_ik()[0][self.arm_dof:]
                self.target_hand[6:] = update_hand[6:].copy() if update_hand is not None else self.target_hand[6:]
            if arm != 'right':
                self.target_arm_q[:self.arm_dof] = update_arm_q[:self.arm_dof].copy() if update_arm_q is not None else self.arm.ik.cal_ik()[0][:self.arm_dof]
                self.target_hand[:6] = update_hand[:6].copy() if update_hand is not None else self.target_hand[:6]

            ratio = np.clip((self.time_ - st_s) / ((end_s - st_s - 0.02)), 0.0, 1.0)
            if self.arm_state_st is None:
                # self.arm_state_st = [self.low_state.motor_state[joint].q for joint in self.arm.arm_joints]
                # self.arm_pos_st_left = copy.deepcopy(self.arm.ik.left_wrist)
                # self.arm_pos_st_right = copy.deepcopy(self.arm.ik.right_wrist)
                self.store_curr_cmd(arm, trans_space=False)
            elif ratio == 1.:
                self.store_curr_cmd(arm, trans_space=False)

            self.arm_control(ratio, arm)
            self.hand_control()

    # 手臂回到零位
    def arm_back_zero(self, st_s, end_s, arm):
        self.arm_hand_control(st_s, end_s, arm, update_arm_q=self.zero_arm_q, update_hand=self.zero_hand)

    # 开始阶段（激活宇树SDK）
    def arm_hand_start(self, st_s, end_s, update_arm_q=None, update_hand=None):
        if st_s <= self.time_ < end_s:
            # 激活SDK 1:Enable arm_sdk, 0:Disable arm_sdk
            self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q =  1.0
            self.arm_hand_control(st_s, end_s, "both", update_arm_q=update_arm_q, update_hand=update_hand)

    # 结束阶段（停止激活宇树SDK，回到初始位置）
    def arm_hand_end(self, st_s):
        if  self.time_ >=st_s:
            # 停止激活SDK 1:Enable arm_sdk, 0:Disable arm_sdk
            ratio = np.clip((self.time_ - st_s) / 4., 0.0, 1.0)
            self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q =  (1 - ratio)
            self.arm_hand_control(st_s, st_s + 4., "both", update_arm_q=self.zero_arm_q, update_hand=self.zero_hand)

    # 两臂位置对应关系
    def target_double_arm(self, arm, pos_arm_right, pos_hand):
        if arm != "left":
            multiplier = [1, 1, 1, 1, 1, 1]
            self.arm.ik.right_wrist = [m * pos for m, pos in zip(multiplier, pos_arm_right)]
            self.target_hand[6:] = pos_hand.copy()
        if arm != "right":
            multiplier = [-1, 1, 1, 1, -1, -1]
            self.arm.ik.left_wrist = [m * pos for m, pos in zip(multiplier, pos_arm_right)]
            self.target_hand[:6] = pos_hand.copy()  

    # 单臂，修改target
    def target_diff_arm(self, arm, pos_arm, pos_hand):
        self.arm.ik.__dict__[arm + "_wrist"] = pos_arm.copy()
        if arm == "left":
            self.target_hand[:6] = pos_hand.copy() 
        elif arm == "right":
            self.target_hand[6:] = pos_hand.copy()

    



            

            