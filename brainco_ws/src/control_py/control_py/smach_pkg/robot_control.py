import sys, os
import numpy as np

from unitree_hg.msg import LowState
from unitree_hg.msg import LowCmd
from sensor_msgs.msg import JointState

pkgs_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

sys.path.append(pkgs_dir)

arm_urdf_path = pkgs_dir + '/arm_ik_control/unitree_g1_description/' # 宇树urdf文件路径
arm_urdf_name = 'g1_23dof.urdf'

from arm_ik_control.cal_arm_ik import Arm, Hand
from arm_ik_control.arm_23_joint import G1JointIndex


class RobotControl:
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.ready_to_start = False
        
        # Unitree Arm
        self.arm = Arm()
        self.low_cmd = LowCmd()
        self.low_state = None 
        self.zero_arm_q = [0.4, 0.2, 0., 0.5, 0.,
                           0.4, -0.2, 0., 0.5, -0.]
        
        self.target_arm_q = [0., 0.3, 0., 1.5, 0.,
                             0., -0.3, 0., 1.5, 0.]
        
        self.target_waist = 0.
        self.arm_state_st = None    # 计算匀速运动的起始位置

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

        # Publisher and Subscriber
        self.publisher = self.create_publisher(LowCmd,"arm_sdk",10) 
        self.publisher_left = self.create_publisher(JointState, 'joint_commands_left', 10)
        self.publisher_right = self.create_publisher(JointState, 'joint_commands_right', 10)
        self.subscriber = self.create_subscription(LowState, "lowstate", self.ArmStateHandler, 10)

         # Timer Init
        self._timer = None

        self.time_ = 0.
        self.control_dt_ = 0.01

        # IK
        self.get_logger().info("Initializing IK...")
        self.arm.init_ik(arm_urdf_path + arm_urdf_name, arm_urdf_path)
        self.arm.ik.curr_lr_q = [0., 0.3, 0., 1.5, 0.,
                                 0., -0.3, 0., 1.5, 0.]
        self.get_logger().info("IK initialization done.\n")

        self.log_time, self.log_arm_state, self.log_arm_cmd, self.log_hand = True, False, True, True

    def ArmStateHandler(self, msg: LowState):
        self.low_state = msg
        if self.log_arm_state:
            low_state_q = [msg.motor_state[joint].q for joint in self.arm.arm_joints]
            self.get_logger().info("Arm State: %s" % np.round(low_state_q, 3))

    def publish_all(self):
        self.publisher.publish(self.low_cmd)
        self.publisher_left.publish(self.hand_cmd_left)
        self.publisher_right.publish(self.hand_cmd_right)

        if self.log_time:
            self.get_logger().info(f"time {round(self.time_, 2)}")
        if self.log_arm_cmd:
            low_cmd = [self.low_cmd.motor_cmd[joint].q for joint in self.arm.arm_joints]
            self.get_logger().info(f"arm_pub: {np.round(low_cmd, 3)}")
        if self.log_hand:
            self.get_logger().info(f"hand_pub: {list(self.hand_cmd_left.position)} {list(self.hand_cmd_right.position)}")

    
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
            if arm == "left" and i<5 or arm == "right" and i>=5 or arm == "both":
                self.low_cmd.motor_cmd[joint].tau = self.arm.tau_ff[i]
                self.low_cmd.motor_cmd[joint].dq = self.arm.dq
                self.low_cmd.motor_cmd[joint].kp = self.arm.kp[i]
                self.low_cmd.motor_cmd[joint].kd = self.arm.kd
                self.low_cmd.motor_cmd[joint].q = ratio * self.target_arm_q[i] + (1.0 - ratio) * self.arm_state_st[i]
                self.arm.ik.curr_lr_q[i] = self.low_cmd.motor_cmd[joint].q
        self.waist_control(1.)  # 腰部固定
        
    # 固定腰部
    def waist_control(self, ratio): 
        self.low_cmd.motor_cmd[12].tau = 0.
        self.low_cmd.motor_cmd[12].dq = self.arm.dq
        self.low_cmd.motor_cmd[12].kp = 60. 
        self.low_cmd.motor_cmd[12].kd = self.arm.kd
        self.low_cmd.motor_cmd[12].q = ratio * self.target_waist + (1.0 - ratio) * self.low_state.motor_state[12].q
    
    # 灵巧手控制
    def hand_control(self):
        self.hand_cmd_left.header.stamp = self.get_clock().now().to_msg()
        self.hand_cmd_right.header.stamp = self.get_clock().now().to_msg()
        self.hand_cmd_left.position = self.target_hand[:6]
        self.hand_cmd_right.position = self.target_hand[6:]

    
    # 为下个匀速运动记录起始状态
    def store_curr_cmd(self, arm):
        if self.low_cmd is not None and self.arm_state_st is not None:
            low_cmd = [self.low_cmd.motor_cmd[joint].q for joint in self.arm.arm_joints]
            if arm != "right":
                self.arm_state_st[:5] =  low_cmd[:5]
            if arm != "left":
                self.arm_state_st[5:] =  low_cmd[5:]


    
    
    # 手臂灵巧手位置控制（匀速运动）
    # 选择 arm 的情况下 update_arm_q, update_hand 只会赋值 arm 侧手臂和手
    def arm_hand_control(self, st_s, end_s, arm, update_arm_q=None, update_hand=None):
        if st_s <= self.time_ < end_s:
            # 使用 update_arm_q, update_hand 更新控制
            # 或使用 self.arm.ik.left_wrist, self.arm.ik.right_wrist, self.target_hand 更新控制
            
            # self.target_arm_q = update_arm_q.copy() if update_arm_q is not None else self.arm.ik.cal_ik()[0]
            # self.target_hand = update_hand.copy() if update_hand is not None else self.target_hand

            if arm != 'left':
                self.target_arm_q[5:] = update_arm_q[5:].copy() if update_arm_q is not None else self.arm.ik.cal_ik()[0][5:]
                self.target_hand[6:] = update_hand[6:].copy() if update_hand is not None else self.target_hand[6:]
            if arm != 'right':
                self.target_arm_q[:5] = update_arm_q[:5].copy() if update_arm_q is not None else self.arm.ik.cal_ik()[0][:5]
                self.target_hand[:6] = update_hand[:6].copy() if update_hand is not None else self.target_hand[:6]

            ratio = np.clip((self.time_ - st_s) / ((end_s - st_s - 0.01)), 0.0, 1.0)
            if self.arm_state_st is None:
                self.arm_state_st = [self.low_state.motor_state[joint].q for joint in self.arm.arm_joints]
            elif ratio == 1.:
                self.store_curr_cmd(arm)

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