import sys, os
import numpy as np
import rclpy
from rclpy.node import Node

sys.path.append(os.getcwd() + '/src/control_py/control_py/')
from arm_ik_control.cal_arm_ik import Arm, Hand
from arm_ik_control.arm_23_joint import G1JointIndex

unitree_msg_path = '/home/unitree/unitree_ros2/cyclonedds_ws/src/unitree/' # 安装宇树ros定义通信msg的位置
arm_urdf_path = '/home/unitree/unitree_sdk2_python/example/assets/g1/' # 宇树urdf文件路径

sys.path.append(unitree_msg_path)
from unitree_hg.msg import LowState
from unitree_hg.msg import LowCmd
from sensor_msgs.msg import JointState


class MainNode(Node):
    def __init__(self, name):
        super().__init__(name)

        # Unitree Arm

        self.arm = Arm()
        self.low_cmd = LowCmd()
        self.low_state = None 

        self.zero_arm_q = [0.4, 0.2, 0., 0.5, 0.,
                           0.4, -0.2, 0., 0.5, -0.]
        
        self.target_arm_q = [0., 0.3, 0., 1.5, 0.,
                             0., -0.3, 0., 1.5, 0.]
        
        self.left_arm_store = [0., 0., 0., 0., 0., 0.]
        self.right_arm_store = [0., 0., 0., 0., 0., 0.]
        
        self.target_waist = 0.
        self.arm_state_st = None

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
        
        # Timer
        self.timer = self.create_timer(0.01, self.LowCmdWrite)
        self.get_logger().info("Start Timer")

        self.time_ = -0.1
        self.control_dt_ = 0.01
        self.prepare = 10    


        # IK
        self.get_logger().info("Initializing IK...")
        self.arm.init_ik(arm_urdf_path + 'g1_body23.urdf', arm_urdf_path)
        self.arm.ik.curr_lr_q = [0., 0.3, 0., 1.5, 0.,
                                 0., -0.3, 0., 1.5, 0.]
        self.get_logger().info("IK initialization done.")


    def ArmStateHandler(self, msg: LowState):
        self.low_state = msg
        low_state_q = [msg.motor_state[joint].q for joint in self.arm.arm_joints]
        self.get_logger().info("Arm State: %s" % np.round(low_state_q, 2))

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
        self.waist_control(ratio)
        
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
    
    # 手臂灵巧手位置控制（匀速运动）
    def arm_hand_control(self, st_s, end_s, arm, update_arm_q=None, update_hand=None):
        if st_s <= self.time_ < end_s:
            # 使用 update_arm_q, update_hand 更新控制
            # 或使用 self.arm.ik.left_wrist, self.arm.ik.right_wrist, self.target_hand 更新控制
            self.target_arm_q = update_arm_q.copy() if update_arm_q is not None else self.arm.ik.cal_ik()[0]
            self.target_hand = update_hand.copy() if update_hand is not None else self.target_hand
            ratio = np.clip((self.time_ - st_s) / ((end_s - st_s - 0.01)), 0.0, 1.0)
            if self.arm_state_st is None:
                self.arm_state_st = [self.low_state.motor_state[joint].q for joint in self.arm.arm_joints]
            elif ratio == 1.:
                low_cmd = [self.low_cmd.motor_cmd[joint].q for joint in self.arm.arm_joints]
                if arm != "right":
                    self.arm_state_st[:5] =  low_cmd[:5]
                if arm != "left":
                    self.arm_state_st[5:] =  low_cmd[5:]

            self.arm_control(ratio, arm)
            self.hand_control()
            

    def timer_close(self, end_s):
        if self.time_ > end_s:
            input("Press Ctrl+C to close.")

    # 挥手
    def show_hello(self, start, end, arm, speed=1.):
        if start <= self.time_ < end:
            duration = round(1/speed, 3)
            pos_hand = [0., 0., 0., 0., 0., 0.]
            if self.time_ < start + 1.:
                pos_arm_right0 = [0.3, -0.05, -0.1, 90., 0., 0.]
                self.target_double_arm(arm, pos_arm_right0, pos_hand)
                self.arm_hand_control(start, start + 1., arm)
            elif self.time_ >= start + 1.:
                pos_arm_right1 = [0.35, -0.05, -0.1, 90., 0., 0.]
                pos_arm_right2 = [0.25, -0.05, -0.1, 90., 0., 0.]
                t = (self.time_ - start - 1.) % duration
                if 0. <= t < duration/2:
                    self.target_double_arm(arm, pos_arm_right1, pos_hand)
                    self.arm_hand_control(self.time_ - t, self.time_ - t + duration/2, arm)

                elif duration/2 <= t < duration:
                    self.target_double_arm(arm, pos_arm_right2, pos_hand)
                    self.arm_hand_control(self.time_ - t + duration/2, self.time_ - t + duration, arm)

    # 点赞
    def show_like(self, start, end, arm):
        if start <= self.time_ < end:
            pos_arm_right = [0.23, -0.3, -0.25, 0., 0., -90.]
            pos_hand = [0., 0., 0.9, 0.9, 0.9, 0.9]
            self.target_double_arm(arm, pos_arm_right, pos_hand)
            self.arm_hand_control(start, end, arm)

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


    def LowCmdWrite(self):
        self.time_ += self.control_dt_

        if self.time_ < 0:
           print("Start")

        else:
            # [Stage 1]: set robot to zero posture
            self.arm_hand_start(0, 2, update_arm_q=self.zero_arm_q, update_hand=self.zero_hand)

            # [Stage 2]: start action
            self.show_hello(2, 10, "right", speed=1.5)
            self.show_like(2, 3, "left")
            
            # [Stage 3]: put the arm down to zero posture
            self.arm_hand_control(11, 13, "both",
                                  update_arm_q=self.zero_arm_q, 
                                  update_hand=self.zero_hand)

            # [Stage 4]: release arm_sdk
            self.arm_hand_end(13)
            self.timer_close(15)

            self.publisher.publish(self.low_cmd)
            self.publisher_left.publish(self.hand_cmd_left)
            self.publisher_right.publish(self.hand_cmd_right)

            low_cmd = [self.low_cmd.motor_cmd[joint].q for joint in self.arm.arm_joints]

            print("arm_pub:", np.round(low_cmd, 2))
            print("hand_pub:", list(self.hand_cmd_left.position), list(self.hand_cmd_right.position))
            


def main(args=None):
    rclpy.init(args=args)
    
    node = MainNode("hello_pub")
    input("Press ENTER to start.")
    
    rclpy.spin(node)
    rclpy.shutdown()
 

if __name__ == "__main__":
    main() 