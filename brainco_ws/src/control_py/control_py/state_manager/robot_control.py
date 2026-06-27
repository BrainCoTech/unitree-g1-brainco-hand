import copy, yaml, cv2, time, gc, math
from typing import Optional
import numpy as np
import pyrealsense2 as rs

from sensor_msgs.msg import JointState, CompressedImage
from unitree_hg.msg import LowState
from unitree_hg.msg import LowCmd
from ros2_stark_msgs.msg import SetMotorMulti
from ros2_stark_msgs.msg import MotorStatus
arm_urdf_path = '/home/unitree/g1_description/' # 宇树urdf文件路径

from rclpy.qos import qos_profile_sensor_data

from control_py.arm_ik_control.arm_joints import G1JointIndex
from control_py.arm_ik_control.cal_arm_ik import Hand
from control_py.unitree_robot.robot_arm_ik import G1_29_ArmIK_new, G1_23_ArmIK_new
from control_py.unitree_robot.robot_arm_real import (
    G1_29_ArmController, G1_29_JointIndex, G1_29_JointArmIndex,
    G1_23_ArmController, G1_23_JointIndex, G1_23_JointArmIndex
)
from control_py.teleop_pkg.opencv_cam import CameraConfig, shutdown_dict
from control_py.teleop_pkg.data_process_new import *

from loguru import logger
from control_py.utils.loguru_settings import setup_loguru, logger_with_params
setup_loguru(log_folder_path="log", show_on_terminal=True, terminal_level='DEBUG') # 设置日志


class RobotControl:
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        # Configs
        self.ready_to_start = False
        self.robot_dof = None
        self.sim_wave_motion = False
        self.enable_teleop = False
        self.state_bridge = False
        self.log_time, self.log_arm_state, self.log_arm_cmd, self.log_wrist_pose, self.log_hand = True, True, True, True, True
        self.log_buffer_debug = False
        self.safe_height_threshold = None
        
        self.robot_control_config()
        
        # Publisher and Subscriber
        self.publisher = self.create_publisher(LowCmd,"/arm_sdk",10) 
        self.publisher_left = self.create_publisher(SetMotorMulti, '/left_hand/set_motor_multi_126', 10)
        self.publisher_right = self.create_publisher(SetMotorMulti, '/right_hand/set_motor_multi_127', 10)

        self.arm_subscriber = self.create_subscription(LowState, "/lowstate", self.ArmStateHandler, 10)
        self.left_hand_subscriber = self.create_subscription(MotorStatus, "/left_hand/motor_status_126", self.LeftHandStateHandler, 10)
        self.right_hand_subscriber = self.create_subscription(MotorStatus, "/right_hand/motor_status_127", self.RightHandStateHandler, 10)

        # Teleop Subscriber
        if self.enable_teleop:
            self.left_calibr_sub = self.create_subscription(JointState, "/to/left_arm_calibr_mat", self.left_calibr_callback, 10)
            self.right_calibr_sub = self.create_subscription(JointState, "/to/right_arm_calibr_mat", self.right_calibr_callback, 10)

            self.leftarm_sub = self.create_subscription(JointState, "/to/left_arm_mat", self.leftarm_callback, 10)
            self.rightarm_sub = self.create_subscription(JointState, "/to/right_arm_mat", self.rightarm_callback, 10)

        if self.arm_state_bridge:
            self.publisher_arm_state_bridge = self.create_publisher(JointState, '/arm_state_bridge/lowstate', 10)

        self.robot_control_initialization()
        
        self.camera_initialization()

        # Timer Init
        self._timer = None
        self._cam_timer = None

        self.time_ = 0.
        self.control_dt_ = 0.01

    
    def robot_control_config(self):
        config_file = 'src/control_py/config/smach_config.yaml'
        with open(config_file, 'r') as f:
            smach_cfg = yaml.safe_load(f)
        
        param_map = {
            'log_time_screen': 'log_time',
            'log_armstate_screen': 'log_arm_state',
            'log_armcmd_screen': 'log_arm_cmd',
            'log_wristpose_screen': 'log_wrist_pose',
            'log_hand_screen': 'log_hand'
        }

        logs_cfg = smach_cfg['logs']
        for yaml_key, attr_name in param_map.items():
            if yaml_key in logs_cfg:
                setattr(self, attr_name, bool(logs_cfg[yaml_key]))

        robot_cfg = smach_cfg['robot']
        self.robot_dof = robot_cfg['robot_dof']
        self.safe_height_threshold = float(robot_cfg.get('safe_height_threshold', self.safe_height_threshold))
        self.camera_cfg = robot_cfg['cameras']

        teleop_cfg = smach_cfg['teleop']
        self.enable_teleop = teleop_cfg['enable']
        self.arm_state_bridge = teleop_cfg['arm_state_bridge']
        self.sim_wave_motion = teleop_cfg['sim_wave_motion']


    def camera_initialization(self):
        width = self.camera_cfg["width"]
        height = self.camera_cfg["height"]
        self.cam_fps = self.camera_cfg["fps"]

        # 获取 RealSense context，选设备
        ctx = rs.context()
        if len(ctx.devices) == 0:
            logger.error("没有检测到 RealSense 设备")
            return

        dev = ctx.devices[0]  # 自动选择第一台 RealSense
        serial = dev.get_info(rs.camera_info.serial_number)

        # 构造 CameraConfig 字典，device 用串号触发自动选择
        self.camera_config = {
            name: CameraConfig(
                device=serial,
                width=width,
                height=height,
                fps=self.cam_fps,
            )
            for name in self.camera_cfg.keys()
            if name not in ("width", "height", "fps")
        }

        self.camera_publishers = {}
        for name in self.camera_config.keys():
            if name == "cam_head":
                topic = "/camera/head/color/image_raw/compressed"
            elif name == "cam_chest":
                topic = "/camera/chest/color/image_raw/compressed"
            else:
                topic = f"/camera/{name}/color/image_raw/compressed"  # 通用 fallback

            self.camera_publishers[name] = self.create_publisher(CompressedImage, topic, qos_profile_sensor_data)

    
    def robot_control_initialization_new(self):
        self.hand_version = 2
        self.body_dof = 15
        self.arm_dof = 7 if self.robot_dof == 29 else 5
        self.hand_dof = self.hand_version + 9  # 手指自由度 (一代手:10个关节, 二代手:11个关节)    

        # 获取手指关节旋转范围(弧度)
        self.hand_joint_range = [[0, 1.51],[0, 1.04],[0, 1.46],[0, 1.46],[0, 1.46],[0, 1.46]]

        # 机械臂和手指初始关节角度
        init_arm_q = [0.] * (self.body_dof + self.arm_dof * 2)  # 机械臂初始关节角度
        init_arm_q_rad = [math.radians(a) for a in init_arm_q] # 机械臂初始关节弧度 
        init_hand_q = [0.] * self.hand_dof * 2 # 手指初始关节角度
        init_hand_q_rad = [math.radians(a) for a in init_hand_q] # 手指初始关节弧度

        # 保存当前关节弧度
        self.curr_arm_qrad_left = np.array(init_arm_q_rad[:self.arm_dof])
        self.curr_hand_qrad_left = np.array(init_arm_q_rad[self.arm_dof:])
        self.curr_arm_qrad_right = np.array(init_hand_q_rad[:self.hand_dof])
        self.curr_hand_qrad_right = np.array(init_hand_q_rad[self.hand_dof:])

        # 保存相对末端姿态
        self.relative_pose_left = [0, 0, 0, 0, 0, 0, 1]
        self.relative_pose_right = [0, 0, 0, 0, 0, 0, 1]

        # IK算法初始化      

        # logger.info("Initializing IK New...")
        # self.arm_ik_new = G1_29_ArmIK_new(Unit_Test = True, Visualization = False)
        # self.arm_new = G1_29_ArmController(simulation_mode=False)
        # self.arm_new = G1_29_ArmController(simulation_mode=False)

        # IK
        logger.info("Initializing IK NEW...")

        if self.robot_dof == 23:
            self.arm_new = G1_23_ArmController(simulation_mode=False)
            self.arm_ik = G1_23_ArmIK_new(Unit_Test = True, Visualization = False)
        else:
            self.arm_new = G1_29_ArmController(simulation_mode=False)
            self.arm_ik = G1_29_ArmIK_new(Unit_Test = True, Visualization = False)
            
        self.get_logger().info("IK initialization done.\n")

        # initial positon
        self.init_arm_pose_left_quat = [0.25, 0.15, 0.095, 0.7071, 0.7071, 0, 0]
        self.init_arm_pose_right_quat = [0.25, -0.15, 0.095, 0.7071, -0.7071, 0, 0]

        self.L_tf_target = pose_quat_to_se3(self.init_arm_pose_left_quat)
        self.R_tf_target = pose_quat_to_se3(self.init_arm_pose_right_quat)

        # # 测试谐波运动
        # if self.sim_wave_motion:
        #     self.rotation_speed = 0.005  # Rotation speed in radians per iteration
        #     self.step = 0
        self.arm_new.speed_gradual_max()


        # Teleop New
        # self.teleop_initialization()

        self.get_logger().info("New IK initialization done.\n")


    def teleop_initialization(self):
        self.position_scale = 0.8

        # 左手初始位姿 4x4 矩阵
        # [0.25, 0.15, 0.095, 0.7071, 0.7071, 0, 0] (wxyz)
        L_tf_mat = np.array([
            [1. , 0. , 0. , 0.25],
            [0. , 0. , -1. , 0.15],
            [0. , 1. , 0. , 0.095],
            [0. , 0. , 0. , 1.],
        ], dtype=np.float64)

        # 右手初始位姿 4x4 矩阵
        # [0.25, -0.15, 0.095, 0.7071, -0.7071, 0, 0] (wxyz)
        R_tf_mat = np.array([
            [1. , 0. , 0. , 0.25],
            [0. , 0. , 1. , -0.15],
            [0. , -1. , 0. , 0.095],
            [0. , 0. , 0. , 1.],
        ], dtype=np.float64)

        self.init_arm_pose_left_quat = mat_to_pos_quat(L_tf_mat, quat_order='wxyz')
        self.init_arm_pose_right_quat = mat_to_pos_quat(R_tf_mat, quat_order='wxyz')

        self.L_tf_target = pose_quat_to_se3(self.init_arm_pose_left_quat)
        self.R_tf_target = pose_quat_to_se3(self.init_arm_pose_right_quat)

        self._init_pos_left: Optional[np.ndarray] = L_tf_mat[:3, 3].copy()
        self._init_pos_right: Optional[np.ndarray] = R_tf_mat[:3, 3].copy()
        self._init_rot_left: Optional[np.ndarray] = L_tf_mat[:3, :3].copy()
        self._init_rot_right: Optional[np.ndarray] = R_tf_mat[:3, :3].copy()

        self.calib_pos_left: Optional[np.ndarray] = None
        self.calib_rot_left: Optional[np.ndarray] = None
        self.calib_pos_right: Optional[np.ndarray] = None
        self.calib_rot_right: Optional[np.ndarray] = None

        # # 遥操 target 位置
        # self.target_left = self.init_arm_pose_left_quat.copy()
        # self.target_right = self.init_arm_pose_right_quat.copy()

        # # 读取 Config 文件
        # tracker_config_file = 'src/control_py/control_py/state_manager/teleop/teleop_tracker_config.yaml'
        # cfg = yaml.safe_load(open(tracker_config_file))
        # key = cfg["adapt_selected"]
        # item = cfg["adapt_group"][key]
        
        # self.coord, self.R_index_left, self.R_index_right = item["coord"], item["left"], item["right"]


        # self.R_axes_left = [get_rot_mat(idx) for idx in self.R_index_left]
        # self.R_axes_right = [get_rot_mat(idx) for idx in self.R_index_right]


    def solve_ee_ik(self, ee_left, ee_right, quat_order="wxyz", euler_order="xyz"):
        """
        Solve IK using end-effector pose in quaternion format.
        Args:
            ee_left, ee_right can be: 
                - pin.SE3
                - quaternion pose: [x, y, z, qx, qy, qz, qw] or [x, y, z, qw, qx, qy, qz]
                - euler pose: [x, y, z, rx, ry, rz]
        Returns:
            sol_q : joint position
            sol_tauff : feedforward torque
        """

        # Convert pose (quat/euler) to SE3 if needed
        ee_left = pose_to_se3(ee_left, quat_order=quat_order, euler_order=euler_order)
        ee_right = pose_to_se3(ee_right, quat_order=quat_order, euler_order=euler_order)

        # Current joint position and velocity
        curr_dual_q = self.arm_new.get_current_dual_arm_q()
        curr_dual_dq = self.arm_new.get_current_dual_arm_dq()

        # Solve IK
        sol_q, sol_tauff = self.arm_ik.solve_ik(
            ee_left.homogeneous,
            ee_right.homogeneous,
            curr_dual_q,
            curr_dual_dq
        )

        return sol_q, sol_tauff

    def robot_control_initialization(self):
        if self.robot_dof not in (23, 29):
            raise ValueError(f"robot_dof must be 23 or 29, got {self.robot_dof}")
        self.arm_dof = 5 if self.robot_dof == 23 else 7

        # Unitree Arm
        # self.arm = Arm(self.robot_dof)
        self.hand = Hand()
        self.low_cmd = LowCmd()
        self.low_state = None 
        if self.robot_dof == 23:
            self.zero_arm_q = [0.4, 0.2, 0., 0.8, 0.,
                            0.4, -0.2, 0., 0.8, -0.]
            
            self.target_arm_q = [0., 0.3, 0., 1.5, 0.,
                                0., -0.3, 0., 1.5, 0.]
        else:
            self.zero_arm_q = [0.4, 0.2, 0., 0.8, 0., 0., 0.,
                            0.4, -0.2, 0., 0.8, 0., 0., 0.]
            
            self.target_arm_q = [0., 0.3, 0., 1.5, 0., 0., 0.,
                                0., -0.3, 0., 1.5, 0., 0., 0.]
        
        self.target_waist = 0.
        # self.arm_state_st = None    # 计算匀速运动的起始关节角(弧度)
        
        self.arm_pos_st_left, self.arm_pos_st_right = None, None  # 计算匀速运动的起始姿态(xyzabc)

        # BrainCo Hand
        self.hand_cmd_left = SetMotorMulti()
        self.hand_cmd_left.mode = 6
        self.hand_cmd_left.speeds = [990] * 6
        self.hand_cmd_right = SetMotorMulti()
        self.hand_cmd_right.mode = 6
        self.hand_cmd_right.speeds = [990] * 6

        self.zero_hand = [0., 0., 0., 0., 0., 0., 
                          0., 0., 0., 0., 0., 0.]
        
        self.target_hand = [0., 0., 0., 0., 0., 0.,
                            0., 0., 0., 0., 0., 0.]
        
        # 运控末端位姿（四元数）
        self.target_ee_quat_left, self.target_ee_quat_right = None, None

        # Buffer
        self.lowcmd_buffer = None               # Arm joints
        self.eecmd_buffer = ArmPoseData()       # Arm End-Effector pose cmd
        self.eecmd_fk_buffer = ArmPoseData()    # Arm End-Effector pose calculated by FK


    def LeftHandStateHandler(self, msg: MotorStatus):
        # logger.debug(f"left hand {msg.positions}")
        pass

    def RightHandStateHandler(self, msg: MotorStatus):
        # logger.debug(f"right hand {msg.positions}")
        pass
        

    def ArmStateHandler(self, msg: LowState):
        self.low_state = msg
        low_state_q = [msg.motor_state[joint].q for joint in self.arm_new.arm_joints]
        if self.log_arm_state:
            logger.info("Arm State: %s" % np.round(low_state_q, 3))

        # New
        if msg is not None:
            lowstate = msg
            for joint in self.arm_new.arm_joints:
                lowstate.motor_state[joint].q  = msg.motor_state[joint].q
                lowstate.motor_state[joint].dq = msg.motor_state[joint].dq
            self.arm_new.lowstate_buffer.SetData(lowstate)

        # Bridge
        if self.arm_state_bridge:
            bridge_msg = JointState()
            bridge_msg.header.stamp = self.get_clock().now().to_msg()
            left_arm_joints = ['left_' + name for name in self.arm_new.arm_joints_name]
            right_arm_joints = ['right_' + name for name in self.arm_new.arm_joints_name]
            bridge_msg.name = left_arm_joints + right_arm_joints
            bridge_msg.position = low_state_q.copy()
            self.publisher_arm_state_bridge.publish(bridge_msg)


    def left_calibr_callback(self, msg: JointState):
        pose_mat = jointstate_msg_to_pose_mat(msg)
        self.calib_pos_left, self.calib_rot_left = extract_mat(pose_mat)


    def right_calibr_callback(self, msg: JointState):
        pose_mat = jointstate_msg_to_pose_mat(msg)
        self.calib_pos_right, self.calib_rot_right = extract_mat(pose_mat)


    def leftarm_callback(self, msg: JointState):
        pose_mat = jointstate_msg_to_pose_mat(msg)
        pos_current, rot_current = extract_mat(pose_mat)

        pos_abs, quat_abs = process_pose(
            pos_current, 
            rot_current,
            self.calib_pos_left,
            self.calib_rot_left,
            self._init_pos_left,
            self._init_rot_left,
            self.R_axes_left,
            self.position_scale,
            coord=self.coord
        )

        self.target_left = pos_abs.tolist() + quat_abs.tolist()
        # logger.info(f"left {pos_abs} {quat_abs}")
        


    def rightarm_callback(self, msg: JointState):

        pose_mat = jointstate_msg_to_pose_mat(msg)
        pos_current, rot_current = extract_mat(pose_mat)

        pos_abs, quat_abs = process_pose(
            pos_current, 
            rot_current,
            self.calib_pos_right,
            self.calib_rot_right,
            self._init_pos_right,
            self._init_rot_right,
            self.R_axes_right,
            self.position_scale,
            coord=self.coord
        )

        self.target_right = pos_abs.tolist() + quat_abs.tolist()
        # logger.info(f"right {pos_abs} {quat_abs}")


    def publish_all(self):
        self.publisher.publish(self.low_cmd)
        self.publisher_left.publish(self.hand_cmd_left)
        self.publisher_right.publish(self.hand_cmd_right)

        if self.log_time:
            logger.info(f"time {round(self.time_, 2)}")
        if self.log_arm_cmd:
            low_cmd = [self.low_cmd.motor_cmd[joint].q for joint in self.arm_new.arm_joints]
            logger.info(f"arm_pub: {np.round(low_cmd, 3)}")
        if self.log_hand:
            logger.info(f"hand_pub: {list(self.hand_cmd_left.positions)} {list(self.hand_cmd_right.positions)}")
        # if self.log_wrist_pose:
        #     logger.info(f"wrist_pose: {np.round(self.arm.ik.left_wrist, 2)} {np.round(self.arm.ik.right_wrist, 2)}")

    def publish_only_arm(self):
        self.publisher.publish(self.low_cmd)

        if self.log_time:
            logger.info(f"time {round(self.time_, 2)}")
        if self.log_arm_cmd:
            low_cmd = [self.low_cmd.motor_cmd[joint].q for joint in self.arm_new.arm_joints]
            logger.info(f"arm_pub: {np.round(low_cmd, 3)}")
        # if self.log_wrist_pose:
        #     logger.info(f"wrist_pose: {np.round(self.arm.ik.left_wrist, 2)} {np.round(self.arm.ik.right_wrist, 2)}")

    
    def clear_timer(self):
        # 停用功能：停止定时器
        if self._timer is not None:
            self._timer.cancel()
            self._timer = None # 清除引用
            self.get_logger().info("Timer stopped.")
        if self._cam_timer is not None:
            self._cam_timer.cancel()
            self._cam_timer = None # 清除引用
            self.get_logger().info("Camera Timer stopped.")
        # 创建并启动定时器
        self.time_ = 0.


    # # 手臂控制 arm=left/right/both
    # def arm_control(self, ratio, arm):
    #     for i, joint in enumerate(self.arm_new.arm_joints):
    #         if arm == "left" and i<self.arm_dof or arm == "right" and i>=self.arm_dof or arm == "both":
    #             self.low_cmd.motor_cmd[joint].tau = self.arm_new.tauff_target[i]
    #             self.low_cmd.motor_cmd[joint].q = ratio * self.target_arm_q[i] + (1.0 - ratio) * self.lowcmd_buffer[i]   

    #     self.waist_control(1.)  # 腰部固定
    
    
    
    def arm_cmd_control(self, ratio, arm_q, arm_tauff=None, armside="both"):
        if arm_tauff is None:
            arm_tauff = np.zeros(2 * self.arm_dof)

        # Set control target values q & tau
        self.arm_new.ctrl_single_arm(arm_q, arm_tauff, armside=armside)

        # Limit joint velocity (限制超限的步长: 会导致无法到达目标位置)
        cliped_arm_q_target = self.arm_new.clip_single_arm_q_target(armside=armside, clip=False)

        # update motor joint cmd based on 'ratio'
        if self.robot_dof == 23:
            JointArmIndex = G1_23_JointArmIndex
        else:
            JointArmIndex = G1_29_JointArmIndex

        for idx, joint in enumerate(JointArmIndex):
            if armside == "left" and idx < self.arm_dof or armside == "right" and idx >= self.arm_dof or armside == "both":
                self.low_cmd.motor_cmd[joint].q = ratio * cliped_arm_q_target[idx] + (1.0 - ratio) * self.lowcmd_buffer[idx]
                self.low_cmd.motor_cmd[joint].tau = self.arm_new.tauff_target[idx]   


    def hand_cmd_control(self, hand_q, handside="both", scale_rad_to_1000=True):
        if handside != "right":
            self.hand_cmd_left.positions = (
                scale_to_0_1000(hand_q[:6], self.hand_joint_range)
                if scale_rad_to_1000 else hand_q[:6]
            )
        if handside != "left":
            self.hand_cmd_right.positions = (
                scale_to_0_1000(hand_q[6:], self.hand_joint_range)
                if scale_rad_to_1000 else hand_q[6:]
            )


    # def arm_control_split(self, ratio_l, ratio_r, arm):
    #     for i, joint in enumerate(self.arm_new.arm_joints):
    #         if arm == "left" and i<self.arm_dof or arm == "right" and i>=self.arm_dof or arm == "both":
    #             self.low_cmd.motor_cmd[joint].tau = self.arm_new.tauff_target[i]
    #             if i >= self.arm_dof:
    #                 self.low_cmd.motor_cmd[joint].q = ratio_r * self.target_arm_q[i] + (1.0 - ratio_r) * self.arm_state_st[i]
    #             else:
    #                 self.low_cmd.motor_cmd[joint].q = ratio_l * self.target_arm_q[i] + (1.0 - ratio_l) * self.arm_state_st[i]
    #     self.waist_control(1.)

    #     if ratio_l == 1. or ratio_r == 1.:
    #         low_cmd = [self.low_cmd.motor_cmd[joint].q for joint in self.arm_new.arm_joints]
    #         if ratio_l == 1.:
    #             self.arm_state_st[:self.arm_dof] =  low_cmd[:self.arm_dof]
    #         if ratio_r == 1.:
    #             self.arm_state_st[self.arm_dof:] =  low_cmd[self.arm_dof:]

        
    # # 固定腰部
    # def waist_control(self, ratio): 
    #     for i in [12, 13, 14]: # 13, 14 未解锁
    #         self.low_cmd.motor_csmd[i].tau = 0.
    #         self.low_cmd.motor_cmd[i].q = ratio * self.target_waist + (1.0 - ratio) * self.low_state.motor_state[i].q
    
    # # 灵巧手控制
    # def hand_control(self):
    #     self.hand_cmd_left.positions = scale_to_0_1000(self.target_hand[:6], self.hand_joint_range)
    #     self.hand_cmd_right.positions = scale_to_0_1000(self.target_hand[6:], self.hand_joint_range)

    def get_low_state(self):
        return [self.low_state.motor_state[joint].q for joint in self.arm_new.arm_joints]
    
    def get_low_cmd(self):
        return [self.low_cmd.motor_cmd[joint].q for joint in self.arm_new.arm_joints]

    
    # 为下个匀速运动记录起始关节角 为下个线性运动记录起始姿态
    def update_cmd_buffer(self, armside):
        """
        self.lowcmd_buffer: 关节弧度缓存
        self.eecmd_buffer: 末端位姿 cmd 缓存
        self.eecmd_fk_buffer: FK 计算的末端位姿缓存
        """

        if self.lowcmd_buffer is None:
            _lowcmd = self.get_low_state()
            self.lowcmd_buffer = self.get_low_state()
        else:
            _lowcmd = self.get_low_cmd()
        left_ee, right_ee = self.arm_ik.solve_fk(_lowcmd)

        if armside != "right":
            self.lowcmd_buffer[:7] = _lowcmd[:7]
            if self.target_ee_quat_left is None:
                self.eecmd_buffer.left.set_pose_mat(left_ee, quat_order='wxyz', euler_order='xyz')
            else:
                self.eecmd_buffer.left.set_pose_quat(self.target_ee_quat_left[:3], self.target_ee_quat_left[3:], 
                                                     quat_order='wxyz', euler_order='xyz')
            self.eecmd_fk_buffer.left.set_pose_mat(left_ee, quat_order='wxyz', euler_order='xyz')
            if self.log_buffer_debug:
                logger.debug(f"L_lowcmd_buffer: {np.round(self.lowcmd_buffer[:7], 3)}")
                logger.debug(f"L_eecmd_buffer: {self.eecmd_buffer.left}")
                logger.debug(f"L_eecmd_FK_buffer: {self.eecmd_fk_buffer.left}")
            
        if armside != "left":
            self.lowcmd_buffer[7:] = _lowcmd[7:]
            if self.target_ee_quat_right is None:
                self.eecmd_buffer.right.set_pose_mat(right_ee, quat_order='wxyz', euler_order='xyz')
            else:
                self.eecmd_buffer.right.set_pose_quat(self.target_ee_quat_right[:3], self.target_ee_quat_right[3:], 
                                                      quat_order='wxyz', euler_order='xyz')
            self.eecmd_fk_buffer.right.set_pose_mat(right_ee, quat_order='wxyz', euler_order='xyz')
            if self.log_buffer_debug:
                logger.debug(f"R_lowcmd_buffer: {np.round(self.lowcmd_buffer[7:], 3)}")
                logger.debug(f"R_eecmd_buffer: {self.eecmd_buffer.right}")
                logger.debug(f"R_eecmd_FK_buffer: {self.eecmd_fk_buffer.right}")
        
        
    
    
    # 臂手关节控制（Joint匀速运动）
    def arm_hand_joint_control(self, start, end, hand_q, arm_q, arm_tau=None, armside="both"):
        if arm_tau is None:
            arm_tau = np.zeros(2 * self.arm_dof)
        if start <= self.time_ < end:
            ratio = np.clip((self.time_ - start) / ((end - start - 0.05)), 0.0, 1.0)
            if self.lowcmd_buffer is None or ratio == 1.:
                self.update_cmd_buffer(armside)
            self.arm_cmd_control(ratio, arm_q, arm_tau, armside=armside)
            self.hand_cmd_control(hand_q, handside=armside, scale_rad_to_1000=True)

    def waist_joint_control(self, start, end, waist_q):
        if start <= self.time_ < end:
            ratio = np.clip((self.time_ - start) / ((end - start)), 0.0, 1.0)
            self.low_cmd.motor_cmd[12].tau = 0.
            self.low_cmd.motor_cmd[12].q = ratio * waist_q + (1.0 - ratio) * self.low_state.motor_state[12].q

    def waist_joint_fix(self, waist_q):
        self.low_cmd.motor_cmd[12].tau = 0.
        self.low_cmd.motor_cmd[12].q = waist_q
    
    # # 手臂灵巧手位置控制（匀速运动）
    # # 选择 arm 的情况下 update_arm_q, update_hand 只会赋值 arm 侧手臂和手
    # def arm_hand_control(self, st_s, end_s, arm, update_arm_q=None, update_hand=None):
    #     if st_s <= self.time_ < end_s:
    #         # 使用 update_arm_q (弧度), update_hand (弧度) 更新控制
    #         # 或使用 self.arm.ik.left_wrist, self.arm.ik.right_wrist, self.target_hand 更新控制

    #         if arm != 'left':
    #             self.target_arm_q[self.arm_dof:] = update_arm_q[self.arm_dof:].copy() if update_arm_q is not None else self.arm.ik.cal_ik()[0][self.arm_dof:]
    #             self.target_hand[6:] = update_hand[6:].copy() if update_hand is not None else self.target_hand[6:]
    #         if arm != 'right':
    #             self.target_arm_q[:self.arm_dof] = update_arm_q[:self.arm_dof].copy() if update_arm_q is not None else self.arm.ik.cal_ik()[0][:self.arm_dof]
    #             self.target_hand[:6] = update_hand[:6].copy() if update_hand is not None else self.target_hand[:6]

    #         ratio = np.clip((self.time_ - st_s) / ((end_s - st_s - 0.02)), 0.0, 1.0)
    #         if self.arm_state_st is None:
    #             self.store_curr_cmd(arm, trans_space=False)
    #         elif ratio == 1.:
    #             self.store_curr_cmd(arm, trans_space=False)

    #         self.arm_control(ratio, arm)
    #         self.hand_control()

    
        # self.arm_hand_control(st_s, end_s, arm, update_arm_q=self.zero_arm_q, update_hand=self.zero_hand)


    # # 两臂位置对应关系
    # def target_double_arm(self, arm, pos_arm_right, pos_hand):
    #     if arm != "left":
    #         multiplier = [1, 1, 1, 1, 1, 1]
    #         self.arm.ik.right_wrist = [m * pos for m, pos in zip(multiplier, pos_arm_right)]
    #         self.target_hand[6:] = pos_hand.copy()
    #     if arm != "right":
    #         multiplier = [-1, 1, 1, 1, -1, -1]
    #         self.arm.ik.left_wrist = [m * pos for m, pos in zip(multiplier, pos_arm_right)]
    #         self.target_hand[:6] = pos_hand.copy()  

    # def target_double_arm2(self, arm, pos_arm_left, pos_arm_right, pos_hand):
    #     if arm != "left":
    #         multiplier = [1, 1, 1, 1, 1, 1]
    #         self.arm.ik.right_wrist = [m * pos for m, pos in zip(multiplier, pos_arm_right)]
    #         self.target_hand[6:] = pos_hand.copy()
    #     if arm != "right":
    #         multiplier = [-1, 1, 1, 1, -1, -1]
    #         self.arm.ik.left_wrist = [m * pos for m, pos in zip(multiplier, pos_arm_left)]
    #         self.target_hand[:6] = pos_hand.copy()  

    # # 单臂，修改target
    # def target_diff_arm(self, arm, pos_arm, pos_hand):
    #     self.arm.ik.__dict__[arm + "_wrist"] = pos_arm.copy()
    #     if arm == "left":
    #         self.target_hand[:6] = pos_hand.copy() 
    #     elif arm == "right":
    #         self.target_hand[6:] = pos_hand.copy()

    
    
    

    def robot_cmd_init(self):
        self.all_motor_q = self.arm_new.get_current_motor_q()
        arm_indices = set(member.value for member in G1_29_JointArmIndex)
        for id in G1_29_JointIndex:
            self.low_cmd.motor_cmd[id].mode = 1
            self.low_cmd.motor_cmd[id].dq = 0.
            if id.value in arm_indices:
                if self.arm_new._Is_wrist_motor(id):
                    self.low_cmd.motor_cmd[id].kp = self.arm_new.kp_wrist
                    self.low_cmd.motor_cmd[id].kd = self.arm_new.kd_wrist
                else:
                    self.low_cmd.motor_cmd[id].kp = self.arm_new.kp_low
                    self.low_cmd.motor_cmd[id].kd = self.arm_new.kd_low
            else:
                if self.arm_new._Is_weak_motor(id):
                    self.low_cmd.motor_cmd[id].kp = self.arm_new.kp_low
                    self.low_cmd.motor_cmd[id].kd = self.arm_new.kd_low
                else:
                    self.low_cmd.motor_cmd[id].kp = self.arm_new.kp_high
                    self.low_cmd.motor_cmd[id].kd = self.arm_new.kd_high
            self.low_cmd.motor_cmd[id].q  = self.all_motor_q[id]
          
            
    def publish_camera_frames(self):
        for cam_name, cam in self.teleop_cameras.items():
            frame = cam.read()
            if frame is None:
                continue

            ok, encoded = cv2.imencode(".jpg", frame)
            if not ok:
                continue

            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = cam_name
            msg.format = "jpeg"
            msg.data = encoded.tobytes()

            self.camera_publishers[cam_name].publish(msg)
            

    def shutdown_cameras(self):
        logger_with_params("\nShutting down cameras...", level="INFO")

        for cam_name in ("infer_cameras", "teleop_cameras"):
            cam_dict = getattr(self, cam_name, None)
            if cam_dict is None:
                logger_with_params(f"[Camera] '{cam_name}' not found, skip shutdown", level="INFO")
                continue
            shutdown_dict(cam_dict, cam_name)

        gc.collect()
        time.sleep(0.5)

        logger_with_params("\nForce camera shutdown complete.", level="INFO")


    
    # 类抛物线手臂轨迹计算
    def generate_parabolic_traj(self, p1, p2, ratio_t, scale_a=80, limit_y=0.02):
        """
        生成三维空间中从起点到终点的抛物线轨迹上的单个插值点。
            p1 : list 起点姿态 [x, y, z, a, b, c]
            p2 : list 终点姿态 [x, y, z, a, b, c]
            ratio_t : float 插值比例 [0, 1]
            scale_a : float, 控制抛物线高度的缩放系数，值越大弧度越高
            limit_y : float, 抛物线在 y 方向的最大抬升高度限制
        返回: 当前 ratio_t 对应的三维坐标点 (x_t, y_t, z_t) 
        """
        ratio_t_smooth = 0.5 - 0.5 * np.cos(np.pi * ratio_t)

        _x0, _y0, _z0 = p1[:3]
        _x1, _y1, _z1 = p2[:3]
        _dist = (_z1 - _z0)**2 + (_x1 - _x0)**2 # 这里不必算根号
        x_t = _x1 * ratio_t_smooth + _x0 * (1-ratio_t_smooth)
        z_t = _z1 * ratio_t_smooth + _z0 * (1-ratio_t_smooth)
        dy_t = np.clip((-ratio_t_smooth**2 + ratio_t_smooth) * _dist * scale_a, 0., limit_y)
        # y_t = max(_y1, _y0) + dy_t
        y_t = _y0 * (1 - ratio_t_smooth) + _y1 * ratio_t_smooth + dy_t
        return x_t, y_t, z_t
    
    # 类抛物线手臂控制
    def arm_control_parabolic(self, st_s, end_s, pos_arm, arm, scale_a=80, limit_y=0.02):
        if st_s <= self.time_ < end_s:
            ratio_t = np.clip((self.time_ - st_s) / (end_s - st_s), 0.0, 1.0)
            p1 = self.__dict__["arm_pos_st_" + arm]
            x, y, z = self.generate_parabolic_traj(p1, pos_arm, ratio_t, scale_a=scale_a, limit_y=limit_y)
            pos_arm[:3] = [x, y, z].copy()

            logger.debug(f"{np.round(ratio_t, 2)} | {arm}: {np.round(pos_arm, 3)}")


            # self.arm.ik.__dict__[arm + "_wrist"] = pos_arm.copy()

            # if arm != 'left':
            #     self.target_arm_q[self.arm_dof:] = self.arm.ik.cal_ik()[0][self.arm_dof:]
            # if arm != 'right':
            #     self.target_arm_q[:self.arm_dof] = self.arm.ik.cal_ik()[0][:self.arm_dof]

            if self.arm_state_st is None:
                self.store_curr_cmd(arm, trans_space=False)
            elif ratio_t == 1.:
                self.store_curr_cmd(arm, trans_space=False)

            # self.arm_control(1., arm)
