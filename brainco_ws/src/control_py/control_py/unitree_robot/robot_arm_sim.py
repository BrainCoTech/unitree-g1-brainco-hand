import numpy as np
import threading
import time
from enum import IntEnum

kTopicLowCommand_Debug  = "rt/lowcmd"
kTopicLowCommand_Motion = "rt/arm_sdk"
kTopicLowState = "rt/lowstate"

G1_29_Num_Motors = 35
G1_23_Num_Motors = 35
H1_2_Num_Motors = 35
H1_Num_Motors = 20

from loguru import logger
from control_py.utils.loguru_settings import setup_loguru, logger_with_params
setup_loguru(log_folder_path="log", show_on_terminal=True) # 设置日志 

class MotorState:
    def __init__(self):
        self.q = None
        self.dq = None

class G1_29_LowState:
    def __init__(self):
        self.motor_state = [MotorState() for _ in range(G1_29_Num_Motors)]

class G1_29_ArmController:
    def __init__(self, motion_mode = False, simulation_mode = False):
        logger.info("Initialize G1_29_ArmController...")
        self.q_target = np.zeros(14)
        self.tauff_target = np.zeros(14)
        self.motion_mode = motion_mode
        self.simulation_mode = simulation_mode
        self.kp_high = 300.0
        self.kd_high = 3.0
        self.kp_low = 80.0
        self.kd_low = 3.0
        self.kp_wrist = 40.0
        self.kd_wrist = 1.5

        self.all_motor_q = None
        self.arm_velocity_limit = 20.0
        self.control_dt = 1.0 / 250.0

        self._speed_gradual_max = False
        self._gradual_start_time = None
        self._gradual_time = None

        self.arm_q_target_sim = None
        self.arm_tauff_target_sim = None

        logger.info("Initialize G1_29_ArmController OK!")


    def clip_arm_q_target(self, target_q, velocity_limit, current_q):
        # current_q = self.get_current_dual_arm_q()
        delta = target_q - current_q
        motion_scale = np.max(np.abs(delta)) / (velocity_limit * self.control_dt)
        cliped_arm_q_target = current_q + delta / max(motion_scale, 1.0)
        return cliped_arm_q_target

    def _ctrl_motor_state(self, current_q):
        # if self.motion_mode:
        #     self.msg.motor_cmd[G1_29_JointIndex.kNotUsedJoint0].q = 1.0;

        # while True:
        start_time = time.time()

        # with self.ctrl_lock:
        arm_q_target     = self.q_target
        arm_tauff_target = self.tauff_target

        if self.simulation_mode:
            cliped_arm_q_target = arm_q_target
        else:
            cliped_arm_q_target = self.clip_arm_q_target(arm_q_target, 
                                                         velocity_limit = self.arm_velocity_limit, 
                                                         current_q=current_q)

        # for idx, id in enumerate(G1_29_JointArmIndex):
        #     self.msg.motor_cmd[id].q = cliped_arm_q_target[idx]
        #     self.msg.motor_cmd[id].dq = 0
        #     self.msg.motor_cmd[id].tau = arm_tauff_target[idx]   

        # self.msg.crc = self.crc.Crc(self.msg)
        # self.lowcmd_publisher.Write(self.msg)

        self.arm_q_target_sim = cliped_arm_q_target.copy()
        self.arm_tauff_target_sim = arm_tauff_target.copy()

        if self._speed_gradual_max is True:
            t_elapsed = start_time - self._gradual_start_time
            self.arm_velocity_limit = 20.0 + (10.0 * min(1.0, t_elapsed / 5.0))

        current_time = time.time()
        all_t_elapsed = current_time - start_time
        sleep_time = max(0, (self.control_dt - all_t_elapsed))
        time.sleep(sleep_time)

        # logger.debug(f"arm_velocity_limit:{self.arm_velocity_limit}")
        # logger.debug(f"sleep_time:{sleep_time}")


    def ctrl_dual_arm(self, q_target, tauff_target):
        '''Set control target values q & tau of the left and right arm motors.'''
        # with self.ctrl_lock:
        self.q_target = q_target
        self.tauff_target = tauff_target
    

    def speed_gradual_max(self, t = 5.0):
        '''Parameter t is the total time required for arms velocity to gradually increase to its maximum value, in seconds. The default is 5.0.'''
        self._gradual_start_time = time.time()
        self._gradual_time = t
        self._speed_gradual_max = True


class G1_29_JointArmIndex(IntEnum):
    # Left arm
    kLeftShoulderPitch = 15
    kLeftShoulderRoll = 16
    kLeftShoulderYaw = 17
    kLeftElbow = 18
    kLeftWristRoll = 19
    kLeftWristPitch = 20
    kLeftWristyaw = 21

    # Right arm
    kRightShoulderPitch = 22
    kRightShoulderRoll = 23
    kRightShoulderYaw = 24
    kRightElbow = 25
    kRightWristRoll = 26
    kRightWristPitch = 27
    kRightWristYaw = 28

class G1_29_JointIndex(IntEnum):
    # Left leg
    kLeftHipPitch = 0
    kLeftHipRoll = 1
    kLeftHipYaw = 2
    kLeftKnee = 3
    kLeftAnklePitch = 4
    kLeftAnkleRoll = 5

    # Right leg
    kRightHipPitch = 6
    kRightHipRoll = 7
    kRightHipYaw = 8
    kRightKnee = 9
    kRightAnklePitch = 10
    kRightAnkleRoll = 11

    kWaistYaw = 12
    kWaistRoll = 13
    kWaistPitch = 14

    # Left arm
    kLeftShoulderPitch = 15
    kLeftShoulderRoll = 16
    kLeftShoulderYaw = 17
    kLeftElbow = 18
    kLeftWristRoll = 19
    kLeftWristPitch = 20
    kLeftWristyaw = 21

    # Right arm
    kRightShoulderPitch = 22
    kRightShoulderRoll = 23
    kRightShoulderYaw = 24
    kRightElbow = 25
    kRightWristRoll = 26
    kRightWristPitch = 27
    kRightWristYaw = 28
    
    # not used
    kNotUsedJoint0 = 29
    kNotUsedJoint1 = 30
    kNotUsedJoint2 = 31
    kNotUsedJoint3 = 32
    kNotUsedJoint4 = 33
    kNotUsedJoint5 = 34