import sys
import os
import numpy as np

pkgs_dir = os.getcwd() + '/src/control_py/control_py/'
sys.path.append(pkgs_dir)

from arm_ik_control.robot_arm_ik import G1_23_ArmIK
from arm_ik_control.tv_wrapper import TeleVisionWrapper

from arm_ik_control.arm_23_joint import G1JointIndex


class ArmIK:

    def __init__(self, urdf_path, urdf_folder):

        self.arm_ik = G1_23_ArmIK(urdf_path, urdf_folder)
        
        self.head_pos = [1., 0., 0., 0., 
                         0., 1., 0., 0., 
                         0., 0., 1., 0., 
                         0., 0., 0., 1.]
        
        # self.left_wrist_pos = [1., 0., 0., 0., 
        #                        0., 1., 0., 0., 
        #                        0., 0., 1., 0., 
        #                        0., 0., 0., 1.]
        self.left_wrist = [0., 0., 0., 0., 0., 0.]

        # self.right_wrist_pos = [1., 0., 0., 0., 
        #                         0., 1., 0., 0., 
        #                         0., 0., 1., 0., 
        #                         0., 0., 0., 1.]
        self.right_wrist = [0., 0., 0., 0., 0., 0.]

        self.curr_lr_q = [0., 0., 0., 0., 0., 0., 0., 0., 0., 0.]
        self.curr_lr_dq = [0., 0., 0., 0., 0., 0., 0., 0., 0., 0.]



    
    # IK计算
    def cal_ik(self):

        self.left_wrist_pos = self.pos_convert(self.left_wrist)
        self.right_wrist_pos = self.pos_convert(self.right_wrist)
        
        head_mat = np.array(self.head_pos).reshape(4, 4, order="F")
        left_wrist_mat = np.array(self.left_wrist_pos).reshape(4, 4, order="F")
        right_wrist_mat = np.array(self.right_wrist_pos).reshape(4, 4, order="F")

        tv_wrapper = TeleVisionWrapper(head_mat, left_wrist_mat, right_wrist_mat)
        _, left_wrist, right_wrist = tv_wrapper.get_data()
        
        return self.arm_ik.solve_ik(left_wrist, right_wrist, np.array(self.curr_lr_q), np.array(self.curr_lr_dq))
    


    # Convert [x, y, z, c, b, a] to 4x4 Matrix
    def pos_convert(self, arm_pos_angle):
        # https://blog.csdn.net/shenmideshuaige/article/details/122269840
        # arm_pos_angle = [x, y, z, c, b, a]

        # Rz = [    cos(a)    -sin(a)   0.
        #           sin(a)    cos(a)    0. 
        #           0.          0.          1.          ]
        
        # Ry = [    cos(b)  0.          sin(b)
        #           0.          1.          0. 
        #           -sin(b) 0.          cos(b)  ]

        # Rx = [    1.          0.          0.
        #           0.          cos(c)   -sin(c) 
        #           0.          sin(c)   cos(c)   ]

        # Rz * Ry * Rx

        c = arm_pos_angle[3]*np.pi/180
        b = arm_pos_angle[4]*np.pi/180
        a = arm_pos_angle[5]*np.pi/180

        rot = np.array([[np.cos(a)*np.cos(b),     
            np.cos(a)*np.sin(b)*np.sin(c) - np.sin(a)*np.cos(c),
            np.cos(a)*np.sin(b)*np.cos(c) + np.sin(a)*np.sin(c),
            ],
            [np.sin(a)*np.cos(b),     
            np.sin(a)*np.sin(b)*np.sin(c) + np.cos(a)*np.cos(c),
            np.sin(a)*np.sin(b)*np.cos(c) - np.cos(a)*np.sin(c)],
            [-np.sin(b),
            np.cos(b)*np.sin(c),
            np.cos(b)*np.cos(c)],
            [0., 0., 0.]])

        rot_list = list(rot.T.flatten())

        return rot_list + arm_pos_angle[:3] + [1.]
    


class Arm:
    def __init__(self):


        self.kp = [80., 80., 80., 80., 80., 80., 90., 60., 60., 40.]
        self.kd = 1.5
        self.dq = 0.
        self.tau_ff = [-1.5, 1.7, 0.7, -0.5, 0., -1.5, -3.0, -0.9, -1.5, -1.7]



        self.ik = None

        self.arm_joints = [
          G1JointIndex.LeftShoulderPitch, 
          G1JointIndex.LeftShoulderRoll, 
          G1JointIndex.LeftShoulderYaw,
          G1JointIndex.LeftElbow, 
          G1JointIndex.LeftWristRoll,
          G1JointIndex.RightShoulderPitch, 
          G1JointIndex.RightShoulderRoll, 
          G1JointIndex.RightShoulderYaw,
          G1JointIndex.RightElbow, 
          G1JointIndex.RightWristRoll,
        ]

        self.arm_joints_name = ['shoulder_pitch_joint', 
                                'shoulder_roll_joint', 
                                'shoulder_yaw_joint',
                                'elbow_joint', 
                                'wrist_roll_joint']
        
        self.print_arm()
    
    def print_arm(self):    
        print("\nArm Joints:")
        for idx, name in enumerate(self.arm_joints_name):
            print('%s: left_%-30s %s: right_%-30s' %(idx, name, idx+5, name)) 

    
    def init_ik(self, urdf_path, urdf_folder):
        self.ik = ArmIK(urdf_path, urdf_folder)



    def motor_adapt(self, _target, _curr, _state):
        for i in range(len(_target)):
            if abs(_curr[i] - _state[i]) > 0.01:
                if _state[i] < _curr[i] <= _target[i] or _target[i] <= _curr[i] < _state[i]:
                    self.kp[i] += 1.
                elif _state[i] > _curr[i] and _target[i] >= _curr[i] or _state[i] < _curr[i] and _target[i] <= _curr[i]:
                    self.kp[i] -= 1.
                else:
                    print(0)
        self.kp = np.clip(self.kp, 50.0, 80.0)
        self.tau_ff = np.clip(self.tau_ff, -10.0, 10.0)
        

class Hand:
    def __init__(self):

        self.joint_names = ['thumb', 
                            'thumb_aux', 
                            'index', 
                            'middle', 
                            'ring', 
                            'pinky']
        
        self.print_hand()
        
    def print_hand(self):
        print("\nHand Joints:")
        for idx, name in enumerate(self.joint_names):
            print('%s: left_%-20s %s: right_%-20s' %(idx, name, idx+6, name))
        print("")