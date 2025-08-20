import casadi                                                                   
import meshcat.geometry as mg
import numpy as np
import pinocchio as pin                             

from pinocchio import casadi as cpin    
from pinocchio.visualize import MeshcatVisualizer   
import os
import sys

parent2_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.append(parent2_dir)

from arm_ik_control.weighted_moving_filter import WeightedMovingFilter
        

        
class G1_23_ArmIK:
    def __init__(self, urdf_path, urdf_folder):
        np.set_printoptions(precision=5, suppress=True, linewidth=200)

        self.robot = pin.RobotWrapper.BuildFromURDF(urdf_path, urdf_folder)

        self.mixed_jointsToLockIDs = [
                                        "left_hip_pitch_joint" ,
                                        "left_hip_roll_joint" ,
                                        "left_hip_yaw_joint" ,
                                        "left_knee_joint" ,
                                        "left_ankle_pitch_joint" ,
                                        "left_ankle_roll_joint" ,
                                        "right_hip_pitch_joint" ,
                                        "right_hip_roll_joint" ,
                                        "right_hip_yaw_joint" ,
                                        "right_knee_joint" ,
                                        "right_ankle_pitch_joint" ,
                                        "right_ankle_roll_joint" ,
                                        "waist_yaw_joint" ,
                                    ]

        self.reduced_robot = self.robot.buildReducedRobot(
            list_of_joints_to_lock=self.mixed_jointsToLockIDs,
            reference_configuration=np.array([0.0] * self.robot.model.nq),
        )

        self.reduced_robot.model.addFrame(
            pin.Frame('L_ee',
                      self.reduced_robot.model.getJointId('left_wrist_roll_joint'),
                      pin.SE3(np.eye(3),
                              np.array([0.20,0,0]).T),
                      pin.FrameType.OP_FRAME)
        )
        
        self.reduced_robot.model.addFrame(
            pin.Frame('R_ee',
                      self.reduced_robot.model.getJointId('right_wrist_roll_joint'),
                      pin.SE3(np.eye(3),
                              np.array([0.20,0,0]).T),
                      pin.FrameType.OP_FRAME)
        )

        self.cmodel = cpin.Model(self.reduced_robot.model)
        self.cdata = self.cmodel.createData()
        

        # Creating symbolic variables
        self.cq = casadi.SX.sym("q", self.reduced_robot.model.nq, 1) 
        self.cq_l = casadi.SX.sym("q", int(self.reduced_robot.model.nq/2), 1) 
        self.cq_r = casadi.SX.sym("q", int(self.reduced_robot.model.nq/2), 1) 
        self.cTf_l = casadi.SX.sym("tf_l", 4, 4)
        self.cTf_r = casadi.SX.sym("tf_r", 4, 4)
        cpin.framesForwardKinematics(self.cmodel, self.cdata, self.cq)

        # Get the hand joint ID and define the error function
        self.L_hand_id = self.reduced_robot.model.getFrameId("L_ee")
        self.R_hand_id = self.reduced_robot.model.getFrameId("R_ee")


        self.translational_error_left = casadi.Function(
            "translational_error_left",
            [self.cq_l, self.cTf_l],
            [
                casadi.vertcat(
                    self.cdata.oMf[self.L_hand_id].translation - self.cTf_l[:3,3],
                )
            ],{"allow_free":True}
        )
        self.rotational_error_left = casadi.Function(
            "rotational_error_left",
            [self.cq_l, self.cTf_l],
            [
                casadi.vertcat(
                    cpin.log3(self.cdata.oMf[self.L_hand_id].rotation @ self.cTf_l[:3,:3].T),
                )
            ],{"allow_free":True}
        )

        self.translational_error_right = casadi.Function(
            "translational_error_right",
            [self.cq_r, self.cTf_r],
            [
                casadi.vertcat(
                    self.cdata.oMf[self.R_hand_id].translation - self.cTf_r[:3,3],
                )
            ],{"allow_free":True}
        )
        self.rotational_error_right = casadi.Function(
            "rotational_error_right",
            [self.cq_r, self.cTf_r],
            [
                casadi.vertcat(
                    cpin.log3(self.cdata.oMf[self.R_hand_id].rotation @ self.cTf_r[:3,:3].T),
                )
            ],{"allow_free":True}
        )




        # Defining the optimization problem left
        self.opti_left = casadi.Opti()
        self.var_q_left = self.opti_left.variable(int(self.reduced_robot.model.nq/2))
        self.var_q_last_left = self.opti_left.parameter(int(self.reduced_robot.model.nq/2))   # for smooth
        self.param_tf_left = self.opti_left.parameter(4, 4)
        self.translational_cost_left = casadi.sumsqr(self.translational_error_left(self.var_q_left, self.param_tf_left))
        self.rotation_cost_left = casadi.sumsqr(self.rotational_error_left(self.var_q_left, self.param_tf_left))
        self.regularization_cost_left = casadi.sumsqr(self.var_q_left)
        self.smooth_cost_left = casadi.sumsqr(self.var_q_left - self.var_q_last_left)


        # Defining the optimization problem right
        self.opti_right = casadi.Opti()
        self.var_q_right = self.opti_right.variable(int(self.reduced_robot.model.nq/2))
        self.var_q_last_right = self.opti_right.parameter(int(self.reduced_robot.model.nq/2))   # for smooth
        self.param_tf_right = self.opti_right.parameter(4, 4)
        self.translational_cost_right = casadi.sumsqr(self.translational_error_right(self.var_q_right, self.param_tf_right))
        self.rotation_cost_right = casadi.sumsqr(self.rotational_error_right(self.var_q_right, self.param_tf_right))
        self.regularization_cost_right = casadi.sumsqr(self.var_q_right)
        self.smooth_cost_right = casadi.sumsqr(self.var_q_right - self.var_q_last_right)


        # Setting optimization constraints and goals left
        self.opti_left.subject_to(self.opti_left.bounded(
            self.reduced_robot.model.lowerPositionLimit[:5],
            self.var_q_left,
            self.reduced_robot.model.upperPositionLimit[:5])
        )
        self.opti_left.minimize(50 * self.translational_cost_left + 0.5 * self.rotation_cost_left + 0.02 * self.regularization_cost_left + 0.1 * self.smooth_cost_left)

        # Setting optimization constraints and goals right
        self.opti_right.subject_to(self.opti_right.bounded(
            self.reduced_robot.model.lowerPositionLimit[5:],
            self.var_q_right,
            self.reduced_robot.model.upperPositionLimit[5:])
        )
        self.opti_right.minimize(50 * self.translational_cost_right + 0.5 * self.rotation_cost_right + 0.02 * self.regularization_cost_right + 0.1 * self.smooth_cost_right)

        opts = {
            'ipopt':{
                'print_level':0,
                'max_iter':50,
                'tol':1e-6
            },
            'print_time':False,# print or not
            'calc_lam_p':False # https://github.com/casadi/casadi/wiki/FAQ:-Why-am-I-getting-%22NaN-detected%22in-my-optimization%3F
        }
        self.opti_left.solver("ipopt", opts)
        self.opti_right.solver("ipopt", opts)

        self.init_data_left = np.zeros(int(self.reduced_robot.model.nq/2))
        self.init_data_right = np.zeros(int(self.reduced_robot.model.nq/2))
        self.smooth_filter_left = WeightedMovingFilter(np.array([0.4, 0.3, 0.2, 0.1]), 10)
        self.smooth_filter_right = WeightedMovingFilter(np.array([0.4, 0.3, 0.2, 0.1]), 10)
        self.vis = None

        
        # both arm
        
        self.translational_error = casadi.Function(
            "translational_error",
            [self.cq, self.cTf_l, self.cTf_r],
            [
                casadi.vertcat(
                    self.cdata.oMf[self.L_hand_id].translation - self.cTf_l[:3,3],
                    self.cdata.oMf[self.R_hand_id].translation - self.cTf_r[:3,3]
                )
            ],
        )
        self.rotational_error = casadi.Function(
            "rotational_error",
            [self.cq, self.cTf_l, self.cTf_r],
            [
                casadi.vertcat(
                    cpin.log3(self.cdata.oMf[self.L_hand_id].rotation @ self.cTf_l[:3,:3].T),
                    cpin.log3(self.cdata.oMf[self.R_hand_id].rotation @ self.cTf_r[:3,:3].T)
                )
            ],
        )

        # Defining the optimization problem
        self.opti = casadi.Opti()
        self.var_q = self.opti.variable(self.reduced_robot.model.nq)
        self.var_q_last = self.opti.parameter(self.reduced_robot.model.nq)   # for smooth
        self.param_tf_l = self.opti.parameter(4, 4)
        self.param_tf_r = self.opti.parameter(4, 4)
        self.translational_cost = casadi.sumsqr(self.translational_error(self.var_q, self.param_tf_l, self.param_tf_r))
        self.rotation_cost = casadi.sumsqr(self.rotational_error(self.var_q, self.param_tf_l, self.param_tf_r))
        self.regularization_cost = casadi.sumsqr(self.var_q)
        self.smooth_cost = casadi.sumsqr(self.var_q - self.var_q_last)

        # Setting optimization constraints and goals
        self.opti.subject_to(self.opti.bounded(
            self.reduced_robot.model.lowerPositionLimit,
            self.var_q,
            self.reduced_robot.model.upperPositionLimit)
        )
        self.opti.minimize(50 * self.translational_cost + 0.5 * self.rotation_cost + 0.02 * self.regularization_cost + 0.1 * self.smooth_cost)
    
        self.opti.solver("ipopt", opts)

        self.init_data = np.zeros(self.reduced_robot.model.nq)
        self.smooth_filter = WeightedMovingFilter(np.array([0.4, 0.3, 0.2, 0.1]), 10)
        self.vis = None
    
    
    
    
    # If the robot arm is not the same size as your arm :)
    def scale_arms(self, human_left_pose, human_right_pose, human_arm_length=0.60, robot_arm_length=0.75):
        scale_factor = robot_arm_length / human_arm_length
        robot_left_pose = human_left_pose.copy()
        robot_right_pose = human_right_pose.copy()
        robot_left_pose[:3, 3] *= scale_factor
        robot_right_pose[:3, 3] *= scale_factor
        return robot_left_pose, robot_right_pose

    
    
    def solve_ik_single(self, arm, arm_wrist, current_arm_motor_q = None, current_arm_motor_dq = None):
        print(arm, arm_wrist, current_arm_motor_q)
        if current_arm_motor_q is not None:
            self.__dict__["init_data_" + arm] = current_arm_motor_q
        self.__dict__["opti_" + arm].set_initial(self.__dict__["var_q_" + arm], self.__dict__["init_data_" + arm])

        self.__dict__["opti_" + arm].set_value(self.__dict__["param_tf_" + arm], arm_wrist)
        self.__dict__["opti_" + arm].set_value(self.__dict__["var_q_last_" + arm], self.__dict__["init_data_" + arm]) # for smooth

        print(self.__dict__["init_data_" + arm], arm_wrist)
        
        try:
            print(0)
            sol = self.__dict__["opti_" + arm].solve()
            print(1)
            sol_q = self.__dict__["opti_" + arm].value(self.__dict__["var_q_" + arm])
            sol_q = self.__dict__["smooth_filter_" + arm].add_data(sol_q)
            sol_q = self.__dict__["smooth_filter_" + arm].filtered_data
            print(2)
            if current_arm_motor_dq is not None:
                v = current_arm_motor_dq * 0.0
            else:
                v = (sol_q - self.__dict__["init_data_" + arm]) * 0.0
            print(3)
            self.__dict__["init_data_" + arm] = sol_q

            sol_tauff = pin.rnea(self.reduced_robot.model, self.reduced_robot.data, sol_q, v, np.zeros(self.reduced_robot.model.nv))

            return sol_q, sol_tauff
        
        except Exception as e:
            print(f"ERROR in convergence, plotting debug info.{e}")

            sol_q = self.__dict__["opti_" + arm].debug.value(self.__dict__["var_q_" + arm])
            self.__dict__["smooth_filter_" + arm].add_data(sol_q)
            sol_q = self.__dict__["smooth_filter_" + arm].filtered_data

            if current_arm_motor_dq is not None:
                v = current_arm_motor_dq * 0.0
            else:
                v = (sol_q - self.__dict__["init_data_" + arm]) * 0.0

            self.__dict__["init_data_" + arm] = sol_q

            sol_tauff = pin.rnea(self.reduced_robot.model, self.reduced_robot.data, sol_q, v, np.zeros(self.reduced_robot.model.nv))

            print(f"sol_q:{sol_q} \nmotorstate: \n{current_arm_motor_q} \nleft_pose: \n{arm_wrist}")

            # return sol_q, sol_tauff
            return current_arm_motor_q, np.zeros(self.reduced_robot.model.nv)



    def solve_ik_single_arm(self, target_wrist, side='left', current_arm_q=None, current_arm_dq=None):
        if current_arm_q is not None:
            self.init_data = current_arm_q
        self.opti.set_initial(self.var_q, self.init_data)

        # 可视化目标末端位姿
        if self.Visualization:
            if side == 'left':
                self.vis.viewer['L_ee_target'].set_transform(target_wrist)
            else:
                self.vis.viewer['R_ee_target'].set_transform(target_wrist)

        # 设置目标位姿参数
        if side == 'left':
            self.opti.set_value(self.param_tf_l, target_wrist)
        else:
            self.opti.set_value(self.param_tf_r, target_wrist)

        self.opti.set_value(self.var_q_last, self.init_data)  # 用于平滑约束

        try:
            sol = self.opti.solve()
            sol_q = self.opti.value(self.var_q)
            self.smooth_filter.add_data(sol_q)
            sol_q = self.smooth_filter.filtered_data

            if current_arm_dq is not None:
                v = current_arm_dq * 0.0
            else:
                v = (sol_q - self.init_data) * 0.0

            self.init_data = sol_q

            sol_tauff = pin.rnea(
                self.reduced_robot.model,
                self.reduced_robot.data,
                sol_q, v,
                np.zeros(self.reduced_robot.model.nv)
            )

            if self.Visualization:
                self.vis.display(sol_q)

            return sol_q, sol_tauff

        except Exception as e:
            print(f"[IK ERROR] Optimization failed: {e}")

            sol_q = self.opti.debug.value(self.var_q)
            self.smooth_filter.add_data(sol_q)
            sol_q = self.smooth_filter.filtered_data

            if current_arm_dq is not None:
                v = current_arm_dq * 0.0
            else:
                v = (sol_q - self.init_data) * 0.0

            self.init_data = sol_q

            sol_tauff = pin.rnea(
                self.reduced_robot.model,
                self.reduced_robot.data,
                sol_q, v,
                np.zeros(self.reduced_robot.model.nv)
            )

            print(f"[DEBUG INFO] sol_q: {sol_q}\nTarget Pose: {target_wrist}")
            if self.Visualization:
                self.vis.display(sol_q)

            return current_arm_q, np.zeros(self.reduced_robot.model.nv)




    def solve_ik(self, left_wrist, right_wrist, current_lr_arm_motor_q = None, current_lr_arm_motor_dq = None):
        if current_lr_arm_motor_q is not None:
            self.init_data = current_lr_arm_motor_q
        self.opti.set_initial(self.var_q, self.init_data)

        # left_wrist, right_wrist = self.scale_arms(left_wrist, right_wrist)

        self.opti.set_value(self.param_tf_l, left_wrist)
        self.opti.set_value(self.param_tf_r, right_wrist)
        self.opti.set_value(self.var_q_last, self.init_data) # for smooth

        try:
            sol = self.opti.solve()
            # sol = self.opti.solve_limited()

            sol_q = self.opti.value(self.var_q)
            self.smooth_filter.add_data(sol_q)
            sol_q = self.smooth_filter.filtered_data

            if current_lr_arm_motor_dq is not None:
                v = current_lr_arm_motor_dq * 0.0
            else:
                v = (sol_q - self.init_data) * 0.0

            self.init_data = sol_q

            sol_tauff = pin.rnea(self.reduced_robot.model, self.reduced_robot.data, sol_q, v, np.zeros(self.reduced_robot.model.nv))


            return sol_q, sol_tauff
        
        except Exception as e:
            print(f"ERROR in convergence, plotting debug info.{e}")

            sol_q = self.opti.debug.value(self.var_q)
            self.smooth_filter.add_data(sol_q)
            sol_q = self.smooth_filter.filtered_data

            if current_lr_arm_motor_dq is not None:
                v = current_lr_arm_motor_dq * 0.0
            else:
                v = (sol_q - self.init_data) * 0.0

            self.init_data = sol_q

            sol_tauff = pin.rnea(self.reduced_robot.model, self.reduced_robot.data, sol_q, v, np.zeros(self.reduced_robot.model.nv))

            print(f"sol_q:{sol_q} \nmotorstate: \n{current_lr_arm_motor_q} \nleft_pose: \n{left_wrist} \nright_pose: \n{right_wrist}")

            # return sol_q, sol_tauff
            return current_lr_arm_motor_q, np.zeros(self.reduced_robot.model.nv)



