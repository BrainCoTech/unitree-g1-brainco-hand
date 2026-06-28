import numpy as np

from control_py.state_manager.basic_states import BasicStates


class SimpleTasks(BasicStates):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.ready_to_start = True

    # 挥手
    def show_hello(self, start, end, armside, speed=1.):
        if start <= self.time_ < end:
            duration = round(1/speed, 3)
            hand_q = [0., 0., 0., 0., 0., 0.] * 2

            target_q_1 = [-0.721,  0.727,  0.193, -0.099,  0.911,  0.,  0.656,
                          -0.721, -0.727, -0.193, -0.099, -0.911,  0., -0.656]
            target_q_2 = [-0.711,  1.029,  0.275,  0.42,   0.641, 0.,  0.873,
                          -0.711, -1.029, -0.275,  0.42,  -0.641, 0., -0.873]
            target_q_1 = self._arm_target_for_current_dof(target_q_1)
            target_q_2 = self._arm_target_for_current_dof(target_q_2)
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
            target_q = self._arm_target_for_current_dof(target_q)
            self.arm_hand_joint_control(start, end, hand_q, target_q, armside=armside)

    # 握手
    def hand_shake(self, start, end, pause=5., armside="right"):
        # 默认右手握手，不存在双手握手
        armside = "right" if armside != "left" else "left"
        target_q = [-0.926,  0.029,  0.152, 1.205, 0.2, 0., 0.,
                    -0.926, -0.029, -0.152, 1.205,  0.2, 0., 0.]
        target_q = self._arm_target_for_current_dof(target_q)
        if start <= self.time_ < end:
            hand_q = [0.5, 1.0, 0.1, 0.1, 0.1, 0.1] * 2
            self.arm_hand_joint_control(start, end, hand_q, target_q, armside=armside)
        if end <= self.time_ < end + pause:
            hand_q = [0.6, 1.0, 0.3, 0.3, 0.3, 0.3] * 2
            self.arm_hand_joint_control(end, end + pause, hand_q, target_q, armside=armside)
        if end <= self.time_ < end + pause + 1.:
            hand_q = [0.5, 1.0, 0.1, 0.1, 0.1, 0.1] * 2
            self.arm_hand_joint_control(end, end + pause + 1., hand_q, target_q, armside=armside)
        if end + pause + 1. <= self.time_ < end + pause + 3.:
            self.arm_back_zero(end + pause + 1., end + pause + 3., armside)

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
        target_q = self._arm_target_for_current_dof(target_q)

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
            
            
            

    def showheart(self, start, end, armside):
        target_q = [-0.66,   0.188,  0.038,  0.326,  0.45,  -1.43,   0.094,
                    -0.599, -0.186,  0.029,  0.098, -0.4 ,  -1.271, -0.062]
        target_q = self._arm_target_for_current_dof(target_q)
        hand_q = [0., 0.7, 0.6, 0.6, 0.6, 0.6] * 2
        if start <= self.time_ < end:
            self.arm_hand_joint_control(start, end, hand_q, target_q, armside=armside)


# Backward-compatible alias for dynamic loader expectations.
RobotTasks = SimpleTasks
