import sys, os
import numpy as np

pkgs_dir = pkgs_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(pkgs_dir)

from smach_pkg.robot_control import *

class RobotTasks(RobotControl):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        
        self.ready_to_start = True
        
        self.num_gestures = [
            [0.7, 0., 0.9, 0.9, 0.9, 0.9],  #0
            [0.7, 0.3, 0., 0.9, 0.9, 0.9],   #1
            [0.7, 0.3, 0., 0., 0.9, 0.9],    #2
            [0.7, 0.3, 0.9, 0., 0., 0.],     #3
            [0.7, 0.3, 0., 0., 0., 0.],      #4
            [0., 0., 0., 0., 0., 0.],       #5
            [0., 0., 0.9, 0.9, 0.9, 0.],    #6
            [0., 0.9, 0.5, 0.5, 0.9, 0.9],  #7
            [0., 0., 0., 0.9, 0.9, 0.9],    #8
            [0.7, 0.3, 0.5, 0.9, 0.9, 0.9],  #9
        ]

        self.rps_gesture = [[0.7, 0., 0.9, 0.9, 0.9, 0.9], # 石头
                            [0.7, 0., 0., 0., 0.9, 0.9],    # 剪刀
                            [0., 0., 0., 0., 0., 0.]]        # 布
    
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

    # 握手
    def hand_shake(self, start, end, wait, arm):
        if start <= self.time_ < end:
            pos_arm_right = [0.08, -0.35, -0.3, -45., 30., -100.]
            pos_hand = [0.3, 0.3, 0.1, 0.1, 0.1, 0.1]
            self.target_double_arm(arm, pos_arm_right, pos_hand)
            self.arm_hand_control(start, end, arm)
        if end + wait <= self.time_ < end + wait + 1.:
            pos_arm_right = [0.08, -0.35, -0.3, -45., 30., -100.]
            pos_hand = [0.3, 0.3, 0.2, 0.2, 0.2, 0.2]
            self.target_double_arm(arm, pos_arm_right, pos_hand)
            self.arm_hand_control(end + wait, end + wait + 1., arm)
        if end + wait + 2.5 <= self.time_ < end + wait + 4.:
            self.arm_back_zero(end + wait + 2.5, end + wait + 4., arm)


    # 石头剪刀布，抬手
    def play_rps_move(self, arm, show, direction):
        # show: 是否出拳
        # direction: "up" or "down" 手臂向上或向下
        up_pos_left = [-0.33, -0.2, -0.12, 25., 0., 80.]
        up_pos_right = [0.34, -0.2, -0.12, 25., 0., -80.]
        down_pos_left = [-0.33, -0.3, -0.15, 25., 0., 80.]
        down_pos_right = [0.34, -0.3, -0.15, 25., 0., -80.]
        if arm != "left":
            hand_gesture = self.rps_gesture[self.curr_note_right] if show else self.rps_gesture[0]
            self.target_diff_arm("right", eval(direction + "_pos_right"), hand_gesture)
        if arm != "right":
            hand_gesture = self.rps_gesture[self.curr_note_left] if show else self.rps_gesture[0]
            self.target_diff_arm("left", eval(direction + "_pos_left"), hand_gesture)

    # 随机石头剪刀布
    def play_rps_shuffle(self, arm, shuffle=True):
        if arm != "left":
            self.curr_note_right = np.random.randint(3) if shuffle else 0
        if arm != "right":
            self.curr_note_left = np.random.randint(3) if shuffle else 0

    # 石头剪刀布
    def play_rps(self, prepare, start, end, inter_stop, inter_m, arm):

        if prepare <= self.time_ < start:
            self.play_rps_move(arm, False, "up")
            self.arm_hand_control(prepare, start, arm)

        elif start <= self.time_ < end:
            t = (self.time_ - start) % (inter_stop + inter_m * 6)

            if 0 <= t < inter_m:
                self.play_rps_move(arm, False, "down")
                self.arm_hand_control(self.time_ - t, self.time_ - t + inter_m, arm)

            elif inter_m <= t < inter_m * 2:
                self.play_rps_move(arm, False, "up")
                self.arm_hand_control(self.time_ - t + inter_m, self.time_ - t + inter_m * 2, arm)
            
            elif inter_m * 2 <= t < inter_m * 3:
                self.play_rps_move(arm, False, "down")
                self.arm_hand_control(self.time_ - t + inter_m * 2, self.time_ - t + inter_m * 3, arm)
                self.play_rps_shuffle(arm)

            elif inter_m * 3 <= t < inter_m * 4:
                self.play_rps_move(arm, False, "up")
                self.arm_hand_control(self.time_ - t + inter_m * 3, self.time_ - t + inter_m * 4, arm)

            elif inter_m * 4 <= t < inter_m * 5:
                self.play_rps_move(arm, True, "down")
                self.arm_hand_control(self.time_ - t + inter_m * 4, self.time_ - t + inter_m * 5, arm)

            elif inter_m * 5 <= t < inter_m * 5 + inter_stop:
                self.play_rps_shuffle(arm, shuffle=False)

            elif inter_m * 5 + inter_stop <= t < inter_m * 6 + inter_stop:
                self.play_rps_move(arm, False, "up")
                self.arm_hand_control(self.time_ - t + inter_m * 5 + inter_stop, self.time_ - t + inter_m * 6 + inter_stop, arm)


    # 测试0-9
    def test_nums(self, start, end, arm):
        if start <= self.time_ < end:
            pos_arm_right = [0.23, -0.3, -0.25, 0., 0., -90.]
            t = (end - start - 2)/10
            if start <= self.time_ < start + 2:
                pos_hand = [0.7, 0., 0.9, 0.9, 0.9, 0.9]
                self.target_double_arm(arm, pos_arm_right, pos_hand)
                self.arm_hand_control(start, start + 2, arm)
            for i in range(10):
                if start + 2 + t * i <= self.time_ < start + 2 + t * i + t:
                    pos_hand = self.num_gestures[i]
                    self.target_double_arm(arm, pos_arm_right, pos_hand)
                    self.hand_control()
    