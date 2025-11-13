class RobotLifecycleActions:

    action_name = {
        '0': 'Stop',
        '1': 'Hello',
        '2': 'Like',
        '3': 'Rock-Paper-Scissors',
        '4': 'Handshake',
    }

    action_num = len(action_name)

    # Active 1 打招呼
    def on_active_1_handler(self):
        self.get_logger().info(f"Enter state {self.sm.get_state()} 'Hello'")
        self.clear_timer()
        self.store_curr_cmd("both")
        self._timer = self.create_timer(0.01, self.timer_hello)
        self.get_logger().info(f"New timer created.")

    # Active 2 点赞
    def on_active_2_handler(self):
        self.get_logger().info(f"Enter state {self.sm.get_state()} 'Like'")
        self.clear_timer()
        self.store_curr_cmd("both")
        self._timer = self.create_timer(0.01, self.timer_like)
        self.get_logger().info(f"New timer created.")

    # Active 3 石头剪刀布
    def on_active_3_handler(self):
        self.get_logger().info(f"Enter state {self.sm.get_state()} 'Rock-Paper-Scissors'")
        self.clear_timer()
        self.store_curr_cmd("both")
        # 石头剪刀布参数
        self.curr_note_left = 0
        self.curr_note_right = 0
        self.gesture_list = ["石头","剪刀","布"]
        self.loop = 100
        self.interval_move = 0.5
        self.interval_stop = 4.
        self.prepare = 0.    
        self.end_ts = self.prepare + 1. + (self.interval_move * 6 + self.interval_stop) * self.loop - self.interval_move

        self._timer = self.create_timer(0.01, self.timer_rps)
        self.get_logger().info(f"New timer created.")

    # Active 4 握手
    def on_active_4_handler(self):
        self.get_logger().info(f"Enter state {self.sm.get_state()} 'Handshake'")
        self.clear_timer()
        self.store_curr_cmd("both")
        self._timer = self.create_timer(0.01, self.timer_hand_shake)
        self.get_logger().info(f"New timer created.")


    def timer_hello(self):
        self.time_ += self.control_dt_
        if self.param.handside != "left" and self.param.handside != "right":
            self.show_hello(0., 600, "both", speed=1.5)
        else:
            arm_side_opp = "left" if self.param.handside == "right" else "right"
            self.show_hello(0., 600, self.param.handside, speed=1.5)
            self.arm_back_zero(0., 2, arm_side_opp)
        self.publish_all()

    def timer_like(self):
        self.time_ += self.control_dt_
        if self.param.handside != "left" and self.param.handside != "right":
            self.show_like(0., 1, "both")
        else:
            arm_side_opp = "left" if self.param.handside == "right" else "right"
            self.show_like(0., 1, self.param.handside)
            self.arm_back_zero(0., 2, arm_side_opp)
        self.publish_all()

    def timer_hand_shake(self):
        self.time_ += self.control_dt_
        if self.param.handside != "left" and self.param.handside != "right":
            self.hand_shake(0., 2, 2, "both")
        else:
            arm_side_opp = "left" if self.param.handside == "right" else "right"
            self.hand_shake(0., 2, 2, self.param.handside)
            self.arm_back_zero(0., 2, arm_side_opp)
        self.publish_all()

    def timer_rps(self):
        self.time_ += self.control_dt_
        if self.param.handside != "left" and self.param.handside != "right":
            self.play_rps(0., 2, self.end_ts, self.interval_stop, self.interval_move, "both")
        else:
            arm_side_opp = "left" if self.param.handside == "right" else "right"
            self.play_rps(0., 2, self.end_ts, self.interval_stop, self.interval_move, self.param.handside)
            self.arm_back_zero(0., 2, arm_side_opp)
        self.publish_all()  