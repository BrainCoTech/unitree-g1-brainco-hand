from loguru import logger

from control_py.state_manager.basic_states import BasicStatesHandler
from control_py.utils.loguru_settings import setup_loguru

setup_loguru(log_folder_path="log", show_on_terminal=True, terminal_level="DEBUG")


class SimpleTasksHandler(BasicStatesHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def _normalize_handside(self):
        return self.param.handside if self.param.handside in ("left", "right") else "both"

    def _on_active_armdown_handler(self, action_idx: int):
        logger.info(f"Enter state {self.sm.get_state()} '{self.action_name[action_idx]}'")
        self.clear_timer()
        self.update_cmd_buffer("both")
        self._timer = self.create_timer(0.01, self.timer_back)
        self._track_active_action(
            action_idx,
            expected_duration=self._get_expected_duration(action_idx),
        )
        self.get_logger().info("New timer created.")

    def _on_active_hello_handler(self, action_idx: int, handside: str):
        logger.info(f"Enter state {self.sm.get_state()} '{self.action_name[action_idx]}'")
        self.clear_timer()
        self.update_cmd_buffer("both")
        self._timer = self.create_timer(0.01, lambda: self.timer_hello(handside=handside))
        self._track_active_action(
            action_idx,
            expected_duration=self._get_expected_duration(action_idx),
        )
        self.get_logger().info("New timer created.")

    def _on_active_like_handler(self, action_idx: int, handside: str):
        logger.info(f"Enter state {self.sm.get_state()} '{self.action_name[action_idx]}'")
        self.clear_timer()
        self.update_cmd_buffer("both")
        self._timer = self.create_timer(0.01, lambda: self.timer_like(handside=handside))
        self._track_active_action(
            action_idx,
            expected_duration=self._get_expected_duration(action_idx),
        )
        self.get_logger().info("New timer created.")

    def _on_active_rps_handler(self, action_idx: int, handside: str):
        logger.info(f"Enter state {self.sm.get_state()} '{self.action_name[action_idx]}'")
        self.clear_timer()
        self.update_cmd_buffer("both")

        self.curr_note_left = 0
        self.curr_note_right = 0
        self.gesture_list = ["石头", "剪刀", "布"]
        self.loop = 100
        self.interval_move = 0.5
        self.interval_stop = 2.0
        self.prepare = 0.0
        self.end_ts = (
            self.prepare + 1.0
            + (self.interval_move * 6 + self.interval_stop) * self.loop
            - self.interval_move
        )

        self._timer = self.create_timer(0.01, lambda: self.timer_rps(handside=handside))
        self._track_active_action(
            action_idx,
            expected_duration=self._get_expected_duration(action_idx, fallback=self.end_ts),
        )
        self.get_logger().info("New timer created.")

    def _on_active_handshake_handler(self, action_idx: int, handside: str):
        logger.info(f"Enter state {self.sm.get_state()} '{self.action_name[action_idx]}'")
        self.clear_timer()
        self.update_cmd_buffer("both")
        self._timer = self.create_timer(0.01, lambda: self.timer_hand_shake(handside=handside))
        self._track_active_action(
            action_idx,
            expected_duration=self._get_expected_duration(action_idx),
        )
        self.get_logger().info("New timer created.")

    # Active 1 打招呼
    def on_active_1_handler(self):
        self._on_active_hello_handler(1, self._normalize_handside())

    # Active 2 点赞
    def on_active_2_handler(self):
        self._on_active_like_handler(2, self._normalize_handside())

    # Active 3 石头剪刀布
    def on_active_3_handler(self):
        self._on_active_rps_handler(3, self._normalize_handside())

    # Active 4 握手
    def on_active_4_handler(self):
        self._on_active_handshake_handler(4, self._normalize_handside())

    # Active 5 比心
    def on_active_5_handler(self):
        logger.info(f"Enter state {self.sm.get_state()} '{self.action_name[5]}'")
        self.clear_timer()
        self.update_cmd_buffer("both")
        self._timer = self.create_timer(0.01, self.timer_showheart)
        self._track_active_action(
            5,
            expected_duration=self._get_expected_duration(5),
        )
        self.get_logger().info("New timer created.")

    # Active 6 手臂放下
    def on_active_6_handler(self):
        self._on_active_armdown_handler(6)

    # Active 7 ArmUP
    def on_active_7_handler(self):
        logger.info(f"Enter state {self.sm.get_state()} '{self.action_name[7]}'")
        self.clear_timer()
        self.update_cmd_buffer("both")
        self._timer = self.create_timer(0.01, self.timer_armup)
        self._track_active_action(
            7,
            expected_duration=self._get_expected_duration(7),
        )
        self.get_logger().info("New timer created.")

    # Active 8 HandClose
    def on_active_8_handler(self):
        logger.info(f"Enter state {self.sm.get_state()} '{self.action_name[8]}'")
        self.clear_timer()
        self.update_cmd_buffer("both")
        self._timer = self.create_timer(0.01, self.timer_handclose)
        self._track_active_action(
            8,
            expected_duration=self._get_expected_duration(8),
        )
        self.get_logger().info("New timer created.")

    # Active 9 HandOpen
    def on_active_9_handler(self):
        logger.info(f"Enter state {self.sm.get_state()} '{self.action_name[9]}'")
        self.clear_timer()
        self.update_cmd_buffer("both")
        self._timer = self.create_timer(0.01, self.timer_handopen)
        self._track_active_action(
            9,
            expected_duration=self._get_expected_duration(9),
        )
        self.get_logger().info("New timer created.")

    # Active 10 手臂放下
    def on_active_10_handler(self):
        self._on_active_armdown_handler(10)

    def timer_back(self):
        self.time_ += self.control_dt_
        self.arm_back_zero(0., 2, "both")
        self.publish_all()

    def timer_hello(self, handside: str):
        self.time_ += self.control_dt_
        if handside != "left" and handside != "right":
            self.show_hello(0., 600, "both", speed=1.5)
        else:
            arm_side_opp = "left" if handside == "right" else "right"
            self.show_hello(0., 600, handside, speed=1.5)
            self.arm_back_zero(0., 2, arm_side_opp)
        self.publish_all()

    def timer_like(self, handside: str):
        self.time_ += self.control_dt_
        if handside != "left" and handside != "right":
            self.show_like(0., 1, "both")
        else:
            arm_side_opp = "left" if handside == "right" else "right"
            self.show_like(0., 1, handside)
            self.arm_back_zero(0., 2, arm_side_opp)
        self.publish_all()

    def timer_hand_shake(self, handside: str):
        self.time_ += self.control_dt_
        if handside != "left" and handside != "right":
            self.hand_shake(0., 2, pause=3., armside="right")
        else:
            arm_side_opp = "left" if handside == "right" else "right"
            self.hand_shake(0., 2, pause=5., armside=handside)
            self.arm_back_zero(0., 2, arm_side_opp)
        self.publish_all()

    def timer_rps(self, handside: str):
        self.time_ += self.control_dt_
        if handside != "left" and handside != "right":
            self.play_rps(0., 2, self.end_ts, self.interval_stop, self.interval_move, "both")
        else:
            arm_side_opp = "left" if handside == "right" else "right"
            self.play_rps(0., 2, self.end_ts, self.interval_stop, self.interval_move, handside)
            self.arm_back_zero(0., 2, arm_side_opp)
        self.publish_all()

    def timer_showheart(self):
        self.time_ += self.control_dt_
        self.showheart(0., 1, "both")
        self.publish_all()

    def timer_armup(self):
        self.time_ += self.control_dt_
        self.showheart(0., 1, "both")
        self.publish_all()

    def timer_handclose(self):
        self.time_ += self.control_dt_
        self.showheart(0., 1, "both")
        self.publish_all()

    def timer_handopen(self):
        self.time_ += self.control_dt_
        self.showheart(0., 1, "both")
        self.publish_all()


# Backward-compatible alias for dynamic loader expectations.
RobotTasksHandler = SimpleTasksHandler
