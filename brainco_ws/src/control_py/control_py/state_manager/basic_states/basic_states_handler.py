from control_py.utils.utils import *

from control_py.state_manager.robot_control import *

from loguru import logger
from control_py.utils.loguru_settings import setup_loguru
setup_loguru(log_folder_path="log", show_on_terminal=True, terminal_level='DEBUG') # 设置日志

class BasicStatesHandler:
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        # ---------- 读取 config.yaml ----------
        CONFIG_PATH = "src/control_py/config/smach_config.yaml"
        _, _, _, states_config_path = load_mode_tasks(CONFIG_PATH)
        self.action_name = get_action_names(states_config_path)
        self._turn_action_future = None
        self._turn_action_result_logged = False
        self._turn_action_name = ""

    def _maybe_shutdown_vision_resources(self):
        if hasattr(self, "_shutdown_vision_resources"):
            self._shutdown_vision_resources()

    def _maybe_prepare_active_vision(self):
        has_vision_actions = False
        if hasattr(self, "_has_active_vision_actions"):
            has_vision_actions = self._has_active_vision_actions()

        if has_vision_actions and hasattr(self, "_get_shared_yolo_seg_model"):
            self._get_shared_yolo_seg_model()

    # 查询当前行为树里与指定 action 关联的并行组定义。
    def _get_active_parallel_group(self, action_idx=None):
        running_bt_def = None
        if hasattr(self, "_get_running_active_bt_def"):
            running_bt_def = self._get_running_active_bt_def()

        candidate_bt_defs = []
        if running_bt_def:
            candidate_bt_defs.append(running_bt_def)

        if action_idx is not None:
            action_idx = str(action_idx)
            inferred_bt_def = None
            if hasattr(self, "_resolve_active_bt_for_node"):
                inferred_bt_def = self._resolve_active_bt_for_node(action_idx)
            if inferred_bt_def is not None and inferred_bt_def not in candidate_bt_defs:
                candidate_bt_defs.append(inferred_bt_def)

        for bt_def in candidate_bt_defs:
            groups = bt_def.get("parallel_groups", [])
            if action_idx is None:
                if groups:
                    return groups[0]
                continue

            for group in groups:
                if action_idx in group.get("members", []):
                    return group

        return None

    # 判断指定 action 是否是并行组的入口节点。
    def _is_parallel_group_entry(self, action_idx: int):
        group = self._get_active_parallel_group(action_idx)
        return group is not None and group.get("entry") == str(action_idx)

    # 判断指定 action 是否是并行组的收口节点。
    def _is_parallel_group_finalizer(self, action_idx: int):
        group = self._get_active_parallel_group(action_idx)
        return group is not None and group.get("finalizer") == str(action_idx)

    # 读取动作的预计时长配置，供自动切换和等待逻辑共用。
    def _get_expected_duration(self, action_idx: int, fallback=None):
        if hasattr(self, "get_active_expected_duration"):
            return self.get_active_expected_duration(action_idx, fallback=fallback)
        return fallback

    # 开始记录当前 active 动作的完成状态与截止时长。
    def _track_active_action(self, action_idx: int, expected_duration=None):
        if hasattr(self, "start_active_action_tracking"):
            self.start_active_action_tracking(action_idx, expected_duration=expected_duration)

    # 启动一个底盘速度类动作，并建立统一的状态跟踪。
    def _start_turn_action(self, action_idx: int, velocity_cmd: str, fallback_duration: float):
        logger.info(f"Enter state {self.sm.get_state()} '{self.action_name[action_idx]}'")
        self.clear_timer()
        self._maybe_shutdown_vision_resources()
        self.update_cmd_buffer("both")

        self._turn_action_result_logged = False
        self._turn_action_future = None
        self._turn_action_name = f"Active{action_idx}"
        if hasattr(self, "call_highcmd_async"):
            self._turn_action_future = self.call_highcmd_async("set_velocity", velocity_cmd)
            logger.info(f"{self._turn_action_name} set_velocity='{velocity_cmd}'")
        else:
            logger.warning(f"{self._turn_action_name} high command client is not available.")

        self._timer = self.create_timer(0.01, self.timer_turn_command)
        self._track_active_action(
            action_idx,
            expected_duration=self._get_expected_duration(action_idx, fallback=fallback_duration)
        )
        self.get_logger().info("New timer created.")

    # 准备阶段
    def on_configure_handler(self):
        if self.ready_to_start:
            logger.info(f"Enter state -> {self.sm.get_state()}")
            self.clear_timer()
            self.update_cmd_buffer("both")
            self._maybe_shutdown_vision_resources()
            if self.param.extra == "safe_table":
                lr_height = [self.eecmd_fk_buffer.left.pos[2], self.eecmd_fk_buffer.right.pos[2]]
                self._timer = self.create_timer(0.01, lambda: self.timer_get_ready_table_safe(lr_height))
                self.param.extra = ""
            else:
                self._timer = self.create_timer(0.01, self.timer_get_ready)
        else:
            logger.info("Waiting ...")

    # 停止程序
    def on_shutdown_handler(self):
        logger.info(f"Shutting down from state {self.sm.get_state()}")
        self.clear_timer()
        self._maybe_shutdown_vision_resources()

    # Active 0 静止
    def on_active_handler(self):
        logger.info(f"Enter state -> active")
        self._maybe_prepare_active_vision()
    
    
    # Active 0 静止
    def on_active_0_handler(self):
        logger.info(f"Enter state -> {self.sm.get_state()} '{self.action_name[0]}'")
        self.clear_timer()
        self.update_cmd_buffer("both")
        # self.store_curr_cmd("both")
        self._maybe_shutdown_vision_resources()


    # Timer
    def timer_get_ready(self):
        self.time_ += self.control_dt_
        self.arm_hand_start(0, 2, update_arm_q=self.zero_arm_q, update_hand=self.zero_hand)
        self.publish_all()

    # Timer table safe
    def timer_get_ready_table_safe(self, lr_height):
        self.time_ += self.control_dt_
        self.arm_hand_start_table_safe(lr_height, handside="both")
        self.publish_all()

    
