import traceback, yaml, importlib
import numpy as np
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger  # 用标准的触发服务，返回string message
import time

from sm_interfaces.srv import SmachCmd, HighCmd
from sm_interfaces.msg import SmachParam, HighCmdParam
from control_py.state_manager.state_machine import LifecycleStateMachine

from loguru import logger
from control_py.utils.loguru_settings import setup_loguru, logger_with_params
from control_py.utils.utils import load_mode_tasks
setup_loguru(log_folder_path="log", show_on_terminal=True, terminal_level='DEBUG') # 设置日志

# ---------- 读取 config.yaml ----------
CONFIG_PATH = "src/control_py/config/smach_config.yaml"
mode, Tasks, TasksHandler, STATES_CONFIG_PATH = load_mode_tasks(CONFIG_PATH)
with open(CONFIG_PATH, "r", encoding="utf-8") as f:
    SMACH_CFG = yaml.safe_load(f)
SWITCH_AFTER_DONE = bool(SMACH_CFG.get("mode", {}).get("switch_after_done", False))
TRIGGER_TEST = bool(SMACH_CFG.get("mode", {}).get("trigger_test", True))
logger_with_params("Mode: {}", mode, level="INFO")


def _parse_active_bt_children(raw_children):
    children = []
    parallel_groups = []

    for item in raw_children:
        group_children = _flatten_active_bt_item(item)
        if len(group_children) <= 1:
            if group_children:
                children.append(group_children[0])
            continue

        if not group_children:
            continue

        children.extend(group_children)
        if len(group_children) >= 2:
            parallel_groups.append({
                "entry": group_children[0],
                "finalizer": group_children[-1],
                "members": group_children,
            })

    return children, parallel_groups


def _flatten_active_bt_item(item):
    if item is None:
        return []

    if isinstance(item, (list, tuple)):
        flattened = []
        for child in item:
            flattened.extend(_flatten_active_bt_item(child))
        return flattened

    if isinstance(item, dict):
        for group_key in ("parallel", "group"):
            if group_key in item:
                return _flatten_active_bt_item(item[group_key])

    return [str(item)]


def _normalize_active_bt_step(item, prefer="entry"):
    flattened = _flatten_active_bt_item(item)
    if not flattened:
        return None
    if prefer == "finalizer":
        return flattened[-1]
    return flattened[0]


class LifecyclePublisher(Tasks, TasksHandler, Node):
    def __init__(self, name):
        super().__init__(name)

        self.switch_after_done = SWITCH_AFTER_DONE
        self.trigger_test = TRIGGER_TEST
        self._tracked_active_state = None
        self._tracked_active_done = True
        self._tracked_active_deadline = None
        self._active_bt_defs = {}
        self._active_bt_root_map = {}
        self._active_bt_internal_nodes = set()
        self._active_bt_node_to_tree = {}
        self._running_active_bt = None
        self._active_bt_transitioning = False

        self.robot_control_initialization_new()
        
        
        # 读取 YAML
        with open(STATES_CONFIG_PATH, 'r', encoding='utf-8') as f:
            cfg = yaml.safe_load(f)

        callbacks = {}

        # main_trans 中 before/after 自动生成 callback
        for trans_name, trans_def in cfg.get('main_trans', {}).items():
            for hook in ['before', 'after']:
                cb_name = trans_def.get(hook)
                if cb_name and cb_name not in callbacks:
                    # 默认生成一个 handler，如果你已经有类方法可以替换
                    callbacks[cb_name] = getattr(self, f"{cb_name}_handler", lambda: print(f"[Callback] {cb_name} triggered"))

        # active 子状态自动生成 on_active_X_handler
        active_children = {}
        for state in cfg['structure']:
            if isinstance(state, dict) and 'active' in state:
                active_children = state['active'].get('children', {})
                break
        self.action_name = active_children
        self.action_num = len(active_children)
        raw_expected_duration = cfg.get('active_expected_duration', {})
        self.active_expected_duration = {
            str(k): v for k, v in raw_expected_duration.items()
        }
        self._load_active_behavior_trees(cfg)

        for k in active_children.keys():
            cb_name = f'on_active_{k}'
            if cb_name not in callbacks:
                callbacks[cb_name] = getattr(self, f"{cb_name}_handler", lambda k=k: print(f"[Callback] {cb_name} triggered"))

        # 然后直接初始化状态机
        self.sm = LifecycleStateMachine(
            STATES_CONFIG_PATH,
            on_callbacks=callbacks,
            switch_after_done=self.switch_after_done,
            trigger_test=self.trigger_test,
            can_switch_active_cb=self.can_switch_active_transition,
        )

        # Service server
        self.srv_trans_cmd = self.create_service(SmachCmd, 'lifecycle_command', self.handle_command)
        self.srv_available_trans = self.create_service(Trigger, 'get_available_transitions', self.get_transitions_callback)
        self.srv_current_state = self.create_service(Trigger, 'get_current_state', self.get_state_callback)
        self.highcmd_client = self.create_client(HighCmd, 'g1_high_cmd')
        self._active_bt_timer = self.create_timer(0.05, self._tick_active_behavior_tree)

        self.param = SmachParam()

        logger.info("Request 'configure' to start\n")

    def call_highcmd_async(self, cmd, value=""):
        if not self.highcmd_client.wait_for_service(timeout_sec=0.1):
            logger.warning("Service 'g1_high_cmd' is not available.")
            return None

        req = HighCmd.Request()
        req.data = "cmd"
        req.param = HighCmdParam()
        req.param.cmd = cmd
        req.param.value = value
        return self.highcmd_client.call_async(req)

    def start_active_action_tracking(self, action_idx, expected_duration=None):
        action_idx = str(action_idx)
        self._tracked_active_state = action_idx
        self._tracked_active_done = (action_idx == "0")
        self._tracked_active_deadline = expected_duration

    def get_active_expected_duration(self, action_idx, fallback=None):
        action_idx = str(action_idx)
        value = self.active_expected_duration.get(action_idx, fallback)

        if value is None:
            return fallback
        if isinstance(value, str):
            lower_value = value.strip().lower()
            if lower_value == "auto":
                return fallback
            try:
                return float(lower_value)
            except ValueError:
                logger.warning(
                    f"Invalid active_expected_duration for action '{action_idx}': {value}. "
                    f"Fallback to {fallback}."
                )
                return fallback

        try:
            return float(value)
        except (TypeError, ValueError):
            logger.warning(
                f"Invalid active_expected_duration for action '{action_idx}': {value}. "
                f"Fallback to {fallback}."
            )
            return fallback

    def mark_active_action_done(self):
        self._tracked_active_done = True

    def _load_active_behavior_trees(self, cfg):
        active_bt_cfg = cfg.get("behavior_trees", {}).get("active", {})

        for tree_name, tree_def in active_bt_cfg.items():
            tree_type = str(tree_def.get("type", "")).strip().lower()
            if tree_type != "sequence":
                logger.warning(
                    f"Unsupported active behavior tree type '{tree_type}' for '{tree_name}', skip."
                )
                continue

            children, parallel_groups = _parse_active_bt_children(tree_def.get("children", []))
            if len(children) < 2:
                logger.warning(
                    f"Active behavior tree '{tree_name}' must contain at least 2 children, skip."
                )
                continue

            if children[0] in self._active_bt_root_map:
                logger.warning(
                    f"Active behavior tree '{tree_name}' reuses root node '{children[0]}', skip."
                )
                continue

            self._active_bt_defs[tree_name] = {
                "name": tree_name,
                "type": tree_type,
                "children": children,
                "root": children[0],
                "parallel_groups": parallel_groups,
                "next_map": {
                    children[idx]: children[idx + 1]
                    for idx in range(len(children) - 1)
                },
            }
            self._active_bt_root_map[children[0]] = tree_name
            self._active_bt_internal_nodes.update(children[1:])
            for node in children:
                self._active_bt_node_to_tree.setdefault(node, set()).add(tree_name)

    def _get_active_bt_names_by_node(self, node_name):
        if node_name is None:
            return set()
        return set(self._active_bt_node_to_tree.get(str(node_name), set()))

    def _resolve_active_bt_for_node(self, node_name, preferred_tree=None):
        candidate_tree_names = self._get_active_bt_names_by_node(node_name)
        if not candidate_tree_names:
            return None

        if preferred_tree in candidate_tree_names:
            return self._active_bt_defs.get(preferred_tree)

        running_bt = self._get_running_active_bt_def()
        if running_bt is not None and running_bt["name"] in candidate_tree_names:
            return running_bt

        for tree_name in sorted(candidate_tree_names):
            return self._active_bt_defs.get(tree_name)

        return None

    def _extract_active_target_from_event(self, event_name):
        if not isinstance(event_name, str) or not event_name.startswith("start_"):
            return None
        return event_name.split("start_", 1)[1]

    def _get_running_active_bt_def(self):
        if self._running_active_bt is None:
            return None
        return self._active_bt_defs.get(self._running_active_bt)

    def _set_running_active_bt(self, tree_name):
        if tree_name == self._running_active_bt:
            return
        self._running_active_bt = tree_name
        if tree_name is not None:
            logger.info(f"Active behavior tree started: {tree_name}")

    def _clear_running_active_bt(self):
        if self._running_active_bt is not None:
            logger.info(f"Active behavior tree cleared: {self._running_active_bt}")
        self._running_active_bt = None

    def _sync_active_behavior_tree(self):
        current_substate = self._get_active_substate(self.sm.get_state())
        if current_substate is None or current_substate == "0":
            self._clear_running_active_bt()
            return

        running_bt = self._get_running_active_bt_def()
        if running_bt is not None and current_substate in running_bt["children"]:
            return

        if current_substate in self._active_bt_root_map:
            self._set_running_active_bt(self._active_bt_root_map[current_substate])
            return

        self._clear_running_active_bt()

    def _is_active_action_done(self, current_substate):
        if current_substate is None or current_substate == "0":
            return True, ""

        if self._tracked_active_state is None:
            return False, (
                f"Previous action state is '{current_substate}', but no tracking info is available."
            )
        if self._tracked_active_state != current_substate:
            return False, (
                f"Previous action tracking mismatch: tracked='{self._tracked_active_state}', "
                f"current='{current_substate}'."
            )
        if self._tracked_active_done:
            return True, ""

        if self._tracked_active_deadline is None:
            return False, (
                ""
                # f"Previous action '{current_substate}' has no completion deadline yet."
            )

        if self.time_ >= self._tracked_active_deadline:
            self.mark_active_action_done()
            return True, ""

        return False, (
            f"Current action '{current_substate}' is still running. "
            f"Wait until it is done before switching."
        )

    def _validate_active_bt_transition(self, current_substate, event_name):
        target_substate = self._extract_active_target_from_event(event_name)
        if target_substate is None:
            return True, ""

        if target_substate in self._active_bt_internal_nodes:
            running_bt = self._get_running_active_bt_def()
            target_bt = self._resolve_active_bt_for_node(
                target_substate,
                preferred_tree=running_bt["name"] if running_bt is not None else None,
            )
            if target_bt is None:
                return False, ""

            if running_bt is None or running_bt["name"] != target_bt["name"]:
                root_state = target_bt["root"]
                return False, (
                    ""
                    # f"Action '{target_substate}' is managed by behavior tree '{tree_name}'. "
                    # f"Start root action '{root_state}' instead."
                )

            expected_target = _normalize_active_bt_step(
                running_bt["next_map"].get(current_substate),
                prefer="entry",
            )
            if expected_target != target_substate:
                return False, (
                    f"Behavior tree '{running_bt['name']}' expects next action '{expected_target}', "
                    f"not '{target_substate}'."
                )

        if current_substate == "0":
            return True, ""

        running_bt = self._get_running_active_bt_def()
        if running_bt is None or current_substate not in running_bt["children"]:
            return True, ""

        expected_target = _normalize_active_bt_step(
            running_bt["next_map"].get(current_substate),
            prefer="entry",
        )
        if expected_target is None:
            return True, ""
        if target_substate in {expected_target, "0"}:
            return True, ""

        return False, (
            f"Behavior tree '{running_bt['name']}' is running. "
            f"Action '{current_substate}' must transition to '{expected_target}' next."
        )

    def _get_active_substate(self, state_name):
        if not isinstance(state_name, str):
            return None
        if state_name.startswith("active."):
            return state_name.split(".", 1)[1]
        if state_name.startswith("active_"):
            return state_name.split("_", 1)[1]
        return None

    def can_switch_active_transition(self, current_state, event_name):
        current_substate = self._get_active_substate(current_state)
        if current_substate is None:
            return True, ""

        if self.switch_after_done:
            is_done, done_message = self._is_active_action_done(current_substate)
            if not is_done:
                # logger.warning(f"{done_message} Block '{event_name}' conservatively.")
                return False, done_message

        is_valid, bt_message = self._validate_active_bt_transition(current_substate, event_name)
        if not is_valid:
            # logger.warning(f"{bt_message} Block '{event_name}'.")
            return False, bt_message

        return True, ""

    def _tick_active_behavior_tree(self):
        if self._active_bt_transitioning:
            return

        running_bt = self._get_running_active_bt_def()
        if running_bt is None:
            return

        current_substate = self._get_active_substate(self.sm.get_state())
        if current_substate not in running_bt["children"]:
            self._clear_running_active_bt()
            return

        is_done, _ = self._is_active_action_done(current_substate)
        if not is_done:
            return

        next_substate = _normalize_active_bt_step(
            running_bt["next_map"].get(current_substate),
            prefer="entry",
        )
        if next_substate is None:
            logger.info(
                f"Active behavior tree '{running_bt['name']}' completed at action '{current_substate}'."
            )
            self._clear_running_active_bt()
            return

        event_name = f"start_{next_substate}"
        param = self.sm.get_substate_param()
        if not hasattr(param, "handside") or not hasattr(param, "extra"):
            param = None

        self._active_bt_transitioning = True
        try:
            success, message = self.sm.trigger_event(event_name, param=param)
        finally:
            self._active_bt_transitioning = False

        if success:
            if param is not None:
                self.param = param
            logger.info(
                f"Active behavior tree '{running_bt['name']}': "
                f"{current_substate} -> {next_substate}"
            )
            self._sync_active_behavior_tree()
        else:
            logger.warning(
                f"Active behavior tree '{running_bt['name']}' failed to trigger '{event_name}': "
                f"{message}"
            )


    def handle_command(self, request, response):
        # 接收服务请求，data为事件名，param为额外参数
        event_name = request.data
        self.param = request.param if request.param else None
        # print(f"Received {event_name} and {self.param}")

        if self._extract_active_target_from_event(event_name) in self._active_bt_internal_nodes:
            response.success = False
            response.message = (
                f"Event '{event_name}' is an internal behavior-tree step. "
                f"Please trigger the root action from CLI instead."
            )
            response.current_state = self.sm.get_state()
            logger.warning(response.message)
            return response

        # 调用状态机事件，支持参数传递
        success, message = self.sm.trigger_event(event_name, param=self.param)

        response.success = success
        response.message = message
        response.current_state = self.sm.get_state()
        if success:
            self._sync_active_behavior_tree()

        # self.get_logger().info(f"Service request: trigger event '{event_name}', parameter '{self.param}'")
        self.get_logger().info(
            f"Service request: trigger event '{event_name}', "
            f"handside='{self.param.handside}', extra='{self.param.extra}'"
        )
        logger.info(f"{message}\n")

        return response
    
    def get_transitions_callback(self, request, response):
        events = self.sm.get_available_events()
        response.success = True
        response.message = ', '.join(events)
        # self.get_logger().info(f'当前状态: {self.sm.state}, 可用事件: {events}')
        return response
    
    def get_state_callback(self, request, response):
        response.success = True
        response.message = self.sm.state
        return response

def main(args=None):
    rclpy.init(args=args)

    executor = rclpy.executors.SingleThreadedExecutor()
    node = LifecyclePublisher("smach_main_node")
    executor.add_node(node)
    
    try:
        executor.spin()
    except Exception as e:
        tb_str = ''.join(traceback.format_exception(type(e), e, e.__traceback__))
        node.get_logger().error(f"Unhandled Exception:\n{tb_str}")
    finally:
        node.get_logger().info("Shutting down node...")
        node.destroy_node()
        if executor.context.ok():
             executor.shutdown()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
