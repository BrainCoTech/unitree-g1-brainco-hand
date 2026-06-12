import rclpy
from rclpy.node import Node
from sm_interfaces.srv import SmachCmd
from sm_interfaces.msg import SmachParam
from std_srvs.srv import Trigger

import yaml

from loguru import logger
from control_py.utils.loguru_settings import setup_loguru
setup_loguru(log_folder_path="log", show_on_terminal=True) # 设置日志

# ---------- 读取 config.yaml ----------
CONFIG_PATH = "src/control_py/config/smach_config.yaml"
with open(CONFIG_PATH, "r", encoding="utf-8") as f:
    smach_cfg = yaml.safe_load(f)
mode_cfg = smach_cfg['mode']
MODE = mode_cfg.get("current_mode", "basic")
STATES_CONFIG_PATH = mode_cfg["states_config"]["SmFolder"] + mode_cfg["states_config"][MODE]

def remove_arrow(s: str) -> str:
        return s.split('->')[0]

def get_key_by_value(d, val):
    for k, v in d.items():
        if v == val:
            return k
    return None  # 找不到返回 None


def get_active_substate(state_name: str):
    if not isinstance(state_name, str):
        return None
    if state_name.startswith("active."):
        return state_name.split(".", 1)[1]
    if state_name.startswith("active_"):
        return state_name.split("_", 1)[1]
    return None


def _flatten_active_bt_children(raw_children):
    children = []

    for item in raw_children:
        if isinstance(item, (list, tuple)):
            children.extend(str(child) for child in item)
            continue

        if isinstance(item, dict):
            group_children = None
            for group_key in ("parallel", "group"):
                if group_key in item:
                    group_children = item[group_key]
                    break
            if group_children is not None:
                children.extend(str(child) for child in group_children)
                continue

        children.append(str(item))

    return children


def load_active_behavior_tree_nodes(cfg):
    active_bt_cfg = cfg.get("behavior_trees", {}).get("active", {})
    root_nodes = set()
    internal_nodes = set()

    for _, tree_def in active_bt_cfg.items():
        if str(tree_def.get("type", "")).strip().lower() != "sequence":
            continue

        children = _flatten_active_bt_children(tree_def.get("children", []))
        if not children:
            continue

        root_nodes.add(children[0])
        internal_nodes.update(children[1:])

    return root_nodes, internal_nodes


class KeyboardServiceCaller(Node):
    def __init__(self, name):
        super().__init__(name)

        self.client = self.create_client(SmachCmd, '/lifecycle_command')
        self.client_available_trans = self.create_client(Trigger, '/get_available_transitions')
        self.client_current_state = self.create_client(Trigger, '/get_current_state')
        while not self.client.wait_for_service(timeout_sec=1.0):
            logger.info('Waiting for service...')

        # 读取 YAML
        with open(STATES_CONFIG_PATH, 'r', encoding='utf-8') as f:
            cfg = yaml.safe_load(f)

        service_keys = {}
        for event_name, trans_def in cfg.get('main_trans', {}).items():
            trans_id = trans_def.get('id')
            if trans_id:
                service_keys[trans_id] = event_name

        # 自动生成 active 子状态 start 事件的 service_keys
        active_children = {}
        for state in cfg.get('structure', []):
            if isinstance(state, dict) and 'active' in state:
                active_children = state['active'].get('children', {})
                break
        
        self.action_name = active_children
        self.action_num = len(active_children)
        self.bt_root_actions, self.bt_internal_actions = load_active_behavior_tree_nodes(cfg)

        for k in active_children.keys():
            action_key = str(k)
            if action_key in self.bt_internal_actions:
                continue
            service_keys[action_key] = f'start_{action_key}'

        self.service_keys = service_keys


        self.action_name_default = ""

        self.state_final = False

    
    # def call_service(self, data: str, param: str):
    def call_service(self, data: str, handside: str = "", extra: str = ""):
        req = SmachCmd.Request()
        req.data = data

        # 构造 SmachParam 对象
        req.param = SmachParam()
        req.param.handside = handside
        req.param.extra = extra

        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            msg = future.result().message
            self.get_logger().info(f'Service response: {msg}\n')

            if "finalized" in msg.lower():
                logger.info("Detected 'finalized' state, Press 'Ctrl + C' to exit.")
                self.state_final = True

        else:
            self.get_logger().error('Service call failed')

    def call_service_available_trans(self):
        req = Trigger.Request()  # Trigger请求为空
        future = self.client_available_trans.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            # self.get_logger().info(f'Success: {future.result().success}, Message: {future.result().message}')
            return future.result()
        else:
            self.get_logger().error("Service 'get_available_transitions' call failed")
            return None

    def call_service_current_state(self):
        req = Trigger.Request()  # Trigger请求为空
        future = self.client_current_state.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            # self.get_logger().info(f'Success: {future.result().success}, Message: {future.result().message}')
            return future.result()
        else:
            self.get_logger().error("Service 'get_current_state' call failed")
            return None

    def key_to_service_call(self, key):
        if self.cmd == key:
            service = self.service_keys[key]
            self.get_logger().info(f"Pressed '{key}', calling service {service} param = '' ...")
            self.call_service(service)

    def key_to_action_call(self, key):
        action, handside, extra = self.parse_cmd(self.cmd)

        if key not in self.service_keys:
            return

        if action == key:
            service = self.service_keys[key]
            self.get_logger().info(
                f"Pressed '{self.cmd}', calling service {service} ({handside}) ..."
            )

            current_state = self.call_service_current_state()
            # 如果同状态切换手，就先换到 action_0 ，再重新进入action
            current_substate = get_active_substate(current_state.message)
            target_substate = service.split("start_", 1)[1] if service.startswith("start_") else None
            if current_substate is not None and current_substate == target_substate:
                self.call_service("start_0", handside="both", extra=extra)
            self.call_service(service, handside=handside, extra=extra)


    def parse_cmd(self, cmd: str):
        i = 0
        while i < len(cmd) and cmd[i].isdigit():
            i += 1

        action = cmd[:i]
        suffix = cmd[i:]

        handside = "both"
        extra = ""

        if suffix.startswith("l"):
            handside = "left"
            extra = suffix[1:]
        elif suffix.startswith("r"):
            handside = "right"
            extra = suffix[1:]
        else:
            extra = suffix

        return action, handside, extra
    
    def run(self):
        try:
            while True:
                available_result = self.call_service_available_trans()
                current_state = self.call_service_current_state()
                available_services = []
                if available_result is not None and available_result.message:
                    available_services = [
                        part.strip() for part in available_result.message.split(',')
                        if part.strip()
                    ]

                logger.info(f"Current state: {current_state.message}")
                logger.info(f"Press [key + Enter] to call service, available keys:")
                print(f"'q/s' : Exit")

                if not available_services:
                    logger.warning("No available transitions right now. The previous action may still be running.")

                for service in available_services:
                    target_substate = service.split("->", 1)[0].split("start_", 1)[1] if service.startswith("start_") else None
                    if target_substate in self.bt_internal_actions:
                        continue
                    key = get_key_by_value(self.service_keys, remove_arrow(service))
                    if key is None:
                        continue
                    hand_side_char = "(l/r)" if key[0].isdigit() else ""
                    print(f"'{key}{hand_side_char}' : {service} {self.action_name.get(key, self.action_name_default)}")
                
                self.cmd = input()
                for key in self.service_keys.keys():
                    if not key.isdigit():  # 只处理 main_trans 的 id
                        self.key_to_service_call(key)
                for key in self.service_keys.keys():
                    if key.isdigit():
                        self.key_to_action_call(key)
                if self.cmd == 'q':
                    self.get_logger().info('Exiting...')
                    break
        except KeyboardInterrupt:
            self.get_logger().info('Interrupted by user.')

def main(args=None):
    rclpy.init(args=args)

    executor = rclpy.executors.SingleThreadedExecutor()
    node = KeyboardServiceCaller("smach_trans_node")
    executor.add_node(node)

    try:
        node.run()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        node.get_logger().info("KeyboardInterrupt by user")
    except Exception as e:
        node.get_logger().error(f"{e}")
    finally:
        node.get_logger().info("Shutting down node...")
        node.destroy_node()
        if executor.context.ok():
            executor.shutdown()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
