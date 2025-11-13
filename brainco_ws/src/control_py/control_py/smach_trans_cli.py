import rclpy
from rclpy.node import Node
from sm_interfaces.srv import SmachCmd
from sm_interfaces.msg import SmachParam
from std_srvs.srv import Trigger
import logging, os, sys

pkgs_dir = os.getcwd() + '/src/control_py/control_py/'
sys.path.append(pkgs_dir)

from action_pkg.tasks_handler import RobotLifecycleActions

from loguru import logger
from utils.loguru_settings import setup_loguru
setup_loguru(log_folder_path="log", show_on_terminal=True) # 设置日志

def remove_arrow(s: str) -> str:
        return s.split('->')[0]

def get_key_by_value(d, val):
    for k, v in d.items():
        if v == val:
            return k
    return None  # 找不到返回 None


class KeyboardServiceCaller(Node):
    def __init__(self, name):
        super().__init__(name)

        self.action_name = RobotLifecycleActions.action_name
        self.action_num = len(self.action_name)

        self.client = self.create_client(SmachCmd, '/lifecycle_command')
        self.client_available_trans = self.create_client(Trigger, '/get_available_transitions')
        self.client_current_state = self.create_client(Trigger, '/get_current_state')
        while not self.client.wait_for_service(timeout_sec=1.0):
            logger.info('Waiting for service...')

        self.service_keys = {
            'f': 'configure',
            'a': 'activate',
            'c': 'cleanup',
            'd': 'deactivate',
            's': 'shutdown'
        }

        self.service_keys.update({
            str(i): f'start_{i}' for i in range(self.action_num)
        })

        self.action_name_default = ""

        self.state_final = False

    
    # def call_service(self, data: str, param: str):
    def call_service(self, data: str, handside: str = "", extra: str = ""):
        req = SmachCmd.Request()
        req.data = data

        # req.param = param
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
    
    
    # def key_to_service_call(self, key, param="0"):
    #     if self.cmd == key:
    #         service = self.service_keys[key]
    #         self.get_logger().info(f"Pressed '{key}', calling service {service} param = {param} ...")
    #         self.call_service(service, param)

    def key_to_service_call(self, key):
        if self.cmd == key:
            service = self.service_keys[key]
            self.get_logger().info(f"Pressed '{key}', calling service {service} param = '' ...")
            self.call_service(service)

    def key_to_action_call(self, key):
        if self.cmd[0] == key:
            service = self.service_keys[key]
            self.get_logger().info(f"Pressed '{key}', calling service {service} ...")

            if len(self.cmd) == 1:
                handside = "both"
            elif self.cmd[1] == "l":
                handside = "left"
            elif self.cmd[1] == "r":
                handside = "right"
            else:
                handside = "both"

            if len(self.cmd) == 1:
                extra = ""
            elif self.cmd[1] in [str(i) for i in range(1, 10)]:
                extra = f"pos{self.cmd[1]}"
            else:
                extra = "pos1"

            current_state = self.call_service_current_state()
            # 如果同状态切换手，就先换到 action_0 ，再重新进入action
            if current_state.message[-1] == service[-1]:
                self.call_service("start_0", handside="both", extra="")
            self.call_service(service, handside=handside, extra=extra)
    
    def run(self):
        try:
            while True:
                available_result = self.call_service_available_trans()
                current_state = self.call_service_current_state()
                available_services = [part.strip() for part in available_result.message.split(',')]
                logger.info(f"Current state: {current_state.message}")
                logger.info(f"Press [key + Enter] to call service, available keys:")
                print(f"'q' : Exit")
                for service in available_services:
                    key = get_key_by_value(self.service_keys, remove_arrow(service))
                    hand_side_char = "(l/r)" if key[0].isdigit() else ""
                    print(f"'{key}{hand_side_char}' : {service} {self.action_name.get(key, self.action_name_default)}")
                
                self.cmd = input()
                self.key_to_service_call('f')
                self.key_to_service_call('a')
                self.key_to_service_call('c')
                self.key_to_service_call('d')
                self.key_to_service_call('s')
                for i in range(self.action_num):
                    self.key_to_action_call(str(i))
                if self.cmd == 'q':
                    self.get_logger().info('Exiting...')
                    break
        except KeyboardInterrupt:
            self.get_logger().info('Interrupted by user.')

def main(args=None):
    # rclpy.init(args=args)
    # keyboard_caller = KeyboardServiceCaller("smach_trans_node")
    # keyboard_caller.run()
    # keyboard_caller.destroy_node()
    # rclpy.shutdown()
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