import rclpy
from rclpy.node import Node
from sm_interfaces.srv import SmachCmd
from std_srvs.srv import Trigger
import logging

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


        self.client = self.create_client(SmachCmd, '/lifecycle_command')
        self.client_available_trans = self.create_client(Trigger, '/get_available_transitions')
        self.client_current_state = self.create_client(Trigger, '/get_current_state')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...\n')

        self.service_keys = {
            'f': 'configure',
            'a': 'activate',
            'c': 'cleanup',
            'd': 'deactivate',
            's': 'shutdown',
            '0': 'start_0',
            '1': 'start_1',
            '2': 'start_2',
            '3': 'start_3',
            '4': 'start_4',
        }

        self.action_name = {
            '0': 'Stop',
            '1': 'Hello',
            '2': 'Like',
            '3': 'Rock-Paper-Scissors',
            '4': 'Handshake',
        }
        self.action_name_default = ""

    
    def call_service(self, data: str, param: str):
        req = SmachCmd.Request()
        req.data = data
        req.param = param
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f'Service response: {future.result().message}\n')
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
    
    
    def key_to_service_call(self, key, param="0"):
        if self.cmd == key:
            service = self.service_keys[key]
            self.get_logger().info(f"Pressed '{key}', calling service {service} param = {param} ...")
            self.call_service(service, param)

    def key_to_action_call(self, key):
        if self.cmd[0] == key:
            service = self.service_keys[key]
            self.get_logger().info(f"Pressed '{key}', calling service {service} ...")
            if len(self.cmd) == 1:
                param = "both"
            elif self.cmd[1] == "l":
                param = "left"
            elif self.cmd[1] == "r":
                param = "right"
            else:
                param = "both"
            current_state = self.call_service_current_state()
            # 如果同状态切换手，就先换到 action_0 ，再重新进入action
            if current_state.message[-1] == service[-1]:
                self.call_service("start_0", "both")
            self.call_service(service, param)
    
    def run(self):
        try:
            while True:
                available_result = self.call_service_available_trans()
                current_state = self.call_service_current_state()
                available_services = [part.strip() for part in available_result.message.split(',')]
                print(f"Current state: {current_state.message}")
                print(f"Press [key + Enter] to call service, available keys:\n")
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
                self.key_to_action_call('0')
                self.key_to_action_call('1')
                self.key_to_action_call('2')
                self.key_to_action_call('3')
                self.key_to_action_call('4')
                if self.cmd == 'q':
                    self.get_logger().info('Exiting...')
                    break
        except KeyboardInterrupt:
            self.get_logger().info('Interrupted by user.')

def main(args=None):
    rclpy.init(args=args)
    keyboard_caller = KeyboardServiceCaller("trans_node")
    keyboard_caller.run()
    keyboard_caller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
