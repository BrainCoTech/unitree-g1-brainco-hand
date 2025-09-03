import sys, os
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger  # 用标准的触发服务，返回string message

pkgs_dir = os.getcwd() + '/src/control_py/control_py/'
sys.path.append(pkgs_dir)

from sm_interfaces.srv import SmachCmd  # 换成你自己的srv包名
from smach_pkg.state_machine import LifecycleStateMachine
from smach_pkg.tasks import *


class LifecyclePublisher(RobotTasks, Node):
    def __init__(self, name):
        super().__init__(name)

        # 获取参数：是否显示日志到终端
        # 参数名和属性名映射
        param_map = {
            'log_time_screen': 'log_time',
            'log_armstate_screen': 'log_arm_state',
            'log_armcmd_screen': 'log_arm_cmd',
            'log_hand_screen': 'log_hand'
        }
        # 循环声明参数并赋值
        for param_name, attr_name in param_map.items():
            self.declare_parameter(param_name, True)  # 默认值 True
            setattr(self, attr_name, self.get_parameter(param_name).get_parameter_value().bool_value)
        
        

        self.action_num = 5 # action 0~4
        
        # action 回调
        active_callbacks = {
            f'on_active_{i}_callback': getattr(self, f'on_active_{i}_handler')
            for i in range(self.action_num)
        }
        
        self.sm = LifecycleStateMachine(self.action_num,  
            on_configure_callback=self.on_configure_handler,
            on_shutdown_callback=self.on_shutdown_handler,
            **active_callbacks
        )


        # Service server
        self.srv_trans_cmd = self.create_service(SmachCmd, 'lifecycle_command', self.handle_command)
        self.srv_available_trans = self.create_service(Trigger, 'get_available_transitions', self.get_transitions_callback)
        self.srv_current_state = self.create_service(Trigger, 'get_current_state', self.get_state_callback)

        self.param = None

        self.get_logger().info("Request 'configure' to start\n")


    # 准备阶段
    def on_configure_handler(self):
        if self.ready_to_start:
            self.get_logger().info(f"Enter state {self.sm.get_state()}")
            self.clear_timer()
            self.store_curr_cmd("both")
            self._timer = self.create_timer(0.01, self.timer_get_ready)
            self.get_logger().info(f"New timer created.")
        else:
            self.get_logger().info("Waiting ...")

    # 停止程序
    def on_shutdown_handler(self):
        self.get_logger().info(f"Shutting down from state {self.sm.get_state()}")
        self.clear_timer()


    # Active 0 静止
    def on_active_0_handler(self):
        self.get_logger().info(f"Enter state {self.sm.get_state()} 'Stationary Arm Activity'")
        self.clear_timer()
        self.store_curr_cmd("both")

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

    # Timer
    def timer_get_ready(self):
        self.time_ += self.control_dt_
        self.arm_hand_start(0, 2, update_arm_q=self.zero_arm_q, update_hand=self.zero_hand)
        self.publish_all()

    def timer_hello(self):
        self.time_ += self.control_dt_
        if self.param != "left" and self.param != "right":
            self.show_hello(0., 10, "both", speed=1.5)
        else:
            arm_side_opp = "left" if self.param == "right" else "right"
            self.show_hello(0., 10, self.param, speed=1.5)
            self.arm_back_zero(0., 2, arm_side_opp)
        self.publish_all()

    def timer_like(self):
        self.time_ += self.control_dt_
        if self.param != "left" and self.param != "right":
            self.show_like(0., 1, "both")
        else:
            arm_side_opp = "left" if self.param == "right" else "right"
            self.show_like(0., 1, self.param)
            self.arm_back_zero(0., 2, arm_side_opp)
        self.publish_all()

    def timer_hand_shake(self):
        self.time_ += self.control_dt_
        if self.param != "left" and self.param != "right":
            self.hand_shake(0., 2, 2, "both")
        else:
            arm_side_opp = "left" if self.param == "right" else "right"
            self.hand_shake(0., 2, 2, self.param)
            self.arm_back_zero(0., 2, arm_side_opp)
        self.publish_all()

    def timer_rps(self):
        self.time_ += self.control_dt_
        if self.param != "left" and self.param != "right":
            self.play_rps(0., 2, self.end_ts, self.interval_stop, self.interval_move, "both")
        else:
            arm_side_opp = "left" if self.param == "right" else "right"
            self.play_rps(0., 2, self.end_ts, self.interval_stop, self.interval_move, self.param)
            self.arm_back_zero(0., 2, arm_side_opp)
        self.publish_all()  



    def handle_command(self, request, response):
        # 接收服务请求，data为事件名，param为额外参数
        event_name = request.data
        self.param = request.param if request.param else None
        print(f"Received {event_name} and {self.param}")

        # 调用状态机事件，支持参数传递
        success, message = self.sm.trigger_event(event_name, param=self.param)

        response.success = success
        response.message = message
        response.current_state = self.sm.get_state()

        self.get_logger().info(f"Service request: trigger event '{event_name}', parameter '{self.param}'")
        self.get_logger().info(f"Result: {message}\n")

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
    
    node = LifecyclePublisher("smach_node")
    executor.add_node(node)
    
    try:
        # spin()会使节点保持活动，并响应外部的生命周期管理命令
        # 它不会自动触发状态转换，这些转换需要外部通过服务调用来发起
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.get_logger().info("Shutting down node...")
        node.destroy_node() # 这会处理一些内部清理
        if executor.context.ok(): # 检查执行器上下文是否仍然有效
             executor.shutdown()
        rclpy.try_shutdown() # 尝试关闭rclpy，如果它尚未关闭


if __name__ == '__main__':
    main()