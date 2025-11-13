import sys, os, traceback
import numpy as np
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger  # 用标准的触发服务，返回string message

pkgs_dir = os.getcwd() + '/src/control_py/control_py/'
sys.path.append(pkgs_dir)

from sm_interfaces.srv import SmachCmd
from sm_interfaces.msg import SmachParam
from action_pkg.state_machine import LifecycleStateMachine
from action_pkg.tasks import RobotTasks
from action_pkg.tasks_handler import RobotLifecycleActions

from loguru import logger
from utils.loguru_settings import setup_loguru
setup_loguru(log_folder_path="log", show_on_terminal=True) # 设置日志


class LifecyclePublisher(RobotTasks, RobotLifecycleActions, Node):
    def __init__(self, name):
        super().__init__(name)

        # 获取参数：是否显示日志到终端
        # 参数名和属性名映射
        param_map = {
            'log_time_screen': 'log_time',
            'log_armstate_screen': 'log_arm_state',
            'log_armcmd_screen': 'log_arm_cmd',
            'log_wristpose_screen': 'log_wrist_pose',
            'log_hand_screen': 'log_hand'
        }
        # 循环声明参数并赋值
        for param_name, attr_name in param_map.items():
            self.declare_parameter(param_name, False)  # 默认值 False
            setattr(self, attr_name, self.get_parameter(param_name).get_parameter_value().bool_value)

        self.declare_parameter('robot_dof', 23) # 声明参数
        self.robot_dof = self.get_parameter('robot_dof').value # 获取参数值
        logger.info(f"1-robot_dof: {self.robot_dof}")

        self.robot_control_initialization()

        self.action_num = RobotLifecycleActions.action_num # 包含静止
        
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

        self.param = SmachParam()

        logger.info("Request 'configure' to start\n")


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

    # Timer
    def timer_get_ready(self):
        self.time_ += self.control_dt_
        self.arm_hand_start(0, 2, update_arm_q=self.zero_arm_q, update_hand=self.zero_hand)
        self.publish_all()


    def handle_command(self, request, response):
        # 接收服务请求，data为事件名，param为额外参数
        event_name = request.data
        self.param = request.param if request.param else None
        # print(f"Received {event_name} and {self.param}")

        # 调用状态机事件，支持参数传递
        success, message = self.sm.trigger_event(event_name, param=self.param)

        response.success = success
        response.message = message
        response.current_state = self.sm.get_state()

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
    # node = LifecyclePublisher(29, "smach_main_node")
    node = LifecyclePublisher("smach_main_node")
    executor.add_node(node)
    
    try:
        # spin()会使节点保持活动，并响应外部的生命周期管理命令
        # 它不会自动触发状态转换，这些转换需要外部通过服务调用来发起
        executor.spin()
    # except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException, KeyError):
    #     node.get_logger().info("KeyboardInterrupt by user")
    except Exception as e:
        # node.get_logger().error(f"{e}")
        tb_str = ''.join(traceback.format_exception(type(e), e, e.__traceback__))
        node.get_logger().error(f"Unhandled Exception:\n{tb_str}")
    finally:
        node.get_logger().info("Shutting down node...")
        node.destroy_node() # 这会处理一些内部清理
        if executor.context.ok(): # 检查执行器上下文是否仍然有效
             executor.shutdown()
        rclpy.try_shutdown() # 尝试关闭rclpy，如果它尚未关闭


if __name__ == '__main__':
    main()