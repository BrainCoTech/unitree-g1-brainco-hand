from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_stark_controller',
            executable='stark_node',
            output='screen',
            parameters=['/home/unitree/stark-serialport-example/ros2_stark_ws/src/ros2_stark_controller/config/params_v2_double.yaml'],  # 直接传递路径
        ),
    ])