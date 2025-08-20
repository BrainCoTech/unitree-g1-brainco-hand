from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # 获取包路径
    pkg_share = get_package_share_directory('ros2_stark_controller')

    # 拼接参数文件的绝对路径
    param_file = os.path.join(pkg_share, 'config', 'params_v2_double.yaml')

    return LaunchDescription([
        Node(
            package='ros2_stark_controller',
            executable='stark_node',
            output='screen',
            parameters=[param_file],
        ),
    ])
