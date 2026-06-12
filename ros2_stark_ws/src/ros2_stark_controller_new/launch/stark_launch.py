from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # 获取包路径
    pkg_share = get_package_share_directory('ros2_stark_controller_new')

    # 拼接参数文件的绝对路径
    param_file_left = os.path.join(pkg_share, 'config', 'params_revo2_left.yaml')
    param_file_right = os.path.join(pkg_share, 'config', 'params_revo2_right.yaml')

    return LaunchDescription([
        Node(
            package='ros2_stark_controller_new',
            executable='stark_node',
            name='stark_node',
            namespace='left_hand',
            output='screen',
            parameters=[param_file_left],
        ),
        Node(
            package='ros2_stark_controller_new',
            executable='stark_node',
            name='stark_node',
            namespace='right_hand',
            output='screen',
            parameters=[param_file_right],
        ),
    ])