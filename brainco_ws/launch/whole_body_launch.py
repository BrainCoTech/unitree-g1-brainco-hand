import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    smach_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('control_py'), 'launch', 'smach_launch.py'
            )
        ])
    )

    hands_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('ros2_stark_controller_new'), 'launch', 'stark_launch.py'
            )
        ])
    )

    high_cmd_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('control_py'), 'launch', 'high_cmd_launch.py'
            )
        ])
    )


    return LaunchDescription([
        smach_node,
        hands_node,
        high_cmd_node
    ])