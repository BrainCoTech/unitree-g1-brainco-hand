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
                get_package_share_directory('ros2_stark_controller'), 'launch', 'stark_launch.py'
            )
        ])
    )


    return LaunchDescription([
        smach_node,
        # trans_node,
        hands_node
    ])