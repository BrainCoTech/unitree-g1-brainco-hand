import os
from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros.actions
  
def generate_launch_description():
	config = os.path.join(
		get_package_share_directory('control_py'), 'config', 'smach_config.yaml')
	return launch.LaunchDescription([
		launch_ros.actions.Node(
			package='control_py',
			executable='smach_node',
			name='smach_node',
			parameters=[config]
		),
	])