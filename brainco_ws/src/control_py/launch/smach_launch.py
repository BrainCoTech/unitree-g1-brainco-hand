import os
from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros.actions
  
def generate_launch_description():
	config = os.path.join(
		get_package_share_directory('control_py'), 'config', 'smach_config.yaml')
	
	cyan = "\033[96m"   # 青色
	reset = "\033[0m"
	bold = "\033[1m"     # 加粗

	return launch.LaunchDescription([
		launch.actions.LogInfo(msg=f"{cyan} Start launching {reset}{bold} smach_main_node ...{reset}"),
        # launch.actions.LogInfo(msg=f"{cyan} Config File: {reset}{config}"),
		launch_ros.actions.Node(
			package='control_py',
			executable='smach_main_node',
			name='smach_main_node',
			# parameters=[config],
			emulate_tty=True
		),
	])