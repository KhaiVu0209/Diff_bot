import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
	bumperbot_controller_pkg = get_package_share_directory('bumperbot_controller')

	use_sim_time_arg = DeclareLaunchArgument(
		name='use_sim_time',
		default_value='True'
	)

	twist_mux_launch = IncludeLaunchDescription(
		os.path.join(
			get_package_share_directory('twist_mux'),
			'launch',
			'twist_mux_launch.py'
		),
		launch_arguments={
			'cmd_vel_out': 'bumperbot_controller/cmd_vel_unstamped',
			'config_locks': os.path.join(
				bumperbot_controller_pkg, 'config', 'twist_mux_locks.yaml'
			),
			'config_topics': os.path.join(
				bumperbot_controller_pkg, 'config', 'twist_mux_topics.yaml'
			),
			'use_sim_time': LaunchConfiguration('use_sim_time'),
		}.items(),
	)

	twist_relay_node = Node(
		package='bumperbot_controller',
		executable='twist_relay',
		name='twist_relay',
		parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
	)

	keyboard_node = Node(
		package='teleop_twist_keyboard',
		executable='teleop_twist_keyboard',
		name='teleop_twist_keyboard',
		remappings=[('cmd_vel', '/key_vel')],
		output='screen',
	)

	return LaunchDescription([
		use_sim_time_arg,
		twist_mux_launch,
		twist_relay_node,
		keyboard_node,
	])