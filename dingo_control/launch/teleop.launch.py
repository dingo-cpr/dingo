import os

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
	dingo_control_directory = get_package_share_directory('dingo_control')

	if os.getenv('DINGO_OMNI', 0):
		teleop_yaml = os.path.join(dingo_control_directory, 'config', 'teleop_omni.yaml')
	else:
		teleop_yaml = os.path.join(dingo_control_directory, 'config', 'teleop_diff.yaml')

	joy_node_yaml = os.path.join(dingo_control_directory, 'config', 'joy_node.yaml')

	joystick = LaunchConfiguration('joystick')
	joy_dev = LaunchConfiguration('joy_dev')

	configured_dev = os.getenv('DINGO_JOY_DEV', '/dev/input/ps4')

	# Declare the launch arguments
	declare_joystick_cmd = DeclareLaunchArgument(
		'joystick',
		default_value='true',
		description='Whether or not to use the joystick')

	declare_joy_dev_cmd = DeclareLaunchArgument(
		'joy_dev',
		default_value=configured_dev,
		description='What joystick device should be used')

	# Specify the actions
	declare_joy_node_cmd = Node(
		condition=IfCondition(joystick),
		package='joy',
		executable='joy_node',
		parameters=[{'device_name': joy_dev}, joy_node_yaml],
		output='screen',
	)

	declare_teleop_joy_node = Node(
		package='teleop_twist_joy',
		executable='teleop_node',
		name='teleop_twist_joy_node',
		parameters=[teleop_yaml],
		remappings={('/cmd_vel', '/dingo_velocity_controller/cmd_vel_unstamped')}, # rename to bluetooth_teleop/cmd_vel is twist_mux works
		output='screen',
	)
		
	ld = LaunchDescription()
	ld.add_action(declare_joystick_cmd)
	ld.add_action(declare_joy_dev_cmd)
	ld.add_action(declare_joy_node_cmd)
	ld.add_action(declare_teleop_joy_node)

	return ld
