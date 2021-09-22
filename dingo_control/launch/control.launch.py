import os

from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
	xacro_path = os.path.join(get_package_share_directory('dingo_description'), 'urdf', 'dingo.urdf.xacro')

	if os.getenv('DINGO_OMNI', 0):
		control_yaml = os.path.join(get_package_share_directory('dingo_control'), 'config', 'control_omni.yaml')
	else:
		control_yaml = os.path.join(get_package_share_directory('dingo_control'), 'config', 'control_diff.yaml')
		
	# $(env DINGO_OMNI 0)

	physical_robot = LaunchConfiguration('physical_robot')

	robot_description_parameter = {"robot_description": Command(['xacro',' ', xacro_path, ' ', 'physical_robot:=', physical_robot])}

	# Declare the launch arguments
	declare_physical_robot_cmd = DeclareLaunchArgument(
		'physical_robot',
		default_value='true',
		description='Physical robot if true, simulation if false')

	# Specify the actions
	controller_manager_node = Node(
		condition=IfCondition(physical_robot),
		package="controller_manager",
		executable="ros2_control_node",
		parameters=[robot_description_parameter, control_yaml],
		output={
			"stdout": "screen",
			"stderr": "screen",
		},
	)

	spawn_dd_controller = Node(
		package="controller_manager",
		executable="spawner.py",
		arguments=["dingo_velocity_controller"],
		output="screen",
	)

	spawn_jsb_controller = Node(
		package="controller_manager",
		executable="spawner.py",
		arguments=["dingo_joint_broadcaster"],
		output="screen",
	)
		
	ld = LaunchDescription()

	# Add any conditioned actions
	ld.add_action(declare_physical_robot_cmd)
	ld.add_action(controller_manager_node)
	ld.add_action(spawn_dd_controller)
	ld.add_action(spawn_jsb_controller)

	return ld
