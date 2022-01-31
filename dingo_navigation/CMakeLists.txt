################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(dingo_navigation)

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)

################################################################################
# Install
################################################################################
install(
	DIRECTORY launch params config rviz map
	DESTINATION share/${PROJECT_NAME}
)

# if(CATKIN_ENABLE_TESTING)
#   find_package(roslaunch REQUIRED)
#   roslaunch_add_file_check(launch/include/amcl.launch)
#   roslaunch_add_file_check(launch/amcl_demo.launch)
#   roslaunch_add_file_check(launch/include/gmapping.launch)
#   roslaunch_add_file_check(launch/gmapping_demo.launch)
#   roslaunch_add_file_check(launch/include/move_base.launch)
#   roslaunch_add_file_check(launch/odom_navigation_demo.launch)
# endif()

################################################################################
# Macro for ament package
################################################################################
ament_package()
