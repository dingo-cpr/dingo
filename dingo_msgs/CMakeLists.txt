################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(dingo_msgs)

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

################################################################################
# Build
################################################################################
rosidl_generate_interfaces(${PROJECT_NAME}
	"msg/Fans.msg"
	"msg/Lights.msg"
	"msg/Status.msg"
	"msg/RGB.msg"
  DEPENDENCIES std_msgs builtin_interfaces
)

################################################################################
# Macro for ament package
################################################################################
ament_export_dependencies(rosidl_default_runtime)
ament_package()