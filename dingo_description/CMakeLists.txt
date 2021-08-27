################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(dingo_description)

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)

################################################################################
# Install
################################################################################
install(
  DIRECTORY env-hooks meshes launch urdf
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/env_run
  DESTINATION share/${PROJECT_NAME}
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/dingo_description.dsv.in")

################################################################################
# Macro for ament package
################################################################################
ament_package()
