cmake_minimum_required(VERSION 2.8.3)
project(dingo_control)

find_package(catkin REQUIRED COMPONENTS roslaunch)

catkin_package()

roslaunch_add_file_check(dingo_d/launch/control.launch)
roslaunch_add_file_check(dingo_d/launch/teleop.launch)
roslaunch_add_file_check(dingo_o/launch/control.launch)
roslaunch_add_file_check(dingo_o/launch/teleop.launch)

install(DIRECTORY dingo_d/config dingo_o/launch dingo_o/config dingo_o/launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
