cmake_minimum_required(VERSION 3.0.2)
project(motion_control)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)


catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# generate_messages(
#   move_base_msgs
# )

catkin_install_python(PROGRAMS scripts/motion_control.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
