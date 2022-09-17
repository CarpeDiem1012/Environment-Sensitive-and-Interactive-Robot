cmake_minimum_required(VERSION 3.0.2)
project(path_planning)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  move_base_msgs
  cv_bridge
  sensor_msgs
  # numpy_msg
)


catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

include_directories(include ${PROJECT_SOURCE_DIR}/scripts)

# generate_messages(
#   move_base_msgs
# )

catkin_install_python(PROGRAMS scripts/path_planning.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
