cmake_minimum_required(VERSION 3.0.2)
project(obstacle_avoidance) # name of the package

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS apf_avoider/apf_avoider.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_install_python(PROGRAMS apf_avoider/apf_real_avoider.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_install_python(PROGRAMS apf_avoider/apf_multi_avoider.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_install_python(PROGRAMS apf_avoider/apf_velocity_avoider.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_install_python(PROGRAMS fake_map/fake_map.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_install_python(PROGRAMS fake_waypoints/fake_waypoints.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})