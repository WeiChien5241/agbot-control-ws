cmake_minimum_required(VERSION 3.0.2)
project(agbot_vision_nav)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  geometry_msgs
  std_msgs
  cv_bridge
)

# This package has a setup.py so src/agbot_vision_nav/*.py is importable
# as a regular python package (e.g. `from agbot_vision_nav import controller`).
catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy sensor_msgs geometry_msgs std_msgs cv_bridge
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/vision_nav_node.py
  scripts/operator_panel.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test)
endif()
