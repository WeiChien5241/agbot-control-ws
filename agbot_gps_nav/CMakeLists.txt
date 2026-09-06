cmake_minimum_required(VERSION 3.0.2)
project(agbot_gps_nav)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  std_srvs
)

# This package has a setup.py so src/agbot_gps_nav/*.py is importable
# as a regular python package (e.g. `from agbot_gps_nav import geo`).
catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy sensor_msgs geometry_msgs nav_msgs std_msgs std_srvs
)

include_directories(${catkin_INCLUDE_DIRS})

catkin_install_python(PROGRAMS
  scripts/gps_nav_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
install(DIRECTORY rviz/   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test)
endif()
