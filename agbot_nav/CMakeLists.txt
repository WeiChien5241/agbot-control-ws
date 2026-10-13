cmake_minimum_required(VERSION 3.0.2)
project(agbot_nav)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS
  scripts/goal_sequence.py
  scripts/pad_map.py
  scripts/gt_occupancy_map.py
  scripts/amcl_truth.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
install(DIRECTORY maps/   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/maps)
install(DIRECTORY models/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models)
