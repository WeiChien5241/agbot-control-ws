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
  scripts/deploy_supervisor.py
  scripts/record_start_pose.py
  scripts/sim_creep_guard.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Shell scripts: NOT catkin_install_python, which wraps them in a python relay.
install(PROGRAMS scripts/save_map.sh scripts/record_mapping_bag.sh DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
install(DIRECTORY maps/   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/maps)
install(DIRECTORY models/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models)
install(DIRECTORY rviz/   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz)
