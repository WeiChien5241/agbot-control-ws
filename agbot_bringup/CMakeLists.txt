cmake_minimum_required(VERSION 3.0.2)
project(agbot_bringup)

find_package(catkin REQUIRED COMPONENTS roslaunch)

catkin_package()

install(DIRECTORY launch urdf rviz scripts config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
