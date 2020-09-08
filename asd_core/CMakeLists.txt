cmake_minimum_required(VERSION 3.0.2)
project(asd_core)

find_package(catkin)

catkin_package()

#############
## Install ##
#############

install(PROGRAMS scripts/pose_relay.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)