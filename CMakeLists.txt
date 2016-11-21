cmake_minimum_required(VERSION 2.8.3)

project(pr2_battery_alert)

find_package(catkin REQUIRED COMPONENTS
  pr2_msgs
  rospy
  sound_play
)

## declare catkin package
catkin_package()

## install ##
install(PROGRAMS
  scripts/pr2_battery_alert.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
