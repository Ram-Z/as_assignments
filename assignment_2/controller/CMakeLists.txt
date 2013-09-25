cmake_minimum_required(VERSION 2.4.6)
project(controller)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS std_msgs)

catkin_package(CATKIN_DEPENDS std_msgs)

## Mark executable scripts (Python etc.) for installation
install(
  FILES controller.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

