cmake_minimum_required(VERSION 2.4.6)
project(bayes_world)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS std_msgs geometry_msgs)

catkin_package(CATKIN_DEPENDS laser_to_wall std_msgs geometry_msgs)

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
install(
  DIRECTORY scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

