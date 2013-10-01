cmake_minimum_required(VERSION 2.8.3)
project(occupancy_grid_utils)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin
  REQUIRED COMPONENTS
  tf
  nav_msgs
  sensor_msgs
  geometry_msgs
  laser_geometry
  map_server
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(SDL   REQUIRED COMPONENTS SDL_image)
find_package(Eigen REQUIRED)
find_package(PythonLibs REQUIRED)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/groovy/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  LocalizedCloud.msg
  NavigationFunction.msg
  OverlayClouds.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  nav_msgs
  sensor_msgs
  geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES occupancy_grid_utils grid_utils
   #   CATKIN_DEPENDS controller geometry_msgs laser_to_wall roscpp
   DEPENDS Boost Eigen SDL PythonLibs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(SYSTEM ${Boost_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS})

include_directories(include
  ${PYTHON_INCLUDE_PATH}
  ${catkin_INCLUDE_DIRS}
)

add_library(grid_utils src/ray_tracer.cpp
                       src/grid_overlay.cpp
                       src/shortest_path.cpp
                       src/coordinate_conversions.cpp
                       src/combine_grids.cpp
                       src/geometry.cpp
                       src/file.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries( grid_utils SDL
                                  SDL_image
                                  ${catkin_LIBRARIES}
)

## Declare a cpp executable
add_executable(grid_construction_node
               src/examples/grid_construction_node.cpp
)
## Specify libraries to link a library or executable target against
target_link_libraries(grid_construction_node grid_utils)

add_library(occupancy_grid_utils src/boost_python_exports.cpp)
target_link_libraries(occupancy_grid_utils grid_utils)
target_link_libraries(occupancy_grid_utils ${PYTHON_LIBRARIES})

set_target_properties(occupancy_grid_utils
  PROPERTIES
  PREFIX "")


## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(bayes_filter_node bayes_filter_generate_messages_cpp)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/groovy/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS bayes_filter bayes_filter_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
catkin_add_gtest(test_grid_utils test/test_grid_utils.cpp)
target_link_libraries(test_grid_utils grid_utils)
if(TARGET test_grid_utils)
  target_link_libraries(test_grid_utils grid_utils)
endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)










#
## Set the build type.  Options are:
##  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
##  Debug          : w/ debug symbols, w/o optimization
##  Release        : w/o debug symbols, w/ optimization
##  RelWithDebInfo : w/ debug symbols, w/ optimization
##  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)
#
#find_package(Eigen REQUIRED)
#include_directories(${Eigen_INCLUDE_DIRS})
#
#rosbuild_init()
#rosbuild_genmsg()
#
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
#
#rosbuild_add_boost_directories()
#
#rosbuild_add_library(grid_utils src/ray_tracer.cpp
#                                src/grid_overlay.cpp
#                                src/shortest_path.cpp
#                                src/coordinate_conversions.cpp
#                                src/combine_grids.cpp
#                                src/geometry.cpp
#                                src/file.cpp)
#target_link_libraries(grid_utils SDL SDL_image)
#
#rosbuild_add_gtest(test_grid_utils test/test_grid_utils.cpp)
#target_link_libraries(test_grid_utils grid_utils)
#
#rosbuild_add_executable(grid_construction_node
#                        src/examples/grid_construction_node.cpp)
#target_link_libraries(grid_construction_node
#                      grid_utils)
#
#
## Python stuff
#
#find_package(PythonLibs REQUIRED)
#include_directories(${PYTHON_INCLUDE_PATH})                    
#
#rosbuild_add_library(occupancy_grid_utils src/boost_python_exports.cpp)
#target_link_libraries(occupancy_grid_utils grid_utils)
#target_link_libraries(occupancy_grid_utils ${PYTHON_LIBRARIES})
#rosbuild_link_boost(occupancy_grid_utils python)
#
#set_target_properties(occupancy_grid_utils
#  PROPERTIES
#  PREFIX "")
#
#rosbuild_make_distribution(0.5.1)
