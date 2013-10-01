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
find_package(Boost REQUIRED COMPONENTS system python3)
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
include_directories(SYSTEM ${Boost_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS} ${PYTHON_INCLUDE_PATH})

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_library(grid_utils
    src/ray_tracer.cpp
    src/grid_overlay.cpp
    src/shortest_path.cpp
    src/coordinate_conversions.cpp
    src/combine_grids.cpp
    src/geometry.cpp
    src/file.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries( grid_utils
    SDL
    SDL_image
)

## Declare a cpp executable
add_executable(grid_construction_node
    src/examples/grid_construction_node.cpp
)
## Specify libraries to link a library or executable target against
target_link_libraries(grid_construction_node
    grid_utils
    ${Boost_LIBRARIES}
    ${PYTHON_LIBRARIES}
    ${catkin_LIBRARIES}
)

add_library(occupancy_grid_utils
    src/boost_python_exports.cpp
)

set_target_properties(occupancy_grid_utils
    PROPERTIES
    PREFIX ""
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
catkin_add_gtest(test_grid_utils test/test_grid_utils.cpp)
target_link_libraries(test_grid_utils grid_utils)
if(TARGET test_grid_utils)
  target_link_libraries(test_grid_utils grid_utils)
endif()
