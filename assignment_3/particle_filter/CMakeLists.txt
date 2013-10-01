cmake_minimum_required(VERSION 2.8.3)
project(particle_filter)

find_package(catkin REQUIRED COMPONENTS std_msgs sensor_msgs nav_msgs tf occupancy_grid_utils roscpp)

find_package(Boost REQUIRED COMPONENTS system python3)
find_package(SDL   REQUIRED COMPONENTS SDL_image)
find_package(Eigen REQUIRED)
find_package(PythonLibs REQUIRED)

catkin_package(

)

include_directories(SYSTEM
    ${Boost_INCLUDE_DIRS}
    ${PYTHON_INCLUDE_PATH}
)

include_directories(include
    ${catkin_INCLUDE_DIRS}
)

add_executable(particle_filter
  src/MCL_node.cpp
  src/MCLocaliser.cpp
  src/ParticleFilter.cpp
)

target_link_libraries(
    particle_filter
    ${Boost_LIBRARIES}
    ${PYTHON_LIBRARIES}
    ${catkin_LIBRARIES}
)
