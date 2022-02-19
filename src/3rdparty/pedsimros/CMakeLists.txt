cmake_minimum_required(VERSION 2.8.3)
project(pedsimros)

#------------------ Configuration ------------------#
option(SHALL_DEBUG "Enable debug features" OFF)
option(SHALL_PROFILE "Enable the code profiling feature" OFF)
option(CMAKE_VERBOSE_MAKEFILE "Full compiler output" ON)


#----------------- Compiler Flags ------------------#
add_definitions(-Wall -Wunused -std=c++0x -pipe)

#debugging
if(SHALL_DEBUG)
  message("Debugging activated")
  add_definitions(-O0 -DDEBUG -ggdb -g3 -rdynamic)
else(SHALL_DEBUG)
  message("Debugging deactivated")
  add_definitions(-Os)
endif(SHALL_DEBUG)


find_package(catkin REQUIRED COMPONENTS
  roscpp
)
find_package(Boost REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS  roscpp
  LIBRARIES pedsimros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include/pedsimros/
)

set(SOURCES
  src/ped_agent.cpp
  src/ped_obstacle.cpp
  src/ped_scene.cpp
  src/ped_tree.cpp
  src/ped_vector.cpp
  src/ped_waypoint.cpp
)

add_library(pedsimros ${SOURCES})

target_link_libraries(pedsimros
  ${BOOST_LIBRARIES}
)
