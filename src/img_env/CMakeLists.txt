cmake_minimum_required(VERSION 2.8.3)
project(img_env)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_BUILD_TYPE Release)
 add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  comn_pkg
  cv_bridge
  std_srvs
  pedsimros
  ervo_ros
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
# find_package(Eigen3)
 find_package(OpenCV)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES img_env
#  CATKIN_DEPENDS roscpp tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

 add_executable(${PROJECT_NAME}_node src/img_env_node.cpp
     src/img_env.cpp
     src/grid_map.cpp
     src/agent.cpp)

## Specify libraries to link a library or executable target against
 target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES} ${OpenCV_LIBS}
 )