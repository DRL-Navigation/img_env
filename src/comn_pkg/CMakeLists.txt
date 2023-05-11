cmake_minimum_required(VERSION 3.0.2)
project(comn_pkg)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  std_srvs
  actionlib_msgs
)

find_package(Boost REQUIRED COMPONENTS system thread)


include_directories(include)
include_directories(
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)


## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   RobotState.msg
   Agent.msg
   AgentState.msg
   Env.msg
   RobotRes.msg
   EpRes.msg
   EnvsInfo.msg
   EnvReset.msg
   EnvInit.msg
   PedInfo.msg
   SpeedLimiter.msg
 )
## Generate services in the 'srv' folder
 add_service_files(
   FILES
   GetVel.srv
   SetPose.srv
   VFHPlan.srv
   CheckGoal.srv
   PathPlan.srv
   MapValue.srv
   ResetEnv.srv
   StepEnv.srv
   InitEnv.srv
   EndEp.srv
 )

add_action_files(
  FILES
  PathFollow.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES drl_console
  CATKIN_DEPENDS geometry_msgs sensor_msgs std_msgs message_runtime
  DEPENDS libedit-dev
)


add_library(drl_console
  src/drl_console.cpp
)

target_link_libraries(drl_console
  ${Boost_LIBRARIES}
  edit
)
