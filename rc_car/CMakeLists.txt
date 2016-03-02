cmake_minimum_required(VERSION 2.8.3)
project(rc_car)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  roslaunch
)

roslaunch_add_file_check(launch)

set(Boost_USE_STATIC_LIBS ON) 
set(Boost_USE_MULTITHREADED ON)  
set(Boost_USE_STATIC_RUNTIME ON) 

find_package(Boost COMPONENTS bind)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   Command.msg
   RSRMsg.msg
   SwitchMsg.msg
   YPR.msg
 )

## Generate services in the 'srv' folder
add_service_files(
   FILES
   waypoint.srv
 )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rc_car
  CATKIN_DEPENDS message_runtime roscpp rospy std_msgs  
  DEPENDS system_lib

)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_library(pwm src/pwm.cpp)
add_library(distance src/distance.cpp)

add_executable(iDCmotor src/DCmotor.cpp)
target_link_libraries(iDCmotor ${catkin_LIBRARIES})
target_link_libraries(iDCmotor pwm)
add_dependencies(iDCmotor rc_car_generate_messages_cpp)

add_executable(iServomotor src/servomotor.cpp)
target_link_libraries(iServomotor ${catkin_LIBRARIES})
target_link_libraries(iServomotor pwm)
add_dependencies(iServomotor rc_car_generate_messages_cpp)

add_executable(iSwitchMode src/switchMode.cpp)
target_link_libraries(iSwitchMode ${catkin_LIBRARIES})
target_link_libraries(iSwitchMode ${Boost_LIBRARIES})
target_link_libraries(iSwitchMode pwm)
add_dependencies(iSwitchMode rc_car_generate_messages_cpp)

add_executable(iRegulation src/Regulation.cpp)
target_link_libraries(iRegulation ${catkin_LIBRARIES})
target_link_libraries(iRegulation distance)
add_dependencies(iRegulation rc_car_generate_messages_cpp)

add_executable(iWayPoint src/waypoint.cpp)
target_link_libraries(iWayPoint ${catkin_LIBRARIES})
add_dependencies(iWayPoint rc_car_gencpp)
add_dependencies(iWayPoint rc_car_generate_messages_cp)

add_executable(iWatchDog src/WatchDog.cpp)
target_link_libraries(iWatchDog ${catkin_LIBRARIES})
target_link_libraries(iWatchDog ${Boost_LIBRARIES})
add_dependencies(iWatchDog rc_car_generate_messages_cpp)

add_executable(Test src/test.cpp)
target_link_libraries(Test ${catkin_LIBRARIES})
target_link_libraries(Test ${Boost_LIBRARIES})
add_dependencies(Test rc_car_generate_messages_cpp)