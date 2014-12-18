cmake_minimum_required(VERSION 2.8.3)
project(summit_controller_dspic)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
    roscpp
    tf
    nav_msgs 
	geometry_msgs
	sensor_msgs
	robotnik_msgs
	std_msgs
    diagnostic_updater
    ackermann_msgs
)

#add_message_files(
#  FILES
#  guardian_state.msg
#)

## Generate services in the 'srv' folder
#add_service_files(DIRECTORY srv 
#  FILES
#  set_odometry.srv
#)

#generate_messages(
#  DEPENDENCIES
#)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES summit_xl_control
    CATKIN_DEPENDS
    roscpp
    tf
    nav_msgs 
    diagnostic_updater
    ackermann_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(summit_controller_dspic src/summit_controller_dspic.cpp  src/SerialDevice.cc)
add_dependencies(summit_controller_dspic summit_controller_dspic_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(summit_controller_dspic
  ${catkin_LIBRARIES}
)
