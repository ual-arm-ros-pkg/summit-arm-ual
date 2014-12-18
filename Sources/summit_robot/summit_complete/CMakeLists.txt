cmake_minimum_required(VERSION 2.8.3)
project(summit_complete)

find_package(catkin REQUIRED COMPONENTS 
    message_runtime 
    std_msgs 
    roscpp
    std_srvs
    std_msgs
    tf
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
