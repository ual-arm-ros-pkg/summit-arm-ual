cmake_minimum_required(VERSION 3.9)
project(summit_controller_dspic)

find_package(ackermann_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(robotnik_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# diagnostic_updater


## Declare a cpp executable
add_executable(${PROJECT_NAME}
  src/summit_controller_dspic.cpp
  src/SerialDevice.cc
  include/summit_controller_dspic/summit_controller_dspic.h
  include/summit_controller_dspic/SerialDevice.h
)

ament_target_dependencies(${PROJECT_NAME}
  ackermann_msgs
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_geometry_msgs
  robotnik_msgs
)

target_include_directories(${PROJECT_NAME} PRIVATE include)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)



set(CATKIN_PACKAGE_LIB_DESTINATION lib)
set(CATKIN_PACKAGE_BIN_DESTINATION lib/${PROJECT_NAME})
set(CATKIN_PACKAGE_INCLUDE_DESTINATION include)
set(CATKIN_PACKAGE_SHARE_DESTINATION share/${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark other files for installation (e.g. launch and bag files, etc.)
install(
  DIRECTORY
    config
    launch
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

ament_package()
