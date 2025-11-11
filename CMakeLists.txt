cmake_minimum_required(VERSION 3.0.2)
project(dwa_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(dwa_test_node 
  src/dwa_node.cpp
  src/dwa.cpp)
target_link_libraries(dwa_test_node
  ${catkin_LIBRARIES}
)

