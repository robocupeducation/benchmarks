cmake_minimum_required(VERSION 2.8.3)
project(open)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  std_msgs
  rospy
  bica
  bica_msgs
  message_generation
)

add_message_files(
  FILES
  locationsArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

include_directories(
# include
 ${catkin_INCLUDE_DIRS}
)

add_executable(open_executor_node
  src/open_executor_node.cpp
  src/open_executor.cpp
  src/open_HFSM.cpp
)

add_dependencies(open_executor_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(open_executor_node
 ${catkin_LIBRARIES}
)
