cmake_minimum_required(VERSION 2.8.3)
project(helpme_carry)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  tf2
  roscpp
  rospy
  bica
  follow_person
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES helpme_carry
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
# include
 ${catkin_INCLUDE_DIRS}
)

add_executable(HelpMeCarry_executor_node
  src/HelpMeCarry_executor_node.cpp
  src/HelpMeCarry_executor.cpp
  src/HelpMeCarry_HFSM.cpp
)

add_dependencies(HelpMeCarry_executor_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(HelpMeCarry_executor_node
 ${catkin_LIBRARIES}
)
